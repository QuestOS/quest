/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <smp/apic.h>
#include <drivers/usb/usb.h>
#include <drivers/usb/uhci.h>
#include <util/printf.h>
#include <mem/virtual.h>
#include <mem/pow2.h>
#include <kernel.h>

#define DEBUG_UHCI
//#define DEBUG_UHCI_VERBOSE

#ifdef DEBUG_UHCI
#define DLOG(fmt,...) DLOG_PREFIX("UHCI",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef DEBUG_UHCI_VERBOSE
#define DLOGV(fmt,...) DLOG_PREFIX("UHCI",fmt,##__VA_ARGS__)
#else
#define DLOGV(fmt,...) ;
#endif

static int bus;                 /* set by PCI probing in uhci_init */
static int dev;
static int func;

static uint32 uhci_irq_handler (uint8 vec);

/* USB I/O space base address */
static uint32_t usb_base = 0x0;

/* UHCI Frame List. 1024 entries aligned to 4KB boundary */
static frm_lst_ptr frame_list[1024] ALIGNED(0x1000);
static UHCI_TD td[TD_POOL_SIZE] ALIGNED(0x1000);
static UHCI_QH qh[QH_POOL_SIZE] ALIGNED(0x10);
static frm_lst_ptr *phys_frm;
static UHCI_QH *int_qh = 0;
static UHCI_QH *ctl_qh = 0;
static UHCI_QH *blk_qh = 0;
static uint32 toggles[128];     /* endpoint toggle bit states (bulk transfer) */

uint32 td_phys, qh_phys;

/* Virtual-to-Physical */
#define TD_V2P(ty,p) ((ty)((((uint) (p)) - ((uint) td))+td_phys))
#define QH_V2P(ty,p) ((ty)((((uint) (p)) - ((uint) qh))+qh_phys))
/* Physical-to-Virtual */
#define TD_P2V(ty,p) ((ty)((((uint) (p)) - td_phys)+((uint) td)))
#define QH_P2V(ty,p) ((ty)((((uint) (p)) - qh_phys)+((uint) qh)))

static task_id uhci_waitq = 0;  /* Tasks waiting for IRQ */

#define TERMINATE 1
#define SELECT_TD 0
#define SELECT_QH 2
#define DEPTH_FIRST 4
#define LINK_MASK (~0xFL)


/*
 * This is non-reentrant! Fortunately, we do not have concurrency yet:-)
 * This function returns the available Queue Head or Transfer Descriptor.
 * It will probably be removed later.
 */
static void *
sched_alloc (int type)
{
  int i = 0;

  switch (type) {
    case TYPE_TD:
      for (i = 0; i < TD_POOL_SIZE; i++) {
        if (td[i].link_ptr == 0) {
          td[i].link_ptr = TERMINATE;
          return &td[i];
        }
      }
      break;

    case TYPE_QH:
      for (i = 0; i < QH_POOL_SIZE; i++) {
        if (qh[i].qh_ptr == 0 && qh[i].qe_ptr == 0) {
          qh[i].qh_ptr = qh[i].qe_ptr = TERMINATE;
          return &qh[i];
        }
      }
      break;

    default:
      DLOG("Unsupported Descriptor Type: %d", type);
      return (void *) 0;

  }

  DLOG("Error! Not enough descriptor in the pool! (type=%s)",
       (type == TYPE_TD ? "TD" : "QH"));
  return (void *) 0;
}

static int
sched_free (int type, void *res)
{
  UHCI_TD *res_td;
  UHCI_QH *res_qh;

  switch (type) {
    case TYPE_TD:
      res_td = (UHCI_TD *) res;
      res_td->link_ptr = 0;
      res_td->raw2 = 0;
      res_td->raw3 = 0;
      res_td->buf_ptr = 0;
      res_td->buf_vptr = 0;
      res_td->call_back = 0;
      break;

    case TYPE_QH:
      res_qh = (UHCI_QH *) res;
      res_qh->qh_ptr = 0;
      res_qh->qe_ptr = 0;
      break;

    default:
      return -1;
  }

  return 0;
}

/* Free continuous TDs of length len */
static int
free_tds (UHCI_TD * tds, int len)
{
  int count = 0, i = 0, end = 0;
  uint32 link_ptr;

  for (i = 0; i < len; i++) {
    link_ptr = tds->link_ptr;

    if (tds->link_ptr & TERMINATE) {
      end++;
    }

    sched_free (TYPE_TD, tds);
    count++;

    if (end)
      break;

    tds = TD_P2V (UHCI_TD *, link_ptr & LINK_MASK);
  }

  return count;
}

static void
disable_ehci (void)
{
  int func = 0x7;
  uint32_t pa_ubase = 0;
  unsigned long eecp = 0;
  uint32_t va_ubase = 0;
  uint32_t opregs = 0;
  uint32_t config_flag = 0;
  uint8_t hcbiossem = 0;

  DLOG ("EHCI: Routing all ports to companion controllers...");

  /* Get physical address of EHCI base */
  pa_ubase = pci_config_rd32 (bus, dev, func, 0x10);
  pa_ubase = pa_ubase & 0xFFFFFF00;

  /* Convert physical address to virtual address */
  va_ubase = (uint32_t) map_virtual_page ((pa_ubase & 0xFFFFF000) + 0x3);
  va_ubase += (pa_ubase & 0x0FFF);

#if 0
  DLOG ("EHCI Base Phys=%p Base Virt=%p", pa_ubase, va_ubase);
#endif

  eecp = (*((unsigned int *) (((char *) va_ubase) + 0x08)) & 0x0FF00) >> 8;

#if 0
  DLOG ("EECP=%p", eecp);
#endif

  hcbiossem = pci_config_rd8 (bus, dev, func, eecp + 0x02) & 0x01;
  DLOG ("HC BIOS SEM: %p", hcbiossem);
  pci_config_wr8 (bus, dev, func, eecp + 0x03, 0x01);

  for (;;) {
    if (((pci_config_rd8 (bus, dev, func, eecp + 0x02) & 0x01) == 0x0) &&
        ((pci_config_rd8 (bus, dev, func, eecp + 0x03) & 0x01)) == 0x01)
      break;
  }

  DLOG ("Quest got ownership of EHCI!");

  opregs = va_ubase + *(char *) va_ubase;
  config_flag = *((uint32_t *) ((char *) opregs + 0x40));

  /* Clear bit 0 of CONFIGFLAG to route all ports */
  *((uint32_t *) ((char *) opregs + 0x40)) = 0x0;
  delay (100);
}

static void
init_schedule (void)
{
  UHCI_QH *q;
  int i = 0;

  phys_frm = (frm_lst_ptr *) get_phys_addr ((void *) frame_list);

  for (i = 0; i < 1024; i++) {
    frame_list[i] = TERMINATE;
  }

  memset ((void *) td, 0, 32 * TD_POOL_SIZE);
  memset ((void *) qh, 0, 16 * QH_POOL_SIZE);

  int_qh = sched_alloc (TYPE_QH);
  ctl_qh = sched_alloc (TYPE_QH);
  blk_qh = sched_alloc (TYPE_QH);

  /* Points to next Queue Head (First Control QH). Set Q=1, T=0 */
  int_qh->qh_ptr =
    (((uint32_t) get_phys_addr ((void *) ctl_qh)) & LINK_MASK) + SELECT_QH;
  int_qh->qe_ptr = TERMINATE;

  /* Points to next Queue Head (First Bulk QH). Set Q=1, T=0 */
  ctl_qh->qh_ptr =
    (((uint32_t) get_phys_addr ((void *) blk_qh)) & LINK_MASK) + SELECT_QH;
  ctl_qh->qe_ptr = TERMINATE;

  /* Set this as the last Queue Head for now. T=1 */
  blk_qh->qh_ptr = TERMINATE;
  blk_qh->qe_ptr = TERMINATE;

  /* create 2nd level for bulk queue */
  q = sched_alloc (TYPE_QH);
  q->qh_ptr = qh->qe_ptr = TERMINATE;

  blk_qh->qe_ptr =
    (((uint32_t) get_phys_addr ((void *) q)) & LINK_MASK) + SELECT_QH;

  blk_qh = q;

#if 0
  DLOG ("int_qh va=%p ctl_qh va=%p qhp va=%p",
      int_qh, ctl_qh, qh);
#endif

#if 0
  DLOG ("frame_list va=%p pa=%p", frame_list, phys_frm);
#endif

  /* Link all the frame list pointers to interrupt queue. No ISO TD present now. */
  for (i = 0; i < 1024; i++) {
    frame_list[i] =
      (((uint32_t) get_phys_addr ((void *) int_qh)) & LINK_MASK) + SELECT_QH;
  }
}

static void
debug_dump_sched (UHCI_TD * tx_tds)
{
  // Dump the schedule for debugging

  for (;;) {
    DLOG ("TD dump: %p %p %p %p %p",
          tx_tds->link_ptr, tx_tds->raw2, tx_tds->raw3,
          tx_tds->buf_ptr, tx_tds->buf_vptr);
    DLOG ("  act_len=%d status=%p ioc=%d iso=%d ls=%d c_err=%d spd=%d",
          tx_tds->act_len, tx_tds->status, tx_tds->ioc, tx_tds->iso,
          tx_tds->ls, tx_tds->c_err, tx_tds->spd);
    DLOG ("  pid=0x%.02X addr=%d ep=%d tog=%d max_len=%d",
          tx_tds->pid, tx_tds->addr, tx_tds->endp, tx_tds->toggle, tx_tds->max_len);
#if 0
    if (tx_tds->buf_vptr) {
      uint32 *p = (uint32 *)tx_tds->buf_vptr;
      DLOG ("    %.08X %.08X %.08X %.08X",
            p[0], p[1], p[2], p[3]);
    }
#endif
    if (tx_tds->link_ptr & TERMINATE) break;
    if (tx_tds->link_ptr == 0) break;
    tx_tds = TD_P2V (UHCI_TD *, tx_tds->link_ptr & LINK_MASK);
  }
}

/* Hack that works around problem where HC does not update the
 * QH->qe_ptr with the current active TD.  For some reason it leaves
 * behind an inactive TD, which causes the HC to halt processing at
 * that point.  This function manually updates the QH with a physical
 * pointer to the next active TD. */
static void
fixup_tds (u32 *p_tdp)
{
  while (!(*p_tdp & TERMINATE)) {
    UHCI_TD *t = TD_P2V (UHCI_TD *, (*p_tdp) & LINK_MASK);
    if (t->status == 0)
      *p_tdp = t->link_ptr & (~DEPTH_FIRST);
    else
      break;
    p_tdp = &t->link_ptr;
  }
}

extern void
debug_dump_bulk_qhs (void)
{
  UHCI_QH *q = blk_qh;
  uint32 link_ptr;

  DLOG ("cmd=0x%.04X sts=0x%.04X frame=%d",
        GET_USBCMD (usb_base), GET_USBSTS (usb_base), GET_FRNUM (usb_base));

  for (;;) {
    link_ptr = q->qe_ptr;
    if (link_ptr & TERMINATE) {
      DLOG ("QH %p has no TDs", q);
    } else {
      DLOG ("QH %p has TD %p:", q, q->qe_ptr);
      debug_dump_sched (TD_P2V (UHCI_TD *, q->qe_ptr & LINK_MASK));
    }
    fixup_tds (&q->qe_ptr);
    link_ptr = q->qh_ptr;
    if (link_ptr & TERMINATE)
      break;
    q = QH_P2V (UHCI_QH *, link_ptr & LINK_MASK);
  }
}

static int
check_tds (UHCI_QH *tx_qh, UHCI_TD *tx_tds, uint32 *act_len)
{
  int status_count = 1;
  int status = 0;
  int len = 0;

  while (status_count) {
    UHCI_TD *tds = tx_tds;
    status_count = 0;

    fixup_tds (&tx_qh->qe_ptr);

    /* wait for IRQ if interrupts enabled */
    if (mp_enabled) {
      DLOGV ("wait %d", status_count);
      queue_append (&uhci_waitq, str ());
      schedule ();
    }

    len = 0;
    while (tds != TD_P2V (UHCI_TD *, 0)) {
      if (tds->status & 0x80) {
        status_count++;
      } else {
        len += (tds->act_len + 1) & 0x7FF;
        /* Check for short packet */
        if (tds->act_len != tds->max_len) {
          DLOG ("Short Packet! after %d bytes", len);
#if 1
          *act_len = len;
          return status;
#else
          debug_dump_sched (tx_tds);
#endif
        }
      }

      /* If the TD is STALLED, we report the error */
      if (tds->status & 0x40) {
        status = tds->status & 0x7F;

        debug_dump_sched (tx_tds);

        return status;
      }

      /* If the TD is timed out */
      if (tds->status & 0x04) {
        status = tds->status & 0x7F;
        return status;
      }

#if 0
      /* Got a NAK */
      if (tds->status & 0x08) {
        DLOG ("NAK! after %d bytes", len);
        *act_len = len;
        return status;
      }
#endif

      /* move to next TD in chain */
      tds = TD_P2V (UHCI_TD *, tds->link_ptr & LINK_MASK);
    }
  }

  *act_len = len;
  return status;
}

int
port_reset (uint8_t port)
{
  uint16_t portsc = 0;
  switch (port) {
  case 0:
    portsc = GET_PORTSC0 (usb_base);
    SET_PORTSC0 (usb_base, portsc | 0x0200);
    delay (10);
    SET_PORTSC0 (usb_base, portsc & ~0x0200);
    delay (1);
    SET_PORTSC0 (usb_base, portsc | 0x0004);
    delay (50);
    break;

  case 1:
    portsc = GET_PORTSC1 (usb_base);
    SET_PORTSC1 (usb_base, portsc | 0x0200);
    delay (10);
    SET_PORTSC1 (usb_base, portsc & ~0x0200);
    delay (1);
    SET_PORTSC1 (usb_base, portsc | 0x0004);
    delay (50);
    break;

  default:
    return -1;
  }

  return 0;
}

int
uhci_reset (void)
{
  if (usb_base == 0)
    return -1;

  /* Reset UHCI host controller (Global Reset) */
  SET_USBCMD (usb_base, 0x0004);
  delay (100);                  // USB Spec specifies 10ms only
  SET_USBCMD (usb_base, 0x0000);
  delay (100);

  SET_USBINTR (usb_base, 0x0F);
  SET_PCICMD (bus, dev, func, GET_PCICMD (bus, dev, func) & 0xFBFF);

  /* Set up the frame list for the HC */
  SET_FRBASEADD (usb_base, (uint32_t) phys_frm);
  /* Set SOF timing */
  SET_SOFMOD (usb_base, 0x40);
  /* Set Frame Number */
  SET_FRNUM (usb_base, 0x0);
  /* Now, start the HC */
  SET_USBCMD (usb_base, 0x0001);

  return 0;
}

#define USB_MAX_DEVICES 32
static USB_DEVICE_INFO devinfo[USB_MAX_DEVICES+1];
static uint next_address = 1;

#define USB_MAX_DEVICE_DRIVERS 32
static USB_DRIVER drivers[USB_MAX_DEVICE_DRIVERS];
static uint num_drivers = 0;

void
find_device_driver (USB_DEVICE_INFO *info, USB_CFG_DESC *cfgd, USB_IF_DESC *ifd)
{
  int d;
  for (d=0; d<num_drivers; d++) {
    if (drivers[d].probe (info, cfgd, ifd))
      return;
  }
}

void
dlog_devd (USB_DEV_DESC *devd)
{
  DLOG ("DEVICE DESCRIPTOR len=%d type=%d bcdUSB=0x%x",
        devd->bLength, devd->bDescriptorType, devd->bcdUSB);
  DLOG ("  class=0x%x subclass=0x%x proto=0x%x maxpkt0=%d",
        devd->bDeviceClass, devd->bDeviceSubClass, devd->bDeviceProtocol,
        devd->bMaxPacketSize0);
  DLOG ("  vendor=0x%x product=0x%x bcdDevice=0x%x numcfgs=%d",
        devd->idVendor, devd->idProduct, devd->bcdDevice,
        devd->bNumConfigurations);
}

void
dlog_info (USB_DEVICE_INFO *info)
{
  DLOG ("ADDRESS %d", info->address);
  dlog_devd (&info->devd);

#if 0
  uint8 strbuf[64];
  uint8 str[32];
#define do_str(slot,label)                                              \
  if (info->devd.slot != 0 &&                                           \
      uhci_get_string (info->address, info->devd.slot, 0,               \
                       sizeof (strbuf), strbuf,                         \
                       info->devd.bMaxPacketSize0)                      \
      == 0) {                                                           \
    memset (str, 0, sizeof (str));                                      \
    if (uhci_interp_string ((USB_STR_DESC *)strbuf, sizeof (strbuf), 0, \
                            str, sizeof (str)-1) > 0) {                 \
      DLOG ("  "label": %s", str);                                      \
    }                                                                   \
  }

  do_str (iManufacturer, "Manufacturer");
  do_str (iProduct, "Product");
#undef do_str
#endif
}

/* figures out what device is attached as address 0 */
bool
uhci_enumerate (void)
{
  USB_DEV_DESC devd;
  USB_CFG_DESC *cfgd;
  USB_IF_DESC *ifd;
#define TEMPSZ 256
  uint8 temp[TEMPSZ], *ptr;
  uint curdev = next_address;
  sint status, c, i, total_length=0;
  USB_DEVICE_INFO *info;

  if (curdev > USB_MAX_DEVICES) {
    DLOG ("uhci_enumerate: too many devices");
    return FALSE;
  }

  DLOG ("uhci_enumerate: curdev=%d", curdev);

  /* clear device info */
  info = &devinfo[curdev];
  memset (info, 0, sizeof (USB_DEVICE_INFO));
  info->address = 0;
  info->host_type = USB_TYPE_HC_UHCI;

  /* OK, here is the deal. The spec says you should use the maximum
   * packet size in the data phase of control transfer if the data
   * is larger than one packet. Since we do not want to support low
   * speed device for now, the bMaxPacketSize0 is always set to 64
   * bytes for full speed device. So, do not be surprised if your USB
   * mouse does not work in Quest!
   */
  info->devd.bMaxPacketSize0 = 64;

  memset (&devd, 0, sizeof (USB_DEV_DESC));

  /* get device descriptor */
  status = usb_get_descriptor (info, USB_TYPE_DEV_DESC, 0, 0,
                               sizeof (USB_DEV_DESC), &devd);
  if (status != 0)
    goto abort;

  if (devd.bMaxPacketSize0 == 8) {
    /* get device descriptor */
    info->devd.bMaxPacketSize0 = 8;
    status = usb_get_descriptor (info, USB_TYPE_DEV_DESC, 0, 0,
                                 sizeof (USB_DEV_DESC), &devd);
    if (status != 0)
      goto abort;
  }

  if (devd.bNumConfigurations == 255)
    devd.bNumConfigurations = 1; /* hack */

  /* Update device info structure. Put it in USB core might be better */
  memcpy (&info->devd, &devd, sizeof (USB_DEV_DESC));

  /* assign an address */
  if (usb_set_address (info, curdev) != 0)
    goto abort;
  DLOG ("uhci_enumerate: set (0x%x, 0x%x, 0x%x) to addr %d",
        devd.bDeviceClass, devd.idVendor, devd.idProduct, curdev);
  delay (2);

  DLOG ("uhci_enumerate: num configs=%d", devd.bNumConfigurations);

  /* Update device info structure for new address. */
  info->address = curdev;

  dlog_info (info);

  for (c=0; c<devd.bNumConfigurations; c++) {
    /* get a config descriptor for size field */
    memset (temp, 0, TEMPSZ);
    status = usb_get_descriptor (info, USB_TYPE_CFG_DESC, c, 0,
                                 sizeof (USB_CFG_DESC), temp);
    if (status != 0) {
      DLOG ("uhci_enumerate: failed to get config descriptor for c=%d", c);
      goto abort;
    }

    cfgd = (USB_CFG_DESC *)temp;
    DLOG ("uhci_enumerate: c=%d cfgd->wTotalLength=%d", c, cfgd->wTotalLength);
    total_length += cfgd->wTotalLength;
  }

  DLOG ("uhci_enumerate: total_length=%d", total_length);

  /* allocate memory to hold everything */
  pow2_alloc (total_length, &info->raw);
  if (!info->raw) {
    DLOG ("uhci_enumerate: pow2_alloc (%d) failed", total_length);
    goto abort;
  }

  /* read all cfg, if, and endpoint descriptors */
  ptr = info->raw;
  for (c=0; c<devd.bNumConfigurations; c++) {
    /* obtain precise size info */
    memset (temp, 0, TEMPSZ);
    status = usb_get_descriptor (info, USB_TYPE_CFG_DESC, c, 0,
                                 sizeof (USB_CFG_DESC), temp);
    if (status != 0) {
      DLOG ("uhci_enumerate: failed to get config descriptor for c=%d", c);
      goto abort_mem;
    }
    cfgd = (USB_CFG_DESC *)temp;

    /* get cfg, if, and endp descriptors */
    status =
      usb_get_descriptor (info, USB_TYPE_CFG_DESC, c, 0, cfgd->wTotalLength, ptr);
    if (status != 0)
      goto abort_mem;

    cfgd = (USB_CFG_DESC *)ptr;
    DLOG ("uhci_enumerate: cfg %d has num_if=%d", c, cfgd->bNumInterfaces);
    ptr += cfgd->wTotalLength;
  }

  /* incr this here because hub drivers may recursively invoke enumerate */
  next_address++;

  /* parse cfg and if descriptors */
  ptr = info->raw;
  for (c=0; c<devd.bNumConfigurations; c++) {
    cfgd = (USB_CFG_DESC *) ptr;
    ptr += cfgd->bLength;
    for (i=0; i<cfgd->bNumInterfaces; i++) {
      /* find the next if descriptor, skipping any class-specific stuff */
      for (ifd = (USB_IF_DESC *) ptr;
           ifd->bDescriptorType != USB_TYPE_IF_DESC;
           ifd = (USB_IF_DESC *)((uint8 *)ifd + ifd->bLength)) {
        //DLOG ("ifd=%p len=%d type=0x%x", ifd, ifd->bLength, ifd->bDescriptorType);
      }
      ptr = (uint8 *) ifd;
      DLOG ("uhci_enumerate: examining (%d, %d) if_class=0x%X sub=0x%X proto=0x%X #endp=%d",
            c, i, ifd->bInterfaceClass,
            ifd->bInterfaceSubClass,
            ifd->bInterfaceProtocol,
            ifd->bNumEndpoints);

      /* find a device driver interested in this interface */
      find_device_driver (info, cfgd, ifd);

      ptr += ifd->bLength;
    }
    ptr = ((uint8 *)cfgd) + cfgd->wTotalLength;
  }

  /* --??-- what happens if more than one driver matches more than one config? */

  return TRUE;

 abort_mem:
  pow2_free (info->raw);
 abort:
  return FALSE;
}

bool
usb_register_driver (USB_DRIVER *driver)
{
  if (num_drivers >= USB_MAX_DEVICE_DRIVERS) return FALSE;
  memcpy (&drivers[num_drivers], driver, sizeof (USB_DRIVER));
  num_drivers++;
  return TRUE;
}

static uint irq_line;

int
uhci_init (void)
{
  uint i, device_index, irq_pin;
  pci_device uhci_device;
  pci_irq_t irq;

  memset (toggles, 0, sizeof (toggles));

  if (mp_ISA_PC) {
    DLOG ("Cannot operate without PCI");
    return FALSE;
  }

  /* Find the UHCI device on the PCI bus */
  device_index = ~0;
  i=0;
  while (pci_find_device (0xFFFF, 0xFFFF, 0x0C, 0x03, i, &i)) {
    if (pci_get_device (i, &uhci_device)) {
      if (uhci_device.progIF == 0) {
        device_index = i;
        break;
      }
      i++;
    } else break;
  }

  if (device_index == ~0) {
    DLOG ("Unable to find compatible device on PCI bus");
    return FALSE;
  }

  if (!pci_get_device (device_index, &uhci_device)) {
    DLOG ("Unable to get PCI device from PCI subsystem");
    return FALSE;
  }

  bus = uhci_device.bus;
  dev = uhci_device.slot;
  func = uhci_device.func;

  DLOG ("Using PCI bus=%x dev=%x func=%x", bus, dev, func);

  if (!pci_get_interrupt (device_index, &irq_line, &irq_pin)) {
    DLOG ("Unable to get IRQ");
    return FALSE;
  }

  DLOG ("Using IRQ pin=%X", irq_pin);

#define UHCI_VECTOR 0x50
  if (pci_irq_find (bus, dev, irq_pin, &irq)) {
    /* use PCI routing table */
    DLOG ("Found PCI routing entry irq.gsi=0x%x", irq.gsi);
    pci_irq_map (&irq, UHCI_VECTOR, 0x01,
                 IOAPIC_DESTINATION_LOGICAL, IOAPIC_DELIVERY_FIXED);
    irq_line = irq.gsi;
  } else {
#define IOAPIC_FLAGS 0x010000000000A800LL
    /* assume irq_line is correct */
    DLOG ("Falling back to PCI config space INT_LN=0x%x", irq_line);
    IOAPIC_map_GSI (irq_line, UHCI_VECTOR, IOAPIC_FLAGS);
  }

  set_vector_handler (UHCI_VECTOR, uhci_irq_handler);


  td_phys = (uint32) get_phys_addr ((void *) td);
  qh_phys = (uint32) get_phys_addr ((void *) qh);
  DLOG ("td@%p td_phys@%p qh@%p qh_phys@%p",
        td, td_phys, qh, qh_phys);


  if (uhci_device.device != 0x7020)
    disable_ehci ();

  /* Disable USB Legacy Support, set PIRQ */
  DLOG ("UHCI LEGSUP: %p", GET_LEGACY (bus, dev, func));
  SET_LEGACY(bus, dev, func, 0x2000); /* PIRQ enabled */
  DLOG ("Set PIRQDEN");

  if (!pci_decode_bar (device_index, 4, NULL, &usb_base, NULL)) {
    DLOG ("unable to decode BAR4");
    return FALSE;
  }

  DLOG ("usb_base=0x%.04X", usb_base);

  /* Initiate UHCI internal data structure */
  init_schedule ();
  /* Perform global reset on host controller */
  uhci_reset ();

#if 1
#include <drivers/usb/usb_tests.h>

  DLOG ("begin enumeration");

  for (i=0;i<2;i++) {
    port_reset (i);

    uhci_enumerate ();
    show_usb_regs (bus, dev, func);
  }
  DLOG ("end enumeration");

#else
#include <drivers/usb/usb_tests.h>

  isochronous_transfer_test ();
  show_usb_regs(bus, dev, func);

#endif

  return 0;
}

int
uhci_isochronous_transfer (
    uint8_t address,
    uint8_t endpoint,
    addr_t data,
    int data_len,
    int *act_len,
    uint16_t frm,
    uint8_t direction,
    void (*func)(addr_t))
{
  UHCI_TD *iso_td = 0;
  UHCI_TD *idx_td = 0;
  int max_packet_len = USB_MAX_LEN;
  frm_lst_ptr entry = 0;

  if ((data_len - 1) > max_packet_len)
    return -1;

  iso_td = sched_alloc (TYPE_TD);
  iso_td->status = 0x80;
  iso_td->c_err = 0;
  iso_td->ioc = 0; // --??-- For test, mask the interrupt
  iso_td->iso = 1;
  iso_td->spd = 0;
  iso_td->pid = (direction == DIR_IN) ? UHCI_PID_IN : UHCI_PID_OUT;
  iso_td->addr = address;
  iso_td->endp = endpoint;
  iso_td->toggle = 0;
  iso_td->max_len = data_len - 1;
  iso_td->buf_ptr = (uint32_t) get_phys_addr ((void *) data);
  iso_td->buf_vptr = data;
  iso_td->call_back = func;

  entry = frame_list[frm];

  if (!(entry & TERMINATE) && !(entry & SELECT_QH)) {
    idx_td = TD_P2V (UHCI_TD *, entry & LINK_MASK);

    while (!(idx_td->link_ptr & TERMINATE) && !(idx_td->link_ptr & SELECT_QH)) {
      idx_td = TD_P2V (UHCI_TD *, idx_td->link_ptr & LINK_MASK);
    }

    iso_td->link_ptr = idx_td->link_ptr;
    idx_td->link_ptr =
      (uint32_t) get_phys_addr ((void *) iso_td) & LINK_MASK;
  } else {
    iso_td->link_ptr =
      (((uint32_t) get_phys_addr ((void *) int_qh)) & LINK_MASK) + SELECT_QH;
    frame_list[frm] = (uint32_t) get_phys_addr ((void *) iso_td) & LINK_MASK;
  }

  //queue_append (&uhci_waitq, str ());
  //schedule ();
  while (iso_td->status & 0x80);

  *act_len = iso_td->act_len + 1;
  frame_list[frm] = iso_td->link_ptr;
  sched_free (TYPE_TD, iso_td);

#if 0
  delay (500);
  delay (1000);
  DLOG ("The status of iso_td: %p ActLen: %p", iso_td->status, iso_td->act_len);
  frame_list[frm] = iso_td->link_ptr;
  sched_free (TYPE_TD, iso_td);
#endif

  return 0;
}

/* find a free QH to queue up the TDs, or alloc a new one */
static UHCI_QH *
link_td_to_free_qh (UHCI_QH *q, UHCI_TD *t)
{
  for (;;) {
    //DLOG ("examining QH: %p qe=%p qh=%p", q, q->qe_ptr, q->qh_ptr);
    if (q->qe_ptr & TERMINATE) {
      q->qe_ptr = TD_V2P (uint32, t) & LINK_MASK;
      //DLOG ("using existing QH %p", q);
      return q;
    }
    if (q->qh_ptr & TERMINATE) {
      UHCI_QH *newq = sched_alloc (TYPE_QH);
      if (newq == NULL) return NULL;
      newq->qe_ptr = (TD_V2P (uint32, t) & LINK_MASK) | SELECT_TD;
      newq->qh_ptr = TERMINATE;
      q->qh_ptr = (QH_V2P (uint32, newq) & LINK_MASK) | SELECT_QH;
      //DLOG ("created new QH %p", newq);
      return newq;
    }

    /* advance to next qh in list */
    q = QH_P2V (UHCI_QH *, q->qh_ptr & LINK_MASK);
  }
}

int
uhci_bulk_transfer(
    uint8_t address,
    uint8_t endpoint,
    addr_t data,
    int data_len,
    int packet_len,
    uint8_t direction,
    uint32 *act_len)
{
  UHCI_TD *tx_tds = 0;
  UHCI_TD *data_td = 0;
  UHCI_TD *idx_td = 0;
  UHCI_QH *act_qh;
  int max_packet_len = ((packet_len - 1) >= USB_MAX_LEN) ? USB_MAX_LEN : packet_len - 1;
  int i = 0, num_data_packets = 0, data_left = 0, return_status = 0, tog_idx;

  DLOGV ("bulk: %d %d %d %c", address, endpoint, data_len,
         direction == DIR_IN ? 'I' : 'O');
  num_data_packets = (data_len + max_packet_len) / (max_packet_len + 1);
  data_left = data_len;

  for(i = 0; i < num_data_packets; i++) {
    data_td = sched_alloc(TYPE_TD);
    data_td->link_ptr = TERMINATE;
    if(tx_tds == 0)
      tx_tds = data_td;
    else
      idx_td->link_ptr = (((uint32_t)get_phys_addr((void*)data_td)) & LINK_MASK) + DEPTH_FIRST;

    idx_td = data_td;
    data_td->status = 0x80;
    data_td->c_err = 3;
    data_td->ioc = data_td->iso = 0;
    data_td->spd = (direction == DIR_IN ? 1 : 0);

    data_td->pid = (direction == DIR_IN) ? UHCI_PID_IN : UHCI_PID_OUT;
    data_td->addr = address;
    data_td->endp = endpoint;

    tog_idx = (address * 32 + (endpoint + ((direction == DIR_IN) << 4)));
    data_td->toggle = (BITMAP_TST (toggles, tog_idx) ? 1 : 0);
    if (data_td->toggle)
      BITMAP_CLR (toggles, tog_idx);
    else
      BITMAP_SET (toggles, tog_idx);

    data_td->max_len = (data_left > (max_packet_len + 1)) ? max_packet_len : data_left - 1;
    data_td->buf_ptr = (uint32_t)get_phys_addr((void*)data);
    data_td->buf_vptr = data;

    if(data_left <= (max_packet_len + 1)) {
      data_td->ioc = 1;
      break;
    } else {
      data += (data_td->max_len + 1);
      data_left -= (data_td->max_len + 1);
    }

    /* set last packet IOC */
    if (i == num_data_packets - 1)
      data_td->ioc = 1;
  }


#if 0
  DLOG ("Dumping tx_tds...");
  debug_dump_sched (tx_tds);
  DLOG ("... done");
#endif

  /* Initiate our bulk transactions */
  act_qh = link_td_to_free_qh (blk_qh, tx_tds);

  if (act_qh == NULL) {
    DLOG ("act_qh == NULL");
    return -1;
  }

  /* Check the status of all the packets in the transaction */
  return_status = check_tds(act_qh, tx_tds, act_len);
  if (return_status != 0) {
    DLOG ("bulk: return_status != 0");
  } else {
    DLOGV ("complete: %d len %d", return_status, *act_len);
  }

  /* unlink any remaining active TDs if short packet was detected */
  if (*act_len != data_len)
    act_qh->qe_ptr = TERMINATE;

  free_tds(tx_tds, num_data_packets);

  return return_status;
}

int
uhci_control_transfer (
    uint8_t address,
    addr_t setup_req,    /* Use virtual address here */
    int setup_len,
    addr_t setup_data,   /* Use virtual address here */
    int data_len,
    int packet_len)      /* 64 bytes for Full-speed and 8 bytes for Low-speed */
{
  UHCI_TD *tx_tds = 0;
  UHCI_TD *td_idx = 0;
  UHCI_TD *status_td = 0;
  UHCI_TD *data_td = 0;
  int i = 0, num_data_packets = 0, data_left = 0, return_status = 0;
  uint8_t pid = 0, toggle = 0;
  addr_t data = 0;
  /* Using the default pipe for control transfer */
  uint8_t endpoint = 0;
  uint32_t act_len;

  /*
   * USB specification max packet length :  1023 bytes
   * UHCI specification max packet length : 1280 bytes
   */
  /* This is encoded as n-1 */
  int max_packet_len =
    ((packet_len - 1) >= USB_MAX_LEN) ? USB_MAX_LEN : packet_len - 1;
  num_data_packets = (data_len + max_packet_len) / (max_packet_len + 1);

  /* Constructing the setup packet */
  tx_tds = sched_alloc (TYPE_TD);
  tx_tds->status = 0x80;        // Set status of the descriptor to active
  tx_tds->c_err = 3;            // Set counter to 3 errors
  tx_tds->ioc = tx_tds->iso = tx_tds->spd = 0;

  tx_tds->pid = UHCI_PID_SETUP;
  tx_tds->addr = address;
  tx_tds->endp = endpoint;
  tx_tds->toggle = toggle;      // For setup packet, it is always DATA0
  tx_tds->max_len = setup_len - 1;      // This field is encoded as n-1

  tx_tds->buf_ptr = (uint32_t) get_phys_addr ((void *) setup_req);
  tx_tds->buf_vptr = setup_req;

  /* Constructing the data packets */
  td_idx = tx_tds;
  toggle = 1;
  data_left = data_len;
  data = setup_data;
  pid = (*((char *) setup_req) & 0x80) ? UHCI_PID_IN : UHCI_PID_OUT;

  for (i = 0; i < num_data_packets; i++) {
    data_td = sched_alloc (TYPE_TD);

    /* Link TDs, depth first */
    td_idx->link_ptr =
      (((uint32_t) get_phys_addr ((void *) data_td)) & LINK_MASK) + DEPTH_FIRST;
    td_idx = data_td;

    data_td->status = 0x80;
    data_td->c_err = 3;
    data_td->ioc = data_td->iso = data_td->spd = 0;
    data_td->pid = pid;
    data_td->addr = address;
    data_td->endp = endpoint;
    data_td->toggle = toggle;
    data_td->max_len =
      (data_left > (max_packet_len + 1)) ? max_packet_len : data_left - 1;
    data_td->buf_ptr = (uint32_t) get_phys_addr ((void *) data);
    data_td->buf_vptr = data;

    if (data_left <= (max_packet_len + 1)) {
      break;
    } else {
      data += (data_td->max_len + 1);
      data_left -= (data_td->max_len + 1);
      toggle = (toggle == 1) ? 0 : 1;
    }
  }

  /* Constructing handshake packet */
  pid = (pid == UHCI_PID_IN) ? UHCI_PID_OUT : UHCI_PID_IN;
  toggle = 1;                   // For status stage, always use DATA1
  status_td = sched_alloc (TYPE_TD);
  td_idx->link_ptr =
    (((uint32_t) get_phys_addr ((void *) status_td)) & LINK_MASK) + DEPTH_FIRST;

  status_td->link_ptr = TERMINATE;   // Terminate
  status_td->status = 0x80;
  status_td->c_err = 3;
  status_td->ioc = status_td->iso = status_td->spd = 0;
  status_td->ioc = 1;
  status_td->pid = pid;
  status_td->addr = address;
  status_td->endp = endpoint;
  status_td->toggle = toggle;
  status_td->max_len = USB_NULL_PACKET;

  /* debug_dump_sched (tx_tds); */

  /* Initiate our control transaction */
  ctl_qh->qe_ptr =
    (((uint32_t) get_phys_addr ((void *) tx_tds)) & LINK_MASK) + 0x0;

  /* Check the status of all the packets in the transaction */
  return_status = check_tds (ctl_qh, tx_tds, &act_len);

  free_tds (tx_tds, num_data_packets + 2);

  return return_status;
}

int
uhci_get_descriptor (
    uint8_t address,
    uint16_t dtype,   /* Descriptor type */
    uint16_t dindex,   /* Descriptor index */
    uint16_t index,    /* Zero or Language ID */
    uint16_t length,   /* Descriptor length */
    addr_t desc,
    uint8_t packet_size)
{
  USB_DEV_REQ setup_req;
  setup_req.bmRequestType = 0x80;       // Characteristics of request, see spec, P183, Rev 1.1
  setup_req.bRequest = USB_GET_DESCRIPTOR;
  setup_req.wValue = (dtype << 8) + dindex;
  setup_req.wIndex = index;
  setup_req.wLength = length;

  return uhci_control_transfer (address,
      (addr_t) & setup_req, sizeof (USB_DEV_REQ),
      desc, length, packet_size);
}

sint
uhci_get_string (uint8_t address, uint16_t index, uint16_t lang,
                 uint16_t length, void *buffer, uint8_t pktsize)
{
  return uhci_get_descriptor (address, USB_TYPE_STR_DESC, index, lang,
                              length, buffer, pktsize);
}

sint
uhci_interp_string (USB_STR_DESC *string, uint16_t length, uint16_t lang,
                    uint8 *output, uint16_t out_len)
{
  sint i, j;
  if (lang == 0) {
    /* assume UTF-16 encoding */
    length = (length < string->bLength ? length : string->bLength);
    /* skip first 2 bytes */
    length -= 2;
    for (i=0, j=0; i<length && j<out_len; i+=2, j++) {
      output[j] = string->bString[i];
    }
    return j;
  } else {
    /* unsupported */
    DLOG ("unimplemented: lang!=0");
    return 0;
  }
}

int
uhci_set_address (uint8_t old_addr, uint8_t new_addr, uint8_t packet_size)
{
  sint status;
  USB_DEV_REQ setup_req;
  setup_req.bmRequestType = 0x0;
  setup_req.bRequest = USB_SET_ADDRESS;
  setup_req.wValue = new_addr;
  setup_req.wIndex = 0;
  setup_req.wLength = 0;

  status = uhci_control_transfer (old_addr,
      (addr_t) & setup_req, sizeof (USB_DEV_REQ), 0,
      0, packet_size);
  if (status == 0) {
    toggles[old_addr] = toggles[new_addr] = 0;
  }
  return status;
}

int
uhci_get_configuration (uint8_t addr, uint8_t packet_size)
{
  USB_DEV_REQ setup_req;
  uint8_t num = -1;
  setup_req.bmRequestType = 0x80;
  setup_req.bRequest = USB_GET_CONFIGURATION;
  setup_req.wValue = 0;
  setup_req.wIndex = 0;
  setup_req.wLength = 1;

  uhci_control_transfer (addr, (addr_t) & setup_req, sizeof (USB_DEV_REQ),
      (addr_t) & num, 1, packet_size);

  return num;
}

int
uhci_set_configuration (uint8_t addr, uint8_t conf, uint8_t packet_size)
{
  USB_DEV_REQ setup_req;
  setup_req.bmRequestType = 0x0;
  setup_req.bRequest = USB_SET_CONFIGURATION;
  setup_req.wValue = conf;
  setup_req.wIndex = 0;
  setup_req.wLength = 0;
  /*
   * A bulk endpoint's toggle is initialized to DATA0 when any
   * configuration event is experienced
   */
  toggles[addr] = 0;


  return uhci_control_transfer (addr,
      (addr_t) & setup_req, sizeof (USB_DEV_REQ), 0,
      0, packet_size);
}

int
uhci_set_interface (uint8_t addr, uint16_t alt, uint16_t interface,
    uint8_t packet_size)
{
  USB_DEV_REQ setup_req;
  setup_req.bmRequestType = 0x01;
  setup_req.bRequest = USB_SET_INTERFACE;
  setup_req.wValue = alt;
  setup_req.wIndex = interface;
  setup_req.wLength = 0;

  return uhci_control_transfer (addr,
      (addr_t) & setup_req, sizeof (USB_DEV_REQ), 0,
      0, packet_size);
}

int
uhci_get_interface (uint8_t addr, uint16_t interface, uint8_t packet_size)
{
  USB_DEV_REQ setup_req;
  uint8_t alt = -1;
  setup_req.bmRequestType = 0x81;
  setup_req.bRequest = USB_GET_INTERFACE;
  setup_req.wValue = 0;
  setup_req.wIndex = interface;
  setup_req.wLength = 1;

  uhci_control_transfer (addr, (addr_t) & setup_req, sizeof (USB_DEV_REQ),
      (addr_t) & alt, 1, packet_size);

  return alt;
}

static uint32
uhci_irq_handler (uint8 vec)
{
  uint64 v;
  uint16 status;

  lock_kernel ();

#if 0
  v = IOAPIC_read64 (0x10 + (irq_line * 2));
  DLOG ("(1) IOAPIC (irq_line=0x%x) says %p %p",
        irq_line, (uint32) (v >> 32), (uint32) v);
#endif

  /* mask it */
  //IOAPIC_map_GSI (irq_line, UHCI_VECTOR, IOAPIC_FLAGS | 0x10000);

  /* Check the source of the interrupt received */
  status = 0;
  status = GET_USBSTS(usb_base);

#if 0
  DLOG ("An interrupt is caught from usb IRQ_LN! (USBSTS=0x%x PCISTS=0x%x)",
        status,
        pci_config_rd16 (bus, dev, func, 0x06));
#endif

  if((status & 0x3F) == 0) {
    DLOG("Interrupt is probably not from UHCI. Nothing will be done.");
    goto finish;
  }

  if(status & 0x20) {
    /* Host Controller Halted */
    DLOG("HCHalted detected!");
    status |= 0x20; /* Clear the interrupt by writing a 1 to it */
    debug_dump_bulk_qhs ();
  }

  if(status & 0x10) {
    /* Host Controller Process Error */
    DLOG("HC Process Error detected!");
    status |= 0x10; /* Clear the interrupt by writing a 1 to it */
  }

  if(status & 0x08) {
    /* Host System Error */
    DLOG("Host System Error detected!");
    status |= 0x08; /* Clear the interrupt by writing a 1 to it */
  }

  if(status & 0x04) {
    /* Resume Detect */
    DLOG("Resume detected!");
    status |= 0x04; /* Clear the interrupt by writing a 1 to it */
  }

  if(status & 0x02) {
    /* USB Error Interrupt */
    DLOG("USB Error Interrupt detected!");
    status |= 0x02; /* Clear the interrupt by writing a 1 to it */

    /* wake-up any waiting threads */
    wakeup_queue (&uhci_waitq);
  }

  if(status & 0x01) {
    DLOGV ("IRQ");
    /*
     * This is possibly an IOC or short packet.
     * We need to visit the whole schedule for now
     * to decide what exactly happened.
     */
    status |= 0x01; /* Clear the interrupt by writing a 1 to it */

    /* wake-up any waiting threads */
    wakeup_queue (&uhci_waitq);

#if 0
    int i;
    for(i = 0; i < TD_POOL_SIZE; i++) {
      if(td[i].link_ptr && !(td[i].status & 0x80)) {
        /* Is this an IOC? */
        if(td[i].ioc) {
          /* Call the call back function from user if it exists */
          if(td[i].call_back) {
            (td[i].call_back)(td[i].buf_vptr);
            DLOG("Call back function called: 0x%x, buf_vptr: 0x%x",
                td[i].call_back, td[i].buf_vptr);
          }
          /* Release this TD if it is isochronous */
          if(td[i].iso) sched_free(TYPE_TD, &td[i]);
        }

        /* Short packet detected */
        if(td[i].spd) {
          /* Release this TD if it is isochronous */
          if(td[i].iso) sched_free(TYPE_TD, &td[i]);
        }
      }
    }
#endif
  }

 finish:

  /* clear the interrupt(s) */
  SET_USBSTS (usb_base, status);

#if 0
  v = IOAPIC_read64 (0x10 + (irq_line * 2));
  DLOG ("(2) IOAPIC (irq_line=0x%x) says %p %p",
        irq_line, (uint32) (v >> 32), (uint32) v);
#endif

  /* unmask it */
  //IOAPIC_map_GSI (irq_line, UHCI_VECTOR, IOAPIC_FLAGS);

  unlock_kernel ();

  return 0;
}

#define printf com1_printf
#define print com1_puts
#define putx  com1_putx
#define putchar com1_putc

extern void
uhci_show_regs (void)
{
  uint16_t base_addr = 0;
  uint32_t ldata = 0;
  uint16_t wdata = 0;
  uint8_t bdata = 0;
#if 1
  print ("\nUHCI Controller PCI Register Address Map\n\n");

  wdata = pci_config_rd16 (bus, dev, func, 0x00);
  print ("VID: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0x02);
  print ("DID: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0x04);
  print ("PCICMD: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0x06);
  print ("PCISTS: 0x");
  putx (wdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x08);
  print ("RID: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x09);
  print ("PI: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x0A);
  print ("SCC: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x0B);
  print ("BCC: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x0D);
  print ("MLT: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x0E);
  print ("HEADTYP: 0x");
  putx (bdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0x20);
  print ("BASE: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0x2C);
  print ("SVID: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0x2E);
  print ("SID: 0x");
  putx (wdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x3C);
  print ("INT_LN: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x3D);
  print ("INT_PN: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x60);
  print ("USB_RELNUM: 0x");
  putx (bdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0xC0);
  print ("USB_LEGKEY: 0x");
  putx (wdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0xC4);
  print ("USB_RES: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0xC8);
  print ("CWP: 0x");
  putx (bdata);
  putchar ('\n');
#endif
  base_addr = pci_config_rd16 (bus, dev, func, 0x20);
  base_addr &= 0xFFE0;

  print ("\nBase address for USB I/O Registers: 0x");
  putx (base_addr);
  putchar ('\n');

  print ("\nUSB I/O Registers\n\n");

  wdata = inw (base_addr + 0x00);
  print ("USBCMD: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = inw (base_addr + 0x02);
  print ("USBSTS: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = inw (base_addr + 0x04);
  print ("USBINTR: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = inw (base_addr + 0x06);
  print ("FRNUM: 0x");
  putx (wdata);
  putchar ('\n');

  ldata = inl (base_addr + 0x08);
  print ("FRBASEADD: 0x");
  putx (ldata);
  putchar ('\n');

  bdata = inb (base_addr + 0x0C);
  print ("SOFMOD: 0x");
  putx (bdata);
  putchar ('\n');

  wdata = inw (base_addr + 0x10);
  print ("PORTSC0: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = inw (base_addr + 0x12);
  print ("PORTSC1: 0x");
  putx (wdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x3C);
  print ("INT_LN: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x3D);
  print ("INT_PN: 0x");
  putx (bdata);
  putchar ('\n');

  return;
}

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
