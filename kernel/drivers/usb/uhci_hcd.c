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
#include <kernel.h>

#define DEBUG_UHCI

#ifdef DEBUG_UHCI
#define DLOG(fmt,...) DLOG_PREFIX("UHCI",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

/* List of compatible cards (ended by { 0xFFFF, 0xFFFF }) */
static struct { uint16 vendor, device; } compatible_ids[] = {
  { 0x8086, 0x27C8 },           /* Intel ICH7 Family UHCI */
  { 0x8086, 0x2934 },           /* Intel ICH9 Family UHCI */
  { 0x8086, 0x7020 },           /* Intel PIIX3 USB controller */
  { 0xFFFF, 0xFFFF }
};

static int bus;                 /* set by PCI probing in uhci_init */
static int dev;
static int func;

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
static uint8_t glb_toggle = 0;

uint32 td_phys;

/* Virtual-to-Physical */
#define TD_V2P(ty,p) ((ty)((((uint) (p)) - ((uint) td))+td_phys))
/* Physical-to-Virtual */
#define TD_P2V(ty,p) ((ty)((((uint) (p)) - td_phys)+((uint) td)))

/*
 * This is non-reentrant! Fortunately, we do not have concurrency yet:-)
 * This function returns the available Queue Head or Transfer Descriptor.
 * It will probably be removed later.
 */
static void *
sched_alloc (int type)
{
  static int i = 0;
  static int j = 0;
  int k = 0;

  switch (type) {
  case TYPE_TD:
    k = ++i;
    while (1) {
      if (td[i].link_ptr == 0)
        return &td[i];
      if (++i >= TD_POOL_SIZE) {
        i = 0;
      }
      if (i == k)
        return (void *) 0;
    }
    break;

  case TYPE_QH:
    k = ++j;
    while (1) {
      if (qh[j].qh_ptr == 0 && qh[j].qe_ptr == 0)
        return &qh[j];
      if (++j >= QH_POOL_SIZE) {
        j = 0;
      }
      if (j == k)
        return (void *) 0;
    }
    break;

  default:
    return (void *) 0;

  }

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

    if ((tds->link_ptr & 0x01) == 0x01) {
      end++;
    }

    if ((tds->status & 0x80) == 0) {
      sched_free (TYPE_TD, tds);
      count++;
    }

    if (end)
      break;

    tds = TD_P2V (UHCI_TD *, link_ptr & 0xFFFFFFF0);
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
  int i = 0;

  phys_frm = (frm_lst_ptr *) get_phys_addr ((void *) frame_list);

  for (i = 0; i < 1024; i++) {
    frame_list[i] = 0x01;
  }

  memset ((void *) td, 0, 32 * TD_POOL_SIZE);
  memset ((void *) qh, 0, 16 * QH_POOL_SIZE);

  int_qh = sched_alloc (TYPE_QH);
  ctl_qh = sched_alloc (TYPE_QH);
  blk_qh = sched_alloc (TYPE_QH);

  /* Points to next Queue Head (First Control QH). Set Q=1, T=0 */
  int_qh->qh_ptr =
    (((uint32_t) get_phys_addr ((void *) ctl_qh)) & 0xFFFFFFF0) + 0x02;
  int_qh->qe_ptr = 0x01;

  /* Points to next Queue Head (First Bulk QH). Set Q=1, T=0 */
  ctl_qh->qh_ptr =
    (((uint32_t) get_phys_addr ((void *) blk_qh)) & 0xFFFFFFF0) + 0x02;
  ctl_qh->qe_ptr = 0x01;

  /* Set this as the last Queue Head for now. T=1 */
  blk_qh->qh_ptr = 0x01;
  blk_qh->qe_ptr = 0x01;

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
      (((uint32_t) get_phys_addr ((void *) int_qh)) & 0xFFFFFFF0) + 0x02;
  }
}

static void
debug_dump_sched (UHCI_TD * tx_tds)
{
  // Dump the schedule for debugging
  uint32_t *dump;
  //uint32_t *dump1;

#if 1
  dump = (uint32_t *) tx_tds;
  DLOG ("setup packet dump: %p:%p:%p:%p",
        dump[0], dump[1], dump[2], dump[3]);
  //dump1 = (uint32_t *) dump[3];
  /* --??-- pagefault here? */
  //DLOG ("setup request: %p:%p", dump1[0], dump1[1]);

  while (dump[0] != 0x01) {
    dump = TD_P2V (uint32 *, dump[0] & 0xFFFFFFF0);
    DLOG ("Data/ACK packet dump: %p:%p:%p:%p",
          dump[0], dump[1], dump[2], dump[3]);
  }
#endif
}

static int
check_tds (UHCI_TD * tx_tds)
{
  int status_count = 1;
  int status = 0;

  while (status_count) {
    UHCI_TD *tds = tx_tds;
    status_count = 0;

    do {
      if (tds->status & 0x80) {
        status_count++;
      }

      /* If the TD is STALLED, we report the error */
      if (tds->status & 0x40) {
        status = tds->status & 0x7F;

        debug_dump_sched (tx_tds);

        return status;
      }

      tds = TD_P2V (UHCI_TD *, tds->link_ptr & 0xFFFFFFF0);
    } while (tds->link_ptr != 0x01);
  }

  delay (10);                   // --??-- Let's wait a little while here. It might be a source of timing bugs

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

static uint32 uhci_irq_handler (uint8 vec);

int
uhci_init (void)
{
  uint i, device_index, irq_pin, irq_line;
  pci_device uhci_device;

  if (mp_ISA_PC) {
    DLOG ("Cannot operate without PCI");
    return FALSE;
  }

  /* Find the UHCI device on the PCI bus */
  for (i=0; compatible_ids[i].vendor != 0xFFFF; i++)
    if (pci_find_device (compatible_ids[i].vendor, compatible_ids[i].device,
                         0xFF, 0xFF, 0, &device_index))
      break;
    else
      device_index = ~0;

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

  DLOG ("Using IRQ line=%X pin=%X", irq_line, irq_pin);

#define UHCI_VECTOR 0x50

  IOAPIC_map_GSI (IRQ_to_GSI (mp_ISA_bus_id, irq_line),
                  UHCI_VECTOR, 0xFF00000000000800LL);
  set_vector_handler (UHCI_VECTOR, uhci_irq_handler);


  td_phys = (uint32) get_phys_addr ((void *) td);
  DLOG ("td@%p td_phys@%p", td, td_phys);


  if (uhci_device.device != 0x7020)
    disable_ehci ();

#if 0
  /* Disable USB Legacy Support */
  DLOG ("UHCI LEGSUP: %p", GET_LEGACY (bus, dev, func));
  DISABLE_LEGACY (bus, dev, func);
  while (GET_LEGACY (bus, dev, func));
  DLOG ("UHCI LEGSUP disabled!");
  //SET_LEGACY(bus, dev, func, GET_LEGACY(bus, dev, func) | 0x2000);
#endif

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

  control_transfer_test ();
  show_usb_regs(bus, dev, func);

#endif

  return 0;
}

int
uhci_isochronous_transfer (uint8_t address,
                           uint8_t endpoint,
                           addr_t data,
                           int data_len, uint16_t frm, uint8_t direction)
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
  iso_td->ioc = 1;
  iso_td->iso = 1;
  iso_td->spd = 0;
  iso_td->pid = (direction == DIR_IN) ? UHCI_PID_IN : UHCI_PID_OUT;
  iso_td->addr = address;
  iso_td->endp = endpoint;
  iso_td->toggle = 0;
  iso_td->max_len = data_len - 1;
  iso_td->buf_ptr = (uint32_t) get_phys_addr ((void *) data);

  entry = frame_list[frm];

  if (!(entry & 0x01) && !(entry & 0x02)) {
    idx_td = TD_P2V (UHCI_TD *, entry & 0xFFFFFFF0);

    while (!(idx_td->link_ptr & 0x01) && !(idx_td->link_ptr & 0x02)) {
      idx_td = TD_P2V (UHCI_TD *, idx_td->link_ptr & 0xFFFFFFF0);
    }

    iso_td->link_ptr = idx_td->link_ptr;
    idx_td->link_ptr =
      (uint32_t) get_phys_addr ((void *) iso_td) & 0xFFFFFFF0;
  } else {
    iso_td->link_ptr =
      (((uint32_t) get_phys_addr ((void *) int_qh)) & 0xFFFFFFF0) + 0x02;
    frame_list[frm] = (uint32_t) get_phys_addr ((void *) iso_td) & 0xFFFFFFF0;
  }

#if 1
  delay (1000);
  delay (500);
  DLOG ("The status of iso_td: %p ActLen: %p", iso_td->status, iso_td->act_len);
#endif

  return 0;
}

int uhci_bulk_transfer(
    uint8_t address,
    uint8_t endpoint,
    addr_t data,
    int data_len,
    int packet_len,
    uint8_t direction)
{
  UHCI_TD *tx_tds = 0;
  UHCI_TD *data_td = 0;
  UHCI_TD *idx_td = 0;
  int max_packet_len = ((packet_len - 1) >= USB_MAX_LEN) ? USB_MAX_LEN : packet_len - 1;
  int i = 0, num_data_packets = 0, data_left = 0, return_status = 0;

  num_data_packets = (data_len + max_packet_len) / (max_packet_len + 1);
  data_left = data_len;

  for(i = 0; i < num_data_packets; i++) {
    data_td = sched_alloc(TYPE_TD);

    if(tx_tds == 0)
      tx_tds = data_td;
    else
      idx_td->link_ptr = (((uint32_t)get_phys_addr((void*)data_td)) & 0xFFFFFFF0) + 0x04;

    idx_td = data_td;
    data_td->status = 0x80;
    data_td->c_err = 3;
    data_td->ioc = data_td->iso = data_td->spd = 0;

    data_td->pid = (direction == DIR_IN) ? UHCI_PID_IN : UHCI_PID_OUT;
    data_td->addr = address;
    data_td->endp = endpoint;
    data_td->toggle = glb_toggle;
    glb_toggle = (glb_toggle == 1) ? 0 : 1;

    data_td->max_len = (data_left > (max_packet_len + 1)) ? max_packet_len : data_left - 1;
    data_td->buf_ptr = (uint32_t)get_phys_addr((void*)data);

    if(data_left <= (max_packet_len + 1)) {
      break;
    } else {
      data += (data_td->max_len + 1);
      data_left -= (data_td->max_len + 1);
    }
  }

  /* Initiate our bulk transactions */
  blk_qh->qe_ptr = (((uint32_t)get_phys_addr((void*)tx_tds)) & 0xFFFFFFF0) + 0x0;

  /* Check the status of all the packets in the transaction */
  return_status = check_tds(tx_tds);
  blk_qh->qe_ptr = 0x01;
  free_tds(tx_tds, num_data_packets);

  return return_status;
}

int
uhci_control_transfer (uint8_t address, addr_t setup_req,       /* Use virtual address here */
                       int setup_len, addr_t setup_data,        /* Use virtual address here */
                       int data_len)
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

  /*
   * USB specification max packet length :  1023 bytes
   * UHCI specification max packet length : 1280 bytes
   */
  int max_packet_len = USB_MAX_LEN;     // This is encoded as n-1
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
      (((uint32_t) get_phys_addr ((void *) data_td)) & 0xFFFFFFF0) + 0x04;
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
    (((uint32_t) get_phys_addr ((void *) status_td)) & 0xFFFFFFF0) + 0x04;

  status_td->link_ptr = 0x01;   // Terminate
  status_td->status = 0x80;
  status_td->c_err = 3;
  status_td->ioc = status_td->iso = status_td->spd = 0;
  status_td->pid = pid;
  status_td->addr = address;
  status_td->endp = endpoint;
  status_td->toggle = toggle;
  status_td->max_len = USB_NULL_PACKET;

  /* debug_dump_sched (tx_tds); */

  /* Initiate our control transaction */
  ctl_qh->qe_ptr =
    (((uint32_t) get_phys_addr ((void *) tx_tds)) & 0xFFFFFFF0) + 0x0;

  /* Check the status of all the packets in the transaction */
  return_status = check_tds (tx_tds);
  ctl_qh->qe_ptr = 0x01;
  free_tds (tx_tds, num_data_packets + 2);

  return return_status;
}

int
uhci_get_descriptor (uint8_t address, uint16_t dtype,   /* Descriptor type */
                     uint16_t dindex,   /* Descriptor index */
                     uint16_t index,    /* Zero or Language ID */
                     uint16_t length,   /* Descriptor length */
                     addr_t desc)
{
  USB_DEV_REQ setup_req;
  setup_req.bmRequestType = 0x80;       // Characteristics of request, see spec, P183, Rev 1.1
  setup_req.bRequest = USB_GET_DESCRIPTOR;
  setup_req.wValue = (dtype << 8) + dindex;
  setup_req.wIndex = index;
  setup_req.wLength = length;

  return uhci_control_transfer (address,
                                (addr_t) & setup_req, sizeof (USB_DEV_REQ),
                                desc, length);
}

int
uhci_set_address (uint8_t old_addr, uint8_t new_addr)
{
  USB_DEV_REQ setup_req;
  setup_req.bmRequestType = 0x0;
  setup_req.bRequest = USB_SET_ADDRESS;
  setup_req.wValue = new_addr;
  setup_req.wIndex = 0;
  setup_req.wLength = 0;

  return uhci_control_transfer (old_addr,
                                (addr_t) & setup_req, sizeof (USB_DEV_REQ), 0,
                                0);
}

int
uhci_get_configuration (uint8_t addr)
{
  USB_DEV_REQ setup_req;
  uint8_t num = -1;
  setup_req.bmRequestType = 0x80;
  setup_req.bRequest = USB_GET_CONFIGURATION;
  setup_req.wValue = 0;
  setup_req.wIndex = 0;
  setup_req.wLength = 1;

  uhci_control_transfer (addr, (addr_t) & setup_req, sizeof (USB_DEV_REQ),
                         (addr_t) & num, 1);

  return num;
}

int
uhci_set_configuration (uint8_t addr, uint8_t conf)
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
  glb_toggle = 0;


  return uhci_control_transfer (addr,
                                (addr_t) & setup_req, sizeof (USB_DEV_REQ), 0,
                                0);
}

int
uhci_set_interface (uint8_t addr, uint16_t alt, uint16_t interface)
{
  USB_DEV_REQ setup_req;
  setup_req.bmRequestType = 0x01;
  setup_req.bRequest = USB_SET_INTERFACE;
  setup_req.wValue = alt;
  setup_req.wIndex = interface;
  setup_req.wLength = 0;

  return uhci_control_transfer (addr,
                                (addr_t) & setup_req, sizeof (USB_DEV_REQ), 0,
                                0);
}

int
uhci_get_interface (uint8_t addr, uint16_t interface)
{
  USB_DEV_REQ setup_req;
  uint8_t alt = -1;
  setup_req.bmRequestType = 0x81;
  setup_req.bRequest = USB_GET_INTERFACE;
  setup_req.wValue = 0;
  setup_req.wIndex = interface;
  setup_req.wLength = 1;

  uhci_control_transfer (addr, (addr_t) & setup_req, sizeof (USB_DEV_REQ),
                         (addr_t) & alt, 1);

  return alt;
}

static uint32
uhci_irq_handler (uint8 vec)
{
  DLOG ("We caught an interrupt from usb IRQ_LN!");
  return 0;
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
