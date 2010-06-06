/* Intel e1000e NIC driver */

#include "drivers/pci/pci.h"
#include "drivers/net/ethernet.h"
#include "arch/i386.h"
#include "util/printf.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "mem/physical.h"
#include "mem/virtual.h"
#include "kernel.h"

#define DEBUG_E1000E

#ifdef DEBUG_E1000E
//#define DLOG(fmt,...) DLOG_PREFIX("e1000e",fmt,##__VA_ARGS__)
#define DLOG(fmt,...) printf("e1000e: "fmt"\n",##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define E1000E_VECTOR 0x4D       /* arbitrary */

#define RDESC_COUNT 8           /* must be multiple of 8 */
#define RDESC_COUNT_MOD_MASK (RDESC_COUNT - 1)
#define RBUF_SIZE   2048        /* configured in RCTL.BSIZE */
#define RBUF_SIZE_MASK 0        /* 0 = 2048 bytes */

#define TDESC_COUNT 8           /* must be multiple of 8 */
#define TDESC_COUNT_MOD_MASK (RDESC_COUNT - 1)
#define TBUF_SIZE   2048        /* configured in TCTL.BSIZE */
#define TBUF_SIZE_MASK 0        /* 0 = 2048 bytes */
#define TCTL_CT_MASK   0x100
#define TCTL_COLD_MASK 0x40000
#define TIPG_MASK (10 | (10 << 10) | (10 << 20))

/* List of compatible cards (ended by { 0xFFFF, 0xFFFF }) */
static struct { uint16 vendor, device; } compatible_ids[] = {
  { 0x8086, 0x10F5 },
  { 0xFFFF, 0xFFFF }
};

static uint8 hwaddr[ETH_ADDR_LEN];
static uint device_index, mem_addr, flash_addr, irq_line, irq_pin, e1000e_phys;
static volatile uint32 *mmio_base, *flash_base;
#define E1000E_MMIO_PAGES 0x10

/* ************************************************** */

/* registers */

#define REG(x) (mmio_base[x])

#define CTRL   (REG (0x00))     /* Control */
#define CTRL_RST (1<<26)        /* Reset */
#define STATUS (REG (0x02))     /* Status */
#define EERD   (REG (0x05))     /* EEPROM Read */
#define ICR    (REG (0x30))     /* Interrupt Cause Read */
#define ICR_RXT (0x80)          /* RX Timer Int. */
#define ICR_RXO (0x40)          /* RX Overrun Int. */
#define ICR_TXQE (0x02)         /* TX Queue Empty Int. */
#define IMS    (REG (0x34))     /* Interrupt Mask Set */
#define IMS_RXT (0x80)          /* RX Timer Int. */
#define IMS_RXO (0x40)          /* RX Overrun Int. */
#define IMS_TXQE (0x02)         /* TX Queue Empty Int. */
#define IMC    (REG (0x36))     /* Interrupt Mask Clear */
#define RCTL   (REG (0x40))     /* Receive Control */
#define RCTL_EN (0x02)          /* RX Enable */
#define RCTL_BAM (1<<15)        /* Accept Broadcast packets */
#define RCTL_BSIZE (0x30000)    /* Buffer size */
#define RCTL_BSEX (1<<25)       /* "Extension" (x16) of size */
#define TCTL   (REG (0x100))    /* Transmit Control */
#define TCTL_EN (0x02)          /* TX Enable */
#define TCTL_PSP (0x08)         /* TX Pad Short Packets */
#define TCTL_CT (0xFF0)         /* TX Collision Threshold */
#define TCTL_COLD (0x3FF000)    /* TX Collision Distance */
#define TIPG   (REG (0x104))    /* TX Inter Packet Gap */
#define RDBAL  (REG (0xA00))    /* RX Desc. Base Address Low */
#define RDBAH  (REG (0xA01))    /* RX Desc. Base Address High */
#define RDLEN  (REG (0xA02))    /* RX Desc. Length */
#define RDH    (REG (0xA04))    /* RX Desc Head */
#define RDT    (REG (0xA06))    /* RX Desc Tail */
#define TDBAL  (REG (0xE00))    /* TX Desc. Base Address Low */
#define TDBAH  (REG (0xE01))    /* TX Desc. Base Address High */
#define TDLEN  (REG (0xE02))    /* TX Desc. Length */
#define TDH    (REG (0xE04))    /* TX Desc Head */
#define TDT    (REG (0xE06))    /* TX Desc Tail */
#define RAL    (REG (0x1500))   /* RX HW Address Low */
#define RAH    (REG (0x1501))   /* RX HW Address High */

/* ************************************************** */

/* Receive descriptor (16-bytes) describes a buffer in memory */
struct e1000e_rdesc {
  uint64 address;
  uint16 length;
  uint16 checksum;
  uint8  status;
  uint8  errors;
  uint16 special;
} PACKED;
#define RDESC_STATUS_DD  0x01    /* indicates hardware done with descriptor */
#define RDESC_STATUS_EOP 0x02    /* indicates end of packet */

/* ************************************************** */

/* Transmit descriptor (16-bytes) describes a buffer in memory */
struct e1000e_tdesc {
  uint64 address;
  uint16 length;
  uint8  cso;                   /* checksum offset */
  uint8  cmd;                   /* command */
  uint8  sta:4;                 /* status */
  uint8  rsv:4;                 /* reserved */
  uint8  css;                   /* checksum start */
  uint16 special;
} PACKED;
#define TDESC_STA_DD  0x01 /* indicates hardware done with descriptor */
#define TDESC_CMD_EOP 0x01 /* indicates end of packet */
#define TDESC_CMD_RS 0x08  /* requests status report */

/* ************************************************** */

static struct e1000e_interface {
  struct e1000e_rdesc rdescs[RDESC_COUNT] ALIGNED(0x10);
  struct e1000e_tdesc tdescs[TDESC_COUNT] ALIGNED(0x10);
  uint8 rbufs[RDESC_COUNT][RBUF_SIZE];
  uint8 tbufs[TDESC_COUNT][TBUF_SIZE];
  uint  rx_idx;                 /* current RX descriptor */
  uint  tx_cnt;                 /* number of pending TX descriptors */
} *e1000e;

/* Virtual-to-Physical */
#define V2P(ty,p) ((ty)((((uint) (p)) - ((uint) e1000e))+e1000e_phys))
/* Physical-to-Virtual */
#define P2V(ty,p) ((ty)((((uint) (p)) - e1000e_phys)+((uint) e1000e)))

static ethernet_device e1000e_ethdev;

/* ************************************************** */

extern bool
e1000e_get_hwaddr (uint8 a[ETH_ADDR_LEN])
{
  int i;
  for (i=0; i<ETH_ADDR_LEN; i++)
    a[i] = hwaddr[i];
  return TRUE;
}

extern sint
e1000e_transmit (uint8* buffer, sint len)
{
  uint32 tdt = TDT;
  DLOG ("TX: (%p, %d) TDH=%d TDT=%d", buffer, len, TDH, tdt);

  if (len > TBUF_SIZE)
    return 0;

  if (e1000e->tx_cnt >= TDESC_COUNT - 1) /* overrun */
    return 0;

  /* set up the first available descriptor */
  memcpy (e1000e->tbufs[tdt], buffer, len);
  e1000e->tdescs[tdt].length = len;
  e1000e->tdescs[tdt].cmd = TDESC_CMD_RS | TDESC_CMD_EOP;

  /* advance the TDT, notifying hardware */
  tdt++;
  if (tdt >= TDESC_COUNT)
    tdt = 0;
  TDT = tdt;

  e1000e->tx_cnt++;

  return len;
}

extern void
e1000e_rx_poll (void)
{
  uint32 entry, rdt;
  uint8 *ptr;

  DLOG ("RX: %d %d %d %d %d %d %d %d",
        e1000e->rdescs[0].status & RDESC_STATUS_DD,
        e1000e->rdescs[1].status & RDESC_STATUS_DD,
        e1000e->rdescs[2].status & RDESC_STATUS_DD,
        e1000e->rdescs[3].status & RDESC_STATUS_DD,
        e1000e->rdescs[4].status & RDESC_STATUS_DD,
        e1000e->rdescs[5].status & RDESC_STATUS_DD,
        e1000e->rdescs[6].status & RDESC_STATUS_DD,
        e1000e->rdescs[7].status & RDESC_STATUS_DD);

  entry = e1000e->rx_idx & RDESC_COUNT_MOD_MASK;
  while (e1000e->rdescs[entry].status & RDESC_STATUS_DD) {
    if (e1000e->rdescs[entry].status & RDESC_STATUS_EOP) {
      uint16 len;
      /* full packet */
      ptr = e1000e->rbufs[entry];
      len = e1000e->rdescs[entry].length;
      DLOG ("RX: full packet@%p len=%d", ptr, len);
      if (e1000e_ethdev.recv_func)
        e1000e_ethdev.recv_func (&e1000e_ethdev, ptr, len);
      else                      /* drop it */
        DLOG ("recv_func is null");
    } else {
      /* error */
      DLOG ("RX: error. status=%p", e1000e->rdescs[entry].status);
    }

    /* clear status */
    e1000e->rdescs[entry].status = 0;

    /* advance "tail" to notify hardware */
    rdt = RDT;
    rdt++;
    if (rdt >= RDESC_COUNT)
      rdt = 0;
    RDT = rdt;

    /* check next entry */
    entry = (++e1000e->rx_idx) & RDESC_COUNT_MOD_MASK;
  }
}

static void
handle_tx (uint32 icr)
{
  uint i;
  DLOG ("TX: tx_cnt=%d", e1000e->tx_cnt);

  /* find descriptors that have completed */
  for (i=0; i<TDESC_COUNT; i++) {
    if (e1000e->tdescs[i].cmd && (e1000e->tdescs[i].sta & TDESC_STA_DD)) {
      e1000e->tdescs[i].cmd = 0;
      e1000e->tdescs[i].sta = 0;
      e1000e->tx_cnt--;
    }
  }
}

extern void
e1000e_poll (void)
{
  e1000e_rx_poll ();
  handle_tx (ICR_TXQE);
}

static uint32
e1000e_irq_handler (uint8 vec)
{
  /* ICR is cleared upon read; this implicitly acknowledges the
   * interrupt. */
  uint32 icr = ICR;
  DLOG ("IRQ: ICR=%p", icr);

  if (icr & ICR_RXT)            /* RX */
    e1000e_rx_poll ();
  if (icr & ICR_TXQE)           /* TX queue empty */
    handle_tx (icr);

  return 0;
}

static void
reset (void)
{
  uint i;

  /* disable PCIe mastering */
  //DLOG ("Master Disable CTRL=%p", CTRL);
  CTRL |= 0x4;

  //DLOG ("Masking interrupts");
  IMC = (~0);

  //DLOG ("Disable TX and RX");
  RCTL = 0;
  TCTL = TCTL_PSP;

  //DLOG ("STATUS=%p RCTL=%p TCTL=%p", STATUS, RCTL, TCTL);

  STATUS &= ~(0x600);

  //DLOG ("cleared PHYRST and INIT_DONE; STATUS=%p", STATUS);

  /* get sw control */
  //DLOG ("(before) EXTCNF_CTRL=%p", REG(0x3C0));
  REG(0x3C0) |= 0x20;
  //DLOG ("(after) EXTCNF_CTRL=%p", REG(0x3C0));

  /* reset */

  //DLOG ("reseting CTRL=%p", CTRL);
  //CTRL |= (0x80000000 | CTRL_RST);
  //  while (CTRL & CTRL_RST) tsc_delay_usec (1);
  DLOG ("CTRL=%p", CTRL);

  /* set hardware address */
  RAL =
    (hwaddr[0] << 0x00) |
    (hwaddr[1] << 0x08) |
    (hwaddr[2] << 0x10) |
    (hwaddr[3] << 0x18);
  RAH = 0x80000000L     |       /* Address Valid */
    (hwaddr[4] << 0x00) |
    (hwaddr[5] << 0x08);

  DLOG ("RAL=%p RAH=%p", RAL, RAH);

  /* set rx buffer size code */
  RCTL &= ~RCTL_BSIZE;
  RCTL |= RBUF_SIZE_MASK;
  RCTL &= ~RCTL_BSEX;

  /* set up rdesc addresses */
  for (i=0; i<RDESC_COUNT; i++) {
    e1000e->rdescs[i].address = V2P (uint64, e1000e->rbufs[i]);
    e1000e->rdescs[i].status = 0;
  }

  /* program the rdesc base address and length */
  RDBAL = V2P (uint32, e1000e->rdescs);
  RDBAH = 0;
  RDLEN = RDESC_COUNT * sizeof (struct e1000e_rdesc);

  /* set head, tail of rx ring buffer */
  RDH = 0;
  RDT = RDESC_COUNT - 1;

  e1000e->rx_idx = 0;

  DLOG ("RDBAL=%p RDLEN=%p RDH=%d RDT=%d",
        V2P (uint32, e1000e->rdescs), RDESC_COUNT * sizeof (struct e1000e_rdesc),
        RDH, RDT);

  /* set up tdesc addresses */
  for (i=0; i<TDESC_COUNT; i++) {
    e1000e->tdescs[i].address = V2P (uint64, e1000e->tbufs[i]);
    e1000e->tdescs[i].sta = 0;
  }

  /* program the tdesc base address and length */
  TDBAL = V2P (uint32, e1000e->tdescs);
  TDBAH = 0;
  TDLEN = TDESC_COUNT * sizeof (struct e1000e_tdesc);

  /* set head, tail of rx ring buffer */
  TDH = 0;
  TDT = 0;

  e1000e->tx_cnt = 0;

  DLOG ("TDBAL=%p TDLEN=%p TDH=%d TDT=%d",
        V2P (uint32, e1000e->tdescs), TDESC_COUNT * sizeof (struct e1000e_tdesc),
        TDH, TDT);

  /* setup RX interrupts */
  IMS |= IMS_RXT;

  /* setup TX interrupts */
  IMS |= IMS_TXQE;

  /* enable RX operation and broadcast reception */
  RCTL |= (RCTL_EN | RCTL_BAM);

  /* enable TX operation */
  TCTL |= (TCTL_EN | TCTL_PSP | TCTL_COLD_MASK);
  TIPG = TIPG_MASK;


  /* re-enable mastering */

  CTRL &= (~0x4);

  /* relinquish SWFLAG */
  REG(0x3C0) &= ~(0x20);

  DLOG ("CTRL=%p STA=%p RCTL=%p TCTL=%p EXT=%p", CTRL, STATUS, RCTL, TCTL, REG(0x3C0));

  //while (! (TCTL & TCTL_EN)) asm volatile ("pause");
}

extern bool
e1000e_init (void)
{
  uint i, io_base, mask, frame_count;

  if (mp_ISA_PC) {
    DLOG ("Requires PCI support");
    goto abort;
  }

  for (i=0; compatible_ids[i].vendor != 0xFFFF; i++)
    if (pci_find_device (compatible_ids[i].vendor, compatible_ids[i].device,
                         0xFF, 0xFF, 0, &device_index))
      break;
    else
      device_index = ~0;

  if (device_index == (uint)(~0)) {
    DLOG ("Unable to detect compatible device.");
    goto abort;
  }

  DLOG ("Found device_index=%d", device_index);

  if (!pci_decode_bar (device_index, 0, &mem_addr, &io_base, &mask)) {
    DLOG ("Invalid PCI configuration or BAR0 not found");
    goto abort;
  }

  if (mem_addr == 0) {
    DLOG ("Unable to detect memory mapped IO region");
    goto abort;
  }

  /* Map some virtual pages to the memory-mapped I/O region */
  mmio_base = map_contiguous_virtual_pages (mem_addr | 3, E1000E_MMIO_PAGES);

  if (mmio_base == NULL) {
    DLOG ("Unable to map page to phys=%p", mem_addr);
    goto abort;
  }

  DLOG ("Using memory mapped IO at phys=%p virt=%p", mem_addr, mmio_base);


  /* 8257x */
  if (!pci_decode_bar (device_index, 1, &flash_addr, &io_base, &mask)) {
    DLOG ("Invalid PCI configuration or BAR1 not found");
    goto abort;
  }

  if (flash_addr == 0) {
    DLOG ("Unable to detect memory mapped flash region");
    goto no_flash;
  }

  /* Map some virtual pages to the memory-mapped flash region */
  flash_base = map_contiguous_virtual_pages (flash_addr | 3, 1);

  if (flash_base == NULL) {
    DLOG ("Unable to map page to phys=%p", flash_addr);
    goto abort;
  }

  DLOG ("Using memory mapped FLASH at phys=%p virt=%p", flash_addr, flash_base);
  DLOG ("flash=%p %p %p %p",
        flash_base[0], flash_base[1], flash_base[2], flash_base[3]);
 no_flash:
  /*  */



  /* I need contiguous physical and virtual memory for DMA memory */
  frame_count = sizeof (struct e1000e_interface) >> 12;
  if (sizeof (struct e1000e_interface) & 0xFFF)
    frame_count++;              /* round up */

  /* Obtain contiguous physical frames. */
  e1000e_phys = alloc_phys_frames (frame_count);

  if (e1000e_phys == -1) {
    DLOG ("Unable to allocate physical memory");
    goto abort_mmap;
  }

  /* Map contiguous virtual pages to contiguous physical frames. */
  e1000e = (struct e1000e_interface *)
    map_contiguous_virtual_pages (e1000e_phys | 3, frame_count);

  if (e1000e == NULL) {
    DLOG ("Unable to allocate virtual memory");
    goto abort_phys;
  }

  /* zero the acquired memory */
  memset (e1000e, 0, sizeof (struct e1000e_interface));

  DLOG ("DMA region at virt=%p phys=%p count=%d", e1000e, e1000e_phys, frame_count);

  if (!pci_get_interrupt (device_index, &irq_line, &irq_pin)) {
    DLOG ("Unable to get IRQ");
    goto abort_virt;
  }

  DLOG ("Using IRQ line=%.02X pin=%X", irq_line, irq_pin);

  /* Map IRQ to handler */
  IOAPIC_map_GSI (IRQ_to_GSI (mp_ISA_bus_id, irq_line),
                  E1000E_VECTOR, 0xFF00000000000800LL);
  set_vector_handler (E1000E_VECTOR, e1000e_irq_handler);

#if 0
  /* read hardware individual address from EEPROM */
  EERD = 0x0001;                /* EERD.START=0 EERD.ADDR=0 */
  while (!(EERD & 0x10)) asm volatile ("pause"); /* wait for EERD.DONE */
  hwaddr[0] = (EERD >> 16) & 0xFF;
  hwaddr[1] = (EERD >> 24) & 0xFF;
  EERD = 0x0101;                /* EERD.START=0 EERD.ADDR=1 */
  while (!(EERD & 0x10)) asm volatile ("pause"); /* wait for EERD.DONE */
  hwaddr[2] = (EERD >> 16) & 0xFF;
  hwaddr[3] = (EERD >> 24) & 0xFF;
  EERD = 0x0201;                /* EERD.START=0 EERD.ADDR=2 */
  while (!(EERD & 0x10)) asm volatile ("pause"); /* wait for EERD.DONE */
  hwaddr[4] = (EERD >> 16) & 0xFF;
  hwaddr[5] = (EERD >> 24) & 0xFF;
#endif
  for (i=0;i<6;i++) hwaddr[i] = i;

  DLOG ("hwaddr=%.02x:%.02x:%.02x:%.02x:%.02x:%.02x",
        hwaddr[0], hwaddr[1], hwaddr[2], hwaddr[3], hwaddr[4], hwaddr[5]);

  reset ();

  /* Register network device with net subsystem */
  e1000e_ethdev.recv_func = NULL;
  e1000e_ethdev.send_func = e1000e_transmit;
  e1000e_ethdev.get_hwaddr_func = e1000e_get_hwaddr;
  e1000e_ethdev.poll_func = e1000e_poll;

  if (!net_register_device (&e1000e_ethdev)) {
    DLOG ("registration failed");
    goto abort_virt;
  }

  return TRUE;

 abort_virt:
  unmap_virtual_pages (e1000e, frame_count);
 abort_phys:
  free_phys_frames (e1000e_phys, frame_count);
 abort_mmap:
  unmap_virtual_pages ((void *)mmio_base, E1000E_MMIO_PAGES);
 abort:
  return FALSE;
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
