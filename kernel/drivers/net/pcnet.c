// AMD 79C90 PCnet32/LANCE NIC driver

#include "drivers/pci/pci.h"
#include "drivers/net/pcnet.h"
#include "arch/i386.h"
#include "util/printf.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "mem/pow2.h"
#include "mem/virtual.h"
#include "kernel.h"

#define DEBUG_PCNET

#ifdef DEBUG_PCNET
#define DLOG(fmt,...) DLOG_PREFIX("PCnet",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

/* ************************************************** */

/* Basic constants */

#define NUM_RX_BUFFERS_LOG2 2 /* 4 Rx buffers */

#define RX_RING_SIZE (1 << NUM_RX_BUFFERS_LOG2)
#define RX_RING_MOD_MASK (RX_RING_SIZE - 1)
#define RX_RING_LEN_BITS ((NUM_RX_BUFFERS_LOG2) << 29)
#define RX_RING_BUF_SIZE MAX_FRAME_SIZE   /* multiple of 16 */
#define TX_RING_BUF_SIZE MAX_FRAME_SIZE   /* multiple of 16 */

/* ************************************************** */

/* PCnet data structures shared by hardware */

/* using data structure mode ssize=1 (see datasheet) */

struct pcnet_init_block {
  /* hardware-specific layout */
  uint16 mode;
  uint8  _reserved1:4;
  uint8  rlen:4;
  uint8  _reserved2:4;
  uint8  tlen:4;
  uint8  phys_addr[ETH_ADDR_LEN];
  uint16 _reserved3;
  uint32 filter[2];
  uint32 rx_ring;
  uint32 tx_ring;
} PACKED;

struct pcnet_rx_head {
  /* hardware-specific layout */
  union {
    uint32 raw;
    uint32 rbadr;          /* recv buf addr */
  } rmd0;
  union {
    uint32 raw;
    struct {
      sint16 buf_length; /* 2s complement */
      uint16 _reserved:4;
      uint16 bam:1;        /* Broadcast Address Match */
      uint16 lafm:1;       /* Logical Address Filter Match */
      uint16 pam:1;        /* Physical Address Match */
      uint16 bpe:1;        /* Bus Parity Error */
      uint16 enp:1;        /* End of Packet */
      uint16 stp:1;        /* Start of Packet */
      uint16 buff:1;       /* Buffer error (does not own next buffer)  */
      uint16 crc:1;        /* CRC error */
      uint16 oflo:1;       /* Overflow error */
      uint16 fram:1;       /* Framing error */
      uint16 err:1;        /* Any error set */
      uint16 own:1;        /* OWN bit (1=hw, 0=driver) */
    };
  } rmd1;
  union {
    uint32 raw;
    struct {
      uint16 msg_length;
      uint8 rpc;
      uint8 rcc;
    };
  } rmd2;
  uint32 _reserved;
} PACKED;

struct pcnet_tx_head {
  /* hardware-specific layout */
  union {
    uint32 raw;
    uint32 tbadr;          /* trans buf addr */
  } rmd0;
  union {
    uint32 raw;
    struct {
      sint16 buf_length; /* 2s complement */
      uint16 _reserved:7;
      uint16 bpe:1;
      uint16 enp:1;
      uint16 stp:1;
      /* bpe, enp, stp, def, one, more, add, err */
      uint16 flags:5;
      uint16 own:1;         /* OWN bit */
    };
  } rmd1;
  union {
    uint32 raw;            /* misc flags */
  } rmd2;
  uint32 _reserved;
} PACKED;

struct pcnet_interface {
  struct pcnet_rx_head rx_ring[RX_RING_SIZE];
  struct pcnet_tx_head tx_ring;
  struct pcnet_init_block init_block;
  uint32 padding;
  uint8 rbuf[RX_RING_SIZE][RX_RING_BUF_SIZE] ALIGNED(16);
  uint8 tbuf[TX_RING_BUF_SIZE] ALIGNED(16);
  sint32 rx_idx;
};

/* ************************************************** */

/* List of compatible cards (ended by { 0xFFFF, 0xFFFF }) */
static struct { uint16 vendor, device; } compatible_ids[] = {
  { 0x1022, 0x2000 },
  { 0xFFFF, 0xFFFF }
};

/* ************************************************** */

static uint device_index, io_base, irq_line, irq_pin, version;

static struct pcnet_interface* card;
static uint card_phys;

static uint8 eth_addr[ETH_ADDR_LEN];

/* ************************************************** */

/* IO ports */

#define ETH   (io_base+0x00)
#define DATA  (io_base+0x10)
#define ADDR  (io_base+0x12)
#define RESET (io_base+0x14)
#define BUS   (io_base+0x16)

/* ************************************************** */

static bool
probe (void)
{
  uint i;

  /* assume io_base set */
  outw (inw (RESET), RESET);

  /* expect set CSR0 bit 4 (STOPped) */
  outw (0, ADDR); (void) inw (ADDR);
  if (inw (DATA) != 4) {
    DLOG ("expected STOP bit set");
    return FALSE;
  }

  /* check BCR20 for 32-bit support */
  outw (20, ADDR); (void) inw (ADDR);
  outw (inw (BUS) | 0x102, BUS); /* set our style */
  outw (20, ADDR); (void) inw (ADDR);
  if ((inw (BUS) & 0x1FF) != 0x102) {
    DLOG ("SSIZE!=1 and SWSTYLE!=2 (BCR20=%p)", inw (BUS));
    return FALSE;
  }

  /* get version (CSR88, CSR89) */
  outw (88, ADDR);
  if (inw (ADDR) != 88) {
    DLOG ("Unsupported (ancient) chip");
    return FALSE;
  } else {
    version = inw (DATA);
    outw (89, ADDR); (void) inw (ADDR);
    version |= inw (DATA) << 16;

    if ((version & 0xfff) != 3) {
      DLOG ("malformed version %p", version);
      return FALSE;
    }

    version >>= 12;
    if (version != 0x2621) {
      DLOG ("version=%p is not supported", version);
      return FALSE;
    }
  }

  /* CSR4: features */
  /* 0x915: bits 0 (JABM), 2 (TXSTRTM), 4 (RCVCCOM), 8 (MFCOM), 11 (APAD_XMIT) set */
#define CSR4_FEATURES 0x915
  outw (4, ADDR); (void) inw (ADDR);
  outw (CSR4_FEATURES, DATA);

  /* CSR0 */
  outw (0, ADDR); (void) inw (ADDR);

  /* Get ethernet hardware address */
  for (i=0; i< ETH_ADDR_LEN; i++)
    eth_addr[i] = inb (ETH+i);

  /* enable auto-select of media (BCR2 bit 1: ASEL) */
  outw (2, ADDR); (void) inw (ADDR);
  outw (inw (BUS) | 2, BUS);

  DLOG ("version=%p eth_addr=%.02x:%.02x:%.02x:%.02x:%.02x:%.02x",
        version,
        eth_addr[0], eth_addr[1], eth_addr[2],
        eth_addr[3], eth_addr[4], eth_addr[5]);

  return TRUE;
}

bool
pcnet_init (void)
{
  uint i, mem_addr, mask;

  for (i=0; compatible_ids[i].vendor != 0xFFFF; i++)
    if (pci_find_device (compatible_ids[i].vendor, compatible_ids[i].device,
                         0xFF, 0xFF, 0, &device_index))
      break;
    else
      device_index = ~0;

  if (device_index == (uint)(~0)) {
    DLOG ("Unable to detect compatible device.");
    goto give_up;
  }

  DLOG ("Found device_index=%d sizeof (pcnet_interface)=%d",
        device_index, sizeof (struct pcnet_interface));

  if (!pci_decode_bar (device_index, 0, &mem_addr, &io_base, &mask)) {
    DLOG ("Invalid PCI configuration or BAR0 not found");
    goto give_up;
  }

  if (io_base == 0) {
    DLOG ("Memory-mapped I/O not implemented");
    goto give_up;
  }

  DLOG ("Using io_base=%.04X", io_base);

  if (!pci_get_interrupt (device_index, &irq_line, &irq_pin)) {
    DLOG ("Unable to get IRQ");
    goto give_up;
  }

  DLOG ("Using IRQ line=%.02X pin=%X", irq_line, irq_pin);

  if (pow2_alloc (sizeof (struct pcnet_interface), (uint8 **)&card) == 0) {
    DLOG ("Unable to allocate memory");
    goto give_up;
  }

  card_phys = (uint) get_phys_addr (card);
  DLOG ("DMA region at virt=%p phys=%p", card, card_phys);

  if (!probe ()) {
    pow2_free ((uint8 *) card);
    goto give_up;
  }

  return TRUE;
 give_up:
  return FALSE;
}

/* ************************************************** */
