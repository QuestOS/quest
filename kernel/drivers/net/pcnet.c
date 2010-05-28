// AMD 79C90 PCnet32/LANCE NIC driver

#include "drivers/pci/pci.h"
#include "drivers/net/pcnet.h"
#include "arch/i386.h"
#include "util/printf.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "mem/physical.h"
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

/* CSR4: features */
/* 0x915: bits 0 (JABM), 2 (TXSTRTM), 4 (RCVCCOM), 8 (MFCOM), 11 (APAD_XMIT) set */
#define CSR4_FEATURES 0x915

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
  uint8 rbuf[RX_RING_SIZE][RX_RING_BUF_SIZE] ALIGNED(16);
  uint8 tbuf[TX_RING_BUF_SIZE] ALIGNED(16);
  sint32 rx_idx;
} PACKED;

/* ************************************************** */

/* List of compatible cards (ended by { 0xFFFF, 0xFFFF }) */
static struct { uint16 vendor, device; } compatible_ids[] = {
  { 0x1022, 0x2000 },
  { 0xFFFF, 0xFFFF }
};

/* ************************************************** */

static uint device_index, io_base, irq_line, irq_pin, version;

static struct pcnet_interface* card;
static uint card_phys, frame_count;

/* Virtual-to-Physical */
#define V2P(ty,p) ((ty)((((uint) (p)) - ((uint) card))+card_phys))
/* Physical-to-Virtual */
#define P2V(ty,p) ((ty)((((uint) (p)) - card_phys)+((uint) card)))

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

  /* set CSR4 (features) */
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

static void
reset (void)
{
  uint i, phys_init;

  (void) inw (RESET);           /* read is sufficient to reset */

  /* enable auto-select of media (BCR2 bit 1: ASEL) */
  outw (2, ADDR); (void) inw (ADDR);
  outw (inw (BUS) | 2, BUS);

  /* setup station hardware address */
  for (i=0; i<ETH_ADDR_LEN; i++)
    card->init_block.phys_addr[i] = eth_addr[i];

  /* preset rx ring headers */
  for (i=0; i<RX_RING_SIZE; i++) {
    card->rx_ring[i].rmd1.buf_length = -RX_RING_BUF_SIZE;
    card->rx_ring[i].rmd0.rbadr = V2P (uint32, card->rbuf[i]);
    card->rx_ring[i].rmd1.own = 1;
  }
  card->tx_ring.rmd1.buf_length = -TX_RING_BUF_SIZE;
  card->tx_ring.rmd0.tbadr = V2P (uint32, card->tbuf);

  card->rx_idx = 0;
  card->init_block.mode = 0;      /* enable Rx and Tx */
  card->init_block.filter[0] = card->init_block.filter[1] = 0;

  /* multiple Rx buffers and one Tx buffer */
  card->init_block.rx_ring = V2P (uint32, card->rx_ring);
  card->init_block.tx_ring = V2P (uint32, &card->tx_ring);
  card->init_block.rlen = NUM_RX_BUFFERS_LOG2;
  card->init_block.tlen = 0;

  phys_init = V2P (uint, &card->init_block);

  DLOG ("phys_init=%p rx_ring=%p tx_ring=%p rbuf[0]=%p tbuf=%p",
        phys_init, card->init_block.rx_ring, card->init_block.tx_ring,
        V2P (uint, card->rbuf[0]), V2P (uint, card->tbuf));

  outw (0, ADDR); (void) inw (ADDR);
  outw (4, DATA);               /* STOP */

  /* tell card where init block is found (physically) */

  /* CSR1=low 16-bits */
  outw (1, ADDR); (void) inw (ADDR);
  outw ((uint16) phys_init, DATA);

  /* CSR2=high 16-bits */
  outw (2, ADDR); (void) inw (ADDR);
  outw ((uint16) (phys_init >> 16), DATA);

  /* CSR4 (features) */
  outw (4, ADDR); (void) inw (ADDR);
  outw (CSR4_FEATURES, DATA);

  outw (0, ADDR); (void) inw (ADDR);
  outw (4, DATA);               /* INIT */

  /* check for IDON (init done) */
  for (i=10000; i > 0; i--)
    if (inw (DATA) & 0x100)
      break;
  if (i <= 0)
    DLOG ("reset: INIT timed out");

  outw (0x42, DATA);            /* START + INTERRUPT enable */
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
    goto abort;
  }

  DLOG ("Found device_index=%d sizeof (pcnet_interface)=%d",
        device_index, sizeof (struct pcnet_interface));

  if (!pci_decode_bar (device_index, 0, &mem_addr, &io_base, &mask)) {
    DLOG ("Invalid PCI configuration or BAR0 not found");
    goto abort;
  }

  if (io_base == 0) {
    DLOG ("Memory-mapped I/O not implemented");
    goto abort;
  }

  DLOG ("Using io_base=%.04X", io_base);

  if (!pci_get_interrupt (device_index, &irq_line, &irq_pin)) {
    DLOG ("Unable to get IRQ");
    goto abort;
  }

  DLOG ("Using IRQ line=%.02X pin=%X", irq_line, irq_pin);

  /* I need contiguous physical and virtual memory here */

  frame_count = sizeof (struct pcnet_interface) >> 12;
  if (sizeof (struct pcnet_interface) & 0xFFF)
    frame_count++;

  card_phys = alloc_phys_frames (frame_count);

  if (card_phys == -1) {
    DLOG ("Unable to allocate physical memory");
    goto abort;
  }

  card = (struct pcnet_interface *)
    map_contiguous_virtual_pages (card_phys | 3, frame_count);

  if (card == NULL) {
    DLOG ("Unable to allocate virtual memory");
    goto abort_phys;
  }

  DLOG ("DMA region at virt=%p phys=%p count=%d", card, card_phys, frame_count);

  if (!probe ()) {
    DLOG ("probe failed");
    goto abort_virt;
  }

  reset ();

  return TRUE;
 abort_virt:
  unmap_virtual_pages (card, frame_count);
 abort_phys:
  free_phys_frames (card_phys, frame_count);
 abort:
  return FALSE;
}

/* ************************************************** */
