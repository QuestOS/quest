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

/* AMD 79C90 PCnet32/LANCE NIC driver */

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

#define PCNET_VECTOR 0x4B

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
  outw (1, DATA);               /* INIT */

  /* check for IDON (init done) */
  for (i=10000; i > 0; i--)
    if (inw (DATA) & 0x100)
      break;
  if (i <= 0)
    DLOG ("reset: INIT timed out");

  outw (0x142, DATA);            /* assert START + clear IDON + set IENA
                                  * (interrupt enable) */

  DLOG ("reset: complete.  CSR0=%p", inw (DATA));
}

extern sint
pcnet_transmit (uint8* buf, sint len)
{
  DLOG ("pcnet_transmit (%p, %d)", buf, len);
  if (card->tx_ring.rmd1.own != 0) {
    /* we don't control the transmit buffer at the moment */
    return 0;
  } else if (len > MAX_FRAME_SIZE) {
    /* too big */
    return -1;
  } else {
    /* copy into transmission buffer */
    memcpy (card->tbuf, buf, len);
    /* set tbuf length */
    card->tx_ring.rmd1.buf_length = -len;
    /* clear misc. info */
    card->tx_ring.rmd2.raw = 0;
    /* this is a single packet, start and end */
    card->tx_ring.rmd1.stp = card->tx_ring.rmd1.enp = 1;
    /* pass ownership to NIC */
    card->tx_ring.rmd1.own = 1;
    /* trigger a send poll */
    outw (0, ADDR); (void) inw (ADDR);
    outw (0x48, DATA);
  }
  return len;
}

static void
pcnet_drop_packet (uint8* packet, uint len)
{
  DLOG ("dropping packet (%p, %d)", packet, len);
}

static ethernet_device pcnet_ethdev;

static void
pcnet_poll (void)
{
  uint32 entry;
  uint32* ptr;

  entry = card->rx_idx & RX_RING_MOD_MASK;

  while (card->rx_ring[entry].rmd1.own == 0) {
    if (card->rx_ring[entry].rmd1.enp == 1 &&
        card->rx_ring[entry].rmd1.stp == 1) {
      /* full packet awaits */
      ptr = P2V (uint32*, card->rx_ring[entry].rmd0.rbadr);

      DLOG ("  recv entry=%d rmd1=%p msglen=%x",
            entry, card->rx_ring[entry].rmd1.raw,
            card->rx_ring[entry].rmd2.msg_length);
      DLOG ("    %.08X %.08X %.08X %.08X",
            ptr[0], ptr[1], ptr[2], ptr[3]);

      /* do something with packet */
      if (pcnet_ethdev.recv_func)
        pcnet_ethdev.recv_func (&pcnet_ethdev, (uint8*)ptr, 
                                card->rx_ring[entry].rmd2.msg_length);
      else
        pcnet_drop_packet ((uint8*)ptr, card->rx_ring[entry].rmd2.msg_length);
    } else {
      /* not a full packet -- error */
      DLOG ("  recv error not full packet entry=%d rmd1=%p",
            entry, card->rx_ring[entry].rmd1.raw);
    }

    /* clear errors */
    card->rx_ring[entry].rmd1.raw &= 0x030fffff;
    /* clear msg_length */
    card->rx_ring[entry].rmd2.raw = 0;
    /* set length */
    card->rx_ring[entry].rmd1.buf_length = -RX_RING_BUF_SIZE;
    /* set OWN */
    card->rx_ring[entry].rmd1.own = 1;

    /* check next entry */
    entry = (++card->rx_idx) & RX_RING_MOD_MASK;
  }
}

static void
handle_rint (void)
{
  DLOG ("IRQ: RINT rx_ring=%.01x %p %.01x %p %.01x %p %.01x %p",
        card->rx_ring[0].rmd1.own, card->rx_ring[0].rmd0.rbadr,
        card->rx_ring[1].rmd1.own, card->rx_ring[1].rmd0.rbadr,
        card->rx_ring[2].rmd1.own, card->rx_ring[2].rmd0.rbadr,
        card->rx_ring[3].rmd1.own, card->rx_ring[3].rmd0.rbadr);

  pcnet_poll ();
}

static void
handle_tint (void)
{
  DLOG ("IRQ: TINT tx_ring=%.01x %p",
        card->tx_ring.rmd1.own, card->tx_ring.rmd0.tbadr);
}

static uint32
pcnet_irq_handler (uint8 vec)
{
  uint16 csr0;
  lock_kernel ();

  outw (0, ADDR); (void) inw (ADDR);
  csr0 = inw (DATA);

  DLOG ("IRQ: vec=%.02X csr0=%.04X", vec, csr0);

  /* acknowledge interrupt sources */
  outw (csr0 & ~0x004f, DATA);

  if (csr0 & 0x8000) {          /* ERR */
    if (csr0 & 0x1000)          /* MISS */
      DLOG ("IRQ: missed packet");
    if (csr0 & 0x2000)          /* CERR */
      DLOG ("IRQ: collision detected");
  }

  if (csr0 & 0x400)             /* RINT */
    handle_rint ();
  if (csr0 & 0x200)             /* TINT */
    handle_tint ();

  /* ack anything further and set IENA */
  outw (0, ADDR); (void) inw (ADDR);
  outw (0x7940, DATA);

  DLOG ("IRQ: finished. csr0=%.04X", inw (DATA));

  unlock_kernel ();
  return 0;
}

extern bool
pcnet_get_hwaddr (uint8 addr[ETH_ADDR_LEN])
{
  uint i;
  for (i=0;i<ETH_ADDR_LEN;i++)
    addr[i] = eth_addr[i];
  return TRUE;
}

extern bool
pcnet_init (void)
{
  uint i;

  if (mp_ISA_PC) {
    DLOG ("Cannot operate without PCI support");
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

  DLOG ("Found device_index=%d sizeof (pcnet_interface)=%d",
        device_index, sizeof (struct pcnet_interface));

  if (!pci_decode_bar (device_index, 0, NULL, &io_base, NULL)) {
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
    frame_count++;              /* round up */

  /* Obtain contiguous physical frames. */
  card_phys = alloc_phys_frames (frame_count);

  if (card_phys == -1) {
    DLOG ("Unable to allocate physical memory");
    goto abort;
  }

  /* Map contiguous virtual pages to contiguous physical frames. */
  card = (struct pcnet_interface *)
    map_contiguous_virtual_pages (card_phys | 3, frame_count);

  if (card == NULL) {
    DLOG ("Unable to allocate virtual memory");
    goto abort_phys;
  }

  /* zero the acquired memory */
  memset (card, 0, sizeof (struct pcnet_interface));

  DLOG ("DMA region at virt=%p phys=%p count=%d", card, card_phys, frame_count);

  if (!probe ()) {
    DLOG ("probe failed");
    goto abort_virt;
  }

  /* Map IRQ to handler */
  pci_irq_t irq;
  pci_device pdev;
  if (!pci_get_device (device_index, &pdev)) {
    DLOG ("pci_get_device");
    goto abort_virt;
  }

  if (pci_irq_find (pdev.bus, pdev.slot, irq_pin, &irq)) {
    /* use PCI routing table */
    DLOG ("Found PCI routing entry irq.gsi=0x%x", irq.gsi);
    if (!pci_irq_map_handler (&irq, pcnet_irq_handler, 0x01,
                              IOAPIC_DESTINATION_LOGICAL,
                              IOAPIC_DELIVERY_FIXED)) {
      DLOG ("Failed to map IRQ");
      goto abort_virt;
    }
    irq_line = irq.gsi;
  } else {
    DLOG ("Unable to find PCI routing entry");
    goto abort_virt;
  }

  reset ();

  /* Register network device with net subsystem */
  pcnet_ethdev.recv_func = NULL;
  pcnet_ethdev.send_func = pcnet_transmit;
  pcnet_ethdev.get_hwaddr_func = pcnet_get_hwaddr;
  pcnet_ethdev.poll_func = pcnet_poll;

  if (!net_register_device (&pcnet_ethdev)) {
    DLOG ("registration failed");
    goto abort_virt;
  }

  return TRUE;
 abort_virt:
  unmap_virtual_pages (card, frame_count);
 abort_phys:
  free_phys_frames (card_phys, frame_count);
 abort:
  return FALSE;
}

/* ************************************************** */

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
