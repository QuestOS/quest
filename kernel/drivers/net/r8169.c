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

/* Realtek 8169 NIC driver */

/* Based on Linux driver by ShuChen, Francois Romieu, et al. */

#include "drivers/pci/pci.h"
#include "drivers/net/ethernet.h"
#include "drivers/net/skbuff.h"
#include "arch/i386.h"
#include "util/printf.h"
#include "util/bitrev.h"
#include "util/crc32.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "mem/physical.h"
#include "mem/virtual.h"
#include "mem/pow2.h"
#include "kernel.h"
#include "sched/vcpu.h"
#include "sched/sched.h"
#include "module/header.h"

#ifdef USE_VMX

#include "vm/shm.h"
#include "vm/spow2.h"
#include "vm/vmx.h"

#define pow2_alloc shm_pow2_alloc
#define pow2_free shm_pow2_free

#endif

static spinlock * r8169_tx_lock = NULL;
static spinlock * r8169_rx_int_lock = NULL;
static spinlock * r8169_tx_int_lock = NULL;
static spinlock * r8169_reg_lock = NULL;
uint32 r8169_master_sandbox = 0;
atomic_t num_sharing = ATOMIC_T_INIT;
struct rtl8169_private *tp = NULL;
static bool r8169_initialized = FALSE;
static pci_irq_t irq_backup;
static void * global_ioa_backup = NULL;

//#define DEBUG_R8169
//#define TX_TIMING

#define MAX_NUM_SHARE    4

#ifdef TX_TIMING
#include "lwip/udp.h"
#endif

#ifdef DEBUG_R8169
#define DLOG(fmt,...) DLOG_PREFIX("r8169",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define udelay tsc_delay_usec
#define net_device _ethernet_device
#define netdev_priv(x) (x)->drvdata
#define pci_get_drvdata(x) (x)->drvdata

#define PCI_SUBSYSTEM_VENDOR_ID 0x2c
#define PCI_SUBSYSTEM_ID        0x2e
#define PCI_CACHE_LINE_SIZE     0x0c    /* 8 bits */
#define PCI_LATENCY_TIMER       0x0d    /* 8 bits */

static void
pci_write_config_word (pci_device *p, u32 offset, u16 val)
{
  pci_write_word (pci_addr (p->bus, p->slot, p->func, offset), val);
}

static void
pci_write_config_byte (pci_device *p, u32 offset, u8 val)
{
  pci_write_byte (pci_addr (p->bus, p->slot, p->func, offset), val);
}

static void
pci_read_config_word (pci_device *p, u32 offset, u16 *val)
{
  *val = pci_read_word (pci_addr (p->bus, p->slot, p->func, offset));
}

/* The forced speed, 10Mb, 100Mb, gigabit, 2.5Gb, 10GbE. */
#define SPEED_10                10
#define SPEED_100               100
#define SPEED_1000              1000
#define SPEED_2500              2500
#define SPEED_10000             10000

/* Duplex, half or full. */
#define DUPLEX_HALF             0x00
#define DUPLEX_FULL             0x01

#define AUTONEG_DISABLE         0x00
#define AUTONEG_ENABLE          0x01

/* 1000BASE-T Control register */
#define ADVERTISE_1000FULL      0x0200  /* Advertise 1000BASE-T full duplex */
#define ADVERTISE_1000HALF      0x0100  /* Advertise 1000BASE-T half duplex */

/* Advertisement control register. */
#define ADVERTISE_SLCT          0x001f  /* Selector bits               */
#define ADVERTISE_CSMA          0x0001  /* Only selector supported     */
#define ADVERTISE_10HALF        0x0020  /* Try for 10mbps half-duplex  */
#define ADVERTISE_1000XFULL     0x0020  /* Try for 1000BASE-X full-duplex */
#define ADVERTISE_10FULL        0x0040  /* Try for 10mbps full-duplex  */
#define ADVERTISE_1000XHALF     0x0040  /* Try for 1000BASE-X half-duplex */
#define ADVERTISE_100HALF       0x0080  /* Try for 100mbps half-duplex */
#define ADVERTISE_1000XPAUSE    0x0080  /* Try for 1000BASE-X pause    */
#define ADVERTISE_100FULL       0x0100  /* Try for 100mbps full-duplex */
#define ADVERTISE_1000XPSE_ASYM 0x0100  /* Try for 1000BASE-X asym pause */
#define ADVERTISE_100BASE4      0x0200  /* Try for 100mbps 4k packets  */
#define ADVERTISE_PAUSE_CAP     0x0400  /* Try for pause               */
#define ADVERTISE_PAUSE_ASYM    0x0800  /* Try for asymetric pause     */
#define ADVERTISE_RESV          0x1000  /* Unused...                   */
#define ADVERTISE_RFAULT        0x2000  /* Say we can detect faults    */
#define ADVERTISE_LPACK         0x4000  /* Ack link partners response  */
#define ADVERTISE_NPAGE         0x8000  /* Next page bit               */

#define MII_BMCR            0x00        /* Basic mode control register */
#define MII_BMSR            0x01        /* Basic mode status register  */
#define MII_PHYSID1         0x02        /* PHYS ID 1                   */
#define MII_PHYSID2         0x03        /* PHYS ID 2                   */
#define MII_ADVERTISE       0x04        /* Advertisement control reg   */
#define MII_LPA             0x05        /* Link partner ability reg    */
#define MII_EXPANSION       0x06        /* Expansion register          */
#define MII_CTRL1000        0x09        /* 1000BASE-T control          */
#define MII_STAT1000        0x0a        /* 1000BASE-T status           */
#define MII_ESTATUS         0x0f        /* Extended Status */
#define MII_DCOUNTER        0x12        /* Disconnect counter          */
#define MII_FCSCOUNTER      0x13        /* False carrier counter       */
#define MII_NWAYTEST        0x14        /* N-way auto-neg test reg     */
#define MII_RERRCOUNTER     0x15        /* Receive error counter       */
#define MII_SREVISION       0x16        /* Silicon revision            */
#define MII_RESV1           0x17        /* Reserved...                 */
#define MII_LBRERROR        0x18        /* Lpback, rx, bypass error    */
#define MII_PHYADDR         0x19        /* PHY address                 */
#define MII_RESV2           0x1a        /* Reserved...                 */
#define MII_TPISTATUS       0x1b        /* TPI status for 10mbps       */
#define MII_NCONFIG         0x1c        /* Network interface config    */

/* Basic mode control register. */
#define BMCR_RESV               0x003f  /* Unused...                   */
#define BMCR_SPEED1000          0x0040  /* MSB of Speed (1000)         */
#define BMCR_CTST               0x0080  /* Collision test              */
#define BMCR_FULLDPLX           0x0100  /* Full duplex                 */
#define BMCR_ANRESTART          0x0200  /* Auto negotiation restart    */
#define BMCR_ISOLATE            0x0400  /* Disconnect DP83840 from MII */
#define BMCR_PDOWN              0x0800  /* Powerdown the DP83840       */
#define BMCR_ANENABLE           0x1000  /* Enable auto negotiation     */
#define BMCR_SPEED100           0x2000  /* Select 100Mbps              */
#define BMCR_LOOPBACK           0x4000  /* TXD loopback bits           */
#define BMCR_RESET              0x8000  /* Reset the DP83840           */


/* MAC address length */
#define MAC_ADDR_LEN    6

#define MAX_READ_REQUEST_SHIFT  12
#define RX_FIFO_THRESH  7       /* 7 means NO threshold, Rx buffer level before first PCI xfer. */
#define RX_DMA_BURST    6       /* Maximum PCI burst, '6' is 1024 */
#define TX_DMA_BURST    6       /* Maximum PCI burst, '6' is 1024 */
#define EarlyTxThld     0x3F    /* 0x3F means NO early transmit */
#define SafeMtu         0x1c20  /* ... actually life sucks beyond ~7k */
#define InterFrameGap   0x03    /* 3 means InterFrameGap = the shortest one */

#define R8169_REGS_SIZE         256
#define R8169_NAPI_WEIGHT       64
#define NUM_TX_DESC     64      /* Number of Tx descriptor registers */
#define NUM_RX_DESC     256     /* Number of Rx descriptor registers */
#define RX_BUF_SIZE     1536    /* Rx Buffer size */
#define R8169_TX_RING_BYTES     (NUM_TX_DESC * sizeof(struct TxDesc))
#define R8169_RX_RING_BYTES     (NUM_RX_DESC * sizeof(struct RxDesc))

#define RTL8169_TX_TIMEOUT      (6*HZ)
#define RTL8169_PHY_TIMEOUT     (10*HZ)

#define RTL_EEPROM_SIG          cpu_to_le32(0x8129)
#define RTL_EEPROM_SIG_MASK     cpu_to_le32(0xffff)
#define RTL_EEPROM_SIG_ADDR     0x0000

static inline u8
__raw_readb(const volatile void *addr)
{
  return *(const volatile u8 *) addr;
}

static inline u16
__raw_readw(const volatile void *addr)
{
  return *(const volatile u16 *) addr;
}

static inline u32
__raw_readl(const volatile void *addr)
{
  return *(const volatile u32 *) addr;
}

static inline void
__raw_writeb(u8 b, volatile void *addr)
{
  *(volatile u8 *) addr = b;
}

static inline void
__raw_writew(u16 b, volatile void *addr)
{
  *(volatile u16 *) addr = b;
}

static inline void
__raw_writel(u32 b, volatile void *addr)
{
  *(volatile u32 *) addr = b;
}

/* write/read MMIO register */
#define RTL_W8(reg, val8)       __raw_writeb ((val8), ioaddr + (reg))
#define RTL_W16(reg, val16)     __raw_writew ((val16), ioaddr + (reg))
#define RTL_W32(reg, val32)     __raw_writel ((val32), ioaddr + (reg))
#define RTL_R8(reg)             __raw_readb (ioaddr + (reg))
#define RTL_R16(reg)            __raw_readw (ioaddr + (reg))
#define RTL_R32(reg)            __raw_readl (ioaddr + (reg))

enum mac_version {
  RTL_GIGA_MAC_NONE   = 0x00,
  RTL_GIGA_MAC_VER_01 = 0x01, // 8169
  RTL_GIGA_MAC_VER_02 = 0x02, // 8169S
  RTL_GIGA_MAC_VER_03 = 0x03, // 8110S
  RTL_GIGA_MAC_VER_04 = 0x04, // 8169SB
  RTL_GIGA_MAC_VER_05 = 0x05, // 8110SCd
  RTL_GIGA_MAC_VER_06 = 0x06, // 8110SCe
  RTL_GIGA_MAC_VER_07 = 0x07, // 8102e
  RTL_GIGA_MAC_VER_08 = 0x08, // 8102e
  RTL_GIGA_MAC_VER_09 = 0x09, // 8102e
  RTL_GIGA_MAC_VER_10 = 0x0a, // 8101e
  RTL_GIGA_MAC_VER_11 = 0x0b, // 8168Bb
  RTL_GIGA_MAC_VER_12 = 0x0c, // 8168Be
  RTL_GIGA_MAC_VER_13 = 0x0d, // 8101Eb
  RTL_GIGA_MAC_VER_14 = 0x0e, // 8101 ?
  RTL_GIGA_MAC_VER_15 = 0x0f, // 8101 ?
  RTL_GIGA_MAC_VER_16 = 0x11, // 8101Ec
  RTL_GIGA_MAC_VER_17 = 0x10, // 8168Bf
  RTL_GIGA_MAC_VER_18 = 0x12, // 8168CP
  RTL_GIGA_MAC_VER_19 = 0x13, // 8168C
  RTL_GIGA_MAC_VER_20 = 0x14, // 8168C
  RTL_GIGA_MAC_VER_21 = 0x15, // 8168C
  RTL_GIGA_MAC_VER_22 = 0x16, // 8168C
  RTL_GIGA_MAC_VER_23 = 0x17, // 8168CP
  RTL_GIGA_MAC_VER_24 = 0x18, // 8168CP
  RTL_GIGA_MAC_VER_25 = 0x19, // 8168D
  RTL_GIGA_MAC_VER_26 = 0x1a, // 8168D
  RTL_GIGA_MAC_VER_27 = 0x1b  // 8168DP
};

#define _R(NAME,MAC,MASK)                                       \
  { .name = NAME, .mac_version = MAC, .RxConfigMask = MASK }

static const struct {
  const char *name;
  u8 mac_version;
  u32 RxConfigMask;     /* Clears the bits supported by this chip */
} rtl_chip_info[] = {
  _R("RTL8169",         RTL_GIGA_MAC_VER_01, 0xff7e1880), // 8169
  _R("RTL8169s",                RTL_GIGA_MAC_VER_02, 0xff7e1880), // 8169S
  _R("RTL8110s",                RTL_GIGA_MAC_VER_03, 0xff7e1880), // 8110S
  _R("RTL8169sb/8110sb",        RTL_GIGA_MAC_VER_04, 0xff7e1880), // 8169SB
  _R("RTL8169sc/8110sc",        RTL_GIGA_MAC_VER_05, 0xff7e1880), // 8110SCd
  _R("RTL8169sc/8110sc",        RTL_GIGA_MAC_VER_06, 0xff7e1880), // 8110SCe
  _R("RTL8102e",                RTL_GIGA_MAC_VER_07, 0xff7e1880), // PCI-E
  _R("RTL8102e",                RTL_GIGA_MAC_VER_08, 0xff7e1880), // PCI-E
  _R("RTL8102e",                RTL_GIGA_MAC_VER_09, 0xff7e1880), // PCI-E
  _R("RTL8101e",                RTL_GIGA_MAC_VER_10, 0xff7e1880), // PCI-E
  _R("RTL8168b/8111b",  RTL_GIGA_MAC_VER_11, 0xff7e1880), // PCI-E
  _R("RTL8168b/8111b",  RTL_GIGA_MAC_VER_12, 0xff7e1880), // PCI-E
  _R("RTL8101e",                RTL_GIGA_MAC_VER_13, 0xff7e1880), // PCI-E 8139
  _R("RTL8100e",                RTL_GIGA_MAC_VER_14, 0xff7e1880), // PCI-E 8139
  _R("RTL8100e",                RTL_GIGA_MAC_VER_15, 0xff7e1880), // PCI-E 8139
  _R("RTL8168b/8111b",  RTL_GIGA_MAC_VER_17, 0xff7e1880), // PCI-E
  _R("RTL8101e",                RTL_GIGA_MAC_VER_16, 0xff7e1880), // PCI-E
  _R("RTL8168cp/8111cp",        RTL_GIGA_MAC_VER_18, 0xff7e1880), // PCI-E
  _R("RTL8168c/8111c",  RTL_GIGA_MAC_VER_19, 0xff7e1880), // PCI-E
  _R("RTL8168c/8111c",  RTL_GIGA_MAC_VER_20, 0xff7e1880), // PCI-E
  _R("RTL8168c/8111c",  RTL_GIGA_MAC_VER_21, 0xff7e1880), // PCI-E
  _R("RTL8168c/8111c",  RTL_GIGA_MAC_VER_22, 0xff7e1880), // PCI-E
  _R("RTL8168cp/8111cp",        RTL_GIGA_MAC_VER_23, 0xff7e1880), // PCI-E
  _R("RTL8168cp/8111cp",        RTL_GIGA_MAC_VER_24, 0xff7e1880), // PCI-E
  _R("RTL8168d/8111d",  RTL_GIGA_MAC_VER_25, 0xff7e1880), // PCI-E
  _R("RTL8168d/8111d",  RTL_GIGA_MAC_VER_26, 0xff7e1880), // PCI-E
  _R("RTL8168dp/8111dp",        RTL_GIGA_MAC_VER_27, 0xff7e1880)  // PCI-E
};
#undef _R

enum cfg_version {
  RTL_CFG_0 = 0x00,
  RTL_CFG_1,
  RTL_CFG_2
};

#define PCI_VENDOR_ID_REALTEK           0x10ec
#define PCI_VENDOR_ID_DLINK             0x1186
#define PCI_VENDOR_ID_AT                0x1259
#define PCI_VENDOR_ID_GIGABYTE          0x1458

/* List of compatible cards (ended by { 0xFFFF, 0xFFFF }) */
static struct {
  uint16 vendor, device, cfg;
} compatible_ids[] = {
  { PCI_VENDOR_ID_REALTEK,   0x8129, RTL_CFG_0 },
  { PCI_VENDOR_ID_REALTEK,   0x8136, RTL_CFG_2 },
  { PCI_VENDOR_ID_REALTEK,   0x8167, RTL_CFG_0 },
  { PCI_VENDOR_ID_REALTEK,   0x8168, RTL_CFG_1 },
  { PCI_VENDOR_ID_REALTEK,   0x8169, RTL_CFG_0 },
  { PCI_VENDOR_ID_DLINK,     0x4300, RTL_CFG_0 },
  { PCI_VENDOR_ID_AT,        0xc107, RTL_CFG_0 },
  { 0x16ec,                  0x0116, RTL_CFG_0 },
  { 0xFFFF, 0xFFFF }
};

#define __iomem

enum rtl_registers {
  MAC0          = 0,    /* Ethernet hardware address. */
  MAC4          = 4,
  MAR0          = 8,    /* Multicast filter. */
  CounterAddrLow                = 0x10,
  CounterAddrHigh               = 0x14,
  TxDescStartAddrLow    = 0x20,
  TxDescStartAddrHigh   = 0x24,
  TxHDescStartAddrLow   = 0x28,
  TxHDescStartAddrHigh  = 0x2c,
  FLASH         = 0x30,
  ERSR          = 0x36,
  ChipCmd               = 0x37,
  TxPoll                = 0x38,
  IntrMask      = 0x3c,
  IntrStatus    = 0x3e,
  TxConfig      = 0x40,
  RxConfig      = 0x44,
  RxMissed      = 0x4c,
  Cfg9346               = 0x50,
  Config0               = 0x51,
  Config1               = 0x52,
  Config2               = 0x53,
  Config3               = 0x54,
  Config4               = 0x55,
  Config5               = 0x56,
  MultiIntr     = 0x5c,
  PHYAR         = 0x60,
  PHYstatus     = 0x6c,
  RxMaxSize     = 0xda,
  CPlusCmd      = 0xe0,
  IntrMitigate  = 0xe2,
  RxDescAddrLow = 0xe4,
  RxDescAddrHigh        = 0xe8,
  EarlyTxThres  = 0xec,
  FuncEvent     = 0xf0,
  FuncEventMask = 0xf4,
  FuncPresetState       = 0xf8,
  FuncForceEvent        = 0xfc,
};

enum rtl8110_registers {
  TBICSR                        = 0x64,
  TBI_ANAR              = 0x68,
  TBI_LPAR              = 0x6a,
};

enum rtl8168_8101_registers {
  CSIDR                 = 0x64,
  CSIAR                 = 0x68,
#define CSIAR_FLAG                      0x80000000
#define CSIAR_WRITE_CMD                 0x80000000
#define CSIAR_BYTE_ENABLE               0x0f
#define CSIAR_BYTE_ENABLE_SHIFT         12
#define CSIAR_ADDR_MASK                 0x0fff

  EPHYAR                        = 0x80,
#define EPHYAR_FLAG                     0x80000000
#define EPHYAR_WRITE_CMD                0x80000000
#define EPHYAR_REG_MASK                 0x1f
#define EPHYAR_REG_SHIFT                16
#define EPHYAR_DATA_MASK                0xffff
  DBG_REG                       = 0xd1,
#define FIX_NAK_1                       (1 << 4)
#define FIX_NAK_2                       (1 << 3)
  EFUSEAR                       = 0xdc,
#define EFUSEAR_FLAG                    0x80000000
#define EFUSEAR_WRITE_CMD               0x80000000
#define EFUSEAR_READ_CMD                0x00000000
#define EFUSEAR_REG_MASK                0x03ff
#define EFUSEAR_REG_SHIFT               8
#define EFUSEAR_DATA_MASK               0xff
};

enum rtl_register_content {
  /* InterruptStatusBits */
  SYSErr        = 0x8000,
  PCSTimeout    = 0x4000,
  SWInt         = 0x0100,
  TxDescUnavail = 0x0080,
  RxFIFOOver    = 0x0040,
  LinkChg       = 0x0020,
  RxOverflow    = 0x0010,
  TxErr         = 0x0008,
  TxOK          = 0x0004,
  RxErr         = 0x0002,
  RxOK          = 0x0001,

  /* RxStatusDesc */
  RxFOVF        = (1 << 23),
  RxRWT = (1 << 22),
  RxRES = (1 << 21),
  RxRUNT        = (1 << 20),
  RxCRC = (1 << 19),

  /* ChipCmdBits */
  CmdReset      = 0x10,
  CmdRxEnb      = 0x08,
  CmdTxEnb      = 0x04,
  RxBufEmpty    = 0x01,

  /* TXPoll register p.5 */
  HPQ           = 0x80,         /* Poll cmd on the high prio queue */
  NPQ           = 0x40,         /* Poll cmd on the low prio queue */
  FSWInt                = 0x01,         /* Forced software interrupt */

  /* Cfg9346Bits */
  Cfg9346_Lock  = 0x00,
  Cfg9346_Unlock        = 0xc0,

  /* rx_mode_bits */
  AcceptErr     = 0x20,
  AcceptRunt    = 0x10,
  AcceptBroadcast       = 0x08,
  AcceptMulticast       = 0x04,
  AcceptMyPhys  = 0x02,
  AcceptAllPhys = 0x01,

  /* RxConfigBits */
  RxCfgFIFOShift        = 13,
  RxCfgDMAShift =  8,

  /* TxConfigBits */
  TxInterFrameGapShift = 24,
  TxDMAShift = 8,       /* DMA burst value (0-7) is shift this many bits */

  /* Config1 register p.24 */
  LEDS1         = (1 << 7),
  LEDS0         = (1 << 6),
  MSIEnable     = (1 << 5),     /* Enable Message Signaled Interrupt */
  Speed_down    = (1 << 4),
  MEMMAP                = (1 << 3),
  IOMAP         = (1 << 2),
  VPD           = (1 << 1),
  PMEnable      = (1 << 0),     /* Power Management Enable */

  /* Config2 register p. 25 */
  PCI_Clock_66MHz = 0x01,
  PCI_Clock_33MHz = 0x00,

  /* Config3 register p.25 */
  MagicPacket   = (1 << 5),     /* Wake up when receives a Magic Packet */
  LinkUp                = (1 << 4),     /* Wake up when the cable connection is re-established */
  Beacon_en     = (1 << 0),     /* 8168 only. Reserved in the 8168b */

  /* Config5 register p.27 */
  BWF           = (1 << 6),     /* Accept Broadcast wakeup frame */
  MWF           = (1 << 5),     /* Accept Multicast wakeup frame */
  UWF           = (1 << 4),     /* Accept Unicast wakeup frame */
  LanWake               = (1 << 1),     /* LanWake enable/disable */
  PMEStatus     = (1 << 0),     /* PME status can be reset by PCI RST# */

  /* TBICSR p.28 */
  TBIReset      = 0x80000000,
  TBILoopback   = 0x40000000,
  TBINwEnable   = 0x20000000,
  TBINwRestart  = 0x10000000,
  TBILinkOk     = 0x02000000,
  TBINwComplete = 0x01000000,

  /* CPlusCmd p.31 */
  EnableBist    = (1 << 15),    // 8168 8101
  Mac_dbgo_oe   = (1 << 14),    // 8168 8101
  Normal_mode   = (1 << 13),    // unused
  Force_half_dup        = (1 << 12),    // 8168 8101
  Force_rxflow_en       = (1 << 11),    // 8168 8101
  Force_txflow_en       = (1 << 10),    // 8168 8101
  Cxpl_dbg_sel  = (1 << 9),     // 8168 8101
  ASF           = (1 << 8),     // 8168 8101
  PktCntrDisable        = (1 << 7),     // 8168 8101
  Mac_dbgo_sel  = 0x001c,       // 8168
  RxVlan                = (1 << 6),
  RxChkSum      = (1 << 5),
  PCIDAC                = (1 << 4),
  PCIMulRW      = (1 << 3),
  INTT_0                = 0x0000,       // 8168
  INTT_1                = 0x0001,       // 8168
  INTT_2                = 0x0002,       // 8168
  INTT_3                = 0x0003,       // 8168

  /* rtl8169_PHYstatus */
  TBI_Enable    = 0x80,
  TxFlowCtrl    = 0x40,
  RxFlowCtrl    = 0x20,
  _1000bpsF     = 0x10,
  _100bps               = 0x08,
  _10bps                = 0x04,
  LinkStatus    = 0x02,
  FullDup               = 0x01,

  /* _TBICSRBit */
  TBILinkOK     = 0x02000000,

  /* DumpCounterCommand */
  CounterDump   = 0x8,
};

enum desc_status_bit {
  DescOwn               = (1 << 31), /* Descriptor is owned by NIC */
  RingEnd               = (1 << 30), /* End of descriptor ring */
  FirstFrag     = (1 << 29), /* First segment of a packet */
  LastFrag      = (1 << 28), /* Final segment of a packet */

  /* Tx private */
  LargeSend     = (1 << 27), /* TCP Large Send Offload (TSO) */
  MSSShift      = 16,        /* MSS value position */
  MSSMask               = 0xfff,     /* MSS value + LargeSend bit: 12 bits */
  IPCS          = (1 << 18), /* Calculate IP checksum */
  UDPCS         = (1 << 17), /* Calculate UDP/IP checksum */
  TCPCS         = (1 << 16), /* Calculate TCP/IP checksum */
  TxVlanTag     = (1 << 17), /* Add VLAN tag */

  /* Rx private */
  PID1          = (1 << 18), /* Protocol ID bit 1/2 */
  PID0          = (1 << 17), /* Protocol ID bit 2/2 */

#define RxProtoUDP      (PID1)
#define RxProtoTCP      (PID0)
#define RxProtoIP       (PID1 | PID0)
#define RxProtoMask     RxProtoIP

  IPFail                = (1 << 16), /* IP checksum failed */
  UDPFail               = (1 << 15), /* UDP/IP checksum failed */
  TCPFail               = (1 << 14), /* TCP/IP checksum failed */
  RxVlanTag     = (1 << 16), /* VLAN tag available */
};

#define RsvdMask        0x3fffc000

struct TxDesc {
  __le32 opts1;
  __le32 opts2;
  __le64 addr;
};

struct RxDesc {
  __le32 opts1;
  __le32 opts2;
  __le64 addr;
};

struct ring_info {
  struct sk_buff        *skb;
  u32           len;
  u8            __pad[sizeof(void *) - sizeof(u32)];
};

enum features {
  RTL_FEATURE_WOL               = (1 << 0),
  RTL_FEATURE_MSI               = (1 << 1),
  RTL_FEATURE_GMII      = (1 << 2),
};

struct rtl8169_counters {
  __le64        tx_packets;
  __le64        rx_packets;
  __le64        tx_errors;
  __le32        rx_errors;
  __le16        rx_missed;
  __le16        align_errors;
  __le32        tx_one_collision;
  __le32        tx_multi_collision;
  __le64        rx_unicast;
  __le64        rx_broadcast;
  __le32        rx_multicast;
  __le16        tx_aborted;
  __le16        tx_underun;
};

typedef spinlock spinlock_t;
struct napi_struct {
};
struct ethtool_cmd {
};
struct mii_ioctl_data {
};

struct rtl8169_private {
  void __iomem *mmio_addr;      /* memory map physical address */
  struct pci_dev *pci_dev;      /* Index of PCI device */
  struct net_device *dev;
  struct napi_struct napi;
  spinlock_t lock;              /* spin lock flag */
  u32 msg_enable;
  int chipset;
  int mac_version;
  u32 cur_rx[MAX_NUM_SHARE]; /* Index into the Rx descriptor buffer of next Rx pkt. */
  u32 cur_tx; /* Index into the Tx descriptor buffer of next Rx pkt. */
  u32 dirty_rx;
  u32 dirty_tx;
  struct TxDesc *TxDescArray;   /* 256-aligned Tx descriptor ring */
  struct RxDesc *RxDescArray;   /* 256-aligned Rx descriptor ring */
  dma_addr_t TxPhyAddr;
  dma_addr_t RxPhyAddr;
  struct sk_buff *Rx_skbuff[NUM_RX_DESC];       /* Rx data buffers */
  struct ring_info tx_skb[NUM_TX_DESC]; /* Tx data buffers */
  unsigned align;
  unsigned rx_buf_sz;
  //struct timer_list timer;
  u16 cp_cmd;
  u16 intr_event;
  u16 napi_event;
  u16 intr_mask;
  int phy_1000_ctrl_reg;
#ifdef CONFIG_R8169_VLAN
  struct vlan_group *vlgrp;
#endif
  int (*set_speed)(struct net_device *, u8 autoneg, u16 speed, u8 duplex);
  int (*get_settings)(struct net_device *, struct ethtool_cmd *);
  void (*phy_reset_enable)(void __iomem *);
  void (*hw_start)(struct net_device *);
  unsigned int (*phy_reset_pending)(void __iomem *);
  unsigned int (*link_ok)(void __iomem *);
  int (*do_ioctl)(struct rtl8169_private *tp, struct mii_ioctl_data *data, int cmd);
  int pcie_cap;
  //struct delayed_work task;
  unsigned features;

  //struct mii_if_info mii;
  struct rtl8169_counters counters;
  u32 saved_wolopts;
  u8 mac_addr[MAC_ADDR_LEN];
  ethernet_device ethdev[MAX_NUM_SHARE];
};

static void
mdio_write(void __iomem *ioaddr, int reg_addr, int value)
{
  int i;

  RTL_W32(PHYAR, 0x80000000 | (reg_addr & 0x1f) << 16 | (value & 0xffff));

  for (i = 20; i > 0; i--) {
    /*
     * Check if the RTL8169 has completed writing to the specified
     * MII register.
     */
    if (!(RTL_R32(PHYAR) & 0x80000000))
      break;
    udelay(25);
  }
  /*
   * According to hardware specs a 20us delay is required after write
   * complete indication, but before sending next command.
   */
  udelay(20);
}

static int
mdio_read(void __iomem *ioaddr, int reg_addr)
{
  int i, value = -1;

  RTL_W32(PHYAR, 0x0 | (reg_addr & 0x1f) << 16);

  for (i = 20; i > 0; i--) {
    /*
     * Check if the RTL8169 has completed retrieving data from
     * the specified MII register.
     */
    if (RTL_R32(PHYAR) & 0x80000000) {
      value = RTL_R32(PHYAR) & 0xffff;
      break;
    }
    udelay(25);
  }
  /*
   * According to hardware specs a 20us delay is required after read
   * complete indication, but before sending next command.
   */
  udelay(20);

  return value;
}

static void
mdio_patch(void __iomem *ioaddr, int reg_addr, int value)
{
  mdio_write(ioaddr, reg_addr, mdio_read(ioaddr, reg_addr) | value);
}

static void
mdio_plus_minus(void __iomem *ioaddr, int reg_addr, int p, int m)
{
  int val;

  val = mdio_read(ioaddr, reg_addr);
  mdio_write(ioaddr, reg_addr, (val | p) & ~m);
}

static void rtl_ephy_write(void __iomem *ioaddr, int reg_addr, int value)
{
  unsigned int i;

  RTL_W32(EPHYAR, EPHYAR_WRITE_CMD | (value & EPHYAR_DATA_MASK) |
          (reg_addr & EPHYAR_REG_MASK) << EPHYAR_REG_SHIFT);

  for (i = 0; i < 100; i++) {
    if (!(RTL_R32(EPHYAR) & EPHYAR_FLAG))
      break;
    udelay(10);
  }
}

static u16 rtl_ephy_read(void __iomem *ioaddr, int reg_addr)
{
  u16 value = 0xffff;
  unsigned int i;

  RTL_W32(EPHYAR, (reg_addr & EPHYAR_REG_MASK) << EPHYAR_REG_SHIFT);

  for (i = 0; i < 100; i++) {
    if (RTL_R32(EPHYAR) & EPHYAR_FLAG) {
      value = RTL_R32(EPHYAR) & EPHYAR_DATA_MASK;
      break;
    }
    udelay(10);
  }

  return value;
}

static void rtl_csi_write(void __iomem *ioaddr, int addr, int value)
{
  unsigned int i;

  RTL_W32(CSIDR, value);
  RTL_W32(CSIAR, CSIAR_WRITE_CMD | (addr & CSIAR_ADDR_MASK) |
          CSIAR_BYTE_ENABLE << CSIAR_BYTE_ENABLE_SHIFT);

  for (i = 0; i < 100; i++) {
    if (!(RTL_R32(CSIAR) & CSIAR_FLAG))
      break;
    udelay(10);
  }
}

static u32 rtl_csi_read(void __iomem *ioaddr, int addr)
{
  u32 value = ~0x00;
  unsigned int i;

  RTL_W32(CSIAR, (addr & CSIAR_ADDR_MASK) |
          CSIAR_BYTE_ENABLE << CSIAR_BYTE_ENABLE_SHIFT);

  for (i = 0; i < 100; i++) {
    if (RTL_R32(CSIAR) & CSIAR_FLAG) {
      value = RTL_R32(CSIDR);
      break;
    }
    udelay(10);
  }

  return value;
}

static u8
rtl8168d_efuse_read(void __iomem *ioaddr, int reg_addr)
{
  u8 value = 0xff;
  unsigned int i;

  RTL_W32(EFUSEAR, (reg_addr & EFUSEAR_REG_MASK) << EFUSEAR_REG_SHIFT);

  for (i = 0; i < 300; i++) {
    if (RTL_R32(EFUSEAR) & EFUSEAR_FLAG) {
      value = RTL_R32(EFUSEAR) & EFUSEAR_DATA_MASK;
      break;
    }
    udelay(100);
  }

  return value;
}

#define DMA_BIT_MASK(n) (((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))

static void rtl_set_rx_tx_desc_registers(struct rtl8169_private *tp,
                                         void __iomem *ioaddr)
{
  /*
   * Magic spell: some iop3xx ARM board needs the TxDescAddrHigh
   * register to be written before TxDescAddrLow to work.
   * Switching from MMIO to I/O access fixes the issue as well.
   */
  RTL_W32(TxDescStartAddrHigh, ((u64) tp->TxPhyAddr) >> 32);
  RTL_W32(TxDescStartAddrLow, ((u64) tp->TxPhyAddr) & DMA_BIT_MASK(32));
  RTL_W32(RxDescAddrHigh, ((u64) tp->RxPhyAddr) >> 32);
  RTL_W32(RxDescAddrLow, ((u64) tp->RxPhyAddr) & DMA_BIT_MASK(32));
}

static u16 rtl_rw_cpluscmd(void __iomem *ioaddr)
{
  u16 cmd;

  cmd = RTL_R16(CPlusCmd);
  RTL_W16(CPlusCmd, cmd);
  return cmd;
}

static void rtl_set_rx_max_size(void __iomem *ioaddr, unsigned int rx_buf_sz)
{
  /* Low hurts. Let's disable the filtering. */
  RTL_W16(RxMaxSize, rx_buf_sz + 1);
}

static void rtl8169_set_magic_reg(void __iomem *ioaddr, unsigned mac_version)
{
  static const struct {
    u32 mac_version;
    u32 clk;
    u32 val;
  } cfg2_info [] = {
    { RTL_GIGA_MAC_VER_05, PCI_Clock_33MHz, 0x000fff00 }, // 8110SCd
    { RTL_GIGA_MAC_VER_05, PCI_Clock_66MHz, 0x000fffff },
    { RTL_GIGA_MAC_VER_06, PCI_Clock_33MHz, 0x00ffff00 }, // 8110SCe
    { RTL_GIGA_MAC_VER_06, PCI_Clock_66MHz, 0x00ffffff }
  }, *p = cfg2_info;
  unsigned int i;
  u32 clk;

  clk = RTL_R8(Config2) & PCI_Clock_66MHz;
  for (i = 0; i < ARRAY_SIZE(cfg2_info); i++, p++) {
    if ((p->mac_version == mac_version) && (p->clk == clk)) {
      RTL_W32(0x7c, p->val);
      break;
    }
  }
}

static const unsigned int rtl8169_rx_config =
  (RX_FIFO_THRESH << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift);

static void rtl_set_rx_tx_config_registers(struct rtl8169_private *tp)
{
  void __iomem *ioaddr = tp->mmio_addr;
  u32 cfg = rtl8169_rx_config;

  cfg |= (RTL_R32(RxConfig) & rtl_chip_info[tp->chipset].RxConfigMask);
  RTL_W32(RxConfig, cfg);

  /* Set DMA burst size and Interframe Gap Time */
  RTL_W32(TxConfig, (TX_DMA_BURST << TxDMAShift) |
          (InterFrameGap << TxInterFrameGapShift));
}

static void rtl_set_rx_mode(struct net_device *dev)
{
  struct rtl8169_private *tp = netdev_priv(dev);
  void __iomem *ioaddr = tp->mmio_addr;
  u32 mc_filter[2];     /* Multicast hash filter */
  int rx_mode;
  u32 tmp = 0;

  rx_mode = AcceptBroadcast | AcceptMyPhys;
  mc_filter[1] = mc_filter[0] = 0;
  int bit_nr = ether_crc(ETH_ADDR_LEN, tp->mac_addr) >> 26;
  mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
  rx_mode |= AcceptMulticast;

  spinlock_lock(&tp->lock);

  tmp = rtl8169_rx_config | rx_mode |
    (RTL_R32(RxConfig) & rtl_chip_info[tp->chipset].RxConfigMask);

  if (tp->mac_version > RTL_GIGA_MAC_VER_06) {
    u32 data = mc_filter[0];

#define swab32 ___constant_swab32
    mc_filter[0] = swab32(mc_filter[1]);
    mc_filter[1] = swab32(data);
  }

  RTL_W32(MAR0 + 4, mc_filter[1]);
  RTL_W32(MAR0 + 0, mc_filter[0]);

  RTL_W32(RxConfig, tmp);

  spinlock_unlock(&tp->lock);
}

static void rtl_tx_performance_tweak(struct pci_dev *pdev, u16 force)
{
  struct net_device *dev = pci_get_drvdata(pdev);
  struct rtl8169_private *tp = netdev_priv(dev);
  int cap = tp->pcie_cap;

  if (cap) {
    u16 ctl;

#define PCI_EXP_DEVCTL          8       /* Device Control */
#define  PCI_EXP_DEVCTL_READRQ  0x7000  /* Max_Read_Request_Size */
    pci_read_config_word(pdev, cap + PCI_EXP_DEVCTL, &ctl);
    ctl = (ctl & ~PCI_EXP_DEVCTL_READRQ) | force;
    pci_write_config_word(pdev, cap + PCI_EXP_DEVCTL, ctl);
  }
}

static void rtl_csi_access_enable(void __iomem *ioaddr)
{
  u32 csi;

  csi = rtl_csi_read(ioaddr, 0x070c) & 0x00ffffff;
  rtl_csi_write(ioaddr, 0x070c, csi | 0x27000000);
}

struct ephy_info {
  unsigned int offset;
  u16 mask;
  u16 bits;
};

static void rtl_ephy_init(void __iomem *ioaddr, const struct ephy_info *e, int len)
{
  u16 w;

  while (len-- > 0) {
    w = (rtl_ephy_read(ioaddr, e->offset) & ~e->mask) | e->bits;
    rtl_ephy_write(ioaddr, e->offset, w);
    e++;
  }
}

static void rtl_disable_clock_request(struct pci_dev *pdev)
{
  struct net_device *dev = pci_get_drvdata(pdev);
  struct rtl8169_private *tp = netdev_priv(dev);
  int cap = tp->pcie_cap;

  if (cap) {
    u16 ctl;

#define PCI_EXP_LNKCTL          16      /* Link Control */
#define  PCI_EXP_LNKCTL_CLKREQ_EN 0x100 /* Enable clkreq */
    pci_read_config_word(pdev, cap + PCI_EXP_LNKCTL, &ctl);
    ctl &= ~PCI_EXP_LNKCTL_CLKREQ_EN;
    pci_write_config_word(pdev, cap + PCI_EXP_LNKCTL, ctl);
  }
}

static void
rtl_hw_start_8169(struct net_device *dev)
{
  struct rtl8169_private *tp = netdev_priv(dev);
  void __iomem *ioaddr = tp->mmio_addr;
  struct pci_dev *pdev = tp->pci_dev;

  if (tp->mac_version == RTL_GIGA_MAC_VER_05) {
    RTL_W16(CPlusCmd, RTL_R16(CPlusCmd) | PCIMulRW);
    pci_write_config_byte(pdev, PCI_CACHE_LINE_SIZE, 0x08);
  }

  RTL_W8(Cfg9346, Cfg9346_Unlock);
  if ((tp->mac_version == RTL_GIGA_MAC_VER_01) ||
      (tp->mac_version == RTL_GIGA_MAC_VER_02) ||
      (tp->mac_version == RTL_GIGA_MAC_VER_03) ||
      (tp->mac_version == RTL_GIGA_MAC_VER_04))
    RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);

  RTL_W8(EarlyTxThres, EarlyTxThld);

  rtl_set_rx_max_size(ioaddr, tp->rx_buf_sz);

  if ((tp->mac_version == RTL_GIGA_MAC_VER_01) ||
      (tp->mac_version == RTL_GIGA_MAC_VER_02) ||
      (tp->mac_version == RTL_GIGA_MAC_VER_03) ||
      (tp->mac_version == RTL_GIGA_MAC_VER_04))
    rtl_set_rx_tx_config_registers(tp);

  tp->cp_cmd |= rtl_rw_cpluscmd(ioaddr) | PCIMulRW;

  if ((tp->mac_version == RTL_GIGA_MAC_VER_02) ||
      (tp->mac_version == RTL_GIGA_MAC_VER_03)) {
    DLOG("Set MAC Reg C+CR Offset 0xE0. "
         "Bit-3 and bit-14 MUST be 1");
    tp->cp_cmd |= (1 << 14);
  }

  RTL_W16(CPlusCmd, tp->cp_cmd);

  rtl8169_set_magic_reg(ioaddr, tp->mac_version);

  /*
   * Undocumented corner. Supposedly:
   * (TxTimer << 12) | (TxPackets << 8) | (RxTimer << 4) | RxPackets
   */
  RTL_W16(IntrMitigate, 0x0000);

  rtl_set_rx_tx_desc_registers(tp, ioaddr);

  if ((tp->mac_version != RTL_GIGA_MAC_VER_01) &&
      (tp->mac_version != RTL_GIGA_MAC_VER_02) &&
      (tp->mac_version != RTL_GIGA_MAC_VER_03) &&
      (tp->mac_version != RTL_GIGA_MAC_VER_04)) {
    RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);
    rtl_set_rx_tx_config_registers(tp);
  }

  RTL_W8(Cfg9346, Cfg9346_Lock);

  /* Initially a 10 us delay. Turned it into a PCI commit. - FR */
  RTL_R8(IntrMask);

  RTL_W32(RxMissed, 0);

  rtl_set_rx_mode(dev);

  /* no early-rx interrupts */
  RTL_W16(MultiIntr, RTL_R16(MultiIntr) & 0xF000);

  /* Enable all known interrupts by setting the interrupt mask. */
  RTL_W16(IntrMask, tp->intr_event);
}

#define R8168_CPCMD_QUIRK_MASK (                        \
                                EnableBist |            \
                                Mac_dbgo_oe |           \
                                Force_half_dup |        \
                                Force_rxflow_en |       \
                                Force_txflow_en |       \
                                Cxpl_dbg_sel |          \
                                ASF |                   \
                                PktCntrDisable |        \
                                Mac_dbgo_sel)

static void rtl_hw_start_8168bb(void __iomem *ioaddr, struct pci_dev *pdev)
{
  RTL_W8(Config3, RTL_R8(Config3) & ~Beacon_en);

  RTL_W16(CPlusCmd, RTL_R16(CPlusCmd) & ~R8168_CPCMD_QUIRK_MASK);

#define  PCI_EXP_DEVCTL_NOSNOOP_EN 0x0800  /* Enable No Snoop */
  rtl_tx_performance_tweak(pdev,
                           (0x5 << MAX_READ_REQUEST_SHIFT) | PCI_EXP_DEVCTL_NOSNOOP_EN);
}

static void rtl_hw_start_8168bef(void __iomem *ioaddr, struct pci_dev *pdev)
{
  rtl_hw_start_8168bb(ioaddr, pdev);

  RTL_W8(EarlyTxThres, EarlyTxThld);

  RTL_W8(Config4, RTL_R8(Config4) & ~(1 << 0));
}

static void __rtl_hw_start_8168cp(void __iomem *ioaddr, struct pci_dev *pdev)
{
  RTL_W8(Config1, RTL_R8(Config1) | Speed_down);

  RTL_W8(Config3, RTL_R8(Config3) & ~Beacon_en);

  rtl_tx_performance_tweak(pdev, 0x5 << MAX_READ_REQUEST_SHIFT);

  rtl_disable_clock_request(pdev);

  RTL_W16(CPlusCmd, RTL_R16(CPlusCmd) & ~R8168_CPCMD_QUIRK_MASK);
}

static void rtl_hw_start_8168cp_1(void __iomem *ioaddr, struct pci_dev *pdev)
{
  static const struct ephy_info e_info_8168cp[] = {
    { 0x01, 0,  0x0001 },
    { 0x02, 0x0800,     0x1000 },
    { 0x03, 0,  0x0042 },
    { 0x06, 0x0080,     0x0000 },
    { 0x07, 0,  0x2000 }
  };

  rtl_csi_access_enable(ioaddr);

  rtl_ephy_init(ioaddr, e_info_8168cp, ARRAY_SIZE(e_info_8168cp));

  __rtl_hw_start_8168cp(ioaddr, pdev);
}

static void rtl_hw_start_8168cp_2(void __iomem *ioaddr, struct pci_dev *pdev)
{
  rtl_csi_access_enable(ioaddr);

  RTL_W8(Config3, RTL_R8(Config3) & ~Beacon_en);

  rtl_tx_performance_tweak(pdev, 0x5 << MAX_READ_REQUEST_SHIFT);

  RTL_W16(CPlusCmd, RTL_R16(CPlusCmd) & ~R8168_CPCMD_QUIRK_MASK);
}

static void rtl_hw_start_8168cp_3(void __iomem *ioaddr, struct pci_dev *pdev)
{
  rtl_csi_access_enable(ioaddr);

  RTL_W8(Config3, RTL_R8(Config3) & ~Beacon_en);

  /* Magic. */
  RTL_W8(DBG_REG, 0x20);

  RTL_W8(EarlyTxThres, EarlyTxThld);

  rtl_tx_performance_tweak(pdev, 0x5 << MAX_READ_REQUEST_SHIFT);

  RTL_W16(CPlusCmd, RTL_R16(CPlusCmd) & ~R8168_CPCMD_QUIRK_MASK);
}

static void rtl_hw_start_8168c_1(void __iomem *ioaddr, struct pci_dev *pdev)
{
  static const struct ephy_info e_info_8168c_1[] = {
    { 0x02, 0x0800,     0x1000 },
    { 0x03, 0,  0x0002 },
    { 0x06, 0x0080,     0x0000 }
  };

  rtl_csi_access_enable(ioaddr);

  RTL_W8(DBG_REG, 0x06 | FIX_NAK_1 | FIX_NAK_2);

  rtl_ephy_init(ioaddr, e_info_8168c_1, ARRAY_SIZE(e_info_8168c_1));

  __rtl_hw_start_8168cp(ioaddr, pdev);
}

static void rtl_hw_start_8168c_2(void __iomem *ioaddr, struct pci_dev *pdev)
{
  static const struct ephy_info e_info_8168c_2[] = {
    { 0x01, 0,  0x0001 },
    { 0x03, 0x0400,     0x0220 }
  };

  rtl_csi_access_enable(ioaddr);

  rtl_ephy_init(ioaddr, e_info_8168c_2, ARRAY_SIZE(e_info_8168c_2));

  __rtl_hw_start_8168cp(ioaddr, pdev);
}

static void rtl_hw_start_8168c_3(void __iomem *ioaddr, struct pci_dev *pdev)
{
  rtl_hw_start_8168c_2(ioaddr, pdev);
}

static void rtl_hw_start_8168c_4(void __iomem *ioaddr, struct pci_dev *pdev)
{
  rtl_csi_access_enable(ioaddr);

  __rtl_hw_start_8168cp(ioaddr, pdev);
}

static void rtl_hw_start_8168d(void __iomem *ioaddr, struct pci_dev *pdev)
{
  rtl_csi_access_enable(ioaddr);

  rtl_disable_clock_request(pdev);

  RTL_W8(EarlyTxThres, EarlyTxThld);

  rtl_tx_performance_tweak(pdev, 0x5 << MAX_READ_REQUEST_SHIFT);

  RTL_W16(CPlusCmd, RTL_R16(CPlusCmd) & ~R8168_CPCMD_QUIRK_MASK);
}

static void
rtl_hw_start_8168(struct net_device *dev)
{
  struct rtl8169_private *tp = netdev_priv(dev);
  void __iomem *ioaddr = tp->mmio_addr;
  struct pci_dev *pdev = tp->pci_dev;

  DLOG ("hw_start_8168 mac_version=0x%X", tp->mac_version);

  RTL_W8(Cfg9346, Cfg9346_Unlock);

  RTL_W8(EarlyTxThres, EarlyTxThld);

  rtl_set_rx_max_size(ioaddr, tp->rx_buf_sz);

  tp->cp_cmd |= RTL_R16(CPlusCmd) | PktCntrDisable | INTT_1;

  RTL_W16(CPlusCmd, tp->cp_cmd);

  RTL_W16(IntrMitigate, 0x5151);

  /* Work around for RxFIFO overflow. */
  if (tp->mac_version == RTL_GIGA_MAC_VER_11) {
    tp->intr_event |= RxFIFOOver | PCSTimeout;
    tp->intr_event &= ~RxOverflow;
  }

  rtl_set_rx_tx_desc_registers(tp, ioaddr);

  rtl_set_rx_mode(dev);

  RTL_W32(TxConfig, (TX_DMA_BURST << TxDMAShift) |
          (InterFrameGap << TxInterFrameGapShift));

  RTL_R8(IntrMask);

  switch (tp->mac_version) {
  case RTL_GIGA_MAC_VER_11:
    rtl_hw_start_8168bb(ioaddr, pdev);
    break;

  case RTL_GIGA_MAC_VER_12:
  case RTL_GIGA_MAC_VER_17:
    rtl_hw_start_8168bef(ioaddr, pdev);
    break;

  case RTL_GIGA_MAC_VER_18:
    rtl_hw_start_8168cp_1(ioaddr, pdev);
    break;

  case RTL_GIGA_MAC_VER_19:
    rtl_hw_start_8168c_1(ioaddr, pdev);
    break;

  case RTL_GIGA_MAC_VER_20:
    rtl_hw_start_8168c_2(ioaddr, pdev);
    break;

  case RTL_GIGA_MAC_VER_21:
    rtl_hw_start_8168c_3(ioaddr, pdev);
    break;

  case RTL_GIGA_MAC_VER_22:
    rtl_hw_start_8168c_4(ioaddr, pdev);
    break;

  case RTL_GIGA_MAC_VER_23:
    rtl_hw_start_8168cp_2(ioaddr, pdev);
    break;

  case RTL_GIGA_MAC_VER_24:
    rtl_hw_start_8168cp_3(ioaddr, pdev);
    break;

  case RTL_GIGA_MAC_VER_25:
  case RTL_GIGA_MAC_VER_26:
  case RTL_GIGA_MAC_VER_27:
    rtl_hw_start_8168d(ioaddr, pdev);
    break;

  default:
    DLOG("unknown chipset (mac_version = %d).",
         tp->mac_version);
    break;
  }

  RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);

  RTL_W8(Cfg9346, Cfg9346_Lock);

  RTL_W16(MultiIntr, RTL_R16(MultiIntr) & 0xF000);

  DLOG ("IntrMask <- 0x%.04X", tp->intr_event);
  RTL_W16(IntrMask, tp->intr_event);
}

static void
rtl_hw_start_8101(struct net_device *dev)
{
#if 0
  struct rtl8169_private *tp = netdev_priv(dev);
  void __iomem *ioaddr = tp->mmio_addr;
  struct pci_dev *pdev = tp->pci_dev;

  if ((tp->mac_version == RTL_GIGA_MAC_VER_13) ||
      (tp->mac_version == RTL_GIGA_MAC_VER_16)) {
    int cap = tp->pcie_cap;

    if (cap) {
      pci_write_config_word(pdev, cap + PCI_EXP_DEVCTL,
                            PCI_EXP_DEVCTL_NOSNOOP_EN);
    }
  }

  switch (tp->mac_version) {
  case RTL_GIGA_MAC_VER_07:
    rtl_hw_start_8102e_1(ioaddr, pdev);
    break;

  case RTL_GIGA_MAC_VER_08:
    rtl_hw_start_8102e_3(ioaddr, pdev);
    break;

  case RTL_GIGA_MAC_VER_09:
    rtl_hw_start_8102e_2(ioaddr, pdev);
    break;
  }

  RTL_W8(Cfg9346, Cfg9346_Unlock);

  RTL_W8(EarlyTxThres, EarlyTxThld);

  rtl_set_rx_max_size(ioaddr, tp->rx_buf_sz);

  tp->cp_cmd |= rtl_rw_cpluscmd(ioaddr) | PCIMulRW;

  RTL_W16(CPlusCmd, tp->cp_cmd);

  RTL_W16(IntrMitigate, 0x0000);

  rtl_set_rx_tx_desc_registers(tp, ioaddr);

  RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);
  rtl_set_rx_tx_config_registers(tp);

  RTL_W8(Cfg9346, Cfg9346_Lock);

  RTL_R8(IntrMask);

  rtl_set_rx_mode(dev);

  RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);

  RTL_W16(MultiIntr, RTL_R16(MultiIntr) & 0xf000);

  RTL_W16(IntrMask, tp->intr_event);
#endif
  DLOG ("8101 not supported");
}

static int
rtl8169_set_speed_tbi(struct net_device *dev,
                      u8 autoneg, u16 speed, u8 duplex)
{
  return 0;
}

static int rtl8169_set_speed_xmii(struct net_device *dev,
                                  u8 autoneg, u16 speed, u8 duplex)
{
  struct rtl8169_private *tp = netdev_priv(dev);
  void __iomem *ioaddr = tp->mmio_addr;
  int giga_ctrl, bmcr;

  if (autoneg == AUTONEG_ENABLE) {
    int auto_nego;

    auto_nego = mdio_read(ioaddr, MII_ADVERTISE);
    auto_nego |= (ADVERTISE_10HALF | ADVERTISE_10FULL |
                  ADVERTISE_100HALF | ADVERTISE_100FULL);
    auto_nego |= ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM;

    giga_ctrl = mdio_read(ioaddr, MII_CTRL1000);
    giga_ctrl &= ~(ADVERTISE_1000FULL | ADVERTISE_1000HALF);

    /* The 8100e/8101e/8102e do Fast Ethernet only. */
    if ((tp->mac_version != RTL_GIGA_MAC_VER_07) &&
        (tp->mac_version != RTL_GIGA_MAC_VER_08) &&
        (tp->mac_version != RTL_GIGA_MAC_VER_09) &&
        (tp->mac_version != RTL_GIGA_MAC_VER_10) &&
        (tp->mac_version != RTL_GIGA_MAC_VER_13) &&
        (tp->mac_version != RTL_GIGA_MAC_VER_14) &&
        (tp->mac_version != RTL_GIGA_MAC_VER_15) &&
        (tp->mac_version != RTL_GIGA_MAC_VER_16)) {
      giga_ctrl |= ADVERTISE_1000FULL | ADVERTISE_1000HALF;
    } else {
      DLOG("PHY does not support 1000Mbps");
    }

    bmcr = BMCR_ANENABLE | BMCR_ANRESTART;

    if ((tp->mac_version == RTL_GIGA_MAC_VER_11) ||
        (tp->mac_version == RTL_GIGA_MAC_VER_12) ||
        (tp->mac_version >= RTL_GIGA_MAC_VER_17)) {
      /*
       * Wake up the PHY.
       * Vendor specific (0x1f) and reserved (0x0e) MII
       * registers.
       */
      mdio_write(ioaddr, 0x1f, 0x0000);
      mdio_write(ioaddr, 0x0e, 0x0000);
    }

    mdio_write(ioaddr, MII_ADVERTISE, auto_nego);
    mdio_write(ioaddr, MII_CTRL1000, giga_ctrl);
  } else {
    giga_ctrl = 0;

    if (speed == SPEED_10)
      bmcr = 0;
    else if (speed == SPEED_100)
      bmcr = BMCR_SPEED100;
    else
      return -1;

    if (duplex == DUPLEX_FULL)
      bmcr |= BMCR_FULLDPLX;

    mdio_write(ioaddr, 0x1f, 0x0000);
  }

  tp->phy_1000_ctrl_reg = giga_ctrl;

  mdio_write(ioaddr, MII_BMCR, bmcr);
  DLOG ("set_speed_xmii: bmcr=0x%p", bmcr);

  if ((tp->mac_version == RTL_GIGA_MAC_VER_02) ||
      (tp->mac_version == RTL_GIGA_MAC_VER_03)) {
    if ((speed == SPEED_100) && (autoneg != AUTONEG_ENABLE)) {
      mdio_write(ioaddr, 0x17, 0x2138);
      mdio_write(ioaddr, 0x0e, 0x0260);
    } else {
      mdio_write(ioaddr, 0x17, 0x2108);
      mdio_write(ioaddr, 0x0e, 0x0000);
    }
  }

  return 0;
}


static int
rtl8169_gset_tbi(struct net_device *dev, struct ethtool_cmd *cmd)
{
#if 0
  struct rtl8169_private *tp = netdev_priv(dev);
  void __iomem *ioaddr = tp->mmio_addr;
  u32 status;

  cmd->supported =
    SUPPORTED_1000baseT_Full | SUPPORTED_Autoneg | SUPPORTED_FIBRE;
  cmd->port = PORT_FIBRE;
  cmd->transceiver = XCVR_INTERNAL;

  status = RTL_R32(TBICSR);
  cmd->advertising = (status & TBINwEnable) ?  ADVERTISED_Autoneg : 0;
  cmd->autoneg = !!(status & TBINwEnable);

  cmd->speed = SPEED_1000;
  cmd->duplex = DUPLEX_FULL; /* Always set */
#endif
  return 0;
}

static int rtl8169_gset_xmii(struct net_device *dev, struct ethtool_cmd *cmd)
{
#if 0
  struct rtl8169_private *tp = netdev_priv(dev);

  return mii_ethtool_gset(&tp->mii, cmd);
#endif
  return 0;
}

static unsigned int
rtl8169_tbi_reset_pending(void __iomem *ioaddr)
{
  return RTL_R32(TBICSR) & TBIReset;
}

static unsigned int
rtl8169_xmii_reset_pending(void __iomem *ioaddr)
{
  return mdio_read(ioaddr, MII_BMCR) & BMCR_RESET;
}

static unsigned int
rtl8169_tbi_link_ok(void __iomem *ioaddr)
{
  return RTL_R32(TBICSR) & TBILinkOk;
}

static unsigned int
rtl8169_xmii_link_ok(void __iomem *ioaddr)
{
  return RTL_R8(PHYstatus) & LinkStatus;
}

static void
rtl8169_tbi_reset_enable(void __iomem *ioaddr)
{
  RTL_W32(TBICSR, RTL_R32(TBICSR) | TBIReset);
}

static void
rtl8169_xmii_reset_enable(void __iomem *ioaddr)
{
  unsigned int val;

  val = mdio_read(ioaddr, MII_BMCR) | BMCR_RESET;
  mdio_write(ioaddr, MII_BMCR, val & 0xffff);
}

static void
rtl8169_check_link_status(struct net_device *dev,
                          struct rtl8169_private *tp,
                          void __iomem *ioaddr)
{
  spinlock_lock(&tp->lock);
  if (tp->link_ok(ioaddr)) {
    DLOG ("link up");
  } else {
    DLOG ("link down");
  }
  spinlock_unlock(&tp->lock);
}

static int
rtl_tbi_ioctl(struct rtl8169_private *tp, struct mii_ioctl_data *data, int cmd)
{
  return -1;
}

static int
rtl_xmii_ioctl(struct rtl8169_private *tp, struct mii_ioctl_data *data, int cmd)
{
#if 0
  switch (cmd) {
  case SIOCGMIIPHY:
    data->phy_id = 32; /* Internal PHY */
    return 0;

  case SIOCGMIIREG:
    data->val_out = mdio_read(tp->mmio_addr, data->reg_num & 0x1f);
    return 0;

  case SIOCSMIIREG:
    mdio_write(tp->mmio_addr, data->reg_num & 0x1f, data->val_in);
    return 0;
  }
#endif
  return -1;
}



static const struct rtl_cfg_info {
  void (*hw_start)(struct net_device *);
  unsigned int region;
  unsigned int align;
  u16 intr_event;
  u16 napi_event;
  unsigned features;
  u8 default_ver;
} rtl_cfg_infos [] = {
  [RTL_CFG_0] = {
    .hw_start    = rtl_hw_start_8169,
    .region      = 1,
    .align       = 0,
    .intr_event  = SYSErr | LinkChg | RxOverflow | RxFIFOOver | TxErr | TxOK | RxOK | RxErr,
    .napi_event  = RxFIFOOver | TxErr | TxOK | RxOK | RxOverflow,
    .features    = RTL_FEATURE_GMII,
    .default_ver = RTL_GIGA_MAC_VER_01,
  },
  [RTL_CFG_1] = {
    .hw_start    = rtl_hw_start_8168,
    .region      = 2,
    .align       = 8,
    .intr_event  = SYSErr | RxFIFOOver | LinkChg | RxOverflow | TxErr | TxOK | RxOK | RxErr,
    .napi_event  = TxErr | TxOK | RxOK | RxOverflow,
    .features    = RTL_FEATURE_GMII | RTL_FEATURE_MSI,
    .default_ver = RTL_GIGA_MAC_VER_11,
  },
  [RTL_CFG_2] = {
    .hw_start    = rtl_hw_start_8101,
    .region      = 2,
    .align       = 8,
    .intr_event  = SYSErr | LinkChg | RxOverflow | PCSTimeout | RxFIFOOver | TxErr | TxOK | RxOK | RxErr,
    .napi_event  = RxFIFOOver | TxErr | TxOK | RxOK | RxOverflow,
    .features    = RTL_FEATURE_MSI,
    .default_ver = RTL_GIGA_MAC_VER_13,
  }
};

static uint device_index;
static pci_device pdev;

static inline void rtl8169_mark_to_asic(struct RxDesc *desc, u32 rx_buf_sz);
extern struct ip_addr ipaddr, netmask, gw;

#define MIN(a,b) ((a) < (b) ? (a) : (b))
static void
rx_int (struct rtl8169_private *tp)
{
  int i;
  bool read = TRUE;
  //uint cur_rx[MAX_NUM_SHARE];
  uint32 cpu;

  cpu = get_pcpu_id ();
  //cur_rx[cpu] = tp->cur_rx[cpu];
  for (;;) {
    //uint entry = cur_rx[cpu] & (NUM_RX_DESC - 1);
    uint entry = tp->cur_rx[cpu] & (NUM_RX_DESC - 1);
    struct RxDesc *desc = tp->RxDescArray + entry;
    u32 status;

    status = __le32_to_cpu (desc->opts1);
    if (status & DescOwn) {
      DLOG ("Driver(%d) : DescOwn", get_pcpu_id ());
      for (i = 0; i < MAX_NUM_SHARE; i++) {
        if (tp->cur_rx[cpu] < tp->cur_rx[i])
          tp->cur_rx[cpu] = tp->cur_rx[i];
      }
      break;
    } else {
      struct sk_buff *skb = tp->Rx_skbuff[entry];
      int pkt_size = (status & 0x00001FFF) - 4;
      //DLOG ("RX: entry=%d size=%d", entry, pkt_size);
      //u8 *p = skb->data;
      //DLOG ("  %.02X %.02X %.02X %.02X %.02X %.02X",
      //      p[0], p[1], p[2], p[3], p[4], p[5]);
      //p+=6;
      //DLOG ("  %.02X %.02X %.02X %.02X %.02X %.02X",
      //      p[0], p[1], p[2], p[3], p[4], p[5]);
      DLOG ("Driver(%d) : IP=0x%X, NM=0x%X, GW=0x%X", get_pcpu_id (),
            ipaddr.addr, netmask.addr, gw.addr);
      if (tp->ethdev[cpu].recv_func)
        tp->ethdev[cpu].recv_func (&tp->ethdev[cpu], skb->data, pkt_size);

      for (i = 0; i < num_sharing.counter; i++) {
        if ((cpu != i) && (tp->cur_rx[i] <= tp->cur_rx[cpu])) {
          read = FALSE;
          /* --??-- Hack! Let's handle this case later */
          if ((tp->cur_rx[cpu] - tp->cur_rx[i]) >= 200)
            com1_printf ("rx falling behind!\n");
        }
      }

      if (read)
        rtl8169_mark_to_asic(desc, tp->rx_buf_sz);
      
      tp->cur_rx[cpu]++;
    }
    //cur_rx[cpu]++;
  }
  //tp->cur_rx[cpu] = cur_rx[cpu];
}

uint32 free_skb_count = 0;
static inline void free_skb (struct sk_buff *);
static void
tx_int (struct rtl8169_private *tp)
{
  void __iomem *ioaddr = tp->mmio_addr;
  uint dirty_tx, tx_left;
#ifdef USE_VMX
  spinlock_lock (r8169_tx_int_lock);
#endif
  dirty_tx = tp->dirty_tx;
  tx_left = tp->cur_tx - dirty_tx;
  while (tx_left > 1) {
    uint entry = dirty_tx & (NUM_TX_DESC - 1);
    struct ring_info *tx_skb = tp->tx_skb + entry;
    struct TxDesc *desc = tp->TxDescArray + entry;
    u32 status;

    status = __le32_to_cpu(tp->TxDescArray[entry].opts1);
    if (status & DescOwn) {
      //com1_printf ("tx own\n");
      //if ((tp->cur_tx - dirty_tx) > 50)
      //  com1_printf ("dirty behind\n");
      break;
    }

    DLOG ("TX: entry %d sent %d bytes", entry, tx_skb->len);
    desc->opts1 = desc->opts2 = desc->addr = 0;
    tx_skb->len = 0;
    free_skb (tx_skb->skb);
    free_skb_count++;

    dirty_tx++;
    tx_left--;
  }
  if (tp->dirty_tx != dirty_tx) {
    tp->dirty_tx = dirty_tx;
    /*
     * 8168 hack: TxPoll requests are lost when the Tx packets are
     * too close. Let's kick an extra TxPoll request when a burst
     * of start_xmit activity is detected (if it is not detected,
     * it is slow enough). -- FR
     */
    if (tp->cur_tx != dirty_tx)
      RTL_W8(TxPoll, NPQ);
  }
#ifdef USE_VMX
  spinlock_unlock (r8169_tx_int_lock);
#endif
}

static u32 r8169_bh_stack[MAX_NUM_SHARE][1024] ALIGNED (0x1000);
static task_id r8169_bh_id[MAX_NUM_SHARE];
static void
r8169_bh_thread (void)
{
  int cpu;
  cpu = get_pcpu_id ();
  logger_printf ("r8169: bh: hello from 0x%x, cpu=%d\n", str (), cpu);
  for (;;) {
    struct rtl8169_private *tp = pci_get_drvdata(&pdev);
    int handled = 0;
    int status;
    void __iomem *ioaddr = tp->mmio_addr;

    status = RTL_R16(IntrStatus);

    //while (status && status != 0xffff) {
      DLOG ("IRQ status=0x%p, cpu=%d", status, cpu);
      handled = 1;

      /* Handle all of the error cases first. These will reset
       * the chip, so just exit the loop.
       */

      /* Work around for rx fifo overflow */
      if (unlikely(status & RxFIFOOver)) {
        //netif_stop_queue(dev);
        //rtl8169_tx_timeout(&tp->ethdev);
        com1_printf ("rx fifo overflow\n");
        break;
      }

      if (unlikely(status & SYSErr)) {
        //rtl8169_pcierr_interrupt(&tp->ethdev);
        com1_printf ("System Error\n");
        break;
      }

      //if (status & LinkChg)
        //rtl8169_check_link_status(&tp->ethdev[0], tp, ioaddr);

      //if (status & RxOK) {
#ifdef USE_VMX
      spinlock_lock (r8169_rx_int_lock);
#endif
      rx_int (tp);
#ifdef USE_VMX
      spinlock_unlock (r8169_rx_int_lock);
#endif
      //}

      if ((status & TxOK) && (cpu == r8169_master_sandbox)) {
        tx_int (tp);
      }

      /* We only get a new MSI interrupt when all active irq
       * sources on the chip have been acknowledged. So, ack
       * everything we've seen and check if new sources have become
       * active to avoid blocking all interrupts from the chip.
       */
      if (cpu == r8169_master_sandbox) {
        status = RTL_R16(IntrStatus);
        RTL_W16(IntrStatus,
            (status & RxFIFOOver) ? (status | RxOK | TxOK | RxOverflow) : status | RxOK | TxOK);
      }
      //status = RTL_R16(IntrStatus);
    //}

    iovcpu_job_completion ();
  }
}

#ifdef TX_TIMING
static u64 tx_start = 0, tx_finish;
#endif

unsigned int n_int = 0;
bool print_int = FALSE;

static uint
irq_handler (u8 vec)
{
  int cpu = get_pcpu_id ();
  if (print_int)
    com1_printf ("r8169 interrupt!\n");

#ifdef TX_TIMING
  /* assume tx_start != 0 means this is TX IRQ */
  if (tx_start > 0) {
    SERIALIZE0;
    RDTSC (tx_finish);
    logger_printf ("r8169: TX: IRQ received: diff=0x%llX\n", tx_finish - tx_start);
    tx_start = 0;
  }
#endif

  if (cpu == 0) n_int++;
  if (r8169_bh_id[cpu]) {
    extern vcpu *vcpu_lookup (int);
    /* hack: use VCPU2's period */
    iovcpu_job_wakeup (r8169_bh_id[cpu], vcpu_lookup (2)->T);
  }

  return 0;
}

static void
get_mac_version(struct rtl8169_private *tp,
                void __iomem *ioaddr)
{
  /*
   * The driver currently handles the 8168Bf and the 8168Be identically
   * but they can be identified more specifically through the test below
   * if needed:
   *
   * (RTL_R32(TxConfig) & 0x700000) == 0x500000 ? 8168Bf : 8168Be
   *
   * Same thing for the 8101Eb and the 8101Ec:
   *
   * (RTL_R32(TxConfig) & 0x700000) == 0x200000 ? 8101Eb : 8101Ec
   */
  static const struct {
    u32 mask;
    u32 val;
    int mac_version;
  } mac_info[] = {
    /* 8168D family. */
    { 0x7cf00000, 0x28300000,   RTL_GIGA_MAC_VER_26 },
    { 0x7cf00000, 0x28100000,   RTL_GIGA_MAC_VER_25 },
    { 0x7c800000, 0x28800000,   RTL_GIGA_MAC_VER_27 },
    { 0x7c800000, 0x28000000,   RTL_GIGA_MAC_VER_26 },

    /* 8168C family. */
    { 0x7cf00000, 0x3cb00000,   RTL_GIGA_MAC_VER_24 },
    { 0x7cf00000, 0x3c900000,   RTL_GIGA_MAC_VER_23 },
    { 0x7cf00000, 0x3c800000,   RTL_GIGA_MAC_VER_18 },
    { 0x7c800000, 0x3c800000,   RTL_GIGA_MAC_VER_24 },
    { 0x7cf00000, 0x3c000000,   RTL_GIGA_MAC_VER_19 },
    { 0x7cf00000, 0x3c200000,   RTL_GIGA_MAC_VER_20 },
    { 0x7cf00000, 0x3c300000,   RTL_GIGA_MAC_VER_21 },
    { 0x7cf00000, 0x3c400000,   RTL_GIGA_MAC_VER_22 },
    { 0x7c800000, 0x3c000000,   RTL_GIGA_MAC_VER_22 },

    /* 8168B family. */
    { 0x7cf00000, 0x38000000,   RTL_GIGA_MAC_VER_12 },
    { 0x7cf00000, 0x38500000,   RTL_GIGA_MAC_VER_17 },
    { 0x7c800000, 0x38000000,   RTL_GIGA_MAC_VER_17 },
    { 0x7c800000, 0x30000000,   RTL_GIGA_MAC_VER_11 },

    /* 8101 family. */
    { 0x7cf00000, 0x34a00000,   RTL_GIGA_MAC_VER_09 },
    { 0x7cf00000, 0x24a00000,   RTL_GIGA_MAC_VER_09 },
    { 0x7cf00000, 0x34900000,   RTL_GIGA_MAC_VER_08 },
    { 0x7cf00000, 0x24900000,   RTL_GIGA_MAC_VER_08 },
    { 0x7cf00000, 0x34800000,   RTL_GIGA_MAC_VER_07 },
    { 0x7cf00000, 0x24800000,   RTL_GIGA_MAC_VER_07 },
    { 0x7cf00000, 0x34000000,   RTL_GIGA_MAC_VER_13 },
    { 0x7cf00000, 0x34300000,   RTL_GIGA_MAC_VER_10 },
    { 0x7cf00000, 0x34200000,   RTL_GIGA_MAC_VER_16 },
    { 0x7c800000, 0x34800000,   RTL_GIGA_MAC_VER_09 },
    { 0x7c800000, 0x24800000,   RTL_GIGA_MAC_VER_09 },
    { 0x7c800000, 0x34000000,   RTL_GIGA_MAC_VER_16 },
    /* FIXME: where did these entries come from ? -- FR */
    { 0xfc800000, 0x38800000,   RTL_GIGA_MAC_VER_15 },
    { 0xfc800000, 0x30800000,   RTL_GIGA_MAC_VER_14 },

    /* 8110 family. */
    { 0xfc800000, 0x98000000,   RTL_GIGA_MAC_VER_06 },
    { 0xfc800000, 0x18000000,   RTL_GIGA_MAC_VER_05 },
    { 0xfc800000, 0x10000000,   RTL_GIGA_MAC_VER_04 },
    { 0xfc800000, 0x04000000,   RTL_GIGA_MAC_VER_03 },
    { 0xfc800000, 0x00800000,   RTL_GIGA_MAC_VER_02 },
    { 0xfc800000, 0x00000000,   RTL_GIGA_MAC_VER_01 },

    /* Catch-all */
    { 0x00000000, 0x00000000,   RTL_GIGA_MAC_NONE   }
  }, *p = mac_info;
  u32 reg;

  reg = RTL_R32(TxConfig);
  while ((reg & p->mask) != p->val)
    p++;
  tp->mac_version = p->mac_version;
}

static void
print_mac_version(struct rtl8169_private *tp)
{
  DLOG ("mac_version = 0x%02x", tp->mac_version);
}

struct phy_reg {
  u16 reg;
  u16 val;
};

static void rtl_phy_write(void __iomem *ioaddr, const struct phy_reg *regs, int len)
{
  while (len-- > 0) {
    mdio_write(ioaddr, regs->reg, regs->val);
    regs++;
  }
}

static void rtl8169s_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init[] = {
    { 0x1f, 0x0001 },
    { 0x06, 0x006e },
    { 0x08, 0x0708 },
    { 0x15, 0x4000 },
    { 0x18, 0x65c7 },

    { 0x1f, 0x0001 },
    { 0x03, 0x00a1 },
    { 0x02, 0x0008 },
    { 0x01, 0x0120 },
    { 0x00, 0x1000 },
    { 0x04, 0x0800 },
    { 0x04, 0x0000 },

    { 0x03, 0xff41 },
    { 0x02, 0xdf60 },
    { 0x01, 0x0140 },
    { 0x00, 0x0077 },
    { 0x04, 0x7800 },
    { 0x04, 0x7000 },

    { 0x03, 0x802f },
    { 0x02, 0x4f02 },
    { 0x01, 0x0409 },
    { 0x00, 0xf0f9 },
    { 0x04, 0x9800 },
    { 0x04, 0x9000 },

    { 0x03, 0xdf01 },
    { 0x02, 0xdf20 },
    { 0x01, 0xff95 },
    { 0x00, 0xba00 },
    { 0x04, 0xa800 },
    { 0x04, 0xa000 },

    { 0x03, 0xff41 },
    { 0x02, 0xdf20 },
    { 0x01, 0x0140 },
    { 0x00, 0x00bb },
    { 0x04, 0xb800 },
    { 0x04, 0xb000 },

    { 0x03, 0xdf41 },
    { 0x02, 0xdc60 },
    { 0x01, 0x6340 },
    { 0x00, 0x007d },
    { 0x04, 0xd800 },
    { 0x04, 0xd000 },

    { 0x03, 0xdf01 },
    { 0x02, 0xdf20 },
    { 0x01, 0x100a },
    { 0x00, 0xa0ff },
    { 0x04, 0xf800 },
    { 0x04, 0xf000 },

    { 0x1f, 0x0000 },
    { 0x0b, 0x0000 },
    { 0x00, 0x9200 }
  };

  rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));
}

static void rtl8169sb_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init[] = {
    { 0x1f, 0x0002 },
    { 0x01, 0x90d0 },
    { 0x1f, 0x0000 }
  };

  rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));
}

static void rtl8169scd_hw_phy_config_quirk(struct rtl8169_private *tp,
                                           void __iomem *ioaddr)
{
  struct pci_dev *pdev = tp->pci_dev;
  u16 vendor_id, device_id;

  pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &vendor_id);
  pci_read_config_word(pdev, PCI_SUBSYSTEM_ID, &device_id);

  if ((vendor_id != PCI_VENDOR_ID_GIGABYTE) || (device_id != 0xe000))
    return;

  mdio_write(ioaddr, 0x1f, 0x0001);
  mdio_write(ioaddr, 0x10, 0xf01b);
  mdio_write(ioaddr, 0x1f, 0x0000);
}

static void rtl8169scd_hw_phy_config(struct rtl8169_private *tp,
                                     void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init[] = {
    { 0x1f, 0x0001 },
    { 0x04, 0x0000 },
    { 0x03, 0x00a1 },
    { 0x02, 0x0008 },
    { 0x01, 0x0120 },
    { 0x00, 0x1000 },
    { 0x04, 0x0800 },
    { 0x04, 0x9000 },
    { 0x03, 0x802f },
    { 0x02, 0x4f02 },
    { 0x01, 0x0409 },
    { 0x00, 0xf099 },
    { 0x04, 0x9800 },
    { 0x04, 0xa000 },
    { 0x03, 0xdf01 },
    { 0x02, 0xdf20 },
    { 0x01, 0xff95 },
    { 0x00, 0xba00 },
    { 0x04, 0xa800 },
    { 0x04, 0xf000 },
    { 0x03, 0xdf01 },
    { 0x02, 0xdf20 },
    { 0x01, 0x101a },
    { 0x00, 0xa0ff },
    { 0x04, 0xf800 },
    { 0x04, 0x0000 },
    { 0x1f, 0x0000 },

    { 0x1f, 0x0001 },
    { 0x10, 0xf41b },
    { 0x14, 0xfb54 },
    { 0x18, 0xf5c7 },
    { 0x1f, 0x0000 },

    { 0x1f, 0x0001 },
    { 0x17, 0x0cc0 },
    { 0x1f, 0x0000 }
  };

  rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));

  rtl8169scd_hw_phy_config_quirk(tp, ioaddr);
}

static void rtl8169sce_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init[] = {
    { 0x1f, 0x0001 },
    { 0x04, 0x0000 },
    { 0x03, 0x00a1 },
    { 0x02, 0x0008 },
    { 0x01, 0x0120 },
    { 0x00, 0x1000 },
    { 0x04, 0x0800 },
    { 0x04, 0x9000 },
    { 0x03, 0x802f },
    { 0x02, 0x4f02 },
    { 0x01, 0x0409 },
    { 0x00, 0xf099 },
    { 0x04, 0x9800 },
    { 0x04, 0xa000 },
    { 0x03, 0xdf01 },
    { 0x02, 0xdf20 },
    { 0x01, 0xff95 },
    { 0x00, 0xba00 },
    { 0x04, 0xa800 },
    { 0x04, 0xf000 },
    { 0x03, 0xdf01 },
    { 0x02, 0xdf20 },
    { 0x01, 0x101a },
    { 0x00, 0xa0ff },
    { 0x04, 0xf800 },
    { 0x04, 0x0000 },
    { 0x1f, 0x0000 },

    { 0x1f, 0x0001 },
    { 0x0b, 0x8480 },
    { 0x1f, 0x0000 },

    { 0x1f, 0x0001 },
    { 0x18, 0x67c7 },
    { 0x04, 0x2000 },
    { 0x03, 0x002f },
    { 0x02, 0x4360 },
    { 0x01, 0x0109 },
    { 0x00, 0x3022 },
    { 0x04, 0x2800 },
    { 0x1f, 0x0000 },

    { 0x1f, 0x0001 },
    { 0x17, 0x0cc0 },
    { 0x1f, 0x0000 }
  };

  rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));
}

static void rtl8168bb_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init[] = {
    { 0x10, 0xf41b },
    { 0x1f, 0x0000 }
  };

  mdio_write(ioaddr, 0x1f, 0x0001);
  mdio_patch(ioaddr, 0x16, 1 << 0);

  rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));
}

static void rtl8168bef_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init[] = {
    { 0x1f, 0x0001 },
    { 0x10, 0xf41b },
    { 0x1f, 0x0000 }
  };

  rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));
}

static void rtl8168cp_1_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init[] = {
    { 0x1f, 0x0000 },
    { 0x1d, 0x0f00 },
    { 0x1f, 0x0002 },
    { 0x0c, 0x1ec8 },
    { 0x1f, 0x0000 }
  };

  rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));
}

static void rtl8168cp_2_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init[] = {
    { 0x1f, 0x0001 },
    { 0x1d, 0x3d98 },
    { 0x1f, 0x0000 }
  };

  mdio_write(ioaddr, 0x1f, 0x0000);
  mdio_patch(ioaddr, 0x14, 1 << 5);
  mdio_patch(ioaddr, 0x0d, 1 << 5);

  rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));
}

static void rtl8168c_1_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init[] = {
    { 0x1f, 0x0001 },
    { 0x12, 0x2300 },
    { 0x1f, 0x0002 },
    { 0x00, 0x88d4 },
    { 0x01, 0x82b1 },
    { 0x03, 0x7002 },
    { 0x08, 0x9e30 },
    { 0x09, 0x01f0 },
    { 0x0a, 0x5500 },
    { 0x0c, 0x00c8 },
    { 0x1f, 0x0003 },
    { 0x12, 0xc096 },
    { 0x16, 0x000a },
    { 0x1f, 0x0000 },
    { 0x1f, 0x0000 },
    { 0x09, 0x2000 },
    { 0x09, 0x0000 }
  };

  rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));

  mdio_patch(ioaddr, 0x14, 1 << 5);
  mdio_patch(ioaddr, 0x0d, 1 << 5);
  mdio_write(ioaddr, 0x1f, 0x0000);
}

static void rtl8168c_2_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init[] = {
    { 0x1f, 0x0001 },
    { 0x12, 0x2300 },
    { 0x03, 0x802f },
    { 0x02, 0x4f02 },
    { 0x01, 0x0409 },
    { 0x00, 0xf099 },
    { 0x04, 0x9800 },
    { 0x04, 0x9000 },
    { 0x1d, 0x3d98 },
    { 0x1f, 0x0002 },
    { 0x0c, 0x7eb8 },
    { 0x06, 0x0761 },
    { 0x1f, 0x0003 },
    { 0x16, 0x0f0a },
    { 0x1f, 0x0000 }
  };

  rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));

  mdio_patch(ioaddr, 0x16, 1 << 0);
  mdio_patch(ioaddr, 0x14, 1 << 5);
  mdio_patch(ioaddr, 0x0d, 1 << 5);
  mdio_write(ioaddr, 0x1f, 0x0000);
}

static void rtl8168c_3_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init[] = {
    { 0x1f, 0x0001 },
    { 0x12, 0x2300 },
    { 0x1d, 0x3d98 },
    { 0x1f, 0x0002 },
    { 0x0c, 0x7eb8 },
    { 0x06, 0x5461 },
    { 0x1f, 0x0003 },
    { 0x16, 0x0f0a },
    { 0x1f, 0x0000 }
  };

  rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));

  mdio_patch(ioaddr, 0x16, 1 << 0);
  mdio_patch(ioaddr, 0x14, 1 << 5);
  mdio_patch(ioaddr, 0x0d, 1 << 5);
  mdio_write(ioaddr, 0x1f, 0x0000);
}

static void rtl8168c_4_hw_phy_config(void __iomem *ioaddr)
{
  rtl8168c_3_hw_phy_config(ioaddr);
}

static void rtl8168d_1_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init_0[] = {
    { 0x1f, 0x0001 },
    { 0x06, 0x4064 },
    { 0x07, 0x2863 },
    { 0x08, 0x059c },
    { 0x09, 0x26b4 },
    { 0x0a, 0x6a19 },
    { 0x0b, 0xdcc8 },
    { 0x10, 0xf06d },
    { 0x14, 0x7f68 },
    { 0x18, 0x7fd9 },
    { 0x1c, 0xf0ff },
    { 0x1d, 0x3d9c },
    { 0x1f, 0x0003 },
    { 0x12, 0xf49f },
    { 0x13, 0x070b },
    { 0x1a, 0x05ad },
    { 0x14, 0x94c0 }
  };
  static const struct phy_reg phy_reg_init_1[] = {
    { 0x1f, 0x0002 },
    { 0x06, 0x5561 },
    { 0x1f, 0x0005 },
    { 0x05, 0x8332 },
    { 0x06, 0x5561 }
  };
  static const struct phy_reg phy_reg_init_2[] = {
    { 0x1f, 0x0005 },
    { 0x05, 0xffc2 },
    { 0x1f, 0x0005 },
    { 0x05, 0x8000 },
    { 0x06, 0xf8f9 },
    { 0x06, 0xfaef },
    { 0x06, 0x59ee },
    { 0x06, 0xf8ea },
    { 0x06, 0x00ee },
    { 0x06, 0xf8eb },
    { 0x06, 0x00e0 },
    { 0x06, 0xf87c },
    { 0x06, 0xe1f8 },
    { 0x06, 0x7d59 },
    { 0x06, 0x0fef },
    { 0x06, 0x0139 },
    { 0x06, 0x029e },
    { 0x06, 0x06ef },
    { 0x06, 0x1039 },
    { 0x06, 0x089f },
    { 0x06, 0x2aee },
    { 0x06, 0xf8ea },
    { 0x06, 0x00ee },
    { 0x06, 0xf8eb },
    { 0x06, 0x01e0 },
    { 0x06, 0xf87c },
    { 0x06, 0xe1f8 },
    { 0x06, 0x7d58 },
    { 0x06, 0x409e },
    { 0x06, 0x0f39 },
    { 0x06, 0x46aa },
    { 0x06, 0x0bbf },
    { 0x06, 0x8290 },
    { 0x06, 0xd682 },
    { 0x06, 0x9802 },
    { 0x06, 0x014f },
    { 0x06, 0xae09 },
    { 0x06, 0xbf82 },
    { 0x06, 0x98d6 },
    { 0x06, 0x82a0 },
    { 0x06, 0x0201 },
    { 0x06, 0x4fef },
    { 0x06, 0x95fe },
    { 0x06, 0xfdfc },
    { 0x06, 0x05f8 },
    { 0x06, 0xf9fa },
    { 0x06, 0xeef8 },
    { 0x06, 0xea00 },
    { 0x06, 0xeef8 },
    { 0x06, 0xeb00 },
    { 0x06, 0xe2f8 },
    { 0x06, 0x7ce3 },
    { 0x06, 0xf87d },
    { 0x06, 0xa511 },
    { 0x06, 0x1112 },
    { 0x06, 0xd240 },
    { 0x06, 0xd644 },
    { 0x06, 0x4402 },
    { 0x06, 0x8217 },
    { 0x06, 0xd2a0 },
    { 0x06, 0xd6aa },
    { 0x06, 0xaa02 },
    { 0x06, 0x8217 },
    { 0x06, 0xae0f },
    { 0x06, 0xa544 },
    { 0x06, 0x4402 },
    { 0x06, 0xae4d },
    { 0x06, 0xa5aa },
    { 0x06, 0xaa02 },
    { 0x06, 0xae47 },
    { 0x06, 0xaf82 },
    { 0x06, 0x13ee },
    { 0x06, 0x834e },
    { 0x06, 0x00ee },
    { 0x06, 0x834d },
    { 0x06, 0x0fee },
    { 0x06, 0x834c },
    { 0x06, 0x0fee },
    { 0x06, 0x834f },
    { 0x06, 0x00ee },
    { 0x06, 0x8351 },
    { 0x06, 0x00ee },
    { 0x06, 0x834a },
    { 0x06, 0xffee },
    { 0x06, 0x834b },
    { 0x06, 0xffe0 },
    { 0x06, 0x8330 },
    { 0x06, 0xe183 },
    { 0x06, 0x3158 },
    { 0x06, 0xfee4 },
    { 0x06, 0xf88a },
    { 0x06, 0xe5f8 },
    { 0x06, 0x8be0 },
    { 0x06, 0x8332 },
    { 0x06, 0xe183 },
    { 0x06, 0x3359 },
    { 0x06, 0x0fe2 },
    { 0x06, 0x834d },
    { 0x06, 0x0c24 },
    { 0x06, 0x5af0 },
    { 0x06, 0x1e12 },
    { 0x06, 0xe4f8 },
    { 0x06, 0x8ce5 },
    { 0x06, 0xf88d },
    { 0x06, 0xaf82 },
    { 0x06, 0x13e0 },
    { 0x06, 0x834f },
    { 0x06, 0x10e4 },
    { 0x06, 0x834f },
    { 0x06, 0xe083 },
    { 0x06, 0x4e78 },
    { 0x06, 0x009f },
    { 0x06, 0x0ae0 },
    { 0x06, 0x834f },
    { 0x06, 0xa010 },
    { 0x06, 0xa5ee },
    { 0x06, 0x834e },
    { 0x06, 0x01e0 },
    { 0x06, 0x834e },
    { 0x06, 0x7805 },
    { 0x06, 0x9e9a },
    { 0x06, 0xe083 },
    { 0x06, 0x4e78 },
    { 0x06, 0x049e },
    { 0x06, 0x10e0 },
    { 0x06, 0x834e },
    { 0x06, 0x7803 },
    { 0x06, 0x9e0f },
    { 0x06, 0xe083 },
    { 0x06, 0x4e78 },
    { 0x06, 0x019e },
    { 0x06, 0x05ae },
    { 0x06, 0x0caf },
    { 0x06, 0x81f8 },
    { 0x06, 0xaf81 },
    { 0x06, 0xa3af },
    { 0x06, 0x81dc },
    { 0x06, 0xaf82 },
    { 0x06, 0x13ee },
    { 0x06, 0x8348 },
    { 0x06, 0x00ee },
    { 0x06, 0x8349 },
    { 0x06, 0x00e0 },
    { 0x06, 0x8351 },
    { 0x06, 0x10e4 },
    { 0x06, 0x8351 },
    { 0x06, 0x5801 },
    { 0x06, 0x9fea },
    { 0x06, 0xd000 },
    { 0x06, 0xd180 },
    { 0x06, 0x1f66 },
    { 0x06, 0xe2f8 },
    { 0x06, 0xeae3 },
    { 0x06, 0xf8eb },
    { 0x06, 0x5af8 },
    { 0x06, 0x1e20 },
    { 0x06, 0xe6f8 },
    { 0x06, 0xeae5 },
    { 0x06, 0xf8eb },
    { 0x06, 0xd302 },
    { 0x06, 0xb3fe },
    { 0x06, 0xe2f8 },
    { 0x06, 0x7cef },
    { 0x06, 0x325b },
    { 0x06, 0x80e3 },
    { 0x06, 0xf87d },
    { 0x06, 0x9e03 },
    { 0x06, 0x7dff },
    { 0x06, 0xff0d },
    { 0x06, 0x581c },
    { 0x06, 0x551a },
    { 0x06, 0x6511 },
    { 0x06, 0xa190 },
    { 0x06, 0xd3e2 },
    { 0x06, 0x8348 },
    { 0x06, 0xe383 },
    { 0x06, 0x491b },
    { 0x06, 0x56ab },
    { 0x06, 0x08ef },
    { 0x06, 0x56e6 },
    { 0x06, 0x8348 },
    { 0x06, 0xe783 },
    { 0x06, 0x4910 },
    { 0x06, 0xd180 },
    { 0x06, 0x1f66 },
    { 0x06, 0xa004 },
    { 0x06, 0xb9e2 },
    { 0x06, 0x8348 },
    { 0x06, 0xe383 },
    { 0x06, 0x49ef },
    { 0x06, 0x65e2 },
    { 0x06, 0x834a },
    { 0x06, 0xe383 },
    { 0x06, 0x4b1b },
    { 0x06, 0x56aa },
    { 0x06, 0x0eef },
    { 0x06, 0x56e6 },
    { 0x06, 0x834a },
    { 0x06, 0xe783 },
    { 0x06, 0x4be2 },
    { 0x06, 0x834d },
    { 0x06, 0xe683 },
    { 0x06, 0x4ce0 },
    { 0x06, 0x834d },
    { 0x06, 0xa000 },
    { 0x06, 0x0caf },
    { 0x06, 0x81dc },
    { 0x06, 0xe083 },
    { 0x06, 0x4d10 },
    { 0x06, 0xe483 },
    { 0x06, 0x4dae },
    { 0x06, 0x0480 },
    { 0x06, 0xe483 },
    { 0x06, 0x4de0 },
    { 0x06, 0x834e },
    { 0x06, 0x7803 },
    { 0x06, 0x9e0b },
    { 0x06, 0xe083 },
    { 0x06, 0x4e78 },
    { 0x06, 0x049e },
    { 0x06, 0x04ee },
    { 0x06, 0x834e },
    { 0x06, 0x02e0 },
    { 0x06, 0x8332 },
    { 0x06, 0xe183 },
    { 0x06, 0x3359 },
    { 0x06, 0x0fe2 },
    { 0x06, 0x834d },
    { 0x06, 0x0c24 },
    { 0x06, 0x5af0 },
    { 0x06, 0x1e12 },
    { 0x06, 0xe4f8 },
    { 0x06, 0x8ce5 },
    { 0x06, 0xf88d },
    { 0x06, 0xe083 },
    { 0x06, 0x30e1 },
    { 0x06, 0x8331 },
    { 0x06, 0x6801 },
    { 0x06, 0xe4f8 },
    { 0x06, 0x8ae5 },
    { 0x06, 0xf88b },
    { 0x06, 0xae37 },
    { 0x06, 0xee83 },
    { 0x06, 0x4e03 },
    { 0x06, 0xe083 },
    { 0x06, 0x4ce1 },
    { 0x06, 0x834d },
    { 0x06, 0x1b01 },
    { 0x06, 0x9e04 },
    { 0x06, 0xaaa1 },
    { 0x06, 0xaea8 },
    { 0x06, 0xee83 },
    { 0x06, 0x4e04 },
    { 0x06, 0xee83 },
    { 0x06, 0x4f00 },
    { 0x06, 0xaeab },
    { 0x06, 0xe083 },
    { 0x06, 0x4f78 },
    { 0x06, 0x039f },
    { 0x06, 0x14ee },
    { 0x06, 0x834e },
    { 0x06, 0x05d2 },
    { 0x06, 0x40d6 },
    { 0x06, 0x5554 },
    { 0x06, 0x0282 },
    { 0x06, 0x17d2 },
    { 0x06, 0xa0d6 },
    { 0x06, 0xba00 },
    { 0x06, 0x0282 },
    { 0x06, 0x17fe },
    { 0x06, 0xfdfc },
    { 0x06, 0x05f8 },
    { 0x06, 0xe0f8 },
    { 0x06, 0x60e1 },
    { 0x06, 0xf861 },
    { 0x06, 0x6802 },
    { 0x06, 0xe4f8 },
    { 0x06, 0x60e5 },
    { 0x06, 0xf861 },
    { 0x06, 0xe0f8 },
    { 0x06, 0x48e1 },
    { 0x06, 0xf849 },
    { 0x06, 0x580f },
    { 0x06, 0x1e02 },
    { 0x06, 0xe4f8 },
    { 0x06, 0x48e5 },
    { 0x06, 0xf849 },
    { 0x06, 0xd000 },
    { 0x06, 0x0282 },
    { 0x06, 0x5bbf },
    { 0x06, 0x8350 },
    { 0x06, 0xef46 },
    { 0x06, 0xdc19 },
    { 0x06, 0xddd0 },
    { 0x06, 0x0102 },
    { 0x06, 0x825b },
    { 0x06, 0x0282 },
    { 0x06, 0x77e0 },
    { 0x06, 0xf860 },
    { 0x06, 0xe1f8 },
    { 0x06, 0x6158 },
    { 0x06, 0xfde4 },
    { 0x06, 0xf860 },
    { 0x06, 0xe5f8 },
    { 0x06, 0x61fc },
    { 0x06, 0x04f9 },
    { 0x06, 0xfafb },
    { 0x06, 0xc6bf },
    { 0x06, 0xf840 },
    { 0x06, 0xbe83 },
    { 0x06, 0x50a0 },
    { 0x06, 0x0101 },
    { 0x06, 0x071b },
    { 0x06, 0x89cf },
    { 0x06, 0xd208 },
    { 0x06, 0xebdb },
    { 0x06, 0x19b2 },
    { 0x06, 0xfbff },
    { 0x06, 0xfefd },
    { 0x06, 0x04f8 },
    { 0x06, 0xe0f8 },
    { 0x06, 0x48e1 },
    { 0x06, 0xf849 },
    { 0x06, 0x6808 },
    { 0x06, 0xe4f8 },
    { 0x06, 0x48e5 },
    { 0x06, 0xf849 },
    { 0x06, 0x58f7 },
    { 0x06, 0xe4f8 },
    { 0x06, 0x48e5 },
    { 0x06, 0xf849 },
    { 0x06, 0xfc04 },
    { 0x06, 0x4d20 },
    { 0x06, 0x0002 },
    { 0x06, 0x4e22 },
    { 0x06, 0x0002 },
    { 0x06, 0x4ddf },
    { 0x06, 0xff01 },
    { 0x06, 0x4edd },
    { 0x06, 0xff01 },
    { 0x05, 0x83d4 },
    { 0x06, 0x8000 },
    { 0x05, 0x83d8 },
    { 0x06, 0x8051 },
    { 0x02, 0x6010 },
    { 0x03, 0xdc00 },
    { 0x05, 0xfff6 },
    { 0x06, 0x00fc },
    { 0x1f, 0x0000 },

    { 0x1f, 0x0000 },
    { 0x0d, 0xf880 },
    { 0x1f, 0x0000 }
  };

  rtl_phy_write(ioaddr, phy_reg_init_0, ARRAY_SIZE(phy_reg_init_0));

  mdio_write(ioaddr, 0x1f, 0x0002);
  mdio_plus_minus(ioaddr, 0x0b, 0x0010, 0x00ef);
  mdio_plus_minus(ioaddr, 0x0c, 0xa200, 0x5d00);

  rtl_phy_write(ioaddr, phy_reg_init_1, ARRAY_SIZE(phy_reg_init_1));

  if (rtl8168d_efuse_read(ioaddr, 0x01) == 0xb1) {
    static const struct phy_reg phy_reg_init[] = {
      { 0x1f, 0x0002 },
      { 0x05, 0x669a },
      { 0x1f, 0x0005 },
      { 0x05, 0x8330 },
      { 0x06, 0x669a },
      { 0x1f, 0x0002 }
    };
    int val;

    rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));

    val = mdio_read(ioaddr, 0x0d);

    if ((val & 0x00ff) != 0x006c) {
      static const u32 set[] = {
        0x0065, 0x0066, 0x0067, 0x0068,
        0x0069, 0x006a, 0x006b, 0x006c
      };
      int i;

      mdio_write(ioaddr, 0x1f, 0x0002);

      val &= 0xff00;
      for (i = 0; i < ARRAY_SIZE(set); i++)
        mdio_write(ioaddr, 0x0d, val | set[i]);
    }
  } else {
    static const struct phy_reg phy_reg_init[] = {
      { 0x1f, 0x0002 },
      { 0x05, 0x6662 },
      { 0x1f, 0x0005 },
      { 0x05, 0x8330 },
      { 0x06, 0x6662 }
    };

    rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));
  }

  mdio_write(ioaddr, 0x1f, 0x0002);
  mdio_patch(ioaddr, 0x0d, 0x0300);
  mdio_patch(ioaddr, 0x0f, 0x0010);

  mdio_write(ioaddr, 0x1f, 0x0002);
  mdio_plus_minus(ioaddr, 0x02, 0x0100, 0x0600);
  mdio_plus_minus(ioaddr, 0x03, 0x0000, 0xe000);

  rtl_phy_write(ioaddr, phy_reg_init_2, ARRAY_SIZE(phy_reg_init_2));
}

static void rtl8168d_2_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init_0[] = {
    { 0x1f, 0x0001 },
    { 0x06, 0x4064 },
    { 0x07, 0x2863 },
    { 0x08, 0x059c },
    { 0x09, 0x26b4 },
    { 0x0a, 0x6a19 },
    { 0x0b, 0xdcc8 },
    { 0x10, 0xf06d },
    { 0x14, 0x7f68 },
    { 0x18, 0x7fd9 },
    { 0x1c, 0xf0ff },
    { 0x1d, 0x3d9c },
    { 0x1f, 0x0003 },
    { 0x12, 0xf49f },
    { 0x13, 0x070b },
    { 0x1a, 0x05ad },
    { 0x14, 0x94c0 },

    { 0x1f, 0x0002 },
    { 0x06, 0x5561 },
    { 0x1f, 0x0005 },
    { 0x05, 0x8332 },
    { 0x06, 0x5561 }
  };
  static const struct phy_reg phy_reg_init_1[] = {
    { 0x1f, 0x0005 },
    { 0x05, 0xffc2 },
    { 0x1f, 0x0005 },
    { 0x05, 0x8000 },
    { 0x06, 0xf8f9 },
    { 0x06, 0xfaee },
    { 0x06, 0xf8ea },
    { 0x06, 0x00ee },
    { 0x06, 0xf8eb },
    { 0x06, 0x00e2 },
    { 0x06, 0xf87c },
    { 0x06, 0xe3f8 },
    { 0x06, 0x7da5 },
    { 0x06, 0x1111 },
    { 0x06, 0x12d2 },
    { 0x06, 0x40d6 },
    { 0x06, 0x4444 },
    { 0x06, 0x0281 },
    { 0x06, 0xc6d2 },
    { 0x06, 0xa0d6 },
    { 0x06, 0xaaaa },
    { 0x06, 0x0281 },
    { 0x06, 0xc6ae },
    { 0x06, 0x0fa5 },
    { 0x06, 0x4444 },
    { 0x06, 0x02ae },
    { 0x06, 0x4da5 },
    { 0x06, 0xaaaa },
    { 0x06, 0x02ae },
    { 0x06, 0x47af },
    { 0x06, 0x81c2 },
    { 0x06, 0xee83 },
    { 0x06, 0x4e00 },
    { 0x06, 0xee83 },
    { 0x06, 0x4d0f },
    { 0x06, 0xee83 },
    { 0x06, 0x4c0f },
    { 0x06, 0xee83 },
    { 0x06, 0x4f00 },
    { 0x06, 0xee83 },
    { 0x06, 0x5100 },
    { 0x06, 0xee83 },
    { 0x06, 0x4aff },
    { 0x06, 0xee83 },
    { 0x06, 0x4bff },
    { 0x06, 0xe083 },
    { 0x06, 0x30e1 },
    { 0x06, 0x8331 },
    { 0x06, 0x58fe },
    { 0x06, 0xe4f8 },
    { 0x06, 0x8ae5 },
    { 0x06, 0xf88b },
    { 0x06, 0xe083 },
    { 0x06, 0x32e1 },
    { 0x06, 0x8333 },
    { 0x06, 0x590f },
    { 0x06, 0xe283 },
    { 0x06, 0x4d0c },
    { 0x06, 0x245a },
    { 0x06, 0xf01e },
    { 0x06, 0x12e4 },
    { 0x06, 0xf88c },
    { 0x06, 0xe5f8 },
    { 0x06, 0x8daf },
    { 0x06, 0x81c2 },
    { 0x06, 0xe083 },
    { 0x06, 0x4f10 },
    { 0x06, 0xe483 },
    { 0x06, 0x4fe0 },
    { 0x06, 0x834e },
    { 0x06, 0x7800 },
    { 0x06, 0x9f0a },
    { 0x06, 0xe083 },
    { 0x06, 0x4fa0 },
    { 0x06, 0x10a5 },
    { 0x06, 0xee83 },
    { 0x06, 0x4e01 },
    { 0x06, 0xe083 },
    { 0x06, 0x4e78 },
    { 0x06, 0x059e },
    { 0x06, 0x9ae0 },
    { 0x06, 0x834e },
    { 0x06, 0x7804 },
    { 0x06, 0x9e10 },
    { 0x06, 0xe083 },
    { 0x06, 0x4e78 },
    { 0x06, 0x039e },
    { 0x06, 0x0fe0 },
    { 0x06, 0x834e },
    { 0x06, 0x7801 },
    { 0x06, 0x9e05 },
    { 0x06, 0xae0c },
    { 0x06, 0xaf81 },
    { 0x06, 0xa7af },
    { 0x06, 0x8152 },
    { 0x06, 0xaf81 },
    { 0x06, 0x8baf },
    { 0x06, 0x81c2 },
    { 0x06, 0xee83 },
    { 0x06, 0x4800 },
    { 0x06, 0xee83 },
    { 0x06, 0x4900 },
    { 0x06, 0xe083 },
    { 0x06, 0x5110 },
    { 0x06, 0xe483 },
    { 0x06, 0x5158 },
    { 0x06, 0x019f },
    { 0x06, 0xead0 },
    { 0x06, 0x00d1 },
    { 0x06, 0x801f },
    { 0x06, 0x66e2 },
    { 0x06, 0xf8ea },
    { 0x06, 0xe3f8 },
    { 0x06, 0xeb5a },
    { 0x06, 0xf81e },
    { 0x06, 0x20e6 },
    { 0x06, 0xf8ea },
    { 0x06, 0xe5f8 },
    { 0x06, 0xebd3 },
    { 0x06, 0x02b3 },
    { 0x06, 0xfee2 },
    { 0x06, 0xf87c },
    { 0x06, 0xef32 },
    { 0x06, 0x5b80 },
    { 0x06, 0xe3f8 },
    { 0x06, 0x7d9e },
    { 0x06, 0x037d },
    { 0x06, 0xffff },
    { 0x06, 0x0d58 },
    { 0x06, 0x1c55 },
    { 0x06, 0x1a65 },
    { 0x06, 0x11a1 },
    { 0x06, 0x90d3 },
    { 0x06, 0xe283 },
    { 0x06, 0x48e3 },
    { 0x06, 0x8349 },
    { 0x06, 0x1b56 },
    { 0x06, 0xab08 },
    { 0x06, 0xef56 },
    { 0x06, 0xe683 },
    { 0x06, 0x48e7 },
    { 0x06, 0x8349 },
    { 0x06, 0x10d1 },
    { 0x06, 0x801f },
    { 0x06, 0x66a0 },
    { 0x06, 0x04b9 },
    { 0x06, 0xe283 },
    { 0x06, 0x48e3 },
    { 0x06, 0x8349 },
    { 0x06, 0xef65 },
    { 0x06, 0xe283 },
    { 0x06, 0x4ae3 },
    { 0x06, 0x834b },
    { 0x06, 0x1b56 },
    { 0x06, 0xaa0e },
    { 0x06, 0xef56 },
    { 0x06, 0xe683 },
    { 0x06, 0x4ae7 },
    { 0x06, 0x834b },
    { 0x06, 0xe283 },
    { 0x06, 0x4de6 },
    { 0x06, 0x834c },
    { 0x06, 0xe083 },
    { 0x06, 0x4da0 },
    { 0x06, 0x000c },
    { 0x06, 0xaf81 },
    { 0x06, 0x8be0 },
    { 0x06, 0x834d },
    { 0x06, 0x10e4 },
    { 0x06, 0x834d },
    { 0x06, 0xae04 },
    { 0x06, 0x80e4 },
    { 0x06, 0x834d },
    { 0x06, 0xe083 },
    { 0x06, 0x4e78 },
    { 0x06, 0x039e },
    { 0x06, 0x0be0 },
    { 0x06, 0x834e },
    { 0x06, 0x7804 },
    { 0x06, 0x9e04 },
    { 0x06, 0xee83 },
    { 0x06, 0x4e02 },
    { 0x06, 0xe083 },
    { 0x06, 0x32e1 },
    { 0x06, 0x8333 },
    { 0x06, 0x590f },
    { 0x06, 0xe283 },
    { 0x06, 0x4d0c },
    { 0x06, 0x245a },
    { 0x06, 0xf01e },
    { 0x06, 0x12e4 },
    { 0x06, 0xf88c },
    { 0x06, 0xe5f8 },
    { 0x06, 0x8de0 },
    { 0x06, 0x8330 },
    { 0x06, 0xe183 },
    { 0x06, 0x3168 },
    { 0x06, 0x01e4 },
    { 0x06, 0xf88a },
    { 0x06, 0xe5f8 },
    { 0x06, 0x8bae },
    { 0x06, 0x37ee },
    { 0x06, 0x834e },
    { 0x06, 0x03e0 },
    { 0x06, 0x834c },
    { 0x06, 0xe183 },
    { 0x06, 0x4d1b },
    { 0x06, 0x019e },
    { 0x06, 0x04aa },
    { 0x06, 0xa1ae },
    { 0x06, 0xa8ee },
    { 0x06, 0x834e },
    { 0x06, 0x04ee },
    { 0x06, 0x834f },
    { 0x06, 0x00ae },
    { 0x06, 0xabe0 },
    { 0x06, 0x834f },
    { 0x06, 0x7803 },
    { 0x06, 0x9f14 },
    { 0x06, 0xee83 },
    { 0x06, 0x4e05 },
    { 0x06, 0xd240 },
    { 0x06, 0xd655 },
    { 0x06, 0x5402 },
    { 0x06, 0x81c6 },
    { 0x06, 0xd2a0 },
    { 0x06, 0xd6ba },
    { 0x06, 0x0002 },
    { 0x06, 0x81c6 },
    { 0x06, 0xfefd },
    { 0x06, 0xfc05 },
    { 0x06, 0xf8e0 },
    { 0x06, 0xf860 },
    { 0x06, 0xe1f8 },
    { 0x06, 0x6168 },
    { 0x06, 0x02e4 },
    { 0x06, 0xf860 },
    { 0x06, 0xe5f8 },
    { 0x06, 0x61e0 },
    { 0x06, 0xf848 },
    { 0x06, 0xe1f8 },
    { 0x06, 0x4958 },
    { 0x06, 0x0f1e },
    { 0x06, 0x02e4 },
    { 0x06, 0xf848 },
    { 0x06, 0xe5f8 },
    { 0x06, 0x49d0 },
    { 0x06, 0x0002 },
    { 0x06, 0x820a },
    { 0x06, 0xbf83 },
    { 0x06, 0x50ef },
    { 0x06, 0x46dc },
    { 0x06, 0x19dd },
    { 0x06, 0xd001 },
    { 0x06, 0x0282 },
    { 0x06, 0x0a02 },
    { 0x06, 0x8226 },
    { 0x06, 0xe0f8 },
    { 0x06, 0x60e1 },
    { 0x06, 0xf861 },
    { 0x06, 0x58fd },
    { 0x06, 0xe4f8 },
    { 0x06, 0x60e5 },
    { 0x06, 0xf861 },
    { 0x06, 0xfc04 },
    { 0x06, 0xf9fa },
    { 0x06, 0xfbc6 },
    { 0x06, 0xbff8 },
    { 0x06, 0x40be },
    { 0x06, 0x8350 },
    { 0x06, 0xa001 },
    { 0x06, 0x0107 },
    { 0x06, 0x1b89 },
    { 0x06, 0xcfd2 },
    { 0x06, 0x08eb },
    { 0x06, 0xdb19 },
    { 0x06, 0xb2fb },
    { 0x06, 0xfffe },
    { 0x06, 0xfd04 },
    { 0x06, 0xf8e0 },
    { 0x06, 0xf848 },
    { 0x06, 0xe1f8 },
    { 0x06, 0x4968 },
    { 0x06, 0x08e4 },
    { 0x06, 0xf848 },
    { 0x06, 0xe5f8 },
    { 0x06, 0x4958 },
    { 0x06, 0xf7e4 },
    { 0x06, 0xf848 },
    { 0x06, 0xe5f8 },
    { 0x06, 0x49fc },
    { 0x06, 0x044d },
    { 0x06, 0x2000 },
    { 0x06, 0x024e },
    { 0x06, 0x2200 },
    { 0x06, 0x024d },
    { 0x06, 0xdfff },
    { 0x06, 0x014e },
    { 0x06, 0xddff },
    { 0x06, 0x0100 },
    { 0x05, 0x83d8 },
    { 0x06, 0x8000 },
    { 0x03, 0xdc00 },
    { 0x05, 0xfff6 },
    { 0x06, 0x00fc },
    { 0x1f, 0x0000 },

    { 0x1f, 0x0000 },
    { 0x0d, 0xf880 },
    { 0x1f, 0x0000 }
  };

  rtl_phy_write(ioaddr, phy_reg_init_0, ARRAY_SIZE(phy_reg_init_0));

  if (rtl8168d_efuse_read(ioaddr, 0x01) == 0xb1) {
    static const struct phy_reg phy_reg_init[] = {
      { 0x1f, 0x0002 },
      { 0x05, 0x669a },
      { 0x1f, 0x0005 },
      { 0x05, 0x8330 },
      { 0x06, 0x669a },

      { 0x1f, 0x0002 }
    };
    int val;

    rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));

    val = mdio_read(ioaddr, 0x0d);
    if ((val & 0x00ff) != 0x006c) {
      u32 set[] = {
        0x0065, 0x0066, 0x0067, 0x0068,
        0x0069, 0x006a, 0x006b, 0x006c
      };
      int i;

      mdio_write(ioaddr, 0x1f, 0x0002);

      val &= 0xff00;
      for (i = 0; i < ARRAY_SIZE(set); i++)
        mdio_write(ioaddr, 0x0d, val | set[i]);
    }
  } else {
    static const struct phy_reg phy_reg_init[] = {
      { 0x1f, 0x0002 },
      { 0x05, 0x2642 },
      { 0x1f, 0x0005 },
      { 0x05, 0x8330 },
      { 0x06, 0x2642 }
    };

    rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));
  }

  mdio_write(ioaddr, 0x1f, 0x0002);
  mdio_plus_minus(ioaddr, 0x02, 0x0100, 0x0600);
  mdio_plus_minus(ioaddr, 0x03, 0x0000, 0xe000);

  mdio_write(ioaddr, 0x1f, 0x0001);
  mdio_write(ioaddr, 0x17, 0x0cc0);

  mdio_write(ioaddr, 0x1f, 0x0002);
  mdio_patch(ioaddr, 0x0f, 0x0017);

  rtl_phy_write(ioaddr, phy_reg_init_1, ARRAY_SIZE(phy_reg_init_1));
}

static void rtl8168d_3_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init[] = {
    { 0x1f, 0x0002 },
    { 0x10, 0x0008 },
    { 0x0d, 0x006c },

    { 0x1f, 0x0000 },
    { 0x0d, 0xf880 },

    { 0x1f, 0x0001 },
    { 0x17, 0x0cc0 },

    { 0x1f, 0x0001 },
    { 0x0b, 0xa4d8 },
    { 0x09, 0x281c },
    { 0x07, 0x2883 },
    { 0x0a, 0x6b35 },
    { 0x1d, 0x3da4 },
    { 0x1c, 0xeffd },
    { 0x14, 0x7f52 },
    { 0x18, 0x7fc6 },
    { 0x08, 0x0601 },
    { 0x06, 0x4063 },
    { 0x10, 0xf074 },
    { 0x1f, 0x0003 },
    { 0x13, 0x0789 },
    { 0x12, 0xf4bd },
    { 0x1a, 0x04fd },
    { 0x14, 0x84b0 },
    { 0x1f, 0x0000 },
    { 0x00, 0x9200 },

    { 0x1f, 0x0005 },
    { 0x01, 0x0340 },
    { 0x1f, 0x0001 },
    { 0x04, 0x4000 },
    { 0x03, 0x1d21 },
    { 0x02, 0x0c32 },
    { 0x01, 0x0200 },
    { 0x00, 0x5554 },
    { 0x04, 0x4800 },
    { 0x04, 0x4000 },
    { 0x04, 0xf000 },
    { 0x03, 0xdf01 },
    { 0x02, 0xdf20 },
    { 0x01, 0x101a },
    { 0x00, 0xa0ff },
    { 0x04, 0xf800 },
    { 0x04, 0xf000 },
    { 0x1f, 0x0000 },

    { 0x1f, 0x0007 },
    { 0x1e, 0x0023 },
    { 0x16, 0x0000 },
    { 0x1f, 0x0000 }
  };

  rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));
}

static void rtl8102e_hw_phy_config(void __iomem *ioaddr)
{
  static const struct phy_reg phy_reg_init[] = {
    { 0x1f, 0x0003 },
    { 0x08, 0x441d },
    { 0x01, 0x9100 },
    { 0x1f, 0x0000 }
  };

  mdio_write(ioaddr, 0x1f, 0x0000);
  mdio_patch(ioaddr, 0x11, 1 << 12);
  mdio_patch(ioaddr, 0x19, 1 << 13);
  mdio_patch(ioaddr, 0x10, 1 << 15);

  rtl_phy_write(ioaddr, phy_reg_init, ARRAY_SIZE(phy_reg_init));
}

static void rtl_hw_phy_config(struct net_device *dev)
{
  struct rtl8169_private *tp = netdev_priv(dev);
  void __iomem *ioaddr = tp->mmio_addr;

  switch (tp->mac_version) {
  case RTL_GIGA_MAC_VER_01:
    break;
  case RTL_GIGA_MAC_VER_02:
  case RTL_GIGA_MAC_VER_03:
    rtl8169s_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_04:
    rtl8169sb_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_05:
    rtl8169scd_hw_phy_config(tp, ioaddr);
    break;
  case RTL_GIGA_MAC_VER_06:
    rtl8169sce_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_07:
  case RTL_GIGA_MAC_VER_08:
  case RTL_GIGA_MAC_VER_09:
    rtl8102e_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_11:
    rtl8168bb_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_12:
    rtl8168bef_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_17:
    rtl8168bef_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_18:
    rtl8168cp_1_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_19:
    rtl8168c_1_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_20:
    rtl8168c_2_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_21:
    rtl8168c_3_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_22:
    rtl8168c_4_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_23:
  case RTL_GIGA_MAC_VER_24:
    rtl8168cp_2_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_25:
    rtl8168d_1_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_26:
    rtl8168d_2_hw_phy_config(ioaddr);
    break;
  case RTL_GIGA_MAC_VER_27:
    rtl8168d_3_hw_phy_config(ioaddr);
    break;

  default:
    break;
  }
}

static int
rtl8169_set_speed(struct net_device *dev,
                  u8 autoneg, u16 speed, u8 duplex)
{
  struct rtl8169_private *tp = netdev_priv(dev);
  int ret;

  ret = tp->set_speed(dev, autoneg, speed, duplex);

  /***************************************************************************
   * if (netif_running(dev) && (tp->phy_1000_ctrl_reg & ADVERTISE_1000FULL)) *
   *   mod_timer(&tp->timer, jiffies + RTL8169_PHY_TIMEOUT);                 *
   ***************************************************************************/

  return ret;
}

static void
rtl8169_phy_reset(struct net_device *dev,
                  struct rtl8169_private *tp)
{
  void __iomem *ioaddr = tp->mmio_addr;
  unsigned int i;

  tp->phy_reset_enable(ioaddr);
  for (i = 0; i < 100; i++) {
    if (!tp->phy_reset_pending(ioaddr))
      return;
    udelay(1000);
  }
  DLOG("PHY reset failed");
}

static void
init_phy(struct net_device *dev, struct rtl8169_private *tp)
{
  void __iomem *ioaddr = tp->mmio_addr;

  rtl_hw_phy_config(dev);

  if (tp->mac_version <= RTL_GIGA_MAC_VER_06) {
    DLOG("Set MAC Reg C+CR Offset 0x82h = 0x01h");
    RTL_W8(0x82, 0x01);
  }

  pci_write_config_byte(tp->pci_dev, PCI_LATENCY_TIMER, 0x40);

  if (tp->mac_version <= RTL_GIGA_MAC_VER_06)
    pci_write_config_byte(tp->pci_dev, PCI_CACHE_LINE_SIZE, 0x08);

  if (tp->mac_version == RTL_GIGA_MAC_VER_02) {
    DLOG("Set MAC Reg C+CR Offset 0x82h = 0x01h");
    RTL_W8(0x82, 0x01);
    DLOG("Set PHY Reg 0x0bh = 0x00h");
    mdio_write(ioaddr, 0x0b, 0x0000); //w 0x0b 15 0 0
  }

  rtl8169_phy_reset(dev, tp);

  /*
   * rtl8169_set_speed_xmii takes good care of the Fast Ethernet
   * only 8101. Don't panic.
   */
  rtl8169_set_speed(dev, AUTONEG_ENABLE, SPEED_1000, DUPLEX_FULL);

  if (RTL_R8(PHYstatus) & TBI_Enable)
    DLOG("TBI auto-negotiating");
}

#define cpu_to_le32 __cpu_to_le32
#define le32_to_cpu __le32_to_cpu
#define cpu_to_le64 __cpu_to_le64
#define wmb() ;

static inline struct sk_buff *
alloc_skb (u32 size)
{
  struct sk_buff *skb;
  pow2_alloc (sizeof (struct sk_buff), (u8 **) &skb);
  if (!skb) return NULL;
  skb->len = size;
  pow2_alloc (size, (u8 **) &skb->data);
  if (!skb->data) return NULL;
  memset (skb->data, 0, size);
  return skb;
}

static inline void
free_skb (struct sk_buff *skb)
{
  pow2_free ((u8 *)skb->data);
  pow2_free ((u8 *)skb);
}

static inline void rtl8169_make_unusable_by_asic(struct RxDesc *desc)
{
  desc->addr = cpu_to_le64(0x0badbadbadbadbadull);
  desc->opts1 &= ~cpu_to_le32(DescOwn | RsvdMask);
}

static void rtl8169_free_rx_skb(struct rtl8169_private *tp,
                                struct sk_buff **sk_buff, struct RxDesc *desc)
{
  free_skb (*sk_buff);
  *sk_buff = NULL;
  rtl8169_make_unusable_by_asic(desc);
}

static inline void rtl8169_mark_to_asic(struct RxDesc *desc, u32 rx_buf_sz)
{
  u32 eor = le32_to_cpu(desc->opts1) & RingEnd;

  desc->opts1 = cpu_to_le32(DescOwn | eor | rx_buf_sz);
}

static inline void rtl8169_map_to_asic(struct RxDesc *desc, dma_addr_t mapping,
                                       u32 rx_buf_sz)
{
  desc->addr = cpu_to_le64(mapping);
  wmb();
  rtl8169_mark_to_asic(desc, rx_buf_sz);
}

static void rtl8169_rx_clear(struct rtl8169_private *tp)
{
  unsigned int i;

  for (i = 0; i < NUM_RX_DESC; i++) {
    if (tp->Rx_skbuff[i]) {
      rtl8169_free_rx_skb(tp, tp->Rx_skbuff + i,
                          tp->RxDescArray + i);
    }
  }
}

typedef int gfp_t;

static struct sk_buff *rtl8169_alloc_rx_skb(struct pci_dev *pdev,
                                            struct net_device *dev,
                                            struct RxDesc *desc, int rx_buf_sz,
                                            unsigned int align, gfp_t gfp)
{
  struct sk_buff *skb;
  dma_addr_t mapping;
  unsigned int pad;

#define NET_IP_ALIGN 2
  pad = align ? align : NET_IP_ALIGN;

  skb = alloc_skb (rx_buf_sz + pad);
  if (!skb)
    goto err_out;

  mapping = (uint) get_phys_addr (skb->data);

  rtl8169_map_to_asic(desc, mapping, rx_buf_sz);
 out:
  return skb;

 err_out:
  rtl8169_make_unusable_by_asic(desc);
  goto out;
}

static u32 rtl8169_rx_fill(struct rtl8169_private *tp, struct net_device *dev,
                           u32 start, u32 end, gfp_t gfp)
{
  u32 cur;

  for (cur = start; end - cur != 0; cur++) {
    struct sk_buff *skb;
    unsigned int i = cur % NUM_RX_DESC;

    if (tp->Rx_skbuff[i])
      continue;

    skb = rtl8169_alloc_rx_skb(tp->pci_dev, dev,
                               tp->RxDescArray + i,
                               tp->rx_buf_sz, tp->align, gfp);
    if (!skb)
      break;

    tp->Rx_skbuff[i] = skb;
  }
  return cur - start;
}

static inline void rtl8169_mark_as_last_descriptor(struct RxDesc *desc)
{
  desc->opts1 |= cpu_to_le32(RingEnd);
}

static void rtl8169_init_ring_indexes(struct rtl8169_private *tp)
{
  int i = 0;
  tp->dirty_tx = tp->dirty_rx = tp->cur_tx = 0;
  for (i = 0; i < MAX_NUM_SHARE; i++) {
    tp->cur_rx[i] = 0;
  }
}

static int rtl8169_init_ring(struct net_device *dev)
{
  struct rtl8169_private *tp = netdev_priv(dev);

  rtl8169_init_ring_indexes(tp);

  memset(tp->tx_skb, 0x0, NUM_TX_DESC * sizeof(struct ring_info));

  memset(tp->Rx_skbuff, 0x0, NUM_RX_DESC * sizeof(struct sk_buff *));

  if (rtl8169_rx_fill(tp, dev, 0, NUM_RX_DESC, 0) != NUM_RX_DESC)
    goto err_out;

  rtl8169_mark_as_last_descriptor(tp->RxDescArray + NUM_RX_DESC - 1);

  return 0;

 err_out:
  rtl8169_rx_clear(tp);
  return -1;
}

static void rtl_hw_start(struct net_device *dev)
{
  struct rtl8169_private *tp = netdev_priv(dev);
  void __iomem *ioaddr = tp->mmio_addr;
  unsigned int i;

  /* Soft reset the chip. */
  RTL_W8(ChipCmd, CmdReset);

  /* Check that the chip has finished the reset. */
  for (i = 0; i < 100; i++) {
    if ((RTL_R8(ChipCmd) & CmdReset) == 0)
      break;
    udelay(1000);
  }

  tp->hw_start(dev);
}

uint32 alloc_skb_count = 0;
static sint
r8169_transmit (u8 *buffer, sint len)
{
  DLOG ("TX: buffer=0x%p len=%d, cpu=%d", buffer, len, get_pcpu_id ());
  u8 *p = buffer;
  DLOG ("  %.02X %.02X %.02X %.02X %.02X %.02X",
        p[0], p[1], p[2], p[3], p[4], p[5]);
  p+=6;
  DLOG ("  %.02X %.02X %.02X %.02X %.02X %.02X",
        p[0], p[1], p[2], p[3], p[4], p[5]);

#ifdef USE_VMX
  spinlock_lock (r8169_tx_lock);
#endif

  uint entry = tp->cur_tx % NUM_TX_DESC;
  struct TxDesc *txd = tp->TxDescArray + entry;
  void __iomem *ioaddr = tp->mmio_addr;
  dma_addr_t mapping;
  u32 status;
  u32 opts1;

  if (le32_to_cpu (txd->opts1) & DescOwn) {
    com1_printf ("TX ring overflow\n");
    goto abort;
  }
  if (len > MAX_FRAME_SIZE) {
    com1_printf ("TX frame too large\n");
    goto abort;
  }

  opts1 = DescOwn | FirstFrag | LastFrag;
  tp->tx_skb[entry].len = len;
  struct sk_buff *skb = alloc_skb (len);
  if (!skb) {com1_printf ("skb alloc failed!\n"); goto abort;}
  alloc_skb_count++;
  memcpy (skb->data, buffer, len);
  tp->tx_skb[entry].skb = skb;
  mapping = (uint) get_phys_addr (skb->data);
  txd->addr = __cpu_to_le64 (mapping);
  status = opts1 | len | (RingEnd * !((entry + 1) % NUM_TX_DESC));
  DLOG ("  cur_tx=%d mapping=0x%p status=0x%p", tp->cur_tx, mapping, status);

#ifdef TX_TIMING
  SERIALIZE0;
  RDTSC (tx_start);
#endif

  /* xmit */
  txd->opts2 = 0;
  txd->opts1 = __cpu_to_le32 (status);

  tp->cur_tx++;

  RTL_W8 (TxPoll, NPQ); /* set polling bit */

#ifdef USE_VMX
  spinlock_unlock (r8169_tx_lock);
#endif
  return len;
 abort:
#ifdef USE_VMX
  spinlock_unlock (r8169_tx_lock);
#endif
  return -1;
}

#ifdef TX_TIMING
static u32 timing_stack[1024] ALIGNED (0x1000);
static task_id timing_id;
static void timing_thread (void) {
  struct udp_pcb *pcb = udp_new ();
  struct pbuf *p = pbuf_alloc (PBUF_TRANSPORT, 4, PBUF_RAM);
  struct ip_addr ip;
  struct in_addr inaddr;
  inet_aton ("192.168.2.123", &inaddr); /* made up address */
  ip.addr = inaddr.s_addr;
  pbuf_take (p, "TEST", 4);
  logger_printf ("r8169: timing_thread: id=0x%x\n", timing_id);
  for (;;) {
    sched_usleep (1000000);
    udp_sendto (pcb, p, &ip, 7890);
  }
}
#endif

static void
r8169_poll (void)
{
  rx_int (tp);
  tx_int (tp);
}

static bool
r8169_get_hwaddr (u8 addr[ETH_ADDR_LEN])
{
  int i;
  for (i=0; i<ETH_ADDR_LEN; i++)
    addr[i] = tp->mac_addr[i];
  return TRUE;
}

extern void
r8169_reset (void)
{
  void * ioaddr = global_ioa_backup;
  RTL_W8(ChipCmd, CmdReset);
}

void
r8169_route_irq (uint8 dest)
{
  u8 vector;
  if (!pci_irq_map_handler (&irq_backup, irq_handler, dest,
        IOAPIC_DESTINATION_LOGICAL,
        IOAPIC_DELIVERY_FIXED)) {
    DLOG ("Failed to map IRQ");
    return;
  }

  IOAPIC_get_GSI_mapping (irq_backup.gsi, &vector, NULL);
  DLOG ("Using vector=0x%X", vector);
}

extern bool
r8169_init (void)
{
  int i;
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

  memset (r8169_bh_id, 0, MAX_NUM_SHARE * sizeof (task_id));

  const struct rtl_cfg_info *cfg = &rtl_cfg_infos[compatible_ids[i].cfg];
  const unsigned int region = cfg->region;
  DLOG ("Found device_index=%d", device_index);

  if (!pci_get_device (device_index, &pdev)) {
    DLOG ("pci_get_device");
    goto abort;
  }

  u32 phys_addr; void *ioaddr;
  pci_decode_bar (pdev.index, region, &phys_addr, NULL, NULL);

  uint irq_line, irq_pin;
  pci_irq_t irq;
  if (!pci_get_interrupt (pdev.index, &irq_line, &irq_pin)) {
    DLOG ("Unable to get IRQ");
    goto abort;
  }

  if (pci_irq_find (pdev.bus, pdev.slot, irq_pin, &irq)) {
    /* use PCI routing table */
    DLOG ("Found PCI routing entry irq.gsi=0x%x", irq.gsi);

    /* 
     * Level triggered mode causes trouble when the interrupt
     * is broadcasted to all LAPICs. PCI-E should use edge mode
     * anyway.
     */
    irq.trigger = TRIGGER_EDGE;
    irq_backup = irq;

    if (!pci_irq_map_handler (&irq, irq_handler, 0x01,
                              IOAPIC_DESTINATION_LOGICAL,
                              IOAPIC_DELIVERY_FIXED)) {
      DLOG ("Failed to map IRQ");
      goto abort;
    }
    irq_line = irq.gsi;
    u8 vector;
    IOAPIC_get_GSI_mapping (irq.gsi, &vector, NULL);
    DLOG ("Using vector=0x%X", vector);
  } else {
    DLOG ("Unable to find PCI routing entry");
    goto abort;
  }

  /* Set master sandbox to be the one doing the hardware initialization */
  r8169_master_sandbox = get_pcpu_id ();

  ioaddr = map_virtual_page (phys_addr | 3);
  global_ioa_backup = ioaddr;

  DLOG ("BAR%d phys=0x%p virt=0x%p", region, phys_addr, ioaddr);

  RTL_W16(IntrMask, 0x0000);

  /* Soft reset the chip. */
  RTL_W8(ChipCmd, CmdReset);

  /* Check that the chip has finished the reset. */
  for (i = 0; i < 100; i++) {
    if ((RTL_R8(ChipCmd) & CmdReset) == 0)
      break;
    udelay (1000);
  }

  RTL_W16(IntrStatus, 0xffff);

  /* enable memory mapped I/O and bus mastering */
  pci_write_word (pci_addr (pdev.bus, pdev.slot, pdev.func, 0x04), 0x0006);

  /* Allocate private data struct */
  pow2_alloc (sizeof (struct rtl8169_private), (u8 **) &tp);
  if (!tp)
    goto abort_virt;
  DLOG ("tp=0x%p (%d bytes)", tp, sizeof (struct rtl8169_private));

  /* Identify chip attached to board */
  get_mac_version(tp, ioaddr);

  /* Use appropriate default if unknown */
  if (tp->mac_version == RTL_GIGA_MAC_NONE) {
    DLOG ("unknown MAC, using family default");
    tp->mac_version = cfg->default_ver;
  }

  print_mac_version(tp);

  for (i = 0; i < ARRAY_SIZE(rtl_chip_info); i++) {
    if (tp->mac_version == rtl_chip_info[i].mac_version)
      break;
  }
  if (i == ARRAY_SIZE(rtl_chip_info)) {
    DLOG ("driver bug, MAC version not found in rtl_chip_info");
    goto abort_tp;
  }
  tp->chipset = i;

  RTL_W8(Cfg9346, Cfg9346_Unlock);
  RTL_W8(Config1, RTL_R8(Config1) | PMEnable);
  RTL_W8(Config5, RTL_R8(Config5) & PMEStatus);
  if ((RTL_R8(Config3) & (LinkUp | MagicPacket)) != 0)
    tp->features |= RTL_FEATURE_WOL;
  if ((RTL_R8(Config5) & (UWF | BWF | MWF)) != 0)
    tp->features |= RTL_FEATURE_WOL;
  RTL_W8(Cfg9346, Cfg9346_Lock);

  if ((tp->mac_version <= RTL_GIGA_MAC_VER_06) &&
      (RTL_R8(PHYstatus) & TBI_Enable)) {
    DLOG ("TBI");
    tp->set_speed = rtl8169_set_speed_tbi;
    tp->get_settings = rtl8169_gset_tbi;
    tp->phy_reset_enable = rtl8169_tbi_reset_enable;
    tp->phy_reset_pending = rtl8169_tbi_reset_pending;
    tp->link_ok = rtl8169_tbi_link_ok;
    tp->do_ioctl = rtl_tbi_ioctl;

    tp->phy_1000_ctrl_reg = ADVERTISE_1000FULL; /* Implied by TBI */
  } else {
    DLOG ("XMII");
    tp->set_speed = rtl8169_set_speed_xmii;
    tp->get_settings = rtl8169_gset_xmii;
    tp->phy_reset_enable = rtl8169_xmii_reset_enable;
    tp->phy_reset_pending = rtl8169_xmii_reset_pending;
    tp->link_ok = rtl8169_xmii_link_ok;
    tp->do_ioctl = rtl_xmii_ioctl;
  }

  spinlock_init (&tp->lock);
  tp->mmio_addr = ioaddr;

  /* Get MAC address */
  for (i = 0; i < MAC_ADDR_LEN; i++)
    tp->mac_addr[i] = RTL_R8(MAC0 + i);
  DLOG ("mac_addr=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
        tp->mac_addr[0], tp->mac_addr[1], tp->mac_addr[2],
        tp->mac_addr[3], tp->mac_addr[4], tp->mac_addr[5]);

  tp->intr_mask = 0xffff;
  tp->align = cfg->align;
  tp->hw_start = cfg->hw_start;
  tp->intr_event = cfg->intr_event;
  tp->napi_event = cfg->napi_event;

  /* Register network device with net subsystem */
  /* Modified for SeQuest. We need a separate ethdev for each sandbox
   * and share all the other things in driver private data structure */
  for (i = 0; i < MAX_NUM_SHARE; i++) {
    tp->ethdev[i].recv_func = NULL;
    tp->ethdev[i].send_func = r8169_transmit;
    tp->ethdev[i].get_hwaddr_func = r8169_get_hwaddr;
    tp->ethdev[i].poll_func = r8169_poll;
    tp->ethdev[i].drvdata = tp;
  }

  tp->pci_dev = &pdev;
  pdev.drvdata = tp;

  init_phy(&tp->ethdev[0], tp);

  //if (!net_register_device (&tp->ethdev)) {
  //  DLOG ("registration failed");
  //  goto abort_tp;
  //}

#define ETH_FCS_LEN 4
#define VLAN_ETH_HLEN 18
  uint mtu = MAX_FRAME_SIZE - VLAN_ETH_HLEN - ETH_FCS_LEN;
  uint max_frame = mtu + VLAN_ETH_HLEN + ETH_FCS_LEN;
  tp->rx_buf_sz = (max_frame > RX_BUF_SIZE) ? max_frame : RX_BUF_SIZE;

  pow2_alloc (R8169_RX_RING_BYTES, (u8 **) &tp->RxDescArray);
  if (!tp->RxDescArray)
    goto abort_tp;
  tp->RxPhyAddr = (uint) get_phys_addr (tp->RxDescArray);

  pow2_alloc (R8169_TX_RING_BYTES, (u8 **) &tp->TxDescArray);
  if (!tp->TxDescArray)
    goto abort_rxdesc;
  tp->TxPhyAddr = (uint) get_phys_addr (tp->TxDescArray);

  sint retval = rtl8169_init_ring(&tp->ethdev[0]);
  if (retval < 0)
    goto abort_txdesc;

  rtl_hw_start(&tp->ethdev[0]);

  //rtl8169_request_timer(dev);

  rtl8169_check_link_status(&tp->ethdev[0], tp, tp->mmio_addr);

//  r8169_bh_id[0] = create_kernel_thread_args ((u32) r8169_bh_thread,
//                                              (u32) &r8169_bh_stack[0][1023],
//                                              FALSE, 0);
//  set_iovcpu (r8169_bh_id[0], IOVCPU_CLASS_NET);

#ifdef USE_VMX
  r8169_initialized = TRUE;
  r8169_tx_lock = shm_alloc_drv_lock ();
  if (r8169_tx_lock == NULL) {
    logger_printf ("Driver lock allocation failed!\n");
  } else {
    spinlock_init (r8169_tx_lock);
  }

  r8169_rx_int_lock = shm_alloc_drv_lock ();
  if (r8169_rx_int_lock == NULL) {
    logger_printf ("Driver lock allocation failed!\n");
  } else {
    spinlock_init (r8169_rx_int_lock);
  }

  r8169_tx_int_lock = shm_alloc_drv_lock ();
  if (r8169_tx_int_lock == NULL) {
    logger_printf ("Driver lock allocation failed!\n");
  } else {
    spinlock_init (r8169_tx_int_lock);
  }

  r8169_reg_lock = shm_alloc_drv_lock ();
  if (r8169_reg_lock == NULL) {
    logger_printf ("Driver lock allocation failed!\n");
  } else {
    spinlock_init (r8169_reg_lock);
  }
#endif

#ifdef TX_TIMING
  timing_id =
    start_kernel_thread ((u32) timing_thread, (u32) &timing_stack[1023]);
#endif

  return TRUE;
 abort_txdesc:
  pow2_free ((u8 *) tp->TxDescArray);
 abort_rxdesc:
  pow2_free ((u8 *) tp->RxDescArray);
 abort_tp:
  pow2_free ((u8 *) tp);
 abort_virt:
  unmap_virtual_page (ioaddr);
 abort:
  return FALSE;
}

/*
 * This is used by sandboxes that share this driver and did not
 * initialize it themself.
 */
extern bool
r8169_register (void)
{
  uint32 cpu;
  cpu = get_pcpu_id ();
  uint8 vector = 0;
  uint64 flags = 0;

  if (cpu >= MAX_NUM_SHARE) {
    DLOG ("Too many sandboxes");
    return FALSE;
  }

#ifdef USE_VMX
  spinlock_lock (r8169_reg_lock);
#endif

  if (!net_register_device (&tp->ethdev[cpu])) {
    DLOG ("registration failed");
    return FALSE;
  }

  if (IOAPIC_get_GSI_mapping (irq_backup.gsi, &vector, &flags) == -1) {
    logger_printf ("r8169: Cannot get GSI mapping from IOAPIC\n");
    return FALSE;
  }

  /* Ask IOAPIC for interrupt delivery to THIS sandbox (core) */
  IOAPIC_map_GSI (irq_backup.gsi, vector, flags | (((uint64)(0x1 << cpu)) << 56));

  atomic_inc (&num_sharing);
  logger_printf ("r8169: %d sandboxes sharing this driver\n", num_sharing.counter);

  r8169_bh_id[cpu] =
      create_kernel_thread_args ((u32) r8169_bh_thread,
                                 (u32) &r8169_bh_stack[cpu][1023],
                                 FALSE, 0);
  set_iovcpu (r8169_bh_id[cpu], IOVCPU_CLASS_NET);

#ifdef USE_VMX
  spinlock_unlock (r8169_reg_lock);
#endif

  return TRUE;
}

extern void
r8169_free ()
{
  unmap_virtual_page (tp->mmio_addr);
  pow2_free ((u8 *) tp->TxDescArray);
  pow2_free ((u8 *) tp->RxDescArray);
  pow2_free ((u8 *) tp);
}

static const struct module_ops mod_ops = {
  //.init = r8169_init
  .init = r8169_register
};

DEF_MODULE (net___r8169, "r8169 network driver", &mod_ops, {"net___ethernet", "pci"});


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
