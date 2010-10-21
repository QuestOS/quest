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

/* Broadcom NetXtreme II driver */

/* Based on Linux driver by Michael Chan. */

#include "drivers/pci/pci.h"
#include "drivers/net/ethernet.h"
#include "arch/i386.h"
#include "util/printf.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "mem/physical.h"
#include "mem/virtual.h"
#include "mem/pow2.h"
#include "kernel.h"
#include "sched/vcpu.h"

#include "bnx2.h"
#include "bnx2_uncompressed_fw.h"

#define BNX2_VECTOR 0x4E       /* arbitrary */

#define DEBUG_BNX2

#ifdef DEBUG_BNX2
#define DLOG(fmt,...) DLOG_PREFIX("bnx2",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

typedef enum {
  BCM5706 = 0,
  NC370T,
  NC370I,
  BCM5706S,
  NC370F,
  BCM5708,
  BCM5708S,
  BCM5709,
  BCM5709S,
  BCM5716,
  BCM5716S,
} board_t;

/* indexed by board_t, above */
static struct {
  char *name;
} board_info[] = {
  { "Broadcom NetXtreme II BCM5706 1000Base-T" },
  { "HP NC370T Multifunction Gigabit Server Adapter" },
  { "HP NC370i Multifunction Gigabit Server Adapter" },
  { "Broadcom NetXtreme II BCM5706 1000Base-SX" },
  { "HP NC370F Multifunction Gigabit Server Adapter" },
  { "Broadcom NetXtreme II BCM5708 1000Base-T" },
  { "Broadcom NetXtreme II BCM5708 1000Base-SX" },
  { "Broadcom NetXtreme II BCM5709 1000Base-T" },
  { "Broadcom NetXtreme II BCM5709 1000Base-SX" },
  { "Broadcom NetXtreme II BCM5716 1000Base-T" },
  { "Broadcom NetXtreme II BCM5716 1000Base-SX" },
};

/* List of compatible cards (ended by { 0xFFFF, 0xFFFF }) */
static struct {
  uint16 vendor, device, subvendor, subdevice, class, classmask, index;
} compatible_ids[] = {
  { PCI_VENDOR_ID_BROADCOM, PCI_DEVICE_ID_NX2_5706,
    PCI_VENDOR_ID_HP, 0x3101, 0, 0, NC370T },
  { PCI_VENDOR_ID_BROADCOM, PCI_DEVICE_ID_NX2_5706,
    PCI_VENDOR_ID_HP, 0x3106, 0, 0, NC370I },
  { PCI_VENDOR_ID_BROADCOM, PCI_DEVICE_ID_NX2_5706,
    PCI_ANY_ID, PCI_ANY_ID, 0, 0, BCM5706 },
  { PCI_VENDOR_ID_BROADCOM, PCI_DEVICE_ID_NX2_5708,
    PCI_ANY_ID, PCI_ANY_ID, 0, 0, BCM5708 },
  { PCI_VENDOR_ID_BROADCOM, PCI_DEVICE_ID_NX2_5706S,
    PCI_VENDOR_ID_HP, 0x3102, 0, 0, NC370F },
  { PCI_VENDOR_ID_BROADCOM, PCI_DEVICE_ID_NX2_5706S,
    PCI_ANY_ID, PCI_ANY_ID, 0, 0, BCM5706S },
  { PCI_VENDOR_ID_BROADCOM, PCI_DEVICE_ID_NX2_5708S,
    PCI_ANY_ID, PCI_ANY_ID, 0, 0, BCM5708S },
  { PCI_VENDOR_ID_BROADCOM, PCI_DEVICE_ID_NX2_5709,
    PCI_ANY_ID, PCI_ANY_ID, 0, 0, BCM5709 },
  { PCI_VENDOR_ID_BROADCOM, PCI_DEVICE_ID_NX2_5709S,
    PCI_ANY_ID, PCI_ANY_ID, 0, 0, BCM5709S },
  { PCI_VENDOR_ID_BROADCOM, 0x163b,
    PCI_ANY_ID, PCI_ANY_ID, 0, 0, BCM5716 },
  { PCI_VENDOR_ID_BROADCOM, 0x163c,
    PCI_ANY_ID, PCI_ANY_ID, 0, 0, BCM5716S },
  { 0xFFFF, 0xFFFF }
};

static const struct flash_spec flash_table[] =
{
#define BUFFERED_FLAGS          (BNX2_NV_BUFFERED | BNX2_NV_TRANSLATE)
#define NONBUFFERED_FLAGS       (BNX2_NV_WREN)
  /* Slow EEPROM */
  {0x00000000, 0x40830380, 0x009f0081, 0xa184a053, 0xaf000400,
   BUFFERED_FLAGS, SEEPROM_PAGE_BITS, SEEPROM_PAGE_SIZE,
   SEEPROM_BYTE_ADDR_MASK, SEEPROM_TOTAL_SIZE,
   "EEPROM - slow"},
  /* Expansion entry 0001 */
  {0x08000002, 0x4b808201, 0x00050081, 0x03840253, 0xaf020406,
   NONBUFFERED_FLAGS, SAIFUN_FLASH_PAGE_BITS, SAIFUN_FLASH_PAGE_SIZE,
   SAIFUN_FLASH_BYTE_ADDR_MASK, 0,
   "Entry 0001"},
  /* Saifun SA25F010 (non-buffered flash) */
  /* strap, cfg1, & write1 need updates */
  {0x04000001, 0x47808201, 0x00050081, 0x03840253, 0xaf020406,
   NONBUFFERED_FLAGS, SAIFUN_FLASH_PAGE_BITS, SAIFUN_FLASH_PAGE_SIZE,
   SAIFUN_FLASH_BYTE_ADDR_MASK, SAIFUN_FLASH_BASE_TOTAL_SIZE*2,
   "Non-buffered flash (128kB)"},
  /* Saifun SA25F020 (non-buffered flash) */
  /* strap, cfg1, & write1 need updates */
  {0x0c000003, 0x4f808201, 0x00050081, 0x03840253, 0xaf020406,
   NONBUFFERED_FLAGS, SAIFUN_FLASH_PAGE_BITS, SAIFUN_FLASH_PAGE_SIZE,
   SAIFUN_FLASH_BYTE_ADDR_MASK, SAIFUN_FLASH_BASE_TOTAL_SIZE*4,
   "Non-buffered flash (256kB)"},
  /* Expansion entry 0100 */
  {0x11000000, 0x53808201, 0x00050081, 0x03840253, 0xaf020406,
   NONBUFFERED_FLAGS, SAIFUN_FLASH_PAGE_BITS, SAIFUN_FLASH_PAGE_SIZE,
   SAIFUN_FLASH_BYTE_ADDR_MASK, 0,
   "Entry 0100"},
  /* Entry 0101: ST M45PE10 (non-buffered flash, TetonII B0) */
  {0x19000002, 0x5b808201, 0x000500db, 0x03840253, 0xaf020406,
   NONBUFFERED_FLAGS, ST_MICRO_FLASH_PAGE_BITS, ST_MICRO_FLASH_PAGE_SIZE,
   ST_MICRO_FLASH_BYTE_ADDR_MASK, ST_MICRO_FLASH_BASE_TOTAL_SIZE*2,
   "Entry 0101: ST M45PE10 (128kB non-bufferred)"},
  /* Entry 0110: ST M45PE20 (non-buffered flash)*/
  {0x15000001, 0x57808201, 0x000500db, 0x03840253, 0xaf020406,
   NONBUFFERED_FLAGS, ST_MICRO_FLASH_PAGE_BITS, ST_MICRO_FLASH_PAGE_SIZE,
   ST_MICRO_FLASH_BYTE_ADDR_MASK, ST_MICRO_FLASH_BASE_TOTAL_SIZE*4,
   "Entry 0110: ST M45PE20 (256kB non-bufferred)"},
  /* Saifun SA25F005 (non-buffered flash) */
  /* strap, cfg1, & write1 need updates */
  {0x1d000003, 0x5f808201, 0x00050081, 0x03840253, 0xaf020406,
   NONBUFFERED_FLAGS, SAIFUN_FLASH_PAGE_BITS, SAIFUN_FLASH_PAGE_SIZE,
   SAIFUN_FLASH_BYTE_ADDR_MASK, SAIFUN_FLASH_BASE_TOTAL_SIZE,
   "Non-buffered flash (64kB)"},
  /* Fast EEPROM */
  {0x22000000, 0x62808380, 0x009f0081, 0xa184a053, 0xaf000400,
   BUFFERED_FLAGS, SEEPROM_PAGE_BITS, SEEPROM_PAGE_SIZE,
   SEEPROM_BYTE_ADDR_MASK, SEEPROM_TOTAL_SIZE,
   "EEPROM - fast"},
  /* Expansion entry 1001 */
  {0x2a000002, 0x6b808201, 0x00050081, 0x03840253, 0xaf020406,
   NONBUFFERED_FLAGS, SAIFUN_FLASH_PAGE_BITS, SAIFUN_FLASH_PAGE_SIZE,
   SAIFUN_FLASH_BYTE_ADDR_MASK, 0,
   "Entry 1001"},
  /* Expansion entry 1010 */
  {0x26000001, 0x67808201, 0x00050081, 0x03840253, 0xaf020406,
   NONBUFFERED_FLAGS, SAIFUN_FLASH_PAGE_BITS, SAIFUN_FLASH_PAGE_SIZE,
   SAIFUN_FLASH_BYTE_ADDR_MASK, 0,
   "Entry 1010"},
  /* ATMEL AT45DB011B (buffered flash) */
  {0x2e000003, 0x6e808273, 0x00570081, 0x68848353, 0xaf000400,
   BUFFERED_FLAGS, BUFFERED_FLASH_PAGE_BITS, BUFFERED_FLASH_PAGE_SIZE,
   BUFFERED_FLASH_BYTE_ADDR_MASK, BUFFERED_FLASH_TOTAL_SIZE,
   "Buffered flash (128kB)"},
  /* Expansion entry 1100 */
  {0x33000000, 0x73808201, 0x00050081, 0x03840253, 0xaf020406,
   NONBUFFERED_FLAGS, SAIFUN_FLASH_PAGE_BITS, SAIFUN_FLASH_PAGE_SIZE,
   SAIFUN_FLASH_BYTE_ADDR_MASK, 0,
   "Entry 1100"},
  /* Expansion entry 1101 */
  {0x3b000002, 0x7b808201, 0x00050081, 0x03840253, 0xaf020406,
   NONBUFFERED_FLAGS, SAIFUN_FLASH_PAGE_BITS, SAIFUN_FLASH_PAGE_SIZE,
   SAIFUN_FLASH_BYTE_ADDR_MASK, 0,
   "Entry 1101"},
  /* Ateml Expansion entry 1110 */
  {0x37000001, 0x76808273, 0x00570081, 0x68848353, 0xaf000400,
   BUFFERED_FLAGS, BUFFERED_FLASH_PAGE_BITS, BUFFERED_FLASH_PAGE_SIZE,
   BUFFERED_FLASH_BYTE_ADDR_MASK, 0,
   "Entry 1110 (Atmel)"},
  /* ATMEL AT45DB021B (buffered flash) */
  {0x3f000003, 0x7e808273, 0x00570081, 0x68848353, 0xaf000400,
   BUFFERED_FLAGS, BUFFERED_FLASH_PAGE_BITS, BUFFERED_FLASH_PAGE_SIZE,
   BUFFERED_FLASH_BYTE_ADDR_MASK, BUFFERED_FLASH_TOTAL_SIZE*2,
   "Buffered flash (256kB)"},
};

static const struct flash_spec flash_5709 = {
  .flags                = BNX2_NV_BUFFERED,
  .page_bits    = BCM5709_FLASH_PAGE_BITS,
  .page_size    = BCM5709_FLASH_PAGE_SIZE,
  .addr_mask    = BCM5709_FLASH_BYTE_ADDR_MASK,
  .total_size   = BUFFERED_FLASH_TOTAL_SIZE*2,
  .name         = "5709 Buffered flash (256kB)",
};

static void
pci_write_config_dword (pci_device *p, u32 offset, u32 val)
{
  pci_write_dword (pci_addr (p->bus, p->slot, p->func, offset), val);
}

#define udelay tsc_delay_usec
#define EBUSY 1
#define ENODEV 2
#define EIO 3

static u32
bnx2_reg_rd_ind(struct bnx2 *bp, u32 offset)
{
  u32 val;

  spinlock_lock(&bp->indirect_lock);
  REG_WR(bp, BNX2_PCICFG_REG_WINDOW_ADDRESS, offset);
  val = REG_RD(bp, BNX2_PCICFG_REG_WINDOW);
  spinlock_unlock(&bp->indirect_lock);
  return val;
}

static void
bnx2_reg_wr_ind(struct bnx2 *bp, u32 offset, u32 val)
{
  spinlock_lock(&bp->indirect_lock);
  REG_WR(bp, BNX2_PCICFG_REG_WINDOW_ADDRESS, offset);
  REG_WR(bp, BNX2_PCICFG_REG_WINDOW, val);
  spinlock_unlock(&bp->indirect_lock);
}

static u32
bnx2_shmem_rd(struct bnx2 *bp, u32 offset)
{
  return (bnx2_reg_rd_ind(bp, bp->shmem_base + offset));
}

static void
bnx2_shmem_wr(struct bnx2 *bp, u32 offset, u32 val)
{
  bnx2_reg_wr_ind(bp, bp->shmem_base + offset, val);
}

static void
bnx2_ctx_wr(struct bnx2 *bp, u32 cid_addr, u32 offset, u32 val)
{
  offset += cid_addr;
  spinlock_lock(&bp->indirect_lock);
  if (CHIP_NUM(bp) == CHIP_NUM_5709) {
    int i;

    REG_WR(bp, BNX2_CTX_CTX_DATA, val);
    REG_WR(bp, BNX2_CTX_CTX_CTRL,
           offset | BNX2_CTX_CTX_CTRL_WRITE_REQ);
    for (i = 0; i < 5; i++) {
      val = REG_RD(bp, BNX2_CTX_CTX_CTRL);
      if ((val & BNX2_CTX_CTX_CTRL_WRITE_REQ) == 0)
        break;
      udelay(5);
    }
  } else {
    REG_WR(bp, BNX2_CTX_DATA_ADR, offset);
    REG_WR(bp, BNX2_CTX_DATA, val);
  }
  spinlock_unlock(&bp->indirect_lock);
}

static int
bnx2_acquire_nvram_lock(struct bnx2 *bp)
{
  u32 val;
  int j;

  /* Request access to the flash interface. */
  REG_WR(bp, BNX2_NVM_SW_ARB, BNX2_NVM_SW_ARB_ARB_REQ_SET2);
  for (j = 0; j < NVRAM_TIMEOUT_COUNT; j++) {
    val = REG_RD(bp, BNX2_NVM_SW_ARB);
    if (val & BNX2_NVM_SW_ARB_ARB_ARB2)
      break;

    udelay(5);
  }

  if (j >= NVRAM_TIMEOUT_COUNT)
    return -EBUSY;

  return 0;
}

static int
bnx2_release_nvram_lock(struct bnx2 *bp)
{
  int j;
  u32 val;

  /* Relinquish nvram interface. */
  REG_WR(bp, BNX2_NVM_SW_ARB, BNX2_NVM_SW_ARB_ARB_REQ_CLR2);

  for (j = 0; j < NVRAM_TIMEOUT_COUNT; j++) {
    val = REG_RD(bp, BNX2_NVM_SW_ARB);
    if (!(val & BNX2_NVM_SW_ARB_ARB_ARB2))
      break;

    udelay(5);
  }

  if (j >= NVRAM_TIMEOUT_COUNT)
    return -EBUSY;

  return 0;
}

static void
bnx2_enable_nvram_access(struct bnx2 *bp)
{
  u32 val;

  val = REG_RD(bp, BNX2_NVM_ACCESS_ENABLE);
  /* Enable both bits, even on read. */
  REG_WR(bp, BNX2_NVM_ACCESS_ENABLE,
         val | BNX2_NVM_ACCESS_ENABLE_EN | BNX2_NVM_ACCESS_ENABLE_WR_EN);
}

static void
bnx2_disable_nvram_access(struct bnx2 *bp)
{
  u32 val;

  val = REG_RD(bp, BNX2_NVM_ACCESS_ENABLE);
  /* Disable both bits, even after read. */
  REG_WR(bp, BNX2_NVM_ACCESS_ENABLE,
         val & ~(BNX2_NVM_ACCESS_ENABLE_EN |
                 BNX2_NVM_ACCESS_ENABLE_WR_EN));
}

static int
bnx2_nvram_read_dword(struct bnx2 *bp, u32 offset, u8 *ret_val, u32 cmd_flags)
{
  u32 cmd;
  int j;

  /* Build the command word. */
  cmd = BNX2_NVM_COMMAND_DOIT | cmd_flags;

  /* Calculate an offset of a buffered flash, not needed for 5709. */
  if (bp->flash_info->flags & BNX2_NV_TRANSLATE) {
    offset = ((offset / bp->flash_info->page_size) <<
              bp->flash_info->page_bits) +
      (offset % bp->flash_info->page_size);
  }

  /* Need to clear DONE bit separately. */
  REG_WR(bp, BNX2_NVM_COMMAND, BNX2_NVM_COMMAND_DONE);

  /* Address of the NVRAM to read from. */
  REG_WR(bp, BNX2_NVM_ADDR, offset & BNX2_NVM_ADDR_NVM_ADDR_VALUE);

  /* Issue a read command. */
  REG_WR(bp, BNX2_NVM_COMMAND, cmd);

  /* Wait for completion. */
  for (j = 0; j < NVRAM_TIMEOUT_COUNT; j++) {
    u32 val;

    udelay(5);

    val = REG_RD(bp, BNX2_NVM_COMMAND);
    if (val & BNX2_NVM_COMMAND_DONE) {
      __be32 v = __cpu_to_be32(REG_RD(bp, BNX2_NVM_READ));
      memcpy(ret_val, &v, 4);
      break;
    }
  }
  if (j >= NVRAM_TIMEOUT_COUNT)
    return -EBUSY;

  return 0;
}

static int
bnx2_nvram_read(struct bnx2 *bp, u32 offset, u8 *ret_buf,
                int buf_size)
{
  int rc = 0;
  u32 cmd_flags, offset32, len32, extra;

  if (buf_size == 0)
    return 0;

  /* Request access to the flash interface. */
  if ((rc = bnx2_acquire_nvram_lock(bp)) != 0)
    return rc;

  /* Enable access to flash interface */
  bnx2_enable_nvram_access(bp);

  len32 = buf_size;
  offset32 = offset;
  extra = 0;

  cmd_flags = 0;

  if (offset32 & 3) {
    u8 buf[4];
    u32 pre_len;

    offset32 &= ~3;
    pre_len = 4 - (offset & 3);

    if (pre_len >= len32) {
      pre_len = len32;
      cmd_flags = BNX2_NVM_COMMAND_FIRST |
        BNX2_NVM_COMMAND_LAST;
    }
    else {
      cmd_flags = BNX2_NVM_COMMAND_FIRST;
    }

    rc = bnx2_nvram_read_dword(bp, offset32, buf, cmd_flags);

    if (rc)
      return rc;

    memcpy(ret_buf, buf + (offset & 3), pre_len);

    offset32 += 4;
    ret_buf += pre_len;
    len32 -= pre_len;
  }
  if (len32 & 3) {
    extra = 4 - (len32 & 3);
    len32 = (len32 + 4) & ~3;
  }

  if (len32 == 4) {
    u8 buf[4];

    if (cmd_flags)
      cmd_flags = BNX2_NVM_COMMAND_LAST;
    else
      cmd_flags = BNX2_NVM_COMMAND_FIRST |
        BNX2_NVM_COMMAND_LAST;

    rc = bnx2_nvram_read_dword(bp, offset32, buf, cmd_flags);

    memcpy(ret_buf, buf, 4 - extra);
  }
  else if (len32 > 0) {
    u8 buf[4];

    /* Read the first word. */
    if (cmd_flags)
      cmd_flags = 0;
    else
      cmd_flags = BNX2_NVM_COMMAND_FIRST;

    rc = bnx2_nvram_read_dword(bp, offset32, ret_buf, cmd_flags);

    /* Advance to the next dword. */
    offset32 += 4;
    ret_buf += 4;
    len32 -= 4;

    while (len32 > 4 && rc == 0) {
      rc = bnx2_nvram_read_dword(bp, offset32, ret_buf, 0);

      /* Advance to the next dword. */
      offset32 += 4;
      ret_buf += 4;
      len32 -= 4;
    }

    if (rc)
      return rc;

    cmd_flags = BNX2_NVM_COMMAND_LAST;
    rc = bnx2_nvram_read_dword(bp, offset32, buf, cmd_flags);

    memcpy(ret_buf, buf, 4 - extra);
  }

  /* Disable access to flash interface */
  bnx2_disable_nvram_access(bp);

  bnx2_release_nvram_lock(bp);

  return rc;
}

static int
bnx2_init_nvram(struct bnx2 *bp)
{
  u32 val;
  int j, entry_count, rc = 0;
  const struct flash_spec *flash;

  if (CHIP_NUM(bp) == CHIP_NUM_5709) {
    bp->flash_info = &flash_5709;
    goto get_flash_size;
  }

  /* Determine the selected interface. */
  val = REG_RD(bp, BNX2_NVM_CFG1);

  entry_count = ARRAY_SIZE(flash_table);

  if (val & 0x40000000) {

    /* Flash interface has been reconfigured */
    for (j = 0, flash = &flash_table[0]; j < entry_count;
         j++, flash++) {
      if ((val & FLASH_BACKUP_STRAP_MASK) ==
          (flash->config1 & FLASH_BACKUP_STRAP_MASK)) {
        bp->flash_info = flash;
        break;
      }
    }
  }
  else {
    u32 mask;
    /* Not yet been reconfigured */

    if (val & (1 << 23))
      mask = FLASH_BACKUP_STRAP_MASK;
    else
      mask = FLASH_STRAP_MASK;

    for (j = 0, flash = &flash_table[0]; j < entry_count;
         j++, flash++) {

      if ((val & mask) == (flash->strapping & mask)) {
        bp->flash_info = flash;

        /* Request access to the flash interface. */
        if ((rc = bnx2_acquire_nvram_lock(bp)) != 0)
          return rc;

        /* Enable access to flash interface */
        bnx2_enable_nvram_access(bp);

        /* Reconfigure the flash interface */
        REG_WR(bp, BNX2_NVM_CFG1, flash->config1);
        REG_WR(bp, BNX2_NVM_CFG2, flash->config2);
        REG_WR(bp, BNX2_NVM_CFG3, flash->config3);
        REG_WR(bp, BNX2_NVM_WRITE1, flash->write1);

        /* Disable access to flash interface */
        bnx2_disable_nvram_access(bp);
        bnx2_release_nvram_lock(bp);

        break;
      }
    }
  } /* if (val & 0x40000000) */

  if (j == entry_count) {
    bp->flash_info = NULL;
    DLOG ("Unknown flash/EEPROM type");
    return -ENODEV;
  }

 get_flash_size:
  val = bnx2_shmem_rd(bp, BNX2_SHARED_HW_CFG_CONFIG2);
  val &= BNX2_SHARED_HW_CFG2_NVM_SIZE_MASK;
  if (val)
    bp->flash_size = val;
  else
    bp->flash_size = bp->flash_info->total_size;

  return rc;
}

#if 0
static void
bnx2_read_vpd_fw_ver(struct bnx2 *bp)
{
  int rc, i, j;
  u8 *data;
  unsigned int block_end, rosize, len;

#define BNX2_VPD_NVRAM_OFFSET   0x300
#define BNX2_VPD_LEN            128
#define BNX2_MAX_VER_SLEN       30

  //data = kmalloc(256, GFP_KERNEL);
  u8 _data[256];
  data = _data;

  if (!data)
    return;

  rc = bnx2_nvram_read(bp, BNX2_VPD_NVRAM_OFFSET, data + BNX2_VPD_LEN,
                       BNX2_VPD_LEN);
  if (rc)
    goto vpd_done;

  for (i = 0; i < BNX2_VPD_LEN; i += 4) {
    data[i] = data[i + BNX2_VPD_LEN + 3];
    data[i + 1] = data[i + BNX2_VPD_LEN + 2];
    data[i + 2] = data[i + BNX2_VPD_LEN + 1];
    data[i + 3] = data[i + BNX2_VPD_LEN];
  }

  i = pci_vpd_find_tag(data, 0, BNX2_VPD_LEN, PCI_VPD_LRDT_RO_DATA);
  if (i < 0)
    goto vpd_done;

  rosize = pci_vpd_lrdt_size(&data[i]);
  i += PCI_VPD_LRDT_TAG_SIZE;
  block_end = i + rosize;

  if (block_end > BNX2_VPD_LEN)
    goto vpd_done;

  j = pci_vpd_find_info_keyword(data, i, rosize,
                                PCI_VPD_RO_KEYWORD_MFR_ID);
  if (j < 0)
    goto vpd_done;

  len = pci_vpd_info_field_size(&data[j]);

  j += PCI_VPD_INFO_FLD_HDR_SIZE;
  if (j + len > block_end || len != 4 ||
      memcmp(&data[j], "1028", 4))
    goto vpd_done;

  j = pci_vpd_find_info_keyword(data, i, rosize,
                                PCI_VPD_RO_KEYWORD_VENDOR0);
  if (j < 0)
    goto vpd_done;

  len = pci_vpd_info_field_size(&data[j]);

  j += PCI_VPD_INFO_FLD_HDR_SIZE;
  if (j + len > block_end || len > BNX2_MAX_VER_SLEN)
    goto vpd_done;

  memcpy(bp->fw_version, &data[j], len);
  bp->fw_version[len] = ' ';

 vpd_done:
  //kfree(data);
}
#endif

static void
bnx2_get_5709_media(struct bnx2 *bp)
{
  u32 val = REG_RD(bp, BNX2_MISC_DUAL_MEDIA_CTRL);
  u32 bond_id = val & BNX2_MISC_DUAL_MEDIA_CTRL_BOND_ID;
  u32 strap;

  if (bond_id == BNX2_MISC_DUAL_MEDIA_CTRL_BOND_ID_C)
    return;
  else if (bond_id == BNX2_MISC_DUAL_MEDIA_CTRL_BOND_ID_S) {
    bp->phy_flags |= BNX2_PHY_FLAG_SERDES;
    return;
  }

  if (val & BNX2_MISC_DUAL_MEDIA_CTRL_STRAP_OVERRIDE)
    strap = (val & BNX2_MISC_DUAL_MEDIA_CTRL_PHY_CTRL) >> 21;
  else
    strap = (val & BNX2_MISC_DUAL_MEDIA_CTRL_PHY_CTRL_STRAP) >> 8;

  if (bp->pdev->func == 0) {
    switch (strap) {
    case 0x4:
    case 0x5:
    case 0x6:
      bp->phy_flags |= BNX2_PHY_FLAG_SERDES;
      return;
    }
  } else {
    switch (strap) {
    case 0x1:
    case 0x2:
    case 0x4:
      bp->phy_flags |= BNX2_PHY_FLAG_SERDES;
      return;
    }
  }
}

static void
bnx2_init_fw_cap(struct bnx2 *bp)
{
  u32 val, sig = 0;

  bp->phy_flags &= ~BNX2_PHY_FLAG_REMOTE_PHY_CAP;
  bp->flags &= ~BNX2_FLAG_CAN_KEEP_VLAN;

  if (!(bp->flags & BNX2_FLAG_ASF_ENABLE))
    bp->flags |= BNX2_FLAG_CAN_KEEP_VLAN;

  val = bnx2_shmem_rd(bp, BNX2_FW_CAP_MB);
  if ((val & BNX2_FW_CAP_SIGNATURE_MASK) != BNX2_FW_CAP_SIGNATURE)
    return;

  if ((val & BNX2_FW_CAP_CAN_KEEP_VLAN) == BNX2_FW_CAP_CAN_KEEP_VLAN) {
    bp->flags |= BNX2_FLAG_CAN_KEEP_VLAN;
    sig |= BNX2_DRV_ACK_CAP_SIGNATURE | BNX2_FW_CAP_CAN_KEEP_VLAN;
  }

  if ((bp->phy_flags & BNX2_PHY_FLAG_SERDES) &&
      (val & BNX2_FW_CAP_REMOTE_PHY_CAPABLE)) {
    u32 link;

    bp->phy_flags |= BNX2_PHY_FLAG_REMOTE_PHY_CAP;

    link = bnx2_shmem_rd(bp, BNX2_LINK_STATUS);
    if (link & BNX2_LINK_STATUS_SERDES_LINK)
      bp->phy_port = PORT_FIBRE;
    else
      bp->phy_port = PORT_TP;

    sig |= BNX2_DRV_ACK_CAP_SIGNATURE |
      BNX2_FW_CAP_REMOTE_PHY_CAPABLE;
  }

#if 0
  if (netif_running(bp->dev) && sig)
    bnx2_shmem_wr(bp, BNX2_DRV_ACK_CAP_MB, sig);
#endif
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

/* Indicates what features are advertised by the interface. */
#define ADVERTISED_10baseT_Half (1 << 0)
#define ADVERTISED_10baseT_Full (1 << 1)
#define ADVERTISED_100baseT_Half        (1 << 2)
#define ADVERTISED_100baseT_Full        (1 << 3)
#define ADVERTISED_1000baseT_Half       (1 << 4)
#define ADVERTISED_1000baseT_Full       (1 << 5)
#define ADVERTISED_Autoneg              (1 << 6)
#define ADVERTISED_TP                   (1 << 7)
#define ADVERTISED_AUI                  (1 << 8)
#define ADVERTISED_MII                  (1 << 9)
#define ADVERTISED_FIBRE                (1 << 10)
#define ADVERTISED_BNC                  (1 << 11)
#define ADVERTISED_10000baseT_Full      (1 << 12)
#define ADVERTISED_Pause                (1 << 13)
#define ADVERTISED_Asym_Pause           (1 << 14)
#define ADVERTISED_2500baseX_Full       (1 << 15)
#define ADVERTISED_Backplane            (1 << 16)
#define ADVERTISED_1000baseKX_Full      (1 << 17)
#define ADVERTISED_10000baseKX4_Full    (1 << 18)
#define ADVERTISED_10000baseKR_Full     (1 << 19)
#define ADVERTISED_10000baseR_FEC       (1 << 20)

#define ETHTOOL_ALL_FIBRE_SPEED                                 \
  (bp->phy_flags & BNX2_PHY_FLAG_2_5G_CAPABLE) ?                \
  (ADVERTISED_2500baseX_Full | ADVERTISED_1000baseT_Full) :     \
  (ADVERTISED_1000baseT_Full)

#define ETHTOOL_ALL_COPPER_SPEED                                \
  (ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |		\
   ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full |        \
   ADVERTISED_1000baseT_Full)

#define PHY_ALL_10_100_SPEED (ADVERTISE_10HALF | ADVERTISE_10FULL |     \
                              ADVERTISE_100HALF | ADVERTISE_100FULL | ADVERTISE_CSMA)

#define PHY_ALL_1000_SPEED (ADVERTISE_1000HALF | ADVERTISE_1000FULL)

static void
bnx2_set_default_remote_link(struct bnx2 *bp)
{
  u32 link;

  if (bp->phy_port == PORT_TP)
    link = bnx2_shmem_rd(bp, BNX2_RPHY_COPPER_LINK);
  else
    link = bnx2_shmem_rd(bp, BNX2_RPHY_SERDES_LINK);

  if (link & BNX2_NETLINK_SET_LINK_ENABLE_AUTONEG) {
    bp->req_line_speed = 0;
    bp->autoneg |= AUTONEG_SPEED;
    bp->advertising = ADVERTISED_Autoneg;
    if (link & BNX2_NETLINK_SET_LINK_SPEED_10HALF)
      bp->advertising |= ADVERTISED_10baseT_Half;
    if (link & BNX2_NETLINK_SET_LINK_SPEED_10FULL)
      bp->advertising |= ADVERTISED_10baseT_Full;
    if (link & BNX2_NETLINK_SET_LINK_SPEED_100HALF)
      bp->advertising |= ADVERTISED_100baseT_Half;
    if (link & BNX2_NETLINK_SET_LINK_SPEED_100FULL)
      bp->advertising |= ADVERTISED_100baseT_Full;
    if (link & BNX2_NETLINK_SET_LINK_SPEED_1GFULL)
      bp->advertising |= ADVERTISED_1000baseT_Full;
    if (link & BNX2_NETLINK_SET_LINK_SPEED_2G5FULL)
      bp->advertising |= ADVERTISED_2500baseX_Full;
  } else {
    bp->autoneg = 0;
    bp->advertising = 0;
    bp->req_duplex = DUPLEX_FULL;
    if (link & BNX2_NETLINK_SET_LINK_SPEED_10) {
      bp->req_line_speed = SPEED_10;
      if (link & BNX2_NETLINK_SET_LINK_SPEED_10HALF)
        bp->req_duplex = DUPLEX_HALF;
    }
    if (link & BNX2_NETLINK_SET_LINK_SPEED_100) {
      bp->req_line_speed = SPEED_100;
      if (link & BNX2_NETLINK_SET_LINK_SPEED_100HALF)
        bp->req_duplex = DUPLEX_HALF;
    }
    if (link & BNX2_NETLINK_SET_LINK_SPEED_1GFULL)
      bp->req_line_speed = SPEED_1000;
    if (link & BNX2_NETLINK_SET_LINK_SPEED_2G5FULL)
      bp->req_line_speed = SPEED_2500;
  }
}

static void
bnx2_set_default_link(struct bnx2 *bp)
{
  if (bp->phy_flags & BNX2_PHY_FLAG_REMOTE_PHY_CAP) {
    bnx2_set_default_remote_link(bp);
    return;
  }

  bp->autoneg = AUTONEG_SPEED | AUTONEG_FLOW_CTRL;
  bp->req_line_speed = 0;
  if (bp->phy_flags & BNX2_PHY_FLAG_SERDES) {
    u32 reg;

    bp->advertising = ETHTOOL_ALL_FIBRE_SPEED | ADVERTISED_Autoneg;

    reg = bnx2_shmem_rd(bp, BNX2_PORT_HW_CFG_CONFIG);
    reg &= BNX2_PORT_HW_CFG_CFG_DFLT_LINK_MASK;
    if (reg == BNX2_PORT_HW_CFG_CFG_DFLT_LINK_1G) {
      bp->autoneg = 0;
      bp->req_line_speed = bp->line_speed = SPEED_1000;
      bp->req_duplex = DUPLEX_FULL;
    }
  } else
    bp->advertising = ETHTOOL_ALL_COPPER_SPEED | ADVERTISED_Autoneg;
}

static u32
bnx2_irq_handler (u8 vec)
{
  DLOG ("IRQ");
  return 0;
}

static int
bnx2_fw_sync(struct bnx2 *bp, u32 msg_data, int ack, int silent)
{
  int i;
  u32 val;

  bp->fw_wr_seq++;
  msg_data |= bp->fw_wr_seq;

  bnx2_shmem_wr(bp, BNX2_DRV_MB, msg_data);

  if (!ack)
    return 0;

  /* wait for an acknowledgement. */
  for (i = 0; i < (BNX2_FW_ACK_TIME_OUT_MS / 10); i++) {
    udelay(10*1000);

    val = bnx2_shmem_rd(bp, BNX2_FW_MB);

    if ((val & BNX2_FW_MSG_ACK) == (msg_data & BNX2_DRV_MSG_SEQ))
      break;
  }
  if ((msg_data & BNX2_DRV_MSG_DATA) == BNX2_DRV_MSG_DATA_WAIT0)
    return 0;

  /* If we timed out, inform the firmware that this is the case. */
  if ((val & BNX2_FW_MSG_ACK) != (msg_data & BNX2_DRV_MSG_SEQ)) {
    if (!silent)
      DLOG ("fw sync timeout, reset code = %x\n", msg_data);

    msg_data &= ~BNX2_DRV_MSG_CODE;
    msg_data |= BNX2_DRV_MSG_CODE_FW_TIMEOUT;

    bnx2_shmem_wr(bp, BNX2_DRV_MB, msg_data);

    return -EBUSY;
  }

  if ((val & BNX2_FW_MSG_STATUS_MASK) != BNX2_FW_MSG_STATUS_OK)
    return -EIO;

  return 0;
}

#if 0
static int
bnx2_reset_chip(struct bnx2 *bp, u32 reset_code)
{
  u32 val;
  int i, rc = 0;
  u8 old_port;

  /* Wait for the current PCI transaction to complete before
   * issuing a reset. */
  REG_WR(bp, BNX2_MISC_ENABLE_CLR_BITS,
         BNX2_MISC_ENABLE_CLR_BITS_TX_DMA_ENABLE |
         BNX2_MISC_ENABLE_CLR_BITS_DMA_ENGINE_ENABLE |
         BNX2_MISC_ENABLE_CLR_BITS_RX_DMA_ENABLE |
         BNX2_MISC_ENABLE_CLR_BITS_HOST_COALESCE_ENABLE);
  val = REG_RD(bp, BNX2_MISC_ENABLE_CLR_BITS);
  udelay(5);

  /* Wait for the firmware to tell us it is ok to issue a reset. */
  bnx2_fw_sync(bp, BNX2_DRV_MSG_DATA_WAIT0 | reset_code, 1, 1);

  /* Deposit a driver reset signature so the firmware knows that
   * this is a soft reset. */
  bnx2_shmem_wr(bp, BNX2_DRV_RESET_SIGNATURE,
                BNX2_DRV_RESET_SIGNATURE_MAGIC);

  /* Do a dummy read to force the chip to complete all current transaction
   * before we issue a reset. */
  val = REG_RD(bp, BNX2_MISC_ID);

  if (CHIP_NUM(bp) == CHIP_NUM_5709) {
    REG_WR(bp, BNX2_MISC_COMMAND, BNX2_MISC_COMMAND_SW_RESET);
    REG_RD(bp, BNX2_MISC_COMMAND);
    udelay(5);

    val = BNX2_PCICFG_MISC_CONFIG_REG_WINDOW_ENA |
      BNX2_PCICFG_MISC_CONFIG_TARGET_MB_WORD_SWAP;

    pci_write_config_dword(bp->pdev, BNX2_PCICFG_MISC_CONFIG, val);

  } else {
    val = BNX2_PCICFG_MISC_CONFIG_CORE_RST_REQ |
      BNX2_PCICFG_MISC_CONFIG_REG_WINDOW_ENA |
      BNX2_PCICFG_MISC_CONFIG_TARGET_MB_WORD_SWAP;

    /* Chip reset. */
    REG_WR(bp, BNX2_PCICFG_MISC_CONFIG, val);

    /* Reading back any register after chip reset will hang the
     * bus on 5706 A0 and A1.  The msleep below provides plenty
     * of margin for write posting.
     */
    if ((CHIP_ID(bp) == CHIP_ID_5706_A0) ||
        (CHIP_ID(bp) == CHIP_ID_5706_A1))
      udelay(20*1000);

    /* Reset takes approximate 30 usec */
    for (i = 0; i < 10; i++) {
      val = REG_RD(bp, BNX2_PCICFG_MISC_CONFIG);
      if ((val & (BNX2_PCICFG_MISC_CONFIG_CORE_RST_REQ |
                  BNX2_PCICFG_MISC_CONFIG_CORE_RST_BSY)) == 0)
        break;
      udelay(10);
    }

    if (val & (BNX2_PCICFG_MISC_CONFIG_CORE_RST_REQ |
               BNX2_PCICFG_MISC_CONFIG_CORE_RST_BSY)) {
      pr_err("Chip reset did not complete\n");
      return -EBUSY;
    }
  }

  /* Make sure byte swapping is properly configured. */
  val = REG_RD(bp, BNX2_PCI_SWAP_DIAG0);
  if (val != 0x01020304) {
    pr_err("Chip not in correct endian mode\n");
    return -ENODEV;
  }

  /* Wait for the firmware to finish its initialization. */
  rc = bnx2_fw_sync(bp, BNX2_DRV_MSG_DATA_WAIT1 | reset_code, 1, 0);
  if (rc)
    return rc;

  spin_lock_bh(&bp->phy_lock);
  old_port = bp->phy_port;
  bnx2_init_fw_cap(bp);
  if ((bp->phy_flags & BNX2_PHY_FLAG_REMOTE_PHY_CAP) &&
      old_port != bp->phy_port)
    bnx2_set_default_remote_link(bp);
  spin_unlock_bh(&bp->phy_lock);

  if (CHIP_ID(bp) == CHIP_ID_5706_A0) {
    /* Adjust the voltage regular to two steps lower.  The default
     * of this register is 0x0000000e. */
    REG_WR(bp, BNX2_MISC_VREG_CONTROL, 0x000000fa);

    /* Remove bad rbuf memory from the free pool. */
    rc = bnx2_alloc_bad_rbuf(bp);
  }

  if (bp->flags & BNX2_FLAG_USING_MSIX) {
    bnx2_setup_msix_tbl(bp);
    /* Prevent MSIX table reads and write from timing out */
    REG_WR(bp, BNX2_MISC_ECO_HW_CTL,
           BNX2_MISC_ECO_HW_CTL_LARGE_GRC_TMOUT_EN);
  }

  return rc;
}

static int
bnx2_init_chip(struct bnx2 *bp)
{
  u32 val, mtu;
  int rc, i;

  /* Make sure the interrupt is not active. */
  REG_WR(bp, BNX2_PCICFG_INT_ACK_CMD, BNX2_PCICFG_INT_ACK_CMD_MASK_INT);

  val = BNX2_DMA_CONFIG_DATA_BYTE_SWAP |
    BNX2_DMA_CONFIG_DATA_WORD_SWAP |
#ifdef __BIG_ENDIAN
    BNX2_DMA_CONFIG_CNTL_BYTE_SWAP |
#endif
    BNX2_DMA_CONFIG_CNTL_WORD_SWAP |
    DMA_READ_CHANS << 12 |
    DMA_WRITE_CHANS << 16;

  val |= (0x2 << 20) | (1 << 11);

  if ((bp->flags & BNX2_FLAG_PCIX) && (bp->bus_speed_mhz == 133))
    val |= (1 << 23);

  if ((CHIP_NUM(bp) == CHIP_NUM_5706) &&
      (CHIP_ID(bp) != CHIP_ID_5706_A0) && !(bp->flags & BNX2_FLAG_PCIX))
    val |= BNX2_DMA_CONFIG_CNTL_PING_PONG_DMA;

  REG_WR(bp, BNX2_DMA_CONFIG, val);

  if (CHIP_ID(bp) == CHIP_ID_5706_A0) {
    val = REG_RD(bp, BNX2_TDMA_CONFIG);
    val |= BNX2_TDMA_CONFIG_ONE_DMA;
    REG_WR(bp, BNX2_TDMA_CONFIG, val);
  }

  if (bp->flags & BNX2_FLAG_PCIX) {
    u16 val16;

    pci_read_config_word(bp->pdev, bp->pcix_cap + PCI_X_CMD,
                         &val16);
    pci_write_config_word(bp->pdev, bp->pcix_cap + PCI_X_CMD,
                          val16 & ~PCI_X_CMD_ERO);
  }

  REG_WR(bp, BNX2_MISC_ENABLE_SET_BITS,
         BNX2_MISC_ENABLE_SET_BITS_HOST_COALESCE_ENABLE |
         BNX2_MISC_ENABLE_STATUS_BITS_RX_V2P_ENABLE |
         BNX2_MISC_ENABLE_STATUS_BITS_CONTEXT_ENABLE);

  /* Initialize context mapping and zero out the quick contexts.  The
   * context block must have already been enabled. */
  if (CHIP_NUM(bp) == CHIP_NUM_5709) {
    rc = bnx2_init_5709_context(bp);
    if (rc)
      return rc;
  } else
    bnx2_init_context(bp);

  if ((rc = bnx2_init_cpus(bp)) != 0)
    return rc;

  bnx2_init_nvram(bp);

  bnx2_set_mac_addr(bp, bp->dev->dev_addr, 0);

  val = REG_RD(bp, BNX2_MQ_CONFIG);
  val &= ~BNX2_MQ_CONFIG_KNL_BYP_BLK_SIZE;
  val |= BNX2_MQ_CONFIG_KNL_BYP_BLK_SIZE_256;
  if (CHIP_NUM(bp) == CHIP_NUM_5709) {
    val |= BNX2_MQ_CONFIG_BIN_MQ_MODE;
    if (CHIP_REV(bp) == CHIP_REV_Ax)
      val |= BNX2_MQ_CONFIG_HALT_DIS;
  }

  REG_WR(bp, BNX2_MQ_CONFIG, val);

  val = 0x10000 + (MAX_CID_CNT * MB_KERNEL_CTX_SIZE);
  REG_WR(bp, BNX2_MQ_KNL_BYP_WIND_START, val);
  REG_WR(bp, BNX2_MQ_KNL_WIND_END, val);

  val = (BCM_PAGE_BITS - 8) << 24;
  REG_WR(bp, BNX2_RV2P_CONFIG, val);

  /* Configure page size. */
  val = REG_RD(bp, BNX2_TBDR_CONFIG);
  val &= ~BNX2_TBDR_CONFIG_PAGE_SIZE;
  val |= (BCM_PAGE_BITS - 8) << 24 | 0x40;
  REG_WR(bp, BNX2_TBDR_CONFIG, val);

  val = bp->mac_addr[0] +
    (bp->mac_addr[1] << 8) +
    (bp->mac_addr[2] << 16) +
    bp->mac_addr[3] +
    (bp->mac_addr[4] << 8) +
    (bp->mac_addr[5] << 16);
  REG_WR(bp, BNX2_EMAC_BACKOFF_SEED, val);

  /* Program the MTU.  Also include 4 bytes for CRC32. */
  mtu = bp->dev->mtu;
  val = mtu + ETH_HLEN + ETH_FCS_LEN;
  if (val > (MAX_ETHERNET_PACKET_SIZE + 4))
    val |= BNX2_EMAC_RX_MTU_SIZE_JUMBO_ENA;
  REG_WR(bp, BNX2_EMAC_RX_MTU_SIZE, val);

  if (mtu < 1500)
    mtu = 1500;

  bnx2_reg_wr_ind(bp, BNX2_RBUF_CONFIG, BNX2_RBUF_CONFIG_VAL(mtu));
  bnx2_reg_wr_ind(bp, BNX2_RBUF_CONFIG2, BNX2_RBUF_CONFIG2_VAL(mtu));
  bnx2_reg_wr_ind(bp, BNX2_RBUF_CONFIG3, BNX2_RBUF_CONFIG3_VAL(mtu));

  memset(bp->bnx2_napi[0].status_blk.msi, 0, bp->status_stats_size);
  for (i = 0; i < BNX2_MAX_MSIX_VEC; i++)
    bp->bnx2_napi[i].last_status_idx = 0;

  bp->idle_chk_status_idx = 0xffff;

  bp->rx_mode = BNX2_EMAC_RX_MODE_SORT_MODE;

  /* Set up how to generate a link change interrupt. */
  REG_WR(bp, BNX2_EMAC_ATTENTION_ENA, BNX2_EMAC_ATTENTION_ENA_LINK);

  REG_WR(bp, BNX2_HC_STATUS_ADDR_L,
         (u64) bp->status_blk_mapping & 0xffffffff);
  REG_WR(bp, BNX2_HC_STATUS_ADDR_H, (u64) bp->status_blk_mapping >> 32);

  REG_WR(bp, BNX2_HC_STATISTICS_ADDR_L,
         (u64) bp->stats_blk_mapping & 0xffffffff);
  REG_WR(bp, BNX2_HC_STATISTICS_ADDR_H,
         (u64) bp->stats_blk_mapping >> 32);

  REG_WR(bp, BNX2_HC_TX_QUICK_CONS_TRIP,
         (bp->tx_quick_cons_trip_int << 16) | bp->tx_quick_cons_trip);

  REG_WR(bp, BNX2_HC_RX_QUICK_CONS_TRIP,
         (bp->rx_quick_cons_trip_int << 16) | bp->rx_quick_cons_trip);

  REG_WR(bp, BNX2_HC_COMP_PROD_TRIP,
         (bp->comp_prod_trip_int << 16) | bp->comp_prod_trip);

  REG_WR(bp, BNX2_HC_TX_TICKS, (bp->tx_ticks_int << 16) | bp->tx_ticks);

  REG_WR(bp, BNX2_HC_RX_TICKS, (bp->rx_ticks_int << 16) | bp->rx_ticks);

  REG_WR(bp, BNX2_HC_COM_TICKS,
         (bp->com_ticks_int << 16) | bp->com_ticks);

  REG_WR(bp, BNX2_HC_CMD_TICKS,
         (bp->cmd_ticks_int << 16) | bp->cmd_ticks);

  if (bp->flags & BNX2_FLAG_BROKEN_STATS)
    REG_WR(bp, BNX2_HC_STATS_TICKS, 0);
  else
    REG_WR(bp, BNX2_HC_STATS_TICKS, bp->stats_ticks);
  REG_WR(bp, BNX2_HC_STAT_COLLECT_TICKS, 0xbb8);  /* 3ms */

  if (CHIP_ID(bp) == CHIP_ID_5706_A1)
    val = BNX2_HC_CONFIG_COLLECT_STATS;
  else {
    val = BNX2_HC_CONFIG_RX_TMR_MODE | BNX2_HC_CONFIG_TX_TMR_MODE |
      BNX2_HC_CONFIG_COLLECT_STATS;
  }

  if (bp->flags & BNX2_FLAG_USING_MSIX) {
    REG_WR(bp, BNX2_HC_MSIX_BIT_VECTOR,
           BNX2_HC_MSIX_BIT_VECTOR_VAL);

    val |= BNX2_HC_CONFIG_SB_ADDR_INC_128B;
  }

  if (bp->flags & BNX2_FLAG_ONE_SHOT_MSI)
    val |= BNX2_HC_CONFIG_ONE_SHOT | BNX2_HC_CONFIG_USE_INT_PARAM;

  REG_WR(bp, BNX2_HC_CONFIG, val);

  for (i = 1; i < bp->irq_nvecs; i++) {
    u32 base = ((i - 1) * BNX2_HC_SB_CONFIG_SIZE) +
      BNX2_HC_SB_CONFIG_1;

    REG_WR(bp, base,
           BNX2_HC_SB_CONFIG_1_TX_TMR_MODE |
           BNX2_HC_SB_CONFIG_1_RX_TMR_MODE |
           BNX2_HC_SB_CONFIG_1_ONE_SHOT);

    REG_WR(bp, base + BNX2_HC_TX_QUICK_CONS_TRIP_OFF,
           (bp->tx_quick_cons_trip_int << 16) |
           bp->tx_quick_cons_trip);

    REG_WR(bp, base + BNX2_HC_TX_TICKS_OFF,
           (bp->tx_ticks_int << 16) | bp->tx_ticks);

    REG_WR(bp, base + BNX2_HC_RX_QUICK_CONS_TRIP_OFF,
           (bp->rx_quick_cons_trip_int << 16) |
           bp->rx_quick_cons_trip);

    REG_WR(bp, base + BNX2_HC_RX_TICKS_OFF,
           (bp->rx_ticks_int << 16) | bp->rx_ticks);
  }

  /* Clear internal stats counters. */
  REG_WR(bp, BNX2_HC_COMMAND, BNX2_HC_COMMAND_CLR_STAT_NOW);

  REG_WR(bp, BNX2_HC_ATTN_BITS_ENABLE, STATUS_ATTN_EVENTS);

  /* Initialize the receive filter. */
  bnx2_set_rx_mode(bp->dev);

  if (CHIP_NUM(bp) == CHIP_NUM_5709) {
    val = REG_RD(bp, BNX2_MISC_NEW_CORE_CTL);
    val |= BNX2_MISC_NEW_CORE_CTL_DMA_ENABLE;
    REG_WR(bp, BNX2_MISC_NEW_CORE_CTL, val);
  }
  rc = bnx2_fw_sync(bp, BNX2_DRV_MSG_DATA_WAIT2 | BNX2_DRV_MSG_CODE_RESET,
                    1, 0);

  REG_WR(bp, BNX2_MISC_ENABLE_SET_BITS, BNX2_MISC_ENABLE_DEFAULT);
  REG_RD(bp, BNX2_MISC_ENABLE_SET_BITS);

  udelay(20);

  bp->hc_cmd = REG_RD(bp, BNX2_HC_COMMAND);

  return rc;
}

static void
bnx2_clear_ring_states(struct bnx2 *bp)
{
  struct bnx2_napi *bnapi;
  struct bnx2_tx_ring_info *txr;
  struct bnx2_rx_ring_info *rxr;
  int i;

  for (i = 0; i < BNX2_MAX_MSIX_VEC; i++) {
    bnapi = &bp->bnx2_napi[i];
    txr = &bnapi->tx_ring;
    rxr = &bnapi->rx_ring;

    txr->tx_cons = 0;
    txr->hw_tx_cons = 0;
    rxr->rx_prod_bseq = 0;
    rxr->rx_prod = 0;
    rxr->rx_cons = 0;
    rxr->rx_pg_prod = 0;
    rxr->rx_pg_cons = 0;
  }
}

static void
bnx2_init_tx_context(struct bnx2 *bp, u32 cid, struct bnx2_tx_ring_info *txr)
{
  u32 val, offset0, offset1, offset2, offset3;
  u32 cid_addr = GET_CID_ADDR(cid);

  if (CHIP_NUM(bp) == CHIP_NUM_5709) {
    offset0 = BNX2_L2CTX_TYPE_XI;
    offset1 = BNX2_L2CTX_CMD_TYPE_XI;
    offset2 = BNX2_L2CTX_TBDR_BHADDR_HI_XI;
    offset3 = BNX2_L2CTX_TBDR_BHADDR_LO_XI;
  } else {
    offset0 = BNX2_L2CTX_TYPE;
    offset1 = BNX2_L2CTX_CMD_TYPE;
    offset2 = BNX2_L2CTX_TBDR_BHADDR_HI;
    offset3 = BNX2_L2CTX_TBDR_BHADDR_LO;
  }
  val = BNX2_L2CTX_TYPE_TYPE_L2 | BNX2_L2CTX_TYPE_SIZE_L2;
  bnx2_ctx_wr(bp, cid_addr, offset0, val);

  val = BNX2_L2CTX_CMD_TYPE_TYPE_L2 | (8 << 16);
  bnx2_ctx_wr(bp, cid_addr, offset1, val);

  val = (u64) txr->tx_desc_mapping >> 32;
  bnx2_ctx_wr(bp, cid_addr, offset2, val);

  val = (u64) txr->tx_desc_mapping & 0xffffffff;
  bnx2_ctx_wr(bp, cid_addr, offset3, val);
}

static void
bnx2_init_tx_ring(struct bnx2 *bp, int ring_num)
{
  struct tx_bd *txbd;
  u32 cid = TX_CID;
  struct bnx2_napi *bnapi;
  struct bnx2_tx_ring_info *txr;

  bnapi = &bp->bnx2_napi[ring_num];
  txr = &bnapi->tx_ring;

  if (ring_num == 0)
    cid = TX_CID;
  else
    cid = TX_TSS_CID + ring_num - 1;

  bp->tx_wake_thresh = bp->tx_ring_size / 2;

  txbd = &txr->tx_desc_ring[MAX_TX_DESC_CNT];

  txbd->tx_bd_haddr_hi = (u64) txr->tx_desc_mapping >> 32;
  txbd->tx_bd_haddr_lo = (u64) txr->tx_desc_mapping & 0xffffffff;

  txr->tx_prod = 0;
  txr->tx_prod_bseq = 0;

  txr->tx_bidx_addr = MB_GET_CID_ADDR(cid) + BNX2_L2CTX_TX_HOST_BIDX;
  txr->tx_bseq_addr = MB_GET_CID_ADDR(cid) + BNX2_L2CTX_TX_HOST_BSEQ;

  bnx2_init_tx_context(bp, cid, txr);
}

static void
bnx2_init_rxbd_rings(struct rx_bd *rx_ring[], dma_addr_t dma[], u32 buf_size,
                     int num_rings)
{
  int i;
  struct rx_bd *rxbd;

  for (i = 0; i < num_rings; i++) {
    int j;

    rxbd = &rx_ring[i][0];
    for (j = 0; j < MAX_RX_DESC_CNT; j++, rxbd++) {
      rxbd->rx_bd_len = buf_size;
      rxbd->rx_bd_flags = RX_BD_FLAGS_START | RX_BD_FLAGS_END;
    }
    if (i == (num_rings - 1))
      j = 0;
    else
      j = i + 1;
    rxbd->rx_bd_haddr_hi = (u64) dma[j] >> 32;
    rxbd->rx_bd_haddr_lo = (u64) dma[j] & 0xffffffff;
  }
}

static void
bnx2_init_rx_ring(struct bnx2 *bp, int ring_num)
{
  int i;
  u16 prod, ring_prod;
  u32 cid, rx_cid_addr, val;
  struct bnx2_napi *bnapi = &bp->bnx2_napi[ring_num];
  struct bnx2_rx_ring_info *rxr = &bnapi->rx_ring;

  if (ring_num == 0)
    cid = RX_CID;
  else
    cid = RX_RSS_CID + ring_num - 1;

  rx_cid_addr = GET_CID_ADDR(cid);

  bnx2_init_rxbd_rings(rxr->rx_desc_ring, rxr->rx_desc_mapping,
                       bp->rx_buf_use_size, bp->rx_max_ring);

  bnx2_init_rx_context(bp, cid);

  if (CHIP_NUM(bp) == CHIP_NUM_5709) {
    val = REG_RD(bp, BNX2_MQ_MAP_L2_5);
    REG_WR(bp, BNX2_MQ_MAP_L2_5, val | BNX2_MQ_MAP_L2_5_ARM);
  }

  bnx2_ctx_wr(bp, rx_cid_addr, BNX2_L2CTX_PG_BUF_SIZE, 0);
  if (bp->rx_pg_ring_size) {
    bnx2_init_rxbd_rings(rxr->rx_pg_desc_ring,
                         rxr->rx_pg_desc_mapping,
                         PAGE_SIZE, bp->rx_max_pg_ring);
    val = (bp->rx_buf_use_size << 16) | PAGE_SIZE;
    bnx2_ctx_wr(bp, rx_cid_addr, BNX2_L2CTX_PG_BUF_SIZE, val);
    bnx2_ctx_wr(bp, rx_cid_addr, BNX2_L2CTX_RBDC_KEY,
                BNX2_L2CTX_RBDC_JUMBO_KEY - ring_num);

    val = (u64) rxr->rx_pg_desc_mapping[0] >> 32;
    bnx2_ctx_wr(bp, rx_cid_addr, BNX2_L2CTX_NX_PG_BDHADDR_HI, val);

    val = (u64) rxr->rx_pg_desc_mapping[0] & 0xffffffff;
    bnx2_ctx_wr(bp, rx_cid_addr, BNX2_L2CTX_NX_PG_BDHADDR_LO, val);

    if (CHIP_NUM(bp) == CHIP_NUM_5709)
      REG_WR(bp, BNX2_MQ_MAP_L2_3, BNX2_MQ_MAP_L2_3_DEFAULT);
  }

  val = (u64) rxr->rx_desc_mapping[0] >> 32;
  bnx2_ctx_wr(bp, rx_cid_addr, BNX2_L2CTX_NX_BDHADDR_HI, val);

  val = (u64) rxr->rx_desc_mapping[0] & 0xffffffff;
  bnx2_ctx_wr(bp, rx_cid_addr, BNX2_L2CTX_NX_BDHADDR_LO, val);

  ring_prod = prod = rxr->rx_pg_prod;
  for (i = 0; i < bp->rx_pg_ring_size; i++) {
    if (bnx2_alloc_rx_page(bp, rxr, ring_prod, GFP_KERNEL) < 0) {
      netdev_warn(bp->dev, "init'ed rx page ring %d with %d/%d pages only\n",
                  ring_num, i, bp->rx_pg_ring_size);
      break;
    }
    prod = NEXT_RX_BD(prod);
    ring_prod = RX_PG_RING_IDX(prod);
  }
  rxr->rx_pg_prod = prod;

  ring_prod = prod = rxr->rx_prod;
  for (i = 0; i < bp->rx_ring_size; i++) {
    if (bnx2_alloc_rx_skb(bp, rxr, ring_prod, GFP_KERNEL) < 0) {
      netdev_warn(bp->dev, "init'ed rx ring %d with %d/%d skbs only\n",
                  ring_num, i, bp->rx_ring_size);
      break;
    }
    prod = NEXT_RX_BD(prod);
    ring_prod = RX_RING_IDX(prod);
  }
  rxr->rx_prod = prod;

  rxr->rx_bidx_addr = MB_GET_CID_ADDR(cid) + BNX2_L2CTX_HOST_BDIDX;
  rxr->rx_bseq_addr = MB_GET_CID_ADDR(cid) + BNX2_L2CTX_HOST_BSEQ;
  rxr->rx_pg_bidx_addr = MB_GET_CID_ADDR(cid) + BNX2_L2CTX_HOST_PG_BDIDX;

  REG_WR16(bp, rxr->rx_pg_bidx_addr, rxr->rx_pg_prod);
  REG_WR16(bp, rxr->rx_bidx_addr, prod);

  REG_WR(bp, rxr->rx_bseq_addr, rxr->rx_prod_bseq);
}

static void
bnx2_init_all_rings(struct bnx2 *bp)
{
  int i;
  u32 val;

  bnx2_clear_ring_states(bp);

  REG_WR(bp, BNX2_TSCH_TSS_CFG, 0);
  for (i = 0; i < bp->num_tx_rings; i++)
    bnx2_init_tx_ring(bp, i);

  if (bp->num_tx_rings > 1)
    REG_WR(bp, BNX2_TSCH_TSS_CFG, ((bp->num_tx_rings - 1) << 24) |
           (TX_TSS_CID << 7));

  REG_WR(bp, BNX2_RLUP_RSS_CONFIG, 0);
  bnx2_reg_wr_ind(bp, BNX2_RXP_SCRATCH_RSS_TBL_SZ, 0);

  for (i = 0; i < bp->num_rx_rings; i++)
    bnx2_init_rx_ring(bp, i);

  if (bp->num_rx_rings > 1) {
    u32 tbl_32;
    u8 *tbl = (u8 *) &tbl_32;

    bnx2_reg_wr_ind(bp, BNX2_RXP_SCRATCH_RSS_TBL_SZ,
                    BNX2_RXP_SCRATCH_RSS_TBL_MAX_ENTRIES);

    for (i = 0; i < BNX2_RXP_SCRATCH_RSS_TBL_MAX_ENTRIES; i++) {
      tbl[i % 4] = i % (bp->num_rx_rings - 1);
      if ((i % 4) == 3)
        bnx2_reg_wr_ind(bp,
                        BNX2_RXP_SCRATCH_RSS_TBL + i,
                        cpu_to_be32(tbl_32));
    }

    val = BNX2_RLUP_RSS_CONFIG_IPV4_RSS_TYPE_ALL_XI |
      BNX2_RLUP_RSS_CONFIG_IPV6_RSS_TYPE_ALL_XI;

    REG_WR(bp, BNX2_RLUP_RSS_CONFIG, val);

  }
}

static u32 bnx2_find_max_ring(u32 ring_size, u32 max_size)
{
  u32 max, num_rings = 1;

  while (ring_size > MAX_RX_DESC_CNT) {
    ring_size -= MAX_RX_DESC_CNT;
    num_rings++;
  }
  /* round to next power of 2 */
  max = max_size;
  while ((max & num_rings) == 0)
    max >>= 1;

  if (num_rings != max)
    max <<= 1;

  return max;
}

static void
bnx2_set_rx_ring_size(struct bnx2 *bp, u32 size)
{
  u32 rx_size, rx_space, jumbo_size;

  /* 8 for CRC and VLAN */
  rx_size = bp->dev->mtu + ETH_HLEN + BNX2_RX_OFFSET + 8;

  rx_space = SKB_DATA_ALIGN(rx_size + BNX2_RX_ALIGN) + NET_SKB_PAD +
    sizeof(struct skb_shared_info);

  bp->rx_copy_thresh = BNX2_RX_COPY_THRESH;
  bp->rx_pg_ring_size = 0;
  bp->rx_max_pg_ring = 0;
  bp->rx_max_pg_ring_idx = 0;
  if ((rx_space > PAGE_SIZE) && !(bp->flags & BNX2_FLAG_JUMBO_BROKEN)) {
    int pages = PAGE_ALIGN(bp->dev->mtu - 40) >> PAGE_SHIFT;

    jumbo_size = size * pages;
    if (jumbo_size > MAX_TOTAL_RX_PG_DESC_CNT)
      jumbo_size = MAX_TOTAL_RX_PG_DESC_CNT;

    bp->rx_pg_ring_size = jumbo_size;
    bp->rx_max_pg_ring = bnx2_find_max_ring(jumbo_size,
                                            MAX_RX_PG_RINGS);
    bp->rx_max_pg_ring_idx = (bp->rx_max_pg_ring * RX_DESC_CNT) - 1;
    rx_size = BNX2_RX_COPY_THRESH + BNX2_RX_OFFSET;
    bp->rx_copy_thresh = 0;
  }

  bp->rx_buf_use_size = rx_size;
  /* hw alignment */
  bp->rx_buf_size = bp->rx_buf_use_size + BNX2_RX_ALIGN;
  bp->rx_jumbo_thresh = rx_size - BNX2_RX_OFFSET;
  bp->rx_ring_size = size;
  bp->rx_max_ring = bnx2_find_max_ring(size, MAX_RX_RINGS);
  bp->rx_max_ring_idx = (bp->rx_max_ring * RX_DESC_CNT) - 1;
}

static void
bnx2_free_tx_skbs(struct bnx2 *bp)
{
  int i;

  for (i = 0; i < bp->num_tx_rings; i++) {
    struct bnx2_napi *bnapi = &bp->bnx2_napi[i];
    struct bnx2_tx_ring_info *txr = &bnapi->tx_ring;
    int j;

    if (txr->tx_buf_ring == NULL)
      continue;

    for (j = 0; j < TX_DESC_CNT; ) {
      struct sw_tx_bd *tx_buf = &txr->tx_buf_ring[j];
      struct sk_buff *skb = tx_buf->skb;
      int k, last;

      if (skb == NULL) {
        j++;
        continue;
      }

      dma_unmap_single(&bp->pdev->dev,
                       dma_unmap_addr(tx_buf, mapping),
                       skb_headlen(skb),
                       PCI_DMA_TODEVICE);

      tx_buf->skb = NULL;

      last = tx_buf->nr_frags;
      j++;
      for (k = 0; k < last; k++, j++) {
        tx_buf = &txr->tx_buf_ring[TX_RING_IDX(j)];
        dma_unmap_page(&bp->pdev->dev,
                       dma_unmap_addr(tx_buf, mapping),
                       skb_shinfo(skb)->frags[k].size,
                       PCI_DMA_TODEVICE);
      }
      dev_kfree_skb(skb);
    }
  }
}

static void
bnx2_free_rx_skbs(struct bnx2 *bp)
{
  int i;

  for (i = 0; i < bp->num_rx_rings; i++) {
    struct bnx2_napi *bnapi = &bp->bnx2_napi[i];
    struct bnx2_rx_ring_info *rxr = &bnapi->rx_ring;
    int j;

    if (rxr->rx_buf_ring == NULL)
      return;

    for (j = 0; j < bp->rx_max_ring_idx; j++) {
      struct sw_bd *rx_buf = &rxr->rx_buf_ring[j];
      struct sk_buff *skb = rx_buf->skb;

      if (skb == NULL)
        continue;

      dma_unmap_single(&bp->pdev->dev,
                       dma_unmap_addr(rx_buf, mapping),
                       bp->rx_buf_use_size,
                       PCI_DMA_FROMDEVICE);

      rx_buf->skb = NULL;

      dev_kfree_skb(skb);
    }
    for (j = 0; j < bp->rx_max_pg_ring_idx; j++)
      bnx2_free_rx_page(bp, rxr, j);
  }
}

static void
bnx2_free_skbs(struct bnx2 *bp)
{
  bnx2_free_tx_skbs(bp);
  bnx2_free_rx_skbs(bp);
}

static int
bnx2_reset_nic(struct bnx2 *bp, u32 reset_code)
{
  int rc;

  rc = bnx2_reset_chip(bp, reset_code);
  bnx2_free_skbs(bp);
  if (rc)
    return rc;

  if ((rc = bnx2_init_chip(bp)) != 0)
    return rc;

  bnx2_init_all_rings(bp);
  return 0;
}

static int
bnx2_init_nic(struct bnx2 *bp, int reset_phy)
{
  int rc;

  if ((rc = bnx2_reset_nic(bp, BNX2_DRV_MSG_CODE_RESET)) != 0)
    return rc;

  spinlock_lock(&bp->phy_lock);
  /*********************************
   * bnx2_init_phy(bp, reset_phy); *
   * bnx2_set_link(bp);            *
   *********************************/
  /*****************************************************
   * if (bp->phy_flags & BNX2_PHY_FLAG_REMOTE_PHY_CAP) *
   *   bnx2_remote_phy_event(bp);                      *
   *****************************************************/
  spinlock_unlock(&bp->phy_lock);
  return 0;
}
#endif

static void
bnx2_mac_reset (struct bnx2 *bp)
{
  int i;

  REG_WR(bp, BNX2_MISC_ENABLE_CLR_BITS,
         BNX2_MISC_ENABLE_CLR_BITS_TX_DMA_ENABLE |
         BNX2_MISC_ENABLE_CLR_BITS_DMA_ENGINE_ENABLE |
         BNX2_MISC_ENABLE_CLR_BITS_RX_DMA_ENABLE |
         BNX2_MISC_ENABLE_CLR_BITS_HOST_COALESCE_ENABLE);
  u32 val = REG_RD(bp, BNX2_MISC_ENABLE_CLR_BITS);
  udelay(5);

#define DRV_RESET_SIGNATURE 0x4841564b /* HAVK */
  bnx2_reg_wr_ind(bp, HOST_VIEW_SHMEM_BASE +
                  //offsetof(shmem_region_t, drv_fw_mb.drv_reset_signature),
                  0,
                  DRV_RESET_SIGNATURE);

  bnx2_fw_sync(bp, BNX2_DRV_MSG_DATA_WAIT0 | BNX2_DRV_MSG_CODE_RESET, 1, 1);

  val = REG_RD(bp, BNX2_MISC_ID);
  val =
    BNX2_PCICFG_MISC_CONFIG_CORE_RST_REQ |
    BNX2_PCICFG_MISC_CONFIG_REG_WINDOW_ENA |
    BNX2_PCICFG_MISC_CONFIG_TARGET_MB_WORD_SWAP;
  REG_WR(bp, BNX2_PCICFG_MISC_CONFIG, val);

  if ((CHIP_ID(bp) == CHIP_ID_5706_A0) || (CHIP_ID(bp) == CHIP_ID_5706_A1)) {
    for (i = 0; i < 500; i++) {
      udelay(30);
    }
  }

  for (i = 0; i < 10; i++) {
    val = REG_RD(bp, BNX2_PCICFG_MISC_CONFIG);
    if ((val & (BNX2_PCICFG_MISC_CONFIG_CORE_RST_REQ |
                BNX2_PCICFG_MISC_CONFIG_CORE_RST_BSY)) == 0) {
      break;
    }
    udelay(10);
  }

  bnx2_fw_sync(bp, BNX2_DRV_MSG_DATA_WAIT1 | BNX2_DRV_MSG_CODE_RESET, 1, 0);
}

static u32
rv2p_fw_fixup(u32 rv2p_proc, int idx, u32 loc, u32 rv2p_code)
{
  switch (idx) {
  case RV2P_P1_FIXUP_PAGE_SIZE_IDX:
    rv2p_code &= ~RV2P_BD_PAGE_SIZE_MSK;
    rv2p_code |= RV2P_BD_PAGE_SIZE;
    break;
  }
  return rv2p_code;
}

#define RV2P_PROC1 0
#define RV2P_PROC2 1
static void
load_rv2p_fw(struct bnx2 *bp, __le32 *rv2p_code, u32 rv2p_code_len,
             u32 rv2p_proc, u32 fixup_loc)
{
  __le32 *rv2p_code_start = rv2p_code;
  int i;
  u32 val, cmd, addr;

  if (rv2p_proc == RV2P_PROC1) {
    cmd = BNX2_RV2P_PROC1_ADDR_CMD_RDWR;
    addr = BNX2_RV2P_PROC1_ADDR_CMD;
  } else {
    cmd = BNX2_RV2P_PROC2_ADDR_CMD_RDWR;
    addr = BNX2_RV2P_PROC2_ADDR_CMD;
  }

  for (i = 0; i < rv2p_code_len; i += 8) {
    REG_WR(bp, BNX2_RV2P_INSTR_HIGH, __le32_to_cpu(*rv2p_code));
    rv2p_code++;
    REG_WR(bp, BNX2_RV2P_INSTR_LOW, __le32_to_cpu(*rv2p_code));
    rv2p_code++;

    val = (i / 8) | cmd;
    REG_WR(bp, addr, val);
  }

  rv2p_code = rv2p_code_start;
  if (fixup_loc && ((fixup_loc * 4) < rv2p_code_len)) {
    u32 code;

    code = __le32_to_cpu(*(rv2p_code + fixup_loc - 1));
    REG_WR(bp, BNX2_RV2P_INSTR_HIGH, code);
    code = __le32_to_cpu(*(rv2p_code + fixup_loc));
    code = rv2p_fw_fixup(rv2p_proc, 0, fixup_loc, code);
    REG_WR(bp, BNX2_RV2P_INSTR_LOW, code);

    val = (fixup_loc / 2) | cmd;
    REG_WR(bp, addr, val);
  }

  /* Reset the processor, un-stall is done later. */
  if (rv2p_proc == RV2P_PROC1) {
    REG_WR(bp, BNX2_RV2P_COMMAND, BNX2_RV2P_COMMAND_PROC1_RESET);
  }
  else {
    REG_WR(bp, BNX2_RV2P_COMMAND, BNX2_RV2P_COMMAND_PROC2_RESET);
  }
}
#define RV2P_PROC1_MAX_BD_PAGE_LOC   9
#define RV2P_PROC1_BD_PAGE_SIZE_MSK	0xffff
#define RV2P_PROC1_BD_PAGE_SIZE	((BCM_PAGE_SIZE / 16) - 1)
#define RV2P_PROC2_MAX_BD_PAGE_LOC   5
#define RV2P_PROC2_BD_PAGE_SIZE_MSK	0xffff
#define RV2P_PROC2_BD_PAGE_SIZE	((BCM_PAGE_SIZE / 16) - 1)
#define XI_RV2P_PROC1_MAX_BD_PAGE_LOC   9
#define XI_RV2P_PROC1_BD_PAGE_SIZE_MSK	0xffff
#define XI_RV2P_PROC1_BD_PAGE_SIZE	((BCM_PAGE_SIZE / 16) - 1)
#define XI90_RV2P_PROC1_MAX_BD_PAGE_LOC   9
#define XI90_RV2P_PROC1_BD_PAGE_SIZE_MSK	0xffff
#define XI90_RV2P_PROC1_BD_PAGE_SIZE	((BCM_PAGE_SIZE / 16) - 1)
#define XI_RV2P_PROC2_MAX_BD_PAGE_LOC   5
#define XI_RV2P_PROC2_BD_PAGE_SIZE_MSK	0xffff
#define XI_RV2P_PROC2_BD_PAGE_SIZE	((BCM_PAGE_SIZE / 16) - 1)
#define XI90_RV2P_PROC2_MAX_BD_PAGE_LOC   5
#define XI90_RV2P_PROC2_BD_PAGE_SIZE_MSK	0xffff
#define XI90_RV2P_PROC2_BD_PAGE_SIZE	((BCM_PAGE_SIZE / 16) - 1)

static const u32 bnx2_COM_b06FwData[(0x0/4) + 1] = { 0x0 };
static const u32 bnx2_COM_b06FwRodata[(0x14/4) + 1] = {
	0x08000d98, 0x08000de0, 0x08000e20, 0x08000e6c, 0x08000ea0, 0x00000000
};
static const u32 bnx2_CP_b06FwData[(0x84/4) + 1] = {
	0x00000000, 0x0000001b, 0x0000000f, 0x0000000a, 0x00000008, 0x00000006,
	0x00000005, 0x00000005, 0x00000004, 0x00000004, 0x00000003, 0x00000003,
	0x00000003, 0x00000003, 0x00000003, 0x00000002, 0x00000002, 0x00000002,
	0x00000002, 0x00000002, 0x00000002, 0x00000002, 0x00000002, 0x00000002,
	0x00000002, 0x00000002, 0x00000002, 0x00000002, 0x00000002, 0x00000002,
	0x00000001, 0x00000001, 0x00000001, 0x00000000 };
static const u32 bnx2_CP_b06FwRodata[(0x154/4) + 1] = {
	0x08000f58, 0x08000db0, 0x08000fec, 0x08001094, 0x08000f80, 0x08000fc0,
	0x080011cc, 0x08000dcc, 0x080011f0, 0x08000e1c, 0x08001634, 0x080015dc,
	0x08000dcc, 0x08000dcc, 0x08000dcc, 0x0800127c, 0x0800127c, 0x08000dcc,
	0x08000dcc, 0x08001580, 0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc,
	0x080013f0, 0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc,
	0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc,
	0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000fe0, 0x08000dcc, 0x08000dcc,
	0x08001530, 0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc,
	0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc,
	0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc, 0x08000dcc,
	0x0800145c, 0x08000dcc, 0x08000dcc, 0x08001370, 0x080012e0, 0x08002e94,
	0x08002e9c, 0x08002e64, 0x08002e70, 0x08002e7c, 0x08002e88, 0x080046b4,
	0x08003f00, 0x08004634, 0x080046b4, 0x080046b4, 0x080044b4, 0x080046b4,
	0x080046fc, 0x08005524, 0x080054e4, 0x080054b0, 0x08005484, 0x08005460,
	0x0800541c, 0x00000000 };
static const u32 bnx2_RXP_b06FwData[(0x0/4) + 1] = { 0x0 };
static const u32 bnx2_RXP_b06FwRodata[(0x24/4) + 1] = {
	0x080033f8, 0x080033f8, 0x08003370, 0x080033a8, 0x080033dc, 0x08003400,
	0x08003400, 0x08003400, 0x080032e0, 0x00000000 };
static const u32 bnx2_TPAT_b06FwData[(0x0/4) + 1] = { 0x0 };
static const u32 bnx2_TPAT_b06FwRodata[(0x0/4) + 1] = { 0x0 };
static const u32 bnx2_TXP_b06FwData[(0x0/4) + 1] = { 0x0 };
static const u32 bnx2_TXP_b06FwRodata[(0x0/4) + 1] = { 0x0 };

static const u32 bnx2_COM_b09FwData[(0x0/4) + 1] = { 0x0 };
static const u32 bnx2_COM_b09FwRodata[(0x38/4) + 1] = {
	0x80080100, 0x80080080, 0x80080000, 0x00000c80, 0x00003200, 0x80080240,
	0x08000f10, 0x08000f68, 0x08000fac, 0x08001044, 0x08001084, 0x80080100,
	0x80080080, 0x80080000, 0x00000000 };
static const u32 bnx2_CP_b09FwData[(0x84/4) + 1] = {
	0x00000000, 0x0000001b, 0x0000000f, 0x0000000a, 0x00000008, 0x00000006,
	0x00000005, 0x00000005, 0x00000004, 0x00000004, 0x00000003, 0x00000003,
	0x00000003, 0x00000003, 0x00000003, 0x00000002, 0x00000002, 0x00000002,
	0x00000002, 0x00000002, 0x00000002, 0x00000002, 0x00000002, 0x00000002,
	0x00000002, 0x00000002, 0x00000002, 0x00000002, 0x00000002, 0x00000002,
	0x00000001, 0x00000001, 0x00000001, 0x00000000 };
static const u32 bnx2_CP_b09FwRodata[(0x1c0/4) + 1] = {
	0x80080100, 0x80080080, 0x80080000, 0x00000c00, 0x00003080, 0x08001020,
	0x080010cc, 0x080010e4, 0x080010f8, 0x0800110c, 0x08001020, 0x08001020,
	0x08001140, 0x08001178, 0x08001188, 0x080011b0, 0x080018a0, 0x080018a0,
	0x080018d8, 0x080018d8, 0x080018ec, 0x080018bc, 0x08001b14, 0x08001ae0,
	0x08001b6c, 0x08001b6c, 0x08001bf4, 0x08001b24, 0x80080240, 0x08002280,
	0x080020cc, 0x080022a8, 0x08002340, 0x08002490, 0x080024dc, 0x08002600,
	0x08002508, 0x0800258c, 0x0800213c, 0x08002aa8, 0x08002a4c, 0x080020e8,
	0x080020e8, 0x080020e8, 0x08002674, 0x08002674, 0x080020e8, 0x080020e8,
	0x08002924, 0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8, 0x08002984,
	0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8,
	0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8,
	0x080020e8, 0x080020e8, 0x080024fc, 0x080020e8, 0x080020e8, 0x080029f4,
	0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8,
	0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8,
	0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8, 0x080020e8, 0x08002848,
	0x080020e8, 0x080020e8, 0x080027bc, 0x08002718, 0x08003860, 0x08003834,
	0x08003800, 0x080037d4, 0x080037b4, 0x08003768, 0x80080100, 0x80080080,
	0x80080000, 0x80080080, 0x080047c8, 0x08004800, 0x08004748, 0x080047c8,
	0x080047c8, 0x08004528, 0x080047c8, 0x08004b9c, 0x00000000 };
static const u32 bnx2_RXP_b09FwData[(0x0/4) + 1] = { 0x0 };
static const u32 bnx2_RXP_b09FwRodata[(0x124/4) + 1] = {
	0x0800330c, 0x0800330c, 0x080033e8, 0x080033bc, 0x080033a0, 0x080032f0,
	0x080032f0, 0x080032f0, 0x08003314, 0x80080100, 0x80080080, 0x80080000,
	0x5f865437, 0xe4ac62cc, 0x50103a45, 0x36621985, 0xbf14c0e8, 0x1bc27a1e,
	0x84f4b556, 0x094ea6fe, 0x7dda01e7, 0xc04d7481, 0x08007a88, 0x08007ab4,
	0x08007a94, 0x080079d0, 0x08007a94, 0x08007ad4, 0x08007a94, 0x080079d0,
	0x080079d0, 0x080079d0, 0x080079d0, 0x080079d0, 0x080079d0, 0x080079d0,
	0x080079d0, 0x080079d0, 0x080079d0, 0x08007ac4, 0x08007aa4, 0x080079d0,
	0x080079d0, 0x080079d0, 0x080079d0, 0x080079d0, 0x080079d0, 0x080079d0,
	0x080079d0, 0x080079d0, 0x080079d0, 0x080079d0, 0x080079d0, 0x08007aa4,
	0x08008090, 0x08007f38, 0x08008058, 0x08007f38, 0x08008028, 0x08007e20,
	0x08007f38, 0x08007f38, 0x08007f38, 0x08007f38, 0x08007f38, 0x08007f38,
	0x08007f38, 0x08007f38, 0x08007f38, 0x08007f38, 0x08007f38, 0x08007f38,
	0x08007f60, 0x00000000 };
static const u32 bnx2_TPAT_b09FwData[(0x0/4) + 1] = { 0x0 };
static const u32 bnx2_TPAT_b09FwRodata[(0x4/4) + 1] = {
	0x00000001, 0x00000000 };
static const u32 bnx2_TXP_b09FwData[(0x0/4) + 1] = { 0x0 };
static const u32 bnx2_TXP_b09FwRodata[(0x30/4) + 1] = {
	0x80000940, 0x80000900, 0x80080100, 0x80080080, 0x80080000, 0x800e0000,
	0x80080080, 0x80080000, 0x80000a80, 0x80000a00, 0x80000980, 0x80000900,
	0x00000000 };

struct fw_info {
	const u32 ver_major;
	const u32 ver_minor;
	const u32 ver_fix;

	const u32 start_addr;

	/* Text section. */
	const u32 text_addr;
	const u32 text_len;
	const u32 text_index;
	__le32 *text;
	const u8 *gz_text;
	const u32 gz_text_len;

	/* Data section. */
	const u32 data_addr;
	const u32 data_len;
	const u32 data_index;
	const u32 *data;

	/* SBSS section. */
	const u32 sbss_addr;
	const u32 sbss_len;
	const u32 sbss_index;

	/* BSS section. */
	const u32 bss_addr;
	const u32 bss_len;
	const u32 bss_index;

	/* Read-only section. */
	const u32 rodata_addr;
	const u32 rodata_len;
	const u32 rodata_index;
	const u32 *rodata;
};

struct cpu_reg {
	u32 mode;
	u32 mode_value_halt;
	u32 mode_value_sstep;

	u32 state;
	u32 state_value_clear;

	u32 gpr0;
	u32 evmask;
	u32 pc;
	u32 inst;
	u32 bp;

	u32 spad_base;

	u32 mips_view_base;
};

/* Initialized Values for the Completion Processor. */
static const struct cpu_reg cpu_reg_com = {
	.mode = BNX2_COM_CPU_MODE,
	.mode_value_halt = BNX2_COM_CPU_MODE_SOFT_HALT,
	.mode_value_sstep = BNX2_COM_CPU_MODE_STEP_ENA,
	.state = BNX2_COM_CPU_STATE,
	.state_value_clear = 0xffffff,
	.gpr0 = BNX2_COM_CPU_REG_FILE,
	.evmask = BNX2_COM_CPU_EVENT_MASK,
	.pc = BNX2_COM_CPU_PROGRAM_COUNTER,
	.inst = BNX2_COM_CPU_INSTRUCTION,
	.bp = BNX2_COM_CPU_HW_BREAKPOINT,
	.spad_base = BNX2_COM_SCRATCH,
	.mips_view_base = 0x8000000,
};

/* Initialized Values the Command Processor. */
static const struct cpu_reg cpu_reg_cp = {
	.mode = BNX2_CP_CPU_MODE,
	.mode_value_halt = BNX2_CP_CPU_MODE_SOFT_HALT,
	.mode_value_sstep = BNX2_CP_CPU_MODE_STEP_ENA,
	.state = BNX2_CP_CPU_STATE,
	.state_value_clear = 0xffffff,
	.gpr0 = BNX2_CP_CPU_REG_FILE,
	.evmask = BNX2_CP_CPU_EVENT_MASK,
	.pc = BNX2_CP_CPU_PROGRAM_COUNTER,
	.inst = BNX2_CP_CPU_INSTRUCTION,
	.bp = BNX2_CP_CPU_HW_BREAKPOINT,
	.spad_base = BNX2_CP_SCRATCH,
	.mips_view_base = 0x8000000,
};

/* Initialized Values for the RX Processor. */
static const struct cpu_reg cpu_reg_rxp = {
	.mode = BNX2_RXP_CPU_MODE,
	.mode_value_halt = BNX2_RXP_CPU_MODE_SOFT_HALT,
	.mode_value_sstep = BNX2_RXP_CPU_MODE_STEP_ENA,
	.state = BNX2_RXP_CPU_STATE,
	.state_value_clear = 0xffffff,
	.gpr0 = BNX2_RXP_CPU_REG_FILE,
	.evmask = BNX2_RXP_CPU_EVENT_MASK,
	.pc = BNX2_RXP_CPU_PROGRAM_COUNTER,
	.inst = BNX2_RXP_CPU_INSTRUCTION,
	.bp = BNX2_RXP_CPU_HW_BREAKPOINT,
	.spad_base = BNX2_RXP_SCRATCH,
	.mips_view_base = 0x8000000,
};

/* Initialized Values for the TX Patch-up Processor. */
static const struct cpu_reg cpu_reg_tpat = {
	.mode = BNX2_TPAT_CPU_MODE,
	.mode_value_halt = BNX2_TPAT_CPU_MODE_SOFT_HALT,
	.mode_value_sstep = BNX2_TPAT_CPU_MODE_STEP_ENA,
	.state = BNX2_TPAT_CPU_STATE,
	.state_value_clear = 0xffffff,
	.gpr0 = BNX2_TPAT_CPU_REG_FILE,
	.evmask = BNX2_TPAT_CPU_EVENT_MASK,
	.pc = BNX2_TPAT_CPU_PROGRAM_COUNTER,
	.inst = BNX2_TPAT_CPU_INSTRUCTION,
	.bp = BNX2_TPAT_CPU_HW_BREAKPOINT,
	.spad_base = BNX2_TPAT_SCRATCH,
	.mips_view_base = 0x8000000,
};

/* Initialized Values for the TX Processor. */
static const struct cpu_reg cpu_reg_txp = {
	.mode = BNX2_TXP_CPU_MODE,
	.mode_value_halt = BNX2_TXP_CPU_MODE_SOFT_HALT,
	.mode_value_sstep = BNX2_TXP_CPU_MODE_STEP_ENA,
	.state = BNX2_TXP_CPU_STATE,
	.state_value_clear = 0xffffff,
	.gpr0 = BNX2_TXP_CPU_REG_FILE,
	.evmask = BNX2_TXP_CPU_EVENT_MASK,
	.pc = BNX2_TXP_CPU_PROGRAM_COUNTER,
	.inst = BNX2_TXP_CPU_INSTRUCTION,
	.bp = BNX2_TXP_CPU_HW_BREAKPOINT,
	.spad_base = BNX2_TXP_SCRATCH,
	.mips_view_base = 0x8000000,
};

CASSERT (0x7534 == sizeof (bnx2_RXP_b06FwText), bnx2_RXP_b06FwText);
static struct fw_info bnx2_rxp_fw_06 = {
  /* Firmware version: 5.0.0j6 */
  .ver_major                    = 0x5,
  .ver_minor                    = 0x0,
  .ver_fix                      = 0x0,

  .start_addr                   = 0x080031d8,

  .text_addr                    = 0x08000000,
  //.text_len                     = 0x7534,
  .text_index                   = 0x0,
  .text                         = (__le32 *) bnx2_RXP_b06FwText,
  .text_len                     = sizeof(bnx2_RXP_b06FwText),

  .data_addr                    = 0x00000000,
  .data_len                     = 0x0,
  .data_index                   = 0x0,
  .data                         = bnx2_RXP_b06FwData,

  .sbss_addr                    = 0x08007580,
  .sbss_len                     = 0x54,
  .sbss_index                   = 0x0,

  .bss_addr                     = 0x080075d8,
  .bss_len                      = 0x450,
  .bss_index                    = 0x0,

  .rodata_addr                  = 0x08007534,
  .rodata_len                   = 0x24,
  .rodata_index                 = 0x0,
  .rodata                               = bnx2_RXP_b06FwRodata,
};

CASSERT (0x175c == sizeof (bnx2_TPAT_b06FwText), bnx2_TPAT_b06FwText);
static struct fw_info bnx2_tpat_fw_06 = {
  /* Firmware version: 5.0.0j6 */
  .ver_major                    = 0x5,
  .ver_minor                    = 0x0,
  .ver_fix                      = 0x0,

  .start_addr                   = 0x08000488,

  .text_addr                    = 0x08000400,
  //.text_len                     = 0x175c,
  .text_index                   = 0x0,
  .text                         = (__le32 *) bnx2_TPAT_b06FwText,
  .text_len                     = sizeof(bnx2_TPAT_b06FwText),

  .data_addr                    = 0x00000000,
  .data_len                     = 0x0,
  .data_index                   = 0x0,
  .data                         = bnx2_TPAT_b06FwData,

  .sbss_addr                    = 0x08001b80,
  .sbss_len                     = 0x44,
  .sbss_index                   = 0x0,

  .bss_addr                     = 0x08001bc4,
  .bss_len                      = 0x450,
  .bss_index                    = 0x0,

  .rodata_addr                  = 0x00000000,
  .rodata_len                   = 0x0,
  .rodata_index                 = 0x0,
  .rodata                               = bnx2_TPAT_b06FwRodata,
};

CASSERT (0x3b38 == sizeof (bnx2_TXP_b06FwText), bnx2_TXP_b06FwText);
static struct fw_info bnx2_txp_fw_06 = {
  /* Firmware version: 5.0.0j6 */
  .ver_major                    = 0x5,
  .ver_minor                    = 0x0,
  .ver_fix                      = 0x0,

  .start_addr                   = 0x080000a8,

  .text_addr                    = 0x08000000,
  //.text_len                     = 0x3b38,
  .text_index                   = 0x0,
  .text                         = (__le32 *) bnx2_TXP_b06FwText,
  .text_len                     = sizeof(bnx2_TXP_b06FwText),

  .data_addr                    = 0x00000000,
  .data_len                     = 0x0,
  .data_index                   = 0x0,
  .data                         = bnx2_TXP_b06FwData,

  .sbss_addr                    = 0x08003b60,
  .sbss_len                     = 0x68,
  .sbss_index                   = 0x0,

  .bss_addr                     = 0x08003bc8,
  .bss_len                      = 0x14c,
  .bss_index                    = 0x0,

  .rodata_addr                  = 0x00000000,
  .rodata_len                   = 0x0,
  .rodata_index                 = 0x0,
  .rodata                               = bnx2_TXP_b06FwRodata,
};

CASSERT (0x4cc8 == sizeof (bnx2_COM_b06FwText), bnx2_COM_b06FwText);
static struct fw_info bnx2_com_fw_06 = {
  /* Firmware version: 5.0.0j6 */
  .ver_major                    = 0x5,
  .ver_minor                    = 0x0,
  .ver_fix                      = 0x0,

  .start_addr                   = 0x08000110,

  .text_addr                    = 0x08000000,
  //.text_len                     = 0x4cc8,
  .text_index                   = 0x0,
  .text                         = (__le32 *) bnx2_COM_b06FwText,
  .text_len                     = sizeof(bnx2_COM_b06FwText),

  .data_addr                    = 0x00000000,
  .data_len                     = 0x0,
  .data_index                   = 0x0,
  .data                         = bnx2_COM_b06FwData,

  .sbss_addr                    = 0x08004d00,
  .sbss_len                     = 0x38,
  .sbss_index                   = 0x0,

  .bss_addr                     = 0x08004d38,
  .bss_len                      = 0xc4,
  .bss_index                    = 0x0,

  .rodata_addr                  = 0x08004cc8,
  .rodata_len                   = 0x14,
  .rodata_index                 = 0x0,
  .rodata                               = bnx2_COM_b06FwRodata,
};

CASSERT (0x58c4 == sizeof (bnx2_CP_b06FwText), bnx2_CP_b06FwText);
static struct fw_info bnx2_cp_fw_06 = {
  /* Firmware version: 5.0.0j6 */
  .ver_major                    = 0x5,
  .ver_minor                    = 0x0,
  .ver_fix                      = 0x0,

  .start_addr                   = 0x08000088,

  .text_addr                    = 0x08000000,
  //.text_len                     = 0x58c4,
  .text_index                   = 0x0,
  .text                         = (__le32 *) bnx2_CP_b06FwText,
  .text_len                     = sizeof(bnx2_CP_b06FwText),

  .data_addr                    = 0x08005a40,
  .data_len                     = 0x84,
  .data_index                   = 0x0,
  .data                         = bnx2_CP_b06FwData,

  .sbss_addr                    = 0x08005ac4,
  .sbss_len                     = 0xf1,
  .sbss_index                   = 0x0,

  .bss_addr                     = 0x08005bb8,
  .bss_len                      = 0x5d8,
  .bss_index                    = 0x0,

  .rodata_addr                  = 0x080058c4,
  .rodata_len                   = 0x154,
  .rodata_index                 = 0x0,
  .rodata                               = bnx2_CP_b06FwRodata,
};

CASSERT (0x51f8 == sizeof (bnx2_COM_b09FwText), bnx2_COM_b09FwText);
static struct fw_info bnx2_com_fw_09 = {
  /* Firmware version: 5.0.0j15 */
  .ver_major                    = 0x5,
  .ver_minor                    = 0x0,
  .ver_fix                      = 0x0,

  .start_addr                   = 0x08000110,

  .text_addr                    = 0x08000000,
  //.text_len                     = 0x51f8,
  .text_index                   = 0x0,
  .text                         = (__le32 *) bnx2_COM_b09FwText,
  .text_len                     = sizeof(bnx2_COM_b09FwText),

  .data_addr                    = 0x00000000,
  .data_len                     = 0x0,
  .data_index                   = 0x0,
  .data                         = bnx2_COM_b09FwData,

  .sbss_addr                    = 0x08005260,
  .sbss_len                     = 0x30,
  .sbss_index                   = 0x0,

  .bss_addr                     = 0x08005290,
  .bss_len                      = 0x10c,
  .bss_index                    = 0x0,

  .rodata_addr                  = 0x080051f8,
  .rodata_len                   = 0x38,
  .rodata_index                 = 0x0,
  .rodata                               = bnx2_COM_b09FwRodata,
};

CASSERT (0x528c == sizeof (bnx2_CP_b09FwText), bnx2_CP_b09FwText);
static struct fw_info bnx2_cp_fw_09 = {
  /* Firmware version: 5.0.0j15 */
  .ver_major                    = 0x5,
  .ver_minor                    = 0x0,
  .ver_fix                      = 0x0,

  .start_addr                   = 0x08000088,

  .text_addr                    = 0x08000000,
  //.text_len                     = 0x528c,
  .text_index                   = 0x0,
  .text                         = (__le32 *) bnx2_CP_b09FwText,
  .text_len                     = sizeof(bnx2_CP_b09FwText),

  .data_addr                    = 0x08005480,
  .data_len                     = 0x84,
  .data_index                   = 0x0,
  .data                         = bnx2_CP_b09FwData,

  .sbss_addr                    = 0x08005508,
  .sbss_len                     = 0x9d,
  .sbss_index                   = 0x0,

  .bss_addr                     = 0x080055a8,
  .bss_len                      = 0x19c,
  .bss_index                    = 0x0,

  .rodata_addr                  = 0x0800528c,
  .rodata_len                   = 0x1c0,
  .rodata_index                 = 0x0,
  .rodata                               = bnx2_CP_b09FwRodata,
};

CASSERT (0x8108 == sizeof (bnx2_RXP_b09FwText), bnx2_RXP_b09FwText);
static struct fw_info bnx2_rxp_fw_09 = {
  /* Firmware version: 5.0.0j15 */
  .ver_major                    = 0x5,
  .ver_minor                    = 0x0,
  .ver_fix                      = 0x0,

  .start_addr                   = 0x080031d8,

  .text_addr                    = 0x08000000,
  //.text_len                     = 0x8108,
  .text_index                   = 0x0,
  .text                         = (__le32 *) bnx2_RXP_b09FwText,
  .text_len                     = sizeof(bnx2_RXP_b09FwText),

  .data_addr                    = 0x00000000,
  .data_len                     = 0x0,
  .data_index                   = 0x0,
  .data                         = bnx2_RXP_b09FwData,

  .sbss_addr                    = 0x08008260,
  .sbss_len                     = 0x60,
  .sbss_index                   = 0x0,

  .bss_addr                     = 0x080082c0,
  .bss_len                      = 0x60,
  .bss_index                    = 0x0,

  .rodata_addr                  = 0x08008108,
  .rodata_len                   = 0x124,
  .rodata_index                 = 0x0,
  .rodata                               = bnx2_RXP_b09FwRodata,
};

CASSERT (0x17ec == sizeof (bnx2_TPAT_b09FwText), bnx2_TPAT_b09FwText);
static struct fw_info bnx2_tpat_fw_09 = {
  /* Firmware version: 5.0.0j15 */
  .ver_major                    = 0x5,
  .ver_minor                    = 0x0,
  .ver_fix                      = 0x0,

  .start_addr                   = 0x08000488,

  .text_addr                    = 0x08000400,
  //.text_len                     = 0x17ec,
  .text_index                   = 0x0,
  .text                         = (__le32 *) bnx2_TPAT_b09FwText,
  .text_len                     = sizeof(bnx2_TPAT_b09FwText),

  .data_addr                    = 0x00000000,
  .data_len                     = 0x0,
  .data_index                   = 0x0,
  .data                         = bnx2_TPAT_b09FwData,

  .sbss_addr                    = 0x08001c20,
  .sbss_len                     = 0x3c,
  .sbss_index                   = 0x0,

  .bss_addr                     = 0x08001c5c,
  .bss_len                      = 0x344,
  .bss_index                    = 0x0,

  .rodata_addr                  = 0x08001bec,
  .rodata_len                   = 0x4,
  .rodata_index                 = 0x0,
  .rodata                               = bnx2_TPAT_b09FwRodata,
};

CASSERT (0x38d0 == sizeof (bnx2_TXP_b09FwText), bnx2_TXP_b09FwText);
static struct fw_info bnx2_txp_fw_09 = {
  /* Firmware version: 5.0.0j15 */
  .ver_major                    = 0x5,
  .ver_minor                    = 0x0,
  .ver_fix                      = 0x0,

  .start_addr                   = 0x080000a8,

  .text_addr                    = 0x08000000,
  //.text_len                     = 0x38d0,
  .text_index                   = 0x0,
  .text                         = (__le32 *) bnx2_TXP_b09FwText,
  .text_len                     = sizeof(bnx2_TXP_b09FwText),

  .data_addr                    = 0x00000000,
  .data_len                     = 0x0,
  .data_index                   = 0x0,
  .data                         = bnx2_TXP_b09FwData,

  .sbss_addr                    = 0x08003920,
  .sbss_len                     = 0x64,
  .sbss_index                   = 0x0,

  .bss_addr                     = 0x08003988,
  .bss_len                      = 0x24c,
  .bss_index                    = 0x0,

  .rodata_addr                  = 0x080038d0,
  .rodata_len                   = 0x30,
  .rodata_index                 = 0x0,
  .rodata                               = bnx2_TXP_b09FwRodata,
};

static int
load_cpu_fw(struct bnx2 *bp, const struct cpu_reg *cpu_reg, struct fw_info *fw)
{
  u32 offset;
  u32 val;
  int rc;

  /* Halt the CPU. */
  val = bnx2_reg_rd_ind(bp, cpu_reg->mode);
  val |= cpu_reg->mode_value_halt;
  bnx2_reg_wr_ind(bp, cpu_reg->mode, val);
  bnx2_reg_wr_ind(bp, cpu_reg->state, cpu_reg->state_value_clear);

  /* Load the Text area. */
  offset = cpu_reg->spad_base + (fw->text_addr - cpu_reg->mips_view_base);

  if (fw->text) {
    int j;
    for (j = 0; j < (fw->text_len / 4); j++, offset += 4) {
      bnx2_reg_wr_ind(bp, offset, __le32_to_cpu(fw->text[j]));
    }
  }

  /* Load the Data area. */
  offset = cpu_reg->spad_base + (fw->data_addr - cpu_reg->mips_view_base);
  if (fw->data) {
    int j;

    for (j = 0; j < (fw->data_len / 4); j++, offset += 4) {
      bnx2_reg_wr_ind(bp, offset, fw->data[j]);
    }
  }

  /* Load the SBSS area. */
  offset = cpu_reg->spad_base + (fw->sbss_addr - cpu_reg->mips_view_base);
  if (fw->sbss_len) {
    int j;

    for (j = 0; j < (fw->sbss_len / 4); j++, offset += 4) {
      bnx2_reg_wr_ind(bp, offset, 0);
    }
  }

  /* Load the BSS area. */
  offset = cpu_reg->spad_base + (fw->bss_addr - cpu_reg->mips_view_base);
  if (fw->bss_len) {
    int j;

    for (j = 0; j < (fw->bss_len/4); j++, offset += 4) {
      bnx2_reg_wr_ind(bp, offset, 0);
    }
  }

  /* Load the Read-Only area. */
  offset = cpu_reg->spad_base +
    (fw->rodata_addr - cpu_reg->mips_view_base);
  if (fw->rodata) {
    int j;

    for (j = 0; j < (fw->rodata_len / 4); j++, offset += 4) {
      bnx2_reg_wr_ind(bp, offset, fw->rodata[j]);
    }
  }

  /* Clear the pre-fetch instruction. */
  bnx2_reg_wr_ind(bp, cpu_reg->inst, 0);
  bnx2_reg_wr_ind(bp, cpu_reg->pc, fw->start_addr);

  /* Start the CPU. */
  val = bnx2_reg_rd_ind(bp, cpu_reg->mode);
  val &= ~cpu_reg->mode_value_halt;
  bnx2_reg_wr_ind(bp, cpu_reg->state, cpu_reg->state_value_clear);
  bnx2_reg_wr_ind(bp, cpu_reg->mode, val);

  return 0;
}

static bool
bnx2_init_cpus(struct bnx2 *bp)
{
  struct fw_info *fw;
  int rc = 0, rv2p_len;
  const void *rv2p;
  u32 fixup_loc;

  /* Initialize the RV2P processor. */
  if (CHIP_NUM(bp) == CHIP_NUM_5709) {
    if ((CHIP_ID(bp) == CHIP_ID_5709_A0) ||
        (CHIP_ID(bp) == CHIP_ID_5709_A1)) {
      rv2p = bnx2_xi90_rv2p_proc1;
      rv2p_len = sizeof(bnx2_xi90_rv2p_proc1);
      fixup_loc = XI90_RV2P_PROC1_MAX_BD_PAGE_LOC;
    } else {
      rv2p = bnx2_xi_rv2p_proc1;
      rv2p_len = sizeof(bnx2_xi_rv2p_proc1);
      fixup_loc = XI_RV2P_PROC1_MAX_BD_PAGE_LOC;
    }
  } else {
    rv2p = bnx2_rv2p_proc1;
    rv2p_len = sizeof(bnx2_rv2p_proc1);
    fixup_loc = RV2P_PROC1_MAX_BD_PAGE_LOC;
  }

  load_rv2p_fw(bp, (__le32 *) rv2p, rv2p_len, RV2P_PROC1, fixup_loc);

  if (CHIP_NUM(bp) == CHIP_NUM_5709) {
    if ((CHIP_ID(bp) == CHIP_ID_5709_A0) ||
        (CHIP_ID(bp) == CHIP_ID_5709_A1)) {
      rv2p = bnx2_xi90_rv2p_proc2;
      rv2p_len = sizeof(bnx2_xi90_rv2p_proc2);
      fixup_loc = XI90_RV2P_PROC2_MAX_BD_PAGE_LOC;
    } else {
      rv2p = bnx2_xi_rv2p_proc2;
      rv2p_len = sizeof(bnx2_xi_rv2p_proc2);
      fixup_loc = XI_RV2P_PROC2_MAX_BD_PAGE_LOC;
    }
  } else {
    rv2p = bnx2_rv2p_proc2;
    rv2p_len = sizeof(bnx2_rv2p_proc2);
    fixup_loc = RV2P_PROC2_MAX_BD_PAGE_LOC;
  }

  load_rv2p_fw(bp, (__le32 *) rv2p, rv2p_len, RV2P_PROC2, fixup_loc);

  /* Initialize the RX Processor. */
  if (CHIP_NUM(bp) == CHIP_NUM_5709)
    fw = &bnx2_rxp_fw_09;
  else
    fw = &bnx2_rxp_fw_06;

  rc = load_cpu_fw(bp, &cpu_reg_rxp, fw);
  if (rc)
    goto init_cpu_err;

  /* Initialize the TX Processor. */
  if (CHIP_NUM(bp) == CHIP_NUM_5709)
    fw = &bnx2_txp_fw_09;
  else
    fw = &bnx2_txp_fw_06;

  rc = load_cpu_fw(bp, &cpu_reg_txp, fw);
  if (rc)
    goto init_cpu_err;

  /* Initialize the TX Patch-up Processor. */
  if (CHIP_NUM(bp) == CHIP_NUM_5709)
    fw = &bnx2_tpat_fw_09;
  else
    fw = &bnx2_tpat_fw_06;

  rc = load_cpu_fw(bp, &cpu_reg_tpat, fw);
  if (rc)
    goto init_cpu_err;

  /* Initialize the Completion Processor. */
  if (CHIP_NUM(bp) == CHIP_NUM_5709)
    fw = &bnx2_com_fw_09;
  else
    fw = &bnx2_com_fw_06;

  rc = load_cpu_fw(bp, &cpu_reg_com, fw);
  if (rc)
    goto init_cpu_err;

  /* Initialize the Command Processor. */
  if (CHIP_NUM(bp) == CHIP_NUM_5709)
    fw = &bnx2_cp_fw_09;
  else
    fw = &bnx2_cp_fw_06;

  rc = load_cpu_fw(bp, &cpu_reg_cp, fw);
  if (rc)
    goto init_cpu_err;

  return TRUE;

 init_cpu_err:
  DLOG ("init_cpu_err rc=%d", rc);
  return FALSE;
}

static void
bnx2_mac_init (struct bnx2 *bp)
{
  /* disable interrupts */
  REG_WR(bp, BNX2_PCICFG_INT_ACK_CMD, BNX2_PCICFG_INT_ACK_CMD_MASK_INT);

  /* enable byte and word swapping */
  u32 val = BNX2_DMA_CONFIG_DATA_BYTE_SWAP | BNX2_DMA_CONFIG_DATA_WORD_SWAP |
    BNX2_DMA_CONFIG_CNTL_WORD_SWAP;

  val |= (5 << 12) | (3 << 16);

  val |= (0x2 << 20) | (1 << 11);

  if ((CHIP_NUM(bp) == CHIP_NUM_5706) &&
      (CHIP_ID(bp) != CHIP_ID_5706_A0) && !(bp->flags & BNX2_FLAG_PCIX)) {
    val |= BNX2_DMA_CONFIG_CNTL_PING_PONG_DMA;
  }
  REG_WR(bp, BNX2_DMA_CONFIG, val);

#if 0
  /* errata */
  if (bp->flags & BNX2_FLAG_PCIX) {
    u16 val16;
    val16 = PCICFG_RD(bp, BNX2_PCICFG_PCIX_COMMAND);
    val16 &= ~BNX2_PCICFG_PCIX_COMMAND_RELAX_ORDER;
    PCICFG_WR(bp, BNX2_PCICFG_PCIX_COMMAND, val16);
  }
#endif

  /* enable context block */
  REG_WR(bp,
         BNX2_MISC_ENABLE_SET_BITS,
         BNX2_MISC_ENABLE_SET_BITS_HOST_COALESCE_ENABLE |
         BNX2_MISC_ENABLE_STATUS_BITS_RX_V2P_ENABLE |
         BNX2_MISC_ENABLE_STATUS_BITS_CONTEXT_ENABLE);

  /* initialize context memory */
  u32 vcid;
  vcid = 96;
  while (vcid) {
    u32 vcid_addr, pcid_addr, offset;
    vcid--;
    if (CHIP_ID(bp) == CHIP_ID_5706_A0) {
      u32 new_vcid;
      vcid_addr = GET_PCID_ADDR(vcid);
      if (vcid & 0x8) {
        new_vcid = 0x60 + (vcid & 0xf0) + (vcid & 0x7);
      }
      else {
        new_vcid = vcid;
      }
      pcid_addr = GET_PCID_ADDR(new_vcid);
    }
    else {
      vcid_addr = GET_CID_ADDR(vcid);
      pcid_addr = vcid_addr;
    }
    vcid_addr = GET_CID_ADDR(vcid);
    pcid_addr = vcid_addr;
    REG_WR(bp, BNX2_CTX_VIRT_ADDR, 0x00);
    REG_WR(bp, BNX2_CTX_PAGE_TBL, pcid_addr);
    /* Zero out the context. */
    for (offset = 0; offset < PHY_CTX_SIZE; offset += 4) {
      bnx2_ctx_wr(bp, 0x00, offset, 0);
    }
    REG_WR(bp, BNX2_CTX_VIRT_ADDR, vcid_addr);
    REG_WR(bp, BNX2_CTX_PAGE_TBL, pcid_addr);
  }

  if (!bnx2_init_cpus (bp)) {
    DLOG ("bnx2_init_cpus failed");
    return;
  }   
  DLOG ("Initialized onboard CPUs");

  /* Program MAC address */

  u8 *mac_addr = bp->mac_addr;
  val = (mac_addr[0] << 8) | mac_addr[1];
  REG_WR(bp, BNX2_EMAC_MAC_MATCH0, val);
  val = (mac_addr[2] << 24) | (mac_addr[3] << 16) | (mac_addr[4] << 8) | mac_addr[5];
  REG_WR(bp, BNX2_EMAC_MAC_MATCH1, val);

  /* Set kernel bypass block size to 256 bytes */
  val = REG_RD(bp, BNX2_MQ_CONFIG);
  val &= ~BNX2_MQ_CONFIG_KNL_BYP_BLK_SIZE;
  val |= BNX2_MQ_CONFIG_KNL_BYP_BLK_SIZE_256;
  REG_WR(bp, BNX2_MQ_CONFIG, val);

  /* Configure size of Kernel Mailbox Window */
  val = 0x10000 + (MAX_CID_CNT * MB_KERNEL_CTX_SIZE);
  REG_WR(bp, BNX2_MQ_KNL_WIND_END, val);
  REG_WR(bp, BNX2_MQ_KNL_BYP_WIND_START, val);

  /* Configure chain page size */
  val = (BCM_PAGE_BITS - 8) << 24;
  REG_WR(bp, BNX2_RV2P_CONFIG, val);
  val = REG_RD(bp, BNX2_TBDR_CONFIG);
  val &= ~BNX2_TBDR_CONFIG_PAGE_SIZE;
  val |= (BCM_PAGE_BITS - 8) << 24 | 0x40;
  REG_WR(bp, BNX2_TBDR_CONFIG, val);

  /* Configure random backoff seed */
  val = mac_addr[0] + (mac_addr[1] << 8) + (mac_addr[2] << 16) +
    mac_addr[3] + (mac_addr[4] << 8) + (mac_addr[5] << 16);
  REG_WR(bp, BNX2_EMAC_BACKOFF_SEED, val);

  /* Configure MTU */
#define MIN_ETHERNET_PACKET_SIZE 60
#define MAX_ETHERNET_PACKET_SIZE 1514
#define MAX_ETHERNET_JUMBO_PACKET_SIZE 9014
#define ETHERNET_FCS_SIZE 4
#define ETHERNET_VLAN_TAG_SIZE 4
#define ETH_HLEN 14
  val = MAX_ETHERNET_PACKET_SIZE + ETH_HLEN + ETHERNET_FCS_SIZE;
  REG_WR(bp, BNX2_EMAC_RX_MTU_SIZE, val);

  /* Interrupt on link state change */
  REG_WR(bp, BNX2_EMAC_ATTENTION_ENA, BNX2_EMAC_ATTENTION_ENA_LINK);

  /* Program host memory status block */
#if 0
  REG_WR(bp, BNX2_HC_STATUS_ADDR_L,
         (u64) bp->status_blk_mapping & 0xffffffff);
  REG_WR(bp, BNX2_HC_STATUS_ADDR_H, (u64) bp->status_blk_mapping >> 32);

  /* Program host memory stats block */
  REG_WR(bp, BNX2_HC_STATISTICS_ADDR_L,
         (u64) bp->stats_blk_mapping & 0xffffffff);
  REG_WR(bp, BNX2_HC_STATISTICS_ADDR_H, (u64) bp->stats_blk_mapping >> 32);
#endif

  /* Host coalescing */
  if (CHIP_ID(bp) == CHIP_ID_5706_A1) {
    REG_WR(bp, BNX2_HC_CONFIG, BNX2_HC_CONFIG_COLLECT_STATS);
  }
  else {
    REG_WR(bp, BNX2_HC_CONFIG,
           BNX2_HC_CONFIG_RX_TMR_MODE |
           BNX2_HC_CONFIG_TX_TMR_MODE |
           BNX2_HC_CONFIG_COLLECT_STATS);
  }
  REG_WR(bp, BNX2_HC_CONFIG, 
         BNX2_HC_CONFIG_RX_TMR_MODE |
         BNX2_HC_CONFIG_TX_TMR_MODE |
         BNX2_HC_CONFIG_COLLECT_STATS);

  /* clear statistics */
  REG_WR(bp, BNX2_HC_COMMAND, BNX2_HC_COMMAND_CLR_STAT_NOW);

  /* link state changes generate interrupts */
  REG_WR(bp, BNX2_HC_ATTN_BITS_ENABLE, STATUS_ATTN_BITS_LINK_STATE);

  /* Rx filtering features */
  u32 rx_mode = BNX2_EMAC_RX_MODE_SORT_MODE;
  u32 sort_mode = BNX2_RPM_SORT_USER0_MC_EN | BNX2_RPM_SORT_USER0_BC_EN | 1;
  REG_WR(bp, BNX2_EMAC_RX_MODE, rx_mode);
  REG_WR(bp, BNX2_RPM_SORT_USER0, 0x0);
  REG_WR(bp, BNX2_RPM_SORT_USER0, sort_mode);
  REG_WR(bp, BNX2_RPM_SORT_USER0, sort_mode | BNX2_RPM_SORT_USER0_ENA);

  /* sync firmware */
#if 0
  fw_sync(bp, DRV_MSG_DATA_WAIT2 | DRV_MSG_CODE_RESET);
#else
  udelay (1000);
#endif

  /* Enable the remainder of the NetXtreme II blocks and delay for 20
   * μs to allow the various blocks to complete initialization.  The
   * NetXtreme II controller is now ready to send and receive
   * traffic. */

  REG_WR(bp, BNX2_MISC_ENABLE_SET_BITS, 0x5ffffff);
  REG_RD(bp, BNX2_MISC_ENABLE_SET_BITS);
  udelay(20);
}


#define subsystem_vendor(pdev) ((pdev)->data[11] & 0x0000FFFF)
#define subsystem_device(pdev) (((pdev)->data[11] & 0xFFFF0000) >> 16)

static bool
bnx2_init_board (pci_device *pdev)
{
  struct bnx2 *bp = pdev->drvdata;
  int rc, i, j;
  pci_irq_t irq;
  uint irq_line, irq_pin;

  bp->flags = bp->phy_flags = 0;

  pow2_alloc (sizeof(struct statistics_block), (u8 **) &bp->temp_stats_blk);

  /* pci enable device */

  /* get pci resources */
  u32 phys_addr; void *base_addr;
  pci_decode_bar (pdev->index, 0, &phys_addr, NULL, NULL);

  if (!pci_get_interrupt (pdev->index, &irq_line, &irq_pin)) {
    DLOG ("Unable to get IRQ");
    goto err_out_free_stats;
  }

  if (pci_irq_find (pdev->bus, pdev->slot, irq_pin, &irq)) {
    /* use PCI routing table */
    DLOG ("Found PCI routing entry irq.gsi=0x%x", irq.gsi);
    pci_irq_map (&irq, BNX2_VECTOR, 0x01,
                 IOAPIC_DESTINATION_LOGICAL, IOAPIC_DELIVERY_FIXED);
    irq_line = irq.gsi;
  } else {
    DLOG ("Unable to find PCI routing entry");
    goto err_out_free_stats;
  }

  /* Map IRQ to handler */
  set_vector_handler (BNX2_VECTOR, bnx2_irq_handler);

  /* map 64kb at phys_addr */
#define NUM_PAGES 16
  base_addr = map_contiguous_virtual_pages (phys_addr | 3, NUM_PAGES);
  if (!base_addr)
    goto err_out_free_stats;

  DLOG ("phys_addr=0x%.08X base_addr=%p", phys_addr, base_addr);

  spinlock_init (&bp->phy_lock);
  spinlock_init (&bp->indirect_lock);

  /* u32 mem_len = MB_GET_CID_ADDR(TX_TSS_CID + TX_MAX_TSS_RINGS + 1); */

  bp->regview = base_addr;
  /* remap [base_addr..base_addr+mem_len] to nocache */

  /* Configure byte swap and enable write to the reg_window registers.
   * Rely on CPU to do target byte swapping on big endian systems
   * The chip's target access swapping will not swap all accesses
   */
  pci_write_config_dword(bp->pdev, BNX2_PCICFG_MISC_CONFIG,
                         BNX2_PCICFG_MISC_CONFIG_REG_WINDOW_ENA |
                         BNX2_PCICFG_MISC_CONFIG_TARGET_MB_WORD_SWAP);

  bp->chip_id = REG_RD(bp, BNX2_MISC_ID);
  DLOG ("chip_id=0x%X", bp->chip_id);

#if 0
  if (CHIP_NUM(bp) == CHIP_NUM_5709) {
    if (pci_find_capability(pdev, PCI_CAP_ID_EXP) == 0) {
      dev_err(&pdev->dev,
              "Cannot find PCIE capability, aborting\n");
      rc = -EIO;
      goto err_out_unmap;
    }
    bp->flags |= BNX2_FLAG_PCIE;
    if (CHIP_REV(bp) == CHIP_REV_Ax)
      bp->flags |= BNX2_FLAG_JUMBO_BROKEN;
  } else {
    bp->pcix_cap = pci_find_capability(pdev, PCI_CAP_ID_PCIX);
    if (bp->pcix_cap == 0) {
      dev_err(&pdev->dev,
              "Cannot find PCIX capability, aborting\n");
      rc = -EIO;
      goto err_out_unmap;
    }
    bp->flags |= BNX2_FLAG_BROKEN_STATS;
  }

  if (CHIP_NUM(bp) == CHIP_NUM_5709 && CHIP_REV(bp) != CHIP_REV_Ax) {
    if (pci_find_capability(pdev, PCI_CAP_ID_MSIX))
      bp->flags |= BNX2_FLAG_MSIX_CAP;
  }

  if (CHIP_ID(bp) != CHIP_ID_5706_A0 && CHIP_ID(bp) != CHIP_ID_5706_A1) {
    if (pci_find_capability(pdev, PCI_CAP_ID_MSI))
      bp->flags |= BNX2_FLAG_MSI_CAP;
  }

  /* 5708 cannot support DMA addresses > 40-bit.  */
  if (CHIP_NUM(bp) == CHIP_NUM_5708)
    persist_dma_mask = dma_mask = DMA_BIT_MASK(40);
  else
    persist_dma_mask = dma_mask = DMA_BIT_MASK(64);

  /* Configure DMA attributes. */
  if (pci_set_dma_mask(pdev, dma_mask) == 0) {
    dev->features |= NETIF_F_HIGHDMA;
    rc = pci_set_consistent_dma_mask(pdev, persist_dma_mask);
    if (rc) {
      dev_err(&pdev->dev,
              "pci_set_consistent_dma_mask failed, aborting\n");
      goto err_out_unmap;
    }
  } else if ((rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) != 0) {
    dev_err(&pdev->dev, "System does not support DMA, aborting\n");
    goto err_out_unmap;
  }

  if (!(bp->flags & BNX2_FLAG_PCIE))
    bnx2_get_pci_speed(bp);

  /* 5706A0 may falsely detect SERR and PERR. */
  if (CHIP_ID(bp) == CHIP_ID_5706_A0) {
    reg = REG_RD(bp, PCI_COMMAND);
    reg &= ~(PCI_COMMAND_SERR | PCI_COMMAND_PARITY);
    REG_WR(bp, PCI_COMMAND, reg);
  }
  else if ((CHIP_ID(bp) == CHIP_ID_5706_A1) &&
           !(bp->flags & BNX2_FLAG_PCIX)) {

    dev_err(&pdev->dev,
            "5706 A1 can only be used in a PCIX bus, aborting\n");
    goto err_out_unmap;
  }

#endif

  bnx2_init_nvram(bp);

  u32 reg = bnx2_reg_rd_ind(bp, BNX2_SHM_HDR_SIGNATURE);

  if ((reg & BNX2_SHM_HDR_SIGNATURE_SIG_MASK) ==
      BNX2_SHM_HDR_SIGNATURE_SIG) {
    u32 off = pdev->func << 2;

    bp->shmem_base = bnx2_reg_rd_ind(bp, BNX2_SHM_HDR_ADDR_0 + off);
  } else
    bp->shmem_base = HOST_VIEW_SHMEM_BASE;

  /* Get the permanent MAC address.  First we need to make sure the
   * firmware is actually running.
   */
  reg = bnx2_shmem_rd(bp, BNX2_DEV_INFO_SIGNATURE);

  if ((reg & BNX2_DEV_INFO_SIGNATURE_MAGIC_MASK) !=
      BNX2_DEV_INFO_SIGNATURE_MAGIC) {
    DLOG("Firmware not running, aborting");
    rc = -ENODEV;
    goto err_out_unmap;
  }

#if 0
  bnx2_read_vpd_fw_ver(bp);

  j = strlen(bp->fw_version);
  reg = bnx2_shmem_rd(bp, BNX2_DEV_INFO_BC_REV);
  for (i = 0; i < 3 && j < 24; i++) {
    u8 num, k, skip0;

    if (i == 0) {
      bp->fw_version[j++] = 'b';
      bp->fw_version[j++] = 'c';
      bp->fw_version[j++] = ' ';
    }
    num = (u8) (reg >> (24 - (i * 8)));
    for (k = 100, skip0 = 1; k >= 1; num %= k, k /= 10) {
      if (num >= k || !skip0 || k == 1) {
        bp->fw_version[j++] = (num / k) + '0';
        skip0 = 0;
      }
    }
    if (i != 2)
      bp->fw_version[j++] = '.';
  }
  reg = bnx2_shmem_rd(bp, BNX2_PORT_FEATURE);
  if (reg & BNX2_PORT_FEATURE_WOL_ENABLED)
    bp->wol = 1;

  if (reg & BNX2_PORT_FEATURE_ASF_ENABLED) {
    bp->flags |= BNX2_FLAG_ASF_ENABLE;

    for (i = 0; i < 30; i++) {
      reg = bnx2_shmem_rd(bp, BNX2_BC_STATE_CONDITION);
      if (reg & BNX2_CONDITION_MFW_RUN_MASK)
        break;
      udelay(10*1000);
    }
  }
  reg = bnx2_shmem_rd(bp, BNX2_BC_STATE_CONDITION);
  reg &= BNX2_CONDITION_MFW_RUN_MASK;
  if (reg != BNX2_CONDITION_MFW_RUN_UNKNOWN &&
      reg != BNX2_CONDITION_MFW_RUN_NONE) {
    u32 addr = bnx2_shmem_rd(bp, BNX2_MFW_VER_PTR);

    if (j < 32)
      bp->fw_version[j++] = ' ';
    for (i = 0; i < 3 && j < 28; i++) {
      reg = bnx2_reg_rd_ind(bp, addr + i * 4);
      reg = ___constant_swab32(reg);
      memcpy(&bp->fw_version[j], &reg, 4);
      j += 4;
    }
  }
#endif

  reg = bnx2_shmem_rd(bp, BNX2_PORT_HW_CFG_MAC_UPPER);
  bp->mac_addr[0] = (u8) (reg >> 8);
  bp->mac_addr[1] = (u8) reg;

  reg = bnx2_shmem_rd(bp, BNX2_PORT_HW_CFG_MAC_LOWER);
  bp->mac_addr[2] = (u8) (reg >> 24);
  bp->mac_addr[3] = (u8) (reg >> 16);
  bp->mac_addr[4] = (u8) (reg >> 8);
  bp->mac_addr[5] = (u8) reg;

  DLOG ("mac_addr=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
        bp->mac_addr[0], bp->mac_addr[1], bp->mac_addr[2],
        bp->mac_addr[3], bp->mac_addr[4], bp->mac_addr[5]);

  bp->tx_ring_size = MAX_TX_DESC_CNT;
#if 0
  bnx2_set_rx_ring_size(bp, 255);
#endif

  bp->rx_csum = 1;

  bp->tx_quick_cons_trip_int = 2;
  bp->tx_quick_cons_trip = 20;
  bp->tx_ticks_int = 18;
  bp->tx_ticks = 80;

  bp->rx_quick_cons_trip_int = 2;
  bp->rx_quick_cons_trip = 12;
  bp->rx_ticks_int = 18;
  bp->rx_ticks = 18;

#define USEC_PER_SEC 1000000
  bp->stats_ticks = USEC_PER_SEC & BNX2_HC_STATS_TICKS_HC_STAT_TICKS;

  bp->current_interval = BNX2_TIMER_INTERVAL;

  bp->phy_addr = 1;

  /* Disable WOL support if we are running on a SERDES chip. */
  if (CHIP_NUM(bp) == CHIP_NUM_5709)
    bnx2_get_5709_media(bp);
  else if (CHIP_BOND_ID(bp) & CHIP_BOND_ID_SERDES_BIT)
    bp->phy_flags |= BNX2_PHY_FLAG_SERDES;

  bp->phy_port = PORT_TP;
  if (bp->phy_flags & BNX2_PHY_FLAG_SERDES) {
    bp->phy_port = PORT_FIBRE;
    reg = bnx2_shmem_rd(bp, BNX2_SHARED_HW_CFG_CONFIG);
    if (!(reg & BNX2_SHARED_HW_CFG_GIG_LINK_ON_VAUX)) {
      bp->flags |= BNX2_FLAG_NO_WOL;
      bp->wol = 0;
    }
    if (CHIP_NUM(bp) == CHIP_NUM_5706) {
      /* Don't do parallel detect on this board because of
       * some board problems.  The link will not go down
       * if we do parallel detect.
       */
      if (subsystem_vendor (bp->pdev) == PCI_VENDOR_ID_HP &&
          subsystem_device (bp->pdev) == 0x310c)
        bp->phy_flags |= BNX2_PHY_FLAG_NO_PARALLEL;
    } else {
      bp->phy_addr = 2;
      if (reg & BNX2_SHARED_HW_CFG_PHY_2_5G)
        bp->phy_flags |= BNX2_PHY_FLAG_2_5G_CAPABLE;
    }
  } else if (CHIP_NUM(bp) == CHIP_NUM_5706 ||
             CHIP_NUM(bp) == CHIP_NUM_5708)
    bp->phy_flags |= BNX2_PHY_FLAG_CRC_FIX;
  else if (CHIP_NUM(bp) == CHIP_NUM_5709 &&
           (CHIP_REV(bp) == CHIP_REV_Ax ||
            CHIP_REV(bp) == CHIP_REV_Bx))
    bp->phy_flags |= BNX2_PHY_FLAG_DIS_EARLY_DAC;

  bnx2_init_fw_cap(bp);

  if ((CHIP_ID(bp) == CHIP_ID_5708_A0) ||
      (CHIP_ID(bp) == CHIP_ID_5708_B0) ||
      (CHIP_ID(bp) == CHIP_ID_5708_B1) ||
      !(REG_RD(bp, BNX2_PCI_CONFIG_3) & BNX2_PCI_CONFIG_3_VAUX_PRESET)) {
    bp->flags |= BNX2_FLAG_NO_WOL;
    bp->wol = 0;
  }

  if (CHIP_ID(bp) == CHIP_ID_5706_A0) {
    bp->tx_quick_cons_trip_int =
      bp->tx_quick_cons_trip;
    bp->tx_ticks_int = bp->tx_ticks;
    bp->rx_quick_cons_trip_int =
      bp->rx_quick_cons_trip;
    bp->rx_ticks_int = bp->rx_ticks;
    bp->comp_prod_trip_int = bp->comp_prod_trip;
    bp->com_ticks_int = bp->com_ticks;
    bp->cmd_ticks_int = bp->cmd_ticks;
  }

  bnx2_set_default_link(bp);
  bp->req_flow_ctrl = FLOW_CTRL_RX | FLOW_CTRL_TX;

#if 0
  init_timer(&bp->timer);
  bp->timer.expires = RUN_AT(BNX2_TIMER_INTERVAL);
  bp->timer.data = (unsigned long) bp;
  bp->timer.function = bnx2_timer;
#endif

  return TRUE;

 err_out_unmap:
  if (bp->regview) {
    /* iounmap(bp->regview); */
    bp->regview = NULL;
  }

 err_out_release:
  /* pci_release_regions(pdev); */
  unmap_virtual_pages (base_addr, NUM_PAGES);

 err_out_disable:
  /* pci_disable_device(pdev);
     pci_set_drvdata(pdev, NULL); */

 err_out_free_stats:
  pow2_free ((u8 *) bp->temp_stats_blk);

  return FALSE;
}

static uint device_index;
static pci_device pdev;

extern bool
bnx2_init (void)
{
  uint i, frame_count;
  pci_irq_t irq;
  struct bnx2 *bp;

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

  if (!pci_get_device (device_index, &pdev)) {
    DLOG ("pci_get_device");
    goto abort;
  }

  pow2_alloc (sizeof (struct bnx2), (u8 **) &bp);

  pdev.drvdata = bp;
  bp->pdev = &pdev;

  if (!bnx2_init_board (&pdev))
    goto abort_bp;

  bnx2_mac_reset (bp);
  bnx2_mac_init (bp);

  return TRUE;

 abort_bp:
  pow2_free ((u8 *) bp);
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
