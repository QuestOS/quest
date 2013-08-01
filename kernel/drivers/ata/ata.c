/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
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

#include "drivers/ata/ata.h"
#include "arch/i386.h"
#include "arch/i386-div64.h"
#include "util/printf.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "sched/sched.h"
#include "sched/vcpu.h"
#include "kernel.h"

//#define DEBUG_ATA
#ifdef DEBUG_ATA
#define DLOG(fmt,...) DLOG_PREFIX("ata",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif


/**************************************************************************
 *  IDENTIFY command                                                      *
 *                                                                        *
 * To use the IDENTIFY command, select a target drive by sending 0xA0 for *
 * the master drive, or 0xB0 for the slave, to the "drive select" IO      *
 * port. On the Primary bus, this would be port 0x1F6. A 400ns delay is a *
 * good idea. Then send the IDENTIFY command (0xEC) to the Command IO     *
 * port (0x1F7). Then read the Status port (0x1F7) again. If the value    *
 * read is 0, the drive does not exist. For any other value: poll the     *
 * Status port (0x1F7) until bit 7 (BSY, value = 0x80) clears, and bit 3  *
 * (DRQ, value = 8) sets -- or until bit 0 (ERR, value = 1) sets. At that *
 * point, if ERR is clear, the data is ready to read from the Data port   *
 * (0x1F0). Read 256 words, and store them.  "Command Aborted"            *
 *                                                                        *
 * ATAPI or SATA devices will respond to an ATA IDENTIFY command by       *
 * immediately reporting an error in the Status Register, rather than     *
 * setting BSY, then DRQ, then sending 256 words of PIO data. These       *
 * devices will also write specific values to the IO ports, that can be   *
 * read. Seeing ATAPI specific values on those ports after this sort of   *
 * "abort" is definitive proof that the device is ATAPI -- on the Primary *
 * bus, IO port 0x1F4 will read as 0x14, and IO port 0x1F5 will read as   *
 * 0xEB. If a normal ATA drive should ever happen to abort an IDENTIFY    *
 * command, the values in those two ports will be 0. A SATA device will   *
 * report 0x3c, and 0xc3 instead. See below for a code example.           *
 *                                                                        *
 * Make sure to note that this means it is important to check the ERR bit *
 * (bit 0, value = 1) in the Regular or Alternate Status register, before *
 * trying to read the 256 words of PIO data.  Interesting information     *
 * returned by IDENTIFY                                                   *
 *                                                                        *
 *     * Word 0: is useful if the device is not a hard disk.              *
 *                                                                        *
 *     * Word 83: Bit 10 is set if the drive supports LBA48 mode.         *
 *                                                                        *
 *     * Word 88: The bits in the low byte tell you the supported UDMA    *
 *     * modes, the upper byte tells you which UDMA mode is active. If    *
 *     * the active mode is not the highest supported mode, you may want  *
 *     * to figure out why.                                               *
 *                                                                        *
 *     * Word 93 from a master drive on the bus: Bit 12 is supposed to be *
 *     * set if the drive detects an 80 pin cable.                        *
 *                                                                        *
 *     * Words 60 & 61 taken as a uint32 contain the total number of 28    *
 *     * bit LBA addressable sectors on the drive. (If non-zero, the      *
 *     * drive supports LBA28.)                                           *
 *                                                                        *
 *     * Words 100 through 103 taken as a uint64 contain the total number  *
 *     * of 48 bit addressable sectors on the drive. (Probably also proof *
 *     * that LBA48 is supported.)                                        *
 **************************************************************************/

ata_info pata_drives[4];
/* ata_current_task has exclusive access to ATA.  If an ATA IRQ comes
 * in, we assume that ata_current_task is waiting on it and needs to
 * be woken. */
static task_id ata_current_task = 0;
/* waitqueue of tasks that want to use ATA. */
static task_id ata_waitqueue = 0;

/* technically I think there could be separate queues for each bus,
 * but, whatever. */

/* Kernel lock should be held while using ATA */

/* relinquish CPU until we get exclusive access to the ATA subsystem */
static void
ata_grab (void)
{
  DLOG ("ata_grab() ata_current_task=%x ata_waitqueue=%x tr=%x",
        ata_current_task, ata_waitqueue, str ());
  while (ata_current_task) {
    queue_append (&ata_waitqueue, str ());
    schedule ();
  }
  ata_current_task = str ();
}

static void
ata_release (void)
{
  DLOG ("ata_release() ata_current_task=%x ata_waitqueue=%x tr=%x",
        ata_current_task, ata_waitqueue, str ());
  wakeup_queue (&ata_waitqueue);
  ata_waitqueue = 0;
  ata_current_task = 0;
}

/* ATA specifies a 400ns delay after drive switching -- often
 * implemented as 4 Alternative Status queries. */
#define ATA_SELECT_DELAY(bus) \
  {inb(ATA_DCR(bus));inb(ATA_DCR(bus));inb(ATA_DCR(bus));inb(ATA_DCR(bus));}

/* Soft-reset the ATA bus. */
void
ata_sreset (uint32 bus)
{
  outb (0x02, ATA_DCR (bus));
  outb (0x00, ATA_DCR (bus));
}

/* Tell the bus controller to select master or slave drive for
 * subsequent operations. */
void
ata_drive_select (uint32 bus, uint32 drive)
{
  outb (drive, ATA_DRIVE_SELECT (bus));
  ATA_SELECT_DELAY (bus);
}

/* Use the ATA IDENTIFY command to find out what kind of drive is
 * attached to the given bus/slot. */
uint32
ata_identify (uint32 bus, uint32 drive)
{
  uint8 status;
  uint16 buffer[256];

  ata_drive_select (bus, drive);

  outb (0xEC, ATA_COMMAND (bus));       /* Send IDENTIFY command */

  ATA_SELECT_DELAY (bus);

  status = inb (ATA_COMMAND (bus));
  if (status == 0) {
    logger_printf ("ATA bus %X drive %X does not exist\n", bus, drive);
    return ATA_TYPE_NONE;
  }

  if (status & 0x1) {
    /* Drive does not support IDENTIFY.  Probably a CD-ROM. */
    goto guess_identity;
  }

  /* Poll the Status port (0x1F7) until bit 7 (BSY, value = 0x80)
   * clears, and bit 3 (DRQ, value = 8) sets -- or until bit 0 (ERR,
   * value = 1) sets. */

  while ((status = inb (ATA_COMMAND (bus))) & 0x80)     /* BUSY */
    asm volatile ("pause");

  while (!((status = inb (ATA_COMMAND (bus))) & 0x8) && !(status & 0x1))
    asm volatile ("pause");

  if (status & 0x1) {
    logger_printf ("ATA bus %X drive %X caused error.\n", bus, drive);
    goto guess_identity;
  }

  /* Read 256 words */
  insw (ATA_DATA (bus), buffer, 256);

#ifdef DEBUG_ATA
  {
    int i, j;

    DLOG ("IDENTIFY (bus: %X drive: %X) command output:", bus, drive);
    /* dump to com1 */
    for (i = 0; i < 32; i++) {
      for (j = 0; j < 8; j++) {
        com1_printf ("%.4X ", buffer[i * 32 + j]);
      }
      com1_printf ("\n");
    }
  }
#endif

  if (buffer[83] & (1 << 10))
    logger_printf ("LBA48 mode supported.\n");
  logger_printf ("LBA48 addressable sectors: %.4X %.4X %.4X %.4X\n",
                 buffer[100], buffer[101], buffer[102], buffer[103]);
  return ATA_TYPE_PATA;

guess_identity:{
    uint8 b1, b2;

    b1 = inb (ATA_ADDRESS2 (bus));
    b2 = inb (ATA_ADDRESS3 (bus));

    com1_printf ("ata_detect: %.2X %.2X\n", b1, b2);

    if (b1 == 0x14 && b2 == 0xEB) {
      logger_printf ("P-ATAPI detected\n");
      return ATA_TYPE_PATAPI;
    }
    if (b1 == 0x69 && b2 == 0x96) {
      logger_printf ("S-ATAPI detected\n");
      return ATA_TYPE_SATAPI;
    }
    if (b1 == 0x3C && b2 == 0xC3) {
      logger_printf ("SATA detected\n");
      return ATA_TYPE_SATA;
    }
    return ATA_TYPE_NONE;
  }
}

static void ata_poll_for_irq (uint32);

/* Read a sector from the bus/drive using LBA into the given buffer
 * and return bytes read. */
int
ata_drive_read_sector (uint32 bus, uint32 drive, uint32 lba, uint8 * buffer)
{
  uint8 status;
  int ret;
  ata_grab ();
  outb (drive | 0x40 /* LBA */  | ((lba >> 24) & 0x0F),
        ATA_DRIVE_SELECT (bus));
  ATA_SELECT_DELAY (bus);
  outb (0x1, ATA_SECTOR_COUNT (bus));
  outb ((uint8) lba, ATA_ADDRESS1 (bus));
  outb ((uint8) (lba >> 8), ATA_ADDRESS2 (bus));
  outb ((uint8) (lba >> 16), ATA_ADDRESS3 (bus));
  outb (0x20, ATA_COMMAND (bus));       /* READ SECTORS (28-bit LBA) */

  if (sched_enabled)
    schedule ();
  else
    ata_poll_for_irq (bus);

  while (!(status = inb (ATA_COMMAND (bus)) & 0x8) && !(status & 0x1))
    asm volatile ("pause");
  if (status & 0x1) {
    ret = -1;
    goto cleanup;
  }
  insw (ATA_DATA (bus), buffer, 256);

  ret = 512;

 cleanup:
  ata_release ();
  return ret;
}

/* Write a sector to the bus/drive using LBA from the given buffer
 * and return bytes written. */
int
ata_drive_write_sector (uint32 bus, uint32 drive, uint32 lba, uint8 * buffer)
{
  uint8 status;
  int i, ret;
  outb (drive | 0x40 /* LBA */  | ((lba >> 24) & 0x0F),
        ATA_DRIVE_SELECT (bus));
  ATA_SELECT_DELAY (bus);
  outb (0x1, ATA_SECTOR_COUNT (bus));
  outb ((uint8) lba, ATA_ADDRESS1 (bus));
  outb ((uint8) (lba >> 8), ATA_ADDRESS2 (bus));
  outb ((uint8) (lba >> 16), ATA_ADDRESS3 (bus));
  outb (0x30, ATA_COMMAND (bus));       /* WRITE SECTORS (28-bit LBA) */
  while (!(status = inb (ATA_COMMAND (bus)) & 0x8) && !(status & 0x1))
    asm volatile ("pause");
  if (status & 0x1) {
    ret = -1;
    goto cleanup;
  }

  /* ``Do not use REP OUTSW to transfer data. There must be a tiny
   * delay between each OUTSW output word. A jmp $+2 size of
   * delay. Make sure to do a Cache Flush (ATA command 0xE7) after
   * each write command completes.'' */

  for (i = 0; i < 256; i++)
    outw (((uint16 *) buffer)[i], ATA_DATA (bus));

  outb (0xE7, ATA_COMMAND (bus));       /* FLUSH */

  ret = 512;

 cleanup:
  ata_release ();
  return ret;
}

/* Count number of times IRQs are triggered. */
uint32 ata_primary_irq_count = 0, ata_secondary_irq_count = 0;
uint32 ata_irq_count = 0;
u64 irq_turnaround = 0, irq_response = 0, irq_start = 0, irq_resp_max = 0;
u64 irq_resp_min = ~0LL;

/* IRQ handler for both drive controllers. */
static uint32
ata_irq_handler (uint8 vec)
{
  lock_kernel ();
  DLOG ("ata_irq_handler(%x) ata_current_task=%x", vec,
        ata_current_task);
  if (vec == ATA_VECTOR_PRIMARY)
    ata_primary_irq_count++;
  else
    ata_secondary_irq_count++;

  if (irq_start != 0)
    ata_irq_count++;

  u64 finish;
  RDTSC (finish);
  if (irq_start != 0) {
    irq_response += finish - irq_start;
    if (irq_resp_max < finish - irq_start)
      irq_resp_max = finish - irq_start;
    if (irq_resp_min > finish - irq_start)
      irq_resp_min = finish - irq_start;
  }

  /* Unblock the task waiting for the IRQ. */
  if (ata_current_task)
    wakeup (ata_current_task);

  unlock_kernel ();
  return 0;
}

/* Temporary handler for timer IRQ. */
void
ata_poll_for_irq_timer_handler (void)
{
  send_eoi ();
  asm volatile ("leave");
  asm volatile ("iret");
}

/* Use this function if ATA IRQs are expected but scheduling is not
 * ready yet.  The original intention of this function was to disable
 * all but the disk IRQ and then wait for it to arrive.  This didn't
 * work out, so for the time being, it is just a hard-coded delay. */
static void
ata_poll_for_irq (uint32 bus)
{
#if 0
  uint32 count, *counter;
  idt_descriptor old_timer;
  /* Use this if scheduling is not enabled yet */
  counter =
    (bus ==
     ATA_BUS_PRIMARY ? &ata_primary_irq_count : &ata_secondary_irq_count);
  disable_idt ();
  get_idt_descriptor (0x20, &old_timer);
  set_idt_descriptor_by_addr (0x20, (void *) &ata_poll_for_irq_timer_handler,
                              0x3);
  enable_idt_entry (ATA_VECTOR (bus));
  count = *counter;
  asm volatile ("sti");
  while (count == *counter)
    asm volatile ("pause");
  asm volatile ("cli");
  set_idt_descriptor (0x20, &old_timer);
  enable_idt ();
#endif

  tsc_delay_usec (50000);      /* wait 50 milliseconds */
}

/* Initialize and identify the ATA drives in the system. */
bool
ata_init (void)
{
  uint32 bus, drive, i;

  i = 0;
  bus = ATA_BUS_PRIMARY;
  drive = ATA_DRIVE_MASTER;
  pata_drives[i].ata_type = ata_identify (bus, drive);
  pata_drives[i].ata_bus = bus;
  pata_drives[i].ata_drive = drive;

  i = 1;
  bus = ATA_BUS_PRIMARY;
  drive = ATA_DRIVE_SLAVE;
  pata_drives[i].ata_type = ata_identify (bus, drive);
  pata_drives[i].ata_bus = bus;
  pata_drives[i].ata_drive = drive;

  i = 2;
  bus = ATA_BUS_SECONDARY;
  drive = ATA_DRIVE_MASTER;
  pata_drives[i].ata_type = ata_identify (bus, drive);
  pata_drives[i].ata_bus = bus;
  pata_drives[i].ata_drive = drive;

  i = 3;
  bus = ATA_BUS_SECONDARY;
  drive = ATA_DRIVE_SLAVE;
  pata_drives[i].ata_type = ata_identify (bus, drive);
  pata_drives[i].ata_bus = bus;
  pata_drives[i].ata_drive = drive;

  if (mp_ISA_PC) {
    set_vector_handler ((ATA_IRQ_PRIMARY - 8) + PIC2_BASE_IRQ,
                        ata_irq_handler);
    set_vector_handler ((ATA_IRQ_SECONDARY - 8) + PIC2_BASE_IRQ,
                        ata_irq_handler);
  } else {
    IOAPIC_map_GSI (IRQ_to_GSI (mp_ISA_bus_id, ATA_IRQ_PRIMARY),
                    ATA_VECTOR_PRIMARY, 0xFF00000000000800LL);
    IOAPIC_map_GSI (IRQ_to_GSI (mp_ISA_bus_id, ATA_IRQ_SECONDARY),
                    ATA_VECTOR_SECONDARY, 0xFF00000000000800LL);
    set_vector_handler (ATA_VECTOR_PRIMARY, ata_irq_handler);
    set_vector_handler (ATA_VECTOR_SECONDARY, ata_irq_handler);
  }

  return TRUE;
}

u64 atapi_cycles=0, atapi_count=0, atapi_max=0, atapi_min=~0LL;
#define ATAPI_MEASURE_START                                     \
  u64 _atapi_start, _atapi_finish;                              \
  RDTSC (_atapi_start);
#define ATAPI_MEASURE_FINISH                                    \
  RDTSC (_atapi_finish);                                        \
  atapi_cycles += _atapi_finish - _atapi_start;                 \
  atapi_count++;                                                \
  if (_atapi_finish - _atapi_start > atapi_max) {               \
    atapi_max = _atapi_finish - _atapi_start;                   \
  }                                                             \
  if (_atapi_finish - _atapi_start < atapi_min) {               \
    atapi_min = _atapi_finish - _atapi_start;                   \
  }


/* ************************************************** */
/* ATAPI */

/* ATAPI is essentially SCSI commands over ATA. */

static u64 atapi_bytes = 0, atapi_timestamps = 0;

/* Indicate which IRQs to measure response time */
#define FIRST
#define SECOND

/* Use the ATAPI protocol to read a single sector from the given
 * bus/drive into the buffer. */
int
_atapi_drive_read_sector (uint32 bus, uint32 drive, uint32 lba, uint8 *buffer)
{
  /* 0xA8 is READ SECTORS command byte. */
  uint8 read_cmd[12] = { 0xA8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  uint8 status;
  int size;
#ifdef DEBUG_ATA
  int fake_size;
#endif
  task_id cpu = 0;

  ata_grab ();

  DLOG ("atapi_drive_read_sector(%X,%X,%X,%p)", bus, drive, lba,
        buffer);

  outb (drive & (1 << 4), ATA_DRIVE_SELECT (bus));      /* select drive (only slavebit needed) */
  ATA_SELECT_DELAY (bus);
  outb (0x0, ATA_FEATURES (bus));       /* PIO mode */
  outb (ATAPI_SECTOR_SIZE & 0xFF, ATA_ADDRESS2 (bus));
  outb (ATAPI_SECTOR_SIZE >> 8, ATA_ADDRESS3 (bus));
  outb (0xA0, ATA_COMMAND (bus));       /* ATA PACKET command */

  while ((status = inb (ATA_COMMAND (bus))) & 0x80)     /* BUSY */
    asm volatile ("pause");

  while (!((status = inb (ATA_COMMAND (bus))) & 0x8) && !(status & 0x1))
    asm volatile ("pause");
  /* DRQ or ERROR set */
  if (status & 0x1) {
    size = -1;
    goto cleanup;
  }

  read_cmd[9] = 1;              /* 1 sector */
  read_cmd[2] = (lba >> 0x18) & 0xFF;   /* most sig. byte of LBA */
  read_cmd[3] = (lba >> 0x10) & 0xFF;
  read_cmd[4] = (lba >> 0x08) & 0xFF;
  read_cmd[5] = (lba >> 0x00) & 0xFF;   /* least sig. byte of LBA */

  /* Send ATAPI/SCSI command */
  outsw (ATA_DATA (bus), (uint16 *) read_cmd, 6);

  /* Wait for IRQ. */
  if (sched_enabled) {
    /* Switch to IO-VCPU */
    cpu = lookup_TSS (str ())->cpu;
    set_iovcpu (str (), IOVCPU_CLASS_ATA | IOVCPU_CLASS_CDROM);
    extern vcpu *vcpu_lookup (int);
    vcpu_lookup (lookup_TSS (str ())->cpu)->T = vcpu_lookup (cpu)->T;

#ifdef FIRST
    ATAPI_MEASURE_START;
    u64 finish;
    RDTSC (irq_start);
#else
    irq_start = 0;
#endif
    schedule ();
#ifdef FIRST
    RDTSC (finish);
    irq_turnaround += finish - irq_start;
    ATAPI_MEASURE_FINISH;
#endif
  } else
    ata_poll_for_irq (bus);

  /* Wait for DRQ to set */
  while (!(inb (ATA_COMMAND (bus)) & 0x08))
    asm volatile ("pause");

  /* Read actual size */
  size =
    (((int) inb (ATA_ADDRESS3 (bus))) << 8) |
    (int) (inb (ATA_ADDRESS2 (bus)));

  DLOG ("atapi_drive_read_sector(%X,%X,%X,%p): actual size = %X",
        bus, drive, lba, buffer, size);

  /* Workaround possible size-reporting bug in hardware */
  if (size > ATAPI_SECTOR_SIZE)
    size = ATAPI_SECTOR_SIZE;

  /* Read data */
  insw (ATA_DATA (bus), buffer, size / 2);

  /* Wait for IRQ indicating next transfer ready.  It will be a
   * zero-byte transfer but we must still wait for the IRQ.*/
  if (sched_enabled) {
    /* Return to Main VCPU */
    lookup_TSS (str ())->cpu = cpu;

#ifdef SECOND
    ATAPI_MEASURE_START;
    u64 finish;
    RDTSC (irq_start);
#else
    irq_start = 0;
#endif
    schedule ();
#ifdef SECOND
    RDTSC (finish);
    irq_turnaround += finish - irq_start;
    ATAPI_MEASURE_FINISH;
#endif
  } else
    ata_poll_for_irq (bus);

#ifdef DEBUG_ATA
  /* Read size of "fake" transfer (may be 0) */
  fake_size =
    (((int) inb (ATA_ADDRESS3 (bus))) << 8) |
    (int) (inb (ATA_ADDRESS2 (bus)));

  DLOG ("atapi_drive_read_sector(%X,%X,%X,%p): fake size = %X",
        bus, drive, lba, buffer, fake_size);
#endif

  /* At this point we already have read all our data, but the hardware
   * may still report the same size value as before.  It is up to us
   * to know that this second IRQ fired but there is no more data to
   * be read. */

  /* To be sure, if the hardware sends me less than a sector at a
   * time, the above code is broken.  Not sure that will be an
   * issue. */

  /* Wait for BSY and DRQ to clear */
  while ((status = inb (ATA_COMMAND (bus))) & 0x88)
    asm volatile ("pause");

 cleanup:
  ata_release ();
  return size;
}

/* instrumentation variables */
u64 atapi_sector_read_time = 0, atapi_sector_cpu_time = 0;
u64 atapi_req_interval = 0;
static u64 prev_req = 0;
u32 atapi_req_count = 0;
u64 atapi_req_diff;

int
atapi_drive_read_sector (uint32 bus, uint32 drive, uint32 lba, uint8 * buffer)
{
  int size;
  u64 start = 0, finish;
  u64 vstart = 0, vfinish;

  /* begin instrumentation */
  if (sched_enabled) {
    RDTSC (start);
    vstart = vcpu_current_vtsc ();

    /* interval between sector requests */
    if (prev_req)
      atapi_req_interval = start - prev_req;
    prev_req = start;
  }

  /* invoke actual function */
  size = _atapi_drive_read_sector (bus, drive, lba, buffer);

  if (sched_enabled) {
    /* conclude instrumentation */
    RDTSC (finish);
    vfinish = vcpu_current_vtsc ();

    /* record size and timing info */
    atapi_bytes += size;
    atapi_timestamps += finish - start;
    atapi_sector_read_time += finish - start;
    atapi_sector_cpu_time += vfinish - vstart;

    /* req_diff measures the time between the end of one request and the
     * beginning of the next request */
    atapi_req_diff += atapi_req_interval - (finish - start);
    atapi_req_count++;
  }

  return size;
}

extern u32
atapi_sample_bps (void)
{
  extern u32 tsc_freq_msec;
  u64 atapi_msec = div64_64 (atapi_timestamps, (u64) tsc_freq_msec);
  u32 bytes_sec = 0;
  if (atapi_msec)
    bytes_sec = (u32) div64_64 (atapi_bytes * 1000, atapi_msec);
  atapi_bytes = 0;
  atapi_timestamps = 0;
  return bytes_sec;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = ata_init
};

DEF_MODULE (storage___ata, "ATA/ATAPI driver", &mod_ops, {});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
