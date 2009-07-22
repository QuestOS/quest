#include "ata.h"
#include "i386.h"
#include "printf.h"
#include "smp.h"
#include "kernel.h"

//#define DEBUG_ATA

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
 *     * Words 60 & 61 taken as a DWORD contain the total number of 28    *
 *     * bit LBA addressable sectors on the drive. (If non-zero, the      *
 *     * drive supports LBA28.)                                           *
 *                                                                        *
 *     * Words 100 through 103 taken as a QWORD contain the total number  *
 *     * of 48 bit addressable sectors on the drive. (Probably also proof *
 *     * that LBA48 is supported.)                                        *
 **************************************************************************/

ata_info pata_drives[4];
/* ata_current_task has exclusive access to ATA.  If an ATA IRQ comes
 * in, we assume that ata_current_task is waiting on it and needs to
 * be woken. */
static WORD ata_current_task = 0;
/* waitqueue of tasks that want to use ATA. */
static WORD ata_waitqueue = 0;

/* technically I think there could be separate queues for each bus,
 * but, whatever. */

/* Kernel lock should be held while using ATA */

/* relinquish CPU until we get exclusive access to the ATA subsystem */
static void ata_grab(void) {
#ifdef DEBUG_ATA
  com1_printf("ata_grab() ata_current_task=%x ata_waitqueue=%x tr=%x\n", 
              ata_current_task, ata_waitqueue, str());
#endif
  while(ata_current_task) {
    queue_append(&ata_waitqueue, str());
    schedule();
  }
  ata_current_task = str();
}

static void ata_release(void) {
#ifdef DEBUG_ATA
  com1_printf("ata_release() ata_current_task=%x ata_waitqueue=%x tr=%x\n", 
              ata_current_task, ata_waitqueue, str());
#endif
  wakeup_queue(&ata_waitqueue);
  ata_waitqueue = 0;
  ata_current_task = 0;
}

#define ATA_SELECT_DELAY(bus) \
  {inb(ATA_DCR(bus));inb(ATA_DCR(bus));inb(ATA_DCR(bus));inb(ATA_DCR(bus));}

void ata_sreset(DWORD bus) {
  outb(0x02, ATA_DCR(bus));
  outb(0x00, ATA_DCR(bus));
}

void ata_drive_select(DWORD bus, DWORD drive) {
  outb(drive, ATA_DRIVE_SELECT(bus));
  ATA_SELECT_DELAY(bus);
}

DWORD ata_identify(DWORD bus, DWORD drive) {
  unsigned char status;
  unsigned short buffer[256];
  int i,j;

  ata_drive_select(bus,drive);

  outb(0xEC, ATA_COMMAND(bus)); /* Send IDENTIFY command */

  ATA_SELECT_DELAY(bus);

  status = inb(ATA_COMMAND(bus));
  if(status == 0) { 
    com1_printf("ATA bus %X drive %X does not exist\n", bus, drive);
    return ATA_TYPE_NONE;
  }

  if(status & 0x1) {
    goto guess_identity;
  }

  /* Poll the Status port (0x1F7) until bit 7 (BSY, value = 0x80)
   * clears, and bit 3 (DRQ, value = 8) sets -- or until bit 0 (ERR,
   * value = 1) sets. */

  while((status = inb(ATA_COMMAND(bus))) & 0x80) /* BUSY */
    asm volatile("pause");

  while(!((status = inb(ATA_COMMAND(bus))) & 0x8) && !(status & 0x1))
    asm volatile("pause");

  if(status & 0x1) {
    com1_printf("ATA bus %X drive %X caused error.\n", bus, drive);
    goto guess_identity;
  }

  /* Read 256 words */
  insw(ATA_DATA(bus), buffer, 256);

  com1_printf("IDENTIFY (bus: %X drive: %X) command output:\n", bus, drive);
  /* dump to com1 */
  for(i=0;i<32;i++) {
    for(j=0;j<8;j++) {
      com1_printf("%.4X ", buffer[i*32+j]);
    }
    com1_printf("\n");
  }

  if(buffer[83] & (1<<10)) com1_printf("LBA48 mode supported.\n");
  com1_printf("LBA48 addressable sectors: %.4X %.4X %.4X %.4X\n", buffer[100], buffer[101], buffer[102], buffer[103]);
  return ATA_TYPE_PATA;

 guess_identity: {
    unsigned char b1, b2;

    b1 = inb(ATA_ADDRESS2(bus));
    b2 = inb(ATA_ADDRESS3(bus));
  
    com1_printf("ata_detect: %.2X %.2X\n", b1, b2);

    if(b1 == 0x14 && b2 == 0xEB) {
      com1_printf("P-ATAPI detected\n");
      return ATA_TYPE_PATAPI;
    }
    if(b1 == 0x69 && b2 == 0x96) {
      com1_printf("S-ATAPI detected\n");
      return ATA_TYPE_SATAPI;
    }
    if(b1 == 0x3C && b2 == 0xC3) {
      com1_printf("SATA detected\n");    
      return ATA_TYPE_SATA;
    }
    return ATA_TYPE_NONE;
  }
}

static void ata_poll_for_irq(DWORD);

int ata_drive_read_sector(DWORD bus, DWORD drive, DWORD lba, BYTE *buffer) {
  BYTE status;
  ata_grab();
  outb(drive | 0x40 /* LBA */ | ((lba >> 24) & 0x0F), ATA_DRIVE_SELECT(bus));
  ATA_SELECT_DELAY(bus);
  outb(0x1, ATA_SECTOR_COUNT(bus));
  outb((BYTE)lba, ATA_ADDRESS1(bus));
  outb((BYTE)(lba >> 8), ATA_ADDRESS2(bus));
  outb((BYTE)(lba >> 16), ATA_ADDRESS3(bus));
  outb(0x20, ATA_COMMAND(bus)); /* READ SECTORS (28-bit LBA) */

  if(sched_enabled) schedule();
  else ata_poll_for_irq(bus);

  while(!(status=inb(ATA_COMMAND(bus)) & 0x8) && !(status & 0x1))
    asm volatile("pause");
  if(status & 0x1) {
    ata_release();
    return -1;
  }
  insw(ATA_DATA(bus), buffer, 256);
  ata_release();
  return 512; 
}

int ata_drive_write_sector(DWORD bus, DWORD drive, DWORD lba, BYTE *buffer) {
  BYTE status;
  int i;
  outb(drive | 0x40 /* LBA */ | ((lba >> 24) & 0x0F), ATA_DRIVE_SELECT(bus));
  ATA_SELECT_DELAY(bus);
  outb(0x1, ATA_SECTOR_COUNT(bus));
  outb((BYTE)lba, ATA_ADDRESS1(bus));
  outb((BYTE)(lba >> 8), ATA_ADDRESS2(bus));
  outb((BYTE)(lba >> 16), ATA_ADDRESS3(bus));
  outb(0x30, ATA_COMMAND(bus)); /* WRITE SECTORS (28-bit LBA) */
  while(!(status=inb(ATA_COMMAND(bus)) & 0x8) && !(status & 0x1))
    asm volatile("pause");
  if(status & 0x1) {
    ata_release();
    return -1;
  }

  /* ``Do not use REP OUTSW to transfer data. There must be a tiny
   * delay between each OUTSW output word. A jmp $+2 size of
   * delay. Make sure to do a Cache Flush (ATA command 0xE7) after
   * each write command completes.'' */
  
  for(i=0;i<256;i++)
    outw(((WORD *)buffer)[i], ATA_DATA(bus));

  outb(0xE7, ATA_COMMAND(bus)); /* FLUSH */
  ata_release();
  return 512; 
}

static DWORD ata_primary_irq_count = 0, ata_secondary_irq_count = 0;
static unsigned ata_irq_handler(BYTE vec) {
  lock_kernel();
#ifdef DEBUG_ATA
  com1_printf("ata_irq_handler(%x) ata_current_task=%x\n", vec, ata_current_task);
#endif
  if(vec == ATA_VECTOR_PRIMARY) ata_primary_irq_count++;
  else ata_secondary_irq_count++;
  if(ata_current_task) wakeup(ata_current_task);
  unlock_kernel();
  return 0;
}


void ata_poll_for_irq_timer_handler(void) {
  send_eoi();
  asm volatile("leave");
  asm volatile("iret");
}

static void ata_poll_for_irq(DWORD bus) {
#if 0
  DWORD count, *counter;
  idt_descriptor old_timer;
  /* Use this if scheduling is not enabled yet */
  counter = (bus == ATA_BUS_PRIMARY ? &ata_primary_irq_count : &ata_secondary_irq_count);
  disable_idt();
  get_idt_descriptor(0x20, &old_timer);
  set_idt_descriptor_by_addr(0x20, (void *)&ata_poll_for_irq_timer_handler, 0x3);
  enable_idt_entry(ATA_VECTOR(bus));
  count = *counter;
  asm volatile("sti");
  while(count == *counter) asm volatile("pause");
  asm volatile("cli");
  set_idt_descriptor(0x20, &old_timer);
  enable_idt();
#endif

  tsc_delay_usec(100000);       /* wait 100 milliseconds */
}


void ata_init(void) {
  DWORD bus, drive, i;
  
  i=0; bus=ATA_BUS_PRIMARY; drive=ATA_DRIVE_MASTER;
  pata_drives[i].ata_type   = ata_identify(bus, drive);
  pata_drives[i].ata_bus    = bus;
  pata_drives[i].ata_drive  = drive;

  i=1; bus=ATA_BUS_PRIMARY; drive=ATA_DRIVE_SLAVE;
  pata_drives[i].ata_type   = ata_identify(bus, drive);
  pata_drives[i].ata_bus    = bus;
  pata_drives[i].ata_drive  = drive;

  i=2; bus=ATA_BUS_SECONDARY; drive=ATA_DRIVE_MASTER;
  pata_drives[i].ata_type   = ata_identify(bus, drive);
  pata_drives[i].ata_bus    = bus;
  pata_drives[i].ata_drive  = drive;

  i=3; bus=ATA_BUS_SECONDARY; drive=ATA_DRIVE_SLAVE;
  pata_drives[i].ata_type   = ata_identify(bus, drive);
  pata_drives[i].ata_bus    = bus;
  pata_drives[i].ata_drive  = drive;
  
  IOAPIC_map_GSI(IRQ_to_GSI(mp_ISA_bus_id, ATA_IRQ_PRIMARY), 
                 ATA_VECTOR_PRIMARY, 0xFF00000000000800LL);
  IOAPIC_map_GSI(IRQ_to_GSI(mp_ISA_bus_id, ATA_IRQ_SECONDARY), 
                 ATA_VECTOR_SECONDARY, 0xFF00000000000800LL);
  set_vector_handler(ATA_VECTOR_PRIMARY, ata_irq_handler);
  set_vector_handler(ATA_VECTOR_SECONDARY, ata_irq_handler);
}

/* ************************************************** */
/* ATAPI */

#define ATAPI_SECTOR_SIZE 2048

int atapi_drive_read_sector(DWORD bus, DWORD drive, DWORD lba, BYTE *buffer) {
  BYTE read_cmd[12] = {0xA8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  BYTE status;
  WORD size;
  ata_grab();
#ifdef DEBUG_ATA
  com1_printf("atapi_drive_read_sector(%X,%X,%X,%p)\n",bus,drive,lba,buffer);
#endif
  outb(drive & (1<<4), ATA_DRIVE_SELECT(bus)); /* select drive (only slavebit needed) */
  ATA_SELECT_DELAY(bus);
  outb(0x0, ATA_FEATURES(bus)); /* PIO mode */
  outb(ATAPI_SECTOR_SIZE & 0xFF, ATA_ADDRESS2(bus));
  outb(ATAPI_SECTOR_SIZE >> 8, ATA_ADDRESS3(bus));
  outb(0xA0, ATA_COMMAND(bus)); /* PACKET command */

  while((status = inb(ATA_COMMAND(bus))) & 0x80) /* BUSY */
    asm volatile("pause");

  while(!((status = inb(ATA_COMMAND(bus))) & 0x8) && !(status & 0x1))
    asm volatile("pause");
  /* DRQ or ERROR set */
  if(status & 0x1) return -1;   /* error */

  read_cmd[9] = 1;                  /* 1 sector */
  read_cmd[2] = (lba >> 0x18) & 0xFF; /* most sig. byte of LBA */
  read_cmd[3] = (lba >> 0x10) & 0xFF;
  read_cmd[4] = (lba >> 0x08) & 0xFF;
  read_cmd[5] = (lba >> 0x00) & 0xFF; /* least sig. byte of LBA */

  /* Send ATAPI/SCSI command */
  outsw(ATA_DATA(bus), (WORD *)read_cmd, 6);
  
  if(sched_enabled) schedule();
  else ata_poll_for_irq(bus);

  /* Read actual size */
  size = (((WORD)inb(ATA_ADDRESS3(bus))) << 8) | (WORD)(inb(ATA_ADDRESS2(bus)));

  /* Read data */
  insw(ATA_DATA(bus), buffer, size/2);

  ata_release();
  return size;
}
