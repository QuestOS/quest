#include "ata.h"
#include "i386.h"
#include "printf.h"

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

  status = inb(ATA_COMMAND(bus));
  if(status == 0) { 
    com1_printf("ATA bus %X drive %X does not exist\n", bus, drive);
    return ATA_TYPE_NONE;
  }

  if(status & 0x1) {
    unsigned char b1, b2;

    b1 = inb(0x1F4);
    b2 = inb(0x1F5);
  
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

  /* Poll the Status port (0x1F7) until bit 7 (BSY, value = 0x80)
   * clears, and bit 3 (DRQ, value = 8) sets -- or until bit 0 (ERR,
   * value = 1) sets. */

  while((status = inb(0x1F7)) & 0x80) /* BUSY */
    asm volatile("pause");

  while(!(status & 0x8) && !(status & 0x1))
    asm volatile("pause");

  if(status & 0x1) {
    com1_printf("ATA bus %X drive %X caused error.\n", bus, drive);
    return ATA_TYPE_NONE;
  }

  /* Read 256 words */
  insw(0x1F0, buffer, 256);

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
}

void ata_init(void) {
  pata_drives[0].ata_type = ata_identify(ATA_BUS_PRIMARY, ATA_DRIVE_MASTER);
  pata_drives[1].ata_type = ata_identify(ATA_BUS_PRIMARY, ATA_DRIVE_SLAVE);
  pata_drives[2].ata_type = ata_identify(ATA_BUS_SECONDARY, ATA_DRIVE_MASTER);
  pata_drives[3].ata_type = ata_identify(ATA_BUS_SECONDARY, ATA_DRIVE_SLAVE);
}
