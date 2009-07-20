

#include "i386.h"
#include "printf.h"

/*

;  Technical Information on the ports:
;      Port    Read/Write   Misc
;     ------  ------------ -------------------------------------------------
;       1f0       r/w       data register, the bytes are written/read here
;       1f1       r         error register  (look these values up yourself)
;       1f2       r/w       sector count, how many sectors to read/write
;       1f3       r/w       sector number, the actual sector wanted
;       1f4       r/w       cylinder low, cylinders is 0-1024
;       1f5       r/w       cylinder high, this makes up the rest of the 1024
;       1f6       r/w       drive/head 
;                              bit 7 = 1
;                              bit 6 = 0
;                              bit 5 = 1
;                              bit 4 = 0  drive 0 select
;                                    = 1  drive 1 select
;                              bit 3-0    head select bits
;       1f7       r         status register
;                              bit 7 = 1  controller is executing a command
;                              bit 6 = 1  drive is ready
;                              bit 5 = 1  write fault
;                              bit 4 = 1  seek complete
;                              bit 3 = 1  sector buffer requires servicing
;                              bit 2 = 1  disk data read corrected
;                              bit 1 = 1  index - set to 1 each revolution
;                              bit 0 = 1  previous command ended in an error
;       1f7       w         command register
;                            commands:
;                              50h format track
;                              20h read sectors with retry
;                              21h read sectors without retry
;                              22h read long with retry
;                              23h read long without retry
;                              30h write sectors with retry
;                              31h write sectors without retry
;                              32h write long with retry
;                              33h write long without retry

*/

/* Read a sector using CHS geometry information */
void ReadSector( void *offset, int cylinder, int head, int sector ) {

  /* Setup drive 0, head 0 */
  outb ( 0xa0 + head, 0x1f6 );

  /* Setup count of sectors to read -- here one sector */
  outb ( 0x1, 0x1f2 );

  /* Read from selected sector */
  outb ( sector, 0x1f3 );

  /* Specify cylinder 0 -- using low port for cylinder */
  outb ( cylinder & 0xFF, 0x1f4 );

  /* Specify cylinder 0 -- using high port */
  outb ( cylinder >> 8 , 0x1f5 );

  /* Issue read sectors with retry command to command register */
  outb ( 0x20, 0x1f7 );

  while( !( inb ( 0x1f7 ) & 0x8 ) ); /* Wait until sector buffer requires 
					servicing */

  /* Read a sector of 512 bytes as 256 short words */
  insw( 0x1f0, offset, 256 );
  
}

/* Write a sector using CHS geometry information */
void WriteSector( void *offset, int cylinder, int head, int sector ) {

  /* Setup drive 0, head 0 */
  outb ( 0xa0 + head, 0x1f6 );

  /* Setup count of sectors to write -- here one sector */
  outb ( 0x1, 0x1f2 );

  /* Write to selected sector */
  outb ( sector, 0x1f3 );

  /* Specify cylinder 0 -- using low port for cylinder */
  outb ( cylinder & 0xFF, 0x1f4 );

  /* Specify cylinder 0 -- using high port */
  outb ( cylinder >> 8, 0x1f5 );

  /* Issue write sectors with retry command to command register */
  outb ( 0x30, 0x1f7 );

  while( !( inb ( 0x1f7 ) & 0x8 ) ); /* Wait until sector buffer requires 
					servicing */

  /* Write a sector of 512 bytes as 256 short words */
  outsw( 0x1f0, offset, 256 );
  
}

/* Read a sector using LBA information */
void ReadSectorLBA( void *offset, unsigned long lba ) {

  /* Setup drive 0 */
  outb ( ( lba >> 24 ) | 0xE0, 0x1f6 );

  /* Setup count of sectors to read -- here one sector */
  outb ( 0x1, 0x1f2 );

  outb ( lba & 0xFF, 0x1f3 );

  outb ( ( lba >> 8 ) & 0xFF, 0x1f4 );

  outb ( ( lba >> 16 ) & 0xFF, 0x1f5 );

  /* Issue read sectors with retry command to command register */
  outb ( 0x20, 0x1f7 );

  while( !( inb ( 0x1f7 ) & 0x8 ) ); /* Wait until sector buffer requires 
					servicing */

  /* Read a sector of 512 bytes as 256 short words */
  insw( 0x1f0, offset, 256 );
  
}

/* Write a sector using LBA information */
void WriteSectorLBA( void *offset, unsigned long lba ) {

  /* Setup drive 0 */
  outb ( ( lba >> 24 ) | 0xE0, 0x1f6 );

  /* Setup count of sectors to write -- here one sector */
  outb ( 0x1, 0x1f2 );

  outb ( lba & 0xFF, 0x1f3 );

  outb ( ( lba >> 8 ) & 0xFF, 0x1f4 );

  outb ( ( lba >> 16 ) & 0xFF, 0x1f5 );

  /* Issue write sectors with retry command to command register */
  outb ( 0x30, 0x1f7 );

  while( !( inb ( 0x1f7 ) & 0x8 ) ); /* Wait until sector buffer requires 
					servicing */

  /* Write a sector of 512 bytes as 256 short words */
  outsw( 0x1f0, offset, 256 );
  
}

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

#define DISKIO_SELECT_DELAY {inb(0x3F6); inb(0x3F6); inb(0x3F6); inb(0x3F6);}
void diskio_sreset(void) {
  outb(0x02, 0x3F6);
  outb(0x00, 0x3F6);
}

void diskio_detect(void) {
  unsigned char b1, b2;

  outb(0xA0, 0x1F6);            /* select Master Drive / Primary Bus */

  DISKIO_SELECT_DELAY;

  b1 = inb(0x1F4);
  b2 = inb(0x1F5);
  
  com1_printf("diskio_detect: %.2X %.2X\n", b1, b2);

  if(b1 == 0x00 && b2 == 0x00)
    com1_printf("PATA detected\n");
  if(b1 == 0x14 && b2 == 0xEB)
    com1_printf("P-ATAPI detected\n");
  if(b1 == 0x69 && b2 == 0x96)
    com1_printf("S-ATAPI detected\n");
  if(b1 == 0x3C && b2 == 0xC3)
    com1_printf("SATA detected\n");    
}

void diskio_identify(void) {
  unsigned char status;
  unsigned short buffer[256];
  int i,j;

  outb(0xA0, 0x1F6);            /* select Master Drive / Primary Bus */

  DISKIO_SELECT_DELAY;

  outb(0xEC, 0x1F7);            /* Send IDENTIFY command */

  status = inb(0x1F7);
  if(status == 0) { 
    com1_printf("ATA drive status == 0\n");

    diskio_detect();

    return;
  }

  /* Poll the Status port (0x1F7) until bit 7 (BSY, value = 0x80)
   * clears, and bit 3 (DRQ, value = 8) sets -- or until bit 0 (ERR,
   * value = 1) sets. */

  while((status = inb(0x1F7)) & 0x80) /* BUSY */
    asm volatile("pause");

  while(!(status & 0x8) && !(status & 0x1))
    asm volatile("pause");

  if(status & 0x1) {
    com1_printf("Drive Error!");
    return;
  }

  /* Read 256 words */
  insw(0x1F0, buffer, 256);

  com1_printf("IDENTIFY command output:\n");
  /* dump to com1 */
  for(i=0;i<32;i++) {
    for(j=0;j<8;j++) {
      com1_printf("%.4X ", buffer[i*32+j]);
    }
    com1_printf("\n");
  }

  if(buffer[83] & (1<<10)) com1_printf("LBA48 mode supported.\n");
  com1_printf("LBA48 addressable sectors: %.4X %.4X %.4X %.4X\n", buffer[100], buffer[101], buffer[102], buffer[103]);

}
