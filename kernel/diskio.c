

#include "i386.h"
#include "ata.h"
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
  ata_drive_read_sector(ATA_BUS_PRIMARY, ATA_DRIVE_MASTER, lba, (BYTE *)offset);
}

/* Write a sector using LBA information */
void WriteSectorLBA( void *offset, unsigned long lba ) {
  ata_drive_write_sector(ATA_BUS_PRIMARY, ATA_DRIVE_MASTER, lba, (BYTE *)offset);
}
