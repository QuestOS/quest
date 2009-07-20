#include "kernel.h"
#include "filesys.h"
#include "i386.h"
#include "printf.h"
#include "ata.h"

void iso9660_date_record(BYTE *buf) {
  com1_printf("%.4s-%.2s-%.2s %.2s:%.2s:%.2s.%.2s %02d",
              buf, buf+4, buf+6, 
              buf+8, buf+10, buf+12, buf+14, *((signed char *)buf+16)/4);
}

int iso9660_mount(DWORD bus, DWORD drive) {
  BYTE *page = MapVirtualPage(AllocatePhysicalPage() | 3);
  int len;
  
  /* The first 16 sectors (0-15) are empty. */

  /* Primary Volume descriptor */
  len = atapi_drive_read_sector(bus, drive, 16, page);
  if(len < 0) {
    com1_printf("CD-ROM read error\n");
    return -1;
  } else {
    int i,j;
    com1_printf("Read %d bytes.\n", len);
    /* dump to com1 */
    for(i=0;i<128;i++) {
      for(j=0;j<16;j++) {
        com1_printf("%.2X ", page[i*16+j]);
      }
      com1_printf("\n");
    }
  }

  com1_printf("Sys ID: %.32s\n", page+8);
  com1_printf("Vol ID: %.32s\n", page+40);
  com1_printf("# Sectors: %.8X\n", *(DWORD *)(page+80));
  com1_printf("Path Table: len=%.8X fst=", *(DWORD *)(page+132));
  com1_printf("%.8X snd=%.8X\n", *(DWORD *)(page+140), *(DWORD *)(page+144));
  /* root dir @ 156 */
  com1_printf("Vol Set ID: %.128s\n", page+190);
  com1_printf("Pub ID: %.128s\n", page+318);
  com1_printf("Preparer ID: %.128s\n", page+318+128);
  com1_printf("Application ID: %.128s\n", page+318+256);
  com1_printf("Copyright File ID: %.37s\n", page+318+384);
  com1_printf("Abstract File ID: %.37s\n", page+318+384+37);
  com1_printf("Bib File ID: %.37s\n", page+318+384+37+37);
  com1_printf("Creation Date: ");
  iso9660_date_record(page+318+384+37+37+37);
  com1_printf("\n");
  com1_printf("Modification Date: ");
  iso9660_date_record(page+318+384+37+37+37+17);
  com1_printf("\n");
  com1_printf("Expiration Date: ");
  iso9660_date_record(page+318+384+37+37+37+34);
  com1_printf("\n");
  com1_printf("Effective Date: ");
  iso9660_date_record(page+318+384+37+37+37+51);
  com1_printf("\n");
  
  
#if 0
  len = atapi_drive_read_sector(bus, drive, 17, page);
  if(len < 0) {
    com1_printf("CD-ROM read error\n");
    return -1;
  } else {
    int i,j;
    com1_printf("Read %d bytes.\n", len);
    /* dump to com1 */
    for(i=0;i<128;i++) {
      for(j=0;j<16;j++) {
        com1_printf("%.2X ", page[i*16+j]);
      }
      com1_printf("\n");
    }
  }
#endif
  
  return 0;
}
