#include "kernel.h"
#include "filesys.h"
#include "i386.h"
#include "printf.h"
#include "ata.h"

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
