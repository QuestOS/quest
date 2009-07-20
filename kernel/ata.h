#ifndef _ATA_H_
#define _ATA_H_
#include "types.h"

#define ATA_TYPE_NONE 0
#define ATA_TYPE_PATA 1
#define ATA_TYPE_PATAPI 2
#define ATA_TYPE_SATA 3
#define ATA_TYPE_SATAPI 4

typedef struct {
  DWORD ata_type;
} ata_info;

extern ata_info pata_drives[4];

#define ATA_BUS_PRIMARY     0x1F0
#define ATA_BUS_SECONDARY   0x170

#define ATA_FEATURES(x)     (x+1)
#define ATA_SECTOR_COUNT(x) (x+2)
#define ATA_ADDRESS1(x)     (x+3)
#define ATA_ADDRESS2(x)     (x+4)
#define ATA_ADDRESS3(x)     (x+5)
#define ATA_DRIVE_SELECT(x) (x+6)
#define ATA_COMMAND(x)      (x+7)
#define ATA_DCR(x)          (x+0x206) /* device control register */

#define ATA_DRIVE_MASTER    0xA0
#define ATA_DRIVE_SLAVE     0xB0

void ata_init(void);

#endif
