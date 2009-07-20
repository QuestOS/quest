#ifndef _ATA_H_
#define _ATA_H_
#include "types.h"

#define ATA_TYPE_NONE 0
#define ATA_TYPE_PATA 1
#define ATA_TYPE_PATAPI 2
#define ATA_TYPE_SATA 3
#define ATA_TYPE_SATAPI 4

typedef struct {
  DWORD ata_type, ata_bus, ata_drive;
} ata_info;

extern ata_info pata_drives[4];

#define ATA_BUS_PRIMARY     0x1F0
#define ATA_BUS_SECONDARY   0x170

#define ATA_IRQ_PRIMARY     0x0E
#define ATA_IRQ_SECONDARY   0x0F
#define ATA_VECTOR_PRIMARY   0x27
#define ATA_VECTOR_SECONDARY 0x28
#define ATA_IRQ(bus)        (bus==ATA_BUS_PRIMARY ? ATA_IRQ_PRIMARY : ATA_IRQ_SECONDARY)
#define ATA_VECTOR(bus)     (bus==ATA_BUS_PRIMARY ? ATA_VECTOR_PRIMARY : ATA_VECTOR_SECONDARY)

#define ATA_DATA(x)         (x)
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
int  ata_drive_read_sector(DWORD bus, DWORD drive, DWORD lba, BYTE *buffer);
int  ata_drive_write_sector(DWORD bus, DWORD drive, DWORD lba, BYTE *buffer);
int  atapi_drive_read_sector(DWORD bus, DWORD drive, DWORD lba, BYTE *buffer);

#endif
