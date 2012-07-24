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

#ifndef _ATA_H_
#define _ATA_H_
#include "types.h"

#define ATA_TYPE_NONE 0
#define ATA_TYPE_PATA 1
#define ATA_TYPE_PATAPI 2
#define ATA_TYPE_SATA 3
#define ATA_TYPE_SATAPI 4

typedef struct
{
  uint32 ata_type, ata_bus, ata_drive;
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
#define ATA_DCR(x)          (x+0x206)   /* device control register */

#define ATA_DRIVE_MASTER    0xA0
#define ATA_DRIVE_SLAVE     0xB0

/* The default and seemingly universal sector size for CD-ROMs. */
#define ATAPI_SECTOR_SIZE 2048

bool ata_init (void);
int ata_drive_read_sector (uint32 bus, uint32 drive, uint32 lba,
                           uint8 * buffer);
int ata_drive_write_sector (uint32 bus, uint32 drive, uint32 lba,
                            uint8 * buffer);
int atapi_drive_read_sector (uint32 bus, uint32 drive, uint32 lba,
                             uint8 * buffer);

#endif

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
