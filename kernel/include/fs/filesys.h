/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
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

#ifndef _FILESYS_H_
#define _FILESYS_H_

#include "types.h"
#include "drivers/ata/ata.h"

#define PATHSEP '/'

int ext2fs_mount (void);
int ext2fs_read (char *buf, int len);
int ext2fs_dir (char *dirname);

struct _iso9660_dir_record
{
  uint8 length;
  uint8 zero_1;
  uint32 first_sector;
  uint32 first_sector_BE;       /* big endian */
  uint32 data_length;
  uint32 data_length_BE;
  uint8 years_since_1900;
  uint8 month;
  uint8 day;
  uint8 hour;
  uint8 minute;
  uint8 second;
  uint8 GMT_offset;
  uint8 flag_hidden:1;
  uint8 flag_dir:1;
  uint8 flag_assoc:1;
  uint8 flag_recfmtspec:1;
  uint8 flag_permspec:1;
  uint8 flag_unused:2;
  uint8 flag_notfinal:1;
  uint8 zero_2;
  uint8 zero_3;
  uint16 volume_sequence;
  uint16 volume_sequence_BE;
  uint8 identifier_length;
  uint8 identifier[32];
} PACKED;
typedef struct _iso9660_dir_record iso9660_dir_record;

typedef struct
{
  uint32 bus, drive, root_dir_sector, root_dir_data_length;
} iso9660_mounted_info;

typedef struct
{
  iso9660_mounted_info *mount;
  uint32 sector, offset, length;
} iso9660_handle;

int iso9660_mount (uint32, uint32, iso9660_mounted_info *);
int iso9660_read (iso9660_handle *, uint8 *, uint32);
int iso9660_open (iso9660_mounted_info *, char *, iso9660_handle *);

int eziso_mount (uint32 bus, uint32 drive);
int eziso_dir (char *pathname);
int eziso_read (char *buf, int len);

int vfat_mount (void);
int vfat_dir (char *pathname);
int vfat_read (char *buf, int len);

#define VFS_FSYS_NONE  0
#define VFS_FSYS_EXT2  1
#define VFS_FSYS_EZISO 2
#define VFS_FSYS_EZUSB 3

void vfs_set_root (int type, ata_info * drive_info);
int vfs_dir (char *);
int vfs_read (char *, int);

#define SECTOR_SIZE            0x200

/* Error codes (taken from grub) */
typedef enum
{
  ERR_NONE = 0,
  ERR_BAD_FILENAME,
  ERR_BAD_FILETYPE,
  ERR_BAD_GZIP_DATA,
  ERR_BAD_GZIP_HEADER,
  ERR_BAD_PART_TABLE,
  ERR_BAD_VERSION,
  ERR_BELOW_1MB,
  ERR_BOOT_COMMAND,
  ERR_BOOT_FAILURE,
  ERR_BOOT_FEATURES,
  ERR_DEV_FORMAT,
  ERR_DEV_VALUES,
  ERR_EXEC_FORMAT,
  ERR_FILELENGTH,
  ERR_FILE_NOT_FOUND,
  ERR_FSYS_CORRUPT,
  ERR_FSYS_MOUNT,
  ERR_GEOM,
  ERR_NEED_LX_KERNEL,
  ERR_NEED_MB_KERNEL,
  ERR_NO_DISK,
  ERR_NO_PART,
  ERR_NUMBER_PARSING,
  ERR_OUTSIDE_PART,
  ERR_READ,
  ERR_SYMLINK_LOOP,
  ERR_UNRECOGNIZED,
  ERR_WONT_FIT,
  ERR_WRITE,
  ERR_BAD_ARGUMENT,
  ERR_UNALIGNED,
  ERR_PRIVILEGED,
  ERR_DEV_NEED_INIT,
  ERR_NO_DISK_SPACE,
  ERR_NUMBER_OVERFLOW,

  MAX_ERR_NUM
} grub_error_t;

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
