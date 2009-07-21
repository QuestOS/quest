#ifndef _FILESYS_H_
#define _FILESYS_H_

#include "types.h"
#include "ata.h"

#define PATHSEP '/'

int ext2fs_mount (void);
int ext2fs_read (char *buf, int len);
int ext2fs_dir (char *dirname);

struct _iso9660_dir_record {
  BYTE length;
  BYTE zero_1;
  DWORD first_sector;
  DWORD first_sector_BE;        /* big endian */
  DWORD data_length;
  DWORD data_length_BE;
  BYTE years_since_1900;
  BYTE month;
  BYTE day;
  BYTE hour;
  BYTE minute;
  BYTE second;
  BYTE GMT_offset;
  BYTE flag_hidden : 1;
  BYTE flag_dir : 1;
  BYTE flag_assoc : 1;
  BYTE flag_recfmtspec : 1;
  BYTE flag_permspec : 1;
  BYTE flag_unused : 2;
  BYTE flag_notfinal : 1;
  BYTE zero_2;
  BYTE zero_3;
  WORD volume_sequence;
  WORD volume_sequence_BE;
  BYTE identifier_length;
  BYTE identifier[32];
} PACKED;
typedef struct _iso9660_dir_record iso9660_dir_record;

typedef struct {
  DWORD bus, drive, root_dir_sector, root_dir_data_length;
} iso9660_mounted_info;

typedef struct {
  iso9660_mounted_info *mount;
  DWORD sector, offset, length;
} iso9660_handle;

int iso9660_mount(DWORD, DWORD, iso9660_mounted_info *);
int iso9660_read(iso9660_handle *, BYTE *, DWORD);
int iso9660_open(iso9660_mounted_info *, char *, iso9660_handle *);

int eziso_mount(DWORD bus, DWORD drive);
int eziso_dir(char *pathname);
int eziso_read(char *buf, int len);

#define VFS_FSYS_NONE  0
#define VFS_FSYS_EXT2  1
#define VFS_FSYS_EZISO 2

void vfs_set_root(int type, ata_info *drive_info);
int vfs_dir(char*);
int vfs_read(char*,int);

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
