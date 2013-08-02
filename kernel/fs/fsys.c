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

#include"fs/filesys.h"
#include"kernel.h"
#include"util/screen.h"
#include"util/printf.h"

static int vfs_root_type = VFS_FSYS_NONE;

typedef struct { char *name; int type; } vfs_table_t;
static vfs_table_t vfs_table[] = {
  { "hd",   VFS_FSYS_EZEXT2 },
  { "cd",   VFS_FSYS_EZISO },
  { "tftp", VFS_FSYS_EZTFTP },
  { "usb",  VFS_FSYS_EZUSB },
  { "ram",  VFS_FSYS_EZRAM },
};
#define NUM_VFS (sizeof (vfs_table) / sizeof (vfs_table_t))

void
vfs_set_root (int type, ata_info * drive_info)
{
  vfs_root_type = type;
}

static int
parse_pathname (char *pathname, char **filepart)
{
  if (pathname[0] == '(') {
    int i;
    for (i=0; i<NUM_VFS; i++) {
      char *name = vfs_table[i].name;
      char *vfs = pathname + 1;
      for (; *vfs && *vfs != ')' && *name; vfs++, name++) {
        if (*vfs != *name)
          break;
      }
      if (*name == '\0' && *vfs == ')') {
        *filepart = vfs + 1;
        return vfs_table[i].type;
      }
    }
    return -1;
  } else {
    *filepart = pathname;
    return vfs_root_type;
  }
}

/* returns file length on success, -1 on failure */
int
vfs_dir (char *pathname)
{
  char *filepart;
  int type = parse_pathname (pathname, &filepart);
  if (type == -1) return -1;
  switch (type) {
  case VFS_FSYS_EZEXT2:
    return ext2fs_dir (filepart);
  case VFS_FSYS_EZISO:
    return eziso_dir (filepart);
  case VFS_FSYS_EZUSB:
    return vfat_dir (filepart);
  case VFS_FSYS_EZTFTP:
    return eztftp_dir (filepart);
  case VFS_FSYS_EZRAM:
    return ramdisk_dir(filepart);
  default:
    print ("Unknown vfs_type");
    return -1;
  }
}

/* returns number of bytes read */
int
vfs_read (char *pathname, char *buf, int len)
{
  char *filepart;
  int type = parse_pathname (pathname, &filepart);
  if (type == -1) return -1;
  switch (type) {
  case VFS_FSYS_EZEXT2:
    return ext2fs_read (buf, len);
  case VFS_FSYS_EZISO:
    return eziso_read (buf, len);
  case VFS_FSYS_EZUSB:
    return vfat_read (buf, len);
  case VFS_FSYS_EZTFTP:
    return eztftp_read (buf, len);
  case VFS_FSYS_EZRAM:
    return ramdisk_read(buf, len);
  default:
    print ("Unknown vfs_type");
    return -1;
  }
}

/* ************************************************** */

bool
vfs_init (void)
{
  extern u32 root_type, boot_device;
  int i;
  switch (root_type) {
  case VFS_FSYS_EZEXT2:
    if (boot_device == 0x8000FFFF && pata_drives[0].ata_type == ATA_TYPE_PATA) {
      printf ("ROOT: HARD DISK DRIVE: EXT2\n");
      /* Mount root filesystem */
      if (!ext2fs_mount ())
        panic ("Filesystem mount failed");
      vfs_set_root (VFS_FSYS_EZEXT2, &pata_drives[0]);
    } else {
      printf ("ROOT: unable to find ext2 drive\n");
    }
    break;
  case VFS_FSYS_EZUSB:
  default:
    printf ("ROOT: USB MASS STORAGE: VFAT\n");
    vfat_mount ();
    vfs_set_root (VFS_FSYS_EZUSB, NULL);
    break;
  case VFS_FSYS_EZISO:
    for (i = 0; i < 4; i++) {
      if (pata_drives[i].ata_type == ATA_TYPE_PATAPI) {
        printf ("ROOT: CD-ROM: ISO-9660\n");
        if (!eziso_mount (pata_drives[i].ata_bus, pata_drives[i].ata_drive))
          panic ("Filesystem mount failed");
        vfs_set_root (VFS_FSYS_EZISO, &pata_drives[i]);
        break;
      }
    }
    if (i == 4)
      printf ("Unable to detect CD-ROM drive.\n");
    break;
  case VFS_FSYS_EZTFTP:
    printf ("ROOT: TFTP\n");
    if (!eztftp_mount ("en0"))
      panic ("TFTP mount failed");
    vfs_set_root (VFS_FSYS_EZTFTP, NULL);
    break;

  case VFS_FSYS_EZRAM:
    {
      printf("ROOT: RAMDISK\n");
      if(!ramdisk_mount()) {
        com1_printf("Ramdisk mount failed\n");
        panic("Ramdisk mount failed");
      }
      vfs_set_root(VFS_FSYS_EZRAM, NULL);
    }
    break;
  }
  return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = vfs_init
};

DEF_MODULE (vfs, "virtual filesystem switch", &mod_ops, {"storage___ata|usbenumeration|netsetup"});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
