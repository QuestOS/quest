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

#include"fs/filesys.h"
#include"kernel.h"
#include"util/screen.h"

static int vfs_root_type = VFS_FSYS_NONE;

void
vfs_set_root (int type, ata_info * drive_info)
{
  vfs_root_type = type;
}

/* returns file length on success, -1 on failure */
int
vfs_dir (char *pathname)
{
  switch (vfs_root_type) {
  case VFS_FSYS_EZEXT2:
    return ext2fs_dir (pathname);
  case VFS_FSYS_EZISO:
    return eziso_dir (pathname);
  case VFS_FSYS_EZUSB:
    return vfat_dir (pathname);
  case VFS_FSYS_EZTFTP:
    return eztftp_dir (pathname);
  default:
    print ("Unknown vfs_root_type");
    return 0;
  }
}

/* returns number of bytes read */
int
vfs_read (char *buf, int len)
{
  switch (vfs_root_type) {
  case VFS_FSYS_EZEXT2:
    return ext2fs_read (buf, len);
  case VFS_FSYS_EZISO:
    return eziso_read (buf, len);
  case VFS_FSYS_EZUSB:
    return vfat_read (buf, len);
  case VFS_FSYS_EZTFTP:
    return eztftp_read (buf, len);
  default:
    print ("Unknown vfs_root_type");
    return 0;
  }
}

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
