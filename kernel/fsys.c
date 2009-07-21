#include"filesys.h"
#include"kernel.h"

static int vfs_root_type = VFS_FSYS_NONE;

void vfs_set_root(int type, ata_info *drive_info) {
  vfs_root_type = type;
}

int vfs_dir(char *pathname) {
  switch(vfs_root_type) {
  case VFS_FSYS_EXT2:
    return ext2fs_dir(pathname);
  case VFS_FSYS_EZISO:
    return eziso_dir(pathname);
  default:
    panic("Unknown vfs_root_type");
    return 0;
  }
}

int vfs_read(char *buf, int len) {
  switch(vfs_root_type) {
  case VFS_FSYS_EXT2:
    return ext2fs_read(buf,len);
  case VFS_FSYS_EZISO:
    return eziso_read(buf,len);
  default:
    panic("Unknown vfs_root_type");
    return 0;
  }
}
