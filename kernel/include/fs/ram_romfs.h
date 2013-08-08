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

#ifndef _RAM_ROMFS_H_
#define _RAM_ROMFS_H_

#include "types.h"
#include "boot/multiboot.h"

bool copy_ramdisk_module(multiboot_module *pmm, int mod_num);

bool ramdisk_mount();

int ramdisk__dir (char *pathname);
int ramdisk_read (char *buf, int len);

typedef enum {
  ROMFS_HARD_LINK = 0,
  ROMFS_DIRECTORY,
  ROMFS_REGULAR_FILE,
  ROMFS_SYMBOLIC_LINK,
  ROMFS_BLOCK_DEVICE,
  ROMFS_CHAR_DEVICE,
  ROMFS_SOCKET,
  ROMFS_FIFO,
} ROMFS_FILE_TYPE;

#define romfs_get_file_type(file_header) (be32toh(file_header->next) & 0x7)


typedef struct {
  uint32_t next;
  uint32_t spec_info;
  uint32_t size;
  uint32_t checksum;
  char filename[16];
  unsigned char file_data[0];
} romfs_file_header_t;

typedef struct {
  char name[8];
  uint32_t full_size;
  uint32_t checksum;
  char volumne_name[16];
  romfs_file_header_t file_headers[0];
} romfs_t;

#define FIRST_FILE_HEADER_OFFSET (sizeof(romfs_t))


#endif // _RAM_ROMFS_H_


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
