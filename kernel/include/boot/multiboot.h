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

/*
 * multiboot.h
 *
 */

#ifndef _MULTIBOOT_H_
#define _MULTIBOOT_H_

#ifndef _ELF_H_
#include "util/elf.h"
#endif

#include"types.h"

/* See 'info multiboot': Specification->Boot info format */
/* The memory map. Be careful that the offset 0 is `base_addr_low'
        instead of the `size' member. */
typedef struct memory_map
{
  uint32 size;
  uint32 base_addr_low;
  uint32 base_addr_high;
  uint32 length_low;
  uint32 length_high;
  uint32 type;
} memory_map_t;

typedef struct _multiboot_module
{
  Elf32_Ehdr *pe;
  void *mod_end;
  char *string;
  uint32 reserved;
} multiboot_module;

typedef struct _multiboot_drive
{
  uint32 size;
  uint8 number;
  uint8 mode;
  uint16 cylinders;
  uint8 heads;
  uint8 sectors;
  uint16 ports[];
} multiboot_drive;

typedef struct _multiboot
{
  uint32 flags;
  uint32 mem_lower;
  uint32 mem_upper;
  uint32 boot_device;
  char *cmdline;
  uint32 mods_count;
  multiboot_module *mods_addr;
  uint32 elf_num;
  uint32 elf_size;
  void *elf_addr;
  void *elf_shndx;
  uint32 mmap_length;
  uint32 mmap_addr;
  uint32 drives_length;
  multiboot_drive *drives_addr;
  void *config_table;
  char *boot_loader_name;
} multiboot;

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
