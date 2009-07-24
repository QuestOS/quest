/*
 * multiboot.h
 *
 */

#ifndef _MULTIBOOT_H_
#define _MULTIBOOT_H_

#ifndef _ELF_H_
#include <elf.h>
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
