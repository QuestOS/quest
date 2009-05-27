/*
 * multiboot.h
 *
 */

#ifndef _MULTIBOOT_H_
#define _MULTIBOOT_H_

#ifndef _ELF_H_
#include <elf.h>
#endif

/* See 'info multiboot': Specification->Boot info format */
/* The memory map. Be careful that the offset 0 is `base_addr_low'
        instead of the `size' member. */
typedef struct memory_map {
    unsigned long size;
    unsigned long base_addr_low;
    unsigned long base_addr_high;
    unsigned long length_low;
    unsigned long length_high;
    unsigned long type;
} memory_map_t;

typedef struct _multiboot_module {
    Elf32_Ehdr *pe;
    void *mod_end;
    char *string;
    unsigned long reserved;
} multiboot_module;

typedef struct _multiboot {
    unsigned long flags;
    unsigned long mem_lower;
    unsigned long mem_upper;
    unsigned long boot_device;
    char *cmdline;
    unsigned long mods_count;
    multiboot_module *mods_addr;
    unsigned long elf_num;
    unsigned long elf_size;
    void *elf_addr;
    void *elf_shndx;
    unsigned long mmap_length;
    unsigned long mmap_addr;
    unsigned long drives_length;
    void *drives_addr;
    void *config_table;
    char *boot_loader_name;
} multiboot;

#endif
