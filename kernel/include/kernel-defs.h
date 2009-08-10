/* -*- Mode: C -*- */

#ifndef _KERNEL_DEFS_H_
#define _KERNEL_DEFS_H_

/* Define some constants for virtual addresses */
#define KERN_STK 0xFF800000     /* Kernel stack */
#define KERN_IDT 0xFFFEF000     /* Kernel IDT */
#define KERN_GDT 0xFFFEF800     /* kernel GDT */
#define KERN_SCR 0xFFFF0000     /* Screen (kernel virtual) memory  */
#define KERN_PGT 0xFFFF1000     /* kernel page table */

#define PHYS_INDEX_MAX 32768

#define NULL 0

#define NR_MODS 10              /* Establish support for modules loaded by
                                   grub at boot time */

#endif
