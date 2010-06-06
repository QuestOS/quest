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

#ifndef _KERNEL_DEFS_H_
#define _KERNEL_DEFS_H_

/* Define some constants for virtual addresses */
#define KERN_STK 0xFF800000     /* Kernel stack */
#define KERN_IDT 0xFFFEF000     /* Kernel IDT */
#define KERN_IDT_LEN 0x7FF      /* 255 entries */
#define KERN_GDT 0xFFFEF800     /* kernel GDT */
#define KERN_SCR 0xFFFF0000     /* Screen (kernel virtual) memory  */
#define KERN_PGT 0xFFFF1000     /* kernel page table */

#define PHYS_INDEX_MAX 32768

#define NR_MODS 10              /* Establish support for modules loaded by
                                   grub at boot time */

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
