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

#include "boot/multiboot.h"

uint8_t reloc_pool[4096] __attribute__ ((align(0x8)));

static uint32_t cur_addr = 0;

static inline void *
memset (void *p, int ch, uint32 cb)
{

  asm volatile ("cld; rep stosb"
                :"=D" (p), "=a" (ch), "=c" (cb)
                :"0" (p), "1" (ch), "2" (cb)
                :"memory","flags");
  return p;
}

static inline void *
memcpy (void *pDest, const void *pSrc, uint32 cb)
{
  asm volatile ("cld; rep movsb"
                :"=c" (cb), "=D" (pDest), "=S" (pSrc)
                :"0" (cb), "1" (pDest), "2" (pSrc)
                :"memory","flags");
  return pDest;
}

static inline uint32
strlen (const char *s)
{
  uint32 i=0;
  while (*(s++)) i++;
  return i;
}

static inline void *
strcpy (char *dest, const char *src)
{
  return memcpy (dest, src, strlen (src) + 1);
}

multiboot *
reloc_mbi (multiboot * old_mbi)
{
  int i = 0;
  multiboot * new_mbi = (multiboot *) reloc_pool;
  cur_addr = (uint32_t) reloc_pool;
  memset (reloc_pool, 0, 4096);

  /* Copy the struct itself first */
  memcpy (new_mbi, old_mbi, sizeof (multiboot));

  /* Update and align cur_addr */
  cur_addr += sizeof (multiboot);
  cur_addr = ((cur_addr + 0x7) & (~((uint32_t) 0x7)));

  /* Copy cmdline */
  if (old_mbi->cmdline) {
    strcpy ((void *) cur_addr, old_mbi->cmdline);
    new_mbi->cmdline = (char *) cur_addr;
    cur_addr += (strlen (old_mbi->cmdline) + 1);
    cur_addr = ((cur_addr + 0x7) & (~((uint32_t) 0x7)));
  }

  /* Copy multiboot_module structs */
  if (old_mbi->mods_count) {
    memcpy ((void *) cur_addr, old_mbi->mods_addr,
            sizeof (multiboot_module) * old_mbi->mods_count);
    new_mbi->mods_addr = (multiboot_module *) cur_addr;
    cur_addr += (sizeof (multiboot_module) * old_mbi->mods_count);
    cur_addr = ((cur_addr + 0x7) & (~((uint32_t) 0x7)));
  }

  /* Copy module cmdlines */
  for (i = 0; i < old_mbi->mods_count; i++) {
    if (old_mbi->mods_addr[i].string) {
      strcpy ((void * ) cur_addr, old_mbi->mods_addr[i].string);
      new_mbi->mods_addr[i].string = (char *) cur_addr;
      cur_addr += (strlen (old_mbi->mods_addr[i].string) + 1);
      cur_addr = ((cur_addr + 0x7) & (~((uint32_t) 0x7)));
    }
  }

  /* Copy mmap structs */
  if (old_mbi->mmap_length) {
    memcpy ((void *) cur_addr, (void *) old_mbi->mmap_addr, old_mbi->mmap_length);
    new_mbi->mmap_addr = cur_addr;
    cur_addr += old_mbi->mmap_length;
    cur_addr = ((cur_addr + 0x7) & (~((uint32_t) 0x7)));
  }

  /* Copy drive structs */
  if (old_mbi->drives_length) {
    memcpy ((void *) cur_addr, old_mbi->drives_addr, old_mbi->drives_length);
    new_mbi->drives_addr = (multiboot_drive *) cur_addr;
    cur_addr += old_mbi->drives_length;
    cur_addr = ((cur_addr + 0x7) & (~((uint32_t) 0x7)));
  }

  /* Copy boot loader name */
  if (old_mbi->boot_loader_name) {
    strcpy ((void *) cur_addr, old_mbi->boot_loader_name);
    new_mbi->boot_loader_name = (char *) cur_addr;
    cur_addr += (strlen (old_mbi->boot_loader_name) + 1);
    cur_addr = ((cur_addr + 0x7) & (~((uint32_t) 0x7)));
  }

  return new_mbi;
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
