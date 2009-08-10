/* -*- Mode: C -*- */

#ifndef _VIRTUAL_H_
#define _VIRTUAL_H_
#include "types.h"

extern void *map_virtual_page (uint32 phys_frame);
extern void unmap_virtual_page (void *virt_addr);
extern void *map_virtual_pages (uint32 * phys_frames, uint32 count);
extern void *map_contiguous_virtual_pages (uint32 phys_frame, uint32 count);
extern void unmap_virtual_pages (void *virt_addr, uint32 count);
extern void *get_phys_addr (void *virt_addr);

#endif

/* vi: set et sw=2 sts=2: */
