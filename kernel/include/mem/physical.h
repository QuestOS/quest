/* -*- Mode: C; c-file-style: "gnu"; c-basic-offset: 2; indent-tabs-mode: nil -*- */

#ifndef _PHYSICAL_H_
#define _PHYSICAL_H_
#include "types.h"

extern uint32 mm_table[];       /* Bitmap for free/mapped physical pages */
extern uint32 mm_limit;         /* Actual physical page limit */
extern uint32 alloc_phys_frame (void);
extern uint32 alloc_phys_frames (uint32);
extern void free_phys_frame (uint32);
extern void free_phys_frames (uint32, uint32);

#endif

/* vi: set et sw=2 sts=2: */
