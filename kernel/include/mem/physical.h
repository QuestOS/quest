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

#ifndef _PHYSICAL_H_
#define _PHYSICAL_H_
#include "types.h"

extern uint32 mm_table[];       /* Bitmap for free/mapped physical pages */
extern uint32 mm_limit;         /* Actual or sandbox high physical page limit */
extern uint32 mm_begin;         /* Actual or sandbox low physical page limit */
extern uint32 alloc_phys_frame (void);
extern uint32 alloc_phys_frame_high (void);
extern uint32 alloc_phys_frames (uint32);
extern void free_phys_frame (uint32);
extern void free_phys_frames (uint32, uint32);

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
