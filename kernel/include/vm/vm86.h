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

#ifndef _VM86_H_
#define _VM86_H_

#include "types.h"
#include "vm/vm86-defs.h"
#include "vm/vmx.h"

#define REAL_TO_LIN32(s,o,type)                                         \
  ((type *) ((((uint32) (s) & 0xFFFF) << 4) + ((uint32) (o) & 0xFFFF)))

struct _vm86_farptr
{
  uint16 offs;
  uint16 segm;
} PACKED;
typedef struct _vm86_farptr vm86_farptr;
#define FP_TO_LIN32(fp,type) (REAL_TO_LIN32 ((fp).segm, (fp).offs, type))
#define LIN32_TO_FP(p)                                                  \
  ((vm86_farptr) { .segm = ((uint32) (p) & (~0xFFFF)) >> 4, .offs = (uint32) (p) & 0xFFFF })

sint32 vmx_vm86_handle_GPF (virtual_machine *);
void vmx_vm86_global_init (void);

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
