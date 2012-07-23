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

#ifndef __I386_MTRR_H__
#define __I386_MTRR_H__
#include"types.h"

#define IA32_MTRRCAP 0xFE
#define IA32_MTRR_DEF_TYPE 0x2FF
#define IA32_MTRR_PHYS_BASE(n) ((0x200) + (n << 1))
#define IA32_MTRR_PHYS_MASK(n) ((0x201) + (n << 1))

#define IA32_MTRR_FIX64K_00000  0x250
#define IA32_MTRR_FIX16K_80000  0x258
#define IA32_MTRR_FIX16K_A0000  0x259
#define IA32_MTRR_FIX4K_C0000   0x268
#define IA32_MTRR_FIX4K_C8000   0x269
#define IA32_MTRR_FIX4K_D0000   0x26A
#define IA32_MTRR_FIX4K_D8000   0x26B
#define IA32_MTRR_FIX4K_E0000   0x26C
#define IA32_MTRR_FIX4K_E8000   0x26D
#define IA32_MTRR_FIX4K_F0000   0x26E
#define IA32_MTRR_FIX4K_F8000   0x26F

#define MAX_MTRR_VAR_REGS 256

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
