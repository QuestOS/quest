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

#ifndef __TYPES_H__
#define __TYPES_H__

#ifndef INT_MAX
#define INT_MAX 0xFFFFFFFF
#endif

#ifndef __ASSEMBLER__

#define TRUE 1
#define FALSE 0
#define PRIVATE static
#define PACKED __attribute__ ((packed))
#define WARN_UNUSED_RESULT __attribute__((warn_unused_result))
#define SQUELCH_UNUSED __attribute__((unused))
#define ALIGNED(x) __attribute__((aligned (x)))

typedef unsigned char uint8;
typedef unsigned short int uint16;
typedef unsigned long int uint32;
typedef unsigned long long int uint64;

typedef signed char sint8, s8;
typedef signed short int sint16, s16;
typedef signed long int sint32, s32;
typedef signed long long int sint64, s64;

#ifndef _SIZE_T
typedef int size_t;
#define _SIZE_T 1
#endif

typedef signed char bool;

typedef unsigned long uint;
typedef signed long sint;

#ifndef _STDINT_
#define _STDINT_
typedef uint8 uint8_t;
typedef uint16 uint16_t;
typedef uint32 uint32_t;
typedef uint64 uint64_t;
#endif

typedef uint8 u8;
typedef uint16 u16, __le16;
typedef uint32 u32, __le32, __be32, dma_addr_t, frame_t, framenum_t, phys_addr_t;
typedef uint64 u64, __le64;
typedef void *addr_t;

typedef int gfp_t;
typedef uint32 task_id;

#endif // [#ifndef __ASSEMBLER__]

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
