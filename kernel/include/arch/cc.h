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

/*
 * Portions Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/*                    The Quest Operating System
 *  Portions Copyright (C) 2005-2010  Richard West, Boston University
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

/* Ported to Quest by: Matthew Danish (md) */

#ifndef __ARCH_CC_H__
#define __ARCH_CC_H__

#include "types.h"
#include "lwipopts.h"

/* Include some files for defining library routines */
//#include <linux/string.h>

/* Define platform endianness */
#ifndef BYTE_ORDER
#define BYTE_ORDER LITTLE_ENDIAN
#endif /* BYTE_ORDER */

/* Define generic types used in lwIP */
typedef uint8  u8_t;
typedef sint8  s8_t;
typedef uint16 u16_t;
typedef sint16 s16_t;
typedef uint32 u32_t;
typedef sint32 s32_t;

typedef u32_t mem_ptr_t;

/* Define (sn)printf formatters for these lwIP types */
#define U16_F "hu"
#define S16_F "hd"
#define X16_F "hx"
#define U32_F "u"
#define S32_F "d"
#define X32_F "x"

/* Compiler hints for packing structures */
#if 1 /* GCC */
#define PACK_STRUCT_FIELD(x) x __attribute__((packed))
#define PACK_STRUCT_STRUCT __attribute__((packed))
#else
#define PACK_STRUCT_FIELD(x) x
#define PACK_STRUCT_STRUCT
#endif
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_END

void printf(const char *fmt, ...);
void com1_printf(const char *fmt, ...);

/* Plaform specific diagnostic output  */
/* Plaform specific diagnostic output */
#define LWIP_PLATFORM_DIAG(x)   do {com1_printf x;} while(0)

#define LWIP_PLATFORM_ASSERT(x) do {com1_printf("Assertion \"%s\" failed at line %d in %s\n", \
                                     x, __LINE__, __FILE__);} while(0)

#define PERF_START   /* null definition */
#define PERF_STOP(x) /* null definition */

#endif /* __ARCH_CC_H__ */
