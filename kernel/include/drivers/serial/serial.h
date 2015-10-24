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

#include "drivers/serial/serial_reg.h"

#define BASE_BAUD         (1843200/16)
#define REAL_BAUD         (115200)
#define BAUD              (REAL_BAUD/24)
#define MMIO32_MEMBASE    (0x9000B000)

#ifndef __ASSEMBLER__

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

#define DIV_ROUND_CLOSEST(x, divisor)(                  \
{                                                       \
        typeof(x) __x = x;                              \
        typeof(divisor) __d = divisor;                  \
        (((typeof(x))-1) > 0 ||                         \
         ((typeof(divisor))-1) > 0 || (__x) > 0) ?      \
                (((__x) + ((__d) / 2)) / (__d)) :       \
                (((__x) - ((__d) / 2)) / (__d));        \
}                                                       \
)

extern void initialize_serial_mmio32 (void);
extern void mmio32_putc (char);
extern int mmio32_getc (void);
extern void * remap_serial_mmio32 (void);

extern void serial_putc (char);
extern void initialize_serial_port (void);
extern int serial_getc (void);

#endif

/* vi: set et sw=2 sts=2: */
