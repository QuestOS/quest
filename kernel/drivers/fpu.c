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


#include <arch/i386.h>
#include <kernel.h>
#include <util/printf.h>

#define DEBUG_FPU
#define DEBUG_FPU_VERBOSE

#ifdef DEBUG_FPU
#define DLOG(fmt,...) DLOG_PREFIX("FPU",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef DEBUG_FPU_VERBOSE
#define DLOGV(fmt,...) DLOG_PREFIX("FPU",fmt,##__VA_ARGS__)
#else
#define DLOGV(fmt,...) ;
#endif

bool fpu_init()
{
  DLOGV("In FPU");

  uint32 cr = get_cr0();

  DLOGV("Before cr = 0x%X", cr);
  set_cr0(cr | 1 << 2);
  cr = get_cr0();
  DLOGV("After cr = 0x%X", cr);
  //while(1);
  return TRUE;
}


#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = fpu_init
};

DEF_MODULE (fpu, "FPU driver", &mod_ops, {});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
