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

#ifndef _VSHM_H_
#define _VSHM_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef int pthread_t;
typedef struct _pthread_attr_t {
  /* Exit function (_pthread_exit) */
  void (* exit_func) (void);
} pthread_attr_t;

int pthread_create (pthread_t * thread, pthread_attr_t * attr,
                    void *(*start_routine) (void *), void *arg);

void pthread_exit (void * value_ptr);
void _pthread_exit (void);

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
