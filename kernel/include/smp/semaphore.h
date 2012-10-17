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

#ifndef _SEMAPHORE_H_
#define _SEMAPHORE_H_
#include "types.h"
#include "smp/spinlock.h"

struct _semaphore
{
  int s, max;
  spinlock lock;
  task_id waitqueue;
};
typedef struct _semaphore semaphore;

int semaphore_init (semaphore * sem, int max, int init);
int semaphore_signal (semaphore * sem, int s);
/* timeout: millisec, (-1) for indefinite */
int semaphore_wait (semaphore * sem, int s, s16 timeout);
int semaphore_destroy (semaphore * sem);

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
