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

#include "kernel.h"
#include "smp/spinlock.h"
#include "sched/sched.h"

int
semaphore_init (semaphore * sem, int max, int init)
{
  sem->s = init;
  sem->max = max;
  sem->waitqueue = 0;
  spinlock_init (&sem->lock);
  return 0;
}

int
semaphore_signal (semaphore * sem, int s)
{
  int status = 0;
  spinlock_lock (&sem->lock);
  sem->s += s;
  if (sem->s > sem->max) {
    sem->s = sem->max;
    status = -1;
  }
  /* wake up waiters */
  wakeup_queue (&sem->waitqueue);
  spinlock_unlock (&sem->lock);
  return status;
}

/* timeout: millisec, (-1) for indefinite */
int
semaphore_wait (semaphore * sem, int s, s16 timeout)
{
  for (;;) {
    spinlock_lock (&sem->lock);
    if (sem->s >= s) {
      sem->s -= s;
      spinlock_unlock (&sem->lock);
      return 0;
    } else {
      queue_append (&sem->waitqueue, str ());
      spinlock_unlock (&sem->lock);
      schedule ();
    }
  }
}

int
semaphore_destroy (semaphore * sem)
{
  return 0;
}

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
