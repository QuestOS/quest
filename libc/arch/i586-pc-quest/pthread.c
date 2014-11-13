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

#include <pthread.h>
#include <stdint.h>
#include <sys/errno.h>

int
pthread_create (pthread_t * thread, pthread_attr_t * attr,
                void *(*start_routine) (void *), void *arg)
{
  int res = 0;
  pthread_attr_t tmp_attr;
  if (!attr) attr = &tmp_attr;
  attr->exit_func = _pthread_exit;
    
  res = syscall_create_thread (thread, attr, (uint32_t) start_routine, arg);
  return 0;  /* Success */
}

void
pthread_exit (void * value_ptr)
{
  syscall_thread_exit (value_ptr);
}

void
_pthread_exit ()
{
  syscall_thread_exit (NULL);
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
