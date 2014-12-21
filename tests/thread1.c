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

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>

#define THREAD_COUNT  3

void *
PrintHello (void *threadid)
{
  int tid;
  tid = (int)threadid;
  usleep (3000000);
  printf ("Hello World! It's me, thread #%d!\n", tid);
  printf ("Now, pthread_exit\n");
  pthread_exit (NULL);
}

int main (int argc, char *argv[])
{
  pthread_t thread[THREAD_COUNT];
  int res = 0, t = 0;

  for (t = 0; t < THREAD_COUNT; t++) {
    res = pthread_create (&thread[t], NULL, PrintHello, (void *) t);
    printf ("In main: creating thread 0x%X\n", thread[t]);
    if (res) {
      printf ("ERROR: pthread_create() returned %d\n", res);
      exit (-1);
    }
  }

  for (t = 0; t < THREAD_COUNT; t++) {
    res = waitpid (thread[t]);
    printf ("Return from waitpid: %d\n", res);
  }

  /* Last thing that main() should do */
  printf ("pthread_exit in main!\n");
  pthread_exit (NULL);
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
