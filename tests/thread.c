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

void *PrintHello(void *threadid)
{
   int tid;
   tid = (int)threadid;
   printf("Hello World! It's me, thread #%d!\n", tid);
   for (;;) {
     printf ("Child thread!\n");
     usleep (2500000);
   }
   pthread_exit(NULL);
}

int main (int argc, char *argv[])
{
   pthread_t thread;
   int res = 0, t = 1;
   printf("In main: creating thread %d\n", t);
   res = pthread_create(&thread, NULL, PrintHello, (void *)t);
   if (res){
      printf("ERROR: pthread_create() returned %d\n", res);
      exit(-1);
   }

   for (;;) {
     printf ("Main thread!\n");
     usleep (2000000);
   }
   /* Last thing that main() should do */
   pthread_exit(NULL);
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
