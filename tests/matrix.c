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
#include <stdio.h>
#include <stdlib.h>

#define M 3
#define K 2
#define N 3
#define NUM_THREADS 10

int A [M][K] = { {1,4}, {2,5}, {3,6} };
int B [K][N] = { {8,7,6}, {5,4,3} };
int C [M][N];

struct v {
   int i;  /* row */
   int j;  /* column */
};

void * worker (void *param); /* Worker thread */

int
main(int argc, char *argv[])
{
  int i, j, count = 0;
  pthread_t tid[M*N];

  for(i = 0; i < M; i++) {
    for(j = 0; j < N; j++) {
      /* Assign a row and column for each thread */
      struct v * data = (struct v *) malloc (sizeof (struct v));
      data->i = i;
      data->j = j;
      pthread_attr_t attr;
      pthread_create (&tid[count], &attr, worker, data);
      count++;
    }
  }

  count = 0;

  for(i = 0; i < M; i++) {
    for(j = 0; j < N; j++) {
      waitpid (tid[count]);
      count++;
    }
  }

  /* Print results */
  for(i = 0; i < M; i++) {
    for(j = 0; j < N; j++) {
      printf ("%d ", C[i][j]);
    }
    printf ("\n");
  }

  exit (0);
}

void *
worker (void * param)
{
   struct v * data = param;
   int n, sum = 0;

   /* Row multiplied by column */
   for(n = 0; n < K; n++){
      sum += A[data->i][n] * B[n][data->j];
   }

   C[data->i][data->j] = sum;
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
