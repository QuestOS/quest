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

static int round_down_division(float n, float d)
{
  return (int)(n / d);
}

static int complicated_float_function(float a, float b, float c, float d, float e)
{
  float temp, temp2;


  temp =  (a + b);
  temp2 = (d - e);
  return temp / c * temp2;
}

int
main ()
{
  printf("In float_test\n");
  int result;
  if(fork()) {
    while(1) {
      result = complicated_float_function(133.0f, 332.0f, 21.0f, 12.0f, 3.0f);
      
      if(result != 199) {
        printf("complicated float function = %d (Should be 199)\n",
               result);
        printf("FAILED\n");
        while(1);
      }
    }
  }
  else {
    while(1) {
      result = complicated_float_function(533.0f, 432.0f, 4321.0f, 312.0f, 4123.0f);
      
      if(result != -851) {
        printf("complicated float function = %d (Should be -851)\n",
               result);
        printf("FAILED\n");
        while(1);
      }
    }
  }
  while(1);
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
