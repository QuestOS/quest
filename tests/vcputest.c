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
#include <vcpu.h>


int main(int argc, char* argv[])
{
  struct sched_param s_params = {.type = MAIN_VCPU, .C = 20, .T = 100};
  struct sched_param s_params2;
  memset(&s_params2, 0, sizeof(s_params2));
  int new_vcpu = vcpu_create(&s_params);

  if(new_vcpu < 0) {
    printf("Failed to create vcpu\n");
    exit(1);
  }

  vcpu_getparams(&s_params2);
  printf("Before vcpu bind task: C = %d, T = %d\n", s_params2.C, s_params2.T);
  vcpu_bind_task(new_vcpu);
  usleep(1000000);
  usleep(1000000);
  usleep(1000000);
  usleep(1000000);
  vcpu_getparams(&s_params2);
  printf("After vcpu bind task: C = %d, T = %d\n", s_params2.C, s_params2.T);
  
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
