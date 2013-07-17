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
#include <vshm.h>


int
main ()
{
  int i = 0;
  uint sandbox = socket_get_sb_id();
  vshm_circular_buffer_t circ_buf;
  printf("In %s sandbox %u\n", __FILE__, sandbox);
  if(sandbox == 0) {
    int res = mk_vshm_circular_buffer(&circ_buf, 1234, 10, sizeof(unsigned int), 0x2,
                                      VSHM_CREATE | VSHM_ALL_ACCESS);
    printf("%d: res = %d\n", sandbox, res);
    if(res < 0) {
      exit(EXIT_FAILURE);
    }
    for(i = 0; i < 1000; ++i) {
      printf("Inserting %d\n", i);
      while(vshm_circular_buffer_insert(&circ_buf, &i) < 0) ;
    }
    printf("Done with inserting\n");
    exit(EXIT_SUCCESS);
  }
  else if(sandbox == 1) {
    int res;
    usleep(4000000);
    res = mk_vshm_circular_buffer(&circ_buf, 1234, 10, sizeof(unsigned int), 0x1,
                                      VSHM_ALL_ACCESS);
    printf("%d: res = %d\n", sandbox, res);
    if(res < 0) {
      exit(EXIT_FAILURE);
    }
    for(i = 0; i < 1000; ++i) {
      int temp;
      while(vshm_circular_buffer_remove(&circ_buf, &temp) < 0) ;
      printf("Removed %d\n", temp);
    }
    exit(EXIT_SUCCESS);
  }
  else exit(EXIT_FAILURE);
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
