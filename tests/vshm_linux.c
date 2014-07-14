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
  uint sandbox = socket_get_sb_id();
  printf("In %s sandbox %u\n", __FILE__, sandbox);
  if (sandbox == 0) {
    unsigned int * addr;
    int res = vshm_map(1234, 0x1000, 0x1 << 0x3, VSHM_CREATE | VSHM_ALL_ACCESS, (void**) &addr);
    printf("%d: res = %d, addr = %p\n", sandbox, res, addr);
    while(1) {
      if((*addr) == 0xCAFEBABE) {
        printf("Got message from Linux!\n");
        break;
      }
    }
    exit (EXIT_SUCCESS);
  }

  exit (EXIT_FAILURE);
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
