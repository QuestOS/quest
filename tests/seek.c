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

#define FILE_TO_OPEN "/boot/sample.txt"
#define BUFFER_SIZE 512

int main()
{
  FILE* f = fopen(FILE_TO_OPEN, "r");
  char buffer[512];
  
  if(!f) {
    printf("Failed to open file\n");
    return EXIT_FAILURE;
  }

  while(1) {
    int i;
    int bytes_read = fread(buffer, 1, BUFFER_SIZE, f);
    for(i = 0; i < bytes_read; ++i) putchar(buffer[i]);
    if(bytes_read != BUFFER_SIZE) break;
  }

  printf("Seeking to start of file\n");
  
  fseek(f, 0, SEEK_SET);


  while(1) {
    int i;
    int bytes_read = fread(buffer, 1, BUFFER_SIZE, f);
    for(i = 0; i < bytes_read; ++i) putchar(buffer[i]);
    if(bytes_read != BUFFER_SIZE) break;
  }

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
