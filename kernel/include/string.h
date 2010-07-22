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

#ifndef _STRING_H_
#define _STRING_H_

#include "arch/i386.h"

/* quick naive version */
static inline int
strncmp (char *s1, char *s2, int n)
{
  int i, ret=0;
  for (i=0;i<n;i++) {
    if (s1[i] == 0 || s2[i] == 0)
      break;
    if (s1[i] != s2[i]) ret++;
  }
  if (s1[i] != s2[i]) ret++;
  return ret;
}

#endif
