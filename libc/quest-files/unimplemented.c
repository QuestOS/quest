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

/* This is the place for all unimplemented functions that are in libc.
   Such functions typically exist because we are porting a program and
   do not have the time to implement all the functions that are not
   used.  If such functions exist instead of being defined locally
   within the program if they belong in libc they should be replaced
   here for three reasons (at least).  (1) eventually the function
   might be implemented and this will create conflicts when the
   program compiled, (2) if we port another program that has the same
   functions it will save the time of trying to get that program to
   compile (of course not much time of those functions are used in the
   new program) and (3) if we want to know which functions are not
   implemented they can all easily be found here.  Unimplemented
   functions should use the macro defined below that will print the
   function name and then exit.  The macro provides the entire
   function body including the opening and closing brackets of the
   function */

#include <unistd.h>
#include <stdlib.h>

#define unimplemented_funcion						\
  {									\
    printf("\n\nExiting because %s in %s is unimplemented\n",           \
           __FUNCTION__, __FILE__);                                     \
    exit(EXIT_FAILURE);                                                 \
   }

int gethostname(char *name, size_t len) unimplemented_funcion



/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
