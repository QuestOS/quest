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

#ifndef _STDIO_H_
#define _STDIO_H_

#include "syscall.h"

#ifndef EOF
#define EOF -1
#endif

#ifndef NULL
#define NULL 0
#endif 

#ifndef _SIZE_T
typedef int size_t;
#define _SIZE_T 1
#endif

typedef struct _file {
  int fd;

} FILE;

extern FILE *fopen( const char *filename, const char *mode );
extern int printf( const char *format, ... );
extern int fprintf( FILE *stream, const char *format, ... );
extern int sprintf( char *str, const char *format, ... );

extern void itoa( char *buf, int base, int d );

extern int fclose( FILE *stream );

extern size_t fread(void *ptr, size_t size, size_t nmemb, FILE *stream);
extern size_t   fwrite(const  void  *ptr,  size_t  size,  size_t  nmemb,  FILE *stream);

extern int puts( const char *s );


#endif

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
