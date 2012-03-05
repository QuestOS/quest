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

#ifndef _SIZE_T
typedef int size_t;
#define _SIZE_T 1
#endif

extern int strcmp ( const char *s1, const char *s2 );
extern void *memset( void *s, int c, size_t n );
extern void *memcpy( void *dest, const void *src, size_t n );
extern size_t strlen( const char *s );
extern int memcmp( const void *s1, const void *s2, size_t n );
extern char *strcpy(char *dest, const char *src);
extern char *strncpy( char *s1, const char *s2, int length );
extern char *strdup(const char *s);
extern char *strchr(const char *s, int c);
extern char *strstr(const char *s1, const char *s2);




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
