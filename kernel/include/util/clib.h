/*                    The Quest Operating System
 *  Copyright (C) 2005-2015  Richard West, Boston University
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


#ifndef _CLIB_H_
#define _CLIB_H_

#ifndef va_arg

#ifndef _VALIST
#define _VALIST
typedef char *va_list;
#endif /* _VALIST */

/* Storage alignment properties */

#define  _AUPBND                (sizeof (sint32) - 1)
#define  _ADNBND                (sizeof (sint32) - 1)

/* Variable argument list macro definitions */

#define _Bnd(X, bnd)            (((sizeof (X)) + (bnd)) & (~(bnd)))
#define va_arg(ap, T)           (*(T *)(((ap) += (_Bnd (T, _AUPBND))) - (_Bnd (T,_ADNBND))))
#define va_end(ap)              (ap = (va_list) NULL)
#define va_start(ap, A)         (void) ((ap) = (((char *) &(A)) + (_Bnd (A,_AUPBND))))

#endif /* va_arg */

extern const uint8 _Ctypes[];

#define _XA     0x00    /* extra alphabetic - not supported */
#define _XS     0x40    /* extra space */
#define _BB     0x00    /* BEL, BS, etc. - not supported */
#define _CN     0x20    /* CR, FF, HT, NL, VT */
#define _DI     0x04    /* '0'-'9' */
#define _LO     0x02    /* 'a'-'z' */
#define _PU     0x10    /* punctuation */
#define _SP     0x08    /* space, tab, CR, LF, VT, FF */
#define _UP     0x01    /* 'A'-'Z' */
#define _XD     0x80    /* '0'-'9', 'A'-'F', 'a'-'f' */

#define isdigit(c)  (_Ctypes[(unsigned char)(c)] & (_DI))
#define isspace(c)  (_Ctypes[(unsigned char)(c)] & (_SP))
#define isxdigit(c) (_Ctypes[(unsigned char)(c)] & (_XD))
#define isupper(c)  (_Ctypes[(unsigned char)(c)] & (_UP))
#define islower(c)  (_Ctypes[(unsigned char)(c)] & (_LO))
#define isprint(c)  (_Ctypes[(unsigned char)(c)] & (_LO | _UP | _DI | _XS | _PU))
#define isalpha(c)  (_Ctypes[(unsigned char)(c)] & (_LO | _UP))

#endif //_CLIB_H_

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
