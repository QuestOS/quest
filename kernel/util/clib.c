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

/* Adopted from ACPICA*/

#include <types.h>
#include <util/clib.h>
#include <kernel.h>

const uint8 _Ctypes[257] = {
    _CN,            /* 0x00     0 NUL */
    _CN,            /* 0x01     1 SOH */
    _CN,            /* 0x02     2 STX */
    _CN,            /* 0x03     3 ETX */
    _CN,            /* 0x04     4 EOT */
    _CN,            /* 0x05     5 ENQ */
    _CN,            /* 0x06     6 ACK */
    _CN,            /* 0x07     7 BEL */
    _CN,            /* 0x08     8 BS  */
    _CN|_SP,   /* 0x09     9 TAB */
    _CN|_SP,   /* 0x0A    10 LF  */
    _CN|_SP,   /* 0x0B    11 VT  */
    _CN|_SP,   /* 0x0C    12 FF  */
    _CN|_SP,   /* 0x0D    13 CR  */
    _CN,            /* 0x0E    14 SO  */
    _CN,            /* 0x0F    15 SI  */
    _CN,            /* 0x10    16 DLE */
    _CN,            /* 0x11    17 DC1 */
    _CN,            /* 0x12    18 DC2 */
    _CN,            /* 0x13    19 DC3 */
    _CN,            /* 0x14    20 DC4 */
    _CN,            /* 0x15    21 NAK */
    _CN,            /* 0x16    22 SYN */
    _CN,            /* 0x17    23 ETB */
    _CN,            /* 0x18    24 CAN */
    _CN,            /* 0x19    25 EM  */
    _CN,            /* 0x1A    26 SUB */
    _CN,            /* 0x1B    27 ESC */
    _CN,            /* 0x1C    28 FS  */
    _CN,            /* 0x1D    29 GS  */
    _CN,            /* 0x1E    30 RS  */
    _CN,            /* 0x1F    31 US  */
    _XS|_SP,   /* 0x20    32 ' ' */
    _PU,            /* 0x21    33 '!' */
    _PU,            /* 0x22    34 '"' */
    _PU,            /* 0x23    35 '#' */
    _PU,            /* 0x24    36 '$' */
    _PU,            /* 0x25    37 '%' */
    _PU,            /* 0x26    38 '&' */
    _PU,            /* 0x27    39 ''' */
    _PU,            /* 0x28    40 '(' */
    _PU,            /* 0x29    41 ')' */
    _PU,            /* 0x2A    42 '*' */
    _PU,            /* 0x2B    43 '+' */
    _PU,            /* 0x2C    44 ',' */
    _PU,            /* 0x2D    45 '-' */
    _PU,            /* 0x2E    46 '.' */
    _PU,            /* 0x2F    47 '/' */
    _XD|_DI,   /* 0x30    48 '0' */
    _XD|_DI,   /* 0x31    49 '1' */
    _XD|_DI,   /* 0x32    50 '2' */
    _XD|_DI,   /* 0x33    51 '3' */
    _XD|_DI,   /* 0x34    52 '4' */
    _XD|_DI,   /* 0x35    53 '5' */
    _XD|_DI,   /* 0x36    54 '6' */
    _XD|_DI,   /* 0x37    55 '7' */
    _XD|_DI,   /* 0x38    56 '8' */
    _XD|_DI,   /* 0x39    57 '9' */
    _PU,            /* 0x3A    58 ':' */
    _PU,            /* 0x3B    59 ';' */
    _PU,            /* 0x3C    60 '<' */
    _PU,            /* 0x3D    61 '=' */
    _PU,            /* 0x3E    62 '>' */
    _PU,            /* 0x3F    63 '?' */
    _PU,            /* 0x40    64 '@' */
    _XD|_UP,   /* 0x41    65 'A' */
    _XD|_UP,   /* 0x42    66 'B' */
    _XD|_UP,   /* 0x43    67 'C' */
    _XD|_UP,   /* 0x44    68 'D' */
    _XD|_UP,   /* 0x45    69 'E' */
    _XD|_UP,   /* 0x46    70 'F' */
    _UP,            /* 0x47    71 'G' */
    _UP,            /* 0x48    72 'H' */
    _UP,            /* 0x49    73 'I' */
    _UP,            /* 0x4A    74 'J' */
    _UP,            /* 0x4B    75 'K' */
    _UP,            /* 0x4C    76 'L' */
    _UP,            /* 0x4D    77 'M' */
    _UP,            /* 0x4E    78 'N' */
    _UP,            /* 0x4F    79 'O' */
    _UP,            /* 0x50    80 'P' */
    _UP,            /* 0x51    81 'Q' */
    _UP,            /* 0x52    82 'R' */
    _UP,            /* 0x53    83 'S' */
    _UP,            /* 0x54    84 'T' */
    _UP,            /* 0x55    85 'U' */
    _UP,            /* 0x56    86 'V' */
    _UP,            /* 0x57    87 'W' */
    _UP,            /* 0x58    88 'X' */
    _UP,            /* 0x59    89 'Y' */
    _UP,            /* 0x5A    90 'Z' */
    _PU,            /* 0x5B    91 '[' */
    _PU,            /* 0x5C    92 '\' */
    _PU,            /* 0x5D    93 ']' */
    _PU,            /* 0x5E    94 '^' */
    _PU,            /* 0x5F    95 '_' */
    _PU,            /* 0x60    96 '`' */
    _XD|_LO,   /* 0x61    97 'a' */
    _XD|_LO,   /* 0x62    98 'b' */
    _XD|_LO,   /* 0x63    99 'c' */
    _XD|_LO,   /* 0x64   100 'd' */
    _XD|_LO,   /* 0x65   101 'e' */
    _XD|_LO,   /* 0x66   102 'f' */
    _LO,            /* 0x67   103 'g' */
    _LO,            /* 0x68   104 'h' */
    _LO,            /* 0x69   105 'i' */
    _LO,            /* 0x6A   106 'j' */
    _LO,            /* 0x6B   107 'k' */
    _LO,            /* 0x6C   108 'l' */
    _LO,            /* 0x6D   109 'm' */
    _LO,            /* 0x6E   110 'n' */
    _LO,            /* 0x6F   111 'o' */
    _LO,            /* 0x70   112 'p' */
    _LO,            /* 0x71   113 'q' */
    _LO,            /* 0x72   114 'r' */
    _LO,            /* 0x73   115 's' */
    _LO,            /* 0x74   116 't' */
    _LO,            /* 0x75   117 'u' */
    _LO,            /* 0x76   118 'v' */
    _LO,            /* 0x77   119 'w' */
    _LO,            /* 0x78   120 'x' */
    _LO,            /* 0x79   121 'y' */
    _LO,            /* 0x7A   122 'z' */
    _PU,            /* 0x7B   123 '{' */
    _PU,            /* 0x7C   124 '|' */
    _PU,            /* 0x7D   125 '}' */
    _PU,            /* 0x7E   126 '~' */
    _CN,            /* 0x7F   127 DEL */

    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  /* 0x80 to 0x8F    */
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  /* 0x90 to 0x9F    */
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  /* 0xA0 to 0xAF    */
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  /* 0xB0 to 0xBF    */
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  /* 0xC0 to 0xCF    */
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  /* 0xD0 to 0xDF    */
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  /* 0xE0 to 0xEF    */
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  /* 0xF0 to 0xFF    */
    0                                 /* 0x100 */
};

char *
strcat (char *DstString, const char *SrcString)
{
    char *String;

    /* Find end of the destination string */
    for (String = DstString; *String++; ) { ; }

    /* Concatenate the string */ 
    for (--String; (*String++ = *SrcString++); ) { ; }

    return (DstString);
}

int
toupper (int c)
{
  return (islower(c) ? ((c)-0x20) : (c));
}

int
tolower (int c)
{
  return (isupper(c) ? ((c)+0x20) : (c));
}

/*******************************************************************************
 *
 * FUNCTION:    strtoul
 *
 * PARAMETERS:  String          - Null terminated string
 *              Terminater      - Where a pointer to the terminating byte is
 *                                returned
 *              Base            - Radix of the string
 *
 * RETURN:      Converted value
 *
 * DESCRIPTION: Convert a string into a 32-bit unsigned value.
 *              Note: use strtoul64 for 64-bit integers.
 *
 ******************************************************************************/

#define SIGN_POSITIVE              0
#define SIGN_NEGATIVE              1
#define UINT32_MAX                 (uint32)(~((uint32) 0))

uint32
strtoul (const char *String, char **Terminator, uint32 Base)
{
    uint32                  converted = 0;
    uint32                  index;
    uint32                  sign;
    const char              *StringStart;
    uint32                  ReturnValue = 0;
    uint32                  Status = 0;

    /*
     * Save the value of the pointer to the buffer's first
     * character, save the current errno value, and then
     * skip over any white space in the buffer:
     */
    StringStart = String;
    while (isspace (*String) || *String == '\t')
    {
        ++String;
    }

    /*
     * The buffer may contain an optional plus or minus sign.
     * If it does, then skip over it but remember what is was:
     */
    if (*String == '-')
    {
        sign = SIGN_NEGATIVE;
        ++String;
    }
    else if (*String == '+')
    {
        ++String;
        sign = SIGN_POSITIVE;
    }
    else
    {
        sign = SIGN_POSITIVE;
    }

    /*
     * If the input parameter Base is zero, then we need to
     * determine if it is octal, decimal, or hexadecimal:
     */
    if (Base == 0)
    {
        if (*String == '0')
        {
            if (tolower (*(++String)) == 'x')
            {
                Base = 16;
                ++String;
            }
            else
            {
                Base = 8;
            }
        }
        else
        {
            Base = 10;
        }
    }
    else if (Base < 2 || Base > 36)
    {
        /*
         * The specified Base parameter is not in the domain of
         * this function:
         */
        goto done;
    }

    /*
     * For octal and hexadecimal bases, skip over the leading
     * 0 or 0x, if they are present.
     */
    if (Base == 8 && *String == '0')
    {
        String++;
    }

    if (Base == 16 &&
        *String == '0' &&
        tolower (*(++String)) == 'x')
    {
        String++;
    }

    /*
     * Main loop: convert the string to an unsigned long:
     */
    while (*String)
    {
        if (isdigit (*String))
        {
            index = (uint32) ((uint8) *String - '0');
        }
        else
        {
            index = (uint32) toupper (*String);
            if (isupper (index))
            {
                index = index - 'A' + 10;
            }
            else
            {
                goto done;
            }
        }

        if (index >= Base)
        {
            goto done;
        }

        /*
         * Check to see if value is out of range:
         */

        if (ReturnValue > ((UINT32_MAX - (uint32) index) /
                            (uint32) Base))
        {
            Status = 1;
            ReturnValue = 0;           /* reset */
        }
        else
        {
            ReturnValue *= Base;
            ReturnValue += index;
            converted = 1;
        }

        ++String;
    }

done:
    /*
     * If appropriate, update the caller's pointer to the next
     * unconverted character in the buffer.
     */
    if (Terminator)
    {
        if (converted == 0 && ReturnValue == 0 && String != NULL)
        {
            *Terminator = (char *) StringStart;
        }
        else
        {
            *Terminator = (char *) String;
        }
    }

    if (Status == 1)
    {
        ReturnValue = UINT32_MAX;
    }

    /*
     * If a minus sign was present, then "the conversion is negated":
     */
    if (sign == SIGN_NEGATIVE)
    {
        ReturnValue = (UINT32_MAX - ReturnValue) + 1;
    }

    return (ReturnValue);
}

char *
strncpy (char *s1, const char *s2, int length)
{

  while ((length--) && (*s1++ = *s2++));

  if (length < 0)
    *(s1 - 1) = '\0';

  return s1;
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
