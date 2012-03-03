/*                    The Quest Operating System
 *  Portions Copyright (C) 2005-2010  Richard West, Boston University
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

#include <arpa/inet.h>

uint32_t
htonl (uint32_t hostint32)
{
  return ((hostint32 & 0xff) << 24) |
    ((hostint32 & 0xff00) << 8) |
    ((hostint32 & 0xff0000UL) >> 8) |
    ((hostint32 & 0xff000000UL) >> 24);
}

uint16_t
htons (uint16_t hostint16)
{
  return ((hostint16 & 0xff) << 8) | ((hostint16 & 0xff00) >> 8);
}

uint32_t
ntohl (uint32_t netint32)
{
  return htonl(netint32);
}

uint16_t
ntohs (uint16_t netint16)
{
  return htons(netint16);
}

#define in_range(c, lo, up)  ((uint8_t)c >= lo && (uint8_t)c <= up)
#define isprint(c)           in_range(c, 0x20, 0x7f)
#define isdigit(c)           in_range(c, '0', '9')
#define isxdigit(c)          (isdigit(c) || in_range(c, 'a', 'f') || in_range(c, 'A', 'F'))
#define islower(c)           in_range(c, 'a', 'z')
#define isspace(c)           (c == ' ' || c == '\f' || c == '\n' || c == '\r' || c == '\t' || c == '\v')

int
inet_pton(int domain, const char *cp, void *addr)
{
  uint32_t val;
  uint8_t base;
  char c;
  uint32_t parts[4];
  uint32_t *pp = parts;

  c = *cp;
  for (;;) {
    /*
     * Collect number up to ``.''.
     * Values are specified as for C:
     * 0x=hex, 0=octal, 1-9=decimal.
     */
    if (!isdigit(c))
      return (0);
    val = 0;
    base = 10;
    if (c == '0') {
      c = *++cp;
      if (c == 'x' || c == 'X') {
        base = 16;
        c = *++cp;
      } else
        base = 8;
    }
    for (;;) {
      if (isdigit(c)) {
        val = (val * base) + (int)(c - '0');
        c = *++cp;
      } else if (base == 16 && isxdigit(c)) {
        val = (val << 4) | (int)(c + 10 - (islower(c) ? 'a' : 'A'));
        c = *++cp;
      } else
        break;
    }
    if (c == '.') {
      /*
       * Internet format:
       *  a.b.c.d
       *  a.b.c   (with c treated as 16 bits)
       *  a.b (with b treated as 24 bits)
       */
      if (pp >= parts + 3)
        return (0);
      *pp++ = val;
      c = *++cp;
    } else
      break;
  }
  /*
   * Check for trailing characters.
   */
  if (c != '\0' && !isspace(c))
    return (0);
  /*
   * Concoct the address according to
   * the number of parts specified.
   */
  switch (pp - parts + 1) {

  case 0:
    return (0);       /* initial nondigit */

  case 1:             /* a -- 32 bits */
    break;

  case 2:             /* a.b -- 8.24 bits */
    if (val > 0xffffffUL)
      return (0);
    val |= parts[0] << 24;
    break;

  case 3:             /* a.b.c -- 8.8.16 bits */
    if (val > 0xffff)
      return (0);
    val |= (parts[0] << 24) | (parts[1] << 16);
    break;

  case 4:             /* a.b.c.d -- 8.8.8.8 bits */
    if (val > 0xff)
      return (0);
    val |= (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8);
    break;
  }
  if (addr)
    *((uint32_t *) addr) = htonl(val);
  return (1);
}

const char *
inet_ntop(int domain, const void *addr, char *str, socklen_t size)
{
  uint32_t s_addr = *((uint32_t *) addr);
  char inv[3];
  char *rp;
  uint8_t *ap;
  uint8_t rem;
  uint8_t n;
  uint8_t i;

  rp = str;
  ap = (uint8_t *)&s_addr;
  for(n = 0; n < 4; n++) {
    i = 0;
    do {
      rem = *ap % (uint8_t)10;
      *ap /= (uint8_t)10;
      inv[i++] = '0' + rem;
    } while(*ap);
    while(i--)
      *rp++ = inv[i];
    *rp++ = '.';
    ap++;
  }
  *--rp = 0;
  return str;
}

int
inet_aton(const char *cp, struct in_addr *addr)
{
  uint32_t val;
  uint8_t base;
  char c;
  uint32_t parts[4];
  uint32_t *pp = parts;

  c = *cp;
  for (;;) {
    /*
     * Collect number up to ``.''.
     * Values are specified as for C:
     * 0x=hex, 0=octal, 1-9=decimal.
     */
    if (!isdigit(c))
      return (0);
    val = 0;
    base = 10;
    if (c == '0') {
      c = *++cp;
      if (c == 'x' || c == 'X') {
        base = 16;
        c = *++cp;
      } else
        base = 8;
    }
    for (;;) {
      if (isdigit(c)) {
        val = (val * base) + (int)(c - '0');
        c = *++cp;
      } else if (base == 16 && isxdigit(c)) {
        val = (val << 4) | (int)(c + 10 - (islower(c) ? 'a' : 'A'));
        c = *++cp;
      } else
        break;
    }
    if (c == '.') {
      /*
       * Internet format:
       *  a.b.c.d
       *  a.b.c   (with c treated as 16 bits)
       *  a.b (with b treated as 24 bits)
       */
      if (pp >= parts + 3)
        return (0);
      *pp++ = val;
      c = *++cp;
    } else
      break;
  }
  /*
   * Check for trailing characters.
   */
  if (c != '\0' && !isspace(c))
    return (0);
  /*
   * Concoct the address according to
   * the number of parts specified.
   */
  switch (pp - parts + 1) {

  case 0:
    return (0);       /* initial nondigit */

  case 1:             /* a -- 32 bits */
    break;

  case 2:             /* a.b -- 8.24 bits */
    if (val > 0xffffffUL)
      return (0);
    val |= parts[0] << 24;
    break;

  case 3:             /* a.b.c -- 8.8.16 bits */
    if (val > 0xffff)
      return (0);
    val |= (parts[0] << 24) | (parts[1] << 16);
    break;

  case 4:             /* a.b.c.d -- 8.8.8.8 bits */
    if (val > 0xff)
      return (0);
    val |= (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8);
    break;
  }
  if (addr)
    addr->s_addr = htonl(val);
  return (1);
}

char *
inet_ntoa(struct in_addr addr)
{
  static char str[16];
  uint32_t s_addr = addr.s_addr;
  char inv[3];
  char *rp;
  uint8_t *ap;
  uint8_t rem;
  uint8_t n;
  uint8_t i;

  rp = str;
  ap = (uint8_t *)&s_addr;
  for(n = 0; n < 4; n++) {
    i = 0;
    do {
      rem = *ap % (uint8_t)10;
      *ap /= (uint8_t)10;
      inv[i++] = '0' + rem;
    } while(*ap);
    while(i--)
      *rp++ = inv[i];
    *rp++ = '.';
    ap++;
  }
  *--rp = 0;
  return str;
}

uint32_t
inet_addr(const char *cp)
{
  struct in_addr val;

  if (inet_aton(cp, &val)) {
    return (val.s_addr);
  }
  return (INADDR_NONE);
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
