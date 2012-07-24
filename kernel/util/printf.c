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

#include "types.h"
#include "kernel.h"
#include "acpi.h"                /* Use ACPICA's va_* macros */
#include "util/screen.h"
#include "lwip/ip.h"
#include "lwip/netif.h"
#include "lwip/udp.h"
#include "util/debug.h"

#ifdef USE_VMX
#include "vm/shm.h"
#endif

static uint32 base10_u32_divisors[10] = {
  1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1
};

#if 0
static uint64 base10_u64_divisors[20] = {
  10000000000000000000ULL, 1000000000000000000ULL, 100000000000000000ULL,
  10000000000000000ULL, 1000000000000000ULL, 100000000000000ULL,
  10000000000000ULL, 1000000000000ULL, 100000000000ULL, 10000000000ULL,
  1000000000ULL, 100000000ULL, 10000000ULL, 1000000ULL, 100000ULL, 10000ULL,
  1000ULL, 100ULL, 10ULL, 1ULL
};
#endif

void
closure_vprintf (void putc_clo (void *, char), void *data, const char *fmt,
                 va_list args)
{
  int precision, width, mode, upper, ells;
  char padding;
#define putc(c) putc_clo(data,c)
  while (*fmt) {
    /* handle ordinary characters and directives */
    switch (*fmt) {
    case '\0':
      return;
    case '%':
      fmt++;
      precision = 0;
      width = 0;
      upper = 1;
      padding = ' ';
      ells = 0;
#define PRINTF_MODE_PRECISION 1
      mode = 0;
      /* handle directive arguments */
      while (*fmt) {
        switch (*fmt) {
        case 'p':{
            /* pointer value */
            uint32 x = va_arg (args, uint32);
            int i, li;

            for (i = 0; i < 8; i++) {
              if ((li = (x >> ((7 - i) << 2)) & 0x0F) > 9)
                putc ('A' + li - 0x0A);
              else
                putc ('0' + li);
            }
            goto directive_finished;
          }
        case 'x':
          upper = 0;
        case 'X':{
            /* hexadecimal output */
            uint64 x;
            int i, li, print_padding = 0, print_digits = 0;
            int w = (ells == 2 ? 16 : 8);

            if (ells == 2)
              x = va_arg (args, uint64);
            else
              x = va_arg (args, uint32);

            for (i = 0; i < w; i++) {
              li = (x >> (((w - 1) - i) << 2)) & 0x0F;

#define HANDLE_OPTIONS(q,max_w,end_i)                                   \
            if (q != 0 || i == end_i)                                   \
              print_digits = 1;                                         \
            if (q == 0 && !print_digits && i >= (max_w - width))        \
              print_padding = 1;                                        \
            if (q == 0 && !print_digits && i >= (max_w - precision))    \
              print_digits = 1;                                         \
            if (q == 0 && print_padding && !print_digits)               \
              putc(padding);

              HANDLE_OPTIONS (li, w, (w-1));

              if (print_digits) {
                if (li > 9)
                  putc ((upper ? 'A' : 'a') + li - 0x0A);
                else
                  putc ('0' + li);
              }
            }

            goto directive_finished;
          }
        case 'u':{
            /* decimal output */
            uint32 x = va_arg (args, uint32);
            int i, q, print_padding = 0, print_digits = 0;
            uint32 *divisors = base10_u32_divisors;

            for (i = 0; i < 10; i++) {
              q = x / divisors[i];
              x %= divisors[i];

              HANDLE_OPTIONS (q, 10, 9);

              if (print_digits)
                putc ('0' + q);
            }

            goto directive_finished;
          }
        case 'd':{
            /* decimal output */
            signed long x = va_arg (args, signed long);
            int i, q, print_padding = 0, print_digits = 0;
            uint32 *divisors = base10_u32_divisors;

            if (x < 0) {
              putc ('-');
              x *= -1;
            }
            for (i = 0; i < 10; i++) {
              q = x / divisors[i];
              x %= divisors[i];

              HANDLE_OPTIONS (q, 10, 9);

              if (print_digits)
                putc ('0' + q);
            }

            goto directive_finished;
          }
        case 's':{
            /* string argument */
            char *s = va_arg (args, char *);
            if (s) {
              if (precision > 0)
                while (*s && precision-- > 0)
                  putc (*s++);
              else
                while (*s)
                  putc (*s++);
            } else {
              putc ('('); putc ('n'); putc ('u'); putc ('l'); putc ('l'); putc (')');
            }
            goto directive_finished;
          }
        case 'c':{
            /* character argument */
            char c = (char) va_arg (args, int);
            /* char is promoted to int when passed through va_arg */
            putc (c);
            goto directive_finished;
          }
        case '%':{
            /* single % */
            putc ('%');
            goto directive_finished;
          }
        case 'l':
          /* "long" annotation */
          ells++;
          break;
        case '.':
          mode = PRINTF_MODE_PRECISION;
          break;
        default:
          if ('0' <= *fmt && *fmt <= '9') {
            if (mode == PRINTF_MODE_PRECISION) {
              /* precision specifier */
              precision *= 10;
              precision += *fmt - '0';
            } else if (mode == 0 && width == 0 && *fmt == '0') {
              /* padding char is zero */
              padding = '0';
            } else {
              /* field width */
              width *= 10;
              width += *fmt - '0';
            }
          }
          break;
        }
        fmt++;
      }
    directive_finished:
      break;
    default:
      /* regular character */
      putc (*fmt);
      break;
    }
    fmt++;
  }
#undef putc
}

static void
putc_fun (void *f, char c)
{
  void (*g) (char) = f;
  g (c);
}

void
fun_vprintf (void putc (char), const char *fmt, va_list args)
{
#ifdef USE_VMX
  if (shm_initialized) spinlock_lock (&(shm->logger_lock));
#endif

  closure_vprintf (putc_fun, (void *) putc, fmt, args);

#ifdef USE_VMX
  if (shm_initialized) spinlock_unlock (&(shm->logger_lock));
#endif
}

void
fun_printf (void putc (char), const char *fmt, ...)
{
  va_list args;
  va_start (args, fmt);
  fun_vprintf (putc, fmt, args);
  va_end (args);
}

void
com1_printf (const char *fmt, ...)
{
  va_list args;
  va_start (args, fmt);
  fun_vprintf (com1_putc, fmt, args);
  va_end (args);
}

/* Feature to send logging output over UDP to a server */
//#define UDP_LOGGING

#define UDP_LOGGING_PORT 4444
#define UDP_LOGGING_NET_IF "en0"
#define UDP_LOGGING_BUF_SIZE 256

#ifdef UDP_LOGGING
static struct netif *logging_if = NULL;
static struct udp_pcb *logging_pcb = NULL;
static char logging_buf[UDP_LOGGING_BUF_SIZE];
static int logging_idx = 0;
static void
udp_logging_putc (char c)
{
  if (logging_idx < UDP_LOGGING_BUF_SIZE)
    logging_buf[logging_idx++] = c;
}

static int
send (uint8 *buf, uint32 len)
{
  struct pbuf *p;
  struct ip_addr server_ip;

  /* assume gateway has logging server */
  server_ip.addr = logging_if->gw.addr;
  if (server_ip.addr == 0) return -1;

  p = pbuf_alloc (PBUF_TRANSPORT, len, PBUF_RAM);

  if (!p) return -1;

  if (pbuf_take (p, buf, len) != ERR_OK) {
    pbuf_free (p);
    return -1;
  }

  if (udp_sendto (logging_pcb, p, &server_ip, UDP_LOGGING_PORT) != ERR_OK) {
    pbuf_free (p);
    return -1;
  }

  pbuf_free (p);

  return len;
}

void
logger_printf (const char *fmt, ...)
{

  if (!mp_enabled) {
    va_list args;
    va_start (args, fmt);
    fun_vprintf (com1_putc, fmt, args);
    va_end (args);
  } else {
    if (!logging_if)
      logging_if = netif_find (UDP_LOGGING_NET_IF);
    if (!logging_pcb) {
      logging_pcb = udp_new ();
      if (!logging_pcb)
        panic ("logging_pcb == NULL");
      if (udp_bind (logging_pcb, IP_ADDR_ANY, 0) != ERR_OK)
        panic ("udp_bind of logging pcb failed");
      char *msg = "INITIALIZED UDP LOGGING\n";
      send ((u8 *) msg, strlen (msg));
    }
    logging_idx = 0;
    memset (logging_buf, 0, UDP_LOGGING_BUF_SIZE);
    va_list args;
    va_start (args, fmt);
    fun_vprintf (udp_logging_putc, fmt, args);
    va_end (args);
    send ((u8 *) logging_buf, logging_idx);
  }
}

#else

void
logger_printf (const char *fmt, ...)
{
  va_list args;
  va_start (args, fmt);
  fun_vprintf (logger_putc, fmt, args);
  va_end (args);
}
#endif

static void
_putc (char c)
{
  _putchar (c);
}

void
printf (const char *fmt, ...)
{
  va_list args;
  va_start (args, fmt);
  spinlock_lock (&screen_lock);
  fun_vprintf (_putc, fmt, args);
  spinlock_unlock (&screen_lock);
  va_end (args);
}


void
_printf (const char *fmt, ...)
{
  va_list args;
  va_start (args, fmt);

  fun_vprintf (_putc, fmt, args);

  va_end (args);
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
