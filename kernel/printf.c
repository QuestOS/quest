#include"types.h"
#include"kernel.h"
#include"acpi.h"                /* Use ACPICA's va_* macros */

void closure_vprintf(void putc_clo(void *,char), void *data, const char *fmt, va_list args) {
  int precision, width, mode, upper;
  char padding;
#define putc(c) putc_clo(data,c)
  while(*fmt) {
    /* handle ordinary characters and directives */
    switch(*fmt) {
    case '\0':
      return;
    case '%':
      fmt++;
      precision = 0;
      width = 0;
      upper = 1;
      padding = ' ';
#define PRINTF_MODE_PRECISION 1
      mode = 0;
      /* handle directive arguments */
      while(*fmt) {
        switch(*fmt) {
        case 'p': {
          /* pointer value */
          uint32 x = va_arg(args,uint32);
          int i, li;

          for( i = 0; i < 8; i++ ) {
            if( ( li = ( x >> ( (7-i) << 2 ) ) & 0x0F ) > 9 )
              putc( 'A' + li - 0x0A );
            else
              putc( '0' + li );
          }
          goto directive_finished;
        }
        case 'x':
          upper = 0;
        case 'X': {
          /* hexadecimal output */
          uint32 x = va_arg(args,uint32);
          int i, li, print_padding = 0, print_digits = 0;

          for( i = 0; i < 8; i++ ) {
            li = ( x >> ( (7-i) << 2 ) ) & 0x0F;

#define HANDLE_OPTIONS(q,max_w,end_i)                                   \
            if (q != 0 || i == end_i)                                   \
              print_digits = 1;                                         \
            if (q == 0 && !print_digits && i >= (max_w - width))        \
              print_padding = 1;                                        \
            if (q == 0 && !print_digits && i >= (max_w - precision))    \
              print_digits = 1;                                         \
            if (q == 0 && print_padding && !print_digits)               \
              putc(padding); 

            HANDLE_OPTIONS(li,8,7);

            if(print_digits) {
              if( li > 9 )
                putc( (upper ? 'A' : 'a') + li - 0x0A );
              else
                putc( '0' + li );
            }
          }

          goto directive_finished;
        }
        case 'u': {
          /* decimal output */
          uint32 x = va_arg(args,uint32);
          int i, q, print_padding = 0, print_digits = 0;
          int divisors[10] = 
            {1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};

          for( i = 0; i < 10; i++ ) {
            q = x / divisors[i];
            x %= divisors[i];

            HANDLE_OPTIONS(q,10,9);
            
            if (print_digits)
              putc('0' + q);
          }

          goto directive_finished;
        }
        case 'd': {
          /* decimal output */
          signed long x = va_arg(args,signed long);
          int i, q, print_padding = 0, print_digits = 0;
          int divisors[10] = 
            {1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};

          if(x < 0) { putc('-'); x *= -1; }
          for( i = 0; i < 10; i++ ) {
            q = x / divisors[i];
            x %= divisors[i];

            HANDLE_OPTIONS(q,10,9);
            
            if (print_digits)
              putc('0' + q);
          }

          goto directive_finished;
        }
        case 's': {
          /* string argument */
          char *s = va_arg(args,char*);
          if (precision > 0) 
            while (*s && precision-- > 0) putc(*s++);
          else 
            while(*s) putc(*s++);
          goto directive_finished;
        }
        case 'c': {
          /* character argument */
          char c = (char)va_arg(args,int); 
          /* char is promoted to int when passed through va_arg */
          putc(c);
          goto directive_finished;
        }
        case '%': {
          /* single % */
          putc('%');
          goto directive_finished;
        }
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
      putc(*fmt);
      break;
    }
    fmt++;
  }
#undef putc
}

static void putc_fun(void *f, char c) {
  void (*g)(char) = f;
  g(c);
}

void fun_vprintf(void putc(char), const char *fmt, va_list args) {
  closure_vprintf(putc_fun, (void *)putc, fmt, args);
}

void fun_printf(void putc(char), const char *fmt, ...) {
  va_list args;
  va_start(args,fmt);
  fun_vprintf(putc,fmt,args);
  va_end(args);
}

void com1_printf(const char *fmt, ...) {
  va_list args;
  va_start(args,fmt);
  fun_vprintf(com1_putc, fmt, args);
  va_end(args);
}

static void _putc(char c) {
  _putchar(c);
}

void printf(const char *fmt, ...) {
  va_list args;
  va_start(args,fmt);
  spinlock_lock(&screen_lock);
  fun_vprintf(_putc, fmt, args);
  spinlock_unlock(&screen_lock);
  va_end(args);
}
