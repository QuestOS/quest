#ifndef _DEBUG_H_
#define _DEBUG_H_

extern void com1_putc (char);
extern void com1_puts (char *);
extern void com1_putx (uint32);

#define DLOG_PREFIX(pre,fmt,...) com1_printf (pre": "fmt"\n", ##__VA_ARGS__)

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
