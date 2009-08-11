#ifndef _SCREEN_H_
#define _SCREEN_H_

extern char *pchVideo;
extern int putchar (int ch);
extern int print (char *pch);
extern void putx (uint32 l);
/* unlocked: */
extern int _putchar (int ch);
extern int _print (char *pch);
extern void _putx (uint32 l);

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
