/* -*- Mode: C -*- */

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

/* vi: set et sw=2 sts=2: */
