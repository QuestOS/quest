/* -*- Mode: C; c-file-style: "gnu"; c-basic-offset: 2 -*- */

#include "smp/spinlock.h"
#include "util/screen.h"
#include "util/debug.h"

spinlock screen_lock = { 0 };

int
_putchar (int ch)
{

  static int x, y;

  if (ch == '\n') {
    x = 0;
    y++;

    if (y > 24)
      y = 0;

    return (int) (uint8) ch;
  }

  pchVideo[y * 160 + x * 2] = ch;
  pchVideo[y * 160 + x * 2 + 1] = 7;
  x++;

  return (int) (uint8) ch;
}

int
putchar (int ch)
{
  int x;
  spinlock_lock (&screen_lock);
  x = _putchar (ch);
  spinlock_unlock (&screen_lock);
  return x;
}

int
print (char *pch)
{
  spinlock_lock (&screen_lock);
  while (*pch)
    _putchar (*pch++);
  spinlock_unlock (&screen_lock);
  return 0;
}


void
putx (uint32 l)
{

  int i, li;

  spinlock_lock (&screen_lock);
  for (i = 7; i >= 0; i--)
    if ((li = (l >> (i << 2)) & 0x0F) > 9)
      _putchar ('A' + li - 0x0A);
    else
      _putchar ('0' + li);
  spinlock_unlock (&screen_lock);
}

int
_print (char *pch)
{
  while (*pch)
    _putchar (*pch++);
  return 0;
}


void
_putx (uint32 l)
{

  int i, li;

  for (i = 7; i >= 0; i--)
    if ((li = (l >> (i << 2)) & 0x0F) > 9)
      _putchar ('A' + li - 0x0A);
    else
      _putchar ('0' + li);
}

/* vi: set et sw=2 sts=2: */
