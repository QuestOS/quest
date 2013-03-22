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

/*
 * Taken from Linux sort.c
 */

/*
 * A fast, small, non-recursive O(nlog n) sort for the Linux kernel
 *
 * Jan 23 2005  Matt Mackall <mpm@selenic.com>
 */

#include <util/printf.h>
#include <types.h>
#include <kernel.h>
#include <mem/pow2.h>

//#define DEBUG_SORT

#ifdef DEBUG_SORT
#define DLOG(fmt,...) DLOG_PREFIX("sort",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

static void u32_swap(void *a, void *b, int size)
{
  u32 t = *(u32 *)a;
  *(u32 *)a = *(u32 *)b;
  *(u32 *)b = t;
}

static void generic_swap(void *a, void *b, int size)
{
  char t;
  
  do {
    t = *(char *)a;
    *(char *)a++ = *(char *)b;
    *(char *)b++ = t;
  } while (--size > 0);
}

/**
 * sort - sort an array of elements
 * @base: pointer to data to sort
 * @num: number of elements
 * @size: size of each element
 * @cmp_func: pointer to comparison function
 * @swap_func: pointer to swap function or NULL
 *
 * This function does a heapsort on the given array. You may provide a
 * swap_func function optimized to your element type.
 *
 * Sorting time is O(n log n) both on average and worst-case. While
 * qsort is about 20% faster on average, it suffers from exploitable
 * O(n*n) worst-case behavior and extra memory requirements that make
 * it less suitable for kernel use.
 */

void sort(void *base, size_t num, size_t size,
          int (*cmp_func)(const void *, const void *),
          void (*swap_func)(void *, void *, int size))
{
  /* pre-scale counters for performance */
  int i = (num/2 - 1) * size, n = num * size, c, r;

  if (!swap_func)
    swap_func = (size == 4 ? u32_swap : generic_swap);
  
  /* heapify */
  for ( ; i >= 0; i -= size) {
    for (r = i; r * 2 + size < n; r  = c) {
      c = r * 2 + size;
      if (c < n - size &&
	  cmp_func(base + c, base + c + size) < 0)
	c += size;
      if (cmp_func(base + r, base + c) >= 0)
	break;
      swap_func(base + r, base + c, size);
    }
  }
  
  /* sort */
  for (i = n - size; i > 0; i -= size) {
    swap_func(base, base + i, size);
    for (r = 0; r * 2 + size < i; r = c) {
      c = r * 2 + size;
      if (c < i - size &&
	  cmp_func(base + c, base + c + size) < 0)
	c += size;
      if (cmp_func(base + r, base + c) >= 0)
	break;
      swap_func(base + r, base + c, size);
    }
  }
}


#if 1
/* a simple boot-time regression test */

int cmpint(const void *a, const void *b)
{
  return *(int *)a - *(int *)b;
}

static bool sort_test(void)
{
  int *a, i, r = 1;

  a = kmalloc(1000 * sizeof(int));
        
  if(a == NULL) {
    DLOG("Could not allocate memory for sort_test");
    return FALSE;
  }

  DLOG("testing sort()");

  for (i = 0; i < 1000; i++) {
    r = (r * 725861) % 6599;
    a[i] = r;
  }

  sort(a, 1000, sizeof(int), cmpint, NULL);

  for (i = 0; i < 999; i++)
    if (a[i] > a[i+1]) {
      DLOG("sort() failed!");
      return FALSE;
    }

  kfree((uint8_t*)a);
  DLOG("Sort test was successful");
  return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = sort_test
};

DEF_MODULE (sorttest, "Sort Test", &mod_ops, {});

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
