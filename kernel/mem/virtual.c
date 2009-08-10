/* -*- Mode: C; c-file-style: "gnu"; c-basic-offset: 2 -*- */

#include "arch/i386.h"
#include "kernel.h"
#include "types.h"


extern uint32 _kernelstart;


/* Find free virtual page and map it to a corresponding physical frame 
 *
 * Returns virtual address
 *
 */
void *
map_virtual_page (uint32 phys_frame)
{

  uint32 *page_table = (uint32 *) KERN_PGT;
  int i;
  void *va;

  for (i = 0; i < 0x400; i++)
    if (!page_table[i]) {       /* Free page */
      page_table[i] = phys_frame;

      va = (char *) &_kernelstart + (i << 12);

      /* Invalidate page in case it was cached in the TLB */
      invalidate_page (va);

      return va;
    }

  return NULL;                  /* Invalid address */
}

/* Map contiguous physical to virtual memory */
void *
map_contiguous_virtual_pages (uint32 phys_frame, uint32 count)
{
  uint32 *page_table = (uint32 *) KERN_PGT;
  int i, j;
  void *va;

  if (count == 0)
    return NULL;

  for (i = 0; i < 0x400 - count + 1; i++) {
    if (!page_table[i]) {       /* Free page */
      for (j = 0; j < count; j++) {
        if (page_table[i + j]) {
          /* Not enough pages in this window */
          i = i + j;
          goto keep_searching;
        }
      }

      for (j = 0; j < count; j++) {
        page_table[i + j] = phys_frame + j * 0x1000;
      }

      va = (char *) &_kernelstart + (i << 12);

      /* Invalidate page in case it was cached in the TLB */
      for (j = 0; j < count; j++) {
        invalidate_page (va + j * 0x1000);
      }

      return va;
    }
  keep_searching:
    ;
  }

  return NULL;                  /* Invalid address */
}

/* Map non-contiguous physical memory to contiguous virtual memory */
void *
map_virtual_pages (uint32 * phys_frames, uint32 count)
{
  uint32 *page_table = (uint32 *) KERN_PGT;
  int i, j;
  void *va;

  if (count == 0)
    return NULL;

  for (i = 0; i < 0x400 - count + 1; i++) {
    if (!page_table[i]) {       /* Free page */
      for (j = 0; j < count; j++) {
        if (page_table[i + j]) {
          /* Not enough pages in this window */
          i = i + j;
          goto keep_searching;
        }
      }

      for (j = 0; j < count; j++) {
        page_table[i + j] = phys_frames[j];
      }

      va = (char *) &_kernelstart + (i << 12);

      /* Invalidate page in case it was cached in the TLB */
      for (j = 0; j < count; j++) {
        invalidate_page (va + j * 0x1000);
      }

      return va;
    }
  keep_searching:
    ;
  }

  return NULL;                  /* Invalid address */
}


/* 
 * Release previously mapped virtual page 
 */
void
unmap_virtual_page (void *virt_addr)
{

  uint32 *page_table = (uint32 *) KERN_PGT;

  page_table[((uint32) virt_addr >> 12) & 0x3FF] = 0;

  /* Invalidate page in case it was cached in the TLB */
  invalidate_page (virt_addr);
}

void
unmap_virtual_pages (void *virt_addr, uint32 count)
{
  int j;
  for (j = 0; j < count; j++)
    unmap_virtual_page (virt_addr + j * 0x1000);
}


void *
get_phys_addr (void *virt_addr)
{

  void *pa;
  uint32 phys_frame;
  uint32 va = (uint32) virt_addr;

  uint32 *kernel_pdbr = (uint32 *) get_pdbr (); /* --WARN-- Assumes
                                                 * virtual and phys addrs
                                                 * are the same. Okay to
                                                 * use at boot time up
                                                 * until we activate the
                                                 * first dynamically
                                                 * created address space
                                                 */
  uint32 *kernel_ptbr;

  kernel_ptbr = (uint32 *) (kernel_pdbr[va >> 22] & 0xFFFFF000);

  phys_frame = kernel_ptbr[(va >> 12) & 0x3FF] & 0xFFFFF000;

  pa = (void *) (phys_frame + (va & 0x00000FFF));

  return pa;
}

/* vi: set et sw=2 sts=2: */
