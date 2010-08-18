/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
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
 *
 * Interrupt handler code: interrupt_handler.c
 *
 */

#include "arch/i386.h"
#include "arch/i386-percpu.h"
#include "arch/i386-measure.h"
#include "kernel.h"
#include "mem/mem.h"
#include "util/elf.h"
#include "fs/filesys.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "util/printf.h"
#include "util/screen.h"
#include "util/debug.h"
#include "drivers/input/keymap.h"
#include "drivers/input/keyboard.h"

//#define DEBUG_SYSCALL
//#define DEBUG_PIT

static char kernel_ver[] = "0.1a";
char *kernel_version = kernel_ver;
uint32 tick;                    /* Software clock tick */

extern uint32 ul_tss[][1024];

/* Table of functions handling interrupt vectors. */
static vector_handler vector_handlers[256];

/* Default function filling above table. */
static uint32
default_vector_handler (uint8 vec)
{
  return 0;
}

/* Program an entry in the table. */
void
set_vector_handler (uint8 vec, vector_handler func)
{
  vector_handlers[vec] = func;
}

/* Reset an entry to the default. */
void
clr_vector_handler (uint8 vec)
{
  vector_handlers[vec] = default_vector_handler;
}

/* Obtain a pointer to the handler. */
vector_handler
get_vector_handler (uint8 vec)
{
  if (vector_handlers[vec])
    return vector_handlers[vec];
  else
    return default_vector_handler;
}

/* This is the function invoked by the CPU which then dispatches to
 * the handler in the table. */
uint32
dispatch_vector (uint32 vec)
{
  vector_handler func = vector_handlers[(uint8) vec];
  uint32 v;
  //com1_printf ("dispatching vec=0x%x\n", vec);

  if (func)
    v = func (vec);
  else
    v = 0;

  if (!mp_apic_mode && PIC2_BASE_IRQ <= vec && vec < (PIC2_BASE_IRQ + 8))
    outb (0x20, 0xA0);          /* send to 8259A slave PIC too */
  send_eoi ();
  return v;
}

/* Duplicate parent TSS -- used with fork */
uint16
duplicate_TSS (uint32 ebp,
               uint32 *esp,
               uint32 child_eip,
               uint32 child_ebp,
               uint32 child_esp,
               uint32 child_eflags, 
               uint32 child_directory)
{

  int i;
  descriptor *ad = (descriptor *) KERN_GDT;
  tss *pTSS;
  uint32 pa;


  pa = alloc_phys_frame (); /* --??-- For now, whole page per tss */
  /* --??-- Error checking in the future */

  /* Establish space for a new TSS: +3 declares present and r/w */

  /* Note, we rely on page being initialised to 0 since EAX contains
   * return value for child
   */

  pTSS = map_virtual_page (pa + 3);

  /* Clear virtual page before use. */
  memset (pTSS, 0, 4096);

  /* Search 2KB GDT for first free entry */
  for (i = 1; i < 256; i++)
    if (!(ad[i].fPresent))
      break;

  if (i == 256)
    panic ("No free selector for TSS");

  logger_printf ("duplicate_TSS: pTSS=%p i=0x%x\n", pTSS, i << 3);

  /* See pp 6-7 in IA-32 vol 3 docs for meanings of these assignments */
  ad[i].uLimit0 = 0xFFF;        /* --??-- Right now, a page per TSS */
  ad[i].uLimit1 = 0;
  ad[i].pBase0 = (uint32) pTSS & 0xFFFF;
  ad[i].pBase1 = ((uint32) pTSS >> 16) & 0xFF;
  ad[i].pBase2 = (uint32) pTSS >> 24;
  ad[i].uType = 0x09;           /* 32-bit tss */
  ad[i].uDPL = 0;               /* Only let kernel perform task-switching */
  ad[i].fPresent = 1;
  ad[i].f0 = 0;
  ad[i].fX = 0;
  ad[i].fGranularity = 0;       /* Set granularity of tss in bytes */

  pTSS->pCR3 = (void *) child_directory;

  /* The child will begin running at the specified EIP */
  pTSS->ulEIP = child_eip;
  pTSS->ulEFlags = child_eflags & 0xFFFFBFFF;   /* Disable NT flag */
  pTSS->ulESP = child_esp;
  pTSS->ulEBP = child_ebp;
  pTSS->usES = 0x10;
  pTSS->usCS = 0x08;
  pTSS->usSS = 0x10;
  pTSS->usDS = 0x10;
  pTSS->usFS = 0x10;
  pTSS->usGS = 0x10;
  pTSS->usIOMap = 0xFFFF;
  pTSS->usSS0 = 0x10;           /* Kernel stack segment */
  pTSS->ulESP0 = (uint32) KERN_STK + 0x1000;

  quest_tss *tssp = (quest_tss *)pTSS;
  tssp->cpu = 0xFF;

  /* Return the index into the GDT for the segment */
  return i << 3;
}

char *exception_messages[] = {
  "Division By Zero",
  "Debug",
  "Non Maskable Interrupt",
  "Breakpoint",
  "Into Detected Overflow",
  "Out of Bounds",
  "Invalid Opcode",
  "No Coprocessor",
  "Double Fault",
  "Coprocessor Segment Overrun",
  "Bad TSS",
  "Segment Not Present",
  "Stack Fault",
  "General Protection Fault",
  "Page Fault",
  "Unknown Interrupt",
  "Coprocessor Fault",
  "Alignment Check",
  "Machine Check",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved"
};

/* Generic CPU fault exception handler -- dumps some info to the
 * screen and serial port, then panics. */
extern void
handle_interrupt (uint32 fs_gs, uint32 ds_es, uint32 ulInt, uint32 ulCode)
{

  uint32 eax, ebx, ecx, edx, esi, edi, eflags, eip, esp, ebp;
  uint32 cr0, cr2, cr3;
  uint16 tr, fs;

  asm volatile ("movl %%eax, %0\n"
                "movl %%ebx, %1\n"
                "movl %%ecx, %2\n"
                "movl %%edx, %3\n"
                "movl %%esi, %4\n"
                "movl %%edi, %5\n"
                "movl (%%ebp), %%eax\n"
                "movl %%eax, %6\n"
                "movl 0x18(%%ebp),%%eax\n"
                "movl %%eax, %7\n"
                "movl 0x20(%%ebp),%%eax\n"
                "movl %%eax, %8\n"
                "movl 0x24(%%ebp),%%eax\n"
                "movl %%eax, %9\n"
                "movl %%cr0, %%eax\n"
                "movl %%eax, %10\n"
                "movl %%cr2, %%eax\n"
                "movl %%eax, %11\n"
                "movl %%cr3, %%eax\n"
                "movl %%eax, %12\n"
                "xorl %%eax, %%eax\n"
                "str  %%ax\n"
                "movw %%ax, %13\n"
                "movw %%fs, %%ax\n"
                "movw %%ax, %14\n"
                :"=m" (eax), "=m" (ebx), "=m" (ecx),
                 "=m" (edx), "=m" (esi), "=m" (edi), "=m" (ebp), "=m" (eip),
                 "=m" (eflags), "=m" (esp), "=m" (cr0), "=m" (cr2), "=m" (cr3),
                 "=m" (tr), "=m" (fs):);

  spinlock_lock (&screen_lock);
  _putchar ('I');
  _putx (ulInt);
  _putchar (' ');
  _putchar ('c');
  _putx (ulCode);
  _putchar (' ');
  if (ulInt < 32)
    _print (exception_messages[ulInt]);
  _putchar ('\n');

#ifndef ENABLE_GDBSTUB
#define _putchar com1_putc
#define _putx com1_putx
#define _print com1_puts
#define _printf com1_printf
#endif

  _printf ("INT=%.2X CODE=%.8X %s\n", 
           ulInt, ulCode, exception_messages[ulInt]);
  _printf ("EAX=%.8X ESI=%.8X\n", eax, esi);
  _printf ("EBX=%.8X EDI=%.8X\n", ebx, edi);
  _printf ("ECX=%.8X EBP=%.8X\n", ecx, ebp);
  _printf ("EDX=%.8X ESP=%.8X\n", edx, esp);
  _printf ("EFL=%.8X EIP=%.8X\n", eflags, eip);
  _printf ("CR0=%.8X CR2=%.8X\nCR3=%.8X TR=%.4X\n", cr0, cr2, cr3, tr);
  _printf (" FS=%.4X\n", fs);
  stacktrace_frame (esp, ebp);

#ifndef ENABLE_GDBSTUB
#undef _putx
#undef _putchar
#undef _print
#undef _printf
#endif

  spinlock_unlock (&screen_lock);

  if (ulInt < 0x20)
    /* unhandled exception - die */
    panic ("Unhandled exception");

  send_eoi ();
}

/* Syscall: putchar */
void
handle_syscall0 (int eax, int ebx)
{

  quest_tss *pTSS = (quest_tss *) ul_tss[1];    /* --??-- tss index hard-coded to 1 for now */
  uint16 head;

  if (eax) {                    /* eax should be 0 for syscall0 */
    print ("Invalid syscall number!\n");
    return;
  }

  /* NB: The code below relies on atomic execution.  Currently, that
     atomicity is guaranteed by assuming we are running on a
     uniprocessor with interrupts disabled. */

  lock_kernel ();

  if (pTSS->busy) {
    /* somebody else is already using the server; block */
    queue_append (&pTSS->waitqueue, str ());    /* add ourselves to the wait
                                                   queue -- this operation
                                                   must be atomic with the
                                                   busy check above */
    schedule ();
    /* We can now safely assume that we have exclusive access to the
       server (since the only way we could possibly have woken up after
       the schedule() call is by the previous head task placing us on
       the run queue -- see below). */
  } else
    pTSS->busy = 1;             /* mark the server busy -- this set must be atomic
                                   with the test above */

  pTSS->tss.ulEBX = ebx;        /* pass arg in EBX from client to server */

  unlock_kernel ();

  call_gate (0x30);             /* --??-- Hard-coded for the segment selector for
                                   terminal server task */

  lock_kernel ();

  if ((head = queue_remove_head (&pTSS->waitqueue)))
    /* Somebody else is waiting for the server -- wake them up (and leave
       the busy flag set).  This will eventually cause the schedule() call
       in the waiting task to return (see above). */
    wakeup (head);
  else
    /* We were the last task using the server; mark it as available.
       Clearing this flag must be atomic with the queue_remove_head()
       check. */
    pTSS->busy = 0;

  unlock_kernel ();
}


/* Syscall: fork
 *
 * esp argument used to find info about parent's eip and other
 * registers inherited by child
 */
task_id
_fork (uint32 ebp, uint32 *esp)
{

  uint16 child_gdt_index;
  void *phys_addr;
  uint32 *virt_addr;
  int i, j;
  uint32 *child_directory, *child_page_table, *parent_page_table;
  void *child_page, *parent_page;
  uint32 tmp_dir, tmp_page, tmp_page_table;
  uint32 priority;
  uint32 eflags, eip, this_esp, this_ebp;

#ifdef DEBUG_SYSCALL
  com1_printf ("_fork (%X, %p)\n", ebp, esp);
#endif
  lock_kernel ();

  child_directory = map_virtual_page ((tmp_dir = alloc_phys_frame ()) | 3);

  /* 
   * This ugly bit of assembly is designed to obtain the value of EIP
   * and allow the `call 1f' to return twice -- first for the parent to
   * obtain EIP, and the second time for the child when it begins running.
   *
   */

  asm volatile ("call 1f\n"
                "movl $0, %0\n"
                "jmp 2f\n"
                "1:\n"
                "movl (%%esp), %0\n" "addl $4, %%esp\n" "2:\n":"=r" (eip):);

  if (eip == 0) {
    /* We are in the child process now */
    unlock_kernel ();
    /* don't need to reload per-CPU segment here because we are going
     * straight to userspace */
    return 0;
  }

  asm volatile ("movl %%ebp, %0\n"
                "movl %%esp, %1\n"
                "pushfl\n"
                "pop %2\n":"=r" (this_ebp), "=r" (this_esp), "=r" (eflags):);

  /* Create a child task which is the same as this task except that it will
   * begin running at the program point after `call 1f' in the above inline asm. */

  child_gdt_index =
    duplicate_TSS (ebp, esp, eip, this_ebp, this_esp, eflags, tmp_dir);

  /* Allocate physical memory for new address space 
   *
   */
  phys_addr = get_pdbr ();      /* Parent page dir base address */

  virt_addr = map_virtual_page ((uint32) phys_addr | 3);        /* Temporary virtual address */

  for (i = 0; i < 0x3FF; i++) { /* Walk user-level regions of pgd */

    if (virt_addr[i] &&         /* Valid page table found, and */
        !(virt_addr[i] & 0x80)) {       /* not 4MB page */
      child_page_table =
        map_virtual_page ((tmp_page_table = alloc_phys_frame ()) | 3);
      parent_page_table = map_virtual_page ((virt_addr[i] & 0xFFFFF000) | 3);

      /* Copy parent's page table mappings to child */
      for (j = 0; j < 1024; j++) {
        if (parent_page_table[j]) {     /* --??-- Assume non-zero means present */
          child_page =
            map_virtual_page ((tmp_page = alloc_phys_frame ()) | 3);
          parent_page =
            map_virtual_page ((parent_page_table[j] & 0xFFFFF000) | 3);

          /* --??-- TODO: Copy-on-write style forking and support
             for physical page frame sharing */
          memcpy (child_page, parent_page, 0x1000);

          child_page_table[j] = tmp_page | (parent_page_table[j] & 0xFFF);

          unmap_virtual_page (child_page);
          unmap_virtual_page (parent_page);
        } else {
          child_page_table[j] = 0;
        }
      }

      /* Create page directory for child */
      child_directory[i] = tmp_page_table | (virt_addr[i] & 0xFFF);

      unmap_virtual_page (child_page_table);
      unmap_virtual_page (parent_page_table);

    } else {
      child_directory[i] = 0;
    }
  }

  /* Copy kernel code and APIC mappings into child's page directory */
  child_directory[1023] = virt_addr[1023];
  child_directory[1019] = virt_addr[1019];

  unmap_virtual_page (virt_addr);

  unmap_virtual_page (child_directory);

  /* Inherit priority from parent */
  priority = lookup_TSS (child_gdt_index)->priority =
    lookup_TSS (str ())->priority;

  wakeup (child_gdt_index);

  /* --??-- Duplicate any other parent resources as necessary */

  unlock_kernel ();

  return child_gdt_index;       /* Use this index for child ID for now */
}


char *
strncpy (char *s1, const char *s2, int length)
{

  while ((length--) && (*s1++ = *s2++));

  if (length < 0)
    *(s1 - 1) = '\0';

  return s1;
}


/* --??-- TODO: Rewrite _exec to create a temporary new address space
 before overwriting the old one in case of errors */

/* Syscall: _exec: replace address space of caller with new memory areas, in part
 * populated by program image on disk
 */
int
_exec (char *filename, char *argv[], uint32 *curr_stack)
{

  uint32 *plPageDirectory = map_virtual_page ((uint32) get_pdbr () | 3);
  uint32 *plPageTable;
  uint32 pStack;
  Elf32_Ehdr *pe = (Elf32_Ehdr *) 0xFF400000;   /* 4MB below KERN_STK virt address */
  Elf32_Phdr *pph;
  void *pEntry;
  int filesize, orig_filesize;
  /* Temporary storage for frame pointers for a file image up to 4MB
     discounting bss */
  uint32 phys_addr = alloc_phys_frame () | 3;
  /* frame_map is a 1024 bit bitmap to mark frames not needed for 
     file of specific size when not all sections need loading into RAM */
  uint32 frame_map[32];
  uint32 *frame_ptr = map_virtual_page (phys_addr);
  uint32 *tmp_page;
  int i, j, c;
  char command_args[80];

  if (!argv || !argv[0]) {
    BITMAP_SET (mm_table, phys_addr >> 12);
    unmap_virtual_page (plPageDirectory);
    unmap_virtual_page (frame_ptr);
    return -1;
  }

  lock_kernel ();
#ifdef DEBUG_SYSCALL
  com1_printf ("_exec (%s, [%s,...], %p)\n", filename, argv[0], curr_stack);
#endif

  /* --??-- Checks should be added here for valid argv[0] etc...
     Allocate space for argument vector passed via exec call.
     Right now, assume max size for prog name and arguments.
   */
  strncpy (command_args, argv[0], 80);

#ifdef DEBUG_SYSCALL
  com1_printf ("_exec: vfs_dir\n");
#endif
  /* Find file on disk -- essentially a basic open call */
  if ((filesize = vfs_dir (filename)) < 0) {    /* Error */
    BITMAP_SET (mm_table, phys_addr >> 12);
    unmap_virtual_page (plPageDirectory);
    unmap_virtual_page (frame_ptr);
    unlock_kernel ();
    return -1;
  }

  /* Free frames used for old address space before _exec was called
   *
   * Reuse page directory
   */
#ifdef DEBUG_SYSCALL
  com1_printf ("_exec: setup page directory\n");
#endif
  for (i = 0; i < 1019; i++) {  /* Skip freeing kernel pg table mapping and
                                   kernel stack space. */
    if (plPageDirectory[i]) {   /* Present in currrent address space */
      tmp_page = map_virtual_page (plPageDirectory[i] | 3);
      for (j = 0; j < 1024; j++) {
        if (tmp_page[j]) {      /* Present in current address space */
          if ((j < 0x200) || (j > 0x20F) || i) {        /* --??-- Don't free
                                                           temp video memory */
            BITMAP_SET (mm_table, tmp_page[j] >> 12);   /* Free frame */
            tmp_page[j] = 0;
          }
        }
      }
      unmap_virtual_page (tmp_page);
      BITMAP_SET (mm_table, plPageDirectory[i] >> 12);
      plPageDirectory[i] = 0;
    }
  }

  /* Allocate space for new page table */
  plPageDirectory[0] = alloc_phys_frame () | 7;
  plPageTable = map_virtual_page (plPageDirectory[0]);
  memset (plPageTable, 0, 0x1000);

  for (i = 0; i < filesize; i += 4096) {
    frame_ptr[i >> 12] = alloc_phys_frame () | 3;
  }

  /* Temporary dir entry for mapping file image into virtual address space */
  plPageDirectory[(uint32) pe >> 22] = phys_addr;

  flush_tlb_all ();

#ifdef DEBUG_SYSCALL
  com1_printf ("_exec: vfs read\n");
#endif
  /* Read into virtual address corresponding to plPageDirectory[1021] */
  orig_filesize = filesize;
  filesize = vfs_read ((void *) pe, orig_filesize);
  if (filesize != orig_filesize) {
    printf ("expected filesize=%d got filesize=%d\n", orig_filesize, filesize);
    panic ("File size mismatch on read");
  }

  pph = (void *) pe + pe->e_phoff;
  pEntry = (void *) pe->e_entry;

  memset (frame_map, 0, 32 * sizeof (uint32));

#ifdef DEBUG_SYSCALL
  com1_printf ("_exec: walk ELF header\n");
#endif
  /* Walk ELF header */
  for (i = 0; i < pe->e_phnum; i++) {
    if (pph->p_type == PT_LOAD) {
      if ((pph->p_offset & 0xFFF) != (pph->p_vaddr & 0xFFF))
        panic ("Misalignment in program header");

      /* map pages loaded from file */
      c = ((pph->p_offset + pph->p_filesz - 1) >> 12) - (pph->p_offset >> 12) + 1;      /* #pages to load for module */

      for (j = 0; j < c; j++) {
        if (j == c - 1) {
          /* Page is the last of this header, and needs to be
             zero-padded, but unfortunately the page may be
             shared with the next phdr.  We copy it to avoid any
             conflicts. */
          uint32 frame = alloc_phys_frame ();
          char *buf = map_virtual_page (frame | 3);
          int partial = (pph->p_offset + pph->p_filesz) & 0xFFF;

          memcpy (buf, (char *) pe + (pph->p_offset & ~0xFFF) +
                  (j << 12), partial);
          memset (buf + partial, 0, 0x1000 - partial);

          unmap_virtual_page (buf);

          plPageTable[((uint32) pph->p_vaddr >> 12) + j] = frame | 7;
        } else {
          BITMAP_SET (frame_map, j + (pph->p_offset >> 12));
          plPageTable[((uint32) pph->p_vaddr >> 12) + j] =
            frame_ptr[j + (pph->p_offset >> 12)] | 7;
        }
      }

      /* map additional zeroed pages */
      c = ((pph->p_offset + pph->p_memsz - 1) >> 12) - (pph->p_offset >> 12) + 1;       /* page limit to clear for module */

      /* Allocate space for bss section.  Use temporary virtual memory for
       * memset call to clear physical frame(s)
       */
      for (; j < c; j++) {
        uint32 page_frame = (uint32) alloc_phys_frame ();
        void *virt_page = map_virtual_page (page_frame | 3);
        plPageTable[((uint32) pph->p_vaddr >> 12) + j] = page_frame | 7;
        memset (virt_page, 0, 0x1000);
        unmap_virtual_page (virt_page);
      }
    }

    pph = (void *) pph + pe->e_phentsize;
  }

  /* Deallocate unsued frames for file that were not loaded with contents */
  for (i = 0; i < filesize; i += 4096) {
    if (!BITMAP_TST (frame_map, i >> 12))
      BITMAP_SET (mm_table, frame_ptr[i >> 12] >> 12);
  }

  /* --??-- temporarily map video memory into exec()ed process */
  for (i = 0; i < 16; i++)
    plPageTable[0x200 + i] = 0xA0000 | (i << 12) | 7;

  /* map stack and clear its contents -- Here, setup 16 pages for stack */
  for (i = 0; i < 16; i++) {
    pStack = alloc_phys_frame ();
    plPageTable[1023 - i] = pStack | 7;
    invalidate_page ((void *) ((1023 - i) << 12));
  }
  memset ((void *) 0x3F0000, 0, 0x10000);       /* Clear 16 page stack */

  plPageDirectory[1021] = 0;
  unmap_virtual_page (plPageDirectory);
  unmap_virtual_page (plPageTable);
  unmap_virtual_page (frame_ptr);
  BITMAP_SET (mm_table, phys_addr >> 12);

  flush_tlb_all ();

  /* Copy command-line arguments to top of new stack */
  memcpy ((void *) (0x400000 - 80), command_args, 80);

  /* Push onto stack argument vector for when we call _start in our "libc"
     library. Here, we work with user-level virtual addresses for when we
     return to user. */
  *(uint32 *) (0x400000 - 84) = 0;    /* argv[1] -- not used right now */
  *(uint32 *) (0x400000 - 88) = 0x400000 - 80;        /* argv[0] */
  *(uint32 *) (0x400000 - 92) = 0x400000 - 88;        /* argv */
  *(uint32 *) (0x400000 - 96) = 1;    /* argc -- hard-coded right now */

  /* Dummy return address placed here for the simulated "call" to our
     library */
  *(uint32 *) (0x400000 - 100) = 0;   /* NULL return address -- never
                                           used */

  /* Patch up kernel stack with new values so that we can start new program
     on return to user-level  */
  curr_stack[0] = 0x00230023;   /* fs/gs selectors */
  curr_stack[1] = 0x00230023;   /* ds/es selectors */
  curr_stack[2] = (uint32) pEntry;        /* eip */
  curr_stack[3] = 0x1B;         /* cs selector */
  /* --??-- Temporarily set IOPL 3 in exec()ed program for VGA/keyboard testing */
  curr_stack[4] = F_1 | F_IF | 0x3000;  /* EFLAGS */
  curr_stack[5] = 0x400000 - 100;       /* -100 after pushing command-line args */
  curr_stack[6] = 0x23;         /* ss selector */

  unlock_kernel ();
  return 0;
}

/* Syscall: getchar / getcode */
uint
_getchar (uint ebx)
{

  uint c = 0;

  lock_kernel ();

  if (ebx == 0)
    c = keymap_getchar ();
  else {
    key_event e;
    uint i;

    keyboard_8042_next (&e);
    for (i=0; i<KEY_EVENT_MAX; i++) {
      if (e.keys[i].latest) {
        c = e.keys[i].scancode;
        if (e.keys[i].release)
          c |= 0x80;            /* restore "break" code */
        break;
      }
    }
  }
  
  unlock_kernel ();

  return c;
}

/* Syscall: switch to other task -- deprecated */
void
_switch_to (uint32 pid)
{

  /* This would cause a #PF in SMP situation, usually: */
  /*******************
   * jmp_gate (pid); *
   *******************/
  
  /* Stick to this */
  uint32 _waitpid(task_id);
  _waitpid(pid);
}


/* Syscall: open --??-- Flags not used for now...a crude open call as
 * it stands  */
int
_open (char *pathname, int flags)
{

  return (vfs_dir (pathname));

}

/* Syscall: read --??-- proess-global file handle */
int
_read (char *pathname, void *buf, int count)
{

  return (vfs_read (buf, count));

}

/* Syscall: uname */
int
_uname (char *name)
{

  /* --??-- Error check in the future */
  memcpy (name, kernel_version, sizeof (kernel_ver));

  return 0;
}

/* Syscalls: meminfo, shared mem alloc, attach, detach, and free. */
uint32
_meminfo (uint32 eax, uint32 edx)
{

  int i, j = 0;

  uint32 frame;
  uint32 pgd;
  uint32 *pgd_virt, *ptab1_virt;
  uint32 addr;

  switch (eax) {
  case 0:
    for (i = 0; i < mm_limit; i++)
      if (BITMAP_TST (mm_table, i))
        j++;

    return j << 12;
  case 1:{
      void *virt;
      /* shared_mem_alloc() */
      frame = alloc_phys_frame ();
      if (frame < 0)
        return -1;
      /* use 'frame' as identifier of shared memory region
       * obvious security flaw -- but its just for testing, atm */
      virt = map_virtual_page (frame | 3);
      memset ((void *) virt, 0, 0x1000);
      unmap_virtual_page (virt);
      return frame;
    }
  case 2:{
      /* shared_mem_attach() */
      frame = edx;
      if ((frame >> 12) >= mm_limit)
        /* invalid frame */
        return -1;
      if (BITMAP_TST (mm_table, frame >> 12))
        /* unallocated frame */
        return -1;
      /* Now find a userspace page to map to this frame */
      pgd = (uint32) get_pdbr ();
      pgd_virt = map_virtual_page (pgd | 3);
      ptab1_virt = map_virtual_page (pgd_virt[0] | 3);
      /* Going to assume I can just use the first page table for this */
      addr = -1;
      for (i = 1; i < 1024; i++) {
        if ((ptab1_virt[i] & 0x1) == 0) {
          /* found empty entry */
          ptab1_virt[i] = frame | 7;
          addr = i << 12;
          break;
        }
      }
      unmap_virtual_page (ptab1_virt);
      unmap_virtual_page (pgd_virt);
      return addr;
    }
  case 3:{
      /* shared_mem_detach() */
      i = (edx >> 12) & 0x3FF;  /* index into page table */
      pgd = (uint32) get_pdbr ();
      pgd_virt = map_virtual_page (pgd | 3);
      ptab1_virt = map_virtual_page (pgd_virt[0] | 3);
      ptab1_virt[i] = 0;
      unmap_virtual_page (ptab1_virt);
      unmap_virtual_page (pgd_virt);
      return 0;
    }
  case 4:{
      /* shared_mem_free() */
      frame = edx;
      /* again, this is insecure atm */
      BITMAP_SET (mm_table, frame >> 12);
      return 0;
    }
  default:
    return -1;
  }
}

/* Syscall: time */
uint32
_time (void)
{

  return tick;
}

/* ACPI System Control Interrupt -- IRQ 9 usually */
extern uint32
_interrupt29 (void)
{
  extern ACPI_OSD_HANDLER acpi_service_routine;
  extern void *acpi_service_routine_context;
  if (acpi_service_routine)
    return acpi_service_routine (acpi_service_routine_context);
  else
    return 0;
}

/* LAPIC timer handler -- used to implement scheduler quantum on a
 * per-CPU basis. */
extern void
_interrupt3e (void)
{
  uint8 phys_id = LAPIC_get_physical_ID ();
  send_eoi ();
  LAPIC_start_timer (cpu_bus_freq / QUANTUM_HZ); /* setup next tick */

  lock_kernel ();

  if (str () != idleTSS_selector[phys_id]) {
    /* CPU was not idling */
    /* add the current task to the back of the run queue */
    wakeup (str ());
  }

  /* with kernel locked, go ahead and schedule */
  schedule ();
  unlock_kernel ();
}

/* IRQ0 system timer interrupt handler: simply updates the system clock
   tick for now */
void
_timer (void)
{
  void begin_kernel_threads (void);
  extern volatile bool mp_enabled;
  extern bool mp_ISA_PC;
  void net_tmr_process (void);

#ifdef DEBUG_PIT
  com1_printf ("tick: %u\n", tick);
#endif

  tick++;

  /* Need to issue an EOI "end of interrupt" to be ready for further
     interrupts */
  send_eoi ();

  if (mp_enabled) {
    lock_kernel ();

    /* check sleeping processes */
    process_sleepqueue ();

    unlock_kernel ();

#ifdef GDBSTUB_TCP
    { 
      extern bool break_requested; 
      if (break_requested) {
        break_requested = FALSE;
        BREAKPOINT ();
      }
    }      
#endif
#if 0
    {
      void umsc_tmr_test (void);
      umsc_tmr_test ();
    }
#endif
  }

  if (!mp_ISA_PC) {
    if (!mp_enabled)
      com1_printf ("timer: enabling scheduling\n");
    mp_enabled = 1;
    begin_kernel_threads ();
  } else {
    begin_kernel_threads ();    /* has internal flag */
    /* On an ISA PC, must use PIT IRQ for scheduling */
    if (str () == idleTSS_selector[0]) {
      /* CPU was idling */
      schedule ();
      /* if returned, go back to idling */
      return;
    } else {
      /* add the current task to the back of the run queue */
      wakeup (str ());
      /* with kernel locked, go ahead and schedule */
      schedule ();
    }
  }
}

/* Syscall: _exit */
void
__exit (int status)
{

  void *phys_addr;
  uint32 *virt_addr;
  uint32 *tmp_page;
  int i, j;
  task_id tss;
  descriptor *ad = (descriptor *) KERN_GDT;
  uint32 *kern_page_table = (uint32 *) KERN_PGT;
  quest_tss *ptss;
  int waiter;

  lock_kernel ();

  /* For now, simply free up memory used by calling process address
     space.  We will pass the exit status to the parent process in the
     future. */

  phys_addr = get_pdbr ();
  virt_addr = map_virtual_page ((uint32) phys_addr | 3);

  /* Free user-level virtual address space */
  for (i = 0; i < 1023; i++) {
    if (virt_addr[i]            /* Free page directory entry */
        &&!(virt_addr[i] & 0x80)) {     /* and not 4MB page */
      tmp_page = map_virtual_page (virt_addr[i] | 3);
      for (j = 0; j < 1024; j++) {
        if (tmp_page[j]) {      /* Free frame */
          if ((j < 0x200) || (j > 0x20F) || i) {        /* --??-- Skip releasing
                                                           video memory */
            BITMAP_SET (mm_table, tmp_page[j] >> 12);
          }
        }
      }
      unmap_virtual_page (tmp_page);
      BITMAP_SET (mm_table, virt_addr[i] >> 12);
    }
  }
  BITMAP_SET (mm_table, (uint32) phys_addr >> 12);    /* Free up page for page directory */
  unmap_virtual_page (virt_addr);

  /* --??-- Need to release TSS used by exiting process. Here, we need a way
     to index GDT based on current PID returned from original fork call.

     NOTE: Here' we shouldn't really release the TSS until the parent has
     been able to check the status of the child... */

  tss = str ();
  ltr (dummyTSS_selector);

  /* Remove space for tss -- but first we need to construct the linear
     address of where it is in memory from the TSS descriptor */
  ptss = lookup_TSS (tss);

  /* All tasks waiting for us now belong on the runqueue. */
  while ((waiter = queue_remove_head (&ptss->waitqueue)))
    wakeup (waiter);

  BITMAP_SET (mm_table,
              kern_page_table[((uint32) ptss >> 12) & 0x3FF] >> 12);

  /* Remove tss descriptor entry in GDT */
  memset (ad + (tss >> 3), 0, sizeof (descriptor));

  unmap_virtual_page (ptss);

  schedule ();
  /* never return */
  panic ("__exit: unreachable");
}

/* Syscall: waitpid */
extern uint32
_waitpid (task_id pid)
{

  quest_tss *ptss;

  lock_kernel ();

  ptss = lookup_TSS (pid);

  if (ptss) {
    /* Destination task exists.  Add ourselves to the queue of tasks
       waiting for it. */
    queue_append (&ptss->waitqueue, str ());
    /* We have to go to sleep now -- find another task. */
    schedule ();
    unlock_kernel ();
    /* We have been woken up (see __exit).  Return successfully. */
    return 0;
  } else {
    unlock_kernel ();
    /* Destination task does not exist.  Return an error. */
    return -1;
  }
}


extern int
_sched_setparam (task_id pid, const struct sched_param *p)
{

  quest_tss *ptss;

  lock_kernel ();

  ptss = lookup_TSS (pid);

  if (ptss) {
    if (p->sched_priority == -1)        /* Assume window-constrained task */
      ptss->priority = (p->k * p->T) / p->m;
    else
      ptss->priority = p->sched_priority;

    wakeup (str ());

    schedule ();
    unlock_kernel ();

    return 0;

  } else
    unlock_kernel ();
  /* Destination task does not exist.  Return an error. */
  return -1;
}

#if 0
static void *tlb_shootdown_page = NULL;
static uint32 tlb_shootdown_count = 0;
static spinlock tlb_shootdown_lock = SPINLOCK_INIT;

extern void
invlpg_shootdown (void *va)
{

  invalidate_page (va);
  send_ipi (0xFF,
            LAPIC_ICR_LEVELASSERT |
            LAPIC_ICR_DS_ALLEX | LAPIC_ICR_DM_LOGICAL | 0xfd);

}

extern uint32
invlpg_handler (uint8 vec)
{
  asm volatile ("invlpg %0"::"m" (*(char *) tlb_shootdown_page));
  asm volatile ("lock decl %0"::"m" (tlb_shootdown_count));
  return 0;
}

extern uint32
flush_tlb_handler (uint8 vec)
{
  asm volatile ("movl %%cr3, %%eax\n"
                "movl %%eax, %%cr3\n"
                "lock decl %0"::"m" (tlb_shootdown_count));
  return 0;
}

#endif

/* Initialize the vector handling table. */
extern void
init_interrupt_handlers (void)
{
  int i;
  for (i = 0; i < 256; i++)
    vector_handlers[i] = default_vector_handler;
  /************************************************
   * set_vector_handler(0xfd, invlpg_handler);    *
   * set_vector_handler(0xfe, flush_tlb_handler); *
   ************************************************/
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
