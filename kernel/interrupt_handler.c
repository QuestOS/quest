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
#include "sched/sched.h"
#include "sched/vcpu.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#ifdef USE_VMX
#include "vm/shm.h"
#include "vm/spow2.h"
#endif

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

bool
vector_used (uint8 vec)
{
  return (get_vector_handler (vec) != (&default_vector_handler));
}

u8
find_unused_vector (u8 min_prio)
{
  u8 i;
  if (min_prio < MINIMUM_VECTOR_PRIORITY || min_prio > 0xF)
    return 0;
  for (i=(min_prio << 4); i < 0xFF; i++) {
    if (vector_handlers[i] == default_vector_handler)
      return i;
  }
  return (vector_handlers[i] == default_vector_handler ? i : 0);
}

char *exception_messages[] = {
  "Division Error",
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
 * screen and serial port, then goes into debugger. */
extern void
handle_interrupt (u32 edi, u32 esi, u32 ebp, u32 _esp, u32 ebx, u32 edx, u32 ecx, u32 eax,
                  u32 fs_gs, u32 ds_es, u32 ulInt, u32 ulCode,
                  u32 eip, u32 cs, u32 eflags, u32 esp, u32 ss)
{
  u32 cr0, cr2, cr3;
  u16 tr, fs, ds;

  asm volatile ("movl %%cr0, %%eax\n"
                "movl %%eax, %0\n"
                "movl %%cr2, %%eax\n"
                "movl %%eax, %1\n"
                "movl %%cr3, %%eax\n"
                "movl %%eax, %2\n"
                "xorl %%eax, %%eax\n"
                "str  %%ax\n"
                "movw %%ax, %3\n"
                "movw %%fs, %%ax\n"
                "movw %%ax, %4\n"
                "movw %%ds, %%ax\n"
                "movw %%ax, %5\n"
                :"=m" (cr0), "=m" (cr2), "=m" (cr3),
                 "=m" (tr), "=m" (fs), "=m" (ds):);

  if ((cs & 0x3) == 0) {
    /* same priv level: ESP and SS were not pushed onto stack by interrupt transfer */
    asm volatile ("movl %%ss, %0":"=r" (ss));
    esp = _esp;
  }

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
  _printf (" CS=%.4X SS=%.4X DS=%.4X FS=%.4X\n", cs, ss, ds, fs);
  _printf ("CURRENT=0x%X\n", percpu_read (current_task));
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
    crash_debug ("Unhandled exception");

  send_eoi ();
}

static int
user_putchar (int ch, int attribute)
{
  static int x, y;

#ifdef USE_VMX
  uint32 cpu;
  cpu = get_pcpu_id ();
#endif

  if (ch == '\n') {
    x = 0;
    y++;

    if (y > 24) {
#ifdef USE_VMX
      if (shm_screen_initialized) {
        if (shm->virtual_display.cur_screen == cpu) {
          memcpy (pchVideo, pchVideo + 160, 24 * 160);
          memset (pchVideo + (24 * 160), 0, 160);
        }
        memcpy (shm_screen, shm_screen + 160, 24 * 160);
        memset (shm_screen + (24 * 160), 0, 160);
        y = 24;
        shm->virtual_display.cursor[cpu].x = x;
        shm->virtual_display.cursor[cpu].y = y;
      } else {
        memcpy (pchVideo, pchVideo + 160, 24 * 160);
        memset (pchVideo + (24 * 160), 0, 160);
        y = 24;
      }
#else
      memcpy (pchVideo, pchVideo + 160, 24 * 160);
      memset (pchVideo + (24 * 160), 0, 160);
      y = 24;
#endif
    }
    return (int) (unsigned char) ch;
  }

  if (y * 160 + x * 2 >= 0x1000) return ch;

#ifdef USE_VMX
  if (shm_screen_initialized) {
    if (shm->virtual_display.cur_screen == cpu) {
      pchVideo[y * 160 + x * 2] = ch;
      pchVideo[y * 160 + x * 2 + 1] = attribute;
    }
    shm_screen[y * 160 + x * 2] = ch;
    shm_screen[y * 160 + x * 2 + 1] = attribute;
    x++;
    shm->virtual_display.cursor[cpu].x = x;
    shm->virtual_display.cursor[cpu].y = y;
  } else {
    pchVideo[y * 160 + x * 2] = ch;
    pchVideo[y * 160 + x * 2 + 1] = attribute;
    x++;
  }
#else
  pchVideo[y * 160 + x * 2] = ch;
  pchVideo[y * 160 + x * 2 + 1] = attribute;
  x++;
#endif

  if (y * 160 + x * 2 >= 0x1000) return ch;

#ifdef USE_VMX
  if (shm_screen_initialized) {
    if (shm->virtual_display.cur_screen == cpu) {
      pchVideo[y * 160 + x * 2] = ' ';
      pchVideo[y * 160 + x * 2 + 1] = attribute;
      shm_screen[y * 160 + x * 2] = ' ';
      shm_screen[y * 160 + x * 2 + 1] = attribute;
    }
    shm->virtual_display.cursor[cpu].x = x;
    shm->virtual_display.cursor[cpu].y = y;
  } else {
    pchVideo[y * 160 + x * 2] = ' ';
    pchVideo[y * 160 + x * 2 + 1] = attribute;
  }
#else
  pchVideo[y * 160 + x * 2] = ' ';
  pchVideo[y * 160 + x * 2 + 1] = attribute;
#endif

#ifdef USE_VMX
  if (shm_screen_initialized) {
    if (shm->virtual_display.cur_screen == cpu) {
      /* Move cursor */
      outb (0x0E, 0x3D4);           /* CRTC Cursor location high index */
      outb ((y * 80 + x) >> 8, 0x3D5);      /* CRTC Cursor location high data */
      outb (0x0F, 0x3D4);           /* CRTC Cursor location low index */
      outb ((y * 80 + x) & 0xFF, 0x3D5);    /* CRTC Cursor location low data */
    }
  } else {
    /* Move cursor */
    outb (0x0E, 0x3D4);           /* CRTC Cursor location high index */
    outb ((y * 80 + x) >> 8, 0x3D5);      /* CRTC Cursor location high data */
    outb (0x0F, 0x3D4);           /* CRTC Cursor location low index */
    outb ((y * 80 + x) & 0xFF, 0x3D5);    /* CRTC Cursor location low data */
  }
#else
  /* Move cursor */
  outb (0x0E, 0x3D4);           /* CRTC Cursor location high index */
  outb ((y * 80 + x) >> 8, 0x3D5);      /* CRTC Cursor location high data */
  outb (0x0F, 0x3D4);           /* CRTC Cursor location low index */
  outb ((y * 80 + x) & 0xFF, 0x3D5);    /* CRTC Cursor location low data */
#endif

  return (int) (unsigned char) ch;
}

static void
_user_putchar_attr_4 (char c)
{
  user_putchar (c, 4);
}

static void
splash_screen (void)
{
  int _uname (char *);
  u32 _meminfo (u32, u32);
#ifdef USE_VMX
  u32 cpu = get_pcpu_id ();
#else
  u32 free = _meminfo (0, 0);
#endif
  char vers[80];

  _uname (vers);

#ifdef USE_VMX
  fun_printf (_user_putchar_attr_4,
              "**** SeQuest kernel version: %s ***"
              "  //--\\ //-- //---\\ \\\\  \\ //-- //--\\ \\\\---\\ \n",
              vers);
#else
  fun_printf (_user_putchar_attr_4,
              "**** Quest kernel version: %s *****"
              "   //---\\ \\\\  \\ //-- //--\\ \\\\---\\ \n",
              vers);
#endif

#ifdef USE_VMX
  fun_printf (_user_putchar_attr_4,
              "* Copyright Boston University, 2011 *"
              "  \\\\--\\ ||-- ||   | ||  | ||-- \\\\--\\   || \n");
#else
  fun_printf (_user_putchar_attr_4,
              "* Copyright Boston University, 2011 *"
              "   ||   | ||  | ||-- \\\\--\\   || \n");
#endif

#ifdef USE_VMX
  fun_printf (_user_putchar_attr_4,
              "****** Current Output: 0x%.04X *******"
              "  \\\\__/ \\\\__ \\\\__\\_  \\\\_/ \\\\__ \\\\__/   || \n",
              cpu);
#else
  fun_printf (_user_putchar_attr_4,
              "******** 0x%.08X bytes free ******"
              "   \\\\__\\_  \\\\_/ \\\\__ \\\\__/   || \n",
              free);
#endif
}

/* Syscalls */
static u32
syscall_putchar (u32 eax, u32 ebx)
{
  static bool first = TRUE;

#ifdef USE_VMX
  uint32 cpu;
  cpu = get_pcpu_id ();
  if (shm_screen_initialized) {
    if ((shm->virtual_display.cursor[cpu].x == -1) &&
        (shm->virtual_display.cursor[cpu].y == -1)) {
          splash_screen ();
          shm->virtual_display.cursor[cpu].x = 0;
          shm->virtual_display.cursor[cpu].y = 0;
          first = FALSE;
        }
  } else if (first) {
    splash_screen ();
    first = FALSE;
  }
#else
  if (first) { splash_screen (); first = FALSE; }
#endif
  user_putchar (ebx, 7);
  return 0;
}

static u32
syscall_usleep (u32 eax, u32 ebx)
{
  sched_usleep (ebx);
  return ebx;
}

struct syscall {
  u32 (*func) (u32, u32);
};
struct syscall syscall_table[] = {
  { .func = syscall_putchar },
  { .func = syscall_usleep },
};
#define NUM_SYSCALLS (sizeof (syscall_table) / sizeof (struct syscall))

u32
handle_syscall0 (u32 eax, u32 ebx)
{
  u32 res;
  lock_kernel ();
  if (eax < NUM_SYSCALLS)
    res = syscall_table[eax].func (eax, ebx);
  else
    res = 0;
  unlock_kernel ();
  return res;
}


/* Syscall: fork
 *
 * esp argument used to find info about parent's eip and other
 * registers inherited by child
 */
task_id
_fork (uint32 ebp, uint32 *esp)
{

  task_id child_tid;
  void *phys_addr;
  uint32 *virt_addr;
  uint32 priority;
  uint32 eflags, eip, this_esp, this_ebp;

#ifdef DEBUG_SYSCALL
  com1_printf ("_fork (%X, %p)\n", ebp, esp);
#endif
  lock_kernel ();

  /* 
   * This ugly bit of assembly is designed to obtain the value of EIP
   * in the parent and return from the `call 1f' in the child.
   */

  asm volatile ("call 1f\n"
                "movl $0, %0\n"
                "jmp 2f\n"
                "1:\n"
                "movl (%%esp), %0\n"
                "addl $4, %%esp\n"
                "2:\n":"=r" (eip):);

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

  /* Create a new address space cloned from this one */

  phys_addr = get_pdbr ();      /* Parent page dir base address */
  virt_addr = map_virtual_page ((uint32) phys_addr | 3);        /* Temporary virtual address */

  if (virt_addr == NULL)
    panic ("_fork: virt_addr: out of memory");

  pgdir_t parentpgd = { .dir_pa = (frame_t) phys_addr,
                        .dir_va = (pgdir_entry_t *) virt_addr };

  pgdir_t childpgd = clone_page_directory (parentpgd);
  if (childpgd.dir_pa == -1)
    panic ("_fork: clone_page_directory: failed");

  unmap_virtual_page (parentpgd.dir_va);
  unmap_virtual_page (childpgd.dir_va);

  /* Create a child task which is the same as this task except that it will
   * begin running at the program point after `call 1f' in the above inline asm. */

  child_tid =
    duplicate_TSS (ebp, esp, eip, this_ebp, this_esp, eflags, childpgd.dir_pa);

  /* Inherit priority from parent */
  priority = lookup_TSS (child_tid)->priority =
    lookup_TSS (str ())->priority;

  wakeup (child_tid);

  /* --??-- Duplicate any other parent resources as necessary */

  unlock_kernel ();

  return child_tid;       /* Use this index for child ID for now */
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
  char filename_bak[256];
  quest_tss * tss;

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

  /*
   * This is a bug fix. We have to backup the file name before
   * erasing the old address space because it will be used later
   * and will already be gone with the old stack at that time.
   */
  strncpy (filename_bak, filename, 256);
  c = strlen (filename);
  if (c > 31) c = 31;
  tss = lookup_TSS (str ());
  memcpy (tss->name, filename, c);
  tss->name[c] = '\0';

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
#ifdef USE_VMX
    if (plPageDirectory[i] && (i < (PHY_SHARED_MEM_POOL_START >> 22) ||
        i >= ((PHY_SHARED_MEM_POOL_START + SHARED_MEM_POOL_SIZE) >> 22))) {
#else
    if (plPageDirectory[i]) {
#endif
      /* Present in currrent address space */
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
  filesize = vfs_read (filename_bak, (void *) pe, orig_filesize);
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
  //for (i = 0; i < 16; i++)
  //  plPageTable[0x200 + i] = 0xA0000 | (i << 12) | 7;

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
  lock_kernel ();
  //logger_printf ("_open (\"%s\", 0x%x)\n", pathname, flags);
  int res = vfs_dir (pathname);
  unlock_kernel ();
  return res;
}

/* Syscall: read --??-- proess-global file handle */
int
_read (char *pathname, void *buf, int count)
{
  lock_kernel ();
  //logger_printf ("_read (\"%s\", %p, 0x%x)\n", pathname, buf, count);
  int act_len = vfs_dir (pathname);
  int res = vfs_read (pathname, buf, count < act_len ? count : act_len);
  unlock_kernel ();
  return res;
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

#if 0
  static uint64 last = 0;
  uint64 now;
  RDTSC (now);
  if (last) {
    com1_printf ("Time (TSC counts): %llX\n", now - last);
    last = 0;
  } else {
    last = now;
  }
#endif

#if 0
  int i = 0;
  uint32 *paddr = NULL;

  for (i = 0; i < 1024; i++) {
    paddr = get_phys_addr ((void*) (i << 12));
    com1_printf ("0x%X mapped to: 0x%X\n", i << 12, (uint32) paddr);
  }
#endif

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
  uint8 phys_id = get_pcpu_id ();
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

#if 0
    extern void vcpu_dump_stats (void);
    if ((tick & 0x1FF) == 0)
      vcpu_dump_stats ();
#endif

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
#ifdef USE_VMX
    if (shm_initialized) shm->bsp_booted = TRUE;
#endif
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
  quest_tss *ptss;
  task_id waiter;

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

  /* Destroyed current page directory, so everything that happens
   * until the next task switch must work within the current TLB. */

  /* --??-- Need to release TSS used by exiting process. Here, we need a way
     to index GDT based on current PID returned from original fork call.

     NOTE: Here' we shouldn't really release the TSS until the parent has
     been able to check the status of the child... */

  tss = str ();
  ltr (0);

  /* Remove space for tss -- but first we need to construct the linear
     address of where it is in memory from the TSS descriptor */
  ptss = lookup_TSS (tss);

  for (i = 3; i < MAX_FD; i++) {
    if (ptss->fd_table[i].entry) {
      switch (ptss->fd_table[i].type) {
        case FD_TYPE_UDP :
          udp_remove ((struct udp_pcb *) ptss->fd_table[i].entry);
          break;
        case FD_TYPE_TCP :
          if (tcp_close ((struct tcp_pcb *) ptss->fd_table[i].entry) != ERR_OK) {
            logger_printf ("TCP PCB close failed in exit\n");
          }
          break;
        default :
          break;
      }
    }
  }

  /* All tasks waiting for us now belong on the runqueue. */
  while ((waiter = queue_remove_head (&ptss->waitqueue)))
    wakeup (waiter);

  /* Remove quest_tss */
  tss_remove (tss);
  free_quest_tss (ptss);

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
  
  /* PID is self? */
  if (pid == -1) pid = str ();
  ptss = lookup_TSS (pid);

  if (ptss) {
    if (p->sched_priority == -1)        /* Assume window-constrained task */
      ptss->priority = (p->k * p->T) / p->m;
    else
      ptss->priority = p->sched_priority;

    if (p->affinity != -1)
      ptss->sandbox_affinity = p->affinity;

    wakeup (str ());

    schedule ();
    unlock_kernel ();

    return 0;

  } else
    unlock_kernel ();
  /* Destination task does not exist.  Return an error. */
  return -1;
}

#ifdef USE_VMX
extern int
_switch_screen (int dir)
{
  char * vscreen = NULL;
  int i;

  vscreen = map_virtual_page (
      (uint32) shm->virtual_display.screen[shm->virtual_display.cur_screen] | 3);
  memcpy (vscreen, pchVideo, 0x1000);
  unmap_virtual_page (vscreen);

  if ((shm->virtual_display.cur_screen == 0) && dir == 0) {
    spinlock_lock (&(shm->shm_lock));
    shm->virtual_display.cur_screen = shm->num_sandbox - 1;
    spinlock_unlock (&(shm->shm_lock));
    logger_printf ("Switch to virtual output %d\n",
                   shm->virtual_display.cur_screen);
  } else if ((shm->virtual_display.cur_screen == (shm->num_sandbox - 1)) && dir == 1) {
    spinlock_lock (&(shm->shm_lock));
    shm->virtual_display.cur_screen = 0;
    spinlock_unlock (&(shm->shm_lock));
    logger_printf ("Switch to virtual output %d\n",
                   shm->virtual_display.cur_screen);
  } else {
    spinlock_lock (&(shm->shm_lock));
    (dir == 0) ? shm->virtual_display.cur_screen-- :
                 shm->virtual_display.cur_screen++;
    spinlock_unlock (&(shm->shm_lock));
    logger_printf ("Switch to virtual output %d\n",
                   shm->virtual_display.cur_screen);
  }

  vscreen = map_virtual_page (
      (uint32) shm->virtual_display.screen[shm->virtual_display.cur_screen] | 3);
  memcpy (pchVideo, vscreen, 0x1000);
  unmap_virtual_page (vscreen);

  i = shm->virtual_display.cur_screen;
  /* Move cursor */
  outb (0x0E, 0x3D4);                                   /* CRTC Cursor location high index */
  outb ((shm->virtual_display.cursor[i].y * 80 +
         shm->virtual_display.cursor[i].x) >> 8, 0x3D5);/* CRTC Cursor location high data */
  outb (0x0F, 0x3D4);                                   /* CRTC Cursor location low index */
  outb ((shm->virtual_display.cursor[i].y * 80 +
         shm->virtual_display.cursor[i].x) & 0xFF, 0x3D5);/* CRTC Cursor location low data */

  return 0;
}
#endif

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
