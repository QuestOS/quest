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

#include "boot/multiboot.h"
#include "arch/i386.h"
#include "arch/i386-percpu.h"
#include "util/cpuid.h"
#include "kernel.h"
#include "fs/filesys.h"
#include "smp/smp.h"
#include "mem/mem.h"
#include "drivers/ata/ata.h"
#include "drivers/pci/pci.h"
#include "util/printf.h"
#include "util/screen.h"
#include "util/debug.h"
#include "util/perfmon.h"
#include "drivers/input/keyboard.h"
#include "sched/sched.h"
#include "sched/proc.h"
#ifdef USE_VMX
#include "vm/ept.h"
#include "vm/shm.h"
#include "drivers/net/ethernet.h"
#include "vm/spow2.h"
#endif

extern descriptor idt[];

extern uint32 _readwrite_pages, _readonly_pages, _bootstrap_pages;
extern uint32 _kernelstart, _physicalkernelstart;

/* initial C stack */
uint32 stack[1024] __attribute__ ((aligned (0x1000)));

/* Each CPU needs a TSS for the SS0, ESP0 fields */
static uint16
alloc_CPU_TSS (tss *tssp)
{
  int i;
  descriptor *ad = (descriptor *)KERN_GDT;

  /* Search 2KB GDT for first free entry */
  for (i = 1; i < 256; i++)
    if (!(ad[i].fPresent))
      break;

  if (i == 256)
    panic ("No free selector for TSS");

  ad[i].uLimit0 = sizeof (tss);
  ad[i].uLimit1 = 0;
  ad[i].pBase0 = (uint32) tssp & 0xFFFF;
  ad[i].pBase1 = ((uint32) tssp >> 16) & 0xFF;
  ad[i].pBase2 = (uint32) tssp >> 24;
  ad[i].uType = 0x09;
  ad[i].uDPL = 0;               /* Only let kernel perform task-switching */
  ad[i].fPresent = 1;
  ad[i].f0 = 0;
  ad[i].fX = 0;
  ad[i].fGranularity = 0;       /* Set granularity of tss in bytes */

  tssp->usSS0 = 0x10;
  tssp->ulESP0 = (uint32) KERN_STK + 0x1000;

  return i << 3;

}

/* Create an address space for boot modules */
static task_id
load_module (multiboot_module * pmm, int mod_num)
{

  uint32 *plPageDirectory = get_phys_addr (pg_dir[mod_num]);
  uint32 *plPageTable = get_phys_addr (pg_table[mod_num]);
  void *pStack = get_phys_addr (ul_stack[mod_num]);
  /* temporarily map pmm->pe in order to read pph->p_memsz */
  Elf32_Ehdr *pe, *pe0 = map_virtual_page ((uint) pmm->pe | 3);
  Elf32_Phdr *pph = (void *) pe0 + pe0->e_phoff;
  void *pEntry = (void *) pe0->e_entry;
  int i, c, j;
  uint32 *stack_virt_addr;
  uint32 page_count = 1;

  /* find out how many pages for the module */
  for (i = 0; i < pe0->e_phnum; i++) {
    if (pph->p_type == PT_LOAD)
      page_count += (pph->p_memsz >> 12);
    pph = (void *) pph + pe0->e_phentsize;
  }

  /* now map the entire module */
  pe = map_contiguous_virtual_pages ((uint) pmm->pe | 3, page_count);

  unmap_virtual_page (pe0);

  pph = (void *) pe + pe->e_phoff;

  /* Populate ring 3 page directory with kernel mappings */
  memcpy (&plPageDirectory[1023], (void *) (((uint32) get_pdbr ()) + 4092),
          4);
  /* LAPIC/IOAPIC mappings */
  memcpy (&plPageDirectory[1019], (void *) (((uint32) get_pdbr ()) + 4076),
          4);
#ifdef USE_VMX
  /* Shared component memory pool */
  for (i = (PHY_SHARED_MEM_POOL_START >> 22);
       i < ((PHY_SHARED_MEM_POOL_START + SHARED_MEM_POOL_SIZE) >> 22); i++) {
    plPageDirectory[i] = ((i << 22) + 0x83);
  }
#endif

  /* Populate ring 3 page directory with entries for its private address
     space */
  plPageDirectory[0] = (uint32) plPageTable | 7;

  plPageDirectory[1022] = (uint32) get_phys_addr (kls_pg_table[mod_num]) | 3;
  kls_pg_table[mod_num][0] = (uint32) get_phys_addr (kl_stack[mod_num]) | 3;

  /* Walk ELF header */
  for (i = 0; i < pe->e_phnum; i++) {

    if (pph->p_type == PT_LOAD) {
      /* map pages loaded from file */
      c = (pph->p_filesz + 0xFFF) >> 12;        /* #pages to load for module */

      for (j = 0; j < c; j++)
        plPageTable[((uint32) pph->p_vaddr >> 12) + j] =
          (uint32) pmm->pe + (pph->p_offset & 0xFFFFF000) + (j << 12) + 7;

      /* zero remainder of final page */
      memset ((void *) pe + pph->p_offset + pph->p_filesz,
              0, (pph->p_memsz - pph->p_filesz) & 0x0FFF);

      /* map additional zeroed pages */
      c = (pph->p_memsz + 0xFFF) >> 12;

      /* Allocate space for bss section.  Use temporary virtual memory for
       * memset call to clear physical frame(s)
       */
      for (; j <= c; j++) {
        uint32 page_frame = (uint32) alloc_phys_frame ();
        void *virt_addr = map_virtual_page (page_frame | 3);
        memset (virt_addr, 0, 0x1000);
        plPageTable[((uint32) pph->p_vaddr >> 12) + j] = page_frame + 7;
        unmap_virtual_page (virt_addr);
      }
    }

    pph = (void *) pph + pe->e_phentsize;
  }

  /* map stack */
  plPageTable[1023] = (uint32) pStack | 7;

  stack_virt_addr = map_virtual_page ((uint32) pStack | 3);

  /* This sets up the module's stack with command-line args from grub */
  memcpy ((char *) stack_virt_addr + 0x1000 - 80, pmm->string, 80);
  *(uint32 *) ((char *) stack_virt_addr + 0x1000 - 84) = 0;   /* argv[1] */
  *(uint32 *) ((char *) stack_virt_addr + 0x1000 - 88) = 0x400000 - 80;       /* argv[0] */
  *(uint32 *) ((char *) stack_virt_addr + 0x1000 - 92) = 0x400000 - 88;       /* argv */
  *(uint32 *) ((char *) stack_virt_addr + 0x1000 - 96) = 1;   /* argc -- hard-coded right now */
  /* Dummy return address placed here for the simulated "call" to our
     library */
  *(uint32 *) ((char *) stack_virt_addr + 0x1000 - 100) = 0;  /* NULL return address -- never used */

  unmap_virtual_page (stack_virt_addr);
  unmap_virtual_pages (pe, page_count);

  task_id tid = alloc_TSS (plPageDirectory, pEntry, mod_num);
  com1_printf ("module %d loaded: task_id=0x%x\n", mod_num, tid);
#if QUEST_SCHED==vcpu
  lookup_TSS (tid)->cpu = 0;
#endif
  return tid;
}


/* Programmable interrupt controller settings */
void
init_pic (void)
{

  /* Remap IRQs to int 0x20-0x2F
   * Need to set initialization command words (ICWs) and
   * operation command words (OCWs) to PIC master/slave
   */

  /* Master PIC */
  outb (0x11, 0x20);            /* 8259 (ICW1) - xxx10x01 */
  outb (PIC1_BASE_IRQ, 0x21);   /* 8259 (ICW2) - set IRQ0... to int 0x20... */
  outb (0x04, 0x21);            /* 8259 (ICW3) - connect IRQ2 to slave 8259 */
  outb (0x0D, 0x21);            /* 8259 (ICW4) - Buffered master, normal EOI, 8086 mode */

  /* Slave PIC */
  outb (0x11, 0xA0);            /* 8259 (ICW1) - xxx10x01 */
  outb (PIC2_BASE_IRQ, 0xA1);   /* 8259 (ICW2) - set IRQ8...to int 0x28... */
  outb (0x02, 0xA1);            /* 8259 (ICW3) - slave ID #2 */
  outb (0x09, 0xA1);            /* 8259 (ICW4) - Buffered slave, normal EOI, 8086 mode */

}


/* Programmable interval timer settings */
void
init_pit (void)
{

  outb (0x34, 0x43);            /* 8254 (control word) - counter 0, mode 2 */

  /* Set interval timer to interrupt once every 1/HZth second */
  outb ((PIT_FREQ / HZ) & 0xFF, 0x40);  /* counter 0 low byte */
  outb ((PIT_FREQ / HZ) >> 8, 0x40);    /* counter 0 high byte */
}

void
initialize_serial_port (void)
{
  outb (0, serial_port1 + 1);          /* Turn off interrupts - Port1 */

  /*         PORT 1 - Communication Settings         */

  outb (0x80, serial_port1 + 3);       /* SET DLAB ON */
  outb (0x03, serial_port1 + 0);       /* Set Baud rate - Divisor Latch Low Byte */
  /* Default 0x03 =  38,400 BPS */
  /*         0x01 = 115,200 BPS */
  /*         0x02 =  57,600 BPS */
  /*         0x06 =  19,200 BPS */
  /*         0x0C =   9,600 BPS */
  /*         0x18 =   4,800 BPS */
  /*         0x30 =   2,400 BPS */
  outb (0x00, serial_port1 + 1);       /* Set Baud rate - Divisor Latch High Byte */
  outb (0x03, serial_port1 + 3);       /* 8 Bits, No Parity, 1 Stop Bit */
  outb (0xC7, serial_port1 + 2);       /* FIFO Control Register */
  outb (0x0B, serial_port1 + 4);       /* Turn on DTR, RTS, and OUT2 */
  com1_puts ("COM1 initialized.\n");
}

int
parse_root_type (char *cmdline)
{
  char *p, *q;
  for (p=cmdline; p[0] && p[1] && p[2] && p[3] && p[4]; p++) {
    if (p[0] == 'r' && p[1] == 'o' && p[2] == 'o' &&
        p[3] == 't' && p[4] == '=') {
      p+=5;
      for (q=p; *q && *q != ' '; q++);
      *q = '\0';
      if (q - p >= 4 &&
          p[0] == '(' && p[1] == 'h' && p[2] == 'd' && p[3] == ')')
        return VFS_FSYS_EZEXT2;
      if (q - p >= 4 &&
          p[0] == '(' && p[1] == 'c' && p[2] == 'd' && p[3] == ')')
        return VFS_FSYS_EZISO;
      if (q - p >= 6 &&
          p[0] == '(' && p[1] == 't' && p[2] == 'f' &&
          p[3] == 't' && p[4] == 'p' && p[5] == ')')
        return VFS_FSYS_EZTFTP;
      if (q - p >= 5 &&
          p[0] == '(' && p[1] == 'u' && p[2] == 's' &&
          p[3] == 'b' && p[4] == ')')
        return VFS_FSYS_EZUSB;
    }
  }
  return VFS_FSYS_NONE;
}

u32 root_type, boot_device=0;
#ifdef USE_VMX
task_id shell_tss = 0;
#endif

void
mem_check (void)
{
  int i = 0;
  int uflag = 0;

  /* Only test first 4GB */
  for (i = 0; i < 0x100000; i++) {
    if (BITMAP_TST (mm_table, i)) {
      /* Free frame */
      if (uflag) {
        com1_printf ("Used frame end: 0x%X\n", (i << 12) - 1);
        uflag = 0;
      }
    } else {
      /* Allocated frame */
      if (uflag == 0) {
        com1_printf ("Used frame begin: 0x%X\n", i << 12);
        uflag = 1;
      }
    }
  }
  if (uflag) {
    com1_printf ("Used frame end: 0x%X\n", (i << 12) - 1);
    uflag = 0;
  }
}

void
init (multiboot * pmb)
{
  int i, j, k, c, num_cpus;
  memory_map_t *mmap;
  uint32 limit;
  Elf32_Phdr *pph;
  Elf32_Ehdr *pe;
  char brandstring[I386_CPUID_BRAND_STRING_LENGTH];
  uint16 tss[NR_MODS];

  /* Initialize Bochs I/O debugging */
  outw (0x8A00, 0x8A00);

  initialize_serial_port ();

  pchVideo = (char *)KERN_SCR;

  /* clear screen */
  for (i = 0; i < 80 * 25; i++) {
    pchVideo[i * 2] = ' ';
    pchVideo[i * 2 + 1] = 7;
  }

  print ("\n\n\n");

  com1_printf ("cmdline: %s\n", pmb->cmdline);
  root_type = parse_root_type (pmb->cmdline);
  com1_printf ("root_type=%d\n", root_type);

  if (pmb->flags & 0x2) {
    multiboot_drive *d;
    boot_device = pmb->boot_device;
    printf ("Boot device: %X\n", boot_device);
    com1_printf ("Boot device: %X\n", boot_device);
    if (pmb->flags & 0x80) {
      for (d = pmb->drives_addr, i = 0;
           i < pmb->drives_length;
           i += d->size, d = (multiboot_drive *) ((uint8 *) d + d->size)) {
        printf ("Found drive. number: %X\n", d->number);
        com1_printf ("Found drive. number: %X\n", d->number);
      }
    }
  }

  cpuid_get_brand_string (brandstring, I386_CPUID_BRAND_STRING_LENGTH);
  printf ("CPUID says: %s\n", brandstring);
  if (!cpuid_msr_support ())
    panic ("Model-specific registers not supported!\n");
  if (!cpuid_tsc_support ())
    panic ("Timestamp counter not supported!");
  if (!cpuid_rdtscp_support ())
    logger_printf ("RDTSCP NOT supported.\n");
  else
    logger_printf ("RDTSCP supported.\n");
  if (cpuid_invariant_tsc_support ()) {
     print ("Invariant TSC support detected\n");
     com1_printf ("Invariant TSC support detected\n");
  }

  for (mmap = (memory_map_t *) pmb->mmap_addr;
       (uint32) mmap < pmb->mmap_addr + pmb->mmap_length;
       mmap = (memory_map_t *) ((uint32) mmap
                                + mmap->size + 4 /*sizeof (mmap->size) */ )) {

    /*
     * Set mm_table bitmap entries to 1 for all pages of RAM that are free.
     */
    if (mmap->type == 1) {      /* Available RAM -- see 'info multiboot' */
      if (mmap->base_addr_high == 0x0) {
        /* restrict to 4GB RAM */
        for (i = 0; i < (mmap->length_low >> 12); i++)
          BITMAP_SET (mm_table, (mmap->base_addr_low >> 12) + i);
        limit = (mmap->base_addr_low >> 12) + i;

        if (limit > mm_limit)
          mm_limit = limit;
      }
    }
  }

#ifdef USE_VMX
  mm_limit = 256 + (SANDBOX_KERN_OFFSET >> 12);
  mm_begin = 0;
#endif

  /*
   * Clear bitmap entries for kernel and bootstrap memory areas,
   * so as not to use them in dynamic memory allocation.
   */
  for (i = 0;
       i <
       (uint32) &_bootstrap_pages + (uint32) &_readwrite_pages +
       (uint32) &_readonly_pages; i++)
    BITMAP_CLR (mm_table, 256 + i);     /* Kernel starts at 256th physical frame
                                         * See quest.ld
                                         */

  /*
   * --??-- Possible optimization is to free mm_table entries corresponding
   * to memory above mm_limit on machines with less physical memory than
   * table keeps track of -- currently 4GB.
   */

  /* Here, clear mm_table entries for any loadable modules. */
  for (i = 0; i < pmb->mods_count; i++) {

    pe = map_virtual_page ((uint32)pmb->mods_addr[i].pe | 3);

    pph = (void *) pe + pe->e_phoff;

    for (j = 0; j < pe->e_phnum; j++) {

      if (pph->p_type == PT_LOAD) {
        c = (pph->p_filesz + 0xFFF) >> 12;      /* #pages required for module */

        for (k = 0; k < c; k++)
          BITMAP_CLR (mm_table, (((uint32) pe + pph->p_offset) >> 12) + k);
      }
      pph = (void *) pph + pe->e_phentsize;
    }

    unmap_virtual_page (pe);
  }

  /* Clear bitmap entries for first megabyte of RAM so we don't
   * overwrite BIOS tables we might be interested in later. */
  for (i=0; i<256; i++)
    BITMAP_CLR (mm_table, i);

  /* Reserve physical frame with GRUB multiboot structure */
  BITMAP_CLR (mm_table, (u32) pmb >> 12);

#if 0
  /* Test that mm_table is setup correct */
  for (i = 0; i < 2000; i++)
    putchar (BITMAP_TST (mm_table, i) ? '1' : '0');
  while (1);
#endif

  /* Now safe to call alloc_phys_frame() as all free/allocated memory is
   *  marked in the mm_table
   */

  init_interrupt_handlers ();

  /* Initialise the programmable interrupt controller (PIC) */
  init_pic ();

  /* Initialise the programmable interval timer (PIT) */
  init_pit ();

  pow2_init ();                 /* initialize power-of-2 memory allocator */

  /* Initialise the floating-point unit (FPU) */
  initialise_fpu();
  
  
  /* Setup per-CPU area for bootstrap CPU */
  percpu_per_cpu_init ();

  /* Start up other processors, which may allocate pages for stacks */
  num_cpus = smp_init ();
  if (num_cpus > 1) {
    print ("Multi-processing detected.  Number of CPUs: ");
    putx (num_cpus);
    putchar ('\n');
  } else {
    print ("Uni-processor mode.\n");
  }

  /* Create CPU and IDLE TSSes */
  for (i = 0; i < num_cpus; i++) {
    cpuTSS_selector[i] = alloc_CPU_TSS (&cpuTSS[i]);
    idleTSS_selector[i] = alloc_idle_TSS (i);
  }

  /* Load modules from GRUB */
  if (!pmb->mods_count)
    panic ("No modules available");
  for (i = 0; i < pmb->mods_count; i++) {
    tss[i] = load_module (pmb->mods_addr + i, i);
    lookup_TSS (tss[i])->priority = MIN_PRIO;
  }

  /* --??-- Assume the first is shell here */
  char * name = "/boot/shell";
  memcpy (lookup_TSS (tss[0])->name, name, strlen (name));
  lookup_TSS (tss[0])->name[strlen(name)] = '\0';

#ifdef USE_VMX
  /* Back up shell module for sandboxes */
  shell_tss = load_module (pmb->mods_addr, NR_MODS - 1);
  lookup_TSS (shell_tss)->priority = MIN_PRIO;
  lookup_TSS (shell_tss)->EFLAGS = F_1 | F_IF | F_IOPL0;
  memcpy (lookup_TSS (shell_tss)->name, name, strlen (name));
  lookup_TSS (shell_tss)->name[strlen(name)] = '\0';
#endif

  /* Remove identity mapping of first 4MB */
  *((uint32 *)get_pdbr()) = 0;
  flush_tlb_all ();

  /* Load the per-CPU TSS for the bootstrap CPU */
  hw_ltr (cpuTSS_selector[0]);

  /* The APs do not begin actually operating until the PIT fires the
   * first IRQ after interrupts are re-enabled.  That's why it is safe
   * to utilize the dummy TSS without locking the kernel yet. */
  smp_secondary_init ();

  mem_check ();

#ifdef USE_VMX
  { extern bool vmx_init (void); vmx_init (); }
#endif

  /* Shared component initialization in Quest-V */
  { extern bool init_keyboard_8042 (void); init_keyboard_8042 (); }
  { extern bool pci_init (void); pci_init (); }
  { extern bool r8169_init (void); r8169_init (); }
  //{ extern bool msgt_mem_init (void); msgt_mem_init (); }

/*
 * --??-- In current boot-strap framework, all the things that need to
 *  be shared across multiple kernels in Quest-V should be initialized
 *  before this point. The private system components can still be
 *  initialized by the module loader on BSP.
 *
 *  We will come up with some convenient module initialization framework
 *  in the future.
 */
#ifdef USE_VMX
  spinlock_lock (&(shm->global_lock));
  print ("Waiting for VM-Forks...\n");
  mp_enabled = 1;
  while (shm->num_sandbox != num_cpus);
  mp_enabled = 0;
  print ("All sandboxes forked\n");
#endif

  /* Load all modules, chasing dependencies */
  { extern bool module_load_all (void); module_load_all (); }
#ifdef USE_VMX
  { extern bool migration_init (void); migration_init (); }
#endif
  //{ extern bool msgt_bandwidth_init (void); msgt_bandwidth_init (); }
  //{ extern bool udp_bandwith_init (void); udp_bandwith_init (); }
  //{ extern bool ipc_send_init (void); ipc_send_init (); }
  //{ extern bool ipc_recv_init (void); ipc_recv_init (); }
  //{ extern bool msgt_init (void); msgt_init (); }
  //{ extern bool hog_thread_init (void); hog_thread_init (); }
  //{
  //  extern bool netsetup_custom_init (struct ip_addr, struct ip_addr, struct ip_addr, int);
  //  struct ip_addr ipaddr, netmask, gw;
  //  IP4_ADDR(&ipaddr, 192, 168, 2, 20);
  //  IP4_ADDR(&netmask, 255, 255, 255, 0);
  //  IP4_ADDR(&gw, 192, 168, 2, 1);
  //  netsetup_custom_init (ipaddr, netmask, gw, 1);
  //}

  /* count free pages for informational purposes */
  u32 *page_table = (u32 *) KERN_PGT;
  u32 free_pages = 0;
  for (i = 0; i < 1024; i++)
    if (page_table[i] == 0)
      free_pages++;
  printf ("free kernel pages=%d\n", free_pages);
  logger_printf ("free kernel pages=%d\n", free_pages);

  /* Start the schedulers */
  smp_enable_scheduling ();

#ifdef ENABLE_GDBSTUB
  {
#ifndef GDBSTUB_TCP
    void set_debug_traps (void);
    set_debug_traps ();
    BREAKPOINT ();
#endif
  }
#endif

#ifdef USE_VMX
  spinlock_unlock (&(shm->global_lock));
#endif

  /* The Shell module is in userspace and therefore interrupts will be
   * enabled after this point.  Then, kernel locking will become
   * necessary. */

  ltr (tss[0]);
  /* task-switch to shell module */
  asm volatile ("jmp _sw_init_user_task"::"D" (lookup_TSS (tss[0])));

  /* never return */
  panic ("BSP: unreachable");
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
