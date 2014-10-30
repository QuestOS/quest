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
#include "drivers/serial/serial.h"
#include "sched/sched.h"
#include "sched/proc.h"
#include "fs/ram_romfs.h"
#ifdef USE_VMX
#include "vm/ept.h"
#include "vm/shm.h"
#include "drivers/net/ethernet.h"
#include "vm/spow2.h"
#endif
#ifdef USE_LINUX_SANDBOX
#include "vm/linux_boot.h"
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
  /* Still have one to one mapping of first 4 megabytes some of the
     below code relies on this */
  uint32 *plPageDirectory = get_phys_addr (pg_dir[mod_num]);
  uint32 *plPageTable = get_phys_addr (pg_table[mod_num]);
  uint32_t* plStackPageTable = get_phys_addr (uls_pg_table[mod_num]);
  void *pStack = get_phys_addr (ul_stack[mod_num]);
  /* temporarily map pmm->pe in order to read pph->p_memsz */
  Elf32_Ehdr *pe, *pe0 = map_virtual_page ((uint) pmm->pe | 3);

  if (!pe0) {
    com1_printf ("map_virtual_page for pe0 failed in load_module()!\n");
    com1_printf ("Phys Addr=0x%X\n", (uint32_t) pmm->pe);
    panic ("load_module failed!");
  }

  Elf32_Phdr *pph = (void *) pe0 + pe0->e_phoff;
  void *pEntry = (void *) pe0->e_entry;
  int i, c, j;
  uint32 *stack_virt_addr;
  uint32 page_count = 0;
  int pg_dir_index, pg_tbl_index;
  uint32 tmp_pf = 0;
  uint32 * tmp_vf = NULL;

  /* find out how many pages for the module */
  for (i = 0; i < pe0->e_phnum; i++) {
    if (pph->p_type == PT_LOAD)
      page_count += ((pph->p_offset + pph->p_memsz - 1) >> 12) - (pph->p_offset >> 12) + 1;
    pph = (void *) pph + pe0->e_phentsize;
  }

  /* now map the entire module */
  pe = map_contiguous_virtual_pages ((uint) pmm->pe | 3, page_count);

  if (!pe) {
    com1_printf ("map_contiguous_virtual_pages failed in load_module()!\n");
    com1_printf ("Addr=0x%X, Page Count=%d\n", (uint32_t) pmm->pe, page_count);
    panic ("load_module failed!");
  }

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
  map_malloc_paging_structures((pgdir_entry_t*)plPageDirectory, 0);
  map_dma_page_tables((pgdir_entry_t*)plPageDirectory, 0);

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

      /* zero remainder of final page */
      memset ((void *) pe + pph->p_offset + pph->p_filesz,
              0, (pph->p_memsz - pph->p_filesz) & 0x0FFF);

      for (j = 0; j < c; j++) {
        tmp_pf = alloc_phys_frame ();
        tmp_vf = map_virtual_page (tmp_pf | 0x3);
        memcpy (tmp_vf, (void *) (((uint32) pe + pph->p_offset) & 0xFFFFF000) + (j << 12),
                0x1000);
        plPageTable[((uint32) pph->p_vaddr >> 12) + j] = (tmp_pf & 0xFFFFF000) | 0x7;
        unmap_virtual_page (tmp_vf);
      }

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
  get_pg_dir_and_table_indices ((void *) (USER_STACK_START - 1), &pg_dir_index, &pg_tbl_index);
  if(pg_dir_index != 0) {
    /* The stack resides in a different page table than the rest
       of the program */
    plPageDirectory[pg_dir_index] = (uint32) plStackPageTable | 7;
    plStackPageTable[pg_tbl_index] = (uint32) pStack | 7;
  }
  else {
    plPageTable[pg_tbl_index] = (uint32) pStack | 7;
  }

  stack_virt_addr = map_virtual_page ((uint32) pStack | 3);

  /* This sets up the module's stack with command-line args from grub */
  memcpy ((char *) stack_virt_addr + 0x1000 - 80, pmm->string, 80);
  *(uint32 *) ((char *) stack_virt_addr + 0x1000 - 84) = 0;   /* argv[1] */
  *(uint32 *) ((char *) stack_virt_addr + 0x1000 - 88) = USER_STACK_START - 80;       /* argv[0] */
  *(uint32 *) ((char *) stack_virt_addr + 0x1000 - 92) = USER_STACK_START - 88;       /* argv */
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
  //com1_puts ("COM1 initialized.\n");
}

char* get_cmdline_arg(char* cmdline, char* key)
{
  char *p = cmdline;
  uint i, cmdline_len, key_len;
  cmdline_len = strlen(cmdline);
  key_len = strlen(key);
  if(!key_len) return NULL;
  
  for(i = 0; i < cmdline_len - key_len; ++i, ++p) {
    if(!strncmp(p, key, key_len) && p[key_len] == '=') {
      return &p[key_len+1];
    }
  }
  return NULL;
}

int
parse_root_type(char *cmdline)
{
  char* arg = get_cmdline_arg(cmdline, "root");
  if(arg) {
    if(!strncmp(arg, "(hd)", 4)) return VFS_FSYS_EZEXT2;
    if(!strncmp(arg, "(cd)", 4)) return VFS_FSYS_EZISO;
    if(!strncmp(arg, "(tftp)", 6)) return VFS_FSYS_EZTFTP;
    if(!strncmp(arg, "(usb)", 5)) return VFS_FSYS_EZUSB;
    if(!strncmp(arg, "(ram)", 5)) return VFS_FSYS_EZRAM;
  }
  return VFS_FSYS_NONE;
}

int get_ramdisk_index(char *cmdline)
{
  char* arg = get_cmdline_arg(cmdline, "ramdisk");
  if(arg) return atoi(arg);
  return -1;
}


u32 root_type, boot_device=0;
int ramdisk_module_index = -1;
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

/*
 * Manage physical memory region used by GRUB modules.
 *
 * If op is MM_MODULE_MASK, mm_module will mask all physical memory used
 * by GRUB modules as used in mm_table.
 * If op is MM_MODULE_UNMASK, mm_module will mask all physical memory
 * used by GRUB modules as free in mm_table.
 *
 * This function is only used during initialisation where Quest relocates
 * GRUB modules. Before relocation, we mask all modules. After relocation,
 * we release them.
 */
#define MM_MODULE_MASK    0x0
#define MM_MODULE_UNMASK  0x1

/*
 * --YL-- We need to revisit the early memory management code. The current
 *  scheme is obviously flawed. For now, let's mark all the memory occupied
 *  by module to be used. When GRUB load modules in low memory, we were
 *  definitely overwriting the module image!
 */
static void
mm_module (multiboot * pmb, int op)
{
  int i, j;
#ifdef USE_VMX
  uint32 limit = 256 + (SANDBOX_KERN_OFFSET >> 12);
#else
  uint32 limit = 0xFFFFF;
#endif

#ifdef GRUB_LOAD_BZIMAGE
  for (i = 0; i < pmb->mods_count - 1; i++) {
#else
  for (i = 0; i < pmb->mods_count; i++) {
#endif

    for (j = (((uint32_t) pmb->mods_addr[i].pe) >> 12);
         j < (((uint32_t) pmb->mods_addr[i].mod_end) >> 12); j++) {
      if (op == MM_MODULE_MASK) {
        if (j < limit) BITMAP_CLR (mm_table, j);
      } else if (op == MM_MODULE_UNMASK) {
        if (j < limit) BITMAP_SET (mm_table, j);
      }
    }

#if 0
    Elf32_Phdr *pph;
    Elf32_Ehdr *pe;
    int k, c;

    pe = map_virtual_page ((uint32)pmb->mods_addr[i].pe | 3);

    pph = (void *) pe + pe->e_phoff;

    for (j = 0; j < pe->e_phnum; j++) {

      if (pph->p_type == PT_LOAD) {
        c = (pph->p_filesz + 0xFFF) >> 12;      /* #pages required for module */

        if (op == MM_MODULE_MASK) {
          for (k = 0; k < c; k++) {
            if (((((uint32) pe + pph->p_offset) >> 12) + k) < limit)
              BITMAP_CLR (mm_table, (((uint32) pe + pph->p_offset) >> 12) + k);
          }
        } else if (op == MM_MODULE_UNMASK) {
          for (k = 0; k < c; k++) {
            if (((((uint32) pe + pph->p_offset) >> 12) + k) < limit)
              BITMAP_SET (mm_table, (((uint32) pe + pph->p_offset) >> 12) + k);
          }
        }
      }
      pph = (void *) pph + pe->e_phentsize;
    }

    unmap_virtual_page (pe);
#endif
  }

  /* Handling Linux kernel bzImage module */
#ifdef GRUB_LOAD_BZIMAGE
  uint32 start = 0, end = 0;
  start = (uint32)pmb->mods_addr[pmb->mods_count - 1].pe;
  end = (uint32)pmb->mods_addr[pmb->mods_count - 1].mod_end;

  for (i = (start >> 12); i < (((end + 0xFFF) >> 12)); i++) {
    if (op == MM_MODULE_MASK) {
      if (i < limit) BITMAP_CLR (mm_table, i);
    } else if (op == MM_MODULE_UNMASK) {
      if (i < limit) BITMAP_SET (mm_table, i);
    }
  }
#endif
}

void
init (multiboot * pmb)
{
  int i, num_cpus;
  memory_map_t *mmap;
  uint32 limit;
  char brandstring[I386_CPUID_BRAND_STRING_LENGTH];
  uint16 tss[NR_MODS];

  /* Initialize Bochs I/O debugging */
  outw (0x8A00, 0x8A00);

#ifdef SERIAL_MMIO32
  initialize_serial_mmio32 ();
#else
  initialize_serial_port ();
#endif

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
  if(root_type == VFS_FSYS_EZRAM) {
    ramdisk_module_index = get_ramdisk_index(pmb->cmdline);
    if( (ramdisk_module_index < 0) || (ramdisk_module_index > (pmb->mods_count - 1)) ) {
      com1_printf("Failed to find valid ramdisk index\n");
      panic("Failed to find valid ramdisk index");
    }
  }
  

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
  mm_module (pmb, MM_MODULE_MASK);

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

  /* initialize kmalloc page tables */
  if(!init_malloc_pool_dir_entries()) {
    com1_printf("Failed to initialise malloc pool");
    panic("Failed to initialise malloc pool");
  }
  
  init_malloc ();                 /* initialize kmalloc memory pool */

  if(!init_dma_pool_page_tables()) {
    com1_printf("Failed to initialise dma pool");
    panic("Failed to initialise dma pool");
  }

  /* Initialise the floating-point unit (FPU) */
#ifdef NO_FPU
  /* Do nothing */
#else
  initialise_fpu_and_mmx();
#endif
  
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
#ifdef GRUB_LOAD_BZIMAGE
  /* If there is only one module, it has to be shell */
  if (pmb->mods_count < 2)
    panic ("GRUB_LOAD_BZIMAGE configured but no bzImage provided!");
#endif
  for (i = 0; i < pmb->mods_count; i++) {
#ifdef GRUB_LOAD_BZIMAGE
    /*
     * If GRUB is used to load Linux kernel bzImage, it *MUST* be placed
     * at the *BOTTOM* of the module list in GRUB configuration file.
     */
    if (i == (pmb->mods_count - 1)) {
      /* Relocate Linux kernel bzImage loaded by GRUB */
      /* LINUX_KERNEL_LOAD_VA was mapped 1 to 1 in boot.S */
      com1_printf ("Relocating Linux bzImage...\n");
      grub_load_linux_bzImage (pmb->mods_addr + i, (uint32 *) LINUX_KERNEL_LOAD_VA);
    } else {
      if(i == ramdisk_module_index) {
        if(!copy_ramdisk_module(pmb->mods_addr + i, i)) {
          com1_printf("Failed to relocate RAM disk\n");
          panic("Failed to relocate RAM disk");
        }
      }
      else {
        tss[i] = load_module (pmb->mods_addr + i, i);
        lookup_TSS (tss[i])->priority = MIN_PRIO;
      }
    }
#else
    if(i == ramdisk_module_index) {
      if(!copy_ramdisk_module(pmb->mods_addr + i, i)) {
        com1_printf("Failed to relocate RAM disk\n");
        panic("Failed to relocate RAM disk");
      }
    }
    else {
      tss[i] = load_module (pmb->mods_addr + i, i);
      lookup_TSS (tss[i])->priority = MIN_PRIO;
    }
#endif
  }

  /* Release physical memory region occupied by modules. They are already relocated. */
  com1_printf ("Releasing module memory...\n");
  mm_module (pmb, MM_MODULE_UNMASK);

#ifdef SERIAL_MMIO32
  com1_printf ("Remapping MMIO32 serial base...\n");
  remap_serial_mmio32 ();
#endif

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

  /* mem_check (); */

#ifdef USE_VMX
#ifdef QUESTV_NO_VMX
  { extern void vmx_vm_fork (uint32); vmx_vm_fork (0); }
#else
  { extern bool vmx_init (void); vmx_init (); }
#endif
#endif

#ifdef USE_VMX
  /* Shared component initialization in Quest-V */
  { extern bool vmx_io_blacklist_init (void); vmx_io_blacklist_init (); }
#endif
  { extern bool init_keyboard_8042 (void); init_keyboard_8042 (); }
  { extern bool pci_init (void); pci_init (); }
  //{ extern bool r8169_init (void); r8169_init (); }
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
    /* Initialise shared memory pools and start any necessary
       communication threads */
  init_shared_memory_pools();
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
  //{ extern bool beacon_thread_init (void); beacon_thread_init (); }

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

#ifdef SERIAL_MMIO32
  /* XXX: a big hack to work around the absense of PIT on Galileo */
  mp_enabled = 1;
#endif

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
