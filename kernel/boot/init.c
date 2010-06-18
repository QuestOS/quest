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

#include "boot/multiboot.h"
#include "arch/i386.h"
#include "util/cpuid.h"
#include "kernel.h"
#include "fs/filesys.h"
#include "smp/smp.h"
#include "mem/mem.h"
#include "drivers/ata/ata.h"
#include "drivers/pci/pci.h"
#include "drivers/net/pcnet.h"
#include "util/printf.h"
#include "util/screen.h"
#include "util/debug.h"
#include "drivers/input/keyboard.h"

extern descriptor idt[];

extern uint32 _readwrite_pages, _readonly_pages, _bootstrap_pages;
extern uint32 _kernelstart, _physicalkernelstart;

/* initial C stack */
uint32 stack[1024] __attribute__ ((aligned (0x1000)));

extern void init_sound (void);

/* We use this function to create a dummy TSS so that when we issue a
   switch_to/jmp_gate e.g. at the end of init() or __exit(), we have a
   valid previous TSS for the processor to store state info */
static uint16
alloc_dummy_TSS (void)
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
  ad[i].pBase0 = (uint32) & dummyTSS & 0xFFFF;
  ad[i].pBase1 = ((uint32) & dummyTSS >> 16) & 0xFF;
  ad[i].pBase2 = (uint32) & dummyTSS >> 24;
  ad[i].uType = 0x09;
  ad[i].uDPL = 0;               /* Only let kernel perform task-switching */
  ad[i].fPresent = 1;
  ad[i].f0 = 0;
  ad[i].fX = 0;
  ad[i].fGranularity = 0;       /* Set granularity of tss in bytes */

  return i << 3;

}

static uint16
alloc_idle_TSS (int cpu_num)
{
  int i;
  descriptor *ad = (descriptor *)KERN_GDT;
  tss *pTSS = (tss *) (&idleTSS[cpu_num]);
  void idle_task (void);

  /* Search 2KB GDT for first free entry */
  for (i = 1; i < 256; i++)
    if (!(ad[i].fPresent))
      break;

  if (i == 256)
    panic ("No free selector for TSS");

  ad[i].uLimit0 = sizeof (idleTSS[cpu_num]) - 1;
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

  pTSS->pCR3 = get_pdbr ();
  pTSS->ulEIP = (uint32) & idle_task;

  pTSS->ulEFlags = F_1 | F_IOPL0;

  pTSS->ulESP =
    (uint32) map_virtual_page (alloc_phys_frame () | 3) + 0x1000;
  pTSS->ulEBP = pTSS->ulESP;
  pTSS->usCS = 0x08;
  pTSS->usES = 0x10;
  pTSS->usSS = 0x10;
  pTSS->usDS = 0x10;
  pTSS->usFS = 0x10;
  pTSS->usGS = 0x10;
  pTSS->usIOMap = 0xFFFF;
  /***********************************************
   * pTSS->usSS0 = 0x10;                         *
   * pTSS->ulESP0 = (uint32)KERN_STK + 0x1000; *
   ***********************************************/

  /* Return the index into the GDT for the segment */
  return i << 3;
}



/* Allocate a basic TSS */
static uint16
alloc_TSS (void *pPageDirectory, void *pEntry, int mod_num)
{

  int i;
  descriptor *ad = (idt + 256); /* Get address of GDT from IDT address */
  tss *pTSS = (tss *) ul_tss[mod_num];

  /* Search 2KB GDT for first free entry */
  for (i = 1; i < 256; i++)
    if (!(ad[i].fPresent))
      break;

  if (i == 256)
    panic ("No free selector for TSS");

  ad[i].uLimit0 = sizeof (ul_tss[mod_num]) - 1;
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

  pTSS->pCR3 = pPageDirectory;
  pTSS->ulEIP = (uint32) pEntry;

  if (mod_num != 1)
    pTSS->ulEFlags = F_1 | F_IF | F_IOPL0;
  else
    pTSS->ulEFlags = F_1 | F_IF | F_IOPL;       /* Give terminal server access to
                                                 * screen memory */

  pTSS->ulESP = 0x400000 - 100;
  pTSS->ulEBP = 0x400000 - 100;
  pTSS->usES = 0x23;
  pTSS->usCS = 0x1B;
  pTSS->usSS = 0x23;
  pTSS->usDS = 0x23;
  pTSS->usFS = 0x23;
  pTSS->usGS = 0x23;
  pTSS->usIOMap = 0xFFFF;
  pTSS->usSS0 = 0x10;
  pTSS->ulESP0 = (uint32) KERN_STK + 0x1000;

  /* Return the index into the GDT for the segment */
  return i << 3;
}


/* Create an address space for boot modules */
static uint16
load_module (multiboot_module * pmm, int mod_num)
{

  uint32 *plPageDirectory = get_phys_addr (pg_dir[mod_num]);
  uint32 *plPageTable = get_phys_addr (pg_table[mod_num]);
  void *pStack = get_phys_addr (ul_stack[mod_num]);
  Elf32_Ehdr *pe = pmm->pe;
  Elf32_Phdr *pph = (void *) pmm->pe + pe->e_phoff;
  void *pEntry = (void *) pe->e_entry;
  int i, c, j;
  uint32 *stack_virt_addr;

  /* Populate ring 3 page directory with kernel mappings */
  memcpy (&plPageDirectory[1023], (void *) (((uint32) get_pdbr ()) + 4092),
          4);
  /* LAPIC/IOAPIC mappings */
  memcpy (&plPageDirectory[1019], (void *) (((uint32) get_pdbr ()) + 4076),
          4);

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
      memset ((void *) pmm->pe + pph->p_offset + pph->p_filesz,
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

  /* --??-- 
   *
   * Special case for tss[1] acting, for now, as our screen/terminal server:
   * Need to map screen memory. Here we map it in the middle of our 
   * page table
   *
   */
  if (mod_num == 1)
    plPageTable[512] = (uint32) 0x000B8000 | 7;

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

  return alloc_TSS (plPageDirectory, pEntry, mod_num);
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
  outb (0, PORT1 + 1);          /* Turn off interrupts - Port1 */

  /*         PORT 1 - Communication Settings         */

  outb (0x80, PORT1 + 3);       /* SET DLAB ON */
  outb (0x03, PORT1 + 0);       /* Set Baud rate - Divisor Latch Low Byte */
  /* Default 0x03 =  38,400 BPS */
  /*         0x01 = 115,200 BPS */
  /*         0x02 =  57,600 BPS */
  /*         0x06 =  19,200 BPS */
  /*         0x0C =   9,600 BPS */
  /*         0x18 =   4,800 BPS */
  /*         0x30 =   2,400 BPS */
  outb (0x00, PORT1 + 1);       /* Set Baud rate - Divisor Latch High Byte */
  outb (0x03, PORT1 + 3);       /* 8 Bits, No Parity, 1 Stop Bit */
  outb (0xC7, PORT1 + 2);       /* FIFO Control Register */
  outb (0x0B, PORT1 + 4);       /* Turn on DTR, RTS, and OUT2 */
  com1_puts ("COM1 initialized.\n");
}

void
init (multiboot * pmb)
{

  int i, j, k, c, num_cpus;
  uint16 tss[NR_MODS];
  memory_map_t *mmap;
  uint32 limit, boot_device = 0;
  Elf32_Phdr *pph;
  Elf32_Ehdr *pe;
  char brandstring[I386_CPUID_BRAND_STRING_LENGTH];

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
  print ("CPUID reports: ");
  print (brandstring);
  putchar ('\n');
  if (cpuid_vmx_support ())
    print ("VMX support detected\n");

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

    pe = pmb->mods_addr[i].pe;

    pph = (void *) pe + pe->e_phoff;

    for (j = 0; j < pe->e_phnum; j++) {

      if (pph->p_type == PT_LOAD) {
        c = (pph->p_filesz + 0xFFF) >> 12;      /* #pages required for module */

        for (k = 0; k < c; k++)
          BITMAP_CLR (mm_table, (((uint32) pe + pph->p_offset) >> 12) + k);
      }
      pph = (void *) pph + pe->e_phentsize;
    }
  }

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

  /* Start up other processors, which may allocate pages for stacks */
  num_cpus = smp_init ();
  if (num_cpus > 1) {
    print ("Multi-processing detected.  Number of CPUs: ");
    putx (num_cpus);
    putchar ('\n');
  } else {
    print ("Uni-processor mode.\n");
  }

  if (!pmb->mods_count)
    panic ("No modules available");

  for (i = 0; i < pmb->mods_count; i++) {
    tss[i] = load_module (pmb->mods_addr + i, i);
    lookup_TSS (tss[i])->priority = MIN_PRIO;
  }

  /* Remove identity mapping of first 4MB */
  *((uint32 *)get_pdbr()) = 0;
  flush_tlb_all ();

  pow2_init ();                 /* initialize power-of-2 memory allocator */

  /* Dummy TSS used when CPU needs somewhere to write scratch values */
  dummyTSS_selector = alloc_dummy_TSS ();

  /* Create IDLE tasks */
  for (i = 0; i < num_cpus; i++)
    idleTSS_selector[i] = alloc_idle_TSS (i);

  /* Load the dummy TSS so that when the CPU executes jmp_gate it has
   * a place to write the state of the CPU -- even though we don't
   * care about the state and it will be discarded. */
  ltr (dummyTSS_selector);

  /* The APs do not begin actually operating until the PIT fires the
   * first IRQ after interrupts are re-enabled.  That's why it is safe
   * to utilize the dummy TSS without locking the kernel yet. */
  smp_secondary_init ();

  /* Initialize interrupt-driven keyboard driver */
  init_keyboard_8042 ();

  /* Initialize PCI */
  pci_init ();

  /* Initialize LWIP/network subsystem */
  net_init ();

  /* Initialize PCnet card (depends on network) */
  pcnet_init ();

  /* Initialize e1000 card (depends on network) */
  { bool e1000_init (void); e1000_init (); }

  /* Initialize e1000e card (depends on network) */
  { bool e1000e_init (void); e1000e_init (); }

  /* Initialize USB hub driver */
  { bool usb_hub_driver_init (void); usb_hub_driver_init (); }

  /* Initialize USB mass storage driver */
  { bool usb_mass_storage_driver_init (void); usb_mass_storage_driver_init (); }

  /* Initialize USB */
  { bool uhci_init (void); uhci_init (); }

  /* hard-code the configuration for now */
  net_set_default ("en0");
  net_dhcp_start ("en0");

  /* Initialize ATA/ATAPI subsystem */
  ata_init ();

#if 0
  if (boot_device == 0x8000FFFF && pata_drives[0].ata_type == ATA_TYPE_PATA) {
    printf ("ROOT: EXT2FS\n");
    /* Mount root filesystem */
    if (!ext2fs_mount ())
      panic ("Filesystem mount failed");
    vfs_set_root (VFS_FSYS_EXT2, &pata_drives[0]);
  } else {
    /* CD-ROM boot, figure out which drive (assume first) */
    for (i = 0; i < 4; i++) {
      if (pata_drives[i].ata_type == ATA_TYPE_PATAPI) {
        printf ("ROOT: ISO9660\n");
        if (!eziso_mount (pata_drives[i].ata_bus, pata_drives[i].ata_drive))
          panic ("Filesystem mount failed");
        vfs_set_root (VFS_FSYS_EZISO, &pata_drives[i]);
        break;
      }
    }
    if (i == 4)
      printf ("Unsupported boot device=%X.\n", boot_device);
  }
#else
  /* hack */
  printf ("ROOT: USB\n");
  vfat_mount ();
  vfs_set_root (VFS_FSYS_EZUSB, NULL);
#endif

  /* Initialise soundcard, if one exists */
  init_sound ();

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

  /* The Shell module is in userspace and therefore interrupts will be
   * enabled after this point.  Then, kernel locking will become
   * necessary. */
  jmp_gate (tss[0]);            /* task-switch to shell module */

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
