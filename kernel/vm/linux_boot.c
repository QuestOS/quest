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
#include "sched/sched.h"
#include "sched/vcpu.h"

#ifdef USE_VMX
#include "vm/shm.h"
#include "vm/spow2.h"
#include "vm/vmx.h"

#ifdef USE_LINUX_SANDBOX
#include "vm/linux_boot.h"
#endif

#endif

#define DEBUG_LINUX_BOOT

#ifdef DEBUG_LINUX_BOOT
#define DLOG(fmt,...) DLOG_PREFIX("Linux boot",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef USE_LINUX_SANDBOX

/*
 * Load Linux initial ramdisk from pathname in filesystem to virtual
 * address load_addr in memory.
 */
int
load_linux_initrd (uint32 * load_addr, char * pathname)
{
  int act_len = 0;
  int eztftp_bulk_read (char *, uint32 *);

  act_len = eztftp_bulk_read (pathname, load_addr);

  if (act_len < 0) {
    DLOG ("Linux initrd load failed!");
    return -1;
  }

  DLOG ("Linux initrd size: 0x%X", act_len);

  return act_len;
}

/*
 * Load Linux kernel bzImage from pathname in filesystem to virtual
 * address load_addr in memory. If there is a ramdisk loaded, also
 * specify the physical address of the ramdisk in memory and its size.
 */
int
load_linux_kernel (uint32 * load_addr, char * pathname, uint32 * initrd_paddr, int initrd_sz)
{
  int act_len = 0;
  linux_setup_header_t * setup_header;
  int eztftp_bulk_read (char *, uint32 *);
  char * command_line = "root=/dev/sda1 pmedia=atahd loglevel=7";

  act_len = eztftp_bulk_read (pathname, load_addr);

  if (act_len < 0) {
    DLOG ("Linux kernel load failed!");
    return -1;
  }
  DLOG ("Linux kernel size: 0x%X", act_len);

  setup_header = (linux_setup_header_t *) (((uint8 *) load_addr) + LINUX_SETUP_HEADER_OFFSET);

  DLOG ("---------------------------");
  DLOG ("| DUMP LINUX SETUP HEADER |");
  DLOG ("---------------------------");
  DLOG (" ");
  DLOG ("setup_sects: 0x%X", setup_header->setup_sects);
  DLOG ("root_flags: 0x%X", setup_header->root_flags);
  DLOG ("syssize: 0x%X", setup_header->syssize);
  DLOG ("ram_size: 0x%X", setup_header->ram_size);
  DLOG ("vid_mode: 0x%X", setup_header->vid_mode);
  DLOG ("root_dev: 0x%X", setup_header->root_dev);
  DLOG ("boot_flag: 0x%X", setup_header->boot_flag);
  DLOG ("jump: 0x%X", setup_header->jump);
  DLOG ("header: 0x%X", setup_header->header);
  DLOG ("version: 0x%X", setup_header->version);
  DLOG ("loadflags: 0x%X", setup_header->loadflags);
  DLOG (" ");

  /* heap_end_ptr = heap_end - 0x200 */
  setup_header->heap_end_ptr = 0x9600;
  /* Set loader type to GRUB */
  setup_header->type_of_loader = 0x7;
  /* Set command line argument to "auto" for now */
  setup_header->cmd_line_ptr = 0xA0000 - strlen (command_line) - 1;
  memcpy ((void *) (setup_header->cmd_line_ptr), command_line, strlen (command_line) + 1);
  /* Print early msg, reload seg registers for 32-bit entry point, and reuse heap */
  setup_header->loadflags = ((setup_header->loadflags) & (~0x60)) | 0x80;
  DLOG ("modified loadflags: 0x%X", setup_header->loadflags);

#ifdef HAS_INITRD
  if (initrd_paddr) {
    /* Set initial ramdisk parameters if it exists */
    setup_header->ramdisk_image = (uint32) initrd_paddr;
    setup_header->ramdisk_size = initrd_sz;
    /* We assume initial ramdisk will not exceed 4MB */
    setup_header->initrd_addr_max = INITRD_LOAD_PADDR + 0x00400000;
  }
#endif

  /* Set kernel start address to LINUX_PHYS_START */
  setup_header->code32_start = LINUX_PHYS_START;

  return act_len;
}

static u32 boot_thread_stack[1024] ALIGNED (0x1000);

#ifdef DEBUG_LINUX_BOOT
static task_id boot_thread_id = 0;
#endif

/* Input parameter for VM-Exit */
static struct _linux_boot_param {
  uint32 kernel_addr;
  int size;
} exit_param;

static void
linux_boot_thread (void)
{
#ifdef DEBUG_LINUX_BOOT
  int cpu;
  cpu = get_pcpu_id ();
#endif
  extern void * vm_exit_input_param;
  uint8_t * src = NULL, * dst = NULL;
  uint32_t pa_dst = 0;
  int i, prot_size, initrd_sz = 0;
  linux_setup_header_t * header = NULL;

  DLOG ("Linux boot thread started in sandbox %d", cpu);

  unlock_kernel ();
  sti ();

  /* TODO: Wait for network initialization here. Other drivers should already be ready. */
  tsc_delay_usec (6000000);
  DLOG ("Loading Linux kernel bzImage...");
  cli ();
  lock_kernel ();

  //uint32 phys_cr3 = (uint32) get_pdbr ();
  //uint32 *virt_pgd = map_virtual_page (phys_cr3 | 3);
  //for (i = 0; i < 0x400; i++) {
  //  com1_printf ("PGD Entry %d: %x\n", i, virt_pgd[i]);
  //}
  //for (;;);

#ifdef HAS_INITRD
  /* Loading initrd for Linux */
  initrd_sz = load_linux_initrd ((uint32 *) INITRD_LOAD_VADDR, LINUX_INITRD_PATH);

  if (initrd_sz == -1) {
    DLOG ("Loading initrd failed.");
    goto finish;
  }

  /* Copy initrd from load address to INITRD_LOAD_PADDR */
  src = (uint8_t *) INITRD_LOAD_VADDR;
  pa_dst = INITRD_LOAD_PADDR;
  /* Number of 4KB pages */
  for (i = 0; i < (initrd_sz >> 12); i++) {
    dst = map_virtual_page (pa_dst | 0x3);
    memcpy ((void *)dst, (const void *)src, 0x1000);
    unmap_virtual_page (dst);
    pa_dst += 0x1000;
    src += 0x1000;
  }
  dst = map_virtual_page (pa_dst | 0x3);
  memcpy ((void *)dst, (const void *)src, initrd_sz & 0xFFF);
  unmap_virtual_page (dst);

  exit_param.size = load_linux_kernel ((uint32 *) LINUX_KERNEL_LOAD_VA, LINUX_BZIMAGE_PATH,
                                       (uint32 *) INITRD_LOAD_PADDR, initrd_sz);
#else
  exit_param.size = load_linux_kernel ((uint32 *) LINUX_KERNEL_LOAD_VA, LINUX_BZIMAGE_PATH,
                                       NULL, initrd_sz);
#endif

  if (exit_param.size == -1) {
    DLOG ("Loading kernel failed.");
    goto finish;
  }
  
  /* Now, trap into monitor and setup Linux VMCS */
  exit_param.kernel_addr = LINUX_KERNEL_LOAD_VA;
  vm_exit_input_param = (void *) &exit_param;
  vm_exit (VM_EXIT_REASON_LINUX_BOOT);

  header = (linux_setup_header_t *) (((uint8 *) LINUX_KERNEL_LOAD_VA) + LINUX_SETUP_HEADER_OFFSET);
  /* Linux protected mode code starts at (setup_sects + 1) * 512 into the bzImage */
  src = (uint8_t *) LINUX_KERNEL_LOAD_VA + (header->setup_sects + 1) * 512;
  /* Copy protected mode Linux code to Guest physical starting from 1MB + sandbox offset */
  pa_dst = LINUX_PHYS_START;
  /* Copy one page at a time because of limited virtual memory space */
  prot_size = exit_param.size - (header->setup_sects + 1) * 512;
  /* Number of 4KB pages */
  for (i = 0; i < (prot_size >> 12); i++) {
    dst = map_virtual_page (pa_dst | 0x3);
    memcpy ((void *)dst, (const void *)src, 0x1000);
    unmap_virtual_page (dst);
    pa_dst += 0x1000;
    src += 0x1000;
  }
  dst = map_virtual_page (pa_dst | 0x3);
  memcpy ((void *)dst, (const void *)src, prot_size & 0xFFF);
  unmap_virtual_page (dst);

  /* Set LAPIC to lowest prio to prevent interrupt broadcasting */
  LAPIC_set_task_priority(0xF0);

//  uint8_t binary[] = {0xc6, 0x05, 0x00, 0x80, 0x0b, 0x00, 0x41};
//  dst = map_virtual_page (0x100000 | 0x3);
//  memcpy ((void *)dst, (const void *)binary, sizeof(binary));
//  unmap_virtual_page (dst);

  /* Jump to realmodestub in boot.S to initialise Linux booting */
  asm volatile ("jmp 0x8000");
  unlock_kernel ();
  sti ();

  finish:
  exit_kernel_thread ();
}

bool
linux_boot_thread_init (void)
{
#ifdef DEBUG_LINUX_BOOT
  boot_thread_id =
#endif
  start_kernel_thread ((uint32) linux_boot_thread,
                       (uint32) &boot_thread_stack, "Linux boot thread");
  return TRUE;
}

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
