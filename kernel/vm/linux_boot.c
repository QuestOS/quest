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

/*
 * Load Linux kernel bzImage from pathname in filesystem to virtual
 * address load_addr in memory.
 */
int
load_linux_kernel (uint32 * load_addr, char * pathname)
{
#ifdef USE_LINUX_SANDBOX
  int act_len = 0, rlen = 0;
  linux_setup_header_t * setup_header;

  act_len = vfs_dir (pathname);
  DLOG ("Linux kernel size: 0x%X", act_len);

  rlen = vfs_read (pathname, (char *) load_addr, act_len);

  if (rlen < 0) {
    DLOG ("Linux kernel load failed!");
    return rlen;
  }

  setup_header = (linux_setup_header_t *) (((uint8 *) load_addr) + LINUX_SETUP_HEADER_OFFSET);

  DLOG ("Linux kernel loaded (0x%X bytes)", rlen);

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
  DLOG (" ");
#endif

  return 0;
}

static u32 boot_thread_stack[1024] ALIGNED (0x1000);

#ifdef DEBUG_LINUX_BOOT
static task_id boot_thread_id = 0;
#endif

static void
linux_boot_thread (void)
{
#ifdef DEBUG_LINUX_BOOT
  int cpu;
  cpu = get_pcpu_id ();
#endif

  DLOG ("Linux boot thread started in sandbox %d", cpu);

  unlock_kernel ();
  sti ();

  /* TODO: Wait for network initialization here. Other drivers should already be ready. */
  tsc_delay_usec (3000000);
  DLOG ("Loading Linux kernel bzImage...");
  cli ();
  lock_kernel ();
  load_linux_kernel ((uint32 *) LINUX_KERNEL_LOAD_VA, "/boot/vmlinuz");
  unlock_kernel ();
  sti ();

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

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
