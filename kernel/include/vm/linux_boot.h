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

#ifndef _LINUX_BOOT_H_
#define _LINUX_BOOT_H_

#ifdef USE_VMX
#include "vm/spow2.h"
#include "vm/ept.h"
#endif

//#define HAS_INITRD

#ifdef HAS_INITRD
#define LINUX_INITRD_PATH     "/boot/initrd.gz"
#endif

#define LINUX_BZIMAGE_PATH    "/boot/vmlinuz"

/* 
 * Size of virtual memory pool reserved for initial kernel load.
 * Size should be multiple of 4MB. This also includes the space reserved
 * for ramdisk if there is one. If a kernel image with a built in initrd
 * needs to be loaded, this size should match the size of the image and
 * probably be big.
 *
 * e.g. If kernel bzImage + initrd = 5.8 MB, LINUX_KERNEL_LOAD_SIZE should
 * be set to 0x00800000 (8MB). If kernel bzImage = 125 MB with built in
 * initrd, LINUX_KERNEL_LOAD_SIZE should be set to at least 0x08000000 (128MB).
 */
#define LINUX_KERNEL_LOAD_SIZE       0x10000000

/* Number of 4MB pages of the value above */
#define LINUX_KERNEL_LOAD_PAGE       (LINUX_KERNEL_LOAD_SIZE >> 22)

/* Start virtual address of the kernel load memory region */
#define LINUX_KERNEL_LOAD_VA    \
    (PHY_SHARED_MEM_POOL_START - LINUX_KERNEL_LOAD_SIZE)

#ifdef HAS_INITRD
/* 
 * This is the offset into LINUX_KERNEL_LOAD_VA where ramdisk should
 * be loaded initially. It depends on the size of kernel bzImage.
 *
 * e.g. If bzImage is less than 4MB, INITRD_LOAD_OFFSET could be 0x00400000
 * If bzImage is around 120MB with built in initrd, 0x08000000 should do.
 */
#define INITRD_LOAD_OFFSET           0x08000000

/* Start virtual address of the ramdisk load memory region */
/* By default, this is 4MB after kernel load address */
#define INITRD_LOAD_VADDR            (LINUX_KERNEL_LOAD_VA + INITRD_LOAD_OFFSET)
#endif

#define LINUX_SETUP_HEADER_OFFSET    0x01F1

#define RAMDISK_IMAGE_START_MASK     0x07FF
#define RAMDISK_PROMPT_FLAG          0x8000
#define RAMDISK_LOAD_FLAG            0x4000

#define LOADED_HIGH                  (1<<0)
#define QUIET_FLAG                   (1<<5)
#define KEEP_SEGMENTS                (1<<6)
#define CAN_USE_HEAP                 (1<<7)

#ifndef __ASSEMBLER__

/* LINUX_SANDBOX is only valid after smp_init () */
extern uint32 mp_num_cpus;
#define LINUX_SANDBOX   (mp_num_cpus - 1)
#define LINUX_PHYS_START  0x100000 //(SANDBOX_KERN_OFFSET * (LINUX_SANDBOX + 1)) 

#ifdef HAS_INITRD
/* Physical load address of Linux initial ramdisk */
/* We need to pick an address that does not overlap with Linux kernel */
#define INITRD_LOAD_PADDR             (LINUX_PHYS_START + INITRD_LOAD_OFFSET)
#endif

/* Setup header version 2.10 */
typedef struct _linux_setup_header {
  uint8   setup_sects;
  uint16  root_flags;
  uint32  syssize;
  uint16  ram_size;
  uint16  vid_mode;
  uint16  root_dev;
  uint16  boot_flag;
  uint16  jump;
  uint32  header;
  uint16  version;
  uint32  realmode_swtch;
  uint16  start_sys;
  uint16  kernel_version;
  uint8	  type_of_loader;
  uint8	  loadflags;
  uint16  setup_move_size;
  uint32  code32_start;
  uint32  ramdisk_image;
  uint32  ramdisk_size;
  uint32  bootsect_kludge;
  uint16  heap_end_ptr;
  uint8	  ext_loader_ver;
  uint8	  ext_loader_type;
  uint32  cmd_line_ptr;
  uint32  initrd_addr_max;
  uint32  kernel_alignment;
  uint8	  relocatable_kernel;
  uint8   min_alignment;
  uint8	  _pad2[2];
  uint32  cmdline_size;
  uint32  hardware_subarch;
  uint64  hardware_subarch_data;
  uint32  payload_offset;
  uint32  payload_length;
  uint64  setup_data;
  uint64  pref_address;
  uint32  init_size;
} PACKED linux_setup_header_t;

extern int load_linux_initrd (uint32 *, char *);
extern int load_linux_kernel (uint32 *, char *, uint32 *, int);
extern bool linux_boot_thread_init (void);

#endif /* __ASSEMBLER__ */

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
