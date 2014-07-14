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

#ifndef _HYPERCALL_H_
#define _HYPERCALL_H_

#ifdef USE_VMX

#include "types.h"
#include "vmx-defs.h"

/* Software VM-Exit reasons */
#define VM_EXIT_REASON_MIGRATION     0x0001  /* Process migration */
#define VM_EXIT_REASON_GET_HPA       0x0002  /* Get HPA for a given GPA */
#define VM_EXIT_REASON_LINUX_BOOT    0x0003  /* Boot Linux sandbox */
#define VM_EXIT_REASON_MASK_SB       0x0004  /* Remove EPT mapping of a sandbox */
#define VM_EXIT_REASON_SET_EPT       0x0005  /* Set page permission in EPT (shared memory) */
#define VM_EXIT_REASON_MAP_EPT       0x0006  /* Map guest physical to machine physical */
#define VM_EXIT_REASON_SHM_ALLOC     0x0007  /* Allocate a machine physical frame from shared memory */

typedef void * vm_exit_param_t;

struct _hypercall_linux_boot_param {
  uint32 kernel_addr;
  int size;
};

struct _hypercall_set_ept_param {
  uint32 phys_frame;
  uint32 count;
  uint8 perm;
};

struct _hypercall_migration_param {
  void * ptss;
  u64 dl;
};

extern struct _hypercall_set_ept_param hypercall_set_ept_param;

/* Input parameter for VM-Exit */
extern struct _hypercall_linux_boot_param hypercall_linux_boot_param;

static inline void
hyper_call (uint32 call_num)
{
  asm volatile ("cpuid"::"a" (0xFFFFFFFF), "c" (call_num));
}

static inline void
hypercall_set_ept (uint32 phys_frame, uint32 count, uint8 perm)
{
  extern void * vm_exit_input_param;
  hypercall_set_ept_param.phys_frame = phys_frame;
  hypercall_set_ept_param.count = count;
  hypercall_set_ept_param.perm = perm;
  vm_exit_input_param = (void *) &hypercall_set_ept_param;
  hyper_call (VM_EXIT_REASON_SET_EPT);
}

static inline void
hypercall_linux_boot (uint32 kernel_addr, int kernel_size)
{
  extern void * vm_exit_input_param;
  hypercall_linux_boot_param.kernel_addr = kernel_addr;
  hypercall_linux_boot_param.size = kernel_size;
  vm_exit_input_param = (vm_exit_param_t) &hypercall_linux_boot_param;
  hyper_call (VM_EXIT_REASON_LINUX_BOOT);
}

static inline uint32
hypercall_get_host_phys_addr (uint32 guest_phys)
{
  extern void * vm_exit_input_param;
  extern void * vm_exit_return_val;
  vm_exit_input_param = (vm_exit_param_t) guest_phys;
  hyper_call (VM_EXIT_REASON_GET_HPA);
  return (uint32) vm_exit_return_val;
}

void vmx_process_hypercall (uint32 status, void * param);

#endif /* USE_VMX */

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
