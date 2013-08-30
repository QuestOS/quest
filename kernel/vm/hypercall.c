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
#include "arch/i386-mtrr.h"
#include "kernel.h"
#include "vm/ept.h"
#include "vm/vmx.h"
#include "vm/migration.h"
#include "vm/shm.h"
#include "mem/mem.h"
#include "util/debug.h"

#ifdef USE_LINUX_SANDBOX
#include "vm/linux_boot.h"
#endif

#define DEBUG_HYPERCALL

#ifdef DEBUG_HYPERCALL
#define DLOG(fmt,...) DLOG_PREFIX("hypercall",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef USE_VMX

/* --YL-- We use two global variables for input and output of vm_exit routine. These
 * variables should be placed in sandbox kernel memory instead of monitor and some
 * more secure method should be prefered. For now, we just place them in monitor
 * for convenience.
 */
vm_exit_param_t vm_exit_input_param = NULL;
vm_exit_param_t vm_exit_return_val = NULL;

struct _hypercall_set_ept_param hypercall_set_ept_param;

/* Input parameter for VM-Exit */
struct _hypercall_linux_boot_param hypercall_linux_boot_param;

#ifdef USE_LINUX_SANDBOX
static void
linux_vm_init (uint32 kernel_addr, int kernel_size)
{
  int real_mode_size = 0;
  uint32 exec_ctrls2 = 0;
  linux_setup_header_t * setup_header = NULL;
  setup_header = (linux_setup_header_t *) (((uint8 *) kernel_addr) + LINUX_SETUP_HEADER_OFFSET);
  /* TODO:Setup EPT for Linux sandbox */
  /* TODO:Enable Unrestricted Guest */
  real_mode_size = (setup_header->setup_sects + 1) * 512;
  /* Move real mode Linux code to 0x00090000 */
  memcpy ((void *) 0x90000, (const void *) kernel_addr, real_mode_size);
  logger_printf ("%d bytes copied to 0x90000\n", real_mode_size);
  exec_ctrls2 = vmread (VMXENC_PROCBASED_VM_EXEC_CTRLS2);
  vmwrite (exec_ctrls2 | 0x80, VMXENC_PROCBASED_VM_EXEC_CTRLS2);
  exec_ctrls2 = vmread (VMXENC_PROCBASED_VM_EXEC_CTRLS2);
  logger_printf ("exec_ctrls2=0x%X\n", exec_ctrls2);
}
#endif

static void
process_hypercall_set_ept (vm_exit_param_t vm_exit_input_param)
{
  int i = 0;
  struct _hypercall_set_ept_param * param = NULL;

  param = (struct _hypercall_set_ept_param *) vm_exit_input_param;

  if (!param) {
    DLOG ("hypercall_set_ept has NULL parameter");
    return;
  }

  for (i = 0; i < param->count; i++) {
    set_ept_page_permission ((param->phys_frame & 0xFFFFF000) + (i << 12), param->perm);
  }
}

static void
process_hypercall_migration (vm_exit_param_t tm)
{
  static quest_tss * ret_tss = NULL;
  static pgdir_t mdir = {-1, 0}, cdir = {-1, 0};
  static bool new_request = TRUE;  /* Is this a new request or a preempted one? */
  struct _hypercall_migration_param * param = (struct _hypercall_migration_param *) tm;

  if (new_request) {
    /* Begin migration by first pulling the whole address space over */
    ret_tss = pull_quest_tss (param->ptss);
  } else {
    /* We were preempted, directly go to address space clone */
    goto resume_clone;
  }
  if (ret_tss) {
    mdir.dir_pa = ret_tss->CR3;
    mdir.dir_va = map_virtual_page (mdir.dir_pa | 3);
    if (!mdir.dir_va) {
      logger_printf ("map_virtual_page failed in migration!\n");
      vm_exit_return_val = NULL;
      free_quest_tss (ret_tss);
      return;
    }

resume_clone:
    cdir = remote_clone_page_directory (mdir, param->dl);
    if (((uint32) (cdir.dir_va)) == 0xFFFFFFFF) {
      /* remote_clone_page_directory preempted */
      vm_exit_return_val = (void *) 0xFFFFFFFF;
      new_request = FALSE;
      goto abort;
    }
    unmap_virtual_page (mdir.dir_va);
    if (!cdir.dir_va) {
      /* Clone failed */
      logger_printf ("Task 0x%X address space clone failed in migration!\n", ret_tss->tid);
      vm_exit_return_val = NULL;
      /* Clean up ret_tss */
      free_quest_tss (ret_tss);
    } else {
      ret_tss->CR3 = cdir.dir_pa;
      unmap_virtual_page (mdir.dir_va);
      vm_exit_return_val = (void *) ret_tss;
    }
  } else {
    vm_exit_return_val = NULL;
    logger_printf ("pull_quest_tss failed!\n");
  }
  /* Reset all static variables */
  ret_tss = NULL;
  new_request = TRUE;
  mdir.dir_pa = cdir.dir_pa = -1;
  mdir.dir_va = cdir.dir_va = NULL;
abort:
  return;
}

static void
process_hypercall_get_host_phys_addr (vm_exit_param_t gphys)
{
  vm_exit_return_val = (void *) get_host_phys_addr ((uint32) gphys);
  DLOG ("Sandbox%d: Host Phys for 0x%X is 0x%X", get_pcpu_id (),
        (uint32) vm_exit_input_param, (uint32) vm_exit_return_val);
}

/*
 * Processing intentional VM-Exit from Sandboxes.
 */
void
vmx_process_hypercall (uint32 status, void * param)
{
  uint cpu = get_pcpu_id ();
  virtual_machine * vm = (virtual_machine *) param;
  DLOG ("Sandbox%d: performing VM-Exit Status: 0x%X Input: 0x%X",
        cpu, status, vm_exit_input_param);

  switch (status) {
#ifdef USE_LINUX_SANDBOX
    case VM_EXIT_REASON_LINUX_BOOT:
      linux_vm_init (((struct _hypercall_linux_boot_param *) vm_exit_input_param)->kernel_addr,
                     ((struct _hypercall_linux_boot_param *) vm_exit_input_param)->size);
      break;
#endif
    case VM_EXIT_REASON_MIGRATION:
#ifdef USE_VMX
      process_hypercall_migration (vm_exit_input_param);
#endif
      break;
    case VM_EXIT_REASON_GET_HPA:
      process_hypercall_get_host_phys_addr (vm_exit_input_param);
      break;
#ifdef USE_LINUX_SANDBOX
    case VM_EXIT_REASON_MASK_SB:
      mask_sandbox ((uint32) LINUX_SANDBOX);
      break;
#endif
    case VM_EXIT_REASON_SET_EPT:
      process_hypercall_set_ept (vm_exit_input_param);
      break;
    case VM_EXIT_REASON_MAP_EPT:
      logger_printf ("Mapping Guest Physical 0x%X to Machine Physical 0x%X\n",
                     (uint32) vm->guest_regs.eax, (uint32) vm->guest_regs.ebx);
      map_ept_page ((uint32) vm->guest_regs.eax, (uint32) vm->guest_regs.ebx,
                    (uint8) vm->guest_regs.edx);
      break;
    default:
      logger_printf ("Unknow reason 0x%X caused VM-Exit in sandbox %d\n", status, cpu);
  }

  return;
}

#endif /* USE_VMX */

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
