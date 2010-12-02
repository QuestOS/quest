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

#include "vm/vmx.h"
#include "vm/vm86.h"
#include "kernel.h"
#include "mem/physical.h"
#include "mem/virtual.h"
#include "util/cpuid.h"
#include "util/printf.h"
#include "smp/apic.h"
#include "arch/i386.h"
#include "sched/sched.h"

#define DEBUG_VMX 2

#define IA32_FEATURE_CONTROL         0x003A
#define IA32_SYSENTER_CS             0x0174
#define IA32_SYSENTER_ESP            0x0175
#define IA32_SYSENTER_EIP            0x0176

/* See Intel System Programming Manual appendix G */
#define IA32_VMX_BASIC               0x0480
#define IA32_VMX_PINBASED_CTLS       0x0481
#define IA32_VMX_PROCBASED_CTLS      0x0482
#define IA32_VMX_EXIT_CTLS           0x0483
#define IA32_VMX_ENTRY_CTLS          0x0484
#define IA32_VMX_MISC                0x0485
#define IA32_VMX_CR0_FIXED0          0x0486
#define IA32_VMX_CR0_FIXED1          0x0487
#define IA32_VMX_CR4_FIXED0          0x0488
#define IA32_VMX_CR4_FIXED1          0x0489
#define IA32_VMX_VMCS_ENUM           0x048A
#define IA32_VMX_PROCBASED_CTLS2     0x048B
#define IA32_VMX_EPT_VPID_CAP        0x048C
#define IA32_VMX_TRUE_PINBASED_CTLS  0x048D
#define IA32_VMX_TRUE_PROCBASED_CTLS 0x048E
#define IA32_VMX_TRUE_EXIT_CTLS      0x048F
#define IA32_VMX_TRUE_ENTRY_CTLS     0x0490

#define VMX_NUM_INSTR_ERRORS 29
#if DEBUG_VMX > 0
static char *vm_instruction_errors[] = {
  /* 00 */ "No error",
  /* 01 */ "VMCALL executed in VMX root operation",
  /* 02 */ "VMCLEAR with invalid physical address",
  /* 03 */ "VMCLEAR with VMXON pointer",
  /* 04 */ "VMLAUNCH with non-clear VMCS",
  /* 05 */ "VMRESUME with non-launched VMCS",
  /* 06 */ "VMRESUME with a corrupted VMCS (indicates corruption of the current VMCS)",
  /* 07 */ "VM entry with invalid control field(s)",
  /* 08 */ "VM entry with invalid host-state field(s)",
  /* 09 */ "VMPTRLD with invalid physical address",
  /* 10 */ "VMPTRLD with VMXON pointer",
  /* 11 */ "VMPTRLD with incorrect VMCS revision identifier",
  /* 12 */ "VMREAD/VMWRITE from/to unsupported VMCS component",
  /* 13 */ "VMWRITE to read-only VMCS component",
  /* 14 */ "unused code: 14",
  /* 15 */ "VMXON executed in VMX root operation",
  /* 16 */ "VM entry with invalid executive-VMCS pointer",
  /* 17 */ "VM entry with non-launched executive VMCS",
  /* 18 */ "VM entry with executive-VMCS pointer not VMXON pointer (when attempting to deactivate the dual-monitor treatment of SMIs and SMM)",
  /* 19 */ "VMCALL with non-clear VMCS (when attempting to activate the dual-monitor treatment of SMIs and SMM)",
  /* 20 */ "VMCALL with invalid VM-exit control fields",
  /* 21 */ "unused code: 21",
  /* 22 */ "VMCALL with incorrect MSEG revision identifier (when attempting to activate the dual-monitor treatment of SMIs and SMM)",
  /* 23 */ "VMXOFF under dual-monitor treatment of SMIs and SMM",
  /* 24 */ "VMCALL with invalid SMM-monitor features (when attempting to activate the dual-monitor treatment of SMIs and SMM)",
  /* 25 */ "VM entry with invalid VM-execution control fields in executive VMCS (when attempting to return from SMM)",
  /* 26 */ "VM entry with events blocked by MOV SS.",
  /* 27 */ "unused code: 27",
  /* 28 */ "Invalid operand to INVEPT/INVVPID."
};

#define VMX_NUM_EXIT_REASONS 56
static char *vm_exit_reasons[] = {
  /* 00 */ "Exception or non-maskable interrupt (NMI).",
  /* 01 */ "External interrupt.",
  /* 02 */ "Triple fault.",
  /* 03 */ "INIT signal.",
  /* 04 */ "Start-up IPI (SIPI).",
  /* 05 */ "I/O system-management interrupt (SMI).",
  /* 06 */ "Other SMI.",
  /* 07 */ "Interrupt window.",
  /* 08 */ "NMI window.",
  /* 09 */ "Task switch.",
  /* 10 */ "CPUID.",
  /* 11 */ "GETSEC.",
  /* 12 */ "HLT.",
  /* 13 */ "INVD.",
  /* 14 */ "INVLPG.",
  /* 15 */ "RDPMC.",
  /* 16 */ "RDTSC.",
  /* 17 */ "RSM.",
  /* 18 */ "VMCALL.",
  /* 19 */ "VMCLEAR.",
  /* 20 */ "VMLAUNCH.",
  /* 21 */ "VMPTRLD.",
  /* 22 */ "VMPTRST.",
  /* 23 */ "VMREAD.",
  /* 24 */ "VMRESUME.",
  /* 25 */ "VMWRITE.",
  /* 26 */ "VMXOFF.",
  /* 27 */ "VMXON.",
  /* 28 */ "Control-register accesses.",
  /* 29 */ "MOV DR.",
  /* 30 */ "I/O instruction.",
  /* 31 */ "RDMSR.",
  /* 32 */ "WRMSR.",
  /* 33 */ "VM-entry failure due to invalid guest state.",
  /* 34 */ "VM-entry failure due to MSR loading.",
  /* 35 */ "reserved (35)",
  /* 36 */ "MWAIT.",
  /* 37 */ "Monitor trap flag.",
  /* 38 */ "reserved (38)",
  /* 39 */ "MONITOR.",
  /* 40 */ "PAUSE.",
  /* 41 */ "VM-entry failure due to machine check.",
  /* 42 */ "reserved (42)",
  /* 43 */ "TPR below threshold.",
  /* 44 */ "APIC access.",
  /* 45 */ "reserved (45)",
  /* 46 */ "Access to GDTR or IDTR.",
  /* 47 */ "Access to LDTR or TR.",
  /* 48 */ "EPT violation.",
  /* 49 */ "EPT misconfiguration.",
  /* 50 */ "INVEPT.",
  /* 51 */ "RDTSCP.",
  /* 52 */ "VMX-preemption timer expired.",
  /* 53 */ "INVVPID.",
  /* 54 */ "WBINVD.",
  /* 55 */ "XSETBV.  ",
};
#endif

bool vmx_enabled = FALSE;

void
vmx_detect (void)
{
  if (cpuid_vmx_support ()) {
    print ("VMX support detected\n");
    vmx_enabled = TRUE;
  }
}

void
vmx_test_guest (void)
{
  for (;;)
    asm volatile ("int $0xE");
}

static char *vmx_cr_access_register_names[] = {
  "EAX", "ECX", "EDX", "EBX", "ESP", "EBP", "ESI", "EDI"
};

void
vmx_vm_exit_reason (void)
{
  uint32 reason = vmread (VMXENC_EXIT_REASON);
  uint32 qualif = vmread (VMXENC_EXIT_QUAL);
  uint32 intinf = vmread (VMXENC_VM_EXIT_INTERRUPT_INFO);
  uint32 ercode = vmread (VMXENC_VM_EXIT_INTERRUPT_ERRCODE);
  /*******************************************************
   * uint32 inslen = vmread (VMXENC_VM_EXIT_INSTR_LEN);  *
   * uint32 insinf = vmread (VMXENC_VM_EXIT_INSTR_INFO); *
   *******************************************************/
  uint8 crnum, type, reg, vec;

  switch (reason) {
  case 0x0:
    /* Exception or NMI */
    if (intinf & 0x80000000) {
      char *cause;
      vec = intinf & 0xFF;
      type = (intinf & 0x700) >> 8;
      switch (type) {
      case 0: cause = "external interrupt"; break;
      case 2: cause = "NMI"; break;
      case 3: cause = "hardware exception"; break;
      case 6: cause = "software exception"; break;
      default: cause = "unknown"; break;
      }
      com1_printf ("  EXCEPTION: vector=%.2X code=%X cause=%s\n",
                   vec, (intinf & 0x800) ? ercode : 0, cause);
      if (vec == 0xE && type == 3) {
        /* Page fault */
        com1_printf ("    Page Fault at %.8X\n", qualif);
      }
    }
    break;
  case 0x1C:
    /* Control Register access */
    crnum = qualif & 0xF;
    type  = (qualif & 0x30) >> 4;
    reg   = (qualif & 0xF00) >> 8;
    switch (type) {
    case 0:
      com1_printf ("  CR WRITE: MOV %%%s, %%CR%d\n",
                   vmx_cr_access_register_names[reg],
                   crnum);
      break;
    case 1:
      com1_printf ("  CR READ: MOV %%CR%d, %%%s\n",
                   crnum,
                   vmx_cr_access_register_names[reg]);
      break;
    case 2:
      com1_printf ("  CLTS\n");
      break;
    case 3:
      com1_printf ("  LMSW\n");
      break;
    }
    break;
  }
}


static uint32 vmxon_frame[MAX_CPUS];
uint32 vmx_vm86_pgt[1024] __attribute__ ((aligned(0x1000)));

void
vmx_global_init (void)
{
  /* Map real-mode code at virtual address 0x8000 for VM86 task */
  extern uint32 _code16start, _code16_pages, _code16physicalstart;
  uint32 phys_pgt = (uint32) get_phys_addr (vmx_vm86_pgt);
  uint32 phys_pgd = (uint32) get_pdbr ();
  uint32 *virt_pgd = map_virtual_page (phys_pgd | 3);
  uint32 i;

  memset (vmx_vm86_pgt, 0, 1024 * sizeof (uint32));
  virt_pgd[0] = (uint32) phys_pgt | 7; /* so it is usable in PL=3 */
  unmap_virtual_page (virt_pgd);

  /* identity map the first megabyte */
  for (i=0; i<256; i++)
    vmx_vm86_pgt[i] = (i << 12) | 7;
  /* but then re-map pages starting at 0x8000 to our real-mode section */
  for (i=0; i<((uint32) &_code16_pages); i++)
    vmx_vm86_pgt[((((uint32) &_code16start) >> 12) & 0x3FF) + i] =
      ((uint32) &_code16physicalstart + (i << 12)) | 7;
  /* and unmap page 0 so that null pointer dereferences cause faults */
  vmx_vm86_pgt[0] = 0;

  flush_tlb_all ();

  /* Initialize the real-mode emulator */
  vmx_vm86_global_init ();
}

void
vmx_processor_init (void)
{
  uint8 phys_id = LAPIC_get_physical_ID ();
  uint32 cr0, cr4;
  uint32 *vmxon_virt;

  if (!vmx_enabled)
    return;

  /* Set the NE bit to satisfy CR0_FIXED0 */
  asm volatile ("movl %%cr0, %0\n"
                "orl $0x20, %0\n"
                "movl %0, %%cr0":"=r" (cr0));

#if DEBUG_VMX > 3
  com1_printf ("IA32_FEATURE_CONTROL: %.8X\n", (uint32) rdmsr (IA32_FEATURE_CONTROL));
  com1_printf ("IA32_VMX_BASIC: %.8X%.8X\n",
               (uint32) (rdmsr (IA32_VMX_BASIC) >> 32),
               (uint32) rdmsr (IA32_VMX_BASIC));
  com1_printf ("IA32_VMX_CR0_FIXED0: %.8X\n", (uint32) rdmsr (IA32_VMX_CR0_FIXED0));
  com1_printf ("IA32_VMX_CR0_FIXED1: %.8X\n", (uint32) rdmsr (IA32_VMX_CR0_FIXED1));
  com1_printf ("IA32_VMX_CR4_FIXED0: %.8X\n", (uint32) rdmsr (IA32_VMX_CR4_FIXED0));
  com1_printf ("IA32_VMX_CR4_FIXED1: %.8X\n", (uint32) rdmsr (IA32_VMX_CR4_FIXED1));

  com1_printf ("IA32_VMX_PINBASED_CTLS: %.8X%.8X\n",
               (uint32) (rdmsr (IA32_VMX_PINBASED_CTLS) >> 32),
               (uint32) rdmsr (IA32_VMX_PINBASED_CTLS));
  com1_printf ("IA32_VMX_TRUE_PINBASED_CTLS: %.8X%.8X\n",
               (uint32) (rdmsr (IA32_VMX_TRUE_PINBASED_CTLS) >> 32),
               (uint32) rdmsr (IA32_VMX_TRUE_PINBASED_CTLS));
  com1_printf ("IA32_VMX_PROCBASED_CTLS: %.8X%.8X\n",
               (uint32) (rdmsr (IA32_VMX_PROCBASED_CTLS) >> 32),
               (uint32) rdmsr (IA32_VMX_PROCBASED_CTLS));
  com1_printf ("IA32_VMX_TRUE_PROCBASED_CTLS: %.8X%.8X\n",
               (uint32) (rdmsr (IA32_VMX_TRUE_PROCBASED_CTLS) >> 32),
               (uint32) rdmsr (IA32_VMX_TRUE_PROCBASED_CTLS));
  com1_printf ("IA32_VMX_PROCBASED_CTLS2: %.8X%.8X\n",
               (uint32) (rdmsr (IA32_VMX_PROCBASED_CTLS2) >> 32),
               (uint32) rdmsr (IA32_VMX_PROCBASED_CTLS2));

  com1_printf ("IA32_VMX_EXIT_CTLS: %.8X%.8X\n",
               (uint32) (rdmsr (IA32_VMX_EXIT_CTLS) >> 32),
               (uint32) rdmsr (IA32_VMX_EXIT_CTLS));
  com1_printf ("IA32_VMX_ENTRY_CTLS: %.8X%.8X\n",
               (uint32) (rdmsr (IA32_VMX_ENTRY_CTLS) >> 32),
               (uint32) rdmsr (IA32_VMX_ENTRY_CTLS));
  com1_printf ("IA32_VMX_TRUE_EXIT_CTLS: %.8X%.8X\n",
               (uint32) (rdmsr (IA32_VMX_TRUE_EXIT_CTLS) >> 32),
               (uint32) rdmsr (IA32_VMX_TRUE_EXIT_CTLS));
  com1_printf ("IA32_VMX_TRUE_ENTRY_CTLS: %.8X%.8X\n",
               (uint32) (rdmsr (IA32_VMX_TRUE_ENTRY_CTLS) >> 32),
               (uint32) rdmsr (IA32_VMX_TRUE_ENTRY_CTLS));
  com1_printf ("IA32_VMX_MISC: %.8X%.8X\n",
               (uint32) (rdmsr (IA32_VMX_MISC) >> 32),
               (uint32) rdmsr (IA32_VMX_MISC));
#endif

  /* Enable VMX */
  asm volatile ("movl %%cr4, %0\n"
                "orl $0x2000, %0\n"
                "movl %0, %%cr4":"=r" (cr4));

  /* Allocate a VMXON memory area */
  vmxon_frame[phys_id] = alloc_phys_frame ();
  vmxon_virt  = map_virtual_page (vmxon_frame[phys_id] | 3);
  *vmxon_virt = rdmsr (IA32_VMX_BASIC);
  unmap_virtual_page (vmxon_virt);

  vmxon (vmxon_frame[phys_id]);

  if (vmx_get_error () != 0) {
#if DEBUG_VMX > 0
    com1_printf ("VMXON error\n");
#endif
    goto abort_vmxon;
  }

  return;

 abort_vmxon:
  free_phys_frame (vmxon_frame[phys_id]);
}

int
vmx_load_VM (virtual_machine *vm)
{
  uint32 phys_id = (uint32)LAPIC_get_physical_ID ();

  if (vm->loaded)
    return -1;

  vmptrld (vm->vmcs_frame);

  if (vmx_get_error () != 0) {
#if DEBUG_VMX > 0
    com1_printf ("VMPTRLD error\n");
#endif
    return -1;
  }

  vm->loaded = TRUE;
  vm->current_cpu = phys_id;
  return 0;
}

int
vmx_unload_VM (virtual_machine *vm)
{
  vmclear (vm->vmcs_frame);

  if (!vm->loaded)
    return -1;

  if (vmx_get_error () != 0) {
#if DEBUG_VMX > 0
    com1_printf ("VMCLEAR error\n");
#endif
    return -1;
  }

  vm->loaded = FALSE;
  return 0;
}

int
vmx_destroy_VM (virtual_machine *vm)
{
  uint32 stack_frame;
  if (vm->loaded)
    vmx_unload_VM (vm);
  free_phys_frame (vm->vmcs_frame);
  vm->vmcs_frame = 0;
  stack_frame = (uint32)get_phys_addr (vm->guest_stack);
  unmap_virtual_page (vm->guest_stack);
  free_phys_frame (stack_frame);
  return 0;
}

int
vmx_create_VM (virtual_machine *vm)
{
  void vmx_code16_entry (void);
  uint32 phys_id = (uint32)LAPIC_get_physical_ID ();
  uint32 *vmcs_virt;
  uint32 cr, stack_frame;
  descriptor ad;

  vm->realmode = TRUE;
  vm->launched = vm->loaded = FALSE;
  vm->current_cpu = phys_id;
  vm->guest_regs.eax = vm->guest_regs.ebx = vm->guest_regs.ecx =
    vm->guest_regs.edx = vm->guest_regs.esi = vm->guest_regs.edi =
    vm->guest_regs.ebp = 0;

  /* Setup the Virtual Machine Control Section */
  vm->vmcs_frame = alloc_phys_frame ();
  vmcs_virt  = map_virtual_page (vm->vmcs_frame | 3);
  vmcs_virt[0] = rdmsr (IA32_VMX_BASIC);
  vmcs_virt[1] = 0;
  unmap_virtual_page (vmcs_virt);

  stack_frame = alloc_phys_frame ();
  vm->guest_stack = map_virtual_page (stack_frame | 3);

  vmclear (vm->vmcs_frame);

  if (vmx_load_VM (vm) != 0)
    goto abort_load_VM;

  /* Setup Guest State */
  vmwrite ((1<<1)  |           /* Reserved bit set */
           (1<<17) |           /* VM86 */
           0,
           VMXENC_GUEST_RFLAGS);
  asm volatile ("movl %%cr0, %0":"=r" (cr));
  vmwrite (cr, VMXENC_GUEST_CR0);
  asm volatile ("movl %%cr3, %0":"=r" (cr));
  vmwrite (cr, VMXENC_GUEST_CR3);
  asm volatile ("movl %%cr4, %0":"=r" (cr));
  vmwrite (cr, VMXENC_GUEST_CR4);
  vmwrite (0x0, VMXENC_GUEST_DR7);
  vmwrite (0x0, VMXENC_GUEST_CS_SEL);
  vmwrite (VMX_VM86_START_SS_SEL, VMXENC_GUEST_SS_SEL);
  vmwrite (0x0, VMXENC_GUEST_DS_SEL);
  vmwrite (0x0, VMXENC_GUEST_ES_SEL);
  vmwrite (0x0, VMXENC_GUEST_FS_SEL);
  vmwrite (0x0, VMXENC_GUEST_GS_SEL);
  vmwrite (str (), VMXENC_GUEST_TR_SEL);
  vmwrite ((uint32) lookup_TSS (str ()), VMXENC_GUEST_TR_BASE);
  vmwrite (0x0, VMXENC_GUEST_CS_BASE);
  vmwrite (VMX_VM86_START_SS_SEL << 4, VMXENC_GUEST_SS_BASE);
  vmwrite (0x0, VMXENC_GUEST_DS_BASE);
  vmwrite (0x0, VMXENC_GUEST_ES_BASE);
  vmwrite (0x0, VMXENC_GUEST_FS_BASE);
  vmwrite (0x0, VMXENC_GUEST_GS_BASE);
  vmwrite ((uint32) sgdtr (), VMXENC_GUEST_GDTR_BASE);
  vmwrite ((uint32) sidtr (), VMXENC_GUEST_IDTR_BASE);
  vmwrite (0xFFFF, VMXENC_GUEST_CS_LIMIT);
  vmwrite (0xFFFF, VMXENC_GUEST_DS_LIMIT);
  vmwrite (0xFFFF, VMXENC_GUEST_ES_LIMIT);
  vmwrite (0xFFFF, VMXENC_GUEST_FS_LIMIT);
  vmwrite (0xFFFF, VMXENC_GUEST_GS_LIMIT);
  vmwrite (0xFFFF, VMXENC_GUEST_SS_LIMIT);
#define ACCESS(ad)                              \
  (( 0x01            << 0x00 ) |                \
   ( ad.uType        << 0x00 ) |                \
   ( ad.uDPL         << 0x05 ) |                \
   ( ad.fPresent     << 0x07 ) |                \
   ( ad.f            << 0x0C ) |                \
   ( ad.f0           << 0x0D ) |                \
   ( ad.fX           << 0x0E ) |                \
   ( ad.fGranularity << 0x0F ))
  vmwrite (0xF3, VMXENC_GUEST_CS_ACCESS);
  vmwrite (0xF3, VMXENC_GUEST_DS_ACCESS);
  vmwrite (0xF3, VMXENC_GUEST_ES_ACCESS);
  vmwrite (0xF3, VMXENC_GUEST_FS_ACCESS);
  vmwrite (0xF3, VMXENC_GUEST_GS_ACCESS);
  vmwrite (0xF3, VMXENC_GUEST_SS_ACCESS);
  vmwrite (0x8B, VMXENC_GUEST_TR_ACCESS);
#undef ACCESS
  vmwrite ((uint32) sgdtr (), VMXENC_GUEST_GDTR_BASE);
  vmwrite ((uint32) sidtr (), VMXENC_GUEST_IDTR_BASE);
  vmwrite (sgdtr_limit (), VMXENC_GUEST_GDTR_LIMIT);
  vmwrite (sidtr_limit (), VMXENC_GUEST_IDTR_LIMIT);
  get_GDT_descriptor (str (), &ad);
  vmwrite (ad.uLimit0 | (ad.uLimit1 << 16), VMXENC_GUEST_TR_LIMIT);
  vmwrite ((uint32) vmx_code16_entry, VMXENC_GUEST_RIP);
  vmwrite ((uint32) VMX_VM86_START_SP, VMXENC_GUEST_RSP);
  vmwrite (0, VMXENC_GUEST_LDTR_SEL);
  vmwrite (0, VMXENC_GUEST_LDTR_BASE);
  vmwrite (0, VMXENC_GUEST_LDTR_LIMIT);
  vmwrite (0x82, VMXENC_GUEST_LDTR_ACCESS);
  vmwrite (0, VMXENC_GUEST_IA32_SYSENTER_CS);
  vmwrite (0, VMXENC_GUEST_IA32_SYSENTER_ESP);
  vmwrite (0, VMXENC_GUEST_IA32_SYSENTER_EIP);
  vmwrite64 (0xFFFFFFFFFFFFFFFFLL, VMXENC_VMCS_LINK_PTR);
  vmwrite (0, VMXENC_GUEST_PENDING_DEBUG_EXCEPTIONS);
  vmwrite (0, VMXENC_GUEST_ACTIVITY);
  vmwrite (0, VMXENC_GUEST_INTERRUPTIBILITY);
  vmwrite (~0, VMXENC_EXCEPTION_BITMAP);
  vmwrite (0, VMXENC_PAGE_FAULT_ERRCODE_MASK);
  vmwrite (0, VMXENC_PAGE_FAULT_ERRCODE_MATCH);
  /* Mask the PG and PE bits. */
  vmwrite (0x80000001, VMXENC_CR0_GUEST_HOST_MASK);
  /* Although we start in real-mode, this read-shadow is not used
   * until the VM86 simulation of real-mode is disabled.  At which
   * point, we are simulating prot-mode.  Therefore, leave PE set in
   * the read-shadow. */
  vmwrite (0x00000001, VMXENC_CR0_READ_SHADOW);

  return 0;

 abort_load_VM:
  vmx_destroy_VM (vm);
  return -1;
}

int
vmx_enter_VM (virtual_machine *vm)
{
  uint32 phys_id = (uint32)LAPIC_get_physical_ID ();
  uint32 cr, esp, eip, state = 0;

  if (!vm->loaded || vm->current_cpu != phys_id)
    goto not_loaded;

 enter:
  /* Save Host State */
  vmwrite (rdmsr (IA32_VMX_PINBASED_CTLS), VMXENC_PINBASED_VM_EXEC_CTRLS);
  vmwrite (rdmsr (IA32_VMX_PROCBASED_CTLS), VMXENC_PROCBASED_VM_EXEC_CTRLS);
  vmwrite (0, VMXENC_CR3_TARGET_COUNT);
  vmwrite (rdmsr (IA32_VMX_EXIT_CTLS), VMXENC_VM_EXIT_CTRLS);
  vmwrite (0, VMXENC_VM_EXIT_MSR_STORE_COUNT);
  vmwrite (0, VMXENC_VM_EXIT_MSR_LOAD_COUNT);
  vmwrite (rdmsr (IA32_VMX_ENTRY_CTLS), VMXENC_VM_ENTRY_CTRLS);
  vmwrite (0, VMXENC_VM_ENTRY_MSR_LOAD_COUNT);
  asm volatile ("movl %%cr0, %0":"=r" (cr));
  vmwrite (cr, VMXENC_HOST_CR0);
  asm volatile ("movl %%cr3, %0":"=r" (cr));
  vmwrite (cr, VMXENC_HOST_CR3);
  asm volatile ("movl %%cr4, %0":"=r" (cr));
  vmwrite (cr, VMXENC_HOST_CR4);
  vmwrite (0x08, VMXENC_HOST_CS_SEL);
  vmwrite (0x10, VMXENC_HOST_SS_SEL);
  vmwrite (0x10, VMXENC_HOST_DS_SEL);
  vmwrite (0x10, VMXENC_HOST_ES_SEL);
  vmwrite (0x10, VMXENC_HOST_FS_SEL);
  vmwrite (0x10, VMXENC_HOST_GS_SEL);
  vmwrite (str (), VMXENC_HOST_TR_SEL);
  vmwrite ((uint32) lookup_TSS (str ()), VMXENC_HOST_TR_BASE);
  vmwrite ((uint32) lookup_GDT_selector (0x10), VMXENC_HOST_FS_BASE);
  vmwrite ((uint32) lookup_GDT_selector (0x10), VMXENC_HOST_GS_BASE);
  vmwrite ((uint32) sgdtr (), VMXENC_HOST_GDTR_BASE);
  vmwrite ((uint32) sidtr (), VMXENC_HOST_IDTR_BASE);
  vmwrite (0, VMXENC_VM_ENTRY_INTERRUPT_INFO);
  vmwrite (rdmsr (IA32_SYSENTER_CS), VMXENC_HOST_IA32_SYSENTER_CS);
  vmwrite (rdmsr (IA32_SYSENTER_ESP), VMXENC_HOST_IA32_SYSENTER_ESP);
  vmwrite (rdmsr (IA32_SYSENTER_EIP), VMXENC_HOST_IA32_SYSENTER_EIP);
  vmwrite (vmread (VMXENC_GUEST_CS_ACCESS) | 0x1, VMXENC_GUEST_CS_ACCESS);

  /* clobber-list is not necessary here because "pusha" below saves
   * HOST registers */
  asm volatile (/* save HOST registers on stack and ESP in VMCS */
                "pusha\n"
                "movl %%esp, %1\n"
                "vmwrite %1, %3\n"
                "addl $8, %%esp\n"
                /* Do trick to get current EIP and differentiate between the first
                 * and second time this code is invoked. */
                "call 1f\n"
                /* On VM-EXIT, resume Host here: */
                "pusha\n"       /* quickly snapshot guest registers to stack */
                "addl $0x20, %%esp\n"
                "popa\n"        /* temporarily restore host registers */
                "movl $8, %%ecx\n"
                "lea -0x40(%%esp), %%esi\n"
                "lea %2, %%edi\n"
                "cld; rep movsd\n" /* save guest registers to memory */
                "subl $0x20, %%esp\n"
                "popa\n"        /* permanently restore host registers */
                "xor %0, %0\n"
                "jmp 2f\n"
                "1:\n"
                "pop %0\n"
                "2:\n"
                :"=r" (eip),"=r" (esp):"m" (vm->guest_regs),"r" (VMXENC_HOST_RSP));

  /* VM-ENTER */
  if (eip) {
    // printf ("Entering VM!\n");
    vmwrite (eip, VMXENC_HOST_RIP);

    if (vm->launched) {
      asm volatile ("movl $1, %0\n"
                    "movl $2, %1\n"
                    /* Restore Guest registers using POPA */
                    "subl $0x20, %%esp\n"
                    "movl %%esp, %%edi\n"
                    "cld; rep movsd\n"
                    "popa\n"
                    "vmresume"
                    :"=m" (vm->launched), "=m"(state)
                    :"c" (8), "S" (&vm->guest_regs):"edi","cc","memory");
    } else {
      asm volatile ("movl $1, %0\n"
                    "movl $1, %1\n"
                    /* Restore Guest registers using POPA */
                    "subl $0x20, %%esp\n"
                    "movl %%esp, %%edi\n"
                    "cld; rep movsd\n"
                    "popa\n"
                    "vmlaunch"
                    :"=m" (vm->launched), "=m"(state)
                    :"c" (8), "S" (&vm->guest_regs):"edi","cc","memory");
    }
  }

  if (!eip) {
    /* VM-exited */
    uint32 reason = vmread (VMXENC_EXIT_REASON);
    uint32 intinf = vmread (VMXENC_VM_EXIT_INTERRUPT_INFO);
#if DEBUG_VMX > 2
    uint32 qualif = vmread (VMXENC_EXIT_QUAL);
    uint32 ercode = vmread (VMXENC_VM_EXIT_INTERRUPT_ERRCODE);
    uint32 inslen = vmread (VMXENC_VM_EXIT_INSTR_LEN);
    uint32 insinf = vmread (VMXENC_VM_EXIT_INSTR_INFO);
#endif

    if (reason & (1 << 31)) {
      /* VM-exit was due to failure during checking of Guest state
       * during VM-entry */
      reason &= ~(1 << 31);
      if (state == 1)
        /* Failure to VMLAUNCH */
        vm->launched = FALSE;
    }

#if DEBUG_VMX > 2
    com1_printf ("VM-EXIT: %s\n  reason=%.8X qualif=%.8X\n  intinf=%.8X ercode=%.8X\n  inslen=%.8X insinf=%.8X\n",
                 (reason < VMX_NUM_EXIT_REASONS ?
                  vm_exit_reasons[reason] : "invalid exit-reason"),
                 reason, qualif, intinf, ercode, inslen, insinf);
    vmx_vm_exit_reason ();
#endif

    if (vm->realmode && reason == 0x0 && (intinf & 0xFF) == 0x0D) {
      /* General Protection Fault in vm86 mode */
      if (vmx_vm86_handle_GPF (vm) == 0)
        /* continue guest */
        goto enter;
    }
  } else if (vmx_get_error () != 0) {
#if DEBUG_VMX > 1
    uint32 error = vmread (VMXENC_VM_INSTR_ERROR);
#endif
    uint32 reason = vmread (VMXENC_EXIT_REASON);

    if (state == 1)
      /* Failure to VMLAUNCH */
      vm->launched = FALSE;

#if DEBUG_VMX > 1
    com1_printf ("VM-ENTRY: error: %.8X (%s)\n  reason: %.8X qual: %.8X\n",
                 error,
                 (error < VMX_NUM_INSTR_ERRORS ? vm_instruction_errors[error] : "n/a"),
                 reason,
                 vmread (VMXENC_EXIT_QUAL));
#endif
    if (reason & 0x80000000) {
#if DEBUG_VMX > 0
      com1_printf ("  VM-ENTRY failure, code: %d\n", reason & 0xFF);
#endif
    }
    goto abort_vmentry;
  }

  return 0;
 abort_vmentry:
 not_loaded:
  return -1;
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
