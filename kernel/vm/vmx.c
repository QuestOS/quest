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

#include "vm/vmx.h"
#include "vm/ept.h"
#include "vm/vm86.h"
#include "kernel.h"
#include "mem/physical.h"
#include "mem/virtual.h"
#include "util/cpuid.h"
#include "util/printf.h"
#include "smp/apic.h"
#include "arch/i386.h"
#include "arch/i386-mtrr.h"
#include "sched/sched.h"
#include "vm/shdr.h"
#include "vm/shm.h"
#include "vm/migration.h"

#ifdef USE_LINUX_SANDBOX
#include "vm/linux_boot.h"
#endif

#define DEBUG_VMX 2
#define VMX_EPT

#if DEBUG_VMX > 2
#define DLOG(fmt,...) DLOG_PREFIX("vmx",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define com1_printf logger_printf

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
bool shared_driver_available = TRUE;

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
static u32 msr_bitmaps[1024] ALIGNED (0x1000);

void
vmx_global_init (void)
{
  DLOG ("global_init");
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

  /* clear MSR bitmaps */
  memset (msr_bitmaps, 0, 0x1000);
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

//#define EXCEPTION_EXIT

int
vmx_create_pmode_VM (virtual_machine *vm, u32 rip0, u32 rsp0)
{
  void vmx_code16_entry (void);
  uint32 phys_id = (uint32)LAPIC_get_physical_ID ();
  uint32 *vmcs_virt;
  uint32 cr, stack_frame, sel, base, limit, access;
  descriptor ad;

  vm->realmode = FALSE;
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
  asm volatile ("pushfl; pop %0":"=r" (cr));
  vmwrite (cr, VMXENC_GUEST_RFLAGS);
  asm volatile ("movl %%cr0, %0":"=r" (cr));
  vmwrite (cr, VMXENC_GUEST_CR0);
  asm volatile ("movl %%cr3, %0":"=r" (cr));
  vmwrite (cr, VMXENC_GUEST_CR3);
  asm volatile ("movl %%cr4, %0":"=r" (cr));
  vmwrite (cr, VMXENC_GUEST_CR4);
  vmwrite (0x0, VMXENC_GUEST_DR7);
  logger_printf ("GUEST-STATE: FLAGS=0x%p CR0=0x%p CR3=0x%p CR4=0x%p\n",
                 vmread (VMXENC_GUEST_RFLAGS),
                 vmread (VMXENC_GUEST_CR0),
                 vmread (VMXENC_GUEST_CR3),
                 vmread (VMXENC_GUEST_CR4));

#define ACCESS(ad)                              \
  (( 0x01            << 0x00 ) |                \
   ( ad.uType        << 0x00 ) |                \
   ( ad.uDPL         << 0x05 ) |                \
   ( ad.fPresent     << 0x07 ) |                \
   ( ad.f            << 0x0C ) |                \
   ( ad.f0           << 0x0D ) |                \
   ( ad.fX           << 0x0E ) |                \
   ( ad.fGranularity << 0x0F ))

  /* Setup segment selector/base/limit/access entries */
#define SETUPSEG(seg) do {                                              \
    asm volatile ("movl %%" __stringify(seg) ", %0":"=r" (sel));        \
    get_GDT_descriptor (sel, &ad);                                      \
    base = (ad.pBase0 | (ad.pBase1 << 16) | (ad.pBase2 << 24));         \
    limit = ad.uLimit0 | (ad.uLimit1 << 16);                            \
    if (ad.fGranularity) { limit <<= 12; limit |= 0xFFF; }              \
    access = ACCESS (ad);                                               \
    vmwrite (sel, VMXENC_GUEST_##seg##_SEL);                            \
    vmwrite (base, VMXENC_GUEST_##seg##_BASE);                          \
    vmwrite (limit, VMXENC_GUEST_##seg##_LIMIT);                        \
    vmwrite (access, VMXENC_GUEST_##seg##_ACCESS);                      \
    logger_printf ("GUEST-STATE: %s=0x%.02X base=0x%p limit=0x%p access=0x%.02X\n", \
                   __stringify(seg), sel, base, limit, access);         \
  } while (0)

  SETUPSEG (CS);
  SETUPSEG (SS);
  SETUPSEG (DS);
  SETUPSEG (ES);
  SETUPSEG (FS);
  SETUPSEG (GS);

  /* TR */
  sel = hw_str ();
  get_GDT_descriptor (sel, &ad);
  base = (ad.pBase0 | (ad.pBase1 << 16) | (ad.pBase2 << 24));
  limit = ad.uLimit0 | (ad.uLimit1 << 16);
  if (ad.fGranularity) { limit <<= 12; limit |= 0xFFF; }
  access = ACCESS (ad);
  vmwrite (sel, VMXENC_GUEST_TR_SEL);
  vmwrite (base, VMXENC_GUEST_TR_BASE);
  vmwrite (limit, VMXENC_GUEST_TR_LIMIT);
  vmwrite (access, VMXENC_GUEST_TR_ACCESS);
  logger_printf ("GUEST-STATE: %s=0x%.02X base=0x%p limit=0x%p access=0x%.02X\n",
                 "TR", sel, base, limit, access);

#undef ACCESS

  /* LDTR */
  vmwrite (0, VMXENC_GUEST_LDTR_SEL);
  vmwrite (0, VMXENC_GUEST_LDTR_BASE);
  vmwrite (0, VMXENC_GUEST_LDTR_LIMIT);
  vmwrite (0x10082, VMXENC_GUEST_LDTR_ACCESS);

  /* GDTR */
  vmwrite ((uint32) sgdtr (), VMXENC_GUEST_GDTR_BASE);
  vmwrite (sgdtr_limit (), VMXENC_GUEST_GDTR_LIMIT);

  /* IDTR */
  vmwrite ((uint32) sidtr (), VMXENC_GUEST_IDTR_BASE);
  vmwrite (sidtr_limit (), VMXENC_GUEST_IDTR_LIMIT);

  /* RIP/RSP */
  vmwrite ((uint32) rip0, VMXENC_GUEST_RIP);
  vmwrite ((uint32) rsp0, VMXENC_GUEST_RSP);
  logger_printf ("GUEST-STATE: RIP=0x%p RSP=0x%p\n", rip0, rsp0);

  /* SYSENTER MSRs */
  vmwrite (0, VMXENC_GUEST_IA32_SYSENTER_CS);
  vmwrite (0, VMXENC_GUEST_IA32_SYSENTER_ESP);
  vmwrite (0, VMXENC_GUEST_IA32_SYSENTER_EIP);

  vmwrite64 (0xFFFFFFFFFFFFFFFFLL, VMXENC_VMCS_LINK_PTR);
  vmwrite (0, VMXENC_GUEST_PENDING_DEBUG_EXCEPTIONS);
  vmwrite (0, VMXENC_GUEST_ACTIVITY);
  vmwrite (0, VMXENC_GUEST_INTERRUPTIBILITY);
#ifdef EXCEPTION_EXIT
  vmwrite (~0, VMXENC_EXCEPTION_BITMAP);
#else
  vmwrite (0, VMXENC_EXCEPTION_BITMAP); /* do not exit on exception (see manual about page faults) */
#endif
  vmwrite (0, VMXENC_PAGE_FAULT_ERRCODE_MASK);
  vmwrite (0, VMXENC_PAGE_FAULT_ERRCODE_MATCH);
  vmwrite (0, VMXENC_CR0_GUEST_HOST_MASK); /* all bits "owned" by guest */
  vmwrite (0, VMXENC_CR0_READ_SHADOW);

  return 0;

 abort_load_VM:
  vmx_destroy_VM (vm);
  return -1;
}

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
  real_mode_size = setup_header->setup_sects * 512;
  /* Move real mode Linux code to 0x00090000 */
  memcpy ((void *) 0x90000, (const void *) kernel_addr, real_mode_size);
  logger_printf ("%d bytes copied to 0x90000\n", real_mode_size);
  exec_ctrls2 = vmread (VMXENC_PROCBASED_VM_EXEC_CTRLS2);
  vmwrite (exec_ctrls2 | 0x80, VMXENC_PROCBASED_VM_EXEC_CTRLS2);
  exec_ctrls2 = vmread (VMXENC_PROCBASED_VM_EXEC_CTRLS2);
  logger_printf ("exec_ctrls2=0x%X\n", exec_ctrls2);
}
#endif

/* --YL-- We use two global variables for input and output of vm_exit routine. These
 * variables should be placed in sandbox kernel memory instead of monitor and some
 * more secure method should be prefered. For now, we just place them in monitor
 * for convenience.
 */
void * vm_exit_input_param = NULL;
void * vm_exit_return_val = NULL;

/*
 * Processing intentional VM-Exit from Sandboxes.
 */
void
vmx_process_exit (uint32 status)
{
  uint cpu = get_pcpu_id ();
  static quest_tss * ret_tss = NULL;
  static pgdir_t mdir = {-1, 0}, cdir = {-1, 0};
  static bool new_request = TRUE;  /* Is this a new request or a preempted one? */
  struct _tm_param {
    void * ptss;
    u64 dl;
  };
  struct _linux_boot_param {
    uint32 kernel_addr;
    int size;
  };
  struct _tm_param * param = (struct _tm_param *) vm_exit_input_param;
  DLOG ("Sandbox%d: performing VM-Exit Status: 0x%X Input: 0x%X\n",
        cpu, status, vm_exit_input_param);

  switch (status) {
    case VM_EXIT_REASON_LINUX_BOOT:
#ifdef USE_LINUX_SANDBOX
      linux_vm_init (((struct _linux_boot_param *) vm_exit_input_param)->kernel_addr,
                     ((struct _linux_boot_param *) vm_exit_input_param)->size);
#endif
      break;
    case VM_EXIT_REASON_MIGRATION:
#ifdef USE_VMX
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
          break;
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
#endif
      break;
    case VM_EXIT_REASON_GET_HPA:
      vm_exit_return_val = (void *) get_host_phys_addr ((uint32) vm_exit_input_param);
      DLOG ("Sandbox%d: Host Phys for 0x%X is 0x%X\n", get_pcpu_id (),
            (uint32) vm_exit_input_param, (uint32) vm_exit_return_val);
      break;
    default:
      logger_printf ("Unknow reason 0x%X caused VM-Exit in sandbox %d\n", status, cpu);
  }

  return;
}

int
vmx_start_VM (virtual_machine *vm)
{
  uint32 phys_id = (uint32)LAPIC_get_physical_ID ();
  uint32 cr, eip, state = 0, err;
  uint16 fs;
  u64 start, finish, proc_msr;

  if (!vm->loaded || vm->current_cpu != phys_id)
    goto not_loaded;

  /* Save Host State */
  vmwrite (rdmsr (IA32_VMX_PINBASED_CTLS), VMXENC_PINBASED_VM_EXEC_CTRLS);
  proc_msr = rdmsr (IA32_VMX_PROCBASED_CTLS) | rdmsr (IA32_VMX_TRUE_PROCBASED_CTLS);
  proc_msr &= ~((1 << 15) | (1 << 16) | (1 << 12) | (1 << 11)); /* allow CR3 load/store, RDTSC, RDPMC */
  proc_msr |= (1 << 28);                                        /* use MSR bitmaps */
  proc_msr |= (1 << 31);                                        /* secondary controls */
  vmwrite (proc_msr, VMXENC_PROCBASED_VM_EXEC_CTRLS);
  vmwrite (0, VMXENC_PROCBASED_VM_EXEC_CTRLS2);
  vmwrite (0, VMXENC_CR3_TARGET_COUNT);
  vmwrite (rdmsr (IA32_VMX_EXIT_CTLS), VMXENC_VM_EXIT_CTRLS);
  vmwrite (0, VMXENC_VM_EXIT_MSR_STORE_COUNT);
  vmwrite (0, VMXENC_VM_EXIT_MSR_LOAD_COUNT);
  /* Do not load debug controls. We have to check the capability regs before doing this. */
  vmwrite (rdmsr (IA32_VMX_ENTRY_CTLS) & (~(1 << 2)), VMXENC_VM_ENTRY_CTRLS);
  vmwrite (0, VMXENC_VM_ENTRY_MSR_LOAD_COUNT);
  vmwrite (0, VMXENC_MSR_BITMAPS_HI);
  vmwrite ((u32) get_phys_addr (msr_bitmaps), VMXENC_MSR_BITMAPS);
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
  asm volatile ("movw %%fs, %0":"=r" (fs));
  vmwrite (fs, VMXENC_HOST_FS_SEL);
  vmwrite (0x10, VMXENC_HOST_GS_SEL);
  vmwrite (hw_str (), VMXENC_HOST_TR_SEL);
  vmwrite ((uint32) lookup_TSS (hw_str ()), VMXENC_HOST_TR_BASE);
  vmwrite ((uint32) lookup_GDT_selector (fs), VMXENC_HOST_FS_BASE);
  vmwrite ((uint32) lookup_GDT_selector (0x10), VMXENC_HOST_GS_BASE);
  vmwrite ((uint32) sgdtr (), VMXENC_HOST_GDTR_BASE);
  vmwrite ((uint32) sidtr (), VMXENC_HOST_IDTR_BASE);
  vmwrite (0, VMXENC_VM_ENTRY_INTERRUPT_INFO);
  vmwrite (rdmsr (IA32_SYSENTER_CS), VMXENC_HOST_IA32_SYSENTER_CS);
  vmwrite (rdmsr (IA32_SYSENTER_ESP), VMXENC_HOST_IA32_SYSENTER_ESP);
  vmwrite (rdmsr (IA32_SYSENTER_EIP), VMXENC_HOST_IA32_SYSENTER_EIP);
  vmwrite (vmread (VMXENC_GUEST_CS_ACCESS) | 0x1, VMXENC_GUEST_CS_ACCESS);

  u32 cpu = get_pcpu_id ();

  /* 
   * vmx_init_mem will essentially do a VM fork. After this point
   * we will be in a new sandbox kernel and with shared memory enabled.
   */
  vmx_init_mem (cpu);

#ifdef VMX_EPT
  vmx_init_ept (cpu);
#endif

  logger_printf ("vmx_start_VM: GUEST-STATE: RIP=0x%p RSP=0x%p RBP=0x%p CR3=0x%p\n",
                 vmread (VMXENC_GUEST_RIP), vmread (VMXENC_GUEST_RSP), vm->guest_regs.ebp,
                 vmread (VMXENC_GUEST_CR3));

//#define VMX_DUMP_GUEST_STATE
//#define VMX_DUMP_CONTROLS

#ifdef VMX_DUMP_CONTROLS
  logger_printf ("-------------------\n");
  logger_printf ("|VMX CONTROLS DUMP|\n");
  logger_printf ("-------------------\n");
  logger_printf ("VMXENC_PINBASED_VM_EXEC_CTRLS=0x%X\n", vmread (VMXENC_PINBASED_VM_EXEC_CTRLS));
  logger_printf ("VMXENC_PROCBASED_VM_EXEC_CTRLS=0x%X\n", vmread (VMXENC_PROCBASED_VM_EXEC_CTRLS));
  logger_printf ("VMXENC_PROCBASED_VM_EXEC_CTRLS2=0x%X\n", vmread (VMXENC_PROCBASED_VM_EXEC_CTRLS2));
  logger_printf ("VMXENC_CR3_TARGET_COUNT=0x%X\n", vmread (VMXENC_CR3_TARGET_COUNT));
  logger_printf ("VMXENC_VM_EXIT_CTRLS=0x%X\n", vmread (VMXENC_VM_EXIT_CTRLS));
  logger_printf ("VMXENC_VM_EXIT_MSR_STORE_COUNT=0x%X\n", vmread (VMXENC_VM_EXIT_MSR_STORE_COUNT));
  logger_printf ("VMXENC_VM_EXIT_MSR_LOAD_COUNT=0x%X\n", vmread (VMXENC_VM_EXIT_MSR_LOAD_COUNT));
  logger_printf ("VMXENC_VM_ENTRY_CTRLS=0x%X\n", vmread (VMXENC_VM_ENTRY_CTRLS));
  logger_printf ("VMXENC_VM_ENTRY_MSR_LOAD_COUNT=0x%X\n", vmread (VMXENC_VM_ENTRY_MSR_LOAD_COUNT));
  logger_printf ("------------------------\n");
  logger_printf ("|VMX CONTROLS DUMP DONE|\n");
  logger_printf ("------------------------\n");
#endif

#ifdef VMX_DUMP_GUEST_STATE
  logger_printf ("----------------------\n");
  logger_printf ("|VMX GUEST STATE DUMP|\n");
  logger_printf ("----------------------\n");
  logger_printf ("VMXENC_GUEST_RFLAGS=0x%X\n", vmread (VMXENC_GUEST_RFLAGS));
  logger_printf ("VMXENC_GUEST_CR0=0x%X\n", vmread (VMXENC_GUEST_CR0));
  logger_printf ("VMXENC_GUEST_CR3=0x%X\n", vmread (VMXENC_GUEST_CR3));
  logger_printf ("VMXENC_GUEST_CR4=0x%X\n", vmread (VMXENC_GUEST_CR4));
  logger_printf ("VMXENC_GUEST_DR7=0x%X\n", vmread (VMXENC_GUEST_DR7));
  logger_printf ("VMXENC_GUEST_CS_SEL=0x%X\n", vmread (VMXENC_GUEST_CS_SEL));
  logger_printf ("VMXENC_GUEST_CS_BASE=0x%X\n", vmread (VMXENC_GUEST_CS_BASE));
  logger_printf ("VMXENC_GUEST_CS_LIMIT=0x%X\n", vmread (VMXENC_GUEST_CS_LIMIT));
  logger_printf ("VMXENC_GUEST_CS_ACCESS=0x%X\n", vmread (VMXENC_GUEST_CS_ACCESS));
  logger_printf ("VMXENC_GUEST_SS_SEL=0x%X\n", vmread (VMXENC_GUEST_SS_SEL));
  logger_printf ("VMXENC_GUEST_SS_BASE=0x%X\n", vmread (VMXENC_GUEST_SS_BASE));
  logger_printf ("VMXENC_GUEST_SS_LIMIT=0x%X\n", vmread (VMXENC_GUEST_SS_LIMIT));
  logger_printf ("VMXENC_GUEST_SS_ACCESS=0x%X\n", vmread (VMXENC_GUEST_SS_ACCESS));
  logger_printf ("VMXENC_GUEST_DS_SEL=0x%X\n", vmread (VMXENC_GUEST_DS_SEL));
  logger_printf ("VMXENC_GUEST_DS_BASE=0x%X\n", vmread (VMXENC_GUEST_DS_BASE));
  logger_printf ("VMXENC_GUEST_DS_LIMIT=0x%X\n", vmread (VMXENC_GUEST_DS_LIMIT));
  logger_printf ("VMXENC_GUEST_DS_ACCESS=0x%X\n", vmread (VMXENC_GUEST_DS_ACCESS));
  logger_printf ("VMXENC_GUEST_ES_SEL=0x%X\n", vmread (VMXENC_GUEST_ES_SEL));
  logger_printf ("VMXENC_GUEST_ES_BASE=0x%X\n", vmread (VMXENC_GUEST_ES_BASE));
  logger_printf ("VMXENC_GUEST_ES_LIMIT=0x%X\n", vmread (VMXENC_GUEST_ES_LIMIT));
  logger_printf ("VMXENC_GUEST_ES_ACCESS=0x%X\n", vmread (VMXENC_GUEST_ES_ACCESS));
  logger_printf ("VMXENC_GUEST_FS_SEL=0x%X\n", vmread (VMXENC_GUEST_FS_SEL));
  logger_printf ("VMXENC_GUEST_FS_BASE=0x%X\n", vmread (VMXENC_GUEST_FS_BASE));
  logger_printf ("VMXENC_GUEST_FS_LIMIT=0x%X\n", vmread (VMXENC_GUEST_FS_LIMIT));
  logger_printf ("VMXENC_GUEST_FS_ACCESS=0x%X\n", vmread (VMXENC_GUEST_FS_ACCESS));
  logger_printf ("VMXENC_GUEST_GS_SEL=0x%X\n", vmread (VMXENC_GUEST_GS_SEL));
  logger_printf ("VMXENC_GUEST_GS_BASE=0x%X\n", vmread (VMXENC_GUEST_GS_BASE));
  logger_printf ("VMXENC_GUEST_GS_LIMIT=0x%X\n", vmread (VMXENC_GUEST_GS_LIMIT));
  logger_printf ("VMXENC_GUEST_GS_ACCESS=0x%X\n", vmread (VMXENC_GUEST_GS_ACCESS));
  logger_printf ("VMXENC_GUEST_TR_SEL=0x%X\n", vmread (VMXENC_GUEST_TR_SEL));
  logger_printf ("VMXENC_GUEST_TR_BASE=0x%X\n", vmread (VMXENC_GUEST_TR_BASE));
  logger_printf ("VMXENC_GUEST_TR_LIMIT=0x%X\n", vmread (VMXENC_GUEST_TR_LIMIT));
  logger_printf ("VMXENC_GUEST_TR_ACCESS=0x%X\n", vmread (VMXENC_GUEST_TR_ACCESS));
  logger_printf ("VMXENC_GUEST_LDTR_SEL=0x%X\n", vmread (VMXENC_GUEST_LDTR_SEL));
  logger_printf ("VMXENC_GUEST_LDTR_BASE=0x%X\n", vmread (VMXENC_GUEST_LDTR_BASE));
  logger_printf ("VMXENC_GUEST_LDTR_LIMIT=0x%X\n", vmread (VMXENC_GUEST_LDTR_LIMIT));
  logger_printf ("VMXENC_GUEST_LDTR_ACCESS=0x%X\n", vmread (VMXENC_GUEST_LDTR_ACCESS));
  logger_printf ("VMXENC_GUEST_GDTR_BASE=0x%X\n", vmread (VMXENC_GUEST_GDTR_BASE));
  logger_printf ("VMXENC_GUEST_GDTR_LIMIT=0x%X\n", vmread (VMXENC_GUEST_GDTR_LIMIT));
  logger_printf ("VMXENC_GUEST_IDTR_BASE=0x%X\n", vmread (VMXENC_GUEST_IDTR_BASE));
  logger_printf ("VMXENC_GUEST_IDTR_LIMIT=0x%X\n", vmread (VMXENC_GUEST_IDTR_LIMIT));
  logger_printf ("VMXENC_GUEST_RIP=0x%X\n", vmread (VMXENC_GUEST_RIP));
  logger_printf ("VMXENC_GUEST_RSP=0x%X\n", vmread (VMXENC_GUEST_RSP));
  logger_printf ("VMXENC_GUEST_IA32_SYSENTER_CS=0x%X\n", vmread (VMXENC_GUEST_IA32_SYSENTER_CS));
  logger_printf ("VMXENC_GUEST_IA32_SYSENTER_ESP=0x%X\n", vmread (VMXENC_GUEST_IA32_SYSENTER_ESP));
  logger_printf ("VMXENC_GUEST_IA32_SYSENTER_EIP=0x%X\n", vmread (VMXENC_GUEST_IA32_SYSENTER_EIP));
  logger_printf ("VMXENC_VMCS_LINK_PTR_HIGH=0x%X\n", vmread (VMXENC_VMCS_LINK_PTR));
  logger_printf ("VMXENC_VMCS_LINK_PTR_LOW=0x%X\n", vmread (VMXENC_VMCS_LINK_PTR + 1));
  logger_printf ("VMXENC_GUEST_PENDING_DEBUG_EXCEPTIONS=0x%X\n", vmread (VMXENC_GUEST_PENDING_DEBUG_EXCEPTIONS));
  logger_printf ("VMXENC_GUEST_ACTIVITY=0x%X\n", vmread (VMXENC_GUEST_ACTIVITY));
  logger_printf ("VMXENC_GUEST_INTERRUPTIBILITY=0x%X\n", vmread (VMXENC_GUEST_INTERRUPTIBILITY));
  logger_printf ("VMXENC_EXCEPTION_BITMAP=0x%X\n", vmread (VMXENC_EXCEPTION_BITMAP));
  logger_printf ("VMXENC_PAGE_FAULT_ERRCODE_MASK=0x%X\n", vmread (VMXENC_PAGE_FAULT_ERRCODE_MASK));
  logger_printf ("VMXENC_PAGE_FAULT_ERRCODE_MATCH=0x%X\n", vmread (VMXENC_PAGE_FAULT_ERRCODE_MATCH));
  logger_printf ("VMXENC_CR0_GUEST_HOST_MASK=0x%X\n", vmread (VMXENC_CR0_GUEST_HOST_MASK));
  logger_printf ("VMXENC_CR0_READ_SHADOW=0x%X\n", vmread (VMXENC_CR0_READ_SHADOW));
  logger_printf ("---------------------------\n");
  logger_printf ("|VMX GUEST STATE DUMP DONE|\n");
  logger_printf ("---------------------------\n");
#endif

 enter:
  RDTSC (start);

  /* clobber-list is not necessary here because "pusha" below saves
   * HOST registers */
  asm volatile (/* save HOST registers on stack and ESP in VMCS */
                "pusha\n"
                "vmwrite %%esp, %2\n"
                /* Do trick to get current EIP and differentiate between the first
                 * and second time this code is invoked. */
                "call 1f\n"
                /* On VM-EXIT, resume Host here: */
                "pusha\n"       /* quickly snapshot guest registers to stack */
                "addl $0x20, %%esp\n"
                "popa\n"        /* temporarily restore host registers */
                "lea %1, %%edi\n"
                "movl $8, %%ecx\n"
                "lea -0x40(%%esp), %%esi\n"
                "cld; rep movsd\n" /* save guest registers to memory */
                "subl $0x20, %%esp\n"
                "popa\n"        /* permanently restore host registers */
                "xor %0, %0\n"
                "jmp 2f\n"
                "1:\n"
                "pop %0\n"
                "2:\n"
                :"=r" (eip):"m" (vm->guest_regs),"r" (VMXENC_HOST_RSP));

  /* VM-ENTER */
  if (eip) {
    DLOG ("Entering VM! host EIP=0x%p", eip);
    vmwrite (eip, VMXENC_HOST_RIP);
    /* Allow shared driver access in sandboxes */
    shared_driver_available = TRUE;
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

    shared_driver_available = FALSE;

    /* Must check if CF=1 or ZF=1 before doing anything else.
     * However, ESP is wiped out.  To restore stack requires a VMREAD.
     * However that would clobber flags.  Therefore, we must check
     * condition codes using "JBE" first.  Then we can restore stack,
     * and also host registers. */

    /* This may be unnecessary, should not reach this point except on error. */
    asm volatile ("xorl %%edi, %%edi; jbe 1f; jmp 2f\n"
                  "1: movl $1, %%edi\n"
                  "2: vmread %1, %%esp; pushl %%edi; addl $4, %%esp\n"
                  "popa\n"
                  /* alt. could modify EDI on stack.. oh well. */
                  "subl $0x24, %%esp\npopl %%edi\naddl $0x20, %%esp"
                  :"=D" (err)
                  :"r" (VMXENC_HOST_RSP));
    if (err) {
#if DEBUG_VMX > 1
      uint32 error = vmread (VMXENC_VM_INSTR_ERROR);
#endif
      uint32 reason = vmread (VMXENC_EXIT_REASON);

      if (state == 1)
        /* Failure to VMLAUNCH */
        vm->launched = FALSE;

#if DEBUG_VMX > 1
      logger_printf ("VM-ENTRY: %d error: %.8X (%s)\n  reason: %.8X qual: %.8X\n",
                     err,
                     error,
                     (error < VMX_NUM_INSTR_ERRORS ? vm_instruction_errors[error] : "n/a"),
                     reason,
                     vmread (VMXENC_EXIT_QUAL));
#endif
      if (reason & 0x80000000) {
#if DEBUG_VMX > 0
        logger_printf ("  VM-ENTRY failure, code: %d\n", reason & 0xFF);
#endif
      }
      goto abort_vmentry;
    }
  }

  if (!eip) {
    /* VM-exited */
    uint32 reason = vmread (VMXENC_EXIT_REASON);
    uint32 intinf = vmread (VMXENC_VM_EXIT_INTERRUPT_INFO);
#if DEBUG_VMX > 1
    uint32 qualif = vmread (VMXENC_EXIT_QUAL);
    uint32 ercode = vmread (VMXENC_VM_EXIT_INTERRUPT_ERRCODE);
    uint32 inslen = vmread (VMXENC_VM_EXIT_INSTR_LEN);
    uint32 insinf = vmread (VMXENC_VM_EXIT_INSTR_INFO);
    uint32 exit_eflags = 0;
    asm volatile ("pushfl; pop %0\n":"=r" (exit_eflags):);
#endif

    /* Cannot use shared driver in monitor */
    shared_driver_available = FALSE;

    if (reason & (1 << 31)) {
      /* VM-exit was due to failure during checking of Guest state
       * during VM-entry */
      reason &= ~(1 << 31);
      if (state == 1)
        /* Failure to VMLAUNCH */
        vm->launched = FALSE;
    }

    RDTSC (finish);

#if DEBUG_VMX > 2
    logger_printf ("VM-EXIT: %s\n  reason=%.8X qualif=%.8X\n  intinf=%.8X ercode=%.8X\n  inslen=%.8X insinf=%.8X\n  guestphys=0x%llX guestlinear=0x%llX\n  cycles=0x%llX cpu=%d\n",
                   (reason < VMX_NUM_EXIT_REASONS ?
                    vm_exit_reasons[reason] : "invalid exit-reason"),
                   reason, qualif, intinf, ercode, inslen, insinf,
                   (u64) vmread (VMXENC_GUEST_PHYS_ADDR),
                   (u64) vmread (VMXENC_GUEST_LINEAR_ADDR),
                   finish - start, get_pcpu_id ());
    vmx_vm_exit_reason ();
    u32 rip = vmread (VMXENC_GUEST_RIP), rsp = vmread (VMXENC_GUEST_RSP);
    logger_printf ("VM-EXIT: GUEST-STATE: RIP=0x%p RSP=0x%p\n", rip, rsp);
    logger_printf ("VM-EXIT: GUEST-STATE: FLAGS=0x%p CR0=0x%p CR3=0x%p CR4=0x%p\n",
                   vmread (VMXENC_GUEST_RFLAGS),
                   vmread (VMXENC_GUEST_CR0),
                   //vmread (VMXENC_GUEST_CR2),
                   vmread (VMXENC_GUEST_CR3),
                   vmread (VMXENC_GUEST_CR4));
#define SHOWSEG(seg) do {                                               \
      logger_printf ("VM-EXIT: GUEST-STATE: %s=0x%.02X base=0x%p limit=0x%p access=0x%p\n", \
                     __stringify (seg),                                 \
                     vmread (VMXENC_GUEST_##seg##_SEL),                 \
                     vmread (VMXENC_GUEST_##seg##_BASE),                \
                     vmread (VMXENC_GUEST_##seg##_LIMIT),               \
                     vmread (VMXENC_GUEST_##seg##_ACCESS)               \
                     );                                                 \
    } while (0)
#define SHOWDTR(seg) do {                                               \
      logger_printf ("VM-EXIT: GUEST-STATE: %s base=0x%p limit=0x%p\n", \
                     __stringify (seg),                                 \
                     vmread (VMXENC_GUEST_##seg##_BASE),                \
                     vmread (VMXENC_GUEST_##seg##_LIMIT)                \
                     );                                                 \
    } while (0)

    SHOWSEG (CS);
    SHOWSEG (SS);
    SHOWSEG (DS);
    SHOWSEG (ES);
    SHOWSEG (FS);
    SHOWSEG (GS);
    SHOWSEG (TR);
    SHOWSEG (LDTR);
    SHOWDTR (GDTR);
    SHOWDTR (IDTR);

#endif

    if (vm->realmode && reason == 0x0 && (intinf & 0xFF) == 0x0D) {
      /* General Protection Fault in vm86 mode */
      if (vmx_vm86_handle_GPF (vm) == 0)
        /* continue guest */
        goto enter;
    } else if (reason == 0x0A) {
      /* We use CPUID as a way of intentional VM-Exit in Guest too */
      if (vm->guest_regs.eax == 0xFFFFFFFF) {
        vmx_process_exit (vm->guest_regs.ecx);
        vmwrite (vmread (VMXENC_GUEST_RIP) + inslen, VMXENC_GUEST_RIP); /* skip instruction */
        goto enter;
      }
      /* CPUID -- unconditional VM-EXIT -- perform in monitor */
      logger_printf ("VM: performing CPUID (0x%p, 0x%p) => ",
                     vm->guest_regs.eax, vm->guest_regs.ecx);
      cpuid (vm->guest_regs.eax, vm->guest_regs.ecx,
             &vm->guest_regs.eax, &vm->guest_regs.ebx, &vm->guest_regs.ecx, &vm->guest_regs.edx);
      logger_printf ("(0x%p, 0x%p, 0x%p, 0x%p)\n",
                     vm->guest_regs.eax, vm->guest_regs.ebx,
                     vm->guest_regs.ecx, vm->guest_regs.edx);
      vmwrite (vmread (VMXENC_GUEST_RIP) + inslen, VMXENC_GUEST_RIP); /* skip instruction */
      goto enter;               /* resume guest */
    } else if (reason == 0x1F || reason == 0x20) {
      /* RDMSR / WRMSR -- conditional on MSR bitmap -- else perform in monitor */
      logger_printf ("VM: use MSR bitmaps=%d MSR_BITMAPS=0x%p bitmap[0x%X]=%d\n",
                     !!(vmread (VMXENC_PROCBASED_VM_EXEC_CTRLS) & (1<<28)),
                     vmread (VMXENC_MSR_BITMAPS),
                     vm->guest_regs.ecx,
                     !!(BITMAP_TST (msr_bitmaps, vm->guest_regs.ecx)));
      if (reason == 0x1F) {
        logger_printf ("VM: performing RDMSR (0x%p) => ", vm->guest_regs.ecx);
        asm volatile ("rdmsr"
                      :"=d" (vm->guest_regs.edx), "=a" (vm->guest_regs.eax)
                      :"c" (vm->guest_regs.ecx));
        logger_printf ("0x%p %p\n", vm->guest_regs.edx, vm->guest_regs.eax);
      }
      if (reason == 0x20) {
        logger_printf ("VM: performing WRMSR (0x%p %p,0x%p)\n",
                        vm->guest_regs.edx, vm->guest_regs.eax, vm->guest_regs.ecx);
        asm volatile ("wrmsr"::"d" (vm->guest_regs.edx), "a" (vm->guest_regs.eax), "c" (vm->guest_regs.ecx));
      }
      vmwrite (vmread (VMXENC_GUEST_RIP) + inslen, VMXENC_GUEST_RIP); /* skip instruction */
      goto enter;               /* resume guest */
#ifdef VMX_EPT
    } else if (reason == 0x31) {
      /* EPT misconfiguration */
      logger_printf ("EPT misconfiguration:\n  VMXENC_EPT_PTR=0x%p\n",
                     vmread (VMXENC_EPT_PTR));
#endif
    } else {
      /* Not a vm86 related VM-EXIT */
#if DEBUG_VMX > 1
      logger_printf ("VM-EXIT: %s\n  reason=%.8X qualif=%.8X\n  intinf=%.8X ercode=%.8X\n  inslen=%.8X insinf=%.8X\n",
                     (reason < VMX_NUM_EXIT_REASONS ?
                      vm_exit_reasons[reason] : "invalid exit-reason"),
                     reason, qualif, intinf, ercode, inslen, insinf);
      logger_printf ("VM-EXIT: EFLAGS=0x%.8X\n", exit_eflags);
      vmx_vm_exit_reason ();
#endif
    }
  }

  DLOG ("start_VM: return 0 -- giving up on virtual machine (cpu#%d)", get_pcpu_id ());
  crash_debug ("stack is probably corrupt now");
  /* control could be resumed where the VM failed.  maybe do this later. */

  return 0;
 abort_vmentry:
 not_loaded:
  return -1;
}

/* start VM guest with state derived from host state */
int
vmx_enter_pmode_VM (virtual_machine *vm)
{
  u32 guest_eip = 0, esp, ebp, cr3;
  u32 hyperstack_frame = alloc_phys_frame ();
  if (hyperstack_frame == (u32) -1) return -1;
  u32 *hyperstack = map_virtual_page (hyperstack_frame | 3);
  if (hyperstack == 0) return -1;

  asm volatile ("call 1f\n"
                /* RESUME POINT */
                "xorl %0, %0\n"
                "jmp 2f\n"
                "1: pop %0; movl %%esp, %1\n"
                "2:":"=r" (guest_eip), "=r" (esp));
  if (guest_eip == 0) {
    /* inside VM  */
    asm volatile ("movl %%esp, %0; movl %%ebp, %1":"=r" (esp), "=r" (ebp));
    asm volatile ("movl %%cr3, %0":"=r" (cr3));
    DLOG ("vmx_enter_pmode_VM: entry success ESP=0x%p EBP=0x%p CR3=0x%p", esp, ebp, cr3);
    //dump_page ((u8 *) (esp & (~0xFFF)));

#if 1
    uint32 cpu = get_pcpu_id ();
    print ("Message Printed to screen:\n");
    switch (cpu) {
      case 0 :
        print ("Welcome to SeQuest Sandbox 0!\n");
        break;
      case 1 :
        print ("Welcome to SeQuest Sandbox 1!\n");
        break;
      case 2 :
        print ("Welcome to SeQuest Sandbox 2!\n");
        break;
      case 3 :
        print ("Welcome to SeQuest Sandbox 3!\n");
        break;
      default:
        print ("Unknown Sandbox!\n");
    }
#endif

#if 0
#include "vm/shm.h"
    /* Shared memory test */
    uint32 frame1, frame2;
    uint32 *vframe1, *vframe2;
    spinlock *test_lock1, *test_lock2;
    test_lock1 = shm_alloc_drv_lock ();
    test_lock2 = shm_alloc_drv_lock ();
    spinlock_lock (test_lock1);
    spinlock_lock (test_lock2);
    spinlock_unlock (test_lock1);
    spinlock_unlock (test_lock2);
    shm_free_drv_lock (test_lock1);
    shm_free_drv_lock (test_lock2);

    frame1 = shm_alloc_phys_frame ();
    vframe1 = map_virtual_page (frame1 | 3);
    frame2 = shm_alloc_phys_frame ();
    vframe2 = map_virtual_page (frame2 | 3);
    *vframe1 = 0xFFFFFFFF;
    *vframe2 = 0xDEADBEEF;
    logger_printf ("(0x%x, 0x%x)\n", *vframe1, *vframe2);
    unmap_virtual_page (vframe1);
    unmap_virtual_page (vframe2);
    shm_free_phys_frame (frame1);
    shm_free_phys_frame (frame2);
#endif

    return 0;
  }

  /* save general registers for guest */
  asm volatile ("pusha; movl %%esp, %%esi; movl $0x20, %%ecx; rep movsb; popa"
                ::"D" (&vm->guest_regs));

  /* copy stack */
  memcpy (hyperstack, (void *) (esp & (~0xFFF)), 0x1000);
  /* change frame pointer in host to hypervisor stack */
  asm volatile ("movl %%ebp, %0":"=r" (ebp));
  ebp = (((u32) &hyperstack) & (~0xFFF)) | (ebp & 0xFFF);
  asm volatile ("movl %0, %%ebp"::"r" (ebp));
  /* switch host stack to hypervisor stack */
  asm volatile ("movl %0, %%esp"::"r" (&hyperstack[(esp & 0xFFF) >> 2]));

  /* hypervisor stack now in effect */

  /* set guest to continue from resume point above */
  vmwrite (guest_eip, VMXENC_GUEST_RIP);
  /* guest takes over original stack */
  vmwrite (esp, VMXENC_GUEST_RSP);

  logger_printf ("vmx_enter_pmode_VM: GUEST-STATE: RIP=0x%p RSP=0x%p RBP=0x%p\n",
                 vmread (VMXENC_GUEST_RIP), vmread (VMXENC_GUEST_RSP), vm->guest_regs.ebp);
  return vmx_start_VM (vm);
}

void
test_pmode_vm (void)
{
  logger_printf ("INSIDE PMODE VM -- going into infinite loop\n");
  for (;;);
}

static virtual_machine VMs[MAX_CPUS] ALIGNED (0x1000);
static int num_VMs = 0;
DEF_PER_CPU (virtual_machine *, cpu_vm);

void
vmx_processor_init (void)
{
  uint8 phys_id = get_pcpu_id ();
  DLOG ("processor_init pcpu_id=%d", phys_id);
  uint32 cr0, cr4;
  uint32 *vmxon_virt;
  virtual_machine *vm = &VMs[phys_id];

  if (!vmx_enabled)
    return;

  /* Set the NE bit to satisfy CR0_FIXED0 */
  asm volatile ("movl %%cr0, %0\n"
                "orl $0x20, %0\n"
                "movl %0, %%cr0":"=r" (cr0));

#if DEBUG_VMX > 1
  com1_printf ("IA32_FEATURE_CONTROL: 0x%.8X\n", (uint32) rdmsr (IA32_FEATURE_CONTROL));
  com1_printf ("IA32_VMX_BASIC: 0x%.16llX\n",
               rdmsr (IA32_VMX_BASIC));
  com1_printf ("IA32_VMX_CR0_FIXED0: 0x%.8X\n", (uint32) rdmsr (IA32_VMX_CR0_FIXED0));
  com1_printf ("IA32_VMX_CR0_FIXED1: 0x%.8X\n", (uint32) rdmsr (IA32_VMX_CR0_FIXED1));
  com1_printf ("IA32_VMX_CR4_FIXED0: 0x%.8X\n", (uint32) rdmsr (IA32_VMX_CR4_FIXED0));
  com1_printf ("IA32_VMX_CR4_FIXED1: 0x%.8X\n", (uint32) rdmsr (IA32_VMX_CR4_FIXED1));

  com1_printf ("IA32_VMX_PINBASED_CTLS: 0x%.16llX\n",
               rdmsr (IA32_VMX_PINBASED_CTLS));
  com1_printf ("IA32_VMX_TRUE_PINBASED_CTLS: 0x%.16llX\n",
               rdmsr (IA32_VMX_TRUE_PINBASED_CTLS));
  com1_printf ("IA32_VMX_PROCBASED_CTLS: 0x%.16llX\n",
               rdmsr (IA32_VMX_PROCBASED_CTLS));
  com1_printf ("IA32_VMX_TRUE_PROCBASED_CTLS: 0x%.16llX\n",
               rdmsr (IA32_VMX_TRUE_PROCBASED_CTLS));
  com1_printf ("IA32_VMX_PROCBASED_CTLS2: 0x%.16llX\n",
               rdmsr (IA32_VMX_PROCBASED_CTLS2));

  com1_printf ("IA32_VMX_EXIT_CTLS: 0x%.16llX\n",
               rdmsr (IA32_VMX_EXIT_CTLS));
  com1_printf ("IA32_VMX_ENTRY_CTLS: 0x%.16llX\n",
               rdmsr (IA32_VMX_ENTRY_CTLS));
  com1_printf ("IA32_VMX_TRUE_EXIT_CTLS: 0x%.16llX\n",
               rdmsr (IA32_VMX_TRUE_EXIT_CTLS));
  com1_printf ("IA32_VMX_TRUE_ENTRY_CTLS: 0x%.16llX\n",
               rdmsr (IA32_VMX_TRUE_ENTRY_CTLS));
  com1_printf ("IA32_VMX_MISC: 0x%.16llX\n",
               rdmsr (IA32_VMX_MISC));
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

  percpu_write (cpu_vm, vm);

  if (vmx_create_pmode_VM (vm, 0, 0) != 0)
    goto vm_error;

  if (vmx_enter_pmode_VM (vm) != 0)
    goto vm_error;

  num_VMs++;

  return;

 vm_error:
  vmxoff ();
 abort_vmxon:
  free_phys_frame (vmxon_frame[phys_id]);
}

bool
vmx_init (void)
{
  vmx_detect ();
  if (!vmx_enabled) {
    DLOG ("VMX not enabled");
    goto vm_error;
  }

  vmx_global_init ();

  vmx_processor_init ();

#if 0
  if (vmx_unload_VM (&first_vm) != 0)
    goto vm_error;
  vmx_destroy_VM (&first_vm);
#endif

  return TRUE;
 vm_error:
  return FALSE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = vmx_init
};

#ifdef USE_VMX
//DEF_MODULE (vm___vmx, "VMX hardware virtualization driver", &mod_ops, {});
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
