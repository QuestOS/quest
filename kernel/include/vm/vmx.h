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

#ifndef _VMX_H_
#define _VMX_H_
#include "types.h"
#include "vmx-defs.h"
#include "smp/spinlock.h"
#include "vm/hypercall.h"

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

extern bool shared_driver_available;

typedef struct
{
  /* Order based on PUSHA/POPA */
  uint32 edi, esi, ebp, esp, ebx, edx, ecx, eax;
} gp_registers;

typedef struct
{
  uint32 vmcs_frame, current_cpu;
  void *guest_stack;
  gp_registers guest_regs;
  uint32 launched, loaded, realmode;
} virtual_machine;

#define VM_REG(n) ((((uint32 *) (&vm->guest_regs)))[7 - (n)])
static const char gp_register_names[][4] = {
   "edi", "esi", "ebp", "esp", "ebx", "edx", "ecx", "eax"
};
#define VM_REG_NAME(n) (gp_register_names[7 - n])

void vmx_detect (void);
int vmx_create_monitor (virtual_machine *, u32, u32);
int vmx_destroy_monitor (virtual_machine *);
int vmx_load_monitor (virtual_machine *);
int vmx_unload_monitor (virtual_machine *);
int vmx_enter_guest (virtual_machine *);
int vmx_start_monitor (virtual_machine *);

static inline int
vmx_get_error (void)
{
  uint32 flags;
  asm volatile ("pushfl\n"
                "pop %0"
                :"=r" (flags));
  if (flags & (1<<0))
    return -1;
  if (flags & (1<<6))
    return 1;
  return 0;
}

static inline void
vmxon (uint32 frame)
{
  uint64 frame64 = (uint64)frame & 0xFFFFFFFF;
  asm volatile ("vmxon %0"::"m" (frame64):"flags");
}

static inline void
vmclear (uint32 frame)
{
  uint64 frame64 = (uint64)frame & 0xFFFFFFFF;
  asm volatile ("vmclear %0"::"m" (frame64):"flags");
}

static inline void
vmptrld (uint32 frame)
{
  uint64 frame64 = (uint64)frame & 0xFFFFFFFF;
  asm volatile ("vmptrld %0"::"m" (frame64):"flags");
}

static inline uint32
vmptrst (void)
{
  uint64 frame64;
  asm volatile ("vmptrst %0":"=m" (frame64));
  return (uint32) frame64;
}

static inline uint32
vmread (uint32 encoding)
{
  uint32 value;
  asm volatile ("vmread %1, %0":"=r" (value):"r" (encoding):"flags");
  return value;
}

static inline void
vmwrite (uint32 val, uint32 encoding)
{
  asm volatile ("vmwrite %0, %1"::"r" (val), "r" (encoding):"flags");
}

static inline void
vmwrite64 (uint64 val, uint32 encoding)
{
  uint32 lo = (uint32) val, hi = (uint32) (val >> 32);
  asm volatile ("vmwrite %0, %1"::"r" (lo), "r" (encoding):"flags");
  asm volatile ("vmwrite %0, %1"::"r" (hi), "r" (encoding+1):"flags");
}

#define CLOBBERS "cc","memory","%eax","%ebx","%ecx","%edx","%esi","%edi","%esp"

static inline void
vmlaunch (void)
{
  asm volatile ("vmlaunch":::CLOBBERS);
}

static inline void
vmresume (void)
{
  asm volatile ("vmresume":::CLOBBERS);
}

#undef CLOBBERS

static inline void
vmxoff (void)
{
  asm volatile ("vmxoff");
}

static inline void
vm_exit (uint32 status)
{
  asm volatile ("cpuid"::"a" (0xFFFFFFFF), "c" (status));
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
