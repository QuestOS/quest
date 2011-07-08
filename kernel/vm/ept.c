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

#if DEBUG_EPT > 0
#define DLOG(fmt,...) DLOG_PREFIX("ept",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define com1_printf logger_printf

void
vmx_init_ept (uint32 cpu)
{
  logger_printf ("Initializing EPT data structures on CPU#%d...\n", cpu);

  vmwrite ((1 << 1)             /* EPT */
           , VMXENC_PROCBASED_VM_EXEC_CTRLS2);
  u32 i,j,k;
  u32 pml4_frame = alloc_phys_frame ();
  u32 pdpt_frame = alloc_phys_frame ();
  u32 pd_frame;
  u32 pt_frame;
  logger_printf ("pml4_frame=0x%p pdpt_frame=0x%p\n", pml4_frame, pdpt_frame);
  u64 *pml4 = map_virtual_page (pml4_frame | 3);
  u64 *pdpt = map_virtual_page (pdpt_frame | 3);
  u64 *pd;
  u64 *pt;
  memset (pml4, 0, 0x1000);
  memset (pdpt, 0, 0x1000);
  pml4[0] = pdpt_frame | 7;

  u8 memtype;
  u8 def_memtype;
  u64 mtrr_cap = rdmsr (IA32_MTRRCAP);
  u64 mtrr_def_type = rdmsr (IA32_MTRR_DEF_TYPE);
  def_memtype = (u8) mtrr_def_type;
  u8 num_var_reg = (u8) mtrr_cap;
  bool fix_supported = (mtrr_cap >> 8) & 0x01;
  u32 var_regs_frame = alloc_phys_frame ();
  /* Allocate 4K page for variable range bases and masks */
  u64 *var_base = map_virtual_page (var_regs_frame | 3);
  u64 *var_mask = &var_base[MAX_MTRR_VAR_REGS];

  u64 msr;
  com1_printf ("IA32_VMX_EPT_VPID_CAP: 0x%.16llX\n",
               msr=rdmsr (IA32_VMX_EPT_VPID_CAP));
  com1_printf ("  %s%s%s%s%s\n",
               msr & (1 << 6) ? "(page-walk=4) ":" ",
               msr & (1 << 8) ? "(support-UC) ":" ",
               msr & (1 << 14) ? "(support-WB) ":" ",
               msr & (1 << 16) ? "(2MB-pages) ":" ",
               msr & (1 << 17) ? "(1GB-pages) ":" ");

  com1_printf ("IA32_MTRRCAP: 0x%llX\n", mtrr_cap);
  com1_printf ("IA32_MTRR_DEF_TYPE: 0x%llX\n", mtrr_def_type);

  for (i=0; i < num_var_reg; i++) {
    var_base[i] = rdmsr (IA32_MTRR_PHYS_BASE (i));
    var_mask[i] = rdmsr (IA32_MTRR_PHYS_MASK (i));

    com1_printf ("IA32_MTRR_PHYS_BASE(%d)=0x%llX\nIA32_MTRR_PHYS_MASK(%d)=0x%llX\n",
                 i, var_base[i], i, var_mask[i]);
  }

  u64 mtrr_fix64k = 0;
  u64 mtrr_fix16k8 = 0;
  u64 mtrr_fix16kA = 0;
  u64 mtrr_fix4kC0 = 0;
  u64 mtrr_fix4kC8 = 0;
  u64 mtrr_fix4kD0 = 0;
  u64 mtrr_fix4kD8 = 0;
  u64 mtrr_fix4kE0 = 0;
  u64 mtrr_fix4kE8 = 0;
  u64 mtrr_fix4kF0 = 0;
  u64 mtrr_fix4kF8 = 0;

  if (fix_supported) {
    mtrr_fix64k = rdmsr (IA32_MTRR_FIX64K_00000);
    mtrr_fix16k8 = rdmsr (IA32_MTRR_FIX16K_80000);
    mtrr_fix16kA = rdmsr (IA32_MTRR_FIX16K_A0000);
    mtrr_fix4kC0 = rdmsr (IA32_MTRR_FIX4K_C0000);
    mtrr_fix4kC8 = rdmsr (IA32_MTRR_FIX4K_C8000);
    mtrr_fix4kD0 = rdmsr (IA32_MTRR_FIX4K_D0000);
    mtrr_fix4kD8 = rdmsr (IA32_MTRR_FIX4K_D8000);
    mtrr_fix4kE0 = rdmsr (IA32_MTRR_FIX4K_E0000);
    mtrr_fix4kE8 = rdmsr (IA32_MTRR_FIX4K_E8000);
    mtrr_fix4kF0 = rdmsr (IA32_MTRR_FIX4K_F0000);
    mtrr_fix4kF8 = rdmsr (IA32_MTRR_FIX4K_F8000);

    com1_printf ("IA32_MTRR_FIX64K_00000=0x%llX\n", mtrr_fix64k);
    com1_printf ("IA32_MTRR_FIX16K_80000=0x%llX\n", mtrr_fix16k8);
    com1_printf ("IA32_MTRR_FIX16K_A0000=0x%llX\n", mtrr_fix16kA);
    com1_printf ("IA32_MTRR_FIX4K_C0000=0x%llX\n", mtrr_fix4kC0);
    com1_printf ("IA32_MTRR_FIX4K_C8000=0x%llX\n", mtrr_fix4kC8);
    com1_printf ("IA32_MTRR_FIX4K_D0000=0x%llX\n", mtrr_fix4kD0);
    com1_printf ("IA32_MTRR_FIX4K_D8000=0x%llX\n", mtrr_fix4kD8);
    com1_printf ("IA32_MTRR_FIX4K_E0000=0x%llX\n", mtrr_fix4kE0);
    com1_printf ("IA32_MTRR_FIX4K_E8000=0x%llX\n", mtrr_fix4kE8);
    com1_printf ("IA32_MTRR_FIX4K_F0000=0x%llX\n", mtrr_fix4kF0);
    com1_printf ("IA32_MTRR_FIX4K_F8000=0x%llX\n", mtrr_fix4kF8);
  }

  for (i=0; i<4; i++) {
    pd_frame = alloc_phys_frame ();
    pd = map_virtual_page (pd_frame | 3);

    for (j=0; j<512; j++) {
      if (i < 3) {
        memtype = 6;
      } else {
        if (j < 128)
          memtype = 6;
        else
          memtype = 0;
      }
      
      /* First 1MB should be treated specially */
      if (i == 0 && j == 0) {
        pt_frame = alloc_phys_frame ();
        pt = map_virtual_page (pt_frame | 3);

        for (k = 0; k < 512; k++) {
          if (k < 160) {
            memtype = 6;
          } else if (k >= 192 && k < 204) {
            memtype = 5;
          } else if (k >= 236 && k < 256) {
            memtype = 5;
          } else if (k >= 256) {
            memtype = 6;
          } else {
            memtype = 0;
          }

          pt[k] = (k << 12) | (memtype << 3) | 7;
        }

        unmap_virtual_page (pt);
        pd[j] = pt_frame | (0 << 7) | 7;
      } else {
        pd[j] = ((i << 30) + (j << 21)) | (1 << 7) | (memtype << 3) | 7;
      }
    }
    logger_printf ("pd[0]=0x%llX\n", pd[0]);
    unmap_virtual_page (pd);
    pdpt[i] = pd_frame | (0 << 7) | 7;
    logger_printf ("pdpt[%d]=0x%llX\n", i, pdpt[i]);
  }

  vmwrite (pml4_frame | (3 << 3) | 6, VMXENC_EPT_PTR);
  vmwrite (0, VMXENC_EPT_PTR_HI);
  logger_printf ("VMXENC_EPT_PTR=0x%p pml4[0]=0x%llX pdpt[0]=0x%llX\n",
                 vmread (VMXENC_EPT_PTR), pml4[0], pdpt[0]);

  unmap_virtual_page (var_base);
  unmap_virtual_page (pml4);
  unmap_virtual_page (pdpt);
}

