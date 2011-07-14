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

/* Function should be called one logical processor at a time */
void
vmx_init_mem (uint32 cpu)
{
  extern uint32 _physicalbootstrapstart;
  extern uint32 _code16start, _code16_pages, _code16physicalstart;

  /* Original Quest Paging Data Structures */
  uint32 phys_cr3 = (uint32) get_pdbr ();
  uint32 *virt_kern_pgt, *vm86_pgt, *virt_pgd;
  uint32 phys_kern_pgt, bios_backup;

  /* New Paging Data Structures */
  uint32 *virt_pgd_new, *virt_kern_pgt_new;

  uint32 cr = 0;
  uint32 physical_offset = SANDBOX_KERN_OFFSET * cpu;
  uint32 physical_page_count = 0;
  uint32 *new_mm_table = 0, new_mm_begin = 0;
  int i;

#if 0
  virt_pgd = map_virtual_page (phys_cr3 | 3);
  for (i = 0; i < 0x400; i++) {
    logger_printf ("PGD Entry %d: %x\n", i, virt_pgd[i]);
  }

  logger_printf ("mm_limit=%d\n", mm_limit);
  for (i = 0; i < mm_limit; i++) {
    if (!BITMAP_TST (mm_table, i)) {
      logger_printf ("Allocated Physical Page: %d\n", i);
    }
  }

  while (1);
#endif

  /* Do not relocate memory for first logical processor */
  if (cpu == 0) {
    logger_printf ("Logical Processor %d, skip relocation.\n", cpu);
    return;
  }

  logger_printf ("Memory Relocation for cpu %d\n", cpu);

  /*
   * In order to rearrange the memory layout for different sandbox
   * kernels, we need to move certain physical memory pages around.
   * To make it simple, we identity map majority of the memories by
   * filling in PDEs other then APIC and kernel, which are entry 1019
   * and 1023 respectively. The first megabyte for BIOS will be wiped
   * out for now but restored later.
   *
   * This should be enough for our purpose now. If the large kernel
   * is used, we can set up a new paging structure for doing this.
   */
  virt_pgd = map_virtual_page (phys_cr3 | 3);
  /* Backup the first directory entry */
  bios_backup = virt_pgd[0];

  for (i = 0; i < (APIC_VIRT_ADDR >> 22); i++) {
    virt_pgd[i] = (i << 22) | 0x83;
  }

  unmap_virtual_page (virt_pgd);
  flush_tlb_all ();

  /* For safety, invalidate all data caches. */
  asm volatile ("wbinvd");

  /* Now begin memory relocation */
  logger_printf ("Relocating kernel to 0x%x for cpu#%d\n", physical_offset, cpu);
  logger_printf ("Duplicating kernel from: 0x%x\n", &_physicalbootstrapstart);
  void *src_page, *des_page;
  uint32 cb = PHYS_PAGE_SIZE;
  for (i = (((uint32)(&_physicalbootstrapstart)) >> 12); i < (SANDBOX_KERN_OFFSET >> 12) +
       (((uint32)(&_physicalbootstrapstart)) >> 12); i++) {
    if (!BITMAP_TST (mm_table, i)) {
      if (!BITMAP_TST (mm_table, i + (SANDBOX_KERN_OFFSET >> 12) * cpu)) {
        logger_printf ("Wiping out memory! CPU#%d\n", get_pcpu_id ());
        while (1);
      }
      /* Physical page allocated, relocate it */
      src_page = (void*) (i << 12);
      des_page = (void*) (((uint32)src_page) + physical_offset);
      /* We'd better not call memcpy here because the stack will be copied too. */
      asm volatile ("cld; rep movsb"
                    : /* Nothing */
                    :"c" (cb), "D" (des_page), "S" (src_page)
                    :"memory","flags");
      /* 
       * Update global physical memory bitmap. This is only for the
       * original kernel. Physical memory subsystem will be configured
       * later when we switch to the new kernel.
       */
      BITMAP_CLR (mm_table, i + (SANDBOX_KERN_OFFSET >> 12) * cpu);
      physical_page_count ++;
      //logger_printf ("Physical Page %d Copied\n", i);
    }
  }

  logger_printf ("%d Physical Pages Relocated for cpu#%d\n", physical_page_count, cpu);

  /* Fix physical memory bitmap for new kernel. Clear the pages for new kernel image. */
  new_mm_table = map_virtual_page (((uint32) get_phys_addr (mm_table) + physical_offset) | 3);
  new_mm_begin = ((uint32)(&_physicalbootstrapstart) + physical_offset) >> 12;
  for (i = new_mm_begin; i < (new_mm_begin + physical_page_count); i++) {
    BITMAP_CLR (new_mm_table, i);
  }
  unmap_virtual_page (new_mm_table);

  /* Change the page table mappings for relocation */
  virt_pgd = map_virtual_page (phys_cr3 | 3);
  virt_pgd_new = map_virtual_page ((phys_cr3 + physical_offset) | 3);
  phys_kern_pgt = virt_pgd[1023] & 0xFFFFF000;
  virt_kern_pgt = map_virtual_page (phys_kern_pgt | 3);
  virt_kern_pgt_new = map_virtual_page (((virt_pgd[1023] & 0xFFFFF000) +
                                        physical_offset) | 3);

#if 0
  logger_printf ("virt_pgd=0x%x\n", virt_pgd);
  logger_printf ("virt_pgd_new=0x%x\n", virt_pgd_new);
  logger_printf ("virt_kern_pgt=0x%x\n", virt_kern_pgt);
  logger_printf ("virt_kern_pgt_new=0x%x\n", virt_kern_pgt_new);

  for (i = 0; i < 0x400; i++) {
    logger_printf ("New PGD Entry %d: %x\n", i, virt_pgd_new[i]);
  }
#endif

  /* Modify the mapping for BIOS */
  virt_pgd_new[0] = ((bios_backup & 0xFFFFF000) + physical_offset) | 7;
  vm86_pgt = map_virtual_page (((bios_backup & 0xFFFFF000) + physical_offset) | 3);

  /* 
   * The identity map should be fine because that part of the physical
   * memory is shared, but pages starting at 0x8000 should be re-mapped.
   * Look at vmx_global_init in vmx.c for detail.
   */
  for (i=0; i<((uint32) &_code16_pages); i++)
    vm86_pgt[((((uint32) &_code16start) >> 12) & 0x3FF) + i] =
    ((uint32) &_code16physicalstart + physical_offset + (i << 12)) | 7;

  unmap_virtual_page (vm86_pgt);

  logger_printf ("BIOS Mapping fixed for cpu#%d\n", cpu);

  /* Modify the mapping for kernel. APIC mapping should be fine. */
  for (i = 0; i < 1024; i++) {
    if (virt_kern_pgt_new[i]) {
      //logger_printf ("Old Kernel Page Table Entry %d: 0x%x\n", i, virt_kern_pgt[i]);
      //logger_printf ("New Kernel Page Table Entry %d: 0x%x\n", i, virt_kern_pgt_new[i]);
      /* If it is kernel screen memory, we keep the original mapping. */
      if (i == ((KERN_SCR >> 12) & 0x3FF)) {
        virt_kern_pgt_new[i] = SCREEN_PHYS | 0x3;
        //logger_printf ("New Entry %d: 0x%x\n", i, virt_kern_pgt_new[i]);
        continue;
      }

      /* For the rest of the kernel, everything is shifted. */
      if (((virt_kern_pgt[i] & 0xFFFFF000) >> 12) < 1024) {
          virt_kern_pgt_new[i] = ((virt_kern_pgt[i] & 0xFFFFF000) + physical_offset) |
              (virt_kern_pgt[i] & 0xF);
      }
      //logger_printf ("New Entry %d: 0x%x\n", i, virt_kern_pgt_new[i]);
    }
  }

  virt_pgd_new[1023] = ((virt_pgd[1023] & 0xFFFFF000) + physical_offset) | 3;

  for (i = 1; i < (APIC_VIRT_ADDR >> 22); i++) {
    virt_pgd_new[i] = 0;
  }

#if 0
  for (i = 0; i < 0x400; i++) {
    logger_printf ("Kernl Page Tabe Entry %d: %x\n", i, virt_kern_pgt_new[i]);
  }
#endif

  logger_printf ("Kernel Mapping fixed for cpu#%d\n", cpu);

  /* Restore Paging Structures for Host */
  virt_pgd[0] = bios_backup;
  for (i = 1; i < (APIC_VIRT_ADDR >> 22); i++) {
    virt_pgd[i] = 0;
  }
  flush_tlb_all ();

  logger_printf ("Host Mapping Restored on cpu#%d\n", cpu);

  unmap_virtual_page (virt_pgd_new);
  unmap_virtual_page (virt_kern_pgt);
  unmap_virtual_page (virt_kern_pgt_new);
  unmap_virtual_page (virt_pgd);

  logger_printf ("phys_cr3=0x%x\n", phys_cr3);
  phys_cr3 += physical_offset;
  logger_printf ("New phys_cr3=0x%x\n", phys_cr3);

  /* 
   * Before switch to new kernel, unlock the kernel lock in old kernel.
   * The kernel lock in new kernel will be unlocked in the idle task.
   */
  unlock_kernel ();

  asm volatile ("movl %0, %%cr3"::"r"(phys_cr3):);

#if 0
  uint32 *new_pgt = (uint32 *) KERN_PGT;
  for (i = 0; i < 1024; i++) {
    logger_printf ("Shifted Kernel Page Table Entry %d: 0x%x\n", i, new_pgt[i]);
  }
  logger_printf ("virt_pgd=0x%x\n", virt_pgd);
  logger_printf ("virt_pgd_new=0x%x\n", virt_pgd_new);
  logger_printf ("virt_kern_pgt=0x%x\n", virt_kern_pgt);
  logger_printf ("virt_kern_pgt_new=0x%x\n", virt_kern_pgt_new);
#endif

  /* Modify the physical memory manager for the new kernel. */
  mm_limit = 256 + ((SANDBOX_KERN_OFFSET * (cpu + 1)) >> 12);
  mm_begin = 256 + (physical_offset >> 12);
  logger_printf ("Physical Memory for CPU %d begins at 0x%x, ends at 0x%x\n",
                 cpu, mm_begin << 12, mm_limit << 12);

  asm volatile ("movl %%cr3, %0":"=r" (cr));
  logger_printf ("Set Host and Guest CR3 to: 0x%x\n", cr);
  vmwrite (cr, VMXENC_HOST_CR3);
  vmwrite (cr, VMXENC_GUEST_CR3);
}

//#define EPT_DEBUG

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
  logger_printf ("pml4=0x%p pdpt=0x%p\n", pml4, pdpt);
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

#if 0
  /* Verify physical memory bitmap */
  logger_printf ("mm_table=%x, mm_begin=%d, mm_limit=%d\n", (uint32) mm_table, mm_begin, mm_limit);
  for (i = mm_begin; i < mm_limit; i++) {
    if (!BITMAP_TST (mm_table, i)) {
      logger_printf ("%d", 1);
    } else {
      logger_printf ("%d", 0);
    }
    if (((i - mm_begin + 1) % 128) == 0) {
      logger_printf ("\n");
    }
  }
#endif

#ifdef EPT_DEBUG
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
#endif

  for (i=0; i < num_var_reg; i++) {
    var_base[i] = rdmsr (IA32_MTRR_PHYS_BASE (i));
    var_mask[i] = rdmsr (IA32_MTRR_PHYS_MASK (i));

#ifdef EPT_DEBUG
    com1_printf ("IA32_MTRR_PHYS_BASE(%d)=0x%llX\nIA32_MTRR_PHYS_MASK(%d)=0x%llX\n",
                 i, var_base[i], i, var_mask[i]);
#endif
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

#ifdef EPT_DEBUG
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
#endif
  }

  for (i=0; i<4; i++) {
    pd_frame = alloc_phys_frame ();
    pd = map_virtual_page (pd_frame | 3);
    logger_printf ("pd_frame=0x%x, pd=0x%x\n", pd_frame, pd);

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

