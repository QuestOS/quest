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
#include "vm/shm.h"
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
#include "vm/spow2.h"

#define DEBUG_EPT    0
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
    /* Initialize shared memory so that we can use the global lock */
    shm_init (cpu);
    DLOG ("Logical Processor %d, skip relocation.", cpu);

#if 0
    int c;
    uint32 spage = shm_alloc_phys_frame ();
    logger_printf ("Got shared page: 0x%x\n", spage);
    uint32 *message = map_virtual_page (spage | 3);
    char *s = "This is message from BSP!";
    for (c = 0;;c++) {
      if (s[c] == '\0') {
        *(((char*)message) + c) = s[c];
        break;
      } else {
        *(((char*)message) + c) = s[c];
      }
    }
    logger_printf ("Message sent to other sandboxes! Content is: %s\n", message);
    unmap_virtual_page (message);
#endif

    return;
  }

  DLOG ("Memory Relocation for cpu %d", cpu);

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
  DLOG ("Relocating kernel to 0x%x for cpu#%d", physical_offset, cpu);
  DLOG ("Duplicating kernel from: 0x%x", &_physicalbootstrapstart);
  void *src_page, *des_page;
  uint32 cb = PHYS_PAGE_SIZE;
  /*
   * The EPT data structures are not copied because they are private to each
   * sandbox kernel. And vmx_init_ept will do that later for each sandbox.
   */
  for (i = (((uint32)(&_physicalbootstrapstart)) >> 12); i < (SANDBOX_KERN_OFFSET >> 12) +
       (((uint32)(&_physicalbootstrapstart)) >> 12) - (EPT_DATA_SIZE >> 12); i++) {
    if (!BITMAP_TST (mm_table, i)) {
      if (!BITMAP_TST (mm_table, i + (SANDBOX_KERN_OFFSET >> 12) * cpu)) {
        com1_printf ("Wiping out memory! CPU#%d\n", get_pcpu_id ());
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

  DLOG ("%d Physical Pages Relocated for cpu#%d", physical_page_count, cpu);

  /*
   * Fix physical memory bitmap for new kernel. Clear the pages for new kernel image.
   * Notice that the physical memory bitmap itself takes up 32 4KB physical pages.
   */
  new_mm_table = map_contiguous_virtual_pages (
          ((uint32) get_phys_addr (mm_table) + physical_offset) | 3, 32);
  new_mm_begin = ((uint32)(&_physicalbootstrapstart) + physical_offset) >> 12;
  for (i = new_mm_begin; i < (new_mm_begin + physical_page_count); i++) {
    BITMAP_CLR (new_mm_table, i);
  }
  unmap_virtual_pages (new_mm_table, 32);

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

  DLOG ("BIOS Mapping fixed for cpu#%d", cpu);

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
      if (((virt_kern_pgt[i] & 0xFFFFF000) >> 12) <
           ((SANDBOX_KERN_OFFSET >> 12) + 256 /* BIOS */)) {
          virt_kern_pgt_new[i] = ((virt_kern_pgt[i] & 0xFFFFF000) + physical_offset) |
              (virt_kern_pgt[i] & 0xF);
      }
      //logger_printf ("New Entry %d: 0x%x\n", i, virt_kern_pgt_new[i]);
    }
  }

  virt_pgd_new[1023] = ((virt_pgd[1023] & 0xFFFFF000) + physical_offset) | 3;

  /*
   * Clear mappings in page table except APIC and shared component memory allocator
   * memory pool. See vm/spow2.h for detail. This pool might be removed in the future.
   * But for now, we need some continuous virtual memory consistently mapped for each
   * sandbox. This way, shared driver efficiency can be improved.
   */
  for (i = 1; i < (APIC_VIRT_ADDR >> 22); i++) {
    if ((i >= (PHY_SHARED_MEM_POOL_START >> 22)) ||
        (i < ((PHY_SHARED_MEM_POOL_START + SHARED_MEM_POOL_SIZE) >> 22))) {
      virt_pgd_new[i] = ((i << 22) + 0x83);
    } else {
      virt_pgd_new[i] = 0;
    }
  }

#if 0
  for (i = 0; i < 0x400; i++) {
    logger_printf ("Kernl Page Tabe Entry %d: %x\n", i, virt_kern_pgt_new[i]);
  }
#endif

  DLOG ("Kernel Mapping fixed for cpu#%d", cpu);

  /* Restore Paging Structures for Host */
  virt_pgd[0] = bios_backup;
  for (i = 1; i < (APIC_VIRT_ADDR >> 22); i++) {
    if ((i >= (PHY_SHARED_MEM_POOL_START >> 22)) ||
        (i < ((PHY_SHARED_MEM_POOL_START + SHARED_MEM_POOL_SIZE) >> 22))) {
      virt_pgd_new[i] = ((i << 22) + 0x83);
    } else {
      virt_pgd_new[i] = 0;
    }
  }
  flush_tlb_all ();

  DLOG ("Host Mapping Restored on cpu#%d", cpu);

  unmap_virtual_page (virt_pgd_new);
  unmap_virtual_page (virt_kern_pgt);
  unmap_virtual_page (virt_kern_pgt_new);
  unmap_virtual_page (virt_pgd);

  DLOG ("phys_cr3=0x%x", phys_cr3);
  phys_cr3 += physical_offset;
  DLOG ("New phys_cr3=0x%x", phys_cr3);

  /* 
   * Before switch to new kernel, unlock the kernel lock in old kernel.
   * The kernel lock in new kernel will be unlocked in the idle task.
   */
  unlock_kernel ();

  asm volatile ("movl %0, %%cr3"::"r"(phys_cr3));

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

  /* Disable shared driver before entering sandbox */
  shared_driver_available = FALSE;

  /* Initialize shared memory so that we can use the global lock */
  shm_init (cpu);

#if 0
  char *m =
    map_virtual_page ((PHYS_SHARED_MEM_HIGH - SHARED_MEM_SIZE) | 3);
  logger_printf ("CPU %d Get message: %s\n", cpu, m);
  unmap_virtual_page (m);
#endif

  /* Modify the physical memory manager for the new kernel. */
  mm_limit = 256 + ((SANDBOX_KERN_OFFSET * (cpu + 1)) >> 12);
  mm_begin = 256 + (physical_offset >> 12);
  DLOG ("Physical Memory for CPU %d begins at 0x%x, ends at 0x%x",
        cpu, mm_begin << 12, mm_limit << 12);

  asm volatile ("movl %%cr3, %0":"=r" (cr));
  DLOG ("Set Host and Guest CR3 to: 0x%x", cr);
  vmwrite (cr, VMXENC_HOST_CR3);
  vmwrite (cr, VMXENC_GUEST_CR3);
}

//#define MTRR_DEBUG

/*
 * vmx_init_ept should be called after the current sandbox kernel
 * switched to the new physical kernel image, namely, after calling
 * vmx_init_mem.
 */
void
vmx_init_ept (uint32 cpu)
{
  DLOG ("Initializing EPT data structures on CPU#%d...", cpu);

  extern uint32 _shared_driver_data_physical, _shared_driver_data_pages;
  extern uint32 _shared_driver_bss_physical, _shared_driver_bss_pages;
  extern uint32 _physicalbootstrapstart, _physicalkernelstart;
  extern uint32 _readonly_pages;

  u32 i, j, k, m, index;
  u32 pml4_frame;
  u32 pdpt_frame;
  u32 pd_frame;
  u32 pt_frame;
  u32 kernel_phys_start, kernel_phys_end;
  u32 ro_phys_start, ro_phys_end;
  u32 sddata_phys_start, sddata_phys_end;
  u32 sdbss_phys_start, sdbss_phys_end;
  u64 *pml4, *pdpt, *pd, *pt;
  u8 memtype, def_memtype;
  u64 mtrr_cap = rdmsr (IA32_MTRRCAP);
  u64 mtrr_def_type = rdmsr (IA32_MTRR_DEF_TYPE);
  u8 num_var_reg = (u8) mtrr_cap;
  bool fix_supported = (mtrr_cap >> 8) & 0x01;
  u32 var_regs_frame;
  u64 *var_base, *var_mask;

  def_memtype = (u8) mtrr_def_type;

  vmwrite ((1 << 1)/* EPT */, VMXENC_PROCBASED_VM_EXEC_CTRLS2);

  pml4_frame = alloc_phys_frame_high ();
  pdpt_frame = alloc_phys_frame_high ();
  /* Allocate 4K page for variable range bases and masks */
  var_regs_frame = alloc_phys_frame ();

  if ((pml4_frame == -1) | (pdpt_frame == -1) | (var_regs_frame == -1)) {
    panic ("Out of Physical RAM for EPT Configuration.");
  }

  DLOG ("pml4_frame=0x%p pdpt_frame=0x%p", pml4_frame, pdpt_frame);

  var_base = map_virtual_page (var_regs_frame | 3);
  var_mask = &var_base[MAX_MTRR_VAR_REGS];

  pml4 = map_virtual_page (pml4_frame | 3);
  pdpt = map_virtual_page (pdpt_frame | 3);

  DLOG ("pml4=0x%p pdpt=0x%p", pml4, pdpt);

  memset (pml4, 0, 0x1000);
  memset (pdpt, 0, 0x1000);
  /* PDPT references a maximum of 512GB memory. Grant all access. */
  pml4[0] = pdpt_frame | 7;

#if 0
  /* Verify physical memory bitmap */
  logger_printf ("mm_table=%x, mm_begin=%d, mm_limit=%d\n",
                 (uint32) mm_table, mm_begin, mm_limit);
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

#ifdef MTRR_DEBUG
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

#ifdef MTRR_DEBUG
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

#ifdef MTRR_DEBUG
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

  kernel_phys_start = (uint32) &_physicalbootstrapstart +
                      SANDBOX_KERN_OFFSET * cpu;
  /* EPT data structure is not mapped for any sandbox kernel */
  kernel_phys_end = (uint32) &_physicalbootstrapstart +
                    SANDBOX_KERN_OFFSET * (cpu + 1) - EPT_DATA_SIZE;

  ro_phys_start = (uint32) &_physicalkernelstart +
                  SANDBOX_KERN_OFFSET * cpu;
  ro_phys_end = ro_phys_start + ((uint32) &_readonly_pages) * 0x1000;

  sddata_phys_start = (uint32) &_shared_driver_data_physical +
                      SANDBOX_KERN_OFFSET * cpu;
  sddata_phys_end = sddata_phys_start +
                    ((uint32) &_shared_driver_data_pages) * 0x1000;

  sdbss_phys_start = (uint32) &_shared_driver_bss_physical +
                     SANDBOX_KERN_OFFSET * cpu;
  sdbss_phys_end = sdbss_phys_start +
                   ((uint32) &_shared_driver_bss_pages) * 0x1000;

  /* Only 4 PDPTEs are needed for 4GB physical memory. */
  for (i = 0; i < 4; i++) {
    pd_frame = alloc_phys_frame_high ();
    pd = map_virtual_page (pd_frame | 3);

    if (pd_frame == -1) {
      panic ("Out of Physical RAM for EPT Configuration.");
    }
    DLOG ("pd_frame=0x%x, pd=0x%x", pd_frame, pd);

    memset (pd, 0, 0x1000);

    /* --!!-- NOTICE
     * The memory types are supposed to be set based on the
     * registers we read above. For now, let's just set them
     * manually.
     */
    for (j = 0; j < 512; j++) {
      if (i < 3) {
        memtype = 6;
      } else {
        if (j < 128)
          memtype = 6;
        else
          memtype = 0;
      }

      pt_frame = alloc_phys_frame_high ();
      pt = map_virtual_page (pt_frame | 3);

      if (pt_frame == -1) {
        panic ("Out of Physical RAM for EPT Configuration.");
      }

      memset (pt, 0, 0x1000);

      /* First 2MB should be treated specially */
      if (i == 0 && j == 0) {
        for (k = 0; k < 512; k++) {
          index = k * 0x1000;
          memtype = def_memtype;

          /*--!!-- NOTICE
           * OK, this chunck of code to set memory type according to MTRRs is a
           * real pain... It works on all the platforms we had in the lab. But
           * double check this if the boot process is extremely slow, the system
           * halts unpredictably or a ping round-trip time without kernel debug
           * output exceeds 3ms.
           */

          /* 
           * If fixed range MTRRs are supported. Memory type should be configured
           * according to those registers. Otherwise, set it to default.
           * Only the first MB is covered by fixed range MTRRs. 
           */
          if (fix_supported && (k < 256)) {
            int sub_range = 0;
            uint32 sub_index = 0;
            /* 512 KB IA32_MTRR_FIX64K */
            if (k < 128) {
              sub_index = k * 0x1000;
              sub_range = index >> 16;
              memtype = (mtrr_fix64k >> (sub_range * 8)) & 0xFF;
            } else if (k < 160) {
              sub_index = (k - 128) * 0x1000;
              sub_range = index >> 14;
              memtype = (mtrr_fix16k8 >> (sub_range * 8)) & 0xFF;
            } else if (k < 192) {
              sub_index = (k - 160) * 0x1000;
              sub_range = index >> 14;
              memtype = (mtrr_fix16kA >> (sub_range * 8)) & 0xFF;
            } else if (k < 200) {
              sub_index = (k - 192) * 0x1000;
              sub_range = index >> 12;
              memtype = (mtrr_fix4kC0 >> (sub_range * 8)) & 0xFF;
            } else if (k < 208) {
              sub_index = (k - 200) * 0x1000;
              sub_range = index >> 12;
              memtype = (mtrr_fix4kC8 >> (sub_range * 8)) & 0xFF;
            } else if (k < 216) {
              sub_index = (k - 208) * 0x1000;
              sub_range = index >> 12;
              memtype = (mtrr_fix4kD0 >> (sub_range * 8)) & 0xFF;
            } else if (k < 224) {
              sub_index = (k - 216) * 0x1000;
              sub_range = index >> 12;
              memtype = (mtrr_fix4kD8 >> (sub_range * 8)) & 0xFF;
            } else if (k < 232) {
              sub_index = (k - 224) * 0x1000;
              sub_range = index >> 12;
              memtype = (mtrr_fix4kE0 >> (sub_range * 8)) & 0xFF;
            } else if (k < 240) {
              sub_index = (k - 232) * 0x1000;
              sub_range = index >> 12;
              memtype = (mtrr_fix4kE8 >> (sub_range * 8)) & 0xFF;
            } else if (k < 248) {
              sub_index = (k - 240) * 0x1000;
              sub_range = index >> 12;
              memtype = (mtrr_fix4kF0 >> (sub_range * 8)) & 0xFF;
            } else {
              sub_index = (k - 248) * 0x1000;
              sub_range = index >> 12;
              memtype = (mtrr_fix4kF8 >> (sub_range * 8)) & 0xFF;
            }
          }

          /* 1 MB to 2 MB, we need to read variable range MTRRs */
          if (k >= 256) {
            /* Find the memory type */
            for (m=0; m < num_var_reg; m++) {
              /* Is the MTRR pair valid? */
              if (var_mask[m] & 0x800) {
                if ((index & (var_mask[m] & 0xFFFFF000)) ==
                    ((var_base[m] & 0xFFFFF000) & (var_mask[m] & 0xFFFFF000))) {
                  memtype = (var_base[m] & 0xFF);
                }
              }
            }
          }

          if (k < 256) {
            /* 1MB shared mapping for BIOS */
            pt[k] = index | (memtype << 3) | EPT_ALL_ACCESS;
          } else if ((index >= kernel_phys_start) && (index < kernel_phys_end)) {
            /* Second mega byte only mapped for Bootstrap Processor */
            if ((index >= ro_phys_start) && (index < ro_phys_end)) {
              /* Kernel read-only pages */
              pt[k] = index | (memtype << 3) | (EPT_READ_ACCESS + EPT_EXEC_ACCESS);
            } else if ((index >= sddata_phys_start) && (index < sddata_phys_end)) {
              /* Shared driver data mapping */
              pt[k] = ((uint32) &_shared_driver_data_physical +
                       (index - sddata_phys_start)) | (memtype << 3) | EPT_ALL_ACCESS;
            } else if ((index >= sdbss_phys_start) && (index < sdbss_phys_end)) {
              /* Shared driver bss mapping */
              pt[k] = ((uint32) &_shared_driver_bss_physical +
                       (index - sdbss_phys_start)) | (memtype << 3) | EPT_ALL_ACCESS;
            } else {
              /* All other pages grant all access */
              pt[k] = (k << 12) | (memtype << 3) | EPT_ALL_ACCESS;
            }
          }
        }

        pd[j] = pt_frame | (0 << 7) | EPT_ALL_ACCESS;
      } else if (i < 2) {
        /* --!!-- NOTICE
         * The assumption here is: all the kernel images in total will not
         * exceed 2GB boundary. If this is not true, we need to change the
         * code a little bit here.
         */
        for (k = 0; k < 512; k++) {
          index = (i << 30) + (j << 21) + (k << 12);

          memtype = def_memtype;
          /* Find the memory type */
          for (m=0; m < num_var_reg; m++) {
            /* Is the MTRR pair valid? */
            if (var_mask[m] & 0x800) {
              if ((index & (var_mask[m] & 0xFFFFF000)) ==
                  ((var_base[m] & 0xFFFFF000) & (var_mask[m] & 0xFFFFF000))) {
                memtype = (var_base[m] & 0xFF);
              }
            }
          }

          if ((index >= kernel_phys_start) && (index < kernel_phys_end)) {
            if ((index >= ro_phys_start) && (index < ro_phys_end)) {
              /* Kernel read-only pages */
              pt[k] = index | (memtype << 3) | (EPT_READ_ACCESS + EPT_EXEC_ACCESS);
            } else if ((index >= sddata_phys_start) && (index < sddata_phys_end)) {
              /* Shared driver data mapping */
              pt[k] = ((uint32) &_shared_driver_data_physical +
                       (index - sddata_phys_start)) | (memtype << 3) | EPT_ALL_ACCESS;
            } else if ((index >= sdbss_phys_start) && (index < sdbss_phys_end)) {
              /* Shared driver bss mapping */
              pt[k] = ((uint32) &_shared_driver_bss_physical +
                       (index - sdbss_phys_start)) | (memtype << 3) | EPT_ALL_ACCESS;
            } else {
              /* All other pages grant all access */
              pt[k] = index | (memtype << 3) | EPT_ALL_ACCESS;
            }
          }
        }

        pd[j] = pt_frame | (0 << 7) | EPT_ALL_ACCESS;
      } else {
        /*
         * Physical memory from 2G to 4G is shared for now.
         * We will work on this area for user space later.
         */
        for (k = 0; k < 512; k++) {
          index = (i << 30) + (j << 21) + (k << 12);

          memtype = def_memtype;
          /* Find the memory type */
          for (m=0; m < num_var_reg; m++) {
            /* Is the MTRR pair valid? */
            if (var_mask[m] & 0x800) {
              if ((index & (var_mask[m] & 0xFFFFF000)) ==
                  ((var_base[m] & 0xFFFFF000) & (var_mask[m] & 0xFFFFF000))) {
                memtype = (var_base[m] & 0xFF);
              }
            }
          }

          pt[k] = index | (memtype << 3) | EPT_ALL_ACCESS;
        }
        pd[j] = pt_frame | (0 << 7) | EPT_ALL_ACCESS;
        //pd[j] = ((i << 30) + (j << 21)) | (1 << 7) | (memtype << 3) | EPT_ALL_ACCESS;
      }

      unmap_virtual_page (pt);
    }
    DLOG ("pd[0]=0x%llX", pd[0]);
    unmap_virtual_page (pd);
    pdpt[i] = pd_frame | (0 << 7) | EPT_ALL_ACCESS;
    DLOG ("pdpt[%d]=0x%llX", i, pdpt[i]);
  }

  vmwrite (pml4_frame | (3 << 3) | 6, VMXENC_EPT_PTR);
  vmwrite (0, VMXENC_EPT_PTR_HI);
  DLOG ("VMXENC_EPT_PTR=0x%p pml4[0]=0x%llX pdpt[0]=0x%llX",
        vmread (VMXENC_EPT_PTR), pml4[0], pdpt[0]);

  unmap_virtual_page (var_base);
  unmap_virtual_page (pml4);
  unmap_virtual_page (pdpt);
  free_phys_frame (var_regs_frame);
}

/*
 * Helper function to verify the EPT mapping of a given guest physical address.
 * This works only for 32-bit address now.
 */
uint32
get_host_phys_addr (uint32 guest_phys_addr)
{
  uint32 pml4_frame = vmread (VMXENC_EPT_PTR);
  uint64 * pml4 = map_virtual_page ((pml4_frame & 0xFFFFF000) | 3);
  uint64 * pdpt = map_virtual_page ((pml4[0] & 0xFFFFF000) | 3);
  uint64 * pd = map_virtual_page ((pdpt[(guest_phys_addr >> 30) & 0x3] & 0xFFFFF000) | 3);
  uint64 * pt = map_virtual_page ((pd[(guest_phys_addr >> 21) & 0x1FF] & 0xFFFFF000) | 3);
  uint64 phys = (pt[(guest_phys_addr >> 12) & 0x1FF] & 0xFFFFF000) +
                (guest_phys_addr & 0x00000FFF);

  unmap_virtual_page (pt);
  unmap_virtual_page (pd);
  unmap_virtual_page (pdpt);
  unmap_virtual_page (pml4);

  return (uint32) phys;
}

/* vi: set et sw=2 sts=2: */
