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

#if DEBUG_SHDR > 0
#define DLOG(fmt,...) DLOG_PREFIX("shared driver",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

/*
 * Recover shared driver in current sandbox with the driver
 * in another sandbox specified by 'sbox'.
 */
bool
recover_driver (uint32 sbox)
{
  extern uint32 _shared_driver_data_physical, _shared_driver_data_pages;
  extern uint32 _shared_driver_bss_physical, _shared_driver_bss_pages;
  int i = 0, j = 0, k = 0;
  int start_i = 0, start_j = 0, start_k = 0;
  int end_i = 0, end_j = 0, end_k = 0;
  uint64 *pml4, *pdpt, *pd, *pt;
  uint32 pml4_frame = vmread (VMXENC_EPT_PTR) & 0xFFFFF000;
  uint32 index;
  uint32 memtype = 6;
  u32 sddata_phys_start, sddata_phys_end;
  u32 sdbss_phys_start, sdbss_phys_end;
  u32 cpu;

  cpu = get_pcpu_id ();
  pml4 = map_virtual_page (pml4_frame | 3);
  pdpt = map_virtual_page ((pml4[0] & 0xFFFFF000) | 3);
  unmap_virtual_page (pml4);

  sddata_phys_start = (uint32) &_shared_driver_data_physical +
                      SANDBOX_KERN_OFFSET * cpu;
  sddata_phys_end = sddata_phys_start +
                    ((uint32) &_shared_driver_data_pages) * 0x1000;

  sdbss_phys_start = (uint32) &_shared_driver_bss_physical +
                     SANDBOX_KERN_OFFSET * cpu;
  sdbss_phys_end = sdbss_phys_start +
                   ((uint32) &_shared_driver_bss_pages) * 0x1000;

  start_i = (sddata_phys_start / 0x40000000);
  end_i = (sddata_phys_end / 0x40000000);

  if (start_i == end_i) {
    start_j = (sddata_phys_start % 0x40000000) / 0x200000;
    end_j = (sddata_phys_end % 0x40000000) / 0x200000;
    if (start_j == end_j) {
      start_k = ((sddata_phys_start % 0x40000000) % 0x200000) / 0x1000;
      end_k = ((sddata_phys_end % 0x40000000) % 0x200000) / 0x1000;
    } else {
      start_k = 0;
      end_k = 512;
    }
  } else {
    start_j = 0;
    end_j = 512;
    start_k = 0;
    end_k = 512;
  }

  for (i = start_i; i < end_i; i++) {
    for (j = start_j; j < end_j; j++) {
      for (k = start_k; k < end_k; k++) {
        index = i * 0x40000000 + j * 0x200000 + k * 0x1000;
        pd = map_virtual_page ((pdpt[i] & 0xFFFFF000) | 3);
        pt = map_virtual_page ((pd[j] & 0xFFFFF000) | 3);

        if ((index >= sddata_phys_start) && (index < sddata_phys_end)) {
          /* Shared driver data mapping */
          pt[k] = ((uint32) &_shared_driver_data_physical + SANDBOX_KERN_OFFSET * sbox +
              (index - sddata_phys_start)) | (memtype << 3) | EPT_ALL_ACCESS;
        } else if ((index >= sdbss_phys_start) && (index < sdbss_phys_end)) {
          /* Shared driver bss mapping */
          pt[k] = ((uint32) &_shared_driver_bss_physical + SANDBOX_KERN_OFFSET * sbox +
              (index - sdbss_phys_start)) | (memtype << 3) | EPT_ALL_ACCESS;
        }

        unmap_virtual_page (pt);
        unmap_virtual_page (pd);
      }
    }
  }

  unmap_virtual_page (pdpt);

  return FALSE;
}

/* vi: set et sw=2 sts=2: */
