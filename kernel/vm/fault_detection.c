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

#include "types.h"
#include "vm/fault_detection.h"
#include "arch/i386-percpu.h"
#include "kernel.h"
#include "sched/sched.h"
#include "mem/mem.h"
#include "util/printf.h"

#define DEBUG_FAULT_DETECTION 
#ifdef DEBUG_FAULT_DETECTION
#define DLOG(fmt,...) DLOG_PREFIX("fault-detection:",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

/* Right now just an xor */
static void hash_page(fault_detection_hash_t* hash)
{
  int i;
  uint32* page = (uint32*)hash->virtual_page;
  //DLOG("Creating hash of data at 0x%X", hash->virtual_page);
  hash->hash[0] = 0;
  for(i = 0; i < 4096/sizeof(uint32); ++i) {
    hash->hash[0] ^= page[i];
  }
}

static bool populate_hash_dump(fault_detection_hash_dumps_t* hash_dumps,
                               bool new_pages_only)
{
  uint32* plPageDirectory = map_virtual_page ((uint32) get_pdbr () | 3);
  if((uint32)plPageDirectory == 0xFFFFFFFF) return FALSE;
  
  hash_dumps->num_hashes = 0;
  
  if(new_pages_only) {
    int i;
    for(i = 0; i < hash_dumps->num_hashes; ++i) {
      hash_page(&hash_dumps->hashes[i]);
    }
  }
  else {
    int i, j;
    for(i = 0; (i < 1024) && (hash_dumps->num_hashes < FAULT_DETECTION_HASH_COUNT_MAX); ++i) {
      if( (plPageDirectory[i] & 4) && ((plPageDirectory[i] & 0x08) == 0) ) {
        uint32* plPageTable = map_virtual_page((plPageDirectory[i] & 0xFFFFF000) | 3);
        if((uint32)plPageTable == 0xFFFFFFFF) {
          unmap_virtual_page(plPageDirectory);
          return FALSE;
        }
        for(j = 0; (j < 1024) && (hash_dumps->num_hashes < FAULT_DETECTION_HASH_COUNT_MAX); ++j) {
          if(plPageTable[j] & 4) {
            //DLOG("plPageDirectory[%d] = 0x%X, plPageTable[%d] = 0x%X",
            //     i, plPageDirectory[i], j, plPageTable[j]);
            hash_dumps->hashes[hash_dumps->num_hashes].virtual_page = i * 0x400000 + j * 0x1000;
            hash_page(&hash_dumps->hashes[hash_dumps->num_hashes++]);
          }
        }
        unmap_virtual_page(plPageTable);
      }
    }
  }
  hash_dumps->checkpoint_passed = FALSE;
  hash_dumps->count++;
  return TRUE;
}


static int _syscall_fault_detection(uint action, uint key, uint sink_sandbox)
{
#ifdef USE_VMX
  quest_tss * tss;
  task_id cur = percpu_read (current_task);

  if (!cur) return -1;
  tss = lookup_TSS(cur);
  if(!tss) return -1;
  if(sink_sandbox >= SHM_MAX_SANDBOX) return -1;
  
  switch((FAULT_DETECTION_ACTION)action) {
  case FDA_REGISTER_PROG:
    {
      int res;

      tss->fdi = kmalloc(sizeof(*tss->fdi));
      if(!tss->fdi) return -1;
      
      res = virtual_shared_mem_map(key, FAULT_DETECTION_POOL_SIZE, 1 << sink_sandbox,
                                   VSHM_ALL_ACCESS | VSHM_CREATE,
                                   (void**)&tss->fdi->hash_dumps, FALSE);
      if(res < 0) return -1;

      tss->fdi->hash_dumps->count = 0;

      populate_hash_dump(tss->fdi->hash_dumps, FALSE);
    }
    break;
    
  case FDA_SYNC:
    {
      populate_hash_dump(tss->fdi->hash_dumps, FALSE);
    }
    break;
    
  }
  
  /* -- EM -- Really ugly polling hack would be much better to use an
     IPI, don't want to create that interface yet since we most likely
     will be changing the monitor soon */
  while(!tss->fdi->hash_dumps->checkpoint_passed) {
    sched_usleep(10000);
  }
  
  return 0;
#else
  return -1;
#endif
}

int syscall_fault_detection(uint action, uint key, uint sink_sandbox)
{
  int res;
  lock_kernel();
  res = _syscall_fault_detection(action, key, sink_sandbox);
  unlock_kernel();
  return res;
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
