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


#include <drivers/usb/usb.h>
#include <sched/vcpu.h>
#include <drivers/usb/usb-migration.h>
#include <util/printf.h>
#include <sched/vcpu.h>
#include <sched/sched.h>
#include <kernel.h>
#include <mem/virtual.h>

//#define DEBUG_USB_MIGRATION
//#define DEBUG_USB_MIGRATION_VERBOSE

#ifdef DEBUG_USB_MIGRATION
#define DLOG(fmt,...) DLOG_PREFIX("usb-migration",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef DEBUG_USB_MIGRATION_VERBOSE
#define DLOGV(fmt,...) DLOG_PREFIX("usb-migration",fmt,##__VA_ARGS__)
#else
#define DLOGV(fmt, ...) ;
#endif


/* This sleep queue is used only by the migration thread.  Used when
   the migration thread has nothing to do or when it is in the middle
   of transmitting data and needs to wait */
static quest_tss *migration_thread_sleep_queue = NULL;

typedef enum {NO_TASKS, BITMAP, PAGE_LIMIT, TSS_IN_FLIGHT} sleep_reason_t;

static sleep_reason_t migration_thread_sleep_reason;

static bool bitmaps_in_flight = FALSE; 

#define MAX_PAGES_IN_FLIGHT 5

#define MIGRATION_THREAD_STACK_SIZE 1024
uint32 migration_thread_stack[MIGRATION_THREAD_STACK_SIZE];

pgdir_entry_t* task_pd_table = NULL;

pgtbl_entry_t* page_table_currently_migrated = NULL;

uint32* bitmap_currently_migrated_page_table = NULL;

#define TRANSMISSION_SIZE (512/4)   /* Specified in 4-bytes */

#define TRANSMISSION_BUFFER_SIZE (TRANSMISSION_SIZE)
static uint32 transmission_buffer[TRANSMISSION_BUFFER_SIZE];

#define MIGRATION_QUEUE_SIZE 1
quest_tss* migration_queue[MIGRATION_QUEUE_SIZE];
uint migration_queue_head = 0;
uint migration_queue_tail = 0;


#define TABLE_BITMAP_SIZE 32    /* Specified in 4-bytes */

CASSERT(TRANSMISSION_SIZE >= TABLE_BITMAP_SIZE, usb_migration_minimum_packet_size)

#define BITMAPS_PER_TRANSMISSION (TRANSMISSION_BUFFER_SIZE / TABLE_BITMAP_SIZE)

#define MIGRATION_PIPE_TYPE PIPE_INTERRUPT

//static phys_addr_t pages_to_migrate[BITMAPS_PER_TRANSMISSION * 1024];
static uint last_page_migrated = 0;

/* -- EM -- If the usb host controller calls the callback
   function in the context of an interrupt we need to protect
   pages_in_flight with a lock */
static uint pages_in_flight = 0;

static struct urb* migration_urb = NULL;
static USB_DEVICE_INFO* migration_dev;
bool device_initialized = FALSE;

void usb_migrate_task(quest_tss* task);

static void usb_migration_thread()
{
  int res;
 resubmit:
  res = usb_submit_urb(migration_urb, 0);
  if(res == -2) {
    sched_usleep(500 * 1000);
    goto resubmit;
  }
  else if(res == -1) {
    DLOG("Failed to submit migration urb");
    panic("Failed to submit migration urb");
  }

  while(TRUE) {
    while(migration_queue[migration_queue_head]) {

#ifdef DEBUG_USB_MIGRATION
      u64 tsc;
      RDTSC(tsc);
      DLOG("Migration start time = 0x%016llX", tsc);
#endif
      usb_migrate_task(migration_queue[migration_queue_head]);
      DLOG("Done migrating task!!");

      /* -- EM -- Need to free pages and other resources associated
         with process, basically have the process exit */
      //free_quest_tss(migration_queue[migration_queue_head]);
      migration_queue[migration_queue_head] = NULL;
      
      if(++migration_queue_head == MIGRATION_QUEUE_SIZE) {
        migration_queue_head = 0;
      }
    }
    
    DLOG("About to sleep: line %d", __LINE__);
    migration_thread_sleep_reason = NO_TASKS;
    queue_append(&migration_thread_sleep_queue, str());
    schedule();
  }

}

static void print_table_bitmap(uint32* bitmap)
{
#ifdef DEBUG_USB_MIGRATION_VERBOSE
  int i;
  DLOGV("\n\n\n");
  for(i = 0; i < 32; i +=4) {
    DLOGV("0x%08X 0x%08X 0x%08X 0x%08X", bitmap[i+3], bitmap[i+2], bitmap[i+1], bitmap[i]);
  }
  DLOGV("\n\n\n");
#endif //DEBUG_USB_MIGRATION_VERBOSE
}

static inline void wait_for_bitmaps()
{
  if(bitmaps_in_flight) {
    DLOG("About to sleep: line %d", __LINE__);
    migration_thread_sleep_reason = BITMAP;
    queue_append(&migration_thread_sleep_queue, str());
    schedule();
  }
}

void create_pdt_bitmap(quest_tss* task, uint32* bitmap)
{
  int i;
  pgdir_entry_t* page = task_pd_table;
  wait_for_bitmaps();
  memset(bitmap, 0, TABLE_BITMAP_SIZE*4);
    
  for(i = 0; i < 1024; ++i, ++page) {
    if(page->flags.present && page->flags.supervisor) {
      BITMAP_SET(bitmap, i);
    }
  }
  BITMAP_SET(bitmap, PGDIR_KERNEL_STACK);
}

void create_pt_bitmap(quest_tss* task, uint pt_index, uint32* bitmap)
{
  int i;
  phys_addr_t pt_phys;
  pgtbl_entry_t* page_table;
  pgtbl_entry_t* page_table_backup;
  wait_for_bitmaps();
  if(!task_pd_table[pt_index].flags.present) {
    
    DLOG("Page table is not present, pt_index = %u", pt_index);
    panic("Page table is not present");
  }

  
  memset(bitmap, 0, TABLE_BITMAP_SIZE*4);

  pt_phys = task_pd_table[pt_index].raw & 0xFFFFF000;
  page_table_backup = page_table = map_virtual_page(pt_phys | 3);

  if(!page_table) {
    DLOG("Failed to page table to a virtual page in %s", __FUNCTION__);
    panic("Failed to page table to a virtual page");
  }

  for(i = 0; i < 1024; ++i, ++page_table) {
    if(page_table->flags.present) {
      BITMAP_SET(bitmap, i);
    }
  }
  unmap_virtual_page(page_table_backup);
  print_table_bitmap(bitmap);
}

static inline void send_page(int page_index)
{
  phys_addr_t frame_addr = page_table_currently_migrated[page_index].raw & 0xFFFFF000;


  DLOGV("Sending page at page index %d", page_index);

  if(usb_rt_push_data(migration_urb, (char*)frame_addr,
                      0x1000, 0x1000, URB_RT_PHYS_ADDR, NULL) != 0x1000) {
    DLOG("Failed to send entire page table");
    panic("Failed to send entire page table");
  }
  pages_in_flight++;
  DLOGV("pages_in_flight = %u", pages_in_flight);
}

void migrate_page_table(quest_tss* task, uint pt_index, uint32* bitmap)
{
  phys_addr_t pt_phys;
  
  if(!task_pd_table[pt_index].flags.present) {
    DLOG("Page table is not present, pt_index = %u", pt_index);
    panic("Page table is not present");
  }
  
  pt_phys = task_pd_table[pt_index].raw & 0xFFFFF000;
  page_table_currently_migrated = map_virtual_page(pt_phys | 3);

  if(page_table_currently_migrated == NULL) {
    DLOG("Failed to map page in %s", __FUNCTION__);
    panic("Failed to map page");
  }
  
  bitmap_currently_migrated_page_table = bitmap;
  for(last_page_migrated = 0; last_page_migrated < 1024; ++last_page_migrated) {
    if(BITMAP_TST(bitmap, last_page_migrated)) {
      send_page(last_page_migrated);
      if(pages_in_flight >= MAX_PAGES_IN_FLIGHT) {
        migration_thread_sleep_reason = PAGE_LIMIT;
        DLOG("About to sleep: line %d", __LINE__);
        queue_append(&migration_thread_sleep_queue, str());
        schedule();
        /* The callback handler will have migrated everything else for
           this page table at this point so we are done with this
           page*/
        break;
      }
    }
  }
  
  unmap_virtual_page(page_table_currently_migrated);
  page_table_currently_migrated = NULL;
  bitmap_currently_migrated_page_table = NULL;
}

static void send_bitmaps(uint bitmaps_count)
{
  int res;
  DLOG("Sending bitmaps: bitmaps_count = %d", bitmaps_count);
  if((res = usb_rt_push_data(migration_urb, (char*)transmission_buffer,
                             TRANSMISSION_BUFFER_SIZE*4, TRANSMISSION_BUFFER_SIZE*4,
                             0, (void*)1)) != TRANSMISSION_BUFFER_SIZE*4 ) {
    DLOG("Failed to send bitmaps, res = %d", res);
    panic("Failed to send bitmaps");
  }
  bitmaps_in_flight = TRUE;
}

void usb_migrate_task(quest_tss* task)
{
  uint i;

  /* Set to 1 since pdt bitmap is filled first */
  uint next_bitmap_to_fill = 1;
  
  int page_tables_to_send[BITMAPS_PER_TRANSMISSION];
  /* Need an extra copy of the pdt_bitmap because it is needed
     throughout the entire migration */
  /* -- EM -- making it static due to stack space limitations */
  static uint32 pdt_bitmap[TABLE_BITMAP_SIZE];
  
  phys_addr_t pdt_phys = task->CR3 & ~0x1F;
  task_pd_table = map_virtual_page(pdt_phys | 3);

  if(task_pd_table == NULL) {
    DLOG("Failed to map pd table to a virtual page in %s", __FUNCTION__);
    panic("Failed to map pd table to a virtual page");
  }
  
  /* First bitmap is not a page table */
  page_tables_to_send[0] = -1;
  create_pdt_bitmap(task, transmission_buffer);
  memcpy(pdt_bitmap, transmission_buffer, sizeof(pdt_bitmap));
  print_table_bitmap(pdt_bitmap);
  if(next_bitmap_to_fill == BITMAPS_PER_TRANSMISSION) {
    int j;
    next_bitmap_to_fill = 0;
    
    DLOG("About to send %d bitmap(s) at %d", BITMAPS_PER_TRANSMISSION, __LINE__);
    send_bitmaps(BITMAPS_PER_TRANSMISSION);
    
    for(j = 0; j < BITMAPS_PER_TRANSMISSION; ++j) {
      if(page_tables_to_send[j] >= 0) {
        migrate_page_table(task, page_tables_to_send[j],
                           &transmission_buffer[TABLE_BITMAP_SIZE * j]);
        //usb_rt_free_write_resources(migration_urb);
      }
    }
  }
  for(i = 0; i < 1024; ++i) {
    if(BITMAP_TST(pdt_bitmap, i)) {
      create_pt_bitmap(task, i, &transmission_buffer[TABLE_BITMAP_SIZE * next_bitmap_to_fill]);
      page_tables_to_send[next_bitmap_to_fill++] = i;
      DLOG("Adding page table %d to bitmap list", i);
      
      if(next_bitmap_to_fill == BITMAPS_PER_TRANSMISSION) {
        int j;
        next_bitmap_to_fill = 0;
        
        DLOG("About to send %d bitmap(s) at %d", BITMAPS_PER_TRANSMISSION, __LINE__);
        send_bitmaps(BITMAPS_PER_TRANSMISSION);
                
        for(j = 0; j < BITMAPS_PER_TRANSMISSION; ++j) {
          if(page_tables_to_send[j] >= 0) {
            migrate_page_table(task, page_tables_to_send[j],
                               &transmission_buffer[TABLE_BITMAP_SIZE * j]);
            //usb_rt_free_write_resources(migration_urb);
          }
        }
      }
    }
  }

  /* At this point need to send remaining bitmaps and then the page
     tables associated with those bitmaps */
  
  if(next_bitmap_to_fill) {
    DLOG("About to send %d bitmap(s) at %d", next_bitmap_to_fill, __LINE__); 
    send_bitmaps(next_bitmap_to_fill);
    
    for(i = 0; i < next_bitmap_to_fill; ++i) {
      if(page_tables_to_send[i] >= 0) {
        migrate_page_table(task, page_tables_to_send[i],
                           &transmission_buffer[TABLE_BITMAP_SIZE * i]);
        //usb_rt_free_write_resources(migration_urb);
      }
    }
  }

  usb_rt_push_data(migration_urb, task, sizeof(*task), sizeof(*task), 0, (void*)2);

  
  DLOG("About to sleep: line %d", __LINE__);
  migration_thread_sleep_reason = TSS_IN_FLIGHT;
  queue_append(&migration_thread_sleep_queue, str());
  schedule();

  unmap_virtual_page(task_pd_table);
  task_pd_table = NULL;
}

static void migration_complete_callback(struct urb* urb)
{
  switch((uint)urb->context) {
  case 1:
    bitmaps_in_flight = FALSE;
    if(migration_thread_sleep_queue && migration_thread_sleep_reason == BITMAP) {
      wakeup_queue(&migration_thread_sleep_queue);
      DLOG("woken up from bitmap sleeping");
    }
    DLOGV("bitmap completely sent");
    break;

  case 0:
    DLOGV("page completely sent");
    
    pages_in_flight--;
    if(page_table_currently_migrated && last_page_migrated < 1024) {
      while(++last_page_migrated < 1024) {
        if(BITMAP_TST(bitmap_currently_migrated_page_table, last_page_migrated)) {
          send_page(last_page_migrated);
          return;
        }
      }
    }
    /* At this point if the migration thread is waiting on the queue we
       can wake it up and let it move forward */
    if(migration_thread_sleep_queue && migration_thread_sleep_reason == PAGE_LIMIT) {
      DLOG("Waking up thread do to page limit");
      wakeup_queue(&migration_thread_sleep_queue);
    }
    break;

  case 2:
    if(!migration_thread_sleep_queue || migration_thread_sleep_reason != TSS_IN_FLIGHT) {
      DLOG("should be sleeping for tss to be sent");
      panic("should be sleeping for tss to be sent");
    }
    DLOG("Waking up thread do to tss having been sent");
    wakeup_queue(&migration_thread_sleep_queue);
    break;
  default:
    DLOG("Unknown context passed to usb migration callback function");
    panic("Unknown context passed to usb migration callback function");
  }
}

static bool migration_probe (USB_DEVICE_INFO *dev, USB_CFG_DESC *cfg,
                             USB_IF_DESC *desc)
{
  USB_EPT_DESC* migration_ep;
  char* temp;
  uint pipe;
  if(dev->devd.idVendor != 0xabc4 ||
     dev->devd.idProduct != 0xabca) {
    return FALSE;
  }

  if(device_initialized) return TRUE;
  
  DLOG("Find migration device");
  usb_set_configuration(dev, cfg->bConfigurationValue);

  memset(migration_queue, 0, sizeof(migration_queue));
  memset(transmission_buffer, 0, sizeof(transmission_buffer));

  temp = kmalloc(cfg->wTotalLength);

  if(temp == NULL) {
    DLOG("Failed to allocate temporary buffer");
    return FALSE;
  }

  usb_get_descriptor(dev, USB_TYPE_CFG_DESC, 0, 0, cfg->wTotalLength, (addr_t)temp);

  
  migration_ep = get_next_desc(USB_TYPE_EPT_DESC, temp, cfg->wTotalLength);
  
  if(migration_ep == NULL ||
     (usb_get_endpoint_transfer_type(migration_ep) != MIGRATION_PIPE_TYPE)) {
    DLOG("Failed to find migration endpoint");
    kfree(temp);
    return FALSE;
  }

  pipe = usb_create_pipe(dev, migration_ep);
  if(usb_pipein(pipe)) {
    DLOG("Migration endpoint should be an out endpoint");
    panic("Migration endpoint should be an out endpoint");
  }
  if(migration_ep->wMaxPacketSize != (TRANSMISSION_SIZE * 4)) {
    DLOG("Transmission size must equal packet size / 4");
    panic("Transmission size must equal packet size / 4");
  }
  migration_dev = dev;
  migration_dev->ep_out[migration_ep->bEndpointAddress & 0xF].desc = *migration_ep;
  migration_urb = usb_alloc_urb(0, 0);
  if(migration_urb == NULL) {
    DLOG("Failed to allocate urb for migration");
    panic("Failed to allocate urb for migration");
  }

  usb_fill_rt_int_urb(migration_urb, dev, pipe, NULL, 0,
                      migration_complete_callback, NULL, migration_ep->bInterval, 0);
  
  migration_urb->transfer_flags |= URB_RT_ZERO_COPY_WRITE;

  DLOG("Creating kernel thread for usb migration");
  create_kernel_thread_args((u32)usb_migration_thread,
                            (u32) &migration_thread_stack[MIGRATION_THREAD_STACK_SIZE-1],
                            "USB Migration Thread",
                            TRUE, 0);
  
  device_initialized = TRUE;
  return TRUE;
}


bool usb_migration_queue_full(void)
{
  return ((migration_queue_head == migration_queue_tail) &&
          (migration_queue[migration_queue_head] != NULL)) || (!device_initialized);
}

bool usb_add_task_to_migration_queue(quest_tss * tss)
{
  if(usb_migration_queue_full()) return FALSE;

  DLOG("Adding task to migration queue");

  migration_queue[migration_queue_head++] = tss;
  
  if(migration_queue_head == MIGRATION_QUEUE_SIZE) {
    migration_queue_head = 0;
  }

  /* migration thread is on the queue */
  if(migration_thread_sleep_queue) {
    wakeup_queue(&migration_thread_sleep_queue);
  }
  
  return TRUE;
}


#include "module/header.h"

USB_DRIVER_INIT(migration_dev, "USB Migration Device", migration_probe, NULL, NULL, NULL, NULL);



/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
