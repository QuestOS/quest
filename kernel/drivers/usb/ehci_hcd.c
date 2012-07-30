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

#include <smp/apic.h>
#include <drivers/usb/usb.h>
#include <drivers/usb/ehci.h>
#include <drivers/usb/ehci_mem.h>
#include <drivers/usb/ehci_debug.h>
#include <mem/virtual.h>
#include <mem/pow2.h>
#include <arch/i386-div64.h>
#include <kernel.h>
#include <sched/sched.h>


/*
 * Following two macros were taken from Linux
 */
// high bandwidth multiplier, as encoded in highspeed endpoint descriptors
#define hb_mult(wMaxPacketSize) (1 + (((wMaxPacketSize) >> 11) & 0x03))

// ... and packet size, for any kind of endpoint descriptor
#define max_packet(wMaxPacketSize) ((wMaxPacketSize) & 0x07ff)

static int ehci_async_transfer(ehci_hcd_t* ehci_hcd, struct urb* urb);

static int ehci_isochronous_transfer(ehci_hcd_t* ehci_hcd, struct urb* urb);

static inline void itd_remove_from_periodic(itd_t* itd);

static int ehci_is_rt_schedulable(ehci_hcd_t* ehci_hcd, struct urb* urb);

/*
 * Default frame list size specified by EHCI Specs
 */
#define DEFAULT_FRM_LST_SIZE 1024 

#define FRM_LIST_SIZE DEFAULT_FRM_LST_SIZE  /* Frame list size used in Quest */
#define DEFAULT_INT_THRESHOLD 8
#define QH_POOL_SIZE  256
#define QTD_POOL_SIZE 256
#define ITD_POOL_SIZE 1024
#define EHCI_COMPLETION_ELEMENT_POOL_SIZE 128

CASSERT((QH_POOL_SIZE  % 32) == 0, ehci_qh_pool_size )
CASSERT((QTD_POOL_SIZE % 32) == 0, ehci_qtd_pool_size)
CASSERT((ITD_POOL_SIZE % 32) == 0, ehci_itd_pool_size)
CASSERT((EHCI_COMPLETION_ELEMENT_POOL_SIZE % 32) == 0, ehci_completion_element_pool_size)

static ehci_hcd_t ehci_hcd;
static frm_lst_lnk_ptr_t frame_list[FRM_LIST_SIZE] ALIGNED(0x1000);
static frm_lst_lnk_ptr_t last_frame_list_entries[FRM_LIST_SIZE] ALIGNED(0x1000);

static uint32_t micro_frame_remaining_bytes[FRM_LIST_SIZE * 8];

static qh_t qh_pool[QH_POOL_SIZE] ALIGNED(0x1000);
static uint32_t used_qh_bitmap [(QH_POOL_SIZE  + EHCI_ELEMENTS_PER_BITMAP_ENTRY - 1)
                                / EHCI_ELEMENTS_PER_BITMAP_ENTRY];

static qtd_t qtd_pool[QTD_POOL_SIZE] ALIGNED(0x1000);
static uint32_t used_qtd_bitmap[(QTD_POOL_SIZE + EHCI_ELEMENTS_PER_BITMAP_ENTRY - 1)
                                / EHCI_ELEMENTS_PER_BITMAP_ENTRY];

static itd_t itd_pool[ITD_POOL_SIZE] ALIGNED(0x1000);
static uint32_t used_itd_bitmap[(ITD_POOL_SIZE + EHCI_ELEMENTS_PER_BITMAP_ENTRY - 1)
                                / EHCI_ELEMENTS_PER_BITMAP_ENTRY];

static ehci_completion_element_t
ehci_completion_element_pool[EHCI_COMPLETION_ELEMENT_POOL_SIZE] ALIGNED(0x1000);

static uint32_t
used_ehci_completion_element_bitmap[(EHCI_COMPLETION_ELEMENT_POOL_SIZE +
                                     EHCI_ELEMENTS_PER_BITMAP_ENTRY - 1)
                                    / EHCI_ELEMENTS_PER_BITMAP_ENTRY];


static void ehci_check_for_urb_completions(ehci_hcd_t* ehci_hcd, bool in_irq_handler)
{
  ehci_completion_element_t* comp_element;
  ehci_completion_element_t* temp_comp_element;
  int i;
  bool done;
  itd_t* itd;
  itd_t* temp_itd;
  qtd_t* qtd;
  qtd_t* temp_qtd;
  int packets_traversed;
  struct urb* urb;

  /*
   * Since this function is called both in a thread that runs with
   * interrupts on and in an interrupt handler if we cannot get the
   * lock when in the interrupt handler then that means something else
   * is handling completions and we should just back off.  This is
   * extremely important for the interrupt handler because if it
   * preempted the backup thread when the backup thread had the lock
   * the interrupt handler would wait indefinitely unless the backup
   * thread was scheduled on another core
   */

  if(in_irq_handler) {
    if(!spinlock_attempt_lock(&ehci_hcd->completion_lock)) {
      return;
    }
  }
  else {
    spinlock_lock(&ehci_hcd->completion_lock);
  }
  
  list_for_each_entry_safe(comp_element, temp_comp_element,
                           &ehci_hcd->completion_list, chain_list) {
    urb = comp_element->urb;
    if(urb->active) {
      
      done = FALSE;
      
      switch(comp_element->pipe_type) {
      case PIPE_ISOCHRONOUS:
        itd = list_entry(comp_element->tds.prev, itd_t, chain_list);
        for(i = 0; i < 8; ++i) {
          if(itd->transaction[i].raw & ITD_ACTIVE) {
            break;
          }
        }
        done = i == 8;
        
        break;
        
      case PIPE_CONTROL:
      case PIPE_BULK:
        panic("async not implemented for ehci_check_for_urb_completions");
        DLOG("async not implemented for ehci_check_for_urb_completions");
        break;
        
      case PIPE_INTERRUPT:
        DLOG("Interrupt case in ehci_check_for_urb_completions not implemented");
        panic("Interrupt case in ehci_check_for_urb_completions not implemented");
        break;
        
      default:
        DLOG("Default case reached in %s", __FUNCTION__);
        panic("Default case reached in ehci_check_for_urb_completions");
      }
      
      if(done) {
        urb->active = FALSE;
        list_del(&comp_element->chain_list);
        
        /*
         * -- EM -- Since this function is called in the context of
         * the interrupt handler (and in the hardware bug backup
         * thread) the better thing to do would be wake up a
         * thread/IOVCPU to handle the URB completion function (maybe
         * might not be worth it since completion callbacks are
         * usually very small, this is something we can compare for a
         * performance/predictability trade-off) but for right now just
         * call it here
         */
        if(urb->complete != NULL) {
          urb->complete(comp_element->urb);
        }
        
        
        /*
         * Do URB bookkeeping and cleanup EHCI resources
         */
        
        switch(comp_element->pipe_type) {
          
        case PIPE_ISOCHRONOUS:
          
          packets_traversed = 0;
          list_for_each_entry(itd, &comp_element->tds, chain_list) {
            for(i = 0; (i < 8) && (packets_traversed < urb->number_of_packets);
                ++i, ++packets_traversed) {
              urb->iso_frame_desc[packets_traversed].actual_length
                = (itd->transaction[i].raw >> 16) & 0xFFF;
              
              urb->iso_frame_desc[packets_traversed].status
                = itd->transaction[i].status;
              
              if(itd->transaction[i].raw & ITD_ACTIVE) {
                print_itd_info(NULL, itd, "");
                
                DLOG("itd should be done but is not");
                panic("itd should be done but is not");
              }
            }
          }
          
          if(urb->real_time) {
            list_for_each_entry(itd, &comp_element->tds, chain_list) {
              /* -- EM -- commenting it out to get it to compile */
              //itd_restore(itd);
            }
          }
          else {
            
            list_for_each_entry(itd, &comp_element->tds, chain_list) {
              itd_remove_from_periodic(itd);
            }
            
            list_for_each_entry_safe(itd, temp_itd, &comp_element->tds,
                                     chain_list) {
              free_itd(ehci_hcd, itd);
            }
          }
          break;
          
        case PIPE_CONTROL:
        case PIPE_BULK:
          panic("async not implemented for ehci_check_for_urb_completions");
          DLOG("async not implemented for ehci_check_for_urb_completions");
          break;
          
        case PIPE_INTERRUPT:
          DLOG("Interrupt case in ehci_check_for_urb_completions not "
               "implemented");
          panic("Interrupt case in ehci_check_for_urb_completions "
                "not implemented");
          break;
          
        default:
          DLOG("Default case reached in %s", __FUNCTION__);
          panic("Default case reached in ehci_check_for_urb_completions");
        }
        
        
      }
    }
    else {
      list_del(&comp_element->chain_list);
    }
    
  }
  
  
  spinlock_unlock(&ehci_hcd->completion_lock);
}

void insert_remaining_realtime_itds(ehci_hcd_t* ehci_hcd, bool in_irq_handler)
{
  ehci_iso_urb_priv_t* iso_urb_priv;
  ehci_iso_urb_priv_t* temp_iso_urb_priv;
  frm_lst_lnk_ptr_t* frame_list;
  uint32_t frame_list_size;
  uint32_t frame_list_mask;

  /*
   * Since this function is called both in a thread that runs with
   * interrupts on and in an interrupt handler if we cannot get the
   * lock when in the interrupt handler then that means something else
   * is handling completions and we should just back off.  This is
   * extremely important for the interrupt handler because if it
   * preempted the backup thread when the backup thread had the lock
   * the interrupt handler would wait indefinitely unless the backup
   * thread was scheduled on another core
   */

  if(in_irq_handler) {
    if(!spinlock_attempt_lock(&ehci_hcd->uninserted_itd_lock)) {
      return;
    }
  }
  else {
    spinlock_lock(&ehci_hcd->uninserted_itd_lock);
  }

  frame_list      = ehci_hcd->frame_list;
  frame_list_size = ehci_hcd->frame_list_size;
  frame_list_mask = frame_list_size - 1;
  
  list_for_each_entry_safe(iso_urb_priv, temp_iso_urb_priv,
                           &ehci_hcd->uninserted_itd_urb_list,
                           uninserted_itd_urb_list) {
    bool all_itds_added = TRUE;
    itd_t* itd;
    itd_t* itd_temp;
    bool looped_around;
    struct urb* urb = iso_urb_priv_to_urb(iso_urb_priv);
    uint32_t current_frame_index, start_frame_index;
    uint32_t frame_interval_offset = iso_urb_priv->interval_offset / 8;
    uint32_t frame_interval        = urb->interval / 8;
    
    if(frame_interval <= 1) {
      frame_interval = 1;
      current_frame_index = EHCI_CURRENT_FRAME_INDEX(ehci_hcd);
      start_frame_index = current_frame_index + EHCI_SAFE_FRAME_OFFSET(ehci_hcd);
    }
    else {
      current_frame_index = EHCI_CURRENT_FRAME_INDEX(ehci_hcd);
      start_frame_index = ((current_frame_index + EHCI_SAFE_FRAME_OFFSET(ehci_hcd))
                           | (frame_interval-1)) + 1 + frame_interval_offset;
    }
    
    start_frame_index &=  frame_list_mask;
    
    looped_around = start_frame_index < current_frame_index;
    /*
     * If looped_around is TRUE we looped around by adding the safe
     * offset and interval offset
     */
    list_for_each_entry_safe(itd, itd_temp, &iso_urb_priv->uninserted_itds_list, uninserted_list) {
      uint32_t frame_index = itd->frame_index;
      if(looped_around) {
	if( (frame_index >= start_frame_index) && (frame_index < current_frame_index) ) {
	  insert_itd_into_frame_list(frame_list, frame_index, itd);
          list_del(&itd->uninserted_list);
	}
	else {
          all_itds_added = FALSE;
        }
      }
      else {
	if( (frame_index >= start_frame_index || frame_index < current_frame_index) ) {
	  insert_itd_into_frame_list(frame_list, frame_index, itd);
          list_del(&itd->uninserted_list);
        }
	else {
          all_itds_added = FALSE;
	}
      }
    }

    if(all_itds_added) {
      list_del(&iso_urb_priv->uninserted_itd_urb_list);
    }
  }  
  spinlock_unlock(&ehci_hcd->uninserted_itd_lock);
}


#ifdef EHCI_IOC_BUG


static void ioc_backup_thread(ehci_hcd_t* ehci_hcd)
{
  unlock_kernel();
  sti();
  
  while(1) {
    
    /*
     * Need to get the kernel lock since sched_usleep will call
     * schedule
     */
    cli();
    lock_kernel();
    // 18750 = 150 * 125
    sched_usleep(18750 * DEFAULT_INT_THRESHOLD );
    unlock_kernel();
    sti();
    
    insert_remaining_realtime_itds(ehci_hcd, FALSE);
    ehci_check_for_urb_completions(ehci_hcd, FALSE);   
  }
}

#endif // EHCI_IOC_BUG



static bool
handshake(uint32_t *ptr, uint32_t mask, uint32_t done, uint32_t usec)
{
  uint32_t result;
  
  do {
    result = *ptr;
    if (result == ~(uint32_t)0) return FALSE; /* card removed */
    result &= mask;
    if (result == done) return TRUE;
    tsc_delay_usec(1);
    usec--;
  } while (usec);
  
  return FALSE;
}

static uint32_t
ehci_irq_handler(uint8 vec)
{
  
  uint32_t status;
  lock_kernel();

  if(vec != ehci_hcd.handler_vector) {
    DLOG("%s called but not for the EHCI chip", __FUNCTION__);
    panic("ehci_irq_handler called but not for the EHCI chip");
  }
  
  status = ehci_hcd.regs->status;

  EHCI_ACK_INTERRUPTS(&ehci_hcd, status & USBINTR_ALL);

  insert_remaining_realtime_itds(&ehci_hcd, TRUE);
  
  if(status & USBINTR_INT) { /* "normal" completion (short, ...) */
    ehci_check_for_urb_completions(&ehci_hcd, TRUE);
  }
  
  if(status & USBINTR_IAA) { /* Interrupted on async advance */
    DLOG("USBINTR_IAA case in %s unhandled status = 0x%X", __FUNCTION__, status);
    panic("USBINTR_IAA case unhandled");
  }

  if(status & USBINTR_HSE) { /* such as some PCI access errors */
    DLOG("USBINTR_HSE case in %s unhandled status = 0x%X", __FUNCTION__, status);
    panic("USBINTR_HSE case unhandled");
  }
  
  if(status & USBINTR_FLR) { /* frame list rolled over */
    DLOG("USBINTR_FLR case in %s unhandled status = 0x%X", __FUNCTION__, status);
    //panic("USBINTR_FLR case unhandled");
  }

  if(status & USBINTR_PCD) { /* port change detect */
    DLOG("USBINTR_PCD case in %s unhandled status = 0x%X", __FUNCTION__, status);
    panic("USBINTR_PCD case unhandled");
  }

  if(status & USBINTR_ERR) { /* "error" completion (overflow, ...) */
    DLOG("USBINTR_ERR case in %s unhandled status = 0x%X", __FUNCTION__, status);
    panic("USBINTR_ERR case unhandled");
  }
  
  unlock_kernel();
  
  return 0;
}

// Restart the EHCI 
static inline bool
restart_ehci_hcd(ehci_hcd_t* ehci_hcd)
{
  if(EHCI_IS_RUNNING(ehci_hcd)) {
    EHCI_HALT(ehci_hcd);
    
    if(!handshake(&(ehci_hcd->regs->status),
                  HALTED, HALTED, 125 * 16)) return FALSE;
  }

  EHCI_RESET(ehci_hcd);

  if(!handshake(&(ehci_hcd->regs->command),
                RESET, 0, 250 * 1000)) return FALSE;
  /* Got the constant 250 * 1000 from ehci-hcd.c of linux */

  return TRUE;
}

static inline bool
initialise_frame_list(ehci_hcd_t* ehci_hcd)
{
  uint32_t           frm_lst_size            = ehci_hcd->frame_list_size;
  frm_lst_lnk_ptr_t* frame_list              = ehci_hcd->frame_list;
  frm_lst_lnk_ptr_t* last_frame_list_entries = ehci_hcd->last_frame_list_entries;
  
  if(EHCI_IS_FRM_LST_LEN_PROG(ehci_hcd)) {
    switch(frm_lst_size) {
    case 256:
      EHCI_SET_FRAME_LIST_SIZE(ehci_hcd, 2);
      break;
      
    case 512:
      EHCI_SET_FRAME_LIST_SIZE(ehci_hcd, 1);
      break;
      
    case 1024:
      EHCI_SET_FRAME_LIST_SIZE(ehci_hcd, 0);
      break;
      
    default:
      DLOG("Unsupported frame list size: %d", frm_lst_size);
      return FALSE;
    }
  }
  else {
    if(frm_lst_size != DEFAULT_FRM_LST_SIZE) {
      DLOG("Unsupported frame list size: %d", frm_lst_size);
      return FALSE;
    }
  }
  
  while(frm_lst_size--) {
    CLEAR_FRAME_LIST_LINK_POINTER(frame_list[frm_lst_size]);
    last_frame_list_entries[frm_lst_size] = frame_list[frm_lst_size];
  }
  
  ehci_hcd->regs->frame_list = (uint32_t)get_phys_addr(ehci_hcd->frame_list);

  gccmb();
  EHCI_ENABLE_PERIODIC(ehci_hcd);
  return TRUE;
}

static inline bool
initialise_async_head(ehci_hcd_t* ehci_hcd)
{
  qh_t* async_head;
  ehci_hcd->async_head = async_head = allocate_qh(ehci_hcd);

  if(async_head == NULL) return FALSE;
  
  /* Set head to point to itself */
  async_head->horizontalPointer.raw = QH_NEXT(ehci_hcd, ehci_hcd->async_head);
  async_head->hw_info1 = QH_HEAD;
  async_head->qtd_token_raw = QTD_HALT;
  async_head->next_qtd_ptr_raw = EHCI_LIST_END;
  async_head->alt_qtd_ptr_raw
    = EHCI_QTD_VIRT_TO_PHYS(ehci_hcd, async_head->dummy_qtd);
  async_head->state = QH_STATE_LINKED;
  
  ehci_hcd->regs->async_next = EHCI_QH_VIRT_TO_PHYS(ehci_hcd, async_head);

  INIT_LIST_HEAD(&ehci_hcd->reclaim_list);
  
  gccmb();
  EHCI_ENABLE_ASYNC(ehci_hcd);

  //gccmb();
  //EHCI_INT_ASYNC_DOORBELL(ehci_hcd);
  
  return TRUE;
}

static inline bool
set_interrupt_threshold(ehci_hcd_t* ehci_hcd)
{
  if(!EHCI_IS_HALTED(ehci_hcd)) {
    return FALSE;
  }
    
  uint32_t int_threshold = ehci_hcd->interrupt_threshold;
  switch(int_threshold) {
  case 1:
  case 2:
  case 4:
  case 8:
  case 16:
  case 32:
  case 64:
    EHCI_SET_INT_THRESHOLD(ehci_hcd, int_threshold);
    break;
    
  default:
    DLOG("Invalid value for interrupt threshold %d", int_threshold);
    return FALSE;
  }

  return TRUE;
}

static inline bool
set_ehci_on(ehci_hcd_t* ehci_hcd)
{
  if(!EHCI_IS_HALTED(ehci_hcd)) {
    return FALSE;
  }

  EHCI_RUN(ehci_hcd);

  return TRUE;
}

static inline void enable_interrupts(ehci_hcd_t* ehci_hcd)
{
  EHCI_ACK_INTERRUPTS(ehci_hcd, USBINTR_ALL);
  EHCI_SET_INTRS(ehci_hcd);
}

static bool
reset_root_port(ehci_hcd_t* ehci_hcd, ehci_port_t* port)
{
  if(EHCI_IS_HALTED(ehci_hcd)) {
    DLOG("Can only reset port if the EHCI chip is not halted");
    return FALSE;
  }
  
  if(!IS_PORT_POWERED(*port)) {
    POWER_PORT(*port);
  }

  /*
   * -- EM -- I don't know if it is necessary to check if a port
   * was successfully powered on or if there should be delay.
   * Right now all the ports I'm using the host controller
   * does not have port power control switches so I can't test it
   */
  if(!IS_PORT_POWERED(*port)) {
    DLOG("Cannot reset port that cannot be powered on");
    return FALSE;
  }

  DLOG("Port before reset 0x%X", *port);

  RESET_PORT(*port);

  DLOG("Port right after reset: 0x%X", *port);

  /*
   * sleep before restarting port, time specified in USB 2 Specs
   * section 7.1.7.5
   */
  sched_usleep(50000);

  PORT_LEAVE_RESET(*port);

  /*
   * Software must wait 2 ms for port to end reset, EHCI Specs page 28
   */
  
  if(!handshake(port, PORT_RESET, 0, 2000)) return FALSE;
  
  DLOG("Port after leave reset 0x%X", *port);
  
  return TRUE;
}

bool ehci_post_enumeration(usb_hcd_t* usb_hcd)
{
  
  ehci_hcd_t* ehci_hcd = hcd_to_ehci_hcd(usb_hcd);
  enable_interrupts(ehci_hcd);
  print_caps_and_regs_info(hcd_to_ehci_hcd(usb_hcd), "In ehci_post_enumeration");
  
  return TRUE;
}

static bool
ehci_reset_root_ports(usb_hcd_t* usb_hcd)
{
  uint32_t num_ports;
  ehci_port_t *ports;
  ehci_hcd_t* ehci_hcd;
  
  DLOG("Reset root ports");

  ehci_hcd = hcd_to_ehci_hcd(usb_hcd);
  num_ports = ehci_hcd->num_ports;
  ports = ehci_hcd->regs->port_status;
  
  while(num_ports--) {
    DLOG("Resetting port %d", num_ports);
    if(!reset_root_port(ehci_hcd, &ports[num_ports])) {
      DLOG("Failed to reset port: %d", num_ports);
      return FALSE;
    }
  }

  return TRUE;
}

static int ehci_submit_urb(struct urb* urb,
                           gfp_t mem_flags)
{
  uint32_t pipe = urb->pipe;
  ehci_hcd_t* ehci_hcd = hcd_to_ehci_hcd(urb->dev->hcd);
  uint16_t packet_len = usb_maxpacket(urb->dev, urb->pipe);
  
  if(packet_len == 0) {
    DLOG("packet_len == 0");
    panic("packet_len == 0");
  }
  register uint32_t pipe_type = usb_pipetype(pipe);
  switch(pipe_type) {
  case PIPE_ISOCHRONOUS:
    return ehci_isochronous_transfer(ehci_hcd, urb);
  case PIPE_INTERRUPT:
    
    DLOG("EHCI Interrupt not implemented");
    panic("EHCI Interrupt not implemented");
    
  case PIPE_CONTROL:
  case PIPE_BULK:
    return ehci_async_transfer(ehci_hcd, urb);
  }

  return -1;
}

void ehci_kill_urb(struct urb* urb)
{
  //ehci_hcd_t* ehci_hcd = hcd_to_ehci_hcd(urb->dev->hcd);
  DLOG("ehci_kill_urb not implemented");
  panic("ehci_kill_urb not implemented");
}

/*
 * Host controller initiailization sequence taken from page 53 of the
 * EHCI Specs
 */

static u32 temp_stack[1024] ALIGNED (0x1000);
static bool
initialise_ehci_hcd(uint32_t usb_base,
                    ehci_hcd_t* ehci_hcd,
                    frm_lst_lnk_ptr_t* frm_lst,
                    frm_lst_lnk_ptr_t* last_frm_lst_entries,
                    uint32_t* micro_frame_remaining_bytes,
                    uint32_t frm_lst_size,
                    qh_t* qh_pool,
                    uint32_t qh_pool_size,
                    uint32_t* used_qh_bitmap,
                    qtd_t* qtd_pool,
                    uint32_t qtd_pool_size,
                    uint32_t* used_qtd_bitmap,
                    itd_t* itd_pool,
                    uint32_t itd_pool_size,
                    uint32_t* used_itd_bitmap,
                    ehci_completion_element_t* ehci_completion_element_pool,
                    uint32_t ehci_completion_element_pool_size,
                    uint32_t* used_ehci_completion_element_bitmap,
                    uint32_t int_threshold)
{
  uint32_t i;
  if(!initialise_usb_hcd(ehci_hcd_to_hcd(ehci_hcd),
                         USB_TYPE_HC_EHCI, ehci_reset_root_ports,
                         ehci_post_enumeration,
                         ehci_submit_urb,
                         ehci_kill_urb)) {
    return FALSE;
  }
  
  ehci_hcd->base_physical_address = (phys_addr_t)usb_base;
  ehci_hcd->base_virtual_address = map_virtual_page(usb_base | 0x3);
  ehci_hcd->caps = ehci_hcd->base_virtual_address;
  ehci_hcd->regs = (ehci_regs_t*)(ehci_hcd->base_virtual_address
                                  + ehci_hcd->caps->cap_length);
  

  memset(ehci_hcd->ehci_devinfo, 0, sizeof(ehci_hcd->ehci_devinfo));
  
  ehci_hcd->frame_list              = frm_lst;
  ehci_hcd->last_frame_list_entries = last_frm_lst_entries;
  ehci_hcd->frame_list_size         = frm_lst_size;
  ehci_hcd->micro_frame_remaining_bytes = micro_frame_remaining_bytes;

  INIT_LIST_HEAD(&ehci_hcd->uninserted_itd_urb_list);
  
  for(i = 0; i < frm_lst_size * 8; ++i) {
    micro_frame_remaining_bytes[i] = EHCI_BYTES_PER_MICRO_FRAME;
  }

  spinlock_init(&ehci_hcd->completion_lock);
  spinlock_init(&ehci_hcd->uninserted_itd_lock);


#define INIT_EHCI_POOL(type)                                            \
  do {                                                                  \
    ehci_hcd->type##_pool = type##_pool;                                \
    ehci_hcd->type##_pool_phys_addr = (phys_addr_t)get_phys_addr(type##_pool); \
    ehci_hcd->type##_pool_size = type##_pool_size;                      \
    ehci_hcd->used_##type##_bitmap = used_##type##_bitmap;              \
    memset(type##_pool, 0, type##_pool_size * sizeof(type##_t));        \
  }                                                                     \
  while(0)
  
  INIT_EHCI_POOL(qh);
  INIT_EHCI_POOL(qtd);
  INIT_EHCI_POOL(itd);
  INIT_EHCI_POOL(ehci_completion_element);

#undef INIT_EHCI_POOL
  
  ehci_hcd->interrupt_threshold = int_threshold;
  ehci_hcd->num_ports           = GET_NUM_PORTS(ehci_hcd);

  /*
   * Set used_qh_bitmap and used_qtd_bitmap to all zeros to
   * mark all elements as free
   */

  ehci_hcd->used_qh_bitmap_size =
    calc_used_qh_bitmap_size(ehci_hcd);

  memset(used_qh_bitmap, 0,
         ehci_hcd->used_qh_bitmap_size * sizeof(*used_qh_bitmap));
  
  ehci_hcd->used_qtd_bitmap_size =
    calc_used_qtd_bitmap_size(ehci_hcd);

  memset(used_qtd_bitmap, 0,
         ehci_hcd->used_qtd_bitmap_size * sizeof(*used_qtd_bitmap));

  ehci_hcd->used_itd_bitmap_size =
    calc_used_itd_bitmap_size(ehci_hcd);

  memset(used_itd_bitmap, 0,
         ehci_hcd->used_itd_bitmap_size * sizeof(*used_itd_bitmap));
  
  if(!restart_ehci_hcd(ehci_hcd)) return FALSE;
  if(!initialise_frame_list(ehci_hcd)) return FALSE;
  if(!initialise_async_head(ehci_hcd)) return FALSE;

  INIT_LIST_HEAD(&ehci_hcd->completion_list);
  
  // Set desired interrupt threshold
  if(!set_interrupt_threshold(ehci_hcd)) return FALSE;

  gccmb();
  
  // Turn EHCI chip on
  if(!set_ehci_on(ehci_hcd)) return FALSE;

  gccmb();
 
  // Set all ports to route to EHCI chip
  EHCI_SET_CONFIG_FLAG(ehci_hcd, 1);

#ifdef EHCI_IOC_BUG
  
  create_kernel_thread_args((u32)ioc_backup_thread,
                            (u32) &temp_stack[1023],
                            "EHCI Backup",
                            TRUE, 1, ehci_hcd);
  
#endif // EHCI_IOC_BUG
  
  return TRUE;
}


/*
 * Called to initialise EHCI, functionality mimics uhci_init in
 *  uhci_hcd
 */
bool
ehci_init(void)
{
  uint32_t usb_base = 0;
  uint i, device_index, irq_pin;
  pci_device ehci_device;
  pci_irq_t irq;

  if(mp_ISA_PC) {
    DLOG("Cannot operate without PCI");
    DLOGV("Exiting %s with FALSE", __FUNCTION__);
    return FALSE;
  }

  /* Find the EHCI device on the PCI bus */
  device_index = ~0;
  i = 0;

  /*
   * -- WARN -- Only looking for 1 specific EHCI host controller device
   * this is D29 for Intel 6 C200 or the qemu ehci chip would be best
   * to add all EHCI chips to an array that is iterated, and each time
   * one is found it is pushed to the usb core
   */
  
  while (pci_find_device (0x8086, 0x1C2D, 0x0C, 0x03, i, &i)) {
    if (pci_get_device (i, &ehci_device)) { 
      if (ehci_device.progIF == 0x20) {
        device_index = i;
        break;
      }
      i++;
    } else break;
  }


  if(device_index == ~0) {
    while (pci_find_device (0x8086, 0x24CD, 0x0C, 0x03, i, &i)) { 
      if (pci_get_device (i, &ehci_device)) { 
        if (ehci_device.progIF == 0x20) {
          device_index = i;
          break;
        }
        i++;
      } else break;
    }
  }

  if(device_index == ~0) {
    while (pci_find_device (0x8086, 0x1c26, 0x0C, 0x03, i, &i)) { 
      if (pci_get_device (i, &ehci_device)) { 
        if (ehci_device.progIF == 0x20) {
          device_index = i;
          break;
        }
        i++;
      } else break;
    }
  }

  
  DLOG("Device %d", device_index);
  
  if (device_index == ~0) {
    DLOG ("Unable to find compatible device on PCI bus");
    DLOGV("Exiting %s with FALSE", __FUNCTION__);

    EHCI_DEBUG_HALT();
    return FALSE;
  }

  

  DLOGV("Found device on PCI bus");
  
  if (!pci_get_device (device_index, &ehci_device)) {
    DLOG ("Unable to get PCI device from PCI subsystem");
    DLOGV("Exiting %s with FALSE", __FUNCTION__);
    EHCI_DEBUG_HALT();
    return FALSE;
  }
  
  ehci_hcd.bus = ehci_device.bus;
  ehci_hcd.dev = ehci_device.slot;
  ehci_hcd.func = ehci_device.func;
  
  DLOG ("Using PCI bus=%x dev=%x func=%x", ehci_hcd.bus,
        ehci_hcd.dev, ehci_hcd.func);

  if (!pci_get_interrupt (device_index, &ehci_hcd.irq_line, &irq_pin)) {
    DLOG ("Unable to get IRQ");
    DLOGV("Exiting %s with FALSE", __FUNCTION__);
    EHCI_DEBUG_HALT();
    return FALSE;
  }
  
  DLOG ("Using IRQ pin=%X", irq_pin);
  
  if (pci_irq_find (ehci_hcd.bus, ehci_hcd.dev, irq_pin, &irq)) {
    /* use PCI routing table */
    DLOG ("Found PCI routing entry irq.gsi=0x%x", irq.gsi);
    if (! (ehci_hcd.handler_vector =
           pci_irq_map_handler (&irq, ehci_irq_handler, 0x01,
                                IOAPIC_DESTINATION_LOGICAL,
                                IOAPIC_DELIVERY_FIXED)) ) {
      DLOG ("Unable to map IRQ handler");
      return FALSE;
    }
    ehci_hcd.irq_line = irq.gsi;
  }
  
  if (!pci_decode_bar (device_index, 0, &usb_base, NULL, NULL)) {
    DLOG ("unable to decode BAR0");
    EHCI_DEBUG_HALT();
    return FALSE;
  }

  DLOG("usb base from BAR0 = 0x%.04X", usb_base);

  
  if(!initialise_ehci_hcd(usb_base, &ehci_hcd, frame_list,
                          last_frame_list_entries,
                          micro_frame_remaining_bytes,
                          sizeof(frame_list)/sizeof(frm_lst_lnk_ptr_t),
                          qh_pool,
                          sizeof(qh_pool)/sizeof(qh_t),
                          used_qh_bitmap,
                          qtd_pool,
                          sizeof(qtd_pool)/sizeof(qtd_t),
                          used_qtd_bitmap,
                          itd_pool,
                          sizeof(itd_pool)/sizeof(itd_t),
                          used_itd_bitmap,
                          ehci_completion_element_pool,
                          sizeof(ehci_completion_element_pool)
                          / sizeof(ehci_completion_element_t),
                          used_ehci_completion_element_bitmap,
                          DEFAULT_INT_THRESHOLD)) {
    EHCI_DEBUG_HALT();
    return FALSE;
  }

  if(!add_usb_hcd(&(ehci_hcd.usb_hcd))) {
    EHCI_DEBUG_HALT();
    return FALSE;
  }


  
  DLOG("Successfully initialised and registered ehci hcd");
  return TRUE;
}



static uint32_t
qtd_fill(qtd_t* qtd,
         phys_addr_t phys_data_addr,   /* Use physical address here */
         uint32_t data_len,
         uint32_t max_packet_len,
         uint32_t token)
{
  uint32_t count, i;
  
  qtd->buffer_page[0].raw = phys_data_addr;
  qtd->ex_buf_ptr_pgs[0] = 0;
  qtd->ex_buf_ptr_pgs[1] = 0;
  qtd->ex_buf_ptr_pgs[2] = 0;
  qtd->ex_buf_ptr_pgs[3] = 0;
  qtd->ex_buf_ptr_pgs[4] = 0;

  /* Number of bytes buffer_page[0] stores */
  count = 0x1000 - (phys_data_addr & 0x0FFF);
  if(data_len < count) {
    count = data_len;
  }
  else {
    /* Increment to next 4K boundary */
    phys_data_addr += 0x1000;
    phys_data_addr &= ~0x0FFF;

    for(i = 1; i < 5 && count < data_len; ++i) {
      qtd->buffer_page[i].raw = phys_data_addr;
      phys_data_addr += 0x1000; /* Increment to next 4k */
      count += 0x1000;
      if(count > data_len) count = data_len;
    }

    /*
     * If this qtd does not fufill the entire transmission then we
     * need to backtrack count so that it lies on a max_packet_len
     * boundary so we do not send a short packet because they are only
     * allowed at the end of a transmission
     */
    if(count != data_len) count -= (count % max_packet_len);

  }
  
  qtd->token = (count << 16) | token;
  
  return count;
}

static bool
create_qtd_chain(ehci_hcd_t* ehci_hcd, 
                 uint8_t address,
                 addr_t setup_req,    /* Use virtual address here */
                 uint32_t setup_len,
                 addr_t data,   /* Use virtual address here */
                 uint32_t data_len,
                 uint32_t packet_len,
                 uint32_t pipe_type,
                 uint32_t is_input,
                 bool enable_ioc,
                 list_head_t* qtd_list) /* Should be empty/uninitialized */
{
  uint32_t    token;
  qtd_t*      current_qtd;
  qtd_t*      previous_qtd;
  phys_addr_t data_phys_addr;
    
  token = QTD_ACTIVE | (EHCI_TUNE_CERR << QTD_CERR);
  INIT_LIST_HEAD(qtd_list);
  current_qtd = allocate_qtd(ehci_hcd);
  
  if(current_qtd == NULL) return FALSE;
  
  list_add_tail(&current_qtd->chain_list, qtd_list);
  
  if(pipe_type == PIPE_CONTROL) {
    if(qtd_fill(current_qtd, (phys_addr_t)get_phys_addr(setup_req),
                setup_len, packet_len, token | QTD_SETUP) != setup_len) {
      goto create_qtd_chain_cleanup;
    }
    
    token ^= QTD_TOGGLE;

    previous_qtd = current_qtd;
    current_qtd = allocate_qtd(ehci_hcd);
    if(!current_qtd) goto create_qtd_chain_cleanup;

    list_add_tail(&current_qtd->chain_list, qtd_list);

    if(data_len == 0) {
      token |= QTD_INPUT;
    }
    previous_qtd->next_pointer_raw = EHCI_QTD_VIRT_TO_PHYS(ehci_hcd, current_qtd);
  }

  /*
   * If zero length data stage then data is input or if request type
   * specifies that data is input
   */
  if(is_input) {
    token |= QTD_INPUT;
  }

  data_phys_addr = (phys_addr_t)get_phys_addr(data);
  
  while(1) {
    int this_qtd_length
      = qtd_fill(current_qtd, data_phys_addr, data_len, packet_len, token);
    data_phys_addr += this_qtd_length;

    if(is_input) {
      current_qtd->alt_pointer_raw = ehci_hcd->async_head->alt_qtd_ptr_raw;
    }


    /*
     * Toggle control bit if necessary.  This test is checking whether
     * an even or odd number of packets was put into the current qtd
     * and flipping the toggle bit if necessary
     */
    
    if ((packet_len & (this_qtd_length + (packet_len - 1))) == 0) {
      token ^= QTD_TOGGLE;
    }

    if(this_qtd_length >= data_len) break;

    data_len -= this_qtd_length;
    
    previous_qtd = current_qtd;
    current_qtd = allocate_qtd(ehci_hcd);
    if(!current_qtd) goto create_qtd_chain_cleanup;
    
    list_add_tail(&current_qtd->chain_list, qtd_list);

    previous_qtd->next_pointer_raw = EHCI_QTD_VIRT_TO_PHYS(ehci_hcd, current_qtd);
  }


  /* Keep queue head running */
  current_qtd->alt_pointer_raw = EHCI_LIST_END;
  
  /*
   * If this is a control request then we need to send a status ack.
   * If it is a bulk we need to send a terminating short packet of
   * length zero if data is not on a packet boundary, USB 2 Specs
   * Sections 5.5 and 5.8 (respectively) and section 8.5
   */

  

  if(data_len != 0) {
    bool send_one_more = FALSE;

    if(pipe_type == PIPE_CONTROL) {
      send_one_more = TRUE;
      /* Switch in and out from what data stage was */
      token ^= 0x0100;
      token |= QTD_TOGGLE; /* Always DATA1 */
    }
    else if(pipe_type == PIPE_BULK && !(data_len % packet_len)) {

      /*
       * Some buggy devices need this but turning it off by now will
       * take care of this later when we have an URB system
       */
      //send_one_more = TRUE;
    }

    

    if(send_one_more) {
      previous_qtd = current_qtd;
      current_qtd = allocate_qtd(ehci_hcd);
      if(!current_qtd) goto create_qtd_chain_cleanup;
      
      list_add_tail(&current_qtd->chain_list, qtd_list);
      
      previous_qtd->next_pointer_raw
        = EHCI_QTD_VIRT_TO_PHYS(ehci_hcd, current_qtd);

      qtd_fill(current_qtd, 0, 0, 0, token);
      
    }
  }

  if(enable_ioc) {
    /* Interrupt on complete */
    QTD_ENABLE_IOC(current_qtd);
  }
  
  return TRUE;
  
 create_qtd_chain_cleanup:
  /* -- !! -- Unimplemented */
  
  DLOG("Unimplemented cleanup in create_qtd_chain!!!!!!!");
  panic("Unimplemented cleanup in create_qtd_chain!!!!!!!");
  return FALSE;
}

/*
 * Same idea as linux qh_refresh combined with qh_update, figure out
 * which qtd to use and then set the qh with that qtd
 */
static void qh_refresh(ehci_hcd_t* ehci_hcd, qh_t* qh)
{
  qtd_t *qtd;
  
  if (list_empty (&qh->qtd_list)) {
    qtd = qh->dummy_qtd;
  }
  else {
    qtd = list_entry(qh->qtd_list.next, qtd_t, chain_list);
    /*
     * first qtd may already be partially processed in which case we
     * don't do anything
     */ 
    if(qh->current_qtd_ptr_raw == EHCI_QTD_VIRT_TO_PHYS(ehci_hcd, qtd)) {
      qtd = NULL;
    }
  }
  
  if (qtd) {
    if(qh->state == QH_STATE_LINKED) {
      DLOG("Should never reached here if qh->state == QH_STATE_LINKED");
      panic("Should never reached here if qh->state == QH_STATE_LINKED");
    }
    qh->next_qtd_ptr_raw = EHCI_QTD_VIRT_TO_PHYS(ehci_hcd, qtd);
    qh->alt_qtd_ptr_raw  = EHCI_LIST_END;
    
    if(!(qh->hw_info1 & QH_DATA_TOGGLE_CONTROL)) {
      qh->data_toggle =
        IS_ENDPOINT_TOGGLED(qh->dev, QH_GET_ENDPOINT(qh), QTD_IS_INPUT(qtd));
    }
  }
  
  qh->qtd_token_raw &= (QTD_TOGGLE | QTD_STS_PING);
  
}

static void qh_prep(ehci_hcd_t* ehci_hcd, qh_t* qh, uint32_t endpoint,
                    uint32_t device_addr, uint32_t max_packet_len,
                    uint32_t dev_speed, uint32_t pipe_type, bool is_input)
{
  uint32_t info1 = 0;
  uint32_t info2 = 0;
  

  if(device_addr != 0) {
    usb_hcd_t* usb_hcd = ehci_hcd_to_hcd(ehci_hcd);
    qh->dev = &usb_hcd->devinfo[device_addr];
  }

  info1  = device_addr;
  info1 |= endpoint << 8;

  if(pipe_type == PIPE_INTERRUPT) {
   
    /*
     * -- !! -- Unimplemented, interrupt pipes have extra work should be
     * done will get to this later, see Linux ehci-q.c function
     * qh_make
     */

    DLOG("Unimplemented case in file %s line %d", __FILE__, __LINE__);
    panic("Unimplemented case");
  }

  switch(dev_speed) {
  case USB_SPEED_LOW:
    info1 |= (1 << 12); // Set EPS to low
  case USB_SPEED_FULL:

    /* full speed -> EPS = 0 */

    if(pipe_type != PIPE_INTERRUPT) {
      info1 |= (EHCI_TUNE_RL_HS << 28);
    }
    if(pipe_type == PIPE_CONTROL) {
      info1 |= (1 << 27);     /* for TT */
      info1 |= 1 << 14;       /* toggle from qtd */
    }

    info1 |= max_packet_len << 16;

    info2 |= (EHCI_TUNE_MULT_TT << 30);

    /* -- !! -- Should set ttport not doing it right now */
    DLOG("this has not been tested if you are seeing this you need to know that and then can remove this panic file %s function %s line %d",
         __FILE__, __FUNCTION__, __LINE__);
    panic("this has not been tested if you are seeing this you need to know that and then can remove this panic");

    break;

  case USB_SPEED_HIGH:
    info1 |= (2 << 12); /* Set end point speed to high */
    if(pipe_type == PIPE_CONTROL) {
      info1 |= (EHCI_TUNE_RL_HS << 28); /* Set nak throttle */
      
      info1 |= (64 << 16); /* Setting max packet length which for usb
                              2 control pipes must be 64 */
      
      info1 |= QH_DATA_TOGGLE_CONTROL; /* Set queue head to get initial data toggle
                                          from qtd */

      info2 |= (EHCI_TUNE_MULT_HS << 30); /* set transactions per
                                             microframe */
    }
    else if (pipe_type == PIPE_BULK) {
      info1 |= (EHCI_TUNE_RL_HS << 28); /* Set nak throttle */

      info1 |= max_packet_len << 16; /* Set max packet length */

      info2 |= (EHCI_TUNE_MULT_HS << 30); /* set transactions per
                                             microframe */
    }
    else if (pipe_type == PIPE_INTERRUPT) {
      /*
       * --!! Unimplemented, will add logic for interrupt pipes
       * later, see Linux ehci-q.c function qh_make
       */
      DLOG("Unimplemented case in file %s line %d", __FILE__, __LINE__);
      panic("Unimplemented case");
    }
    break;
  default:
    /*
     * -- !! -- Should fail more gracefully here but don't care right
     * now
     */
    DLOG("Unknown device speed, see file %s line %d", __FILE__, __LINE__);
    panic("Unknown device speed");
  }

  qh->hw_info1 = info1;
  qh->hw_info2 = info2;

  qh_refresh(ehci_hcd, qh);
}


void qh_append_qtds(ehci_hcd_t* ehci_hcd, qh_t* qh, list_head_t* qtd_list,
                    uint32_t pipe_type)
{
  
  qtd_t* qtd;
  qtd_t* dummy;
  uint32_t token;
  phys_addr_t new_dummy_phys_addr;
  if(list_empty(qtd_list)) return;

  qtd = list_entry(qtd_list->next, qtd_t, chain_list);

  token = qtd->token;
  qtd->token = QTD_HALT;
  gccmb();

  dummy = qh->dummy_qtd;
  *dummy = *qtd;

  list_del(&qtd->chain_list);
  list_add(&dummy->chain_list, qtd_list);
  list_splice_tail(qtd_list, &qh->qtd_list);
  initialise_qtd(ehci_hcd, qtd);
  qh->dummy_qtd = qtd;
  new_dummy_phys_addr = EHCI_QTD_VIRT_TO_PHYS(ehci_hcd, qtd);
  qtd = list_entry(qh->qtd_list.prev, qtd_t, chain_list);
  qtd->next_pointer_raw = new_dummy_phys_addr;
  gccmb();
  dummy->token = token;
}

/*
 * Simplified version of UnlinkQueueHead on page 72 of EHCI
 * specifications because we are only removing one queue head
 */
static inline void unlink_single_queue_head(qh_t* previous, qh_t* headToUnlink)
{
  previous->horizontalPointer = headToUnlink->horizontalPointer;
}

static void add_qh_to_reclaim_list(ehci_hcd_t* ehci_hcd, qh_t* qh)
{
  qh->state = QH_STATE_RECLAIM;
  list_add(&qh->reclaim_chain, &ehci_hcd->reclaim_list);
}

void link_qh_to_async(ehci_hcd_t* ehci_hcd, qh_t* qh, bool ioc_enabled,
                      uint32_t pipe_type, uint32_t address,
                      uint16_t endpoint, bool is_input, bool save_qh)
{
  if(qh->state == QH_STATE_NOT_LINKED) {
    qh_refresh(ehci_hcd, qh);
  }

  qh->state = QH_STATE_LINKED;
  qh->horizontalPointer = ehci_hcd->async_head->horizontalPointer;
  gccmb();
  ehci_hcd->async_head->horizontalPointer.raw = QH_NEXT(ehci_hcd, qh);
}

static sint32 spin_for_transfer_completion(ehci_hcd_t* ehci_hcd,
                                           qh_t* qh, uint8_t address,
                                           uint32_t endpoint,
                                           uint8_t pipe_type, bool is_input,
                                           bool save_qh)
{
  qtd_t* last_qtd;
  qtd_t* temp_qtd;
  list_for_each_entry(temp_qtd, &qh->qtd_list, chain_list) {
    last_qtd = temp_qtd;
  }

  /*
   * -- EM -- I am not sure if wait spinning on the last qtd is
   * sufficient to make sure the queue head is done (besides the
   * fact that we might not detect errors on qtds that don't occur
   * at the end), if it is not this could result in some nasty
   * side-effects of us trying to use a qh that is not finished,
   * although combined with the doorbell I think it is safe
   */

  /*
   * -- EM --Here is an outline of the precedding code.  I am not
   * sure if it is the most efficient but I am pretty sure it is
   * sufficient (except for the note above)
   *
   * 1) Check if the iaa interrupt is on, if it is panic, could be
   * removed later once we know everything is working
   *
   * 2) Spin until the last qtd is complete or an error occurs
   * (don't have error checking yet which is very bad)
   *
   * 3) Unlink 
   *
   * 4) Acknowledge current doorbell to clear the bit
   *
   * 5) Add the queue and its tds to be reclaimed
   */

  /* Step 1 */
  if( EHCI_INTR_ENABLED(ehci_hcd, USBINTR_IAA) ) {
    /*
     * -- !! -- Should fail more gracefully here but its not that
     * big a deal since non-ioc transactions should only occur at
     * boot
     */
    DLOG("Should only be using non-ioc transactions "
         "if the interrupt is already disabled");
    panic("Should only be using non-ioc transactions "
          "if the interrupt is already disabled");
  }

  /*
   * -- !! -- If an error occurs and it isn't on the last qtd then
   * this will spin forever -> very bad also this is a hack right
   * now to figure out if a td is never going to end would be better
   * to check various things for errors
   */
  /* Step 2 */
  uint32_t spin_tick = 0;
  while(last_qtd->token & QTD_ACTIVE) {
    tsc_delay_usec(100);
    if(++spin_tick == 100000) {
      print_qh_info(ehci_hcd, qh, TRUE, "In Spin");
      print_caps_and_regs_info(ehci_hcd, "In Spin");
      while(1);
    }
  }
  
  if(pipe_type == PIPE_BULK) {
    USB_DEVICE_INFO* dev_info = &ehci_hcd_to_hcd(ehci_hcd)->devinfo[address];
    SET_ENDPOINT_TOGGLE(dev_info, endpoint, is_input, qh->data_toggle );
  }

  if(last_qtd->token & QTD_HALT) {
    print_caps_and_regs_info(ehci_hcd, "in failure");
    print_qh_info(ehci_hcd, qh, TRUE, "in failure");
    DLOG("pci status in failure = 0x%X",
         pci_get_status(ehci_hcd->bus, ehci_hcd->dev,ehci_hcd->func));
    DLOG("pci command in failure = 0x%X",
         pci_get_command(ehci_hcd->bus, ehci_hcd->dev,ehci_hcd->func));

    /*-- !! -- Need a better cleanup here */
    DLOG("%s failed at line %d", __FUNCTION__, __LINE__);
    panic("link_qh_to_async failed");
  }

  list_for_each_entry(temp_qtd, &qh->qtd_list, chain_list) {
    if(temp_qtd->total_bytes_to_transfer != 0) {
      
      DLOG("Failed to transfer everything");
      DLOG("%d bytes left", temp_qtd->total_bytes_to_transfer);
      panic("Failed to transfer everything");
    }
  }

  if(save_qh) {
    qtd_t* qtd_to_remove;
    qtd_t* qtd_tmp;
  
    list_for_each_entry_safe(qtd_to_remove, qtd_tmp, &qh->qtd_list, chain_list) {
      if(EHCI_QTD_VIRT_TO_PHYS(ehci_hcd, qtd_to_remove) == qh->current_qtd_ptr_raw) {
        break;
      }
      else {
        list_del(&qtd_to_remove->chain_list);
        free_qtd(ehci_hcd, qtd_to_remove);
      }
    }
  
    if(list_empty(&qh->qtd_list)) {
      /*
       * -- EM -- ALL QTDS REMOVED this can happen because a silicon
       * quirk with some ehci host controllers moving the dummy qtd as
       * the active one, I have not tested the functionally when this
       * occurs so all bets are off
       */
      DLOG("ALL QTDS REMOVED see comment at file %s line %d", __FILE__, __LINE__);
      panic("ALL QTDS REMOVED");
    }

  }
  else {
    /* Step 3 */
    unlink_single_queue_head(ehci_hcd->async_head, qh);
      
    /* Step 4 */
    EHCI_ACK_DOORBELL(ehci_hcd); /* Ack current doorbell to clear bit */
      
    /* Step 5 */
    add_qh_to_reclaim_list(ehci_hcd, qh);
    qh->state = QH_STATE_RECLAIM;
    
  }
  
  if(pci_get_status(ehci_hcd->bus, ehci_hcd->dev,ehci_hcd->func) & 0x2000) {
    DLOG("EHCI PCI master abort: function %s line %d", __FUNCTION__, __LINE__);
    panic("PCI master abort");
  }
  
  return 0;
}

static sint32 submit_async_qtd_chain(ehci_hcd_t* ehci_hcd, qh_t** qh,
                                     uint32_t endpoint, uint32_t device_addr,
                                     uint32_t dev_speed, uint32_t pipe_type,
                                     uint32_t max_packet_len, bool is_input,
                                     bool ioc_enabled, list_head_t* qtd_list,
                                     bool save_qh)
{
  sint32 result;
  if(*qh == NULL) {
    *qh = allocate_qh(ehci_hcd);
    qh_prep(ehci_hcd, *qh, endpoint, device_addr,
            max_packet_len, dev_speed, pipe_type, is_input);
  }
  if(save_qh) {
    EHCI_SET_DEVICE_QH(ehci_hcd, device_addr, is_input, endpoint, *qh);
  }
  
  qh_append_qtds(ehci_hcd, *qh, qtd_list, pipe_type);
  if((*qh)->state == QH_STATE_NOT_LINKED) {
    link_qh_to_async(ehci_hcd, *qh, ioc_enabled, pipe_type,
                     device_addr, endpoint, is_input, save_qh);
  }
  if(ioc_enabled) {
    print_caps_and_regs_info(ehci_hcd, "just turned doorbell on");
    return 0;
  }
  else {
    
    if((result = spin_for_transfer_completion(ehci_hcd, *qh, device_addr,
                                              endpoint, pipe_type, is_input,
                                              save_qh)) < 0) {

      DLOG("spin_for_transfer_completion returned %d", result);
      return result;
    }
  }
  
  return 0;
}

static int
ehci_async_transfer(ehci_hcd_t* ehci_hcd, struct urb* urb)
{
  unsigned int pipe = urb->pipe;
  uint8_t address = usb_pipedevice(pipe);
  bool is_input = usb_pipein(pipe);
  uint8_t endpoint = usb_pipeendpoint(pipe);
  qh_t* qh  = EHCI_GET_DEVICE_QH(ehci_hcd, address, is_input, endpoint);
  list_head_t qtd_list;
  uint8_t pipe_type = usb_pipetype(pipe);
  int packet_len = usb_maxpacket(urb->dev, urb->pipe);
  
  
  if(!create_qtd_chain(ehci_hcd, address, urb->setup_packet, sizeof(USB_DEV_REQ),
                       urb->transfer_buffer,
                       urb->transfer_buffer_length, packet_len,
                       pipe_type, is_input,
                       mp_enabled, &qtd_list)) {
    return -1;
  }
  
  submit_async_qtd_chain(ehci_hcd, &qh, endpoint, address,
                         USB_SPEED_HIGH, pipe_type, packet_len,
                         is_input, mp_enabled,
                         &qtd_list, TRUE);
  
  return 0;
}

int itd_fill(itd_t* itd, uint8_t address, uint8_t endpoint,
             int max_packet_len, bool is_input, phys_addr_t data_phys,
             usb_iso_packet_descriptor_t* packets, int num_packets)
{
  
  int i;
  int cur_buf_ptr = 0;
  uint32_t old_buf_ptr_value;
  phys_addr_t data_phys_start = data_phys;

  data_phys = data_phys_start + packets[0].offset;
  old_buf_ptr_value = data_phys & ~0x0FFF;
  
  itd->ex_buf_ptr_pgs[0] = 0;
  itd->ex_buf_ptr_pgs[1] = 0;
  itd->ex_buf_ptr_pgs[2] = 0;
  itd->ex_buf_ptr_pgs[3] = 0;
  itd->ex_buf_ptr_pgs[4] = 0;
  itd->ex_buf_ptr_pgs[5] = 0;
  itd->ex_buf_ptr_pgs[6] = 0;

  EHCI_ASSERT(max_packet(max_packet_len) > 0);
  EHCI_ASSERT(max_packet(max_packet_len) <= 1024);
  EHCI_ASSERT(hb_mult(max_packet_len) > 0);
  EHCI_ASSERT(hb_mult(max_packet_len) <= 3);

  itd->next_link_pointer.raw = EHCI_LIST_END;

  itd->buf_ptr[0] = old_buf_ptr_value;
  itd->buf_ptr[0] |= address;
  itd->buf_ptr[0] |= (endpoint << 8);
  itd->buf_ptr[1] = max_packet(max_packet_len);

  
  if(is_input) {
    itd->buf_ptr[1] |= ITD_INPUT;
  }
  
  itd->buf_ptr[2] = hb_mult(max_packet_len);

  for(i = 0; i < num_packets; ++i) {
    data_phys = data_phys_start + packets[i].offset;
    
    /*
     * Check to see if we have crossed a page boundary and if so fill
     * the buffer pointer with the appropriate address and increment
     * our current buffer pointer
     */
    if(old_buf_ptr_value != (data_phys & ~0x0FFF)) {
      if(cur_buf_ptr == 6) {
        /*
         * This can occur if the offset between iso packets is too
         * larger, if this occurs we cannot make an iTD that can
         * handle the given packets
         */
        DLOG("Cannot fit iso packets into a single iTD, crosses "
             "too many buffer boundaries");
        return -1;
      }
      old_buf_ptr_value = data_phys & ~0x0FFF;
      cur_buf_ptr++;
      ITD_SET_BUF_PTR(itd, cur_buf_ptr, data_phys);
    }
    
    itd->transaction[i].raw = ITD_ACTIVE;
    itd->transaction[i].offset = data_phys & 0x0FFF;
    itd->transaction[i].length = packets[i].length;
    itd->transaction[i].page_selector = cur_buf_ptr;
  }
  
  
  return 0;
}

static int create_itd_chain(ehci_hcd_t* ehci_hcd, uint8_t address,
                            uint8_t endpoint, addr_t data,
                            usb_iso_packet_descriptor_t* packets,
                            int num_packets, 
                            int max_packet_len, bool is_input,
                            list_head_t* itd_list, bool ioc)
{
  int i;
  phys_addr_t data_phys = (phys_addr_t)get_phys_addr(data);
  INIT_LIST_HEAD(itd_list);
  
  i = 0;
  while(i < num_packets) {
    
    int packets_this_itd = 8 < (num_packets - i) ? 8 : num_packets - i;
    itd_t* current_itd = allocate_itd(ehci_hcd);
    usb_iso_packet_descriptor_t* itd_packets = &(packets[i]);
    
    if(current_itd == NULL) {
      DLOG("Failed to allocate itd");
      panic("Failed to allocate itd");
      return -1;
    }
    
    list_add_tail(&current_itd->chain_list, itd_list);
    
    if(itd_fill(current_itd, address, endpoint, max_packet_len,
                is_input, data_phys, itd_packets, packets_this_itd) < 0) {
      return -1;
    }

    i += packets_this_itd;
  }
  if(ioc) {
    itd_t* ioc_itd = list_entry(itd_list->prev, itd_t, chain_list);
    int packets_last_itd = num_packets & 0x7; // num_packets % 8
    if(packets_last_itd == 0) packets_last_itd = 8;
    ioc_itd->transaction[packets_last_itd-1].raw |= ITD_IOC;
  }
  
  return 0;
}


static int submit_itd_chain(ehci_hcd_t* ehci_hcd, list_head_t* itd_list,
                            struct urb* urb)
{
  itd_t* itd;
  ehci_iso_urb_priv_t* iso_urb_priv = get_iso_urb_priv(urb);
  frm_lst_lnk_ptr_t* frame_list  = ehci_hcd->frame_list;
  uint32_t current_frame_index, start_frame_index;
  uint32_t frame_list_size       = ehci_hcd->frame_list_size;
  uint32_t frame_list_mask       = frame_list_size - 1;
  uint32_t frame_interval_offset = iso_urb_priv->interval_offset / 8;
  uint32_t frame_interval        = urb->interval / 8;
  uint32_t frame_index;
  
  
  /*
   * -- EM -- Right now following the ideas of KISS and just adding a
   * single iTD to each index with nothing following it.  And that
   * nothing else is using the periodic list.  This will obviously be
   * changed later but right now just want to make sure the simplest
   * thing is working first
   */

  if(urb->real_time) {
    bool looped_around;
    if(frame_interval <= 1) {
      frame_interval = 1;
      current_frame_index = EHCI_CURRENT_FRAME_INDEX(ehci_hcd);
      frame_index = current_frame_index + EHCI_SAFE_FRAME_OFFSET(ehci_hcd);
    }
    else {
      current_frame_index = EHCI_CURRENT_FRAME_INDEX(ehci_hcd);
      frame_index = ((current_frame_index + EHCI_SAFE_FRAME_OFFSET(ehci_hcd))
                     | (frame_interval-1)) + 1 + frame_interval_offset;
    }
    
    start_frame_index = frame_index = frame_index & frame_list_mask;
    
    looped_around = frame_index < current_frame_index;
    /*
     * If looped_around is TRUE we looped around by adding the safe
     * offset and interval offset
     */
    list_for_each_entry(itd, itd_list, chain_list) {
      
      if(looped_around) {
	if( (frame_index >= start_frame_index) && (frame_index < current_frame_index) ) {
	  insert_itd_into_frame_list(frame_list, frame_index, itd);
	}
	else {
	  list_add_tail(&iso_urb_priv->uninserted_itds_list, &itd->uninserted_list);
	}
      }
      else {
	if( (frame_index >= start_frame_index || frame_index < current_frame_index) ) {
	  insert_itd_into_frame_list(frame_list, frame_index, itd);
	}
	else {
	  list_add_tail(&iso_urb_priv->uninserted_itds_list, &itd->uninserted_list);
	}
      }
      
      itd->frame_index = frame_index;
      frame_index = (frame_interval + frame_index) & frame_list_mask;
    }

    spinlock_lock(&ehci_hcd->uninserted_itd_lock);
    list_add_tail(&ehci_hcd->uninserted_itd_urb_list,
                  &iso_urb_priv->uninserted_itd_urb_list);
    spinlock_unlock(&ehci_hcd->uninserted_itd_lock);
  }
  else {
    /*
     * -- EM -- This method does not take into account remaining bytes
     * per microframe and interval, need to fix it
     */
    DLOG("Non realtime isochronous is broken");
    panic("Non realtime isochronous is broken");
    //frame_index = EHCI_GET_SAFE_FRAME_INDEX(ehci_hcd);
    list_for_each_entry(itd, itd_list, chain_list) {
      frame_index = frame_index & frame_list_mask;
      insert_itd_into_frame_list(frame_list, frame_index, itd);
      ++frame_index;
    }
  }
  return 0;
}


ehci_completion_element_t iso_completion_element;

static int
ehci_isochronous_transfer(ehci_hcd_t* ehci_hcd, struct urb* urb)
{
  unsigned int pipe = urb->pipe;
  list_head_t itd_list;
  itd_t* itd;
  itd_t* temp_itd;
  bool done = FALSE;
  int j, i;
  int packets_traversed;
  int num_packets = urb->number_of_packets;
  usb_iso_packet_descriptor_t* packets = urb->iso_frame_desc;
  uint32_t timeout_in_usec;
  ehci_iso_urb_priv_t* iso_urb_priv;

  if(urb->hcpriv == NULL) {
    pow2_alloc(sizeof(ehci_iso_urb_priv_t), (uint8_t**)&urb->hcpriv);
    initialise_iso_urb_priv(urb->hcpriv);
  }

  iso_urb_priv = urb->hcpriv;
  
  if(urb->real_time) {
    
    if((iso_urb_priv->interval_offset = ehci_is_rt_schedulable(ehci_hcd, urb)) < 0) {
      DLOG("urb is not real time schedulable");
      return -1;
    }
  }
  
  if(create_itd_chain(ehci_hcd, usb_pipedevice(pipe),
                      usb_pipeendpoint(pipe), urb->transfer_buffer,
                      packets,
                      num_packets,
                      usb_maxpacket(urb->dev, urb->pipe),
                      usb_pipein(pipe),
                      &itd_list, mp_enabled) < 0) {
    DLOG("create_itd_chain failed");
    panic("create_itd_chain failed");
    return -1;
  }

  if(submit_itd_chain(ehci_hcd, &itd_list, urb) < 0) {
    DLOG("submit_itd_chain failed");
    panic("submit_itd_chain failed");
    return -1;
  }

  if(mp_enabled) {

    /*
     * -- EM -- Not putting it in list just some dummy element right now
     */

    iso_completion_element.urb = urb;
    INIT_LIST_HEAD(&iso_completion_element.tds);
    list_splice(&itd_list, &iso_completion_element.tds);
    iso_completion_element.pipe_type = PIPE_ISOCHRONOUS;
    spinlock_lock(&ehci_hcd->completion_lock);
    list_add_tail(&iso_completion_element.chain_list, &ehci_hcd->completion_list);
    spinlock_unlock(&ehci_hcd->completion_lock);

    return 0;
  }
  else {
    timeout_in_usec = urb->timeout * USB_JIFFIES_TO_USEC;
    itd = list_entry(itd_list.prev, itd_t, chain_list);
  
    j = 0;
    while(!done) {
      for(i = 0; i < 8; ++i) {
        if(itd->transaction[i].raw & ITD_ACTIVE) {
          break;
        }
      }
      done = i == 8;
      ++j;
      tsc_delay_usec(1);
      //DLOG("in spin wait");
      if(j == timeout_in_usec) {
        DLOG("isochronous transaction timed out");
        list_for_each_entry(itd, &itd_list, chain_list) {
          print_itd_info(ehci_hcd, itd, "In Spin");
        }
        print_caps_and_regs_info(ehci_hcd, "In Spin");
        
        DLOGV("pci status in spin = 0x%X",
              pci_get_status(ehci_hcd->bus, ehci_hcd->dev,ehci_hcd->func));
        DLOGV("pci command in spin = 0x%X",
              pci_get_command(ehci_hcd->bus, ehci_hcd->dev,ehci_hcd->func));
        
        list_for_each_entry(itd, &itd_list, chain_list) {
          itd_remove_from_periodic(itd);
        }
        
        list_for_each_entry_safe(itd, temp_itd, &itd_list, chain_list) {
          free_itd(ehci_hcd, itd);
        }
        return 0;
      }
    }

    packets_traversed = 0;
    list_for_each_entry(itd, &itd_list, chain_list) {
      for(i = 0; (i < 8) && (packets_traversed < num_packets);
          ++i, ++packets_traversed) {
      
        packets[packets_traversed].actual_length =
          (itd->transaction[i].raw >> 16) & 0xFFF;
        packets[packets_traversed].status = itd->transaction[i].status;
        if(itd->transaction[i].raw & ITD_ACTIVE) {
          DLOG("itd should be done but is not");
          panic("itd should be done but is not");
        }
      }
    }

    list_for_each_entry(itd, &itd_list, chain_list) {
      itd_remove_from_periodic(itd);
    }
  
    list_for_each_entry_safe(itd, temp_itd, &itd_list, chain_list) {
      free_itd(ehci_hcd, itd);
    }

    if(urb->complete != NULL) {
      urb->complete(urb);
    }
  }
  
  return 0;
}

static inline int calc_total_bytes_cost(struct urb* urb)
{
  int result;
  uint16_t maxpacket = usb_maxpacket(urb->dev, urb->pipe);
  
  switch(usb_pipetype(urb->pipe)) {
  case PIPE_ISOCHRONOUS:

    result = max_packet(maxpacket) + 192;
    if(result >= 128) {
      result += 128;
    }
    
    return result * hb_mult(maxpacket);
    
  case PIPE_INTERRUPT:
  case PIPE_BULK:
  case PIPE_CONTROL:
    DLOG("USB transaction type not handled in calc_total_bytes_cost");
    panic("USB transaction type not handled in calc_total_bytes_cost");
    
  default:
    DLOG("Unknown pipe type passed to calc_total_bytes_cost");
    return -1;
  }
}
static inline void
itd_remove_from_periodic(itd_t* itd)
{
  *(itd->previous) = itd->next_link_pointer;
}

/*
 * Returns a value < 0 if it is not schedulable if it is schedulable
 * it returns the following depending on the type:
 *
 * Isochronous - returns the offset from interval we should start on
 */
static int ehci_is_rt_schedulable(ehci_hcd_t* ehci_hcd, struct urb* urb)
{
  uint32_t  i, j;
  uint32_t  frame_list_size = ehci_hcd->frame_list_size;
  int interval = urb->interval;
  uint32_t  bytes_per_micro_frame;
  uint32_t* micro_frame_remaining_bytes = ehci_hcd->micro_frame_remaining_bytes;
  bool      schedulable;

  bytes_per_micro_frame = calc_total_bytes_cost(urb);
  if(bytes_per_micro_frame < 0) {
    DLOG("calc_total_bytes_cost returned a value < 0");
    return -1;
  }
  switch(usb_pipetype(urb->pipe)) {
    
  case PIPE_ISOCHRONOUS:
    if(ehci_hcd->frame_list_size * 8 / interval != urb->number_of_packets) {
      DLOG("number of packets does not match interval and frame size");
      return -1;
    }
    
    /*
     * -- EM -- Periodic scheduling is bin packing-like.  This is
     * our simple greedy algorithm that we can improve on later
     */
    
    for(i = 0, schedulable = FALSE; i < interval; ++i) {
      schedulable = TRUE;
      for(j = i; j < frame_list_size; j += interval) {
        if(micro_frame_remaining_bytes[j] < bytes_per_micro_frame) {
          schedulable = FALSE;
          break;
        }
      }
      if(schedulable) break;
    }

    if(!schedulable) {
      return -1;
    }

    return i;

    /*
     * -- EM -- unhandled transactions types
     */
  case PIPE_INTERRUPT:
  case PIPE_BULK:
  case PIPE_CONTROL:
    DLOG("USB transaction type not handled in ehci_is_rt_schedulable");
    panic("USB transaction type not handled in ehci_is_rt_schedulable");

  default:
    DLOG("Unknown pipe type passed to ehci_is_rt_schedulable");
    return -1;
  }
}


#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = ehci_init
};

DEF_MODULE (usb___ehci, "EHCI driver", &mod_ops, {"usb", "pci"});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
