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

#include <smp/apic.h>
#include <drivers/usb/usb.h>
#include <drivers/usb/ehci.h>
#include <drivers/usb/ehci_mem.h>
#include <util/printf.h>
#include <mem/virtual.h>
#include <mem/pow2.h>
#include <arch/i386-div64.h>
#include <kernel.h>
#include "sched/sched.h"
#include <util/cassert.h>


#define DEBUG_EHCI
#define DEBUG_EHCI_VERBOSE

#ifdef DEBUG_EHCI
#define DLOG(fmt,...) DLOG_PREFIX("EHCI",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef DEBUG_EHCI_VERBOSE
#define DLOGV(fmt,...) DLOG_PREFIX("EHCI",fmt,##__VA_ARGS__)
#else
#define DLOGV(fmt,...) ;
#endif

#ifdef DEBUG_EHCI_VERBOSE
#define EHCI_DEBUG_HALT()                                               \
  do { DLOGV("HALTING file %s line %d",                                 \
             __FILE__, __LINE__); while(1); } while(0)
#else
#define EHCI_DEBUG_HALT() ;
#endif

#ifdef DEBUG_EHCI_VERBOSE
#define EHCI_ASSERT(test)                                               \
  do {                                                                  \
    if(!(test)) {                                                       \
      DLOG("assert in file " __FILE__ " line %d failed", __LINE__);     \
      panic("assert failed");                                           \
    }                                                                   \
  }                                                                     \
  while(0)
#endif
  


/*
 * Used for debug purposes, prints state of EHCI registers through
 * DLOGV
 */
static void SQUELCH_UNUSED
print_caps_and_regs_info(ehci_hcd_t* ehci_hcd, char* msg)
{
#ifdef DEBUG_EHCI_VERBOSE
  sint32 num_ports, i;

#define PRINT_CAP_INFO(REGISTER)                        \
  DLOGV(#REGISTER ": 0x%X", ehci_hcd->caps->REGISTER);

#define PRINT_REG_INFO(REGISTER)                        \
  DLOGV(#REGISTER ": 0x%X", ehci_hcd->regs->REGISTER);

  DLOGV("\n\n\n\n\nCompiled at %s - %s", __TIME__, msg);
  
  DLOGV("Caps info");
  DLOGV("caps virtual address: 0x%X", ehci_hcd->caps);
  PRINT_CAP_INFO(cap_length);
  PRINT_CAP_INFO(hci_version);
  PRINT_CAP_INFO(hcs_params);
  PRINT_CAP_INFO(hcc_params);

  DLOGV("Regs info");
  DLOGV("reg virtual address: 0x%X", ehci_hcd->regs);
  PRINT_REG_INFO(command);
  PRINT_REG_INFO(status);
  PRINT_REG_INFO(interrupt_enable);
  PRINT_REG_INFO(frame_index);
  PRINT_REG_INFO(segment);
  PRINT_REG_INFO(frame_list);
  PRINT_REG_INFO(async_next);
  PRINT_REG_INFO(configured_flag);

  num_ports = GET_NUM_PORTS(ehci_hcd);

  for(i = 0; i < num_ports; ++i) {
    DLOGV("Port %d status: 0x%X", i, ehci_hcd->regs->port_status[i]);
  }
  
  DLOGV("\n\n\n\n");
  
#endif
}

static void SQUELCH_UNUSED
print_qtd_info(qtd_t* qtd, char* msg)
{
#ifdef DEBUG_EHCI_VERBOSE

#define PRINT_QTD_MEMBER(member) DLOGV(#member ": 0x%X", qtd->member)

  DLOGV("%s", msg);
  
  PRINT_QTD_MEMBER(next_pointer_raw);
  PRINT_QTD_MEMBER(alt_pointer_raw);
  PRINT_QTD_MEMBER(token);
  PRINT_QTD_MEMBER(buffer_page[0]);
  PRINT_QTD_MEMBER(buffer_page[1]);
  PRINT_QTD_MEMBER(buffer_page[2]);
  PRINT_QTD_MEMBER(buffer_page[3]);
  PRINT_QTD_MEMBER(buffer_page[4]);
  
#endif  
}

/*
 * Used for debug purposes, prints state of qh
 */

static void SQUELCH_UNUSED
print_qh_info(ehci_hcd_t* ehci_hcd, qh_t* qh, bool print_tds ,char* msg)
{
#ifdef DEBUG_EHCI_VERBOSE
#define PRINT_QH_MEMBER(member) DLOGV(#member ": 0x%X", qh->member)

  int qtd_count = 0;
  
  DLOGV("**************************************************************************");
  DLOGV("%s", msg);

  PRINT_QH_MEMBER(horizontalPointer.raw);
  PRINT_QH_MEMBER(hw_info1);
  PRINT_QH_MEMBER(hw_info2);
  PRINT_QH_MEMBER(current_qtd_ptr_raw);
  PRINT_QH_MEMBER(next_qtd_ptr_raw);
  PRINT_QH_MEMBER(alt_qtd_ptr_raw);
  PRINT_QH_MEMBER(qtd_token_raw);
  PRINT_QH_MEMBER(raw5);
  PRINT_QH_MEMBER(raw6);
  PRINT_QH_MEMBER(raw7);
  PRINT_QH_MEMBER(raw8);
  PRINT_QH_MEMBER(raw9);
  DLOG("Software only members:");
  PRINT_QH_MEMBER(state);


  if(print_tds) {
    qtd_t* qtd;
    qtd_t* alt_qtd = NULL;
    if(qh->current_qtd_ptr_raw > 32) {
      qtd = EHCI_QTD_PHYS_TO_VIRT(ehci_hcd, qh->current_qtd_ptr_raw);
      print_qtd_info(qtd, "Current QH QTD");
    }
    qtd = NULL;
    if(qh->next_qtd_ptr_raw > 32) {
      qtd = EHCI_QTD_PHYS_TO_VIRT(ehci_hcd, qh->next_qtd_ptr_raw);
    }
    if(qh->alt_qtd_ptr_raw > 32) {
      
      alt_qtd = EHCI_QTD_PHYS_TO_VIRT(ehci_hcd, qh->alt_qtd_ptr_raw);
    }
    while(qtd != NULL) {
      DLOGV("QTD %d info", qtd_count);
      print_qtd_info(qtd, "");
      if(alt_qtd != NULL) {
        DLOGV("Alt qtd %d info:", qtd_count);
          print_qtd_info(alt_qtd, "");
      }
      else {
        DLOGV("Alt qtd %d is NULL", qtd_count);
      }
      qtd_count++;
      alt_qtd = NULL;
      if(qtd->next_pointer_raw > 32) {
        qtd = EHCI_QTD_PHYS_TO_VIRT(ehci_hcd, qtd->next_pointer_raw);
        
        if(qtd->alt_pointer_raw > 32) {
          alt_qtd = EHCI_QTD_PHYS_TO_VIRT(ehci_hcd, qtd->alt_pointer_raw);
        }
      }
      else {
        qtd = NULL;
      }
    }
  }
  
  
#endif  
}




#define DEFAULT_FRM_LST_SIZE 1024 /*
                                   * Default frame list size specified
                                   * by EHCI Specs
                                   */

#define FRM_LIST_SIZE DEFAULT_FRM_LST_SIZE        /* Frame list size used in Quest */
#define QH_POOL_SIZE 256
#define DEFAULT_INT_THRESHOLD 4
#define QTD_POOL_SIZE 256


static ehci_hcd_t ehci_hcd;
static frm_lst_lnk_ptr_t frame_list[FRM_LIST_SIZE] ALIGNED(0x1000);
static qh_t queue_head_pool[QH_POOL_SIZE] ALIGNED(0x1000);
static uint32_t used_queue_head_bitmap[(QH_POOL_SIZE + 31)/ 32];
static qtd_t qtd_pool[QTD_POOL_SIZE] ALIGNED(0x1000);
static uint32_t used_qtd_bitmap[(QH_POOL_SIZE + EHCI_ELEMENTS_PER_BITMAP_ENTRY - 1) / EHCI_ELEMENTS_PER_BITMAP_ENTRY];

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
  DLOG("%s: %d", __FUNCTION__, vec);
  panic("IN UNIMPLEMENTED IRQ HANDLER");
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
  uint32_t frm_lst_size = ehci_hcd->frame_list_size;
  frm_lst_lnk_ptr_t* frame_list = ehci_hcd->frame_list;
  
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
  }
  
  ehci_hcd->regs->frame_list = (uint32_t)ehci_hcd->frame_list;
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
    
  ehci_hcd->reclaim_list = NULL;

  gccmb();
  EHCI_ENABLE_ASYNC(ehci_hcd);
  
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
   * --??-- I don't know if it is necessary to check if a port
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
  sched_usleep(50 * 1000);

  PORT_LEAVE_RESET(*port);

  /*
   * Software must wait 2 ms for port to end reset, EHCI Specs page 28
   */
  if(!handshake(port, PORT_RESET, 0, 2 * 1000)) return FALSE;

  DLOG("Port after leave reset 0x%X", *port);

  return TRUE;
}

bool
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
    }
  }

  return TRUE;
}

/*
 * Host controller initiailization sequence taken from page 53 of the
 * EHCI Specs
 */

static bool
initialise_ehci_hcd(uint32_t usb_base,
                    ehci_hcd_t* ehci_hcd,
                    frm_lst_lnk_ptr_t* frm_lst,
                    uint32_t frm_lst_size,
                    qh_t* queue_head_pool,
                    uint32_t queue_head_pool_size,
                    uint32_t* used_queue_head_bitmap,
                    qtd_t* qtd_pool,
                    uint32_t qtd_pool_size,
                    uint32_t* used_qtd_bitmap,
                    uint32_t int_threshold)
{ 
  initialise_usb_hcd(&ehci_hcd_to_hcd(ehci_hcd),
                     USB_TYPE_HC_EHCI, ehci_reset_root_ports);
  
  ehci_hcd->base_physical_address = (phys_addr_t)usb_base;
  ehci_hcd->base_virtual_address = map_virtual_page(usb_base | 0x3);
  ehci_hcd->caps = ehci_hcd->base_virtual_address;
  ehci_hcd->regs = (ehci_regs_t*)(ehci_hcd->base_virtual_address
                                  + ehci_hcd->caps->cap_length);

  ehci_hcd->frame_list      = frm_lst;
  ehci_hcd->frame_list_size = frm_lst_size;

  ehci_hcd->queue_head_pool           = queue_head_pool;
  ehci_hcd->queue_head_pool_phys_addr = (phys_addr_t)get_phys_addr(queue_head_pool);
  ehci_hcd->queue_head_pool_size      = queue_head_pool_size;
  ehci_hcd->used_queue_head_bitmap    = used_queue_head_bitmap;
  memset(queue_head_pool, 0, queue_head_pool_size * sizeof(qh_t));

  ehci_hcd->qtd_pool           = qtd_pool;
  ehci_hcd->qtd_pool_phys_addr = (phys_addr_t)get_phys_addr(qtd_pool);
  ehci_hcd->qtd_pool_size      = qtd_pool_size;
  ehci_hcd->used_qtd_bitmap    = used_qtd_bitmap;
  memset(qtd_pool, 0, qtd_pool_size * sizeof(qtd_t));

  
  ehci_hcd->interrupt_threshold = int_threshold;
  ehci_hcd->num_ports           = GET_NUM_PORTS(ehci_hcd);

  /*
   * Set used_queue_head_bitmap and used_qtd_bitmap to all zero to
   * mark all elements as free
   */

  ehci_hcd->used_queue_head_bitmap_size =
    calc_used_queue_head_bitmap_size(ehci_hcd);

  memset(used_queue_head_bitmap, 0, ehci_hcd->used_queue_head_bitmap_size * sizeof(*used_queue_head_bitmap));
  
  ehci_hcd->used_qtd_bitmap_size =
    calc_used_qtd_bitmap_size(ehci_hcd);

  memset(used_qtd_bitmap, 0, ehci_hcd->used_qtd_bitmap_size * sizeof(*used_qtd_bitmap));
  
  if(!restart_ehci_hcd(ehci_hcd)) return FALSE;
  if(!initialise_frame_list(ehci_hcd)) return FALSE;
  if(!initialise_async_head(ehci_hcd)) return FALSE;
  
  // Set desired interrupt threshold
  if(!set_interrupt_threshold(ehci_hcd)) return FALSE;

  /* --??-- write memory barrier might be needed here */
  
  // Turn EHCI chip on
  if(!set_ehci_on(ehci_hcd)) return FALSE;

  /* --??-- write memory barrier might be needed here */

  // Set all ports to route to EHCI chip
  EHCI_SET_CONFIG_FLAG(ehci_hcd, 1);

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

  DLOGV("***************************************************");
  DLOGV("Entering %s compiled at %s", __FUNCTION__, __TIMESTAMP__);

  if(mp_ISA_PC) {
    DLOG("Cannot operate without PCI");
    DLOGV("Exiting %s with FALSE", __FUNCTION__);
    return FALSE;
  }

  /* Find the EHCI device on the PCI bus */
  device_index = ~0;
  i = 0;

  /*
   * --WARN-- Only looking for 1 specific EHCI host controller device
   * this is D29 for Intel 6 C200 or qemu ehci chip
   */
  
  //while (pci_find_device (0x8086, 0x1C26, 0x0C, 0x03, i, &i)) {
  while (pci_find_device (0x8086, 0x24CD, 0x0C, 0x03, i, &i)) { 
    if (pci_get_device (i, &ehci_device)) { 
      if (ehci_device.progIF == 0x20) {
        device_index = i;
        break;
      }
      i++;
    } else break;
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
    if (!pci_irq_map_handler (&irq, ehci_irq_handler, 0x01,
                              IOAPIC_DESTINATION_LOGICAL,
                              IOAPIC_DELIVERY_FIXED)) {
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

  DLOG("usb base from BAR0= 0x%.04X", usb_base);

  
  if(!initialise_ehci_hcd(usb_base, &ehci_hcd, frame_list,
                          sizeof(frame_list)/sizeof(frm_lst_lnk_ptr_t),
                          queue_head_pool,
                          sizeof(queue_head_pool)/sizeof(qh_t),
                          used_queue_head_bitmap,
                          qtd_pool,
                          sizeof(qtd_pool)/sizeof(qtd_t),
                          used_qtd_bitmap,
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
  
  /* Number of bytes buffer_page[0] stores */
  count = 0x1000 - (phys_data_addr & 0x0FFF);
  if(data_len < count) {
    count = data_len;
  }
  else {
    /* Increment to next 4K boundary */
    phys_data_addr += 0x1000;
    phys_data_addr &= 0xF000;

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


static qtd_t*
create_qtd_chain(ehci_hcd_t* ehci_hcd, 
                 uint8_t address,
                 addr_t setup_req,    /* Use virtual address here */
                 uint32_t setup_len,
                 addr_t setup_data,   /* Use virtual address here */
                 uint32_t data_len,
                 uint32_t packet_len,
                 uint32_t pipe_type,
                 uint32_t is_input)
{
  uint32_t     token;
  qtd_t*      first_qtd;
  qtd_t*      current_qtd;
  qtd_t*      previous_qtd;
  phys_addr_t data_phys_addr;
    
  token = QTD_ACTIVE | (EHCI_TUNE_CERR << QTD_CERR);
  
  first_qtd = current_qtd = allocate_qtd(ehci_hcd);
  
  if(!current_qtd) return NULL;

  if(pipe_type == PIPE_CONTROL) {
    if(qtd_fill(current_qtd, (phys_addr_t)get_phys_addr(setup_req),
                setup_len, packet_len, token | QTD_SETUP) != setup_len) {
      goto create_qtd_chain_cleanup;
    }
    
    token ^= QTD_TOGGLE;

    previous_qtd = current_qtd;
    current_qtd = allocate_qtd(ehci_hcd);
    if(!current_qtd) goto create_qtd_chain_cleanup;
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

  data_phys_addr = (phys_addr_t)get_phys_addr(setup_data);
  
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
      send_one_more = TRUE;
    }

    if(send_one_more) {
      previous_qtd = current_qtd;
      current_qtd = allocate_qtd(ehci_hcd);
      if(!current_qtd) goto create_qtd_chain_cleanup;
      
      previous_qtd->next_pointer_raw
        = EHCI_QTD_VIRT_TO_PHYS(ehci_hcd, current_qtd);

      qtd_fill(current_qtd, 0, 0, 0, token);
    }
  }

  /* Interrupt on complete */
  QTD_ENABLE_IOC(current_qtd);
      
  return first_qtd;
  
 create_qtd_chain_cleanup:
  /* --!!-- Unimplemented */
  
  panic("Unimplemented cleanup in create_qtd_chain!!!!!!!");
  return NULL;
}

static void qh_prep(qh_t* qh, uint32_t endpoint, uint32_t device_addr,
                    uint32_t max_packet_len, uint32_t dev_speed,
                    uint32_t pipe_type, bool is_input)
{
  uint32_t info1 = 0;
  uint32_t info2 = 0;

  info1  = device_addr;
  info1 |= endpoint;

  if(pipe_type == PIPE_INTERRUPT) {
   
    /*
     * --!!-- Unimplemented, interrupt pipes have extra work should be
     * done will get to this later, see Linux ehci-q.c function
     * qh_make
     */

    DLOG("Unimplemented case in file %s line %d", __FILE__, __LINE__);
    panic("Unimplemented case");
  }

  switch(dev_speed) {
  case USB_SPEED_LOW:
  case USB_SPEED_FULL:
    /*
     * --!! Unimplemented, will add logic for low and full speed
     * devices later, see Linux ehci-q.c function qh_make
     */
    DLOG("Unimplemented case in file %s line %d", __FILE__, __LINE__);
    panic("Unimplemented case");

    break;

  case USB_SPEED_HIGH:
    info1 |= (2 << 12); /* Set end point speed to high */
    if(pipe_type == PIPE_CONTROL) {
      info1 |= (EHCI_TUNE_RL_HS << 28); /* Set nak throttle */
      
      info1 |= (64 << 16); /* Setting max packet length which for usb
                              2 control pipes must be 64 */
      
      info1 |= (1 << 14); /* Set queue head to get initial data toggle
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
     * --!!-- Should fail more gracefully here but don't care right
     * now
     */
    DLOG("Unknown device speed, see file %s line %d", __FILE__, __LINE__);
    panic("Unknown device speed");
  }

  qh->hw_info1 = info1;
  qh->hw_info2 = info2;
}

/*
 * --WARN-- This function only works if the QH is inactive 
 */
void qh_append_qtds(ehci_hcd_t* ehci_hcd, qh_t* qh, qtd_t* start_qtd)
{
  if(qh->state != QH_STATE_NOT_LINKED) {
    DLOG("Should never pass an active qh to qh_append_qtds see file %s line %d", __FILE__, __LINE__);
    panic("Passed an active qh to qh_append_qtds");
  }

  qh->next_qtd_ptr_raw = EHCI_QTD_VIRT_TO_PHYS(ehci_hcd, start_qtd);
  qh->alt_qtd_ptr_raw  = EHCI_LIST_END;
}

void link_qh_to_async(ehci_hcd_t* ehci_hcd, qh_t* qh)
{
  qh->state = QH_STATE_LINKED;
  
  print_qh_info(ehci_hcd, qh, TRUE, "******************Before Link***************************");
  qh->horizontalPointer = ehci_hcd->async_head->horizontalPointer;
  gccmb();
  ehci_hcd->async_head->horizontalPointer.raw = QH_NEXT(ehci_hcd, qh);

  tsc_delay_usec(10000000);
  print_qh_info(ehci_hcd, qh, TRUE, "********************************After Link***************************");

  while(1);
}

static int submit_async_qtd_chain(ehci_hcd_t* ehci_hcd, qh_t** qh, qtd_t* start_qtd,
                                  uint32_t endpoint, uint32_t device_addr,
                                  uint32_t dev_speed, uint32_t pipe_type,
                                  uint32_t max_packet_len, bool is_input)
{
  if(*qh == NULL) {
    *qh = allocate_qh(ehci_hcd);
    qh_prep(*qh, endpoint, device_addr, max_packet_len, dev_speed, pipe_type, is_input);
  }
  else {
    /*
     * --!!-- Right now the logic for adding qtds to an active queue
     * head is not implemented (see worklog note for 2/8/12, so a new
     * qh must be created each time
     */
    DLOG("Tried to reuse a qh. See file %s line %d", __FILE__, __LINE__);
    panic("tried to resue a qh");
  }
  
  qh_append_qtds(ehci_hcd, *qh, start_qtd);

  link_qh_to_async(ehci_hcd, *qh);

  EHCI_DEBUG_HALT();
  return 0;
}

int
ehci_control_transfer(ehci_hcd_t* ehci_hcd, 
                      uint8_t address,
                      addr_t setup_req,    /* Use virtual address here */
                      uint32_t setup_len,
                      addr_t setup_data,   /* Use virtual address here */
                      uint32_t data_len,
                      uint32_t packet_len)
{
  qh_t* qh = NULL;
  qtd_t* start_qtd = create_qtd_chain(ehci_hcd, address, setup_req, setup_len,
                                      setup_data, data_len, packet_len,
                                      PIPE_CONTROL, IS_INPUT_USB_DEV_REQ(setup_req));
  if(start_qtd == NULL) return -1;

  /*
   * --WARN-- Right now I am hardcoding the speed to USB_SPEED_HIGH.
   * Also our USB core assumes control transfers are to endpoint 0
   * (which is a fair assumption) so the endpoint is being hardcoded
   * in as well
   */

  submit_async_qtd_chain(ehci_hcd, &qh, start_qtd, 0, address,
                         USB_SPEED_HIGH, PIPE_CONTROL, packet_len,
                         IS_INPUT_USB_DEV_REQ(setup_req));
  
  return -1;
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
