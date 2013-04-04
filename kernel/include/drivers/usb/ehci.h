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


#ifndef _EHCI_H_
#define _EHCI_H_


#include <types.h>
#include <arch/i386.h>
#include <drivers/pci/pci.h>
#include <drivers/usb/usb.h>
#include <util/list.h>
#include <util/cassert.h>
#include "smp/spinlock.h"



#define hcd_to_ehci_hcd(hcd) container_of((hcd), ehci_hcd_t, usb_hcd)
#define ehci_hcd_to_hcd(ehci_hcd) (&((ehci_hcd)->usb_hcd))




/*
 * -- EM -- So here is the deal with EHCI irq handling.  For most
 * interrupts the chip is suppose to set the interrupt's bit in the
 * status register and then when the time passed for the interrupt
 * threshold has passed if any bits that are turned on in the
 * interrupt enable register have their corresponding status bits set
 * then an interrupt is generated.  For the interrupts that it does
 * not do this it does not wait for the interrupt threshold, it just
 * sets the status bit and generates the interrupt if the
 * corresponding interrupt enable bit is set.  However some EHCI
 * controllers do not seem to set the status bit but just generate an
 * interrupt for interrupt on complete/short packet.  The same code
 * has caused the status bit 0 to be set for some EHCI chips and not
 * set for others.  This could be because some EHCI implementations
 * are more lenient then others and there is something wrong with the
 * code but I have spent two days working on this "bug" and Linux has
 * a watchdog timer that goes off and causes it to do the work that
 * would normally occur for an interrupt on complete/short packet so I
 * am going to assume that it is a problem with some EHCI chips.  What
 * this means is that every time an interrupt occurs just assume that
 * it was caused by an interrupt on complete/short packet.  Since most
 * of the time the interrupt handler will only be called for interrupt
 * on complete/short packets this assumption should not affect
 * performance by much.  Also the same chips that do not set the
 * status bit for completion/short packet also do not reliably
 * generate an interrupt to fix this I am create a kernel thread that
 * performs similar duties to the interrupt handler.  This kernel
 * thread will sleep for INTERRUPT_THRESHOLD * 150.  It would be
 * better to do what Linux does and use a watchdog timer that is set
 * to go off a little bit after the transaction should finish so it
 * never goes off if the chip does not have the bug and will go off
 * shortly after the irq would go off if necessary.  We don't have
 * watchdog timers yet so a periodic tick will have to do.  To disable
 * all the extra software for the hardware bug comment out the macro
 * declaration below
 */

//#define EHCI_IOC_BUG

#define EHCI_USE_IOVCPU



/*
 * These constants affect system performance, would be worth playing
 * around with these numbers are some point to see how much they
 * affect overall performance
 */


/* Number of qtd retries Possible values: 0-3   0 -> don't stop */
#define EHCI_TUNE_CERR 3

/* nak throttle, see section 4.9 of EHCI specs */
#define EHCI_TUNE_RL_HS 4

/* 1-3 transactions per microframe see section 4.10.3 of EHCI specs */
#define EHCI_TUNE_MULT_HS 1

#define EHCI_TUNE_MULT_TT 1

typedef uint32_t ehci_port_t;

#define TYPE_ITD  0
#define TYPE_QH   1
#define TYPE_SITD 2
#define TYPE_FSTN 3

typedef struct
{
  uint8_t  cap_length;
  uint8_t  reserved;
  uint16_t hci_version;
  uint32_t hcs_params;

#define N_PORTS (0xF<<0)

#define GET_NUM_PORTS(hcd) ((hcd)->caps->hcs_params & N_PORTS)
  
  uint32_t hcc_params;

#define PROG_FRM_LST_LENGTH (1<<1)
  
#define EHCI_IS_FRM_LST_LEN_PROG(hcd)           \
  ((hcd)->caps->hcc_params & PROG_FRM_LST_LENGTH)
  
  uint8_t port_route[8];
} ehci_caps_t PACKED;

typedef struct
{
  uint32_t command;

#define RUN (1<<0)
#define RESET (1<<1)
#define PERIODIC_SCHED_ENABLE (1<<4)
#define ASYNC_SCHED_ENABLE (1<<5)
#define INT_ASYNC_ADV_DOORBELL (1<<6)
#define INT_THRESHOLD (0xFF<<16)


#define EHCI_ENABLE_ASYNC(hcd) ((hcd)->regs->command |= ASYNC_SCHED_ENABLE)
#define EHCI_ENABLE_PERIODIC(hcd) ((hcd)->regs->command |= PERIODIC_SCHED_ENABLE)
#define EHCI_IS_RUNNING(hcd) ((hcd)->regs->command & RUN)
#define EHCI_RUN(hcd) (hcd)->regs->command |= RUN
#define EHCI_HALT(hcd) (hcd)->regs->command &= ~RUN
#define EHCI_RESET(hcd) (hcd)->regs->command |= RESET
#define EHCI_ASYNC_DOORBELL_ENABLED(hcd)                \
  ((hcd)->regs->command | INT_ASYNC_ADV_DOORBELL)
#define EHCI_ENABLE_ASYNC_DOORBELL(hcd)                 \
  ((hcd)->regs->command |= INT_ASYNC_ADV_DOORBELL)
#define EHCI_DISABLE_ASYNC_DOORBELL(hcd)                \
  ((hcd)->regs->command &= ~INT_ASYNC_ADV_DOORBELL)
  
#define EHCI_SET_FRAME_LIST_SIZE(hcd, size)                       \
  do {                                                            \
    uint32_t __command_temp = (hcd)->regs->command;               \
    __command_temp &= ~(3<<2);                                    \
    (hcd)->regs->command = __command_temp | ((size)<<2);          \
  } while(0)
  
#define EHCI_SET_INT_THRESHOLD(hcd, threshold)                    \
  do {                                                            \
    uint32_t __command_temp = (hcd)->regs->command;               \
    __command_temp &= ~INT_THRESHOLD;                             \
    (hcd)->regs->command = __command_temp | ((threshold) << 16);  \
  } while(0)
  
  uint32_t status;
  
#define HALTED (1<<12)
#define EHCI_IS_HALTED(hcd) ((hcd)->regs->status & HALTED)
  
  
  
  uint32_t interrupt_enable;

#define USBINTR_IAA  (1<<5)          /* Interrupted on async advance       */
#define USBINTR_HSE  (1<<4)          /* such as some PCI access errors     */
#define USBINTR_FLR  (1<<3)          /* frame list rolled over             */
#define USBINTR_PCD  (1<<2)          /* port change detect                 */
#define USBINTR_ERR  (1<<1)          /* "error" completion (overflow, ...) */
#define USBINTR_INT  (1<<0)          /* "normal" completion (short, ...)   */
#define USBINTR_MASK                                                    \
  (USBINTR_IAA | USBINTR_HSE | USBINTR_PCD | USBINTR_ERR | USBINTR_INT )

#define USBINTR_ALL \
  (USBINTR_IAA | USBINTR_HSE | USBINTR_PCD | USBINTR_ERR | USBINTR_INT | \
   USBINTR_FLR )
  
#define EHCI_ENABLE_INTR(hcd, intr) ((hcd)->regs->interrupt_enable |= (intr))
#define EHCI_SET_INTRS(hcd) EHCI_ENABLE_INTR((hcd), USBINTR_MASK)
#define EHCI_DISABLE_INTR(hcd, intr) ((hcd)->regs->interrupt_enable &= ~(intr))
#define EHCI_INTR_ENABLED(hcd, intr) ((hcd)->regs->interrupt_enable & (intr))
#define EHCI_ASYNC_DOORBELL_RUNG(hcd) ((hcd)->regs->status & USBINTR_IAA)
#define EHCI_ACK_INTERRUPTS(hcd, intrs) ((hcd)->regs->status |= (intrs))
#define EHCI_ACK_DOORBELL(hcd) ((hcd)->regs->status |= USBINTR_IAA)
  
  uint32_t frame_index;

  
  /*
   * Should only call this macro right before adding itds to periodic
   * list
   */
#define EHCI_CURRENT_FRAME_INDEX(hcd)                                   \
  ( (((hcd)->regs->frame_index) >> 3) & ((hcd)->frame_list_size - 1) )


  /*
   * -- EM -- Right now being lazy and making an extremely safe (10
   * frames) estimate of the safest frame index offset.  Fix this
   * later to appropriately check how much the EHCI chip caches,
   * because of how reordering works this value now also depends on
   * the max isochronous device in the periodic list.  This is because
   * this value is also used for reordering and we can't reorder an
   * itd to be behind the frame entry register's current value.  The max
   * interval should be stored (which it isn't right now) and then
   * used in the calculations too.
   */
#define EHCI_SAFE_FRAME_INSERT_OFFSET(hcd) (3)
  
  /*
   * Always best to make this a little larger than the actual max look
   * ahead
   */
#define MAX_SAFE_FRAME_INDEX_LOOK_AHEAD (7)

  uint32_t segment;
  volatile uint32_t frame_list;
  uint32_t async_next;

  
  uint32_t reserved[9];
  uint32_t configured_flag;

#define  EHCI_SET_CONFIG_FLAG(hcd, flag)        \
  (hcd)->regs->configured_flag = 0x1 & flag
  
  ehci_port_t port_status[0];

#define CURR_CONN_STATUS (1<<0)
#define CONN_STATUS_CHANGE (1<<1)
#define PORT_ENABLED (1<<2)
#define PORT_ENABLE_CHANGE (1<<3)
#define OVER_CURRENT_ACTIVE (1<<4)
#define OVER_CURRENT_CHANGE (1<<5)
#define FORCE_PORT_RESUME (1<<6)
#define SUSPEND (1<<7)
#define PORT_RESET (1<<8)
#define LINE_STATUS (3<<10)
#define PORT_POWER (1<<12)
#define PORT_OWNER (1<<13)
#define PORT_INDIC_CONTROL (3<<14)
#define PORT_TEST (0xF<<16)
#define WAKE_ON_CON_ENABLED (1<<20)
#define WAKE_ON_DISCON_ENABLED (1<<21)
#define WAKE_ON_OVER_CURR_ENABLED (1<<22)

#define IS_DEVICE_PRESENT(port) (port & CURR_CONN_STATUS)
#define IS_PORT_POWERED(port) (port & PORT_POWER)
#define POWER_PORT(port) port |= PORT_POWER
#define IS_PORT_ENABLED(port) (port & PORT_ENABLED)
#define IS_LOW_SPEED_DEVICE_PRESENT(port) ((port & LINE_STATUS) == (1<<10))
#define RESET_PORT(port)                        \
  do {                                          \
    ehci_port_t __port = port;                  \
    __port |= PORT_RESET;                       \
    __port &= ~PORT_ENABLED;                    \
    port = __port;                              \
  } while(0)
  
#define PORT_LEAVE_RESET(port) port &= ~PORT_RESET
  
} PACKED ehci_regs_t;



typedef union {
  uint32_t raw;
  struct
  {
    uint32_t tBit:1;
    uint32_t type:2;
    uint32_t reserved:2;
    uint32_t pointer:27;
  } PACKED;
} PACKED frm_lst_lnk_ptr_t;


#define EHCI_LIST_END 1 


typedef union {
  uint32_t raw;
  struct {
    uint32_t buffer_point:20;
    uint32_t current_offset:12;  /*
                                  * Also reserved in qtd buffer page
                                  * pointer that do not use current
                                  * offset
                                  */
  } PACKED;
} qtd_buffer_page_pointer_t;

typedef struct _qtd_t
{
  // dword 1, next qTD
  union {
    uint32_t next_pointer_raw;
    struct {
      uint32_t terminate:1;
      uint32_t reserved1:4;
      uint32_t next_qtd_pointer:27;
    } PACKED;
  };


  // dword 2, alternative next qTD
  union {
    uint32_t alt_pointer_raw; 
    struct {
      uint32_t alternative_terminate:1;
      uint32_t reserved2:4;
      uint32_t alternate_next_qtd_pointer:27;
    } PACKED;
  };

  
  // dword 3, qTD token
  union {
    uint32_t token;
    struct {
      uint32_t status:8;

#define QTD_ACTIVE (1 << 7)
#define QTD_HALT (1 << 6)
#define QTD_STS_PING (1 << 0)
      
      uint32_t pid_code:2;

#define QTD_SETUP (2 << 8)
#define QTD_INPUT (1 << 8)
#define QTD_IS_OUT(qtd) (!((qtd)->token & 0x300))
#define QTD_IS_INPUT(qtd) ((qtd)->token & 0x100)
      
      uint32_t error_counter:2;

#define QTD_CERR 10
      
      uint32_t curr_page:3;
      uint32_t ioc:1;

#define QTD_IOC (1 << 15)

#define QTD_ENABLE_IOC(qtd) do { (qtd->token |= QTD_IOC); (qtd->token_backup |= QTD_IOC); } while(0)
      
      uint32_t total_bytes_to_transfer:15;
      uint32_t data_toggle:1;

#define QTD_TOGGLE (1 << 31)
#define QTD_REMAINING_DATA(qtd)  ((qtd->token >> 16) & 0x7FFF)
    } PACKED;
  };

  // dword 4-8, buffer page 0-4 ,current offset is used only in buffer page 0

  qtd_buffer_page_pointer_t buffer_page[5];
  uint32_t ex_buf_ptr_pgs[5];

  /* Software only members */

  list_head_t chain_list;
  uint32_t original_total_bytes_to_transfer;
  uint32_t token_backup;
  qtd_buffer_page_pointer_t buffer_page_zero_backup;
  bool ioc_called;
  void* completion_context;
  phys_addr_t dma_addr;
  uint32_t padding[6];
} PACKED ALIGNED(32) qtd_t;


CASSERT( (sizeof(qtd_t) % 32) == 0, ehci_qtd_size);

CASSERT( (sizeof(qtd_t) & (sizeof(qtd_t) - 1)) == 0, qtd_alignedment)

#define QH_NEXT(hcd, qh_virt)                                   \
  ( ((qh_virt)->dma_addr) | (TYPE_QH << 1) )

typedef struct _qh_t{
  
  // dword 0
  frm_lst_lnk_ptr_t horizontalPointer;

  // dword 1
  union {
    uint32_t hw_info1;
    struct {
      uint32_t device_address:7;
      uint32_t inactive_on_next:1;
      uint32_t endpoint_num:4;

#define QH_GET_ENDPOINT(qh) (((qh)->hw_info1 >> 8) & 0x0F)
      
      uint32_t endpoint_speed:2;
      uint32_t data_toggle_control:1;
      uint32_t head_reclam:1;
      uint32_t max_packet_length:11;
      uint32_t cnt_endpoint:1;
      uint32_t nak_cnt_reload:4;
    } PACKED;
  };

#define QH_DATA_TOGGLE_CONTROL (1 << 14)
#define QH_HEAD (0x00008000)

  // dword 2
  union {
    uint32_t hw_info2;;
    struct
    {
      uint32_t interrupt_sched_mask:8;
      uint32_t split_comp_mask:8;
      uint32_t hub_addr:7;
      uint32_t port_num:7;
      uint32_t mult:2;
    } PACKED;
  };

  // dword 3
  union {
    uint32_t current_qtd_ptr_raw;
    struct {
      uint32_t reserved0:5;
      uint32_t current_qtd_ptr:27;
    } PACKED;
  };

  // dword 4
  union {
    uint32_t next_qtd_ptr_raw;
    struct {
      uint32_t next_tbit:1;
      uint32_t reserved1:4;
      uint32_t next_qtd_ptr:27;
    } PACKED;
  };

  // dword 5
  union {
    uint32_t alt_qtd_ptr_raw;
    struct {
      uint32_t alt_tbit:1;
      uint32_t nak_cnt:4;
      uint32_t alt_next_qtd_ptr:27;
    } PACKED;
  };

  // dword 6
  union {
    uint32_t qtd_token_raw;
    struct {
      uint32_t status:8;
      uint32_t pid_code:2;
      uint32_t cerr:2;
      uint32_t c_page:3;
      uint32_t ioc:1; /* interrupt on complete */
      uint32_t total_bytes_transfer:15;
      uint32_t data_toggle:1;
    } PACKED;
  };

  // dword 7
  union {
    uint32_t raw5;
    struct {
      uint32_t current_offset:12; 
      uint32_t buff_ptr0:20;
    } PACKED;
  };

  // dword 8
  union {
    uint32_t raw6;
    struct {
      uint32_t c_prog_mask:16;
      uint32_t reserved2:4;
      uint32_t buff_ptr1:20;
    } PACKED;
  };

  // dword 9
  union {
    uint32_t raw7;
    struct {
      uint32_t frame_tag:5;
      uint32_t s_bytes:7;
      uint32_t buff_ptr2:20;
    } PACKED;
  };

  // dword 10
  union {
    uint32_t raw8;
    struct {
      uint32_t reserved3:12; 
      uint32_t buff_ptr3:20;
    } PACKED;
  };

  // dword 11
  union {
    uint32_t raw9;
    struct {
      uint32_t reserved4:12; 
      uint32_t buff_ptr4:20;
    } PACKED;
  };

  uint32_t ex_buf_ptr_pgs[5];

  /*
   * Everything after this is used by software only and is not
   * specified by EHCI
   */

  qtd_t* dummy_qtd;
  uint32_t state;
  list_head_t qtd_list;
  list_head_t reclaim_chain;
  USB_DEVICE_INFO* dev;
  uint interval;
  frm_lst_lnk_ptr_t* next_virt;


#define QH_STATE_NOT_LINKED 1 /* Not in the buffer at all */
#define QH_STATE_LINKED     2 /* In the async circular list */
#define QH_STATE_RECLAIM    3 /* In the reclamation list, which means it
                                 is being removed from the circular
                                 buffer but the HC might still have
                                 links to it */

  phys_addr_t dma_addr;
  frm_lst_lnk_ptr_t* previous;  /* pointer to physical address */
} PACKED ALIGNED(32) qh_t;

CASSERT( (sizeof(qh_t) % 32) == 0, ehci_qh_size);

CASSERT( (sizeof(qh_t) & (sizeof(qh_t) - 1)) == 0, qh_alignedment)

#define EHCI_TRANSACTION_LENGTH

typedef struct {
  union {
    uint32_t raw;
    struct {
      uint32_t offset:12;
      uint32_t page_selector:3;
      uint32_t ioc:1;

#define ITD_IOC (1 << 15)
      
      uint32_t length:12;
      uint32_t status:4;
    } PACKED;
  };
} PACKED itd_transaction_t;

#define ITD_NEXT(hcd, itd_virt)                                 \
  ( ((itd_virt)->dma_addr ) | (TYPE_ITD << 1) )

typedef struct _itd_t {
  frm_lst_lnk_ptr_t next_link_pointer;
  itd_transaction_t transaction[8];

#define ITD_ACTIVE  (1 << 31)
#define ITD_BUF_ERR (1 << 30)
#define ITD_BAB_ERR (1 << 29)
#define ITD_TRN_ERR (1 << 28)

#define IS_ITD_TRANSACTION_ACTIVE(itd, transaction_num)         \
  ((itd)->transaction[transaction_num].raw & ITD_ACTIVE)
  
  uint32_t buf_ptr[7];

#define ITD_INPUT (1 << 11)

#define ITD_SET_BUF_PTR(itd, page_num, phys_addr)               \
  ( (itd)->buf_ptr[(page_num)] |= ( (phys_addr) & ~0x0FFF ) )
  
  uint32_t ex_buf_ptr_pgs[7];

  /* Everything after this is used by software only and is not
     specified by EHCI */
  itd_transaction_t transaction_backup[8];

  list_head_t chain_list;
  uint total_bytes_to_transfer;
  uint frame_index;
  list_head_t uninserted_list;
  //usb_iso_packet_descriptor_t *iso_packet_descs[8];
  
  /* previous is used when the itd is removed */
  frm_lst_lnk_ptr_t* previous;
  frm_lst_lnk_ptr_t* next_virt;

   /* Could be just 8 bits but using unit to keep everything aligned,
      used for rt-urbs to keep track of which transactions have had
      their interrupt handler processed, should be cleared when itd is
      being inserted into periodic list or restarted to receive more
      data */
  uint ioc_processed_bitmap;
  phys_addr_t dma_addr;
  void* completion_context;
  
  uint32_t padding[17];
} PACKED ALIGNED(32) itd_t;

CASSERT( (sizeof(itd_t) % 32) == 0, ehci_itd_size)

CASSERT( (sizeof(itd_t) & (sizeof(itd_t) - 1)) == 0, itd_alignedment)

/*-- EM -- There are better ways to store queue heads for endpoints
 * but for now this is good enough, right now 128 instances of this
 * struct takes 4 pages, running even lower on kernel memory */ 
typedef struct
{
  /* First Index: Out = 0, In = 1, Second Index: Endpoint # */
  qh_t* queue_heads[2][16];
} ehci_dev_info_t;

typedef struct {
  list_head_t itds;             /* -- EM -- Remove this to use urb private data like the rest */
  struct urb* urb;
  int pipe_type;                /* -- EM -- Remove this to use pipe in urb */
  list_head_t chain_list;
} ehci_completion_element_t;


typedef struct
{
  usb_hcd_t usb_hcd;

  ehci_dev_info_t ehci_devinfo[USB_MAX_DEVICES+1];
  
  uint32_t bus;
  uint32_t dev;
  uint32_t func;
  uint32_t irq_line;
  uint8_t  handler_vector;
  
  phys_addr_t   base_physical_address;
  addr_t        base_virtual_address;
  ehci_caps_t*  caps;
  ehci_regs_t*  regs;

  frm_lst_lnk_ptr_t* frame_list;
  frm_lst_lnk_ptr_t** frame_list_virt;
  frm_lst_lnk_ptr_t* last_frame_list_entries;
  uint32_t           frame_list_size;
  
  uint32_t* micro_frame_remaining_time_periodic;
  uint32_t* micro_frame_remaining_time_async;

  spinlock uninserted_itd_lock;
  list_head_t uninserted_itd_urb_list;
  
#define EHCI_DECLARE_POOL(type)                 \
  type##_t*   type##_pool;                      \
  phys_addr_t type##_pool_phys_addr;            \
  uint32_t    type##_pool_size;                 \
  uint32_t*   used_##type##_bitmap;             \
  uint32_t    used_##type##_bitmap_size;        
  
  
  EHCI_DECLARE_POOL(qh)
  EHCI_DECLARE_POOL(qtd)
  EHCI_DECLARE_POOL(itd)
  EHCI_DECLARE_POOL(ehci_completion_element)

#undef EHCI_DECLARE_POOL
  
  uint32_t interrupt_threshold;
  uint32_t num_ports;
  
  bool periodic_list_on;

  qh_t* async_head;
  list_head_t reclaim_list;

  spinlock completion_lock; /* Protects both completion list and the
                             * completion_element pool related
                             * items */
  list_head_t completion_list;

  /* These are used during enumeration because if possible we use the
     same QH, if all we had were high speed devices we would not need
     this as the same QH could be used.  For low and full speed
     devices we need to specify the hub address and port number so we
     need to change the QH */
  int zero_qh_hub_addr;         /* Initialized to -1 */
  int zero_qh_port_num;        /* Initialized to -1 */

#define MAX_RT_URBS 30
  struct urb* rt_urbs[MAX_RT_URBS];
  int num_rt_urbs;
  int new_assignments[MAX_RT_URBS];

  bool reordered_map[1024];

#ifdef EHCI_USE_IOVCPU
#define EHCI_IOC_BH_THREAD_STACK_SIZE 1024
  
  uint32_t bh_stack[EHCI_IOC_BH_THREAD_STACK_SIZE];

  task_id master_iovcpu;
#endif


#define EHCI_RUS_NONE 0
#define EHCI_RUS_REORDER_OLD 1
  
  int reorder_urbs_stage;
  
#ifdef EHCI_IOC_BUGEHCI_IOC_BUG

#define IOC_BACKUP_THREAD_STACK_SIZE 1024
  
  uint32_t ioc_backup_thread_stack[IOC_BACKUP_THREAD_STACK_SIZE];

#endif // EHCI_IOC_BUG
  
} ehci_hcd_t;



/***************************************************************
 * EHCI URB private data start
 ***************************************************************/

/*
 * All ehci private urb data contain this struct
 */

typedef struct
{
  uint usecs_per_transactions;
  int sched_assignment;
} ehci_urb_priv_t;

ehci_urb_priv_t* get_ehci_urb_priv(struct urb* urb);

/***************************************************************
 * EHCI URB private data start
 ***************************************************************/


/***************************************************************
 * Isochronous URB private data start
 ***************************************************************/

typedef struct
{
  ehci_urb_priv_t ehci_urb_priv;
  list_head_t uninserted_itds_list;
  list_head_t uninserted_itd_urb_list;
  uint next_packet_to_free;
  uint next_packet_to_make_available;
  int next_itd_for_sending_data;
  uint num_itds;
  uint next_packet_for_sending_data;
  uint packets_in_use;
  uint last_transaction_ioc_processed;
  list_head_t write_itds_to_free;
  ehci_completion_element_t* completion_element;
  
  /* itds Must be last */
  itd_t* itds[0];
} ehci_iso_urb_priv_t;


/* This should always be a macro, inserting qhs into the frame list is
   time sensitive */

/*
 * -- EM -- insert_qh_into_frame_list is also broken because it does
 * not update previous for the element it links to but that should not
 * matter until we start removing qhs from the periodic list
 */
#define insert_qh_into_frame_list(lst_lnk_ptr, lst_lnk_ptr_virt, qh)    \
  do {                                                                  \
    (qh)->next_virt = *lst_lnk_ptr_virt;                                \
    *lst_lnk_ptr_virt = qh;                                             \
    (qh)->horizontalPointer = *lst_lnk_ptr;                             \
    lst_lnk_ptr->raw = QH_NEXT(ehci_hcd, qh);                           \
} while (0)

/*
 * -- EM -- unlink_qh_from_periodic is broken!!
 */
#define unlink_qh_from_periodic(qh)                     \
  do {                                                  \
    DLOG("unlink_qh_from_periodic is broken");          \
    panic("unlink_qh_from_periodic is broken");         \
    (qh)->previous->raw = (qh)->horizontalPointer.raw;  \
  } while(0)                                            \


static inline void
initialise_iso_urb_priv(ehci_iso_urb_priv_t* iso_urb_priv, struct urb* urb)
{
  uint32_t microframe_interval = urb->interval;
  uint32_t packets_per_itd = microframe_interval < 8 ? 8 / microframe_interval : 1;
  
  iso_urb_priv->ehci_urb_priv.sched_assignment      = -1;
  iso_urb_priv->next_packet_to_free                 = 0;
  iso_urb_priv->next_packet_to_make_available       = 0;
  iso_urb_priv->next_itd_for_sending_data           = -1;
  iso_urb_priv->next_packet_for_sending_data        = 0;
  iso_urb_priv->last_transaction_ioc_processed      = 0;
  iso_urb_priv->completion_element                  = NULL;
  iso_urb_priv->packets_in_use = (packets_per_itd * MAX_SAFE_FRAME_INDEX_LOOK_AHEAD);
  //memset(iso_urb_priv->used_table_entries_bitmap, 0, 1024/32);
  INIT_LIST_HEAD(&iso_urb_priv->write_itds_to_free);
  INIT_LIST_HEAD(&iso_urb_priv->uninserted_itds_list);
  INIT_LIST_HEAD(&iso_urb_priv->uninserted_itd_urb_list);
}


static inline
ehci_iso_urb_priv_t* ehci_alloc_iso_urb_priv(struct urb* urb)
{
  int frame_interval = urb->interval / 8;
  if(frame_interval == 0) frame_interval = 1;
  int num_itds = 1024 / frame_interval;
  ehci_iso_urb_priv_t* temp;
  temp = kmalloc(sizeof(ehci_iso_urb_priv_t) +
             num_itds * sizeof(itd_t*));
  if(temp == NULL) {
    return NULL;
  }
  initialise_iso_urb_priv(temp, urb);
  return temp;
}

#define iso_urb_priv_to_urb(iso_urb_priv)                       \
  (container_of((void*)(iso_urb_priv), struct urb, hcpriv))

#define iso_urb_priv_data_available(iso_urb_priv)                       \
  ((iso_urb_priv)->last_frame_read != (iso_urb_priv)->last_frame_processed)

#define get_iso_urb_priv(urb) ((ehci_iso_urb_priv_t*)urb->hcpriv)

/***************************************************************
 * Isochronous URB private data end
 ***************************************************************/

/***************************************************************
 * QH URB private data start
 *
 * Both bulk, control and interrupt URB private data share similar
 * data elements because they all use QHs.  This is where the common
 * data elements are located
 ***************************************************************/

typedef struct
{
  qh_t* qh;
  uint32_t next_qtd_to_free;
  uint32_t next_byte_to_free_in_qtd;
  uint32_t next_qtd_to_make_available;
  uint32_t next_byte_to_make_available_in_qtd;
  
  uint32_t next_byte_to_free_for_writing;
  uint32_t next_byte_to_use_for_writing;
  uint32_t bytes_in_buffer;
  uint32_t buffer_size;
  unsigned int num_qtds;
  uint next_qtd_for_ioc;
  ehci_completion_element_t* completion_element;
  bool extra_nonrt_qtd;
  qtd_t* qtds[0];
} ehci_qh_urb_priv_t;


ehci_qh_urb_priv_t* ehci_get_qh_urb_priv(struct urb* urb);

/***************************************************************
 * QH URB private data end
 ***************************************************************/

/***************************************************************
 * Interrupt URB private data start
 ***************************************************************/

typedef struct
{
  ehci_urb_priv_t ehci_urb_priv;
  ehci_qh_urb_priv_t qh_urb_priv;
  /* Nothing can be after this because qh_urb_priv has a variable size
   * array at its end */
} ehci_int_urb_priv_t;

static void
initialise_int_urb_priv(ehci_int_urb_priv_t* priv, unsigned int num_qtds)
{
  memset(priv, 0,
         sizeof(ehci_int_urb_priv_t) + num_qtds * sizeof(qtd_t*));
  priv->qh_urb_priv.num_qtds = num_qtds;
}

static inline
ehci_int_urb_priv_t* ehci_alloc_int_urb_priv(unsigned int num_qtds)
{
  ehci_int_urb_priv_t* temp;
  temp = kmalloc(sizeof(ehci_int_urb_priv_t) +
             num_qtds * sizeof(qtd_t*));
  if(temp == NULL) {
    return NULL;
  }
  initialise_int_urb_priv(temp, num_qtds);
  return temp;
}

#define int_urb_priv_to_urb(int_urb_priv)                       \
  (container_of((void*)(int_urb_priv), struct urb, hcpriv))

#define get_int_urb_priv(urb) ((ehci_int_urb_priv_t*)urb->hcpriv)

/***************************************************************
 * Interrupt URB private data end
 ***************************************************************/

/***************************************************************
 * Bulk URB private data start
 ***************************************************************/

typedef struct
{
  ehci_urb_priv_t ehci_urb_priv;
  ehci_qh_urb_priv_t qh_urb_priv;
  /* Nothing can be after this because qh_urb_priv has a variable size
   * array at the end */
} ehci_bulk_urb_priv_t;

static void
initialise_bulk_urb_priv(ehci_bulk_urb_priv_t* priv, unsigned int num_qtds)
{
  memset(priv, 0,
         sizeof(ehci_bulk_urb_priv_t) + num_qtds * sizeof(qtd_t*));
  priv->qh_urb_priv.num_qtds = num_qtds;
}

static inline
ehci_bulk_urb_priv_t* ehci_alloc_bulk_urb_priv(unsigned int num_qtds)
{
  ehci_bulk_urb_priv_t* temp;
  temp = kmalloc(sizeof(ehci_bulk_urb_priv_t) +
             num_qtds * sizeof(qtd_t*));
  if(temp == NULL) {
    return NULL;
  }
  initialise_bulk_urb_priv(temp, num_qtds);
  return temp;
}

#define bulk_urb_priv_to_urb(bulk_urb_priv)                       \
  (container_of((void*)(bulk_urb_priv), struct urb, hcpriv))

#define get_bulk_urb_priv(urb) ((ehci_bulk_urb_priv_t*)urb->hcpriv)

/***************************************************************
 * Bulk URB private data end
 ***************************************************************/




/***************************************************************
 * Control URB private data start
 ***************************************************************/

typedef struct
{
  ehci_urb_priv_t ehci_urb_priv;
  ehci_qh_urb_priv_t qh_urb_priv;
  /* Nothing can be after this because qh_urb_priv has a variable size
   * array at the end */
} ehci_ctrl_urb_priv_t;

static void
initialise_ctrl_urb_priv(ehci_ctrl_urb_priv_t* priv, unsigned int num_qtds)
{
  memset(priv, 0,
         sizeof(ehci_ctrl_urb_priv_t) + num_qtds * sizeof(qtd_t*));
  priv->qh_urb_priv.num_qtds = num_qtds;
}

static inline
ehci_ctrl_urb_priv_t* ehci_alloc_ctrl_urb_priv(unsigned int num_qtds)
{
  ehci_ctrl_urb_priv_t* temp;
  temp = kmalloc(sizeof(ehci_ctrl_urb_priv_t) +
             num_qtds * sizeof(qtd_t*));
  if(temp == NULL) {
    return NULL;
  }
  initialise_ctrl_urb_priv(temp, num_qtds);
  return temp;
}

#define ctrl_urb_priv_to_urb(ctrl_urb_priv)                       \
  (container_of((void*)(ctrl_urb_priv), struct urb, hcpriv))

#define get_ctrl_urb_priv(urb) ((ehci_ctrl_urb_priv_t*)urb->hcpriv)

/***************************************************************
 * Control URB private data end
 ***************************************************************/





#define EHCI_GET_DEVICE_QH(ehci_hcd, addr, is_input, endpoint)          \
  ((ehci_hcd)->ehci_devinfo[(addr)]                                     \
   .queue_heads[(is_input) && ((endpoint) != 0)][(endpoint)])

#define EHCI_SET_DEVICE_QH(ehci_hcd, addr, is_input, endpoint, qh)      \
  ((ehci_hcd)->ehci_devinfo[(addr)]                                     \
   .queue_heads[(is_input) && ((endpoint) != 0)][(endpoint)] = qh)

#define calc_bitmap_size(hcd, pool_size)                        \
  ((hcd)->pool_size + (EHCI_ELEMENTS_PER_BITMAP_ENTRY - 1))     \
  / EHCI_ELEMENTS_PER_BITMAP_ENTRY;

#define calc_used_qh_bitmap_size(hcd)           \
  calc_bitmap_size(hcd, qh_pool_size)

#define calc_used_qtd_bitmap_size(hcd)          \
  calc_bitmap_size(hcd, qtd_pool_size)

#define calc_used_itd_bitmap_size(hcd)          \
  calc_bitmap_size(hcd, itd_pool_size)

#define calc_used_ehci_completion_element_bitmap_size(hcd)    \
  calc_bitmap_size(hcd, ehci_completion_element_pool_size)

#endif // _EHCI_H_

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
