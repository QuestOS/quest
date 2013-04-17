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

/*
 * -- EM --
 * To do:
 *
 * 1) Devices really shouldn't do initialisation at boot they should
 * most of their initialisation when they are opened, i.e. in open not
 * probe
 *
 * 2) Reordering of write itd transactions
 *
 * 3) Reordering of itds that change frame list position
 *
 */

#include <smp/apic.h>
#include <drivers/usb/usb.h>
#include <drivers/usb/ehci.h>
#include <drivers/usb/ehci_mem.h>
#include <drivers/usb/ehci_debug.h>
#include <mem/virtual.h>
#include <mem/malloc.h>
#include <arch/i386-div64.h>
#include <kernel.h>
#include <sched/sched.h>
#include <util/sort.h>
#include <sched/vcpu.h>



#define EHCI_IS_INDEX_IN_CIRC_BUF_REGION(test_index, start_index, end_index) \
  (start_index < end_index ?                                            \
   start_index <= test_index && test_index <= end_index                 \
   : test_index >= start_index || test_index <= end_index)

/*
 * Following two macros were taken from Linux
 */
// high bandwidth multiplier, as encoded in highspeed endpoint descriptors
#define hb_mult(wMaxPacketSize) (1 + (((wMaxPacketSize) >> 11) & 0x03))

// ... and packet size, for any kind of endpoint descriptor
#define max_packet(wMaxPacketSize) ((wMaxPacketSize) & 0x07ff)

static int ehci_async_transfer(ehci_hcd_t* ehci_hcd, struct urb* urb);

static int ehci_isochronous_transfer(ehci_hcd_t* ehci_hcd, struct urb* urb);

static int ehci_interrupt_transfer(ehci_hcd_t* ehci_hcd, struct urb* urb);

static int ehci_is_rt_schedulable(ehci_hcd_t* ehci_hcd, struct urb* urb);

static void qh_append_qtds(ehci_hcd_t* ehci_hcd, struct urb* urb,
                           qh_t* qh, list_head_t* qtd_list);

static void qh_append_qtd(ehci_hcd_t* ehci_hcd, struct urb* urb,
                          qh_t* qh, qtd_t* qtd);

static int submit_itd_chain(ehci_hcd_t* ehci_hcd, list_head_t* itd_list,
                            struct urb* urb);

static bool
create_qtd_chain(ehci_hcd_t* ehci_hcd,
                 struct urb* urb, addr_t setup_req,
                 uint32_t setup_len, addr_t data,
                 uint32_t data_len, bool enable_ioc,
                 list_head_t* qtd_list, /* Should be empty/uninitialized */
                 uint interrupt_byte_rate,
                 bool phys_data_addr,
                 void* completion_context);

static int create_itd_chain(ehci_hcd_t* ehci_hcd, uint8_t address,
                            uint8_t endpoint, addr_t data,
                            usb_iso_packet_descriptor_t* packets,
                            int num_packets,
                            int max_packet_len, bool is_input,
                            list_head_t* itd_list, bool ioc,
                            struct urb* urb, uint interrupt_frame_rate,
                            void* completion_context);

static inline frm_lst_lnk_ptr_t**
get_next_frm_lst_lnk_ptrs_previous(ehci_hcd_t* ehci_hcd, frm_lst_lnk_ptr_t* ptr,
                                   frm_lst_lnk_ptr_t* next_virt);

static void insert_remaining_realtime_itds(ehci_hcd_t* ehci_hcd, struct urb* urb);

/* Reminder: inserting itds is time sensitive */
#define insert_itd_into_frame_list(hcd, list, list_virt, index, itd)    \
  do {                                                                  \
    frm_lst_lnk_ptr_t** __nexts_prev =                                  \
      get_next_frm_lst_lnk_ptrs_previous(hcd, &(list)[index],           \
                                         (hcd)->frame_list_virt[index]); \
    (itd)->frame_index = index;                                         \
    if(__nexts_prev) *__nexts_prev = &(itd)->next_link_pointer;         \
    (itd)->next_link_pointer = (list)[index];                           \
    (itd)->previous = &(list[index]);                                   \
    (list)[index].raw = ITD_NEXT(hcd, (itd));                           \
    (itd)->next_virt = (list_virt)[index];                              \
    (list_virt)[index] = (frm_lst_lnk_ptr_t*)(itd);                     \
  } while(0)


#define unlink_itd_from_frame_list(hcd, itd)                            \
  do {                                                                  \
    frm_lst_lnk_ptr_t** __itds_nexts_prev =                             \
      get_next_frm_lst_lnk_ptrs_previous(hcd, &(itd)->next_link_pointer, itd->next_virt); \
    if(__itds_nexts_prev) (*__itds_nexts_prev) = (itd)->previous;       \
    *((itd)->previous) = (itd)->next_link_pointer;                      \
  } while(0)

/*
 * Default frame list size specified by EHCI Specs
 */
#define DEFAULT_FRM_LST_SIZE 1024 

#define FRM_LIST_SIZE 1024  /* Frame list size used in Quest */
#define DEFAULT_INT_THRESHOLD 8

static ehci_hcd_t ehci_hcd;
static frm_lst_lnk_ptr_t frame_list[FRM_LIST_SIZE] ALIGNED(0x1000);
static frm_lst_lnk_ptr_t last_frame_list_entries[FRM_LIST_SIZE] ALIGNED(0x1000);

static uint32_t micro_frame_remaining_time_periodic[FRM_LIST_SIZE * 8];
static uint32_t micro_frame_remaining_time_async[FRM_LIST_SIZE * 8];



static int ehci_rt_int_data_lost(struct urb* urb);
static int ehci_rt_bulk_data_lost(struct urb* urb);
static int ehci_rt_iso_data_lost(struct urb* urb);
static inline bool is_itd_partially_active(itd_t* itd);

static int ehci_rt_free_write_resources(struct urb* urb)
{
  ehci_hcd_t* ehci_hcd = hcd_to_ehci_hcd(urb->dev->hcd);
  
  if(usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS) {
    ehci_iso_urb_priv_t* iso_urb_priv = get_iso_urb_priv(urb);
    itd_t* itd_to_free;
    itd_t* temp_itd;
    uint32_t packets_per_itd = urb->interval < 8 ? 8 / urb->interval : 1;
    
    
    list_for_each_entry_safe(itd_to_free, temp_itd, &iso_urb_priv->write_itds_to_free,
                             chain_list) {
      if(is_itd_partially_active(itd_to_free)) {
        break;
      }
      unlink_itd_from_frame_list(ehci_hcd, itd_to_free);
      if(!(urb->transfer_flags & URB_RT_ZERO_COPY_WRITE)) {
        iso_urb_priv->packets_in_use -= packets_per_itd;
      }
      
      free_itd(ehci_hcd, itd_to_free);
    }
  }
  else {
    qh_t* qh;
    qtd_t* temp_qtd;
    qtd_t* qtd_to_remove;
    ehci_qh_urb_priv_t* qh_urb_priv;
    uint32_t buffer_size;
    
    switch(usb_pipetype(urb->pipe)) {
    case PIPE_BULK:
      qh_urb_priv = &(get_bulk_urb_priv(urb)->qh_urb_priv);
      break;

    case PIPE_INTERRUPT:
      qh_urb_priv = &(get_int_urb_priv(urb)->qh_urb_priv);
      break;

    default: // PIPE_CONTROL
      
      DLOG("Reached unimplemented case in %s", __FUNCTION__);
      panic("Reached unimplemented case in ehci_rt_free_write_resources");
    }
    
    buffer_size = qh_urb_priv->buffer_size;
    qh = qh_urb_priv->qh;
    
    list_for_each_entry_safe(qtd_to_remove, temp_qtd, &qh->qtd_list, chain_list) {

      
      if(qh->current_qtd_ptr_raw == qtd_to_remove->dma_addr ||
         qh->dummy_qtd == qtd_to_remove) {
        break;
      }
      else {
        if(!(urb->transfer_flags & URB_RT_ZERO_COPY_WRITE)) {
          if(qh_urb_priv->extra_nonrt_qtd) {
            qh_urb_priv->extra_nonrt_qtd = FALSE;
          }
          else {
            qh_urb_priv->next_byte_to_free_for_writing
              += qtd_to_remove->original_total_bytes_to_transfer;
            qh_urb_priv->bytes_in_buffer -= qtd_to_remove->original_total_bytes_to_transfer;
            if(qh_urb_priv->next_byte_to_free_for_writing >= buffer_size) {
              qh_urb_priv->next_byte_to_free_for_writing -= buffer_size;
            }
          }
        }
        free_qtd(ehci_hcd, qtd_to_remove);
      }
    }
  }

  return 0;
}

static int ehci_rt_data_lost(struct urb* urb)
{
  switch(usb_pipetype(urb->pipe)) {
  case PIPE_BULK:
    return ehci_rt_bulk_data_lost(urb);
  case PIPE_ISOCHRONOUS:
    return ehci_rt_iso_data_lost(urb);
  case PIPE_INTERRUPT:
    return ehci_rt_int_data_lost(urb);
  default: // case PIPE_CONTROL:
    return -1;
  }
}

/*
 * -- EM -- Need to actually implement this function just a dummy
 * right now
 */
static int ehci_rt_int_data_lost(struct urb* urb)
{
  DLOG("ehci_rt_int_data_lost is not implemented");
  panic("ehci_rt_int_data_lost is not implemented");
  return 0;
}

/*
 * -- EM -- Need to actually implement this function just a dummy
 * right now
 */
static int ehci_rt_bulk_data_lost(struct urb* urb)
{
  DLOG("ehci_rt_bulk_data_lost is not implemented");
  panic("ehci_rt_bulk_data_lost is not implemented");
  return 0;
}



static int ehci_rt_qh_update_data(struct urb* urb, ehci_qh_urb_priv_t* qh_urb_priv,
                                  int max_count)
{
  qtd_t** qtds = qh_urb_priv->qtds;
  int num_qtds = qh_urb_priv->num_qtds;
  int original_max_count = max_count;
  int next_qtd_to_free = qh_urb_priv->next_qtd_to_free;
  int next_byte_to_free_in_qtd = qh_urb_priv->next_byte_to_free_in_qtd;

  while(max_count) {
    int next_qtd_to_make_available = qh_urb_priv->next_qtd_to_make_available;
    int next_byte_to_make_available_in_qtd
      = qh_urb_priv->next_byte_to_make_available_in_qtd;
    
    qtd_t* current_qtd = qtds[next_qtd_to_make_available];
    int bytes_remaining_to_transfer = (current_qtd->token >> 16) & 0x7FFF;
    
    int newly_available_bytes = 
      current_qtd->original_total_bytes_to_transfer
      - bytes_remaining_to_transfer
      - qh_urb_priv->next_byte_to_make_available_in_qtd;
    
    EHCI_ASSERT(max_count > 0);
    EHCI_ASSERT(newly_available_bytes >= 0);
    
#if 0
    DLOG_INT(current_qtd->original_total_bytes_to_transfer);
    DLOG_INT(next_qtd_to_make_available);
    DLOG_INT(next_byte_to_make_available_in_qtd);
    DLOG_INT(bytes_remaining_to_transfer);
    DLOG_INT(newly_available_bytes);
    DLOG_INT(next_qtd_to_free);
    DLOG_INT(next_byte_to_free_in_qtd);
    DLOG_INT(qh_urb_priv->buffer_size);
    DLOG_INT(qh_urb_priv->bytes_in_buffer);
#endif

    if(qh_urb_priv->bytes_in_buffer == qh_urb_priv->buffer_size) {
      break;
    }
    
    if( (next_qtd_to_free == next_qtd_to_make_available) &&
        (next_byte_to_make_available_in_qtd < next_byte_to_free_in_qtd)) {
      int diff = next_byte_to_free_in_qtd - next_byte_to_make_available_in_qtd;
      if(diff > max_count) {
        qh_urb_priv->next_byte_to_make_available_in_qtd += max_count;
        qh_urb_priv->bytes_in_buffer += max_count;
        max_count = 0;
      }
      else {
        qh_urb_priv->next_byte_to_make_available_in_qtd += diff;
        max_count -= diff;
        qh_urb_priv->bytes_in_buffer += diff;
      }
      break;
    }
    else {
      if(newly_available_bytes) {
        if(newly_available_bytes > max_count) {
          /*
           * More bytes than requested are available, don't advance
           * qtd counter
           */
          qh_urb_priv->next_byte_to_make_available_in_qtd += max_count;
          qh_urb_priv->bytes_in_buffer += max_count;
          max_count = 0;
        }
        else {
          /*
           * Take everything available in this qtd and advance qtd
           * counter and reset qtd_byte counter to 0 if we have reached
           * the end of the qtd
           */
          max_count -= newly_available_bytes; 
          qh_urb_priv->bytes_in_buffer += newly_available_bytes;
          if(bytes_remaining_to_transfer) {
            qh_urb_priv->next_byte_to_make_available_in_qtd
              += newly_available_bytes;
            break;
          }
          else {
            qh_urb_priv->next_byte_to_make_available_in_qtd = 0;
            qh_urb_priv->next_qtd_to_make_available++;
            if(qh_urb_priv->next_qtd_to_make_available == num_qtds) {
              qh_urb_priv->next_qtd_to_make_available = 0;
            }
          }
        }
      }
      else {
        break;
      }
    }
  }
  
  EHCI_ASSERT(max_count >= 0);
  
  return original_max_count - max_count;
}


static int ehci_rt_update_data(struct urb* urb, int max_count)
{
  switch(usb_pipetype(urb->pipe)) {
  case PIPE_BULK:
    return ehci_rt_qh_update_data(urb, &(get_bulk_urb_priv(urb)->qh_urb_priv),
                                  max_count);
  case PIPE_ISOCHRONOUS:
    return -1;
  case PIPE_INTERRUPT:
    return ehci_rt_qh_update_data(urb, &(get_int_urb_priv(urb)->qh_urb_priv),
                                  max_count);
  default: // case PIPE_CONTROL:
    return -1;
  }
}

static inline bool is_itd_partially_active(itd_t* itd)
{
  itd_transaction_t* transactions = itd->transaction;
  return ((transactions[0].raw & ITD_ACTIVE) ||
          (transactions[1].raw & ITD_ACTIVE) ||
          (transactions[2].raw & ITD_ACTIVE) ||
          (transactions[3].raw & ITD_ACTIVE) ||
          (transactions[4].raw & ITD_ACTIVE) ||
          (transactions[5].raw & ITD_ACTIVE) ||
          (transactions[6].raw & ITD_ACTIVE) ||
          (transactions[7].raw & ITD_ACTIVE));
}

static inline int add_data_to_itd(struct urb* urb, itd_t* itd,
                                  char* data, int count,
                                  usb_iso_packet_descriptor_t* packet_descs,
                                  uint32_t max_transaction_size)
{
  int i, j;
  int interval                      = urb->interval;
  char* data_start                  = urb->transfer_buffer;
  uint32_t bytes_added              = 0;
  ehci_iso_urb_priv_t* iso_urb_priv = get_iso_urb_priv(urb);
  int start_transaction             = iso_urb_priv->ehci_urb_priv.sched_assignment;
  
  while(start_transaction > 8) start_transaction -= 8;

  for(i = start_transaction, j = 0;
      (i < 8) && (bytes_added < count);
      i += interval, ++j) {
    itd_transaction_t* transaction = &itd->transaction[i];
    uint32_t bytes_to_add_for_transaction = max_transaction_size;
    if(transaction->raw & ITD_ACTIVE) {
      bytes_to_add_for_transaction -= transaction->length;
    }
    
    if(bytes_to_add_for_transaction > count) {
      bytes_to_add_for_transaction = count;
    }

    if(bytes_to_add_for_transaction > 0) {
      
      memcpy(&data_start[packet_descs[j].offset], &data[bytes_added],
             bytes_to_add_for_transaction);
      
      transaction->length += bytes_to_add_for_transaction;
      bytes_added += bytes_to_add_for_transaction;
      if(!(transaction->raw & ITD_ACTIVE)) {
        transaction->raw |= ITD_ACTIVE;
      }
    }
  }

  return bytes_added;
}

/*
 * Only valid for write urbs because the itds are placed at the start
 * of the periodic list
 */
#define get_itd_index_from_frame_index(frame_index, frame_interval,     \
                                       frame_interval_offset)           \
  (((frame_index) - (frame_interval_offset)) / (frame_interval))



int fill_iso_push_packets(struct urb* urb, int count,
                          uint32_t start_packet)
{
  usb_iso_packet_descriptor_t* packets = urb->iso_frame_desc;
  int i;

  uint32_t max_transaction_size =
    usb_payload_size(urb->dev, usb_pipein(urb->pipe) ?
                     &urb->dev->ep_in[usb_pipeendpoint(urb->pipe)].desc :
                     &urb->dev->ep_out[usb_pipeendpoint(urb->pipe)].desc);

  i = 0;
  while(count) {
    if(count < max_transaction_size) {
      packets[start_packet].length = count;
      count = 0;
    }
    else {
      packets[start_packet].length = max_transaction_size;
      count -= max_transaction_size;
    }
    ++start_packet;
    ++i;

    if(start_packet == urb->number_of_packets) {
      start_packet = 0;
    }
  }

  return i;
}


#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

int ehci_rt_iso_push_data(struct urb* urb, char* data, int count,
                          uint interrupt_frame_rate, uint flags, void* completion_context)
{
  unsigned int pipe = urb->pipe;
  uint32_t microframe_interval = urb->interval;
  uint32_t packets_per_itd = microframe_interval < 8 ? 8 / microframe_interval : 1;
  ehci_iso_urb_priv_t* iso_urb_priv = get_iso_urb_priv(urb);
  usb_iso_packet_descriptor_t* packets = urb->iso_frame_desc;
  list_head_t itd_list;
  ehci_hcd_t* ehci_hcd = hcd_to_ehci_hcd(urb->dev->hcd);
  uint32_t packets_remaining;
  int num_packets;
  uint32_t start_packet_for_data = iso_urb_priv->next_packet_for_sending_data;
  int effective_packet_num;
  ehci_completion_element_t* comp_elem = NULL;

  insert_remaining_realtime_itds(hcd_to_ehci_hcd(urb->dev->hcd), urb);


  if(urb->transfer_flags & URB_RT_ZERO_COPY_WRITE) {
    /* -- EM -- Implement this later */
    DLOG("ZERO COPY WRITE Not support for isochronous endpoints yet");
    panic("ZERO COPY WRITE Not support for isochronous endpoints yet");
  }
  
  if(iso_urb_priv->completion_element == NULL && interrupt_frame_rate != 0) {
    comp_elem = allocate_ehci_completion_element(ehci_hcd);
    if(comp_elem == NULL) {
      DLOG("Failed to allocate completion element");
      return -1;
    }
  }
  
  packets_remaining = urb->number_of_packets - iso_urb_priv->packets_in_use;
  uint32_t max_transaction_size =
    usb_payload_size(urb->dev, usb_pipein(urb->pipe) ?
                     &urb->dev->ep_in[usb_pipeendpoint(urb->pipe)].desc :
                     &urb->dev->ep_out[usb_pipeendpoint(urb->pipe)].desc);
  
  if(count > packets_remaining * max_transaction_size) {
    count = packets_remaining * max_transaction_size;
  }

  num_packets = fill_iso_push_packets(urb, count, start_packet_for_data);
  
  
  if(start_packet_for_data + num_packets < urb->number_of_packets) {
    memcpy(&((char*)urb->transfer_buffer)[packets[start_packet_for_data].offset],
           data, count);
  }
  else {
    int packets_in_first_memcpy = urb->number_of_packets - start_packet_for_data;
    int bytes_in_first_memcpy = packets_in_first_memcpy * max_transaction_size;
    int bytes_in_second_memcpy = count - bytes_in_first_memcpy;
    
    
    memcpy(&((char*)urb->transfer_buffer)[packets[start_packet_for_data].offset],
           data, bytes_in_first_memcpy);
    memcpy(urb->transfer_buffer, &data[bytes_in_first_memcpy],
           bytes_in_second_memcpy);
  }

  effective_packet_num = DIV_ROUND_UP(num_packets, packets_per_itd) * packets_per_itd;
  
  iso_urb_priv->next_packet_for_sending_data += effective_packet_num;
  if(iso_urb_priv->next_packet_for_sending_data >= urb->number_of_packets) {
    iso_urb_priv->next_packet_for_sending_data -= urb->number_of_packets;
  }

  iso_urb_priv->packets_in_use += effective_packet_num;
  EHCI_ASSERT(iso_urb_priv->packets_in_use <= urb->number_of_packets);
    
  
  if(create_itd_chain(ehci_hcd, usb_pipedevice(pipe),
                      usb_pipeendpoint(pipe),
                      urb->transfer_buffer,
                      &packets[start_packet_for_data],
                      num_packets,
                      usb_maxpacket(urb->dev, urb->pipe),
                      usb_pipein(pipe),
                      &itd_list, mp_enabled,
                      urb, interrupt_frame_rate,
                      completion_context) < 0) {
    return -1;

  }

  if(submit_itd_chain(ehci_hcd, &itd_list, urb) < 0) {
    return -1;
  }

  if(comp_elem) {
    iso_urb_priv->completion_element = comp_elem;
    comp_elem->urb = urb;
    INIT_LIST_HEAD(&comp_elem->itds);
    list_splice(&itd_list, &comp_elem->itds);
    comp_elem->pipe_type = PIPE_ISOCHRONOUS;
    /* -- EM -- Using completion lock to also protect
       write_itds_to_free */
    spinlock_lock(&ehci_hcd->completion_lock);
    list_splice_tail(&itd_list, &iso_urb_priv->write_itds_to_free);
    list_add_tail(&comp_elem->chain_list, &ehci_hcd->completion_list);
    spinlock_unlock(&ehci_hcd->completion_lock);
  }
  else {
    /* -- EM -- Using completion lock to also protect
       write_itds_to_free */
    spinlock_lock(&ehci_hcd->completion_lock);
    list_splice_tail(&itd_list, &iso_urb_priv->write_itds_to_free);
    spinlock_unlock(&ehci_hcd->completion_lock);
  }

  return count;
}


static inline frm_lst_lnk_ptr_t**
get_next_frm_lst_lnk_ptrs_previous(ehci_hcd_t* ehci_hcd, frm_lst_lnk_ptr_t* ptr,
                                   frm_lst_lnk_ptr_t* next_virt)
{
  if(ptr->tBit) return NULL;

  switch(ptr->type) {
  case TYPE_ITD:
    return &((itd_t*)next_virt)->previous;
    
  case TYPE_QH:
    return &((qh_t*)next_virt)->previous;
    
  default: // Other two types we don't support yet
    DLOG("Reached default in %s", __FUNCTION__);
    panic("Default case reached in get_next_frm_lst_lnk_ptr");
    return NULL;
  }
}


static void qtd_restore(qtd_t* qtd)
{
  qtd->token = qtd->token_backup;
  qtd->buffer_page[0] = qtd->buffer_page_zero_backup;
  qtd->ioc_called = FALSE;
}

static int ehci_rt_qh_free_data(struct urb* urb, ehci_qh_urb_priv_t* qh_urb_priv,
                                int count)
{
  qtd_t** qtds = qh_urb_priv->qtds;
  int num_qtds = qh_urb_priv->num_qtds;
  int next_qtd_to_make_available = qh_urb_priv->next_qtd_to_make_available;
  int next_byte_to_make_available_in_qtd
    = qh_urb_priv->next_byte_to_make_available_in_qtd;
  int original_count = count;
  qh_t* qh = qh_urb_priv->qh;
  ehci_hcd_t* ehci_hcd = hcd_to_ehci_hcd(urb->dev->hcd);

  
  while(count) {
    int next_qtd_to_free = qh_urb_priv->next_qtd_to_free;
    int next_byte_to_free_in_qtd = qh_urb_priv->next_byte_to_free_in_qtd;
    qtd_t* current_qtd = qtds[next_qtd_to_free];

    if(qh_urb_priv->bytes_in_buffer == 0) {
      break;
    }
    
    EHCI_ASSERT(count > 0);
    
    if((next_qtd_to_make_available == next_qtd_to_free) &&
       (next_byte_to_free_in_qtd <= next_byte_to_make_available_in_qtd)) {
      if(qh_urb_priv->bytes_in_buffer == qh_urb_priv->buffer_size) {
        /*
         * Edge case buffer is full, free as many bytes left in the
         * current qtd
         */
        int bytes_to_free = 
          current_qtd->original_total_bytes_to_transfer
          - qh_urb_priv->next_byte_to_free_in_qtd;
        
        if(qh->current_qtd_ptr_raw == current_qtd->dma_addr) {
          /*
           * The current qtd in the qh is the one we are trying to
           * free, can't free it yet, need to wait for hardware to
           * advance, so just break out, 
           */
          break;
        }
        
        count -= bytes_to_free;
        qh_urb_priv->bytes_in_buffer -= bytes_to_free;
        qh_urb_priv->next_byte_to_free_in_qtd = 0;
        qh_urb_priv->next_qtd_to_free++;
        if(qh_urb_priv->next_qtd_to_free == num_qtds) {
          qh_urb_priv->next_qtd_to_free = 0;
        }
        
        qtd_restore(current_qtd);
        current_qtd->next_pointer_raw = EHCI_LIST_END;
        qh_append_qtd(ehci_hcd, urb, qh_urb_priv->qh, current_qtd);
      }
      else {
        int diff = next_byte_to_make_available_in_qtd - next_byte_to_free_in_qtd;
        EHCI_ASSERT(diff >= 0);
        if(diff > count) {
          qh_urb_priv->next_byte_to_free_in_qtd += count;
          qh_urb_priv->bytes_in_buffer -= count;
          count = 0;
        }
        else {
          qh_urb_priv->next_byte_to_free_in_qtd += diff;
          qh_urb_priv->bytes_in_buffer -= diff;
          count -= diff;
        }
        break;
      }
      
    }
    else {
      int bytes_remaining_to_transfer = (current_qtd->token >> 16) & 0x7FFF;
      int max_bytes_to_free = 
        current_qtd->original_total_bytes_to_transfer
        - bytes_remaining_to_transfer
        - qh_urb_priv->next_byte_to_free_in_qtd;
      
      if(max_bytes_to_free) {
        if(max_bytes_to_free > count) {
          qh_urb_priv->next_byte_to_free_in_qtd += count;
          qh_urb_priv->bytes_in_buffer -= count;
          count = 0;
          
          break;
        }
        else {
          if(bytes_remaining_to_transfer) {
            qh_urb_priv->next_byte_to_free_in_qtd += max_bytes_to_free;
            count -= max_bytes_to_free;
            qh_urb_priv->bytes_in_buffer -= max_bytes_to_free;
            
            break;
          }
          else {
            /*
             * If we are here we are about to free the last bytes in
             * the qtd, reset the qtd and put the qtd back into list
             * at the end, however we do not want to reset the qtd if
             * the qh is still pointing to it as the qh will then
             * process this qtd next, instead of last.
             */

            if(qh->current_qtd_ptr_raw == current_qtd->dma_addr) {
              /*
               * The current qtd in the qh is the one we are trying to
               * free, can't free it yet, need to wait for hardware to
               * advance, so just break out, 
               */
              break;
            }
            
            count -= max_bytes_to_free;
            qh_urb_priv->bytes_in_buffer -= max_bytes_to_free;
            qh_urb_priv->next_byte_to_free_in_qtd = 0;
            qh_urb_priv->next_qtd_to_free++;
            if(qh_urb_priv->next_qtd_to_free == num_qtds) {
              qh_urb_priv->next_qtd_to_free = 0;
            }
            
            qtd_restore(current_qtd);
            current_qtd->next_pointer_raw = EHCI_LIST_END;
            qh_append_qtd(ehci_hcd, urb, qh_urb_priv->qh, current_qtd);
          }
        }
      }
      else {
        break;
        
      }
    }
  }

  return original_count - count;
}

static int ehci_rt_free_data(struct urb* urb, int count)
{
  switch(usb_pipetype(urb->pipe)) {
  case PIPE_BULK:
    return ehci_rt_qh_free_data(urb, &(get_bulk_urb_priv(urb)->qh_urb_priv), count);
  case PIPE_INTERRUPT:
    return ehci_rt_qh_free_data(urb, &(get_int_urb_priv(urb)->qh_urb_priv), count);
  case PIPE_ISOCHRONOUS:
    return -1;
  default: // case PIPE_CONTROL:
    return -1;
  }
}

/*
 * -- EM -- For the sake of simplicity for writing we do not free
 * bytes in a qtd until the entire qtd is done (like for reading when
 * we tell the device driver bytes are read).  This should not matter
 * because for reading we do it so bytes are available as soon as
 * possible for writing if the program is trying to write so much that
 * we run out of buffer space the program can either increase the
 * buffer size or increase the bytes per microframe sent, the delay is
 * not as important for writing
 */

static int ehci_rt_qh_push_data(struct urb* urb, ehci_qh_urb_priv_t* qh_urb_priv,
                                char* data, int count, uint interrupt_rate, uint flags,
                                void* completion_context)
{
  qh_t* qh = qh_urb_priv->qh;
  int bytes_sent = 0;
  char* urb_buffer = urb->transfer_buffer;
  list_head_t qtd_list;
  ehci_hcd_t* ehci_hcd = hcd_to_ehci_hcd(urb->dev->hcd);
  uint32_t buffer_size = qh_urb_priv->buffer_size;
  ehci_completion_element_t* completion_element = NULL;
  
  while(bytes_sent < count) {
    uint32_t bytes_for_this_chain;
    uint32_t max_possible_for_this_chain;

    if(!(urb->transfer_flags & URB_RT_ZERO_COPY_WRITE) &&
       qh_urb_priv->bytes_in_buffer == buffer_size) {
  
      break;
    }

    if(interrupt_rate != 0 && qh_urb_priv->completion_element == NULL) {
      completion_element = allocate_ehci_completion_element(ehci_hcd);
      if(completion_element == NULL) {
        DLOG("Failed to allocate completion element");
        return -1;
      }
    }
    
    if(urb->transfer_flags & URB_RT_ZERO_COPY_WRITE) {
      bytes_for_this_chain = count;
      if(!create_qtd_chain(ehci_hcd, urb, NULL, 0, data, bytes_for_this_chain,
                           mp_enabled, &qtd_list, interrupt_rate, !!(flags & URB_RT_PHYS_ADDR),
                           completion_context)) {
        return -1;
      }
    }
    else {
      if(!(qh_urb_priv->bytes_in_buffer < buffer_size)) {
        DLOG_UINT(qh_urb_priv->bytes_in_buffer);
        DLOG_UINT(buffer_size);
      }

      EHCI_ASSERT(qh_urb_priv->bytes_in_buffer < buffer_size);
    
      if(qh_urb_priv->next_byte_to_free_for_writing
         <= qh_urb_priv->next_byte_to_use_for_writing) {
        /* We might run into the end of the buffer */
        max_possible_for_this_chain
          = buffer_size - qh_urb_priv->next_byte_to_use_for_writing;
      }
      else {
        /* We might run into data */
        max_possible_for_this_chain = qh_urb_priv->next_byte_to_free_for_writing
          - qh_urb_priv->next_byte_to_use_for_writing;
      }
    
      EHCI_ASSERT(max_possible_for_this_chain > 0);

      if(count < max_possible_for_this_chain) {
        bytes_for_this_chain = count;
      }
      else {
        bytes_for_this_chain = max_possible_for_this_chain;
      }

      memcpy(&urb_buffer[qh_urb_priv->next_byte_to_use_for_writing],
             &data[bytes_sent],
             bytes_for_this_chain);
    
      if(!create_qtd_chain(ehci_hcd, urb, NULL, 0,
                           (addr_t)&urb_buffer[qh_urb_priv->next_byte_to_use_for_writing],
                           bytes_for_this_chain, mp_enabled, &qtd_list, interrupt_rate,
                           !!(flags & URB_RT_PHYS_ADDR), completion_context)) {
        return -1;
      }
      
      qh_urb_priv->bytes_in_buffer += bytes_for_this_chain;
      qh_urb_priv->next_byte_to_use_for_writing += bytes_for_this_chain;
      if(qh_urb_priv->next_byte_to_use_for_writing == buffer_size) {
        qh_urb_priv->next_byte_to_use_for_writing = 0;
      }
    }
    bytes_sent += bytes_for_this_chain;
    
    if(completion_element) {
      qh_urb_priv->completion_element = completion_element;
      completion_element->urb = urb;
      completion_element->pipe_type = usb_pipetype(urb->pipe);
      /* -- EM -- Using completion lock to also protect
         qh qtd list */
      spinlock_lock(&ehci_hcd->completion_lock);
      qh_append_qtds(ehci_hcd, urb, qh, &qtd_list);
      list_add_tail(&completion_element->chain_list, &ehci_hcd->completion_list);
      spinlock_unlock(&ehci_hcd->completion_lock);
      completion_element = NULL;
    }
    else {
      /* -- EM -- Using completion lock to also protect
         qh qtd list */
      //spinlock_lock(&ehci_hcd->completion_lock);
      qh_append_qtds(ehci_hcd, urb, qh, &qtd_list);
      //spinlock_unlock(&ehci_hcd->completion_lock);
    }
  }
  return bytes_sent;
}


static int ehci_rt_push_data(struct urb* urb, char* data, int count, uint interrupt_rate,
                             uint flags, void* completion_context)
{
  if(flags & URB_RT_PHYS_ADDR && (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS)) {
    DLOG("Isochronous does not support the URB_RT_PHYS_ADDR flag");
    panic("Isochronous does not support the URB_RT_PHYS_ADDR flag");
  }
  
  switch(usb_pipetype(urb->pipe)) {
  case PIPE_BULK:
  case PIPE_INTERRUPT:
    return ehci_rt_qh_push_data(urb, ehci_get_qh_urb_priv(urb),
                                data, count, interrupt_rate, flags, completion_context);

  case PIPE_ISOCHRONOUS:
    return ehci_rt_iso_push_data(urb, data, count, interrupt_rate, flags, completion_context);
  default: // case PIPE_CONTROL:
    return -1;
  }
}

static int ehci_rt_iso_update_packets(struct urb* urb, int max_packets)
{
  int i, j;
  ehci_iso_urb_priv_t* iso_urb_priv = get_iso_urb_priv(urb);
  itd_t** urb_itds = iso_urb_priv->itds;
  uint32_t urb_num_packets = urb->number_of_packets;
  //uint32_t microframe_offset = iso_urb_priv->ehci_urb_priv.sched_assignment & 0x7;
  uint32_t interval = urb->interval;
  uint32_t packets_per_itd = interval < 8 ? 8 / interval : 1;
  uint32_t packets_per_itd_mask = packets_per_itd - 1;
  uint32_t next_packet_to_make_available =
    iso_urb_priv->next_packet_to_make_available;
  //int next_packet_for_this_itd;


  insert_remaining_realtime_itds(hcd_to_ehci_hcd(urb->dev->hcd), urb);
  //DLOG("%s called", __FUNCTION__);
    
  for(i = 0; i < max_packets; ++i) {
    usb_iso_packet_descriptor_t* packet =
      &urb->iso_frame_desc[next_packet_to_make_available];
    itd_t* itd;
    int transaction_num;
    int transaction_index = -1;
    if(packet->data_available) {
      DLOG("Lost data!");
      //while(1);
      break;
    }
    
    itd = urb_itds[next_packet_to_make_available / packets_per_itd];

    
    
    transaction_num = next_packet_to_make_available & packets_per_itd_mask;
    
    for(j = 0; j < 8; ++j) {
      if(itd->transaction[j].raw > 0) {
        if(transaction_num == 0) {
          transaction_index = j;
          break;
        }
        else {
          transaction_num--;
        }
      }
    }

    EHCI_ASSERT(j != 8);
    EHCI_ASSERT(transaction_index >= 0);
        
    if(itd->transaction[transaction_index].raw & ITD_ACTIVE) {
      break;
    }

    packet->actual_length =
      (itd->transaction[transaction_index].raw >> 16) & 0xFFF;
    packet->status = (itd->transaction[transaction_index].raw >> 28) & 0xF;
    
    packet->data_available = TRUE;
    
    if((++next_packet_to_make_available) == urb_num_packets) {
      next_packet_to_make_available = 0;
    }
    
  }

  iso_urb_priv->next_packet_to_make_available = next_packet_to_make_available;
  return i;
}

static void handle_input_iso_rt_urb_completion(ehci_hcd_t* ehci_hcd,
                                               ehci_completion_element_t* comp_element)
{
  struct urb* urb = comp_element->urb;
  int i;
  uint urb_num_packets = urb->number_of_packets;
  ehci_iso_urb_priv_t* iso_urb_priv = get_iso_urb_priv(urb);
  uint num_transactions_with_ioc_set =
    ehci_hcd->frame_list_size / urb->interval / urb->interrupt_frame_rate;;
  uint interval = urb->interval;
  uint packets_per_itd = interval < 8 ? 8 / interval : 1;;
  uint packets_per_itd_mask = packets_per_itd - 1;
  uint next_transaction_to_check =
    iso_urb_priv->last_transaction_ioc_processed + urb->interrupt_frame_rate;
  uint call_counter = 0;
  
  if(next_transaction_to_check >= urb_num_packets) {
    next_transaction_to_check = 0;
  }
  
  for(i = 0; i <  num_transactions_with_ioc_set; ++i) {
    itd_t* itd = itd = iso_urb_priv->itds[next_transaction_to_check / packets_per_itd];
    uint transaction_num = next_transaction_to_check & packets_per_itd_mask;
    int j;
    uint transaction_index;

    for(j = 0; j < 8; ++j) {
      if(itd->transaction[j].raw > 0) {
        if(transaction_num == 0) {
          transaction_index = j;
          break;
        }
        else {
          transaction_num--;
        }
      }
    }

    if(itd->transaction[transaction_index].raw & ITD_ACTIVE ||
       (itd->ioc_processed_bitmap & (1 << transaction_index))) {
      break;
    }
    call_counter++;
    itd->ioc_processed_bitmap |= 1 << transaction_index;
    iso_urb_priv->last_transaction_ioc_processed = next_transaction_to_check;
    next_transaction_to_check += urb->interrupt_frame_rate;
    if(next_transaction_to_check >= urb_num_packets) {
      next_transaction_to_check = 0;
    }
  }

  while(call_counter--) {
    urb->complete(urb);
    if(urb->transfer_flags & URB_RT_ONE_HANDLER_CALL) {
      break;
    }
  }
}

static void handle_output_iso_rt_urb_completion(ehci_hcd_t* ehci_hcd,
                                                ehci_completion_element_t* comp_element)
{
  struct urb* urb = comp_element->urb;
  ehci_iso_urb_priv_t* iso_urb_priv = get_iso_urb_priv(urb);
  itd_t* current_itd;
  itd_t* temp_itd;
  bool done = FALSE;
  void* last_context = NULL;
  bool need_to_call = FALSE;
  
  list_for_each_entry_safe(current_itd, temp_itd, &iso_urb_priv->write_itds_to_free, chain_list) {
    int i;
    for(i = 0; i < 8; ++i) {
      if(current_itd->transaction[i].raw & ITD_ACTIVE) {
        done = TRUE;
        break;
      }
      else {
        if( (current_itd->transaction[i].raw & ITD_IOC) &&
            (!(current_itd->ioc_processed_bitmap & (1 << i))) ) {
          current_itd->ioc_processed_bitmap |= 1 << i;
          if(urb->transfer_flags & URB_RT_ONE_HANDLER_CALL) {
            need_to_call = TRUE;
            last_context = current_itd->completion_context;
          }
          else {
            urb->context = current_itd->completion_context;
            urb->complete(urb);
          }
        }
      }
    }
    if(done) break;
  }
  if(need_to_call) {
    urb->context = last_context;
    urb->complete(urb);
  }
}

static void handle_input_qh_rt_urb_completion(ehci_hcd_t* ehci_hcd,
                                              ehci_completion_element_t* comp_element)
{
  struct urb* urb = comp_element->urb;
  uint call_counter = 0;
  ehci_qh_urb_priv_t* qh_urb_priv = ehci_get_qh_urb_priv(urb);
  int i;
  int bytes_remaining_to_transfer;
  
  for(i = 0; i < qh_urb_priv->num_qtds; ++i) {
    qtd_t* current_qtd;
    current_qtd = qh_urb_priv->qtds[qh_urb_priv->next_qtd_for_ioc];
    bytes_remaining_to_transfer = (current_qtd->token >> 16) & 0x7FFF;
    
    if(bytes_remaining_to_transfer == 0 && !current_qtd->ioc_called ) {
    
      if(current_qtd->token & QTD_IOC) {
        current_qtd->ioc_called = TRUE;
        call_counter++;
      }
    }
    else {
      break;
    }
    qh_urb_priv->next_qtd_for_ioc++;
    
    if(qh_urb_priv->next_qtd_for_ioc >= qh_urb_priv->num_qtds) {
      qh_urb_priv->next_qtd_for_ioc = 0;
    }
  }
  //DLOG_UINT(call_counter);
  while(call_counter--) {
    urb->complete(urb);
    if(urb->transfer_flags & URB_RT_ONE_HANDLER_CALL) {
      break;
    }
  }
}

static void handle_output_qh_rt_urb_completion(ehci_hcd_t* ehci_hcd,
                                              ehci_completion_element_t* comp_element)
{
  struct urb* urb = comp_element->urb;
  ehci_qh_urb_priv_t* qh_urb_priv = ehci_get_qh_urb_priv(urb);
  qh_t* qh = qh_urb_priv->qh;
  qtd_t* current_qtd;
  qtd_t* temp_qtd;
  int bytes_remaining_to_transfer;
  void* last_context = NULL;
  bool need_to_call = FALSE;

#if 0
  list_for_each_entry_safe(current_qtd, temp_qtd, &qh->qtd_list, chain_list) {
    if(current_qtd->token & QTD_ACTIVE) {
      DLOG("Active rt output qtd");
    }
  }
#endif

  list_for_each_entry_safe(current_qtd, temp_qtd, &qh->qtd_list, chain_list) {
    bytes_remaining_to_transfer = (current_qtd->token >> 16) & 0x7FFF;


    if(bytes_remaining_to_transfer == 0) {
      if(!current_qtd->ioc_called && current_qtd->token & QTD_IOC ) {
        current_qtd->ioc_called = TRUE;
        if(urb->transfer_flags & URB_RT_ONE_HANDLER_CALL) {
          need_to_call = TRUE;
          last_context = current_qtd->completion_context;
        }
        else {
          urb->context = current_qtd->completion_context;
          urb->complete(urb);
        }
      }
    }
    else {
      break;
    }
  }
  
  if(need_to_call) {
    urb->context = last_context;
    urb->complete(urb);
  }
}

static void handle_rt_urb_completion(ehci_hcd_t* ehci_hcd, ehci_completion_element_t* comp_element)
{
  struct urb* urb = comp_element->urb;
  if(comp_element->pipe_type == PIPE_ISOCHRONOUS) {
    if(usb_pipein(urb->pipe)) {
      handle_input_iso_rt_urb_completion(ehci_hcd, comp_element);
    }
    else {
      handle_output_iso_rt_urb_completion(ehci_hcd, comp_element);
    }
  }
  else {
    if(usb_pipein(urb->pipe)) {
      handle_input_qh_rt_urb_completion(ehci_hcd, comp_element);
    }
    else {
      handle_output_qh_rt_urb_completion(ehci_hcd, comp_element);
    }
  }
}

static bool handle_non_rt_urb_completion(ehci_hcd_t* ehci_hcd,
                                         ehci_completion_element_t* comp_element)
{
  int i;
  struct urb* urb = comp_element->urb;
  bool done = FALSE;
  itd_t* itd;
  itd_t* temp_itd;
  int packets_traversed;
        
  if(comp_element->pipe_type == PIPE_ISOCHRONOUS) {
    itd = list_entry(comp_element->itds.prev, itd_t, chain_list);
    for(i = 0; i < 8; ++i) {
      if(itd->transaction[i].raw & ITD_ACTIVE) {
        break;
      }
    }
    done = i == 8;
  }
  else {
    ehci_qh_urb_priv_t* qh_urb_priv = ehci_get_qh_urb_priv(urb);
    done = TRUE;
    for(i = 0; i < qh_urb_priv->num_qtds; ++i) {
      if(QTD_REMAINING_DATA(qh_urb_priv->qtds[i])) {
        done = FALSE;
        break;
      }
    }
  }
      
  if(done) {
    urb->active = FALSE;
    if(urb->complete != NULL) {
      urb->complete(comp_element->urb);
    }
        
    /* Do URB bookkeeping and cleanup EHCI resources */
        
    if(comp_element->pipe_type == PIPE_ISOCHRONOUS) {
      packets_traversed = 0;
      list_for_each_entry(itd, &comp_element->itds, chain_list) {
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
            
      list_for_each_entry(itd, &comp_element->itds, chain_list) {
        unlink_itd_from_frame_list(ehci_hcd, itd);
      }
            
      list_for_each_entry_safe(itd, temp_itd, &comp_element->itds,
                               chain_list) {
        free_itd(ehci_hcd, itd);
      }
    }
    else {
      ehci_qh_urb_priv_t* qh_urb_priv = ehci_get_qh_urb_priv(urb);
      free_qtds(ehci_hcd, qh_urb_priv->qtds, qh_urb_priv->num_qtds);
    }
    return TRUE;
  }
  return FALSE;
}

static void ehci_check_for_urb_completions(ehci_hcd_t* ehci_hcd)
{
  ehci_completion_element_t* comp_element;
  ehci_completion_element_t* temp_comp_element;

  spinlock_lock(&ehci_hcd->completion_lock);
  
  list_for_each_entry_safe(comp_element, temp_comp_element,
                           &ehci_hcd->completion_list, chain_list) {
    if(comp_element->urb->active) {
      if(comp_element->urb->realtime) {
        if(comp_element->urb->complete) {
          /* It only makes sense to do things in this function if there is a
             completion callback otherwise just skip it */
          handle_rt_urb_completion(ehci_hcd, comp_element);
        }
      }
      else {
        if(handle_non_rt_urb_completion(ehci_hcd, comp_element)) {
          free_ehci_completion_element(comp_element);
        }
      }
    }
  }

  spinlock_unlock(&ehci_hcd->completion_lock);
}


void remap_iso_itd(struct urb* urb, uint32 frame_index, int new_assignment)
{
  int i;
  ehci_iso_urb_priv_t* iso_urb_priv = get_iso_urb_priv(urb);
  int old_assignment                = iso_urb_priv->ehci_urb_priv.sched_assignment;
  uint32_t interval                 = urb->interval;
  uint32_t num_itds = iso_urb_priv->num_itds;
  itd_t* itd = NULL;
  itd_t** urb_itds = iso_urb_priv->itds;

  if(new_assignment == old_assignment) { return; }

  if((old_assignment / 8) != (new_assignment / 8)) {
    /* -- EM -- Add this functionality later */
    DLOG ("We don't support rescheduling where the itd has to change position yet");
    panic("We don't support rescheduling where the itd has to change position yet");
  }

  //DLOG("urb interval = %d", urb->interval);
  //DLOG("num itds = %d", num_itds);

  /* -- EM -- Could make this more efficient by just doing the math to
     figure out the correct index but don't care right now */
  
  for(i = 0; (i < num_itds); ++i) {
    if(urb_itds[i]->frame_index == frame_index) {
      itd = urb_itds[i];
      break;
    }
  }

  if(itd == NULL) { return; }
  
  for(i = 0; i * interval < 8; ++i) {
    int transaction_new_assignment = (i * interval) + new_assignment;
    int transaction_old_assignment = (i * interval) + old_assignment;

    itd_transaction_t temp;
    temp = itd->transaction[transaction_old_assignment];
    itd->transaction[transaction_old_assignment] = itd->transaction[transaction_new_assignment];
    itd->transaction[transaction_new_assignment] = temp;

    temp = itd->transaction_backup[transaction_old_assignment];
    itd->transaction_backup[transaction_old_assignment] = itd->transaction_backup[transaction_new_assignment];
    itd->transaction_backup[transaction_new_assignment] = temp;

    
  }
}

void reorder_transactions(ehci_hcd_t* ehci_hcd)
{
  //static int call_counter = 0;
  int i,j;
  int num_rt_urbs = ehci_hcd->num_rt_urbs;
  struct urb** rt_urbs = ehci_hcd->rt_urbs;
  //frm_lst_lnk_ptr_t* frame_list     = ehci_hcd->frame_list;
  uint32_t frame_list_size          = ehci_hcd->frame_list_size;
  uint32_t frame_list_mask          = frame_list_size - 1;
  uint32_t current_frame_index;
  uint32_t frame_index;
  int* new_assignments = ehci_hcd->new_assignments;

  
  /* -- EM -- Overloading uninserted_itd_lock to also be used for
     reordering */


  if(ehci_hcd->reorder_urbs_stage == EHCI_RUS_NONE) {
    return;
  }
  


  spinlock_lock(&ehci_hcd->uninserted_itd_lock);

  
  
  current_frame_index = EHCI_CURRENT_FRAME_INDEX(ehci_hcd);
  frame_index = ((current_frame_index + EHCI_SAFE_FRAME_INSERT_OFFSET(ehci_hcd))) & frame_list_mask;

  
  
  while(frame_index != current_frame_index) {
    if(!ehci_hcd->reordered_map[frame_index]) {
      /* Don't have to reorder the last urb's element because it is
         the one that caused us to reorder everything so it is put
         into the right spot, also this function is called before the
         last urb's elements are put into the list */
      for(j = 0; j < num_rt_urbs - 1; ++j) { 
        if(usb_pipetype(rt_urbs[j]->pipe) == PIPE_ISOCHRONOUS) {
          remap_iso_itd(rt_urbs[j], frame_index, new_assignments[j]);
        }
      }
      ehci_hcd->reordered_map[frame_index] = TRUE;
    }
    
    frame_index = (frame_index + 1) & frame_list_mask;
  }

  for(i = 0; i < frame_list_size; ++i) {
    if(!ehci_hcd->reordered_map[i]) {
      break;
    }
  }
  
  if(i == frame_list_size) {
    //DLOG("Reorder done!");
    ehci_hcd->reorder_urbs_stage = EHCI_RUS_NONE;
    for(i = 0; i < num_rt_urbs; ++i) {
      get_ehci_urb_priv(rt_urbs[i])->sched_assignment = new_assignments[i];
    }
  }
  
  spinlock_unlock(&ehci_hcd->uninserted_itd_lock);
    
}

void insert_remaining_realtime_itds_for_urb(ehci_hcd_t* ehci_hcd, ehci_iso_urb_priv_t* iso_urb_priv)
{

  frm_lst_lnk_ptr_t* frame_list = ehci_hcd->frame_list;
  frm_lst_lnk_ptr_t** frame_list_virt = ehci_hcd->frame_list_virt;
  uint32_t frame_list_size = ehci_hcd->frame_list_size;
  uint32_t frame_list_mask = frame_list_size - 1;
  
  bool all_itds_added = TRUE;
  itd_t* itd;
  itd_t* itd_temp;
  bool looped_around;
  struct urb* urb = iso_urb_priv_to_urb(iso_urb_priv);
  uint32_t current_frame_index, start_frame_index;
  uint32_t frame_interval_offset = iso_urb_priv->ehci_urb_priv.sched_assignment / 8;
  uint32_t frame_interval        = urb->interval / 8;
    
  if(frame_interval <= 1) {
    frame_interval = 1;
    current_frame_index = EHCI_CURRENT_FRAME_INDEX(ehci_hcd);
    start_frame_index =
      current_frame_index + EHCI_SAFE_FRAME_INSERT_OFFSET(ehci_hcd);
  }
  else {
    current_frame_index = EHCI_CURRENT_FRAME_INDEX(ehci_hcd);
    start_frame_index =
      ((current_frame_index + EHCI_SAFE_FRAME_INSERT_OFFSET(ehci_hcd))
       | (frame_interval-1)) + 1 + frame_interval_offset;
  }
    
  start_frame_index &= frame_list_mask;
  looped_around = start_frame_index < current_frame_index;
  /*
   * If looped_around is TRUE we looped around by adding the safe
   * offset and interval offset
   */
    
  list_for_each_entry_safe(itd, itd_temp, &iso_urb_priv->uninserted_itds_list,
                           uninserted_list) {
    uint32_t frame_index = itd->frame_index;
    if(looped_around) {
      if( (frame_index >= start_frame_index) &&
          (frame_index < current_frame_index) ) {
        insert_itd_into_frame_list(ehci_hcd, frame_list, frame_list_virt, frame_index, itd);
          
        list_del(&itd->uninserted_list);
      }
      else {
        all_itds_added = FALSE;
      }
    }
    else {
      if( (frame_index >= start_frame_index  ) ||
          (frame_index <  current_frame_index) ) {
        insert_itd_into_frame_list(ehci_hcd, frame_list, frame_list_virt, frame_index, itd);
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

static void insert_remaining_realtime_itds(ehci_hcd_t* ehci_hcd, struct urb* urb)
{
  ehci_iso_urb_priv_t* iso_urb_priv;
  ehci_iso_urb_priv_t* temp_iso_urb_priv;
  ehci_iso_urb_priv_t* specific_iso_urb_priv = NULL;
  
  if(urb) {
    if(usb_pipetype(urb->pipe) != PIPE_ISOCHRONOUS) {
      return;
    }
    specific_iso_urb_priv = get_iso_urb_priv(urb);
    if(list_empty(&specific_iso_urb_priv->uninserted_itds_list)) {
      return;
    }
  }

  
  spinlock_lock(&ehci_hcd->uninserted_itd_lock);

  
  /* At this point we have the uninserted_itd_lock lock */


  if(specific_iso_urb_priv) {
    insert_remaining_realtime_itds_for_urb(ehci_hcd, specific_iso_urb_priv);
  }
  else {
    list_for_each_entry_safe(iso_urb_priv, temp_iso_urb_priv,
                             &ehci_hcd->uninserted_itd_urb_list,
                             uninserted_itd_urb_list) {
      
      insert_remaining_realtime_itds_for_urb(ehci_hcd, iso_urb_priv);
    }
  }

  
  spinlock_unlock(&ehci_hcd->uninserted_itd_lock);
  
}




static void ehci_irq_bottom_half(ehci_hcd_t* ehci_hcd)
{
  insert_remaining_realtime_itds(ehci_hcd, NULL);
  reorder_transactions(ehci_hcd);
  ehci_check_for_urb_completions(ehci_hcd);
}


#ifdef EHCI_USE_IOVCPU

static void ehci_bh_thread(ehci_hcd_t* ehci_hcd)
{
  while(1) {
    ehci_irq_bottom_half(ehci_hcd);
    iovcpu_job_completion();
  }
}


static void ehci_iovcpu_job_wakeup(ehci_hcd_t* ehci_hcd)
{
  /* -- EM -- hack: use VCPU2's period */
  iovcpu_job_wakeup (ehci_hcd->master_iovcpu, vcpu_lookup (2)->T);
}

#endif


#ifdef EHCI_IOC_BUG


static void ioc_backup_thread(ehci_hcd_t* ehci_hcd)
{
  
  while(1) {
    
    /*
     * Need to get the kernel lock since sched_usleep will call
     * schedule
     */
    /* 18750 - just a random number that works */
    sched_usleep(18750 * DEFAULT_INT_THRESHOLD );

#ifdef EHCI_USE_IOVCPU
    ehci_iovcpu_job_wakeup(ehci_hcd);
#else
    ehci_irq_bottom_half(ehci_hcd);
#endif
  }
}

#endif // EHCI_IOC_BUG

static uint32_t
ehci_irq_handler(uint8 vec)
{
  uint32_t status;
  lock_kernel();

  if(vec != ehci_hcd.handler_vector) {
    return 0;
  }
  
  status = ehci_hcd.regs->status;

  EHCI_ACK_INTERRUPTS(&ehci_hcd, status & USBINTR_ALL);

  if(status & USBINTR_INT) { /* "normal" completion (short, ...) */ }
  
  if(status & USBINTR_IAA) { /* Interrupted on async advance */ }

  if(status & USBINTR_HSE) { /* such as some PCI access errors */
    DLOG("USBINTR_HSE case in %s unhandled status = 0x%X", __FUNCTION__, status);
    print_caps_and_regs_info(&ehci_hcd, "In IRQ HSE");
    print_periodic_list(&ehci_hcd, FALSE, FALSE, TRUE);
    panic("USBINTR_HSE case unhandled");
  }
  
  if(status & USBINTR_FLR) { /* frame list rolled over */ }

  if(status & USBINTR_PCD) { /* port change detect */ }

  if(status & USBINTR_ERR) { /* "error" completion (overflow, ...) */
    DLOG("USBINTR_ERR case in %s unhandled status = 0x%X", __FUNCTION__, status);
    panic("USBINTR_ERR case unhandled");
  }

#ifdef EHCI_USE_IOVCPU
  ehci_iovcpu_job_wakeup(&ehci_hcd);
#else
  ehci_irq_bottom_half(&ehci_hcd);
#endif
    
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

  ehci_hcd->frame_list_virt = kmalloc(frm_lst_size * sizeof(frm_lst_lnk_ptr_t*));
  if(ehci_hcd->frame_list_virt == NULL) return FALSE;
  memset(ehci_hcd->frame_list_virt, 0, frm_lst_size * sizeof(frm_lst_lnk_ptr_t*));
  while(frm_lst_size--) {
    frame_list[frm_lst_size].raw = EHCI_LIST_END;
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
  async_head->alt_qtd_ptr_raw = async_head->dummy_qtd->dma_addr;
  async_head->state = QH_STATE_LINKED;
  async_head->next_virt = (frm_lst_lnk_ptr_t*)async_head;
  
  ehci_hcd->regs->async_next = async_head->dma_addr;

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

static void unlink_all_qhs_from_async(ehci_hcd_t* ehci_hcd)
{
  qh_t* async_head = ehci_hcd->async_head;
  uint32_t async_head_hw = QH_NEXT(ehci_hcd, async_head);
  qh_t* current_qh = async_head;
  qh_t* next_qh;
  
  do {
    next_qh = (qh_t*)current_qh->next_virt;
    current_qh->horizontalPointer.raw = async_head_hw;
    current_qh->next_virt = (frm_lst_lnk_ptr_t*)async_head;
    /*
     * -- EM -- Should really free the qh but that would involve going
     * into the device struct and release the qh there as well so just
     * mark it as not linked for now
     */
    current_qh->state = QH_STATE_NOT_LINKED;
    current_qh = next_qh;
  } while(next_qh != async_head);
}

bool ehci_post_enumeration(usb_hcd_t* usb_hcd)
{  
  ehci_hcd_t* ehci_hcd = hcd_to_ehci_hcd(usb_hcd);
  enable_interrupts(ehci_hcd);
  unlink_all_qhs_from_async(ehci_hcd);
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
  uint32_t pipe_type;

  if(ehci_hcd->reorder_urbs_stage != EHCI_RUS_NONE) {
    reorder_transactions(ehci_hcd);
    if(ehci_hcd->reorder_urbs_stage != EHCI_RUS_NONE) {
      DLOGV("reordering transactions cannot submit urb, returning -2");
      return -2;
    }
  }
  
  if(packet_len == 0) {
    DLOGV("packet_len is zero, returning -1");
    return -1;
  }
  
  pipe_type = usb_pipetype(pipe);

  if(!(urb->transfer_flags & URB_RT_ZERO_COPY_WRITE)) {
    if(urb->transfer_buffer) {
      urb->transfer_dma = (phys_addr_t)get_phys_addr(urb->transfer_buffer);
    }
    else {
      if(usb_pipetype(urb->pipe) != PIPE_CONTROL) {
        /* -- EM -- Right now must use transfer_buffer can't use transfer dma */
        return -1;
      }
    }
  }
  
  switch(pipe_type) {
  case PIPE_ISOCHRONOUS:
    return ehci_isochronous_transfer(ehci_hcd, urb);
  case PIPE_INTERRUPT:
    return ehci_interrupt_transfer(ehci_hcd, urb);
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
 * -- EM -- Need to actually implement this function just a dummy right now
 */
int ehci_rt_iso_data_lost(struct urb* urb)
{
  insert_remaining_realtime_itds(hcd_to_ehci_hcd(urb->dev->hcd), urb);
  DLOG("ehci_rt_iso_data_lost is not implemented");
  panic("ehci_rt_iso_data_lost is not implemented");
  return 0;
}


int ehci_rt_iso_free_packets(struct urb* urb, int number_of_packets)
{
  int i, j;
  ehci_iso_urb_priv_t* iso_urb_priv = get_iso_urb_priv(urb);
  uint32_t next_packet_to_free = iso_urb_priv->next_packet_to_free;
  uint32_t urb_num_packets = urb->number_of_packets;
  //uint32_t microframe_offset = iso_urb_priv->ehci_urb_priv.sched_assignment & 0x7;
  uint32_t interval = urb->interval;
  itd_t** urb_itds = iso_urb_priv->itds;
  uint32_t packets_per_itd = interval < 8 ? 8 / interval : 1;
  uint32_t packets_per_itd_mask = packets_per_itd - 1;

  insert_remaining_realtime_itds(hcd_to_ehci_hcd(urb->dev->hcd), urb);

  for(i = 0; i < number_of_packets; ++i) {
    itd_t* itd;
    int transaction_num;
    int transaction_index = -1;
    usb_iso_packet_descriptor_t* packet = &urb->iso_frame_desc[next_packet_to_free];
    
    if(!packet->data_available) {
      break;
    }
    
    itd = urb_itds[next_packet_to_free / packets_per_itd];
    
    transaction_num = next_packet_to_free & packets_per_itd_mask;
    for(j = 0; j < 8; ++j) {
      if(itd->transaction[j].raw > 0) {
        if(transaction_num == 0) {
          transaction_index = j;
          break;
        }
        else {
          transaction_num--;
        }
      }
    }

    
    EHCI_ASSERT(j != 8);
    EHCI_ASSERT(transaction_index >= 0);

    
    EHCI_ASSERT(!(itd->transaction[transaction_index].raw & ITD_ACTIVE));
    
    itd->transaction[transaction_index].raw =
      itd->transaction_backup[transaction_index].raw;
    itd->ioc_processed_bitmap &= ~(1 << transaction_index); /* Reset as interrupt not processed */

    if(!(itd->transaction[transaction_index].raw & ITD_ACTIVE)) {
      /* This occurs when we have remapped stuff, the solution is to
         scan the backups for the correct transaction_num in backup
         and then copy that backup */

      transaction_num = next_packet_to_free & packets_per_itd_mask;
      
      for(j = 0; j < 8; ++j) {
        if(itd->transaction_backup[j].raw > 0) {
          if(transaction_num == 0) {
            transaction_index = j;
            break;
          }
          else {
            transaction_num--;
          }
        }
      }
      
      EHCI_ASSERT(j != 8);
      EHCI_ASSERT(transaction_index >= 0);
      EHCI_ASSERT(!(itd->transaction[transaction_index].raw & ITD_ACTIVE));
    
      itd->transaction[transaction_index].raw =
        itd->transaction_backup[transaction_index].raw;

      /* Reset as interrupt not processed, not sure if necessary here
         because this is in the remap and the new ioc bit shouldn't
         have been set but setting it right now anyway */
      itd->ioc_processed_bitmap &= ~(1 << transaction_index); 

      if(!(itd->transaction[transaction_index].raw & ITD_ACTIVE)) {
        /* Should never get here */
        DLOG ("Failed to set transaction as active");
        panic("Failed to set transaction as active");
      }
    }
    

    packet->data_available = FALSE;
    ++next_packet_to_free;
    if(next_packet_to_free == urb_num_packets) {
      next_packet_to_free = 0;
    }
  }

  iso_urb_priv->next_packet_to_free = next_packet_to_free;

  return i;
}


static bool initialise_ehci_dma_pools(ehci_hcd_t* ehci_hcd)
{
  ehci_hcd->qh_pool = ehci_hcd->qtd_pool = ehci_hcd->itd_pool = NULL;

  ehci_hcd->qh_pool = dma_pool_create("ehci-qh", sizeof(qh_t), 32, 0x1000);
  if(ehci_hcd->qh_pool == NULL) goto dma_pool_failure;

  ehci_hcd->qtd_pool = dma_pool_create("ehci-qtd", sizeof(qtd_t), 32, 0x1000);
  if(ehci_hcd->qtd_pool == NULL) goto dma_pool_failure;

  ehci_hcd->itd_pool = dma_pool_create("ehci-itd", sizeof(itd_t), 32, 0x1000);
  if(ehci_hcd->itd_pool == NULL) goto dma_pool_failure;
  

  return TRUE;
 dma_pool_failure:

  if(ehci_hcd->qh_pool) dma_pool_destroy(ehci_hcd->qh_pool);
  if(ehci_hcd->qtd_pool) dma_pool_destroy(ehci_hcd->qtd_pool);
  if(ehci_hcd->itd_pool) dma_pool_destroy(ehci_hcd->itd_pool);
  return FALSE;
}


/*
 * Host controller initiailization sequence taken from page 53 of the
 * EHCI Specs
 */

/*
 * -- EM -- Move this into ehci_hcd to get more than 1 ehci chip to work!!!!
 */

#ifdef EHCI_IOC_BUG
static u32 temp_stack[1024] ALIGNED (0x1000);
#endif


static bool
initialise_ehci_hcd(uint32_t usb_base,
                    ehci_hcd_t* ehci_hcd,
                    frm_lst_lnk_ptr_t* frm_lst,
                    frm_lst_lnk_ptr_t* last_frm_lst_entries,
                    uint32_t* micro_frame_remaining_time_periodic,
                    uint32_t* micro_frame_remaining_time_async,
                    uint32_t frm_lst_size,
                    uint32_t int_threshold)
{
  uint32_t i;
  if(!initialise_usb_hcd(ehci_hcd_to_hcd(ehci_hcd),
                         USB_TYPE_HC_EHCI, ehci_reset_root_ports,
                         ehci_post_enumeration,
                         ehci_submit_urb,
                         ehci_kill_urb,
                         ehci_rt_iso_update_packets,
                         ehci_rt_iso_free_packets,
                         ehci_rt_data_lost,
                         ehci_rt_update_data,
                         ehci_rt_free_data,
                         ehci_rt_push_data,
                         ehci_rt_free_write_resources)) {
    return FALSE;
  }
  
  ehci_hcd->base_physical_address = (phys_addr_t)usb_base;
  ehci_hcd->base_virtual_address  = map_virtual_page(usb_base | 0x3);
  ehci_hcd->caps                  = ehci_hcd->base_virtual_address;
  ehci_hcd->regs                  = (ehci_regs_t*)(ehci_hcd->base_virtual_address
                                                   + ehci_hcd->caps->cap_length);

  ehci_hcd->reorder_urbs_stage = EHCI_RUS_NONE;
  
  memset(ehci_hcd->ehci_devinfo, 0, sizeof(ehci_hcd->ehci_devinfo));
  
  ehci_hcd->frame_list                  = frm_lst;
  ehci_hcd->last_frame_list_entries     = last_frm_lst_entries;
  ehci_hcd->frame_list_size             = frm_lst_size;
  ehci_hcd->micro_frame_remaining_time_periodic  = micro_frame_remaining_time_periodic;
#ifdef USB_REALTIME_ASYNC
  ehci_hcd->micro_frame_remaining_time_async  = micro_frame_remaining_time_async;
#endif
  ehci_hcd->num_rt_urbs = 0;
  
  ehci_hcd->zero_qh_hub_addr = -1;
  ehci_hcd->zero_qh_port_num = -1;
  
  INIT_LIST_HEAD(&ehci_hcd->uninserted_itd_urb_list);
  
  for(i = 0; i < frm_lst_size * 8; ++i) {
    micro_frame_remaining_time_periodic[i] = USEC_PER_MICRO_FRAME_FOR_PERIODIC;
  }

#ifdef USB_REALTIME_ASYNC
  for(i = 0; i < frm_lst_size * 8; ++i) {
    micro_frame_remaining_time_async[i] = USEC_PER_MICRO_FRAME_FOR_ASYNC;
  }
#endif

  DLOG("ehci_hcd->completion_lock = 0x%p", &ehci_hcd->completion_lock);
  spinlock_init(&ehci_hcd->completion_lock);
  spinlock_init(&ehci_hcd->uninserted_itd_lock);

  if(!initialise_ehci_dma_pools(ehci_hcd)) return FALSE;
  
  ehci_hcd->interrupt_threshold = int_threshold;
  ehci_hcd->num_ports           = GET_NUM_PORTS(ehci_hcd);

  /*
   * Set used_qh_bitmap and used_qtd_bitmap to all zeros to
   * mark all elements as free
   */

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


#ifdef EHCI_USE_IOVCPU
  ehci_hcd->master_iovcpu =
    create_kernel_thread_args ((u32) ehci_bh_thread,
                               (u32) &ehci_hcd->bh_stack[EHCI_IOC_BH_THREAD_STACK_SIZE-1],
                               "EHCI Bottom Half", FALSE, 1, ehci_hcd);
  set_iovcpu (ehci_hcd->master_iovcpu, IOVCPU_CLASS_USB);
#endif

#ifdef EHCI_IOC_BUG
  
  create_kernel_thread_args((u32)ioc_backup_thread,
    (u32) &temp_stack[1023],
    "EHCI Backup",
    TRUE, 1, ehci_hcd);
  
#endif // EHCI_IOC_BUG
  
  return TRUE;
}


/* Called to initialise EHCI, functionality mimics uhci_init in
   uhci_hcd */


bool ehci_init(void)
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
   
  
  /* -- WARN -- Only looking for 1 specific EHCI host controller
     device this is D29 for Intel 6 C200 or the qemu ehci chip would
     be best to add all EHCI chips to an array that is iterated, and
     each time one is found it is pushed to the usb core */

  while (pci_find_device (0x8086, 0x24CD, 0x0C, 0x03, i, &i)) { 
    if (pci_get_device (i, &ehci_device)) { 
      if (ehci_device.progIF == 0x20) {
        device_index = i;
        break;
      }
      i++;
    } else break;
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
   
  if(device_index == ~0) {
    while (pci_find_device (0x8086, 0x1C2D, 0x0C, 0x03, i, &i)) {
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
                          micro_frame_remaining_time_periodic,
                          micro_frame_remaining_time_async,
                          sizeof(frame_list)/sizeof(frm_lst_lnk_ptr_t),
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
  qtd->original_total_bytes_to_transfer = count;
  qtd->token_backup = qtd->token;
  qtd->buffer_page_zero_backup = qtd->buffer_page[0];
  return count;
}

static bool
create_qtd_chain(ehci_hcd_t* ehci_hcd,
                 struct urb* urb, addr_t setup_req,
                 uint32_t setup_len, addr_t data,
                 uint32_t data_len, bool enable_ioc,
                 list_head_t* qtd_list, /* Should be empty/uninitialized */
                 uint interrupt_byte_rate,
                 bool phys_data_addr, void* completion_context)
{
  uint32_t    token;
  qtd_t*      current_qtd;
  qtd_t*      previous_qtd;
  phys_addr_t data_phys_addr;
  qtd_t* safe_qtd;

  unsigned int pipe       = urb->pipe;
  bool         is_input   = usb_pipein(pipe);
  uint8_t      pipe_type  = usb_pipetype(pipe);
  int          packet_len = usb_maxpacket(urb->dev, urb->pipe);
  uint         bytes_since_last_ioc = 0;

  token = QTD_ACTIVE | (EHCI_TUNE_CERR << QTD_CERR);
  INIT_LIST_HEAD(qtd_list);

  if( (data_len == 0) && (( (setup_len == 0) || (pipe_type != PIPE_CONTROL) )) ) {
    DLOG("Don't add any qtds to chain");
    return TRUE;
  }
  
  current_qtd = allocate_qtd(ehci_hcd);

  
  
  if(current_qtd == NULL) return FALSE;
  
  list_add_tail(&current_qtd->chain_list, qtd_list);
  
  if(pipe_type == PIPE_CONTROL) {
    if(qtd_fill(current_qtd, (phys_addr_t)get_phys_addr(setup_req),
                setup_len, packet_len, token | QTD_SETUP) != setup_len) {
      goto cleanup;
    }
    
    token ^= QTD_TOGGLE;

    previous_qtd = current_qtd;
    current_qtd = allocate_qtd(ehci_hcd);
    if(!current_qtd) goto cleanup;

    list_add_tail(&current_qtd->chain_list, qtd_list);

    if(data_len == 0) {
      token |= QTD_INPUT;
    }
    previous_qtd->next_pointer_raw = current_qtd->dma_addr;
  }

  /* If zero length data stage then data is input or if request type
     specifies that data is input */
  if(is_input) {
    token |= QTD_INPUT;
  }

  data_phys_addr = (phys_addr_t)(phys_data_addr ? data : get_phys_addr(data));
  
  while(1) {
    uint max_bytes_for_this_qtd = data_len;
    if(interrupt_byte_rate != 0 && interrupt_byte_rate < max_bytes_for_this_qtd) {
      max_bytes_for_this_qtd = interrupt_byte_rate;
    }
    int this_qtd_length
      = qtd_fill(current_qtd, data_phys_addr, max_bytes_for_this_qtd,
                 packet_len, token);
    data_phys_addr += this_qtd_length;
    bytes_since_last_ioc += this_qtd_length;
    if(bytes_since_last_ioc >= interrupt_byte_rate && interrupt_byte_rate != 0) {
      bytes_since_last_ioc = 0;
      QTD_ENABLE_IOC(current_qtd);
      current_qtd->completion_context = completion_context;
    }

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
    if(!current_qtd) goto cleanup;
    
    list_add_tail(&current_qtd->chain_list, qtd_list);

    previous_qtd->next_pointer_raw = current_qtd->dma_addr;
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
      if(!current_qtd) goto cleanup;
      
      list_add_tail(&current_qtd->chain_list, qtd_list);
      
      previous_qtd->next_pointer_raw = current_qtd->dma_addr;

      qtd_fill(current_qtd, 0, 0, 0, token);
      
    }
  }

  if(enable_ioc && !urb->realtime) {
    /* Interrupt on complete */
    
    QTD_ENABLE_IOC(current_qtd);
    current_qtd->completion_context = completion_context;
  }
  
  return TRUE;
  
 cleanup:
  list_for_each_entry_safe(current_qtd, safe_qtd, qtd_list, chain_list) {
    free_qtd(ehci_hcd, current_qtd);
  }
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
    if(qh->current_qtd_ptr_raw == qtd->dma_addr) {
      qtd = NULL;
    }
  }
  
  if (qtd) {
    if(qh->state == QH_STATE_LINKED) {
      DLOG("Should never reached here if qh->state == QH_STATE_LINKED");
      panic("Should never reached here if qh->state == QH_STATE_LINKED");
    }
    qh->next_qtd_ptr_raw = qtd->dma_addr;
    qh->alt_qtd_ptr_raw  = EHCI_LIST_END;
    
    if(!(qh->hw_info1 & QH_DATA_TOGGLE_CONTROL)) {
      qh->data_toggle =
        IS_USB_ENDPOINT_TOGGLED(qh->dev, QH_GET_ENDPOINT(qh), QTD_IS_INPUT(qtd));
    }
  }
  
  qh->qtd_token_raw &= (QTD_TOGGLE | QTD_STS_PING);
}

static inline uint16_t create_int_sched_mask(struct urb* urb)
{
  uint16_t mask = 0;
  int i;
  uint32_t interval_offset = get_int_urb_priv(urb)->ehci_urb_priv.sched_assignment;

  for(i = interval_offset; i < 8; i += urb->interval) {
    mask |= 1 << i;
  }

  return mask;
}

static void qh_prep(ehci_hcd_t* ehci_hcd, struct urb* urb, qh_t* qh, uint32_t endpoint,
                    uint32_t device_addr, uint32_t max_packet_len,
                    uint32_t dev_speed, uint32_t pipe_type, bool is_input)
{
  uint32_t info1 = 0;
  uint32_t info2 = 0;

  if(device_addr != 0) {
    usb_hcd_t* usb_hcd = ehci_hcd_to_hcd(ehci_hcd);
    qh->dev = &usb_hcd->devinfo[device_addr];
  }

  qh->interval = urb->interval;

  info1  = device_addr;
  info1 |= endpoint << 8;

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

    /* Some Freescale processors have start the numbering from 0
       instead of 1, if we every test a processor with this bug we
       should use port_num - 1 instead of port_num. */
    
    info2 |= urb->dev->port_num << 23;
    
    /* set the address of the TT; for TDI's integrated
     * root hub tt, leave it zeroed.
     */
    if(urb->dev->hub_addr) {
      info2 |= urb->dev->hub_addr << 16;
    }

    if(pipe_type == PIPE_INTERRUPT) {
      info2 |= 0x0401;
    }
    
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

      info1 |= max_packet(max_packet_len) << 16; /* Set max packet length */

      info2 |= (EHCI_TUNE_MULT_HS << 30); /* set transactions per
                                             microframe */
    }
    else if (pipe_type == PIPE_INTERRUPT) {
      info1 |= max_packet(max_packet_len) << 16;
      info2 |= hb_mult(max_packet_len) << 30;
      info2 |= create_int_sched_mask(urb) & 0xFF;
    }
    break;
  default:
    /*
     * -- EM -- Should fail more gracefully here but don't care right
     * now
     */
    DLOG("Unknown device speed, see file %s line %d", __FILE__, __LINE__);
    panic("Unknown device speed");
  }

  qh->hw_info1 = info1;
  qh->hw_info2 = info2;
  
  qh_refresh(ehci_hcd, qh);
}

/*
 * -- EM -- The urb private data list of qtds might have to be
 * protected via a lock because qh_append_qtds changes it.  It doesn't
 * now because the kernel is non-preemptive
 */
static void qh_append_qtds(ehci_hcd_t* ehci_hcd, struct urb* urb,
                           qh_t* qh, list_head_t* qtd_list)
{
  
  qtd_t* qtd;
  qtd_t* dummy;
  uint32_t token;
  ehci_qh_urb_priv_t* qh_urb_priv = NULL;
  int urb_swap_index;
  qtd_t** urb_qtd_list = NULL;
  int i, num_qtds;
  phys_addr_t dma;

  urb_qh_compatiblity_check(urb, qh);
  
  if(list_empty(qtd_list)) return;

  qtd = list_entry(qtd_list->next, qtd_t, chain_list);


  if(urb->hcpriv != NULL) {
    switch(usb_pipetype(urb->pipe)) {
    case PIPE_BULK:
      qh_urb_priv = &(get_bulk_urb_priv(urb))->qh_urb_priv;
      break;
      
    case PIPE_INTERRUPT:
      qh_urb_priv = &(get_int_urb_priv(urb))->qh_urb_priv;
      break;
      
    case PIPE_CONTROL:
    qh_urb_priv = &(get_ctrl_urb_priv(urb))->qh_urb_priv;
    break;
    }
    
    if(qh_urb_priv != NULL && qh_urb_priv->num_qtds > 0) {
      num_qtds = qh_urb_priv->num_qtds;
      urb_qtd_list = qh_urb_priv->qtds;
      urb_swap_index = -1;
      for(i = 0; i < num_qtds; ++i) {
        if(urb_qtd_list[i] == qtd) {
          urb_swap_index = i;
        break;
        }
      }
    
    
      if(urb_swap_index < 0) {
        DLOG("urb_swap_index = %d in %s this should never happen",
             urb_swap_index, __FUNCTION__);
        panic("urb_swap_index < 0 in qh_append_qtds this should never happen");
      }
    }
  }

  token = qtd->token;
  qtd->token = QTD_HALT;
  gccmb();

  dummy = qh->dummy_qtd;
  dma = dummy->dma_addr;
  *dummy = *qtd;
  dummy->dma_addr = dma;

  if(urb_qtd_list) {
    urb_qtd_list[urb_swap_index] = dummy;
  }
  
  list_del(&qtd->chain_list);
  list_add(&dummy->chain_list, qtd_list);
  list_splice_tail(qtd_list, &qh->qtd_list);
  initialise_qtd(qtd, qtd->dma_addr);
  qh->dummy_qtd = qtd;
  dma = qtd->dma_addr;
  qtd = list_entry(qh->qtd_list.prev, qtd_t, chain_list);
  qtd->next_pointer_raw = dma;
  gccmb();
  dummy->token = token;
}

static void qh_append_qtd(ehci_hcd_t* ehci_hcd, struct urb* urb,
                          qh_t* qh, qtd_t* qtd)
{
  list_head_t temp_list;
  INIT_LIST_HEAD(&temp_list);
  list_move_tail(&qtd->chain_list, &temp_list);
  qh_append_qtds(ehci_hcd, urb, qh, &temp_list);
}

/*
 * Simplified version of UnlinkQueueHead on page 72 of EHCI
 * specifications because we are only removing one queue head
 */
static inline void unlink_async_queue_head(ehci_hcd_t* ehci_hcd, qh_t* head_to_unlink)
{
  qh_t* previous  = (qh_t*)head_to_unlink->previous; 
  qh_t* next = (qh_t*)head_to_unlink->next_virt;
  
  next->previous = head_to_unlink->previous;
  previous->horizontalPointer = head_to_unlink->horizontalPointer;
  previous->next_virt = head_to_unlink->next_virt;
  
  /* Add qh to reclaim list */
  head_to_unlink->state = QH_STATE_RECLAIM;
  list_add(&head_to_unlink->reclaim_chain, &ehci_hcd->reclaim_list);
}

void link_qh_to_async(ehci_hcd_t* ehci_hcd, qh_t* qh)
{
  qh_t* next = (qh_t*)ehci_hcd->async_head->next_virt;
  
  qh_refresh(ehci_hcd, qh);
  qh->previous = next->previous;
  next->previous = &qh->horizontalPointer;
  qh->state = QH_STATE_LINKED;
  qh->horizontalPointer = ehci_hcd->async_head->horizontalPointer;
  qh->next_virt = ehci_hcd->async_head->next_virt;
  qh->previous = &ehci_hcd->async_head->horizontalPointer;
  gccmb();
  ehci_hcd->async_head->horizontalPointer.raw = QH_NEXT(ehci_hcd, qh);
  ehci_hcd->async_head->next_virt = (frm_lst_lnk_ptr_t*)qh;
}

static sint32 spin_for_transfer_completion(ehci_hcd_t* ehci_hcd,
                                           struct urb* urb,
                                           qh_t* qh, uint8_t address,
                                           uint32_t endpoint,
                                           uint8_t pipe_type, bool is_input,
                                           int timeout)
{
  int i;
  qtd_t* last_qtd;
  qtd_t* temp_qtd;
  qtd_t* qtd_to_remove;
  qtd_t* qtd_tmp;
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
   * 1) Check if the iaa interrupt is on, if it is return an error
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
  if( (pipe_type != PIPE_INTERRUPT) && EHCI_INTR_ENABLED(ehci_hcd, USBINTR_IAA) ) {
    if(EHCI_INTR_ENABLED(ehci_hcd, USBINTR_IAA)) { DLOG("EHCI_INTR_ENABLED(ehci_hcd, USBINTR_IAA) is true"); }
    DLOG("AT %d", __LINE__);
    //return -1;
  }

  /*
   * -- EM -- If an error occurs and it isn't on the last qtd then
   * this will spin forever -> very bad also this is a hack right
   * now to figure out if a td is never going to end would be better
   * to check various things for errors
   */
  /* Step 2 */

  for(i = 0; (last_qtd->token & QTD_ACTIVE) && (i < (timeout / USB_MSG_SLEEP_INTERVAL) + 1 ); ++i) {
    sched_usleep(4000 * USB_MSG_SLEEP_INTERVAL);
  }

  if(last_qtd->token & QTD_ACTIVE) {
    DLOG("At %d", __LINE__);
    return -1;
  }
  
  if(pipe_type == PIPE_BULK) {
    USB_DEVICE_INFO* dev_info = &ehci_hcd_to_hcd(ehci_hcd)->devinfo[address];
    SET_USB_ENDPOINT_TOGGLE(dev_info, endpoint, is_input, qh->data_toggle );
  }

  if(last_qtd->token & QTD_HALT) {
    print_caps_and_regs_info(ehci_hcd, "in failure");
    print_qh_info(ehci_hcd, qh, TRUE, "in failure");
    DLOG("pci status in failure = 0x%X",
         pci_get_status(ehci_hcd->bus, ehci_hcd->dev,ehci_hcd->func));
    DLOG("pci command in failure = 0x%X",
         pci_get_command(ehci_hcd->bus, ehci_hcd->dev,ehci_hcd->func));

    /*-- EM -- Need a better cleanup here */
    DLOG("%s failed at line %d", __FUNCTION__, __LINE__);
    panic("spin_for_transfer_completion failed");
  }

  list_for_each_entry(temp_qtd, &qh->qtd_list, chain_list) {
    if(temp_qtd->total_bytes_to_transfer != 0) {
      
      DLOG("Failed to transfer everything");
      DLOG("%d bytes left", temp_qtd->total_bytes_to_transfer);
      panic("Failed to transfer everything");
    }
  }


  
  list_for_each_entry_safe(qtd_to_remove, qtd_tmp, &qh->qtd_list, chain_list) {
    if(qtd_to_remove->dma_addr == qh->current_qtd_ptr_raw) {
      break;
    }
    else {
      free_qtd(ehci_hcd, qtd_to_remove);
    }
  }
  
  if(list_empty(&qh->qtd_list)) {
    /*
     * -- EM -- ALL QTDS REMOVED this can happen because a silicon
     * quirk with some ehci host controllers moving the dummy qtd as
     * the active one, I have not tested the functionally (since I
     * haven't found a chip that does this) when this occurs so all
     * bets are off
     */
    DLOG("ALL QTDS REMOVED see comment at file %s line %d", __FILE__, __LINE__);
    panic("ALL QTDS REMOVED");
  }
  
  
  
  if(pci_get_status(ehci_hcd->bus, ehci_hcd->dev,ehci_hcd->func) & 0x2000) {
    DLOG("EHCI PCI master abort: function %s line %d", __FUNCTION__, __LINE__);
    panic("PCI master abort");
  }

  /*
   * -- EM -- Right now for the spin for completion transfers the
   * transfer is only successful if it completely finishes, otherwise
   * it fails.  When this is fixed and the panic is removed we will
   * have to read how many bytes where left in the tds, until now just
   * doing this so calling devices drivers work correctly
   */
  urb->actual_length = urb->transfer_buffer_length;
  
  return 0;
}

static int submit_async_qtd_chain(ehci_hcd_t* ehci_hcd, struct urb* urb, qh_t** qh,
                                  uint32_t endpoint, uint32_t device_addr,
                                  uint32_t dev_speed, uint32_t pipe_type,
                                  uint32_t max_packet_len, bool is_input,
                                  bool ioc_enabled, list_head_t* qtd_list)
{
  int result;
  if(*qh == NULL) {
    *qh = allocate_qh(ehci_hcd);
    qh_prep(ehci_hcd, urb, *qh, endpoint, device_addr,
            max_packet_len, dev_speed, pipe_type, is_input);
  }


  
  
  EHCI_SET_DEVICE_QH(ehci_hcd, device_addr, is_input, endpoint, *qh);
  
  if(get_bulk_urb_priv(urb)) {
    get_bulk_urb_priv(urb)->qh_urb_priv.qh = *qh;

  }
  qh_append_qtds(ehci_hcd, urb, *qh, qtd_list);
  if((*qh)->state == QH_STATE_NOT_LINKED) {
    link_qh_to_async(ehci_hcd, *qh);
  }
  if(ioc_enabled) {
    return 0;
  }
  else {
    
    if((result = spin_for_transfer_completion(ehci_hcd, urb, *qh, device_addr,
                                              endpoint, pipe_type, is_input, urb->timeout)) < 0) {

      DLOG("spin_for_transfer_completion returned %d", result);
      return result;
    }
  }
  if(urb->complete) urb->complete(urb);
  return 0;
}


void link_qh_to_periodic_list(ehci_hcd_t* ehci_hcd, qh_t* qh, struct urb* urb)
{
  qh_t* temp_qh;
  itd_t* temp_itd;
  int i;
  unsigned int temp_frame_interval;
  ehci_int_urb_priv_t* int_urb_priv  = get_int_urb_priv(urb);
  frm_lst_lnk_ptr_t* frame_list      = ehci_hcd->frame_list;
  frm_lst_lnk_ptr_t** frame_list_virt = ehci_hcd->frame_list_virt;
  uint32_t frame_list_size           = ehci_hcd->frame_list_size;
  uint32_t frame_interval            = urb->interval / 8;
  uint32_t frame_interval_offset     = int_urb_priv->ehci_urb_priv.sched_assignment / 8;

  if(frame_interval == 0) {
    frame_interval = 1;
  }

  if(urb->realtime) {
    for(i = frame_interval_offset; i < frame_list_size; i += frame_interval) {
      bool not_inserted = TRUE;
      frm_lst_lnk_ptr_t* current = &frame_list[i];
      frm_lst_lnk_ptr_t** current_virt = &frame_list_virt[i];
      while(not_inserted) {
        /*
         * If the tBit is 1 then we have reached the end of the list
         */
        if(current->tBit) {
          insert_qh_into_frame_list(current, current_virt, qh);
          not_inserted = FALSE;
        }
        else {
          switch(current->type) {
          case TYPE_ITD:
            temp_itd = (itd_t*) *current_virt;
            
            current = &temp_itd->next_link_pointer;
            current_virt = &temp_itd->next_virt;
            break;
            
          case TYPE_QH:
            temp_qh = (qh_t*)*current_virt;
            
            if(temp_qh == qh) {
              not_inserted = FALSE;
              break;
            }
            temp_frame_interval = temp_qh->interval / 8;
            if(temp_frame_interval == 0) {
              temp_frame_interval = 1;
            }
            
            if(frame_interval > temp_frame_interval) {
              /*
               * New qh's frame interval is greater than current qh
               * interval so we insert the element right before
               * temp_qh
               */
              insert_qh_into_frame_list(current, current_virt, qh);
              not_inserted = FALSE;
            }
            else {
              /*
               * Need to keep traversing list
               */
              current = &temp_qh->horizontalPointer;
              current_virt = &temp_qh->next_virt;
            }
            break;


          case TYPE_SITD:
          case TYPE_FSTN:
          default:
            /*
             * -- EM -- These shouldn't ever be in the periodic list right
             * now because we don't use them at all yet
             */
            DLOG("Unhandled case in %s line %d", __FILE__, __LINE__);
            panic("Unhandled case in link_qh_to_periodic_list");
          }
        }
      }
    }
  }
  else {
    /*
     * -- EM -- This method does not take into account remaining bytes
     * per microframe and interval, need to fix it
     */
    DLOG("Non realtime interrupt is broken");
    panic("Non realtime interrupt is broken");
  }
  
  qh->state = QH_STATE_LINKED;
}


static int
submit_interrupt_qtd(ehci_hcd_t* ehci_hcd, struct urb* urb,
                     qh_t** qh, uint32_t endpoint,
                     uint32_t device_addr, uint32_t dev_speed,
                     uint32_t pipe_type, uint32_t max_packet_len,
                     bool is_input, bool ioc_enabled,
                     list_head_t* qtd_list)
{
  if(*qh == NULL) {
    *qh = allocate_qh(ehci_hcd);
    qh_prep(ehci_hcd, urb, *qh, endpoint, device_addr,
            max_packet_len, dev_speed, pipe_type, is_input);
  }

  get_int_urb_priv(urb)->qh_urb_priv.qh = *qh;
  
  EHCI_SET_DEVICE_QH(ehci_hcd, device_addr, is_input, endpoint, *qh);

  qh_append_qtds(ehci_hcd, urb, *qh, qtd_list);
  if((*qh)->state == QH_STATE_NOT_LINKED) {
    link_qh_to_periodic_list(ehci_hcd, *qh, urb);
  }

  return 0;
}




static int
ehci_interrupt_transfer(ehci_hcd_t* ehci_hcd, struct urb* urb)
{
  unsigned int pipe = urb->pipe;
  uint8_t address = usb_pipedevice(pipe);
  bool is_input = usb_pipein(pipe);
  uint8_t endpoint = usb_pipeendpoint(pipe);
  qh_t* qh  = EHCI_GET_DEVICE_QH(ehci_hcd, address, is_input, endpoint);
  list_head_t qtd_list;
  uint8_t pipe_type = usb_pipetype(pipe);
  int packet_len = usb_maxpacket(urb->dev, urb->pipe);
  ehci_int_urb_priv_t* int_urb_priv;
  int interval_offset;
  ehci_completion_element_t* completion_element = NULL;

  if(qh != NULL) {
    urb_qh_compatiblity_check(urb, qh);
  }

  INIT_LIST_HEAD(&qtd_list);

  //DLOG("urb_pipe = %d", urb->pipe);
  
  if(!urb->realtime) {
    DLOG("Non realtime interrupts urbs are broken right now");
    panic("Non realtime interrupts urbs are broken right now");
  }

  

  if(is_input) {

    completion_element = allocate_ehci_completion_element(ehci_hcd);
    if(completion_element == NULL) {
      /* -- EM -- Better cleanup here later */
      DLOG("Failed to allocate ehci_completion_element at line %d", __LINE__);
      return -1;
    }
    
    
    if(!create_qtd_chain(ehci_hcd, urb, urb->setup_packet, sizeof(USB_DEV_REQ),
                         urb->transfer_buffer, urb->transfer_buffer_length,
                         mp_enabled, &qtd_list, urb->interrupt_byte_rate, FALSE,
                         urb->context)) {
      return -1;
    }
  }
  
  if(urb->hcpriv == NULL) {
    list_head_t* temp_list;
    int num_qtds;
    qtd_t* temp_qtd;
    int i;
    num_qtds = 0;
    if(is_input) {
      list_for_each(temp_list, &qtd_list) { num_qtds++; }
    }
    
    
    urb->hcpriv = ehci_alloc_int_urb_priv(num_qtds);
    if(urb->hcpriv == NULL) {
      qtd_t* safe_qtd;
      DLOG("Failed to allocate ehci_iso_urb_priv");
      list_for_each_entry_safe(temp_qtd, safe_qtd, &qtd_list, chain_list) {
        free_qtd(ehci_hcd, temp_qtd);
      }
      return -1;
    }

    interval_offset = ehci_is_rt_schedulable(ehci_hcd, urb);
    
    if(interval_offset < 0) {
      DLOG("Cannot schedule URB");
      return -1;
    }
    
    int_urb_priv = urb->hcpriv;
    int_urb_priv->qh_urb_priv.buffer_size = urb->transfer_buffer_length;
    int_urb_priv->ehci_urb_priv.sched_assignment = interval_offset;

    if(is_input) {
      i = 0;
      list_for_each_entry(temp_qtd, &qtd_list, chain_list) {
        int_urb_priv->qh_urb_priv.qtds[i++] = temp_qtd;
      }
    }
  }
  
  if(submit_interrupt_qtd(ehci_hcd, urb, &qh, endpoint, address,
                          urb->dev->speed, pipe_type, packet_len,
                          is_input, mp_enabled,
                          &qtd_list) < 0) {
    return -1;
  }

  if(urb->realtime) {
    if(completion_element) {
      completion_element->urb = urb;
      completion_element->pipe_type = pipe_type;
      spinlock_lock(&ehci_hcd->completion_lock);
      list_add_tail(&completion_element->chain_list, &ehci_hcd->completion_list);
      spinlock_unlock(&ehci_hcd->completion_lock);
    }
    return 0;
  }
  else {
    if(mp_enabled) {
      return 0;
    }
    else {
      int result;
      if((result = spin_for_transfer_completion(ehci_hcd, urb, qh, address,
                                                endpoint, pipe_type, is_input, urb->timeout)) < 0) {
        
        DLOG("spin_for_transfer_completion returned %d", result);
        return result;
      }
      
      if(urb->complete) {
        urb->complete(urb);
      }
    }
    return 0;
  }
}
/* -- EM -- This function needs to be cleaned up very hard to follow
   with how it works for real-time and non-real-time async
   transfers */
static int
ehci_async_transfer(ehci_hcd_t* ehci_hcd, struct urb* urb)
{
  unsigned int pipe = urb->pipe;
  uint8_t address = usb_pipedevice(pipe);
  bool is_input = usb_pipein(pipe);
  uint8_t endpoint = usb_pipeendpoint(pipe);
  qh_t* qh;
  list_head_t qtd_list;
  uint8_t pipe_type = usb_pipetype(pipe);
  int packet_len = usb_maxpacket(urb->dev, urb->pipe);
  ehci_completion_element_t* completion_element = NULL;
  
  
  qh = EHCI_GET_DEVICE_QH(ehci_hcd, address, is_input, endpoint);
  if(qh != NULL) {
    urb_qh_compatiblity_check(urb, qh);
  }
  //DLOG("qh returned by get device qh = 0x%p", qh);
  if(address == 0 && qh) {
    bool change_qh;
    /* Check to see if we need to change the QH for address zero */
    switch(urb->dev->speed) {
    case USB_SPEED_LOW:
    case USB_SPEED_FULL:
      change_qh =
        urb->dev->hub_addr != ehci_hcd->zero_qh_hub_addr ||
        urb->dev->port_num != ehci_hcd->zero_qh_port_num;
      break;
    case USB_SPEED_HIGH:
      change_qh = ehci_hcd->zero_qh_hub_addr > 0 || ehci_hcd->zero_qh_port_num > 0;
      break;
    default:
      DLOG("Unknown device speed");
      return -1;
    }

    if(change_qh) {
      unlink_async_queue_head(ehci_hcd, EHCI_GET_DEVICE_QH(ehci_hcd, address, is_input, endpoint));
      EHCI_SET_DEVICE_QH(ehci_hcd, address, is_input, endpoint, NULL);
      qh = NULL;
      switch(urb->dev->speed) {
      case USB_SPEED_LOW:
      case USB_SPEED_FULL:
        ehci_hcd->zero_qh_hub_addr = urb->dev->hub_addr;
        ehci_hcd->zero_qh_port_num = urb->dev->port_num;
        break;

      case USB_SPEED_HIGH:
        ehci_hcd->zero_qh_hub_addr = -1;
        ehci_hcd->zero_qh_port_num = -1;
        break;
        /* Don't have a default case here since we have one above */
      }
    }
  }
  
  INIT_LIST_HEAD(&qtd_list);

  if(mp_enabled && usb_pipetype(pipe) == PIPE_BULK && !urb->realtime) {
    //  if(usb_pipetype(pipe) != PIPE_BULK &&
    //     mp_enabled && !urb->realtime) {
    DLOG("Can only do control transactions or real time bulk transactions");
    panic("Can only do control transactions or real time bulk transactions");
  }

  /*
   * -- EM -- high jack function if it is bulk and realtime urb
   */
  if(usb_pipetype(pipe) == PIPE_BULK && urb->realtime) {
    bool extra_nonrt_qtd = FALSE;
    /* If there is a qh associated with this device and endpoint then
       non real-time transactions were performed on this device
       before, need to make sure all the qtds associated with this qh
       are done and removed before continuing */
    if(qh) {
      qtd_t* qtd_to_remove;
      qtd_t* qtd_tmp;
      list_for_each_entry_safe(qtd_to_remove, qtd_tmp, &qh->qtd_list, chain_list) {
        if(qtd_to_remove->dma_addr == qh->current_qtd_ptr_raw) {
          /* If we are not at the last qtd or are at the last qtd but
             there is still data remaining */
          if(qtd_to_remove->chain_list.next != &qh->qtd_list ||
             ((qtd_to_remove->token >> 16) & 0x7FFF) > 0 ) {
            return -2;
          }
          extra_nonrt_qtd = TRUE;
          break;
        }
        else {
          free_qtd(ehci_hcd, qtd_to_remove);
        }
      }
    }
    if(is_input) {
      completion_element = allocate_ehci_completion_element(ehci_hcd);
      if(completion_element == NULL) {
        /* -- EM -- Better cleanup here later */
        DLOG("Failed to allocate ehci_completion_element at line %d", __LINE__);
        return -1;
      }
      
      if(!create_qtd_chain(ehci_hcd, urb, urb->setup_packet, sizeof(USB_DEV_REQ),
                           urb->transfer_buffer, urb->transfer_buffer_length,
                           mp_enabled, &qtd_list, urb->interrupt_byte_rate, FALSE,
                           urb->context)) {
        DLOG("Failed to create qtd chain");
        return -1;
      }
    }
    
    if(urb->hcpriv == NULL) {
      list_head_t* temp_list;
      int num_qtds;
      qtd_t* temp_qtd;
      int i;
      ehci_bulk_urb_priv_t* bulk_urb_priv;
      
      num_qtds = 0;
      if(is_input) {
        list_for_each(temp_list, &qtd_list) { num_qtds++; }
      }
      urb->hcpriv = ehci_alloc_bulk_urb_priv(num_qtds);

      if(urb->hcpriv == NULL) {
        qtd_t* safe_qtd;
        DLOG("Failed to allocate ehci_iso_urb_priv");
        list_for_each_entry_safe(temp_qtd, safe_qtd, &qtd_list, chain_list) {
          free_qtd(ehci_hcd, temp_qtd);
        }
        return -1;
      }

      if(ehci_is_rt_schedulable(ehci_hcd, urb) < 0) {
        qtd_t* safe_qtd;
        DLOG("URB not schedulable");
        list_for_each_entry_safe(temp_qtd, safe_qtd, &qtd_list, chain_list) {
          free_qtd(ehci_hcd, temp_qtd);
        }
        return -1;
      }
            
      
      bulk_urb_priv = urb->hcpriv;
      bulk_urb_priv->qh_urb_priv.extra_nonrt_qtd = extra_nonrt_qtd;
      bulk_urb_priv->qh_urb_priv.buffer_size = urb->transfer_buffer_length;
      
      if(is_input) {
        i = 0;
        list_for_each_entry(temp_qtd, &qtd_list, chain_list) {
          bulk_urb_priv->qh_urb_priv.qtds[i++] = temp_qtd;
        }
      }
    }
  
    if(submit_async_qtd_chain(ehci_hcd, urb, &qh, endpoint, address,
                              urb->dev->speed, pipe_type, packet_len,
                              is_input, mp_enabled,
                              &qtd_list) < 0) {
      /* -- EM -- need to do proper clean up of resources */
      return -1;
    }

    if(is_input && completion_element) {
      completion_element->urb = urb;
      completion_element->pipe_type = pipe_type;
      spinlock_lock(&ehci_hcd->completion_lock);
      list_add_tail(&completion_element->chain_list, &ehci_hcd->completion_list);
      spinlock_unlock(&ehci_hcd->completion_lock);
    }
    
    
    return 0;
  }

  if(mp_enabled) {
    completion_element = allocate_ehci_completion_element(ehci_hcd);
    if(completion_element == NULL) {
      DLOG("Failed to allocate completion element");
      panic("Failed to allocate completion element");
    }
  }
  
  if(!create_qtd_chain(ehci_hcd, urb, urb->setup_packet, sizeof(USB_DEV_REQ),
                       urb->transfer_buffer, urb->transfer_buffer_length,
                       mp_enabled, &qtd_list, 0, FALSE, urb->context)) {
    return -1;
  }

  
  if(mp_enabled) {
    int i;
    int num_qtds = 0;
    qtd_t* temp_qtd;
    ehci_ctrl_urb_priv_t* ctrl_urb_priv;
    list_for_each_entry(temp_qtd, &qtd_list, chain_list) {
      ++num_qtds;
    }

    ctrl_urb_priv = ehci_alloc_ctrl_urb_priv(num_qtds);
    
    urb->hcpriv = ctrl_urb_priv;
    
    i = 0;
    list_for_each_entry(temp_qtd, &qtd_list, chain_list) {
      ctrl_urb_priv->qh_urb_priv.qtds[i++] = temp_qtd;
    }
    completion_element->urb = urb;
    completion_element->pipe_type = pipe_type;
    spinlock_lock(&ehci_hcd->completion_lock);
    list_add_tail(&completion_element->chain_list, &ehci_hcd->completion_list);
    spinlock_unlock(&ehci_hcd->completion_lock);
  }
  
  return submit_async_qtd_chain(ehci_hcd, urb, &qh, endpoint, address,
                                urb->dev->speed, pipe_type, packet_len,
                                is_input, mp_enabled,
                                &qtd_list);
}

int itd_fill(itd_t* itd, struct urb* urb, uint8_t address, uint8_t endpoint,
             int max_packet_len, bool is_input, phys_addr_t data_phys,
             usb_iso_packet_descriptor_t* packets, int num_packets,
             uint32_t microframe_offset)
{ 
  int i;
  int cur_buf_ptr = 0;
  uint32_t old_buf_ptr_value;
  phys_addr_t data_phys_start = data_phys;
  int packet_interval = 8 / num_packets;

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
  
  for(i = microframe_offset; i < 8; i += packet_interval) {
    data_phys = data_phys_start + packets[i].offset;
    
    /* Check to see if we have crossed a page boundary and if so fill
       the buffer pointer with the appropriate address and increment
       our current buffer pointer */
    if(old_buf_ptr_value != (data_phys & ~0x0FFF)) {
      if(cur_buf_ptr == 6) {
        /* This can occur if the offset between iso packets is too
           larger, if this occurs we cannot make an iTD that can
           handle the given packets */;
        return -1;
      }
      old_buf_ptr_value = data_phys & ~0x0FFF;
      cur_buf_ptr++;
      ITD_SET_BUF_PTR(itd, cur_buf_ptr, data_phys);
    }
    
    /* -- EM -- Change this to be more efficient later */
    
    itd->transaction[i].raw = ITD_ACTIVE;
    itd->transaction[i].length = packets[i].length;
    itd->transaction[i].offset = data_phys & 0x0FFF;
    itd->transaction[i].page_selector = cur_buf_ptr;
  }
  
  
  return 0;
}

static int create_itd_chain(ehci_hcd_t* ehci_hcd, uint8_t address,
                            uint8_t endpoint, addr_t data,
                            usb_iso_packet_descriptor_t* packets,
                            int num_packets, 
                            int max_packet_len, bool is_input,
                            list_head_t* itd_list, bool ioc,
                            struct urb* urb, uint interrupt_frame_rate,
                            void* completion_context)
{
  int i;
  itd_t* itd;
  uint32_t packets_per_itd;
  uint32_t microframe_interval = urb->interval;
  ehci_iso_urb_priv_t* iso_urb_priv = get_iso_urb_priv(urb);
  uint32_t microframe_offset = iso_urb_priv->ehci_urb_priv.sched_assignment & 0x7;
  phys_addr_t data_phys = (phys_addr_t)get_phys_addr(data);
  INIT_LIST_HEAD(itd_list);
  iso_urb_priv->num_itds = 0;
  
  packets_per_itd = microframe_interval < 8 ? 8 / microframe_interval : 1;
  
  i = 0;
  while(i < num_packets) {
    
    int packets_this_itd = packets_per_itd < (num_packets - i) ?
      packets_per_itd : num_packets - i;
    itd_t* current_itd = allocate_itd(ehci_hcd);
    usb_iso_packet_descriptor_t* itd_packets = &packets[i];
    
    if(current_itd == NULL) {
      DLOG("Failed to allocate itd");
      panic("Failed to allocate itd");
      return -1;
    }
    
    list_add_tail(&current_itd->chain_list, itd_list);
    (iso_urb_priv->num_itds)++;
    if(itd_fill(current_itd, urb, address, endpoint, max_packet_len,
                is_input, data_phys, itd_packets, packets_this_itd,
                microframe_offset) < 0) {
      return -1;
    }
    
    i += packets_this_itd;
  }
  
  if(urb->realtime) {
    int j = 0;
    int last_interrupt = interrupt_frame_rate;
    
    list_for_each_entry(itd, itd_list, chain_list) {
      iso_urb_priv->itds[j++] = itd;
      for(i = 0; i < 8; ++i) {
        if(IS_ITD_TRANSACTION_ACTIVE(itd, i)) {
          
          if(interrupt_frame_rate != 0) {
            last_interrupt--;
            if(last_interrupt == 0) {
              itd->transaction[i].raw |= ITD_IOC;
              itd->completion_context = completion_context;
              last_interrupt = interrupt_frame_rate;
            }
          }
        }
      }
      memcpy(itd->transaction_backup, itd->transaction,
             8 * sizeof(itd_transaction_t));
    }
  }
  else {
    if(ioc) {
      itd_t* ioc_itd = list_entry(itd_list->prev, itd_t, chain_list);
      int packets_last_itd = num_packets & 0x7; // num_packets % 8
      if(packets_last_itd == 0) packets_last_itd = 8;
      ioc_itd->transaction[packets_last_itd-1].raw |= ITD_IOC;
    }
  }
  
  return 0;
}


static int submit_itd_chain(ehci_hcd_t* ehci_hcd, list_head_t* itd_list,
                            struct urb* urb)
{
  itd_t* itd;
  uint32_t current_frame_index, start_frame_index;
  uint32_t frame_index;
  ehci_iso_urb_priv_t* iso_urb_priv = get_iso_urb_priv(urb);
  frm_lst_lnk_ptr_t* frame_list     = ehci_hcd->frame_list;
  frm_lst_lnk_ptr_t** frame_list_virt     = ehci_hcd->frame_list_virt;
  uint32_t frame_list_size          = ehci_hcd->frame_list_size;
  uint32_t frame_list_mask          = frame_list_size - 1;
  uint32_t frame_interval_offset    = iso_urb_priv->ehci_urb_priv.sched_assignment / 8;
  uint32_t frame_interval           = urb->interval / 8;
  bool     uninserted_items;
  
  /*
   * -- EM -- Right now we are only reading the frame index register
   * once and assuming we insert elements faster than the ehci chip
   * traverses the list, this is only true because the kernel is
   * non-preemptive, when it is made preemptive this needs to be
   * fixed, of course an ugly hack would be to turn interrupts off for
   * just this portion but a more elegant solution might exists.
   */

  list_for_each_entry(itd, itd_list, chain_list) {
    itd->ioc_processed_bitmap = 0;
  }
  
    
  if(frame_interval == 0) {
    frame_interval = 1;
  }
  
  if(urb->realtime) {
    
    current_frame_index = EHCI_CURRENT_FRAME_INDEX(ehci_hcd);
    frame_index = ((current_frame_index + EHCI_SAFE_FRAME_INSERT_OFFSET(ehci_hcd))
                   | (frame_interval-1)) + 1 + frame_interval_offset;
    
    start_frame_index = frame_index = frame_index & frame_list_mask;
    
    if(start_frame_index == 0) {
      start_frame_index = frame_list_size - 1;
    }
    else {
      start_frame_index = start_frame_index - 1;
    }

    INIT_LIST_HEAD(&iso_urb_priv->uninserted_itds_list);

    if(usb_pipein(urb->pipe)) {
      /* Realtime Input */
      uninserted_items = FALSE;
      list_for_each_entry(itd, itd_list, chain_list) {
        
        if(EHCI_IS_INDEX_IN_CIRC_BUF_REGION(frame_index,
                                            current_frame_index,
                                            start_frame_index)) {
          list_add_tail(&itd->uninserted_list, &iso_urb_priv->uninserted_itds_list);
          
          uninserted_items = TRUE;
        }
        else {
          insert_itd_into_frame_list(ehci_hcd, frame_list, frame_list_virt, frame_index, itd);
        }
        
        itd->frame_index = frame_index;
        frame_index = (frame_interval + frame_index) & frame_list_mask;
      }

      if(uninserted_items) {
        spinlock_lock(&ehci_hcd->uninserted_itd_lock);
        
        list_add_tail(&iso_urb_priv->uninserted_itd_urb_list,
                      &ehci_hcd->uninserted_itd_urb_list);
        spinlock_unlock(&ehci_hcd->uninserted_itd_lock);
      }
    }
    else {
      /* Realtime Output */
      /*
       * At this point frame_index points to the first safe element to
       * inserted items, however if there are already itds in the list
       * and they are still active we should put the next itds after
       * them as long as it is still safe
       */
      
      if(!list_empty(&iso_urb_priv->write_itds_to_free)) {
        
        itd_t* last_itd = list_entry(iso_urb_priv->write_itds_to_free.prev,
                                     itd_t, chain_list);
        
        if((!EHCI_IS_INDEX_IN_CIRC_BUF_REGION(last_itd->frame_index,
                                              current_frame_index,
                                              start_frame_index)) &&
           is_itd_partially_active(last_itd)) {
          DLOG("Setting itd from last itd");
          frame_index = (last_itd->frame_index + frame_interval) & frame_list_mask;
        }
      }

      /*
       * At this point frame_index is where we should start inserting
       * items
       */


      list_for_each_entry(itd, itd_list, chain_list) {
        if(EHCI_IS_INDEX_IN_CIRC_BUF_REGION(frame_index,
                                            current_frame_index,
                                            start_frame_index)) {
          DLOG_UINT(frame_index);
          DLOG_UINT(current_frame_index);
          DLOG_UINT(start_frame_index);
          
        }
        EHCI_ASSERT(!EHCI_IS_INDEX_IN_CIRC_BUF_REGION(frame_index,
                                                      current_frame_index,
                                                      start_frame_index));

        insert_itd_into_frame_list(ehci_hcd, frame_list, frame_list_virt, frame_index, itd);
        itd->frame_index = frame_index;
        frame_index = (frame_index + frame_interval) & frame_list_mask;
      }
    }
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
      insert_itd_into_frame_list(ehci_hcd, frame_list, frame_list_virt, frame_index, itd);
      ++frame_index;
    }
  }
  
  return 0;
}


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
    urb->hcpriv = ehci_alloc_iso_urb_priv(urb);
    if(urb->hcpriv == NULL) {
      DLOG("Failed to allocate ehci_iso_urb_priv");
      return -1;
    }
  }

  iso_urb_priv = urb->hcpriv;
  
  if(urb->realtime) {
    if((iso_urb_priv->ehci_urb_priv.sched_assignment
        = ehci_is_rt_schedulable(ehci_hcd, urb)) < 0) {
      DLOG("urb is not real time schedulable");
      return iso_urb_priv->ehci_urb_priv.sched_assignment;
    }
  }

  if(urb->realtime && usb_pipeout(urb->pipe)) {
    /* Just need to reserve the bandwidth for an out pipe, data
       arrives via pushes later one */
    return 0;
  }
  if(create_itd_chain(ehci_hcd, usb_pipedevice(pipe),
                      usb_pipeendpoint(pipe), urb->transfer_buffer,
                      packets,
                      num_packets,
                      usb_maxpacket(urb->dev, urb->pipe),
                      usb_pipein(pipe),
                      &itd_list, mp_enabled,
                      urb, urb->interrupt_frame_rate, urb->context) < 0) {
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
    if( ((!urb->realtime) && (!(urb->transfer_flags & URB_NO_INTERRUPT))) ||
        (urb->realtime && usb_pipein(urb->pipe) && (urb->interrupt_frame_rate != 0)) ) {
      ehci_completion_element_t* comp_elem = allocate_ehci_completion_element(ehci_hcd);
      if(comp_elem == NULL) {
        /* -- EM -- Better cleanup here later */
        DLOG("Failed to allocate ehci_completion_element at line %d", __LINE__);
        panic("Failed to allocate ehci_completion_element");
      }
      DLOG("Creating and inserting ehci_completion_element");
      comp_elem->urb = urb;
      INIT_LIST_HEAD(&comp_elem->itds);
      list_splice(&itd_list, &comp_elem->itds);
      comp_elem->pipe_type = PIPE_ISOCHRONOUS;
      spinlock_lock(&ehci_hcd->completion_lock);
      list_add_tail(&comp_elem->chain_list, &ehci_hcd->completion_list);
      spinlock_unlock(&ehci_hcd->completion_lock);
    }
    
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
          unlink_itd_from_frame_list(ehci_hcd, itd);
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
      unlink_itd_from_frame_list(ehci_hcd, itd);
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


ehci_qh_urb_priv_t* ehci_get_qh_urb_priv(struct urb* urb)
{
  switch(usb_pipetype(urb->pipe)) {
  case PIPE_BULK:
    return &(get_bulk_urb_priv(urb)->qh_urb_priv);
  case PIPE_INTERRUPT:
    return &(get_int_urb_priv(urb)->qh_urb_priv);
  case PIPE_CONTROL:
    return &(get_ctrl_urb_priv(urb)->qh_urb_priv);
  }
  return NULL;
}

ehci_urb_priv_t* get_ehci_urb_priv(struct urb* urb)
{
  switch(usb_pipetype(urb->pipe)) {
  case PIPE_BULK:
    return &(get_bulk_urb_priv(urb)->ehci_urb_priv);
  case PIPE_INTERRUPT:
    return &(get_int_urb_priv(urb)->ehci_urb_priv);
  case PIPE_ISOCHRONOUS:
    return &(get_iso_urb_priv(urb)->ehci_urb_priv);
  case PIPE_CONTROL:
    return &(get_ctrl_urb_priv(urb)->ehci_urb_priv);
  }

  return NULL;
}

static inline uint calc_total_bytes_cost(struct urb* urb)
{
  uint16_t maxpacket = usb_maxpacket(urb->dev, urb->pipe);
  uint32_t bytes_per_packet = max_packet(maxpacket);
  uint32_t num_packets = hb_mult(maxpacket);
  
  return (usb_pipetype(urb->pipe) ==  PIPE_ISOCHRONOUS ?
          HS_USECS_ISO(bytes_per_packet) : HS_USECS(bytes_per_packet))
    * num_packets;
}


static void allocate_bandwidth(ehci_hcd_t* ehci_hcd, struct urb* urb, uint32_t assignment)
{
  ehci_urb_priv_t* ehci_urb_priv = get_ehci_urb_priv(urb);
  int interval = urb->interval;
  uint32_t  frame_list_size = ehci_hcd->frame_list_size;
#ifdef USB_REALTIME_ASYNC
  uint32_t* micro_frame_remaining_time_async = ehci_hcd->micro_frame_remaining_time_async;
#endif
  uint32_t* micro_frame_remaining_time_periodic = ehci_hcd->micro_frame_remaining_time_periodic;
  uint usecs_per_transaction = ehci_urb_priv->usecs_per_transactions;
  uint32_t i;
  uint pipe_type = usb_pipetype(urb->pipe);

  if(pipe_type == PIPE_ISOCHRONOUS || pipe_type == PIPE_INTERRUPT) {
    for(i = assignment; i < frame_list_size * 8; i += interval) {
      micro_frame_remaining_time_periodic[i] -= usecs_per_transaction;
    }
  }
#ifdef USB_REALTIME_ASYNC
  for(i = assignment; i < frame_list_size * 8; i += interval) {
    micro_frame_remaining_time_async[i] -= usecs_per_transaction;
  }
#endif
}

int fits_in_current_schedule(uint32_t* micro_frame_remaining_time_periodic,
                             uint32_t* micro_frame_remaining_time_async, int interval,
                             uint bytes_per_micro_frame, uint32_t frame_list_size,
                             uint pipe_type)
{
  int i, j;
  bool schedulable;
  for(i = 0, schedulable = FALSE; i < interval; ++i) {
    schedulable = TRUE;
    for(j = i; j < frame_list_size * 8; j += interval) {
      if(pipe_type == PIPE_ISOCHRONOUS &&
         (micro_frame_remaining_time_periodic[j] < bytes_per_micro_frame)) {
        schedulable = FALSE;
        break;
      }
#ifdef USB_REALTIME_ASYNC
      else if(micro_frame_remaining_time_async[j] < bytes_per_micro_frame) {
        schedulable = FALSE;
        break;
      }
#endif
      
    }
    if(schedulable) break;
  }
  return schedulable ? i : -1;
}

typedef struct {
  int urb_index;
  int interval;
  int usec;
  int type;
  int original_assignment;
} usb_sched_tuple_t;

int usb_sched_tuple_cmp(const void* a, const void* b)
{
  const usb_sched_tuple_t* aa = a;
  const usb_sched_tuple_t* bb = b;

  return aa->interval == bb->interval ? bb->usec - aa->usec : aa->interval - bb->interval;
}

bool first_fit_usb_scheduling(usb_sched_tuple_t sched_tuples[MAX_RT_URBS], int num_reqs,
                              int max_interval,
                              int assignments[MAX_RT_URBS])
{
  int* periodic_bandwidth_remaining;
  int* async_bandwidth_remaining;
  int i, j, k;
  periodic_bandwidth_remaining = kmalloc(sizeof(int) * max_interval);
  async_bandwidth_remaining = kmalloc(sizeof(int) * max_interval);
  bool valid_scheduling = TRUE;

  for(i = 0; i < max_interval; ++i) {
    periodic_bandwidth_remaining[i] = USEC_PER_MICRO_FRAME_FOR_PERIODIC;
#ifdef USB_REALTIME_ASYNC
    async_bandwidth_remaining[i] = USEC_PER_MICRO_FRAME_FOR_ASYNC;
#endif
  }

  /* We can't move interrupt qh's in the periodic list because we have
     to stagger itd remapping and you can't stagger interrupt qh
     mapping, so its really one or the other and we choose to be able
     to remap isochronous, it might be possible to either remap only
     isochronous or only interrupt so we should try to find a
     scheduling with all the isochronous pinned and all the interrupts
     moving but for now just only remap isochronous */
  for(i = 0; i < num_reqs; ++i) {
    if(sched_tuples[i].type == PIPE_INTERRUPT) {
      for(j = sched_tuples[i].original_assignment;
          j < max_interval; j += sched_tuples[i].interval) {
        periodic_bandwidth_remaining[j] -= sched_tuples[i].usec;
#ifdef USB_REALTIME_ASYNC
        async_bandwidth_remaining[j] -= sched_tuples[i].usec;
#endif
      }
    }
  }
  
  for(i = 0; i < num_reqs && valid_scheduling; ++i) {
    int sched_interval = sched_tuples[i].interval;
    int sched_bandwidth = sched_tuples[i].usec;
    assignments[i] = -1;

    if(sched_tuples[i].type == PIPE_INTERRUPT) {
      assignments[i] = sched_tuples[i].original_assignment;
      continue;
    }
    /* -- EM -- Some of the functionality here is duplicated in
          fits_in_current_schedule */
    for(j = 0; j < sched_interval; ++j) {
      bool schedulable = TRUE;
      for(k = j; k < max_interval; k += sched_interval) {
	//DLOG("k = %d, j = %d, i = %d", k, j, i);

        if(sched_tuples[i].type == PIPE_ISOCHRONOUS &&
           (periodic_bandwidth_remaining[k] < sched_bandwidth)) {
          schedulable = FALSE;
          break;
        }
#ifdef USB_REALTIME_ASYNC
        else if(async_bandwidth_remaining[k] < sched_bandwidth) {
          schedulable = FALSE;
          break;
        }
#endif
      }
      if(schedulable) {
        assignments[i] = j;
        
	for(k = j; k < max_interval; k += sched_interval) {
#ifdef USB_REALTIME_ASYNC
          async_bandwidth_remaining[k] -= sched_bandwidth;
#endif
          if(sched_tuples[i].type == PIPE_ISOCHRONOUS) {
            periodic_bandwidth_remaining[k] -= sched_bandwidth;
          }
        }
	break;
      }
    }
    valid_scheduling = assignments[i] >= 0;
  }
  
  kfree(periodic_bandwidth_remaining);
  kfree(async_bandwidth_remaining);
  return valid_scheduling;
}

static bool valid_scheduling_exists(ehci_hcd_t* ehci_hcd, struct urb* urb)
{
  int i;
  usb_sched_tuple_t sched_tuples[MAX_RT_URBS];
  int num_rt_urbs = ehci_hcd->num_rt_urbs;
  int max_interval = 1;
  int sorted_assignments[MAX_RT_URBS];
  bool schedulable;
  
  for(i = 0; i < num_rt_urbs; ++i) {
    sched_tuples[i].type = usb_pipetype(ehci_hcd->rt_urbs[i]->pipe);
    sched_tuples[i].original_assignment = get_ehci_urb_priv(ehci_hcd->rt_urbs[i])->sched_assignment;
    sched_tuples[i].urb_index = i;
    sched_tuples[i].interval = ehci_hcd->rt_urbs[i]->interval;
    sched_tuples[i].usec = get_ehci_urb_priv(ehci_hcd->rt_urbs[i])->usecs_per_transactions;
  }
  
  sort(sched_tuples, num_rt_urbs, sizeof(usb_sched_tuple_t), usb_sched_tuple_cmp, NULL);
  
  max_interval = sched_tuples[num_rt_urbs - 1].interval;
  
  schedulable = first_fit_usb_scheduling(sched_tuples, num_rt_urbs, max_interval,
                                         sorted_assignments);
  
  if(schedulable) {
    for(i = 0; i < num_rt_urbs; ++i) {
      ehci_hcd->new_assignments[sched_tuples[i].urb_index] = sorted_assignments[i];
    }
  }

  return schedulable;
  
}

static void clear_reordered_map(ehci_hcd_t* ehci_hcd)
{
  /* -- EM -- Could probably just do a memset here but being safe for
     now and setting each value explicitly */
  int i;
  int frame_list_size = ehci_hcd->frame_list_size;
  bool* reordered_map = ehci_hcd->reordered_map;
  for(i = 0; i < frame_list_size; ++i) {
    reordered_map[i] = FALSE;
  }
}

/*
 * Returns a value < 0 if it is not schedulable if it is schedulable
 * it returns the following depending on the type:
 *
 * Isochronous/Interrupt- returns the offset from interval we should
 * start on
 *
 * Bulk/Control - returns 0
 */
static int ehci_is_rt_schedulable(ehci_hcd_t* ehci_hcd, struct urb* urb)
{
  uint32_t i;
  uint32_t frame_list_size = ehci_hcd->frame_list_size;
  int interval = urb->interval;
  int new_assignment;
  uint  bytes_per_micro_frame;
  uint32_t* micro_frame_remaining_time_periodic = ehci_hcd->micro_frame_remaining_time_periodic;
  uint32_t* micro_frame_remaining_time_async = ehci_hcd->micro_frame_remaining_time_async;
  ehci_urb_priv_t* ehci_urb_priv = get_ehci_urb_priv(urb);
  uint pipe_type = usb_pipetype(urb->pipe);
  int min_async_interval = 2048;
  int num_async_rt_urbs = 0;

  
  if(ehci_hcd->num_rt_urbs == MAX_RT_URBS) {
    DLOG("Reached limit of RT urbs");
    return -1;
  }

  bytes_per_micro_frame
    = ehci_urb_priv->usecs_per_transactions
    = calc_total_bytes_cost(urb);
  
#ifndef USB_REALTIME_ASYNC
  /* Can't do real-time async transactions but so devices written for
     real-time async don't fail and to allow the same interface even
     when we aren't doing realtime just return 0 better solution would
     be return an errno message indicating that the functionality is
     not supported */
  if((pipe_type == PIPE_CONTROL) || (pipe_type == PIPE_BULK)) {
    return 0;
  }
#endif

  for(i = 0; i < ehci_hcd->num_rt_urbs; ++i) {
    struct urb* rt_urb = ehci_hcd->rt_urbs[i];
    if((usb_pipetype(rt_urb->pipe) == PIPE_CONTROL) ||
       (usb_pipetype(rt_urb->pipe) == PIPE_BULK)) {
      num_async_rt_urbs++;
      if(rt_urb->interval < min_async_interval) {
        min_async_interval = rt_urb->interval;
      }
    }
  }

  if((pipe_type == PIPE_ISOCHRONOUS) &&
     (ehci_hcd->frame_list_size * 8 / interval != urb->number_of_packets)) {
    DLOG("number of packets does not match interval and frame size");
    return -1;
  }
  
  /* Put new urb request into real-time urbs array and remove it at
     the end of the function if it is not schedulable */
  
  ehci_hcd->rt_urbs[ehci_hcd->num_rt_urbs++] = urb;
  
  
  if((pipe_type == PIPE_CONTROL) || (pipe_type == PIPE_BULK)) {
    if(num_async_rt_urbs > 0) {
      if(interval < min_async_interval) {

        /* For the sake of simplicity we do not first check to see if it
           is schedulable under the old mapping of periodic
           transactions, we first test to see if it is schedulable if we
           reorder itds and if it is we find the scheduling.  After if
           we didn't move any itds around we don't go into remap
           mode.  START HERE */

        DLOG("Case not implemented file %s line %d", __FILE__, __LINE__);
        panic("case not implemented in ehci_hcd.c function ehci_is_rt_schedulable");
      }
      else {
        /* Just set the interval to the be the same as min interval and
           continue on */
        interval = min_async_interval;
      }
    }
  }
  
  /* First try to schedule it using first fit, if this works all is
     good if it doesn't we later see if it is schedulable if we
     reorder transactions */
  new_assignment = fits_in_current_schedule(micro_frame_remaining_time_periodic,
                                            micro_frame_remaining_time_async,
                                            interval, bytes_per_micro_frame,
                                            frame_list_size, usb_pipetype(urb->pipe));
  
  if(new_assignment < 0) {
    if(valid_scheduling_exists(ehci_hcd, urb)) {
      /* Need to reorder everything then we can insert the new URB */
      clear_reordered_map(ehci_hcd);
      ehci_hcd->reorder_urbs_stage = EHCI_RUS_REORDER_OLD;
      reorder_transactions(ehci_hcd);

      for(i = 0; i < ehci_hcd->frame_list_size * 8; ++i) {
        ehci_hcd->micro_frame_remaining_time_periodic[i] = USEC_PER_MICRO_FRAME_FOR_PERIODIC;
#ifdef USB_REALTIME_ASYNC
        ehci_hcd->micro_frame_remaining_time_async[i] = USEC_PER_MICRO_FRAME_FOR_ASYNC;
#endif
      }
      
      for(i = 0; i < ehci_hcd->num_rt_urbs; ++i) {
        allocate_bandwidth(ehci_hcd, ehci_hcd->rt_urbs[i], ehci_hcd->new_assignments[i]);
      }
      
      /* Return the new assignment found by the reordering algorithm,
         itd_submit will call reorder right before it starts to insert
         new elements */
      return ehci_hcd->new_assignments[ehci_hcd->num_rt_urbs-1];
    }
    else {
      /* Not schedulable even if we move stuff around */
      DLOG("Not schedulable even if we move stuff around");
      ehci_hcd->num_rt_urbs--;
      return -1;
    }
  }

  allocate_bandwidth(ehci_hcd, urb, new_assignment);
  return new_assignment;
}


#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = ehci_init
};

//DEF_MODULE (usb___ehci, "EHCI driver", &mod_ops, {"usb", "pci"});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
