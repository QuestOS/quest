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

#include <drivers/usb/ehci_debug.h>
#include <drivers/usb/ehci_mem.h>

/*
 * Used for debug purposes, prints state of EHCI registers through
 * DLOGV
 */
void SQUELCH_UNUSED
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

void SQUELCH_UNUSED
print_qtd_info(ehci_hcd_t* ehci, qtd_t* qtd, char* msg)
{
#ifdef DEBUG_EHCI_VERBOSE

#define PRINT_QTD_MEMBER(member) DLOGV(#member ": 0x%X", qtd->member)

  DLOGV("%s", msg);
  DLOGV("This QTD: 0x%X", EHCI_QTD_VIRT_TO_PHYS(ehci,qtd));
  PRINT_QTD_MEMBER(next_pointer_raw);
  PRINT_QTD_MEMBER(alt_pointer_raw);
  PRINT_QTD_MEMBER(token);
  PRINT_QTD_MEMBER(status);
  PRINT_QTD_MEMBER(pid_code);
  PRINT_QTD_MEMBER(error_counter);
  PRINT_QTD_MEMBER(curr_page);
  PRINT_QTD_MEMBER(ioc);
  PRINT_QTD_MEMBER(total_bytes_to_transfer);
  PRINT_QTD_MEMBER(data_toggle);
  PRINT_QTD_MEMBER(buffer_page[0]);
  PRINT_QTD_MEMBER(buffer_page[1]);
  PRINT_QTD_MEMBER(buffer_page[2]);
  PRINT_QTD_MEMBER(buffer_page[3]);
  PRINT_QTD_MEMBER(buffer_page[4]);
  PRINT_QTD_MEMBER(ex_buf_ptr_pgs[0]);
  PRINT_QTD_MEMBER(ex_buf_ptr_pgs[1]);
  PRINT_QTD_MEMBER(ex_buf_ptr_pgs[2]);
  PRINT_QTD_MEMBER(ex_buf_ptr_pgs[3]);
  PRINT_QTD_MEMBER(ex_buf_ptr_pgs[4]);

#undef PRINT_QTD_MEMBER
  
#endif  
}

/*
 * Used for debug purposes, prints state of qh
 */

void SQUELCH_UNUSED
print_qh_info(ehci_hcd_t* ehci_hcd, qh_t* qh, bool print_tds ,char* msg)
{
#ifdef DEBUG_EHCI_VERBOSE
#define PRINT_QH_MEMBER(member) DLOGV(#member ": 0x%X", qh->member)
#define QUEUE_HEAD_PRINT_VERBOSE
  int qtd_count = 0;
  
  DLOGV("**************************************************************************");
  DLOGV("%s", msg);

  PRINT_QH_MEMBER(horizontalPointer.raw);
  PRINT_QH_MEMBER(hw_info1);
#ifdef QUEUE_HEAD_PRINT_VERBOSE
  PRINT_QH_MEMBER(device_address);
  PRINT_QH_MEMBER(inactive_on_next);
  PRINT_QH_MEMBER(endpoint_num);
  PRINT_QH_MEMBER(endpoint_speed);
  PRINT_QH_MEMBER(data_toggle_control);
  PRINT_QH_MEMBER(head_reclam);
  PRINT_QH_MEMBER(max_packet_length);
  PRINT_QH_MEMBER(cnt_endpoint);
  PRINT_QH_MEMBER(nak_cnt_reload);
#endif
  PRINT_QH_MEMBER(hw_info2);
#ifdef QUEUE_HEAD_PRINT_VERBOSE
  
  PRINT_QH_MEMBER(interrupt_sched_mask);
  PRINT_QH_MEMBER(split_comp_mask);
  PRINT_QH_MEMBER(hub_addr);
  PRINT_QH_MEMBER(port_num);
  PRINT_QH_MEMBER(mult);
#endif
  PRINT_QH_MEMBER(current_qtd_ptr_raw);
  PRINT_QH_MEMBER(next_qtd_ptr_raw);
  PRINT_QH_MEMBER(alt_qtd_ptr_raw);
  PRINT_QH_MEMBER(qtd_token_raw);
  PRINT_QH_MEMBER(data_toggle);
  PRINT_QH_MEMBER(raw5);
  PRINT_QH_MEMBER(raw6);
  PRINT_QH_MEMBER(raw7);
  PRINT_QH_MEMBER(raw8);
  PRINT_QH_MEMBER(raw9);
  PRINT_QH_MEMBER(ex_buf_ptr_pgs[0]);
  PRINT_QH_MEMBER(ex_buf_ptr_pgs[1]);
  PRINT_QH_MEMBER(ex_buf_ptr_pgs[2]);
  PRINT_QH_MEMBER(ex_buf_ptr_pgs[3]);
  PRINT_QH_MEMBER(ex_buf_ptr_pgs[4]);
  DLOG("Software only members:");
  PRINT_QH_MEMBER(state);

#undef PRINT_QH_MEMBER


  if(print_tds) {
    qtd_t* qtd;
    qtd_t* alt_qtd = NULL;
    if(qh->current_qtd_ptr_raw > 32) {
      qtd = EHCI_QTD_PHYS_TO_VIRT(ehci_hcd, qh->current_qtd_ptr_raw);
      print_qtd_info(ehci_hcd, qtd, "\n\n\nCurrent QTD");
    }
    qtd = NULL;
    if(qh->next_qtd_ptr_raw > 32) {
      qtd = EHCI_QTD_PHYS_TO_VIRT(ehci_hcd, qh->next_qtd_ptr_raw);
    }
    if(qh->alt_qtd_ptr_raw > 32) {
      
      alt_qtd = EHCI_QTD_PHYS_TO_VIRT(ehci_hcd, qh->alt_qtd_ptr_raw);
    }
    while(qtd != NULL) {
      DLOGV("\n\n\nQTD %d info", qtd_count);
      print_qtd_info(ehci_hcd, qtd, "");
      if(alt_qtd != NULL) {
        DLOGV("Alt qtd %d info:", qtd_count);
        print_qtd_info(ehci_hcd, alt_qtd, "");
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

void SQUELCH_UNUSED
print_itd_info(ehci_hcd_t* ehci_hcd, itd_t* itd ,char* msg)
{
  DLOGV("%s", msg);
  
#define PRINT_ITD_MEMBER(member) DLOGV(#member ": 0x%X", itd->member)

  //#define ITD_PRINT_VERBOSE

  DLOG("This ITD = 0x%p", itd);
  PRINT_ITD_MEMBER(next_link_pointer);
  PRINT_ITD_MEMBER(transaction[0]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction[0].status);
  PRINT_ITD_MEMBER(transaction[0].length);
  PRINT_ITD_MEMBER(transaction[0].ioc);
  PRINT_ITD_MEMBER(transaction[0].page_selector);
  PRINT_ITD_MEMBER(transaction[0].offset);
#endif
  PRINT_ITD_MEMBER(transaction[1]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction[1].status);
  PRINT_ITD_MEMBER(transaction[1].length);
  PRINT_ITD_MEMBER(transaction[1].ioc);
  PRINT_ITD_MEMBER(transaction[1].page_selector);
  PRINT_ITD_MEMBER(transaction[1].offset);
#endif
  PRINT_ITD_MEMBER(transaction[2]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction[2].status);
  PRINT_ITD_MEMBER(transaction[2].length);
  PRINT_ITD_MEMBER(transaction[2].ioc);
  PRINT_ITD_MEMBER(transaction[2].page_selector);
  PRINT_ITD_MEMBER(transaction[2].offset);
#endif
  PRINT_ITD_MEMBER(transaction[3]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction[3].status);
  PRINT_ITD_MEMBER(transaction[3].length);
  PRINT_ITD_MEMBER(transaction[3].ioc);
  PRINT_ITD_MEMBER(transaction[3].page_selector);
  PRINT_ITD_MEMBER(transaction[3].offset);
#endif
  PRINT_ITD_MEMBER(transaction[4]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction[4].status);
  PRINT_ITD_MEMBER(transaction[4].length);
  PRINT_ITD_MEMBER(transaction[4].ioc);
  PRINT_ITD_MEMBER(transaction[4].page_selector);
  PRINT_ITD_MEMBER(transaction[4].offset);
#endif
  PRINT_ITD_MEMBER(transaction[5]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction[5].status);
  PRINT_ITD_MEMBER(transaction[5].length);
  PRINT_ITD_MEMBER(transaction[5].ioc);
  PRINT_ITD_MEMBER(transaction[5].page_selector);
  PRINT_ITD_MEMBER(transaction[5].offset);
#endif
  PRINT_ITD_MEMBER(transaction[6]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction[6].status);
  PRINT_ITD_MEMBER(transaction[6].length);
  PRINT_ITD_MEMBER(transaction[6].ioc);
  PRINT_ITD_MEMBER(transaction[6].page_selector);
  PRINT_ITD_MEMBER(transaction[6].offset);
#endif
  PRINT_ITD_MEMBER(transaction[7]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction[7].status);
  PRINT_ITD_MEMBER(transaction[7].length);
  PRINT_ITD_MEMBER(transaction[7].ioc);
  PRINT_ITD_MEMBER(transaction[7].page_selector);
  PRINT_ITD_MEMBER(transaction[7].offset);
#endif
  #ifdef ITD_PRINT_VERBOSE
  DLOGV("endpoint = 0x%X", (itd->buf_ptr[0] & 0x00000F00) >> 8);
  DLOGV("address = 0x%X", itd->buf_ptr[0] & 0x000000FF);
  DLOGV("max packet size = %d", itd->buf_ptr[1] & 0x000007FF);
  DLOGV("I/O = 0x%X", !!(itd->buf_ptr[1] | ITD_INPUT));
  DLOGV("mult = 0x%X", itd->buf_ptr[2] & 0x3);
#endif
  PRINT_ITD_MEMBER(buf_ptr[0]);
  PRINT_ITD_MEMBER(buf_ptr[1]);
  PRINT_ITD_MEMBER(buf_ptr[2]);
  PRINT_ITD_MEMBER(buf_ptr[3]);
  PRINT_ITD_MEMBER(buf_ptr[4]);
  PRINT_ITD_MEMBER(buf_ptr[5]);
  PRINT_ITD_MEMBER(buf_ptr[6]);
  PRINT_ITD_MEMBER(ex_buf_ptr_pgs[0]);
  PRINT_ITD_MEMBER(ex_buf_ptr_pgs[1]);
  PRINT_ITD_MEMBER(ex_buf_ptr_pgs[2]);
  PRINT_ITD_MEMBER(ex_buf_ptr_pgs[3]);
  PRINT_ITD_MEMBER(ex_buf_ptr_pgs[4]);
  PRINT_ITD_MEMBER(ex_buf_ptr_pgs[5]);
  PRINT_ITD_MEMBER(ex_buf_ptr_pgs[6]);
  PRINT_ITD_MEMBER(frame_index);

#if 0
    PRINT_ITD_MEMBER(transaction_backup[0]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction_backup[0].status);
  PRINT_ITD_MEMBER(transaction_backup[0].length);
  PRINT_ITD_MEMBER(transaction_backup[0].ioc);
  PRINT_ITD_MEMBER(transaction_backup[0].page_selector);
  PRINT_ITD_MEMBER(transaction_backup[0].offset);
#endif
  
    PRINT_ITD_MEMBER(transaction_backup[1]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction_backup[1].status);
  PRINT_ITD_MEMBER(transaction_backup[1].length);
  PRINT_ITD_MEMBER(transaction_backup[1].ioc);
  PRINT_ITD_MEMBER(transaction_backup[1].page_selector);
  PRINT_ITD_MEMBER(transaction_backup[1].offset);
#endif
    PRINT_ITD_MEMBER(transaction_backup[2]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction_backup[2].status);
  PRINT_ITD_MEMBER(transaction_backup[2].length);
  PRINT_ITD_MEMBER(transaction_backup[2].ioc);
  PRINT_ITD_MEMBER(transaction_backup[2].page_selector);
  PRINT_ITD_MEMBER(transaction_backup[2].offset);
#endif
    PRINT_ITD_MEMBER(transaction_backup[3]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction_backup[3].status);
  PRINT_ITD_MEMBER(transaction_backup[3].length);
  PRINT_ITD_MEMBER(transaction_backup[3].ioc);
  PRINT_ITD_MEMBER(transaction_backup[3].page_selector);
  PRINT_ITD_MEMBER(transaction_backup[3].offset);
#endif
    PRINT_ITD_MEMBER(transaction_backup[4]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction_backup[4].status);
  PRINT_ITD_MEMBER(transaction_backup[4].length);
  PRINT_ITD_MEMBER(transaction_backup[4].ioc);
  PRINT_ITD_MEMBER(transaction_backup[4].page_selector);
  PRINT_ITD_MEMBER(transaction_backup[4].offset);
#endif
    PRINT_ITD_MEMBER(transaction_backup[5]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction_backup[5].status);
  PRINT_ITD_MEMBER(transaction_backup[5].length);
  PRINT_ITD_MEMBER(transaction_backup[5].ioc);
  PRINT_ITD_MEMBER(transaction_backup[5].page_selector);
  PRINT_ITD_MEMBER(transaction_backup[5].offset);
#endif
    PRINT_ITD_MEMBER(transaction_backup[6]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction_backup[6].status);
  PRINT_ITD_MEMBER(transaction_backup[6].length);
  PRINT_ITD_MEMBER(transaction_backup[6].ioc);
  PRINT_ITD_MEMBER(transaction_backup[6].page_selector);
  PRINT_ITD_MEMBER(transaction_backup[6].offset);
#endif
    PRINT_ITD_MEMBER(transaction_backup[7]);
#ifdef ITD_PRINT_VERBOSE
  PRINT_ITD_MEMBER(transaction_backup[7].status);
  PRINT_ITD_MEMBER(transaction_backup[7].length);
  PRINT_ITD_MEMBER(transaction_backup[7].ioc);
  PRINT_ITD_MEMBER(transaction_backup[7].page_selector);
  PRINT_ITD_MEMBER(transaction_backup[7].offset);
#endif

#endif

#undef PRINT_ITD_MEMBER
  

}


uint32_t SQUELCH_UNUSED
print_itd_dma(ehci_hcd_t* ehci_hcd, itd_t* itd, int transaction, char* msg)
{
  DLOGV("%s", msg);
  if(itd->transaction[transaction].raw & ITD_ACTIVE) {
    int buffer_page = (itd->transaction[transaction].raw >> 12) & 0x7;
    uint32_t address = itd->transaction[transaction].raw & 0x0FFF;
    address = address | (itd->buf_ptr[buffer_page] & 0xFFFFF000);
    DLOGV("Transaction %d is active with dma address 0x%p", transaction, address);
    return address;
  }
  else {
    DLOGV("Transaction %d is not active", transaction);
    return 0;
  }
}


void SQUELCH_UNUSED
print_periodic_list(ehci_hcd_t* ehci_hcd, bool first_row_only,
		    bool print_qhs, bool print_itds)
{
  int i;
  frm_lst_lnk_ptr_t* frame_list = ehci_hcd->frame_list;
  for(i = 0; i < ehci_hcd->frame_list_size; ++i) {
    if(!first_row_only) {
      qh_t* temp_qh;
      itd_t* temp_itd;
      frm_lst_lnk_ptr_t* current = &frame_list[i];
      while(current->raw >= 32) {
	switch(current->type) {
          case TYPE_ITD:
            temp_itd = EHCI_ITD_PHYS_TO_VIRT(ehci_hcd, current->raw & (~0x1F));
	    if(print_itds) print_itd_info(ehci_hcd, temp_itd, "In periodic list");
            current = &temp_itd->next_link_pointer;
	    break;
	    
          case TYPE_QH:
            temp_qh = EHCI_QH_PHYS_TO_VIRT(ehci_hcd, current->raw & (~0x1F));
	    if(print_qhs) print_qh_info(ehci_hcd, temp_qh, TRUE, "In periodic list");
	    current = &temp_qh->horizontalPointer;
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
	DLOG("----------- 0x%p",  current->raw);
      }
    }
  }
}
