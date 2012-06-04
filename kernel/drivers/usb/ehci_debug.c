#include <drivers/usb/ehci_debug.h>

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
  PRINT_QH_MEMBER(ex_buf_ptr_pgs[0]);
  PRINT_QH_MEMBER(ex_buf_ptr_pgs[1]);
  PRINT_QH_MEMBER(ex_buf_ptr_pgs[2]);
  PRINT_QH_MEMBER(ex_buf_ptr_pgs[3]);
  PRINT_QH_MEMBER(ex_buf_ptr_pgs[4]);
  DLOG("Software only members:");
  PRINT_QH_MEMBER(state);


  if(print_tds) {
    qtd_t* qtd;
    qtd_t* alt_qtd = NULL;
    if(qh->current_qtd_ptr_raw > 32) {
      qtd = EHCI_QTD_PHYS_TO_VIRT(ehci_hcd, qh->current_qtd_ptr_raw);
      print_qtd_info(ehci_hcd, qtd, "\n\n\nCurrent QH");
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

