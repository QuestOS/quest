#ifndef _EHCI_DEBUG_H_
#define _EHCI_DEBUG_H_

#define DEBUG_EHCI
#define DEBUG_EHCI_VERBOSE

#include <drivers/usb/ehci.h>
#include <util/printf.h>
#include <util/cassert.h>


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


void SQUELCH_UNUSED
print_caps_and_regs_info(ehci_hcd_t* ehci_hcd, char* msg);


void SQUELCH_UNUSED
print_qtd_info(ehci_hcd_t* ehci, qtd_t* qtd, char* msg);


  
void SQUELCH_UNUSED
print_qh_info(ehci_hcd_t* ehci_hcd, qh_t* qh, bool print_tds ,char* msg);


#endif // _EHCI_DEBUG_H_



