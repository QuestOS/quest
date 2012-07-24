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


void SQUELCH_UNUSED
print_itd_info(ehci_hcd_t* ehci_hcd, itd_t* itd ,char* msg);

uint32_t SQUELCH_UNUSED
print_itd_dma(ehci_hcd_t* ehci_hcd, itd_t* itd, int transaction,char* msg);

#endif // _EHCI_DEBUG_H_



