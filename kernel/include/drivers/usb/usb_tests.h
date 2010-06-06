#ifndef _USB_TESTS_H
#define _USB_TESTS_H

#include "drivers/pci/pci.h"
#include "types.h"
#include "arch/i386.h"
#include "drivers/usb/uhci.h"
#include "drivers/usb/usb.h"
#include "drivers/usb/uvc.h"

extern void show_usb_regs (int bus, int dev, int func);
extern void control_transfer_test (void);

#endif

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
