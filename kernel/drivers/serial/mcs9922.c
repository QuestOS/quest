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

/* MosChip 9922 PCIE - To - Serial Card Driver */

#include "drivers/pci/pci.h"
#include "arch/i386.h"
#include "arch/i386-percpu.h"
#include "util/printf.h"
#include "mem/physical.h"
#include "mem/virtual.h"
#include "kernel.h"

#define MCS9922_VID    0x9710
#define MCS9922_DID    0x9922

#define DEBUG_MCS9922

#ifdef DEBUG_MCS9922
#define DLOG(fmt,...) DLOG_PREFIX("MCS9922",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

//uint16 serial_port1 = 0x03F8;    /* Default COM1 */
uint16 serial_port1 = 0xDF00;    /* Default COM1 (Shoebox) */
//uint16 serial_port1 = 0xEF00;    /* Default COM1 */

extern void initialize_serial_port (void);

bool
mcs9922_init (void)
{
  uint device_index, port_num;

  if (!pci_find_device (MCS9922_VID, MCS9922_DID, 0xFF, 0xFF, 0, &device_index)) {
    return FALSE;
  }

  if (device_index == (uint)(~0)) {
    DLOG ("Unable to detect compatible device.");
    return FALSE;
  }

  if (pci_decode_bar (device_index, 0, NULL, &port_num, NULL)) {
    serial_port1 = port_num;
    DLOG ("Found Port Number: 0x%X", serial_port1);
    initialize_serial_port ();
    return TRUE;
  } else {
    DLOG ("Bar decoding failed!");
    return FALSE;
  }
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = mcs9922_init
};

DEF_MODULE (serial___mcs9922, "MosChip 9922 PCI-E serial card driver", &mod_ops, {"pci"});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
