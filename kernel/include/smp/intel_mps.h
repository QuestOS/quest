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

#ifndef _INTEL_MPS_H_
#define _INTEL_MPS_H_

#include "types.h"

#define MP_FP_SIGNATURE ('_' | ('M'<<8) | ('P'<<16) | ('_'<<24))
#define MP_CFG_SIGNATURE ('P' | ('C'<<8) | ('M'<<16) | ('P'<<24))

#define MP_CFG_TYPE_PROCESSOR 0
#define MP_CFG_TYPE_BUS       1
#define MP_CFG_TYPE_IO_APIC   2
#define MP_CFG_TYPE_IO_INT    3
#define MP_CFG_TYPE_LOCAL_INT 4

struct mp_fp
{
  uint32 signature;
  uint32 mpconfig_ptr;
  uint8 length;
  uint8 version;
  uint8 checksum;
  uint8 features[5];
} PACKED;

struct mp_config_processor_entry
{
  /* type == 0 for processor */
  uint8 type;
  uint8 APIC_id;
  uint8 APIC_version;
  uint8 flags;                  /* bits: 0 -> CPU enabled?
                                 *       1 -> is BSP?
                                 *  others -> unused      */
  uint32 CPU_signature;
  uint32 CPU_features;
  uint8 reserved[8];
} PACKED;

struct mp_config_bus_entry
{
  /* type == 1 for bus */
  uint8 type;
  uint8 id;
  uint8 bus_type[6];
} PACKED;

struct mp_config_IO_APIC_entry
{
  /* type == 2 for IO APIC */
  uint8 type;
  uint8 id;
  uint8 version;
  uint8 flags;                  /* bits: 0 -> IO APIC enabled? 
                                 *  others -> unused      */
  uint32 address;
} PACKED;

struct mp_config_interrupt_entry
{
  /* type == 3 or 4 for IO or local interrupt */
  uint8 type;
  uint8 int_type;
  uint16 flags;
  uint8 source_bus_id;
  uint8 source_bus_irq;
  uint8 dest_APIC_id;
  uint8 dest_APIC_intin;
} PACKED;

struct mp_config_entry
{
  union
  {
    struct mp_config_processor_entry processor;
    struct mp_config_bus_entry bus;
    struct mp_config_IO_APIC_entry IO_APIC;
    struct mp_config_interrupt_entry IO_int;
    struct mp_config_interrupt_entry local_int;
  } PACKED;
} PACKED;

struct mp_config
{
  uint32 signature;
  uint16 base_table_length;
  uint8 specification_revision;
  uint8 checksum;
  uint8 OEM_id[8];
  uint8 product_id[12];
  uint32 OEM_table_ptr;
  uint16 OEM_table_size;
  uint16 entry_count;
  uint32 local_APIC;
  uint16 extended_table_length;
  uint8 extended_table_checksum;
  uint8 reserved;
  struct mp_config_entry entries[];
} PACKED;

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
