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

#ifndef _APIC_H_
#define _APIC_H_

#include "types.h"
#include "smp/apic-defs.h"
#include "smp/smp.h"

uint8 LAPIC_get_physical_ID (void);
void LAPIC_set_logical_destination (uint32);
void LAPIC_enable_timer (uint8, bool, uint8);
void send_eoi (void);
void LAPIC_start_timer (uint32);
int LAPIC_send_ipi (uint32, uint32);
uint32 LAPIC_clear_error (void);
void LAPIC_set_task_priority (uint8);

#define MAX_IOAPICS MAX_CPUS

uint64 IOAPIC_read64 (uint8);
void IOAPIC_write64 (uint8, uint64);
mp_IOAPIC_info *IOAPIC_lookup (uint8);
int IOAPIC_map_GSI (uint32, uint8, uint64);
int IOAPIC_get_GSI_mapping (uint32 GSI, uint8 *vector, uint64 *flags);
uint32 IOAPIC_num_entries (void);
extern uint32 mp_ISA_bus_id;

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
