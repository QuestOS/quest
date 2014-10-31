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

#ifndef _SMP_H_
#define _SMP_H_

#include "types.h"
#include "kernel.h"

#define MP_BOOTADDR 0x70000

#ifndef __ASSEMBLER__

typedef struct
{
  uint8 id;
  uint32 address, startGSI, numGSIs;
} mp_IOAPIC_info;

typedef struct
{
  uint32 src_bus, src_IRQ, dest_GSI;
} mp_int_override;

extern uint32 cpu_bus_freq;
extern uint64 tsc_freq;

int smp_init (void);
void smp_secondary_init (void);
void smp_enable_scheduling (void);
uint32 smp_boot_cpu (uint8 apic_id, uint8 version);

extern uint8 CPU_to_APIC[MAX_CPUS];
extern uint8 APIC_to_CPU[MAX_CPUS];
       
extern uint32 mp_num_cpus;
extern bool mp_ISA_PC;
extern bool mp_apic_mode;
extern bool ioapic_exists;

#define MAX_INT_OVERRIDES 128

uint32 IRQ_to_GSI (uint32 bus, uint32 irq);

#endif /* ifndef __ASSEMBLER__ */

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
