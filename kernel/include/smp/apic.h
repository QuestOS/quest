/* -*- Mode: C; c-file-style: "gnu"; c-basic-offset: 2; indent-tabs-mode: nil -*- */

#ifndef _APIC_H_
#define _APIC_H_

#include "types.h"
#include "smp/apic-defs.h"

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
uint32 IOAPIC_num_entries (void);
extern uint32 mp_ISA_bus_id;

#endif 

/* vi: set et sw=2 sts=2: */
