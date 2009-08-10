/* -*- Mode: C -*- */

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

#define MAX_INT_OVERRIDES 128

uint32 IRQ_to_GSI (uint32 bus, uint32 irq);

#endif /* ifndef __ASSEMBLER__ */

#endif

/* vi: set et sw=2 sts=2: */
