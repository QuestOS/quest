#ifndef _SMP_H_
#define _SMP_H_

#include "types.h"

#define MP_FP_SIGNATURE ('_' | ('M'<<8) | ('P'<<16) | ('_'<<24))
#define MP_CFG_SIGNATURE ('P' | ('C'<<8) | ('M'<<16) | ('P'<<24))

#define MP_CFG_TYPE_PROCESSOR 0
#define MP_CFG_TYPE_BUS       1
#define MP_CFG_TYPE_IO_APIC   2
#define MP_CFG_TYPE_IO_INT    3
#define MP_CFG_TYPE_LOCAL_INT 4

#define MP_BOOTADDR 0x70000

#ifndef __ASSEMBLER__

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
void smp_enable (void);
void smp_enable_scheduling (void);
uint64 IOAPIC_read64 (uint8);
void IOAPIC_write64 (uint8, uint64);
uint8 LAPIC_get_physical_ID (void);
void send_eoi (void);
void LAPIC_start_timer (uint32);
int send_ipi (uint32, uint32);
mp_IOAPIC_info *IOAPIC_lookup (uint8);
uint32 IRQ_to_GSI (uint32 bus, uint32 irq);
int IOAPIC_map_GSI (uint32 GSI, uint8 vec, uint64 flags);
extern uint32 mp_ISA_bus_id;


static inline uint8
checksum (uint8 * ptr, int length)
{
  uint8 sum = 0;
  while (length-- > 0)
    sum += *ptr++;
  return sum;
}

#endif /* ifndef __ASSEMBLER__ */

#endif
