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

struct mp_fp {
  DWORD signature;
  DWORD mpconfig_ptr;
  BYTE  length;
  BYTE  version;
  BYTE  checksum;
  BYTE  features[5];
} PACKED;

struct mp_config_processor_entry {
  /* type == 0 for processor */
  BYTE  type;
  BYTE  APIC_id;
  BYTE  APIC_version;
  BYTE  flags;                  /* bits: 0 -> CPU enabled?
                                 *       1 -> is BSP?
                                 *  others -> unused      */
  DWORD CPU_signature;
  DWORD CPU_features;
  BYTE  reserved[8];
} PACKED;

struct mp_config_bus_entry {
 /* type == 1 for bus */
  BYTE  type;
  BYTE  id;
  BYTE  bus_type[6];
} PACKED;

struct mp_config_IO_APIC_entry {
  /* type == 2 for IO APIC */
  BYTE  type;
  BYTE  id;
  BYTE  version;
  BYTE  flags;                  /* bits: 0 -> IO APIC enabled? 
                                 *  others -> unused      */
  DWORD address;
} PACKED;

struct mp_config_interrupt_entry {
  /* type == 3 or 4 for IO or local interrupt */
  BYTE  type;
  BYTE  int_type;
  WORD  flags;
  BYTE  source_bus_id;
  BYTE  source_bus_irq;
  BYTE  dest_APIC_id;
  BYTE  dest_APIC_intin;
} PACKED;

struct mp_config_entry {
  union {
    struct mp_config_processor_entry processor;
    struct mp_config_bus_entry       bus;
    struct mp_config_IO_APIC_entry   IO_APIC;
    struct mp_config_interrupt_entry IO_int;
    struct mp_config_interrupt_entry local_int;
  } PACKED;
} PACKED;

struct mp_config {
  DWORD signature;
  WORD  base_table_length;
  BYTE  specification_revision;
  BYTE  checksum;
  BYTE  OEM_id[8];
  BYTE  product_id[12];
  DWORD OEM_table_ptr;
  WORD  OEM_table_size;
  WORD  entry_count;
  DWORD local_APIC;
  WORD  extended_table_length;
  BYTE  extended_table_checksum;
  BYTE  reserved;
  struct mp_config_entry entries[];
} PACKED;

extern unsigned long cpu_bus_freq;

int smp_init(void);
void smp_enable(void);
QWORD IOAPIC_read64(BYTE);
void IOAPIC_write64(BYTE, QWORD);
BYTE LAPIC_get_physical_ID(void);
void send_eoi(void);
int send_ipi(DWORD, DWORD);

#endif  /* __ASSEMBLER__ */

#endif
