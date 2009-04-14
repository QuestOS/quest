/* Based on http://www.uruk.org/mps/, http://www.osdev.org/, and http://www.osdever.net/ */
#include "i386.h"
#include "kernel.h"
#include "smp.h"
#include "apic.h"
                    
/* CMOS write */
static inline void cmos_write(BYTE i, BYTE v) {
  /* Assuming interrupts are disabled */
  outb(i, 0x70);
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  outb(v, 0x71);
}
#define CMOS_WRITE_BYTE(x,y) cmos_write(x,y)

#define DEBUG_SMP 1

#define APIC_BROADCAST_ID 0xFF
#define MAX_CPUS          APIC_BROADCAST_ID

#define MP_READ(x)    (*((volatile DWORD *) (x)))
#define MP_WRITE(x,y) (*((volatile DWORD *) (x)) = (y))


#define EBDA_SEG_ADDR       0x40E
#define BIOS_RESET_VECTOR   0x467
#define LAPIC_ADDR_DEFAULT  0xFEE00000uL
#define IOAPIC_ADDR_DEFAULT 0xFEC00000uL
#define CMOS_RESET_CODE     0xF
#define CMOS_RESET_JUMP     0xA
#define CMOS_BASE_MEMORY    0x15

int mp_enabled=0, mp_num_cpus=1;

DWORD mp_LAPIC_addr = LAPIC_ADDR_DEFAULT;
#define MP_LAPIC_READ(x)   (*((volatile DWORD *) (mp_LAPIC_addr+(x))))
#define MP_LAPIC_WRITE(x,y) (*((volatile DWORD *) (mp_LAPIC_addr+(x))) = (y))

BYTE CPU_to_APIC[MAX_CPUS];
BYTE APIC_to_CPU[MAX_CPUS];

static BYTE checksum(BYTE *, int);
static int send_ipi(DWORD, DWORD);
static int process_mp_config(struct mp_config *);
static void add_processor(struct mp_config_processor_entry *);
static int boot_cpu(struct mp_config_processor_entry *);

int process_mp_fp (struct mp_fp *ptr) {
  struct mp_config *cfg;

  /* Sanity checks */
  if (ptr == NULL) {
    print ("No SMP support detected.\n");
    return 1;
  }
  
  if(ptr->signature != MP_FP_SIGNATURE) {
    print ("MP floating pointer structure signature invalid.\n");
    return 1;
  }

  if(ptr->length != 1) {
    print ("MP floating pointer structure reports length != 16 bytes.\n");
    return 1;
  }
  
  switch(ptr->version) {
  case 1:
    print ("Intel MP specification v1.1\n");
    break;
  case 4:
    print ("Intel MP specification v1.4\n");
    break;
  default:
    print ("Unknown MP specification reported.\n");
    return 1;
  }

  if (checksum((BYTE *)ptr, sizeof(struct mp_fp)) != 0) {
    print ("MP floating pointer structure failed checksum.\n");
    return 1;
  }

  /* Check MP config table given by floating pointer struct */
  cfg = (struct mp_config*) ptr->mpconfig_ptr;

  process_mp_config(cfg);

  return mp_num_cpus;
}

static int process_mp_config(struct mp_config *cfg) {
  int i,j;
  BYTE *ptr;

  /* Sanity checks */
  if (cfg == NULL) {
    print ("MP config pointer is NULL.\n");
    return 1;
  }

  if (cfg->signature != MP_CFG_SIGNATURE) {
    print ("MP config signature invalid.\n");
    return 1;
  }

  if (cfg->specification_revision != 1 && 
      cfg->specification_revision != 4) {
    print ("Unknown MP specification reported by MP config table.\n");
    return 1;
  }

  if(checksum((BYTE *)cfg, cfg->base_table_length) != 0) {
    print ("MP config table failed checksum.\n");
    return 1;
  }

  print("Manufacturer: ");
  for(i=0;i<8;i++) putchar(cfg->OEM_id[i]);
  
  print(" Product: ");
  for(i=0;i<12;i++) putchar(cfg->product_id[i]);
  putchar('\n');
  
  print("Entry count: ");
  putx(cfg->entry_count);

  print(" Local APIC: ");
  putx(cfg->local_APIC);
  if(cfg->local_APIC) mp_LAPIC_addr = cfg->local_APIC;
 
  print(" Extended table length: ");
  putx(cfg->extended_table_length);
  putchar('\n');

  /* Check entries */
  ptr = (BYTE *)cfg->entries;
  for(i=0;i<cfg->entry_count;i++) {
    struct mp_config_entry *entry = (struct mp_config_entry*) ptr;
    switch (*ptr) {
    case MP_CFG_TYPE_PROCESSOR:
      print ("Processor APIC id: ");
      putx (entry->processor.APIC_id);
      print (" APIC version: ");
      putx (entry->processor.APIC_version);
      if (entry->processor.flags & 1) 
        print (" (enabled)");
      else
        print (" (disabled)");
      if (entry->processor.flags & 2)
        print (" (bootstrap)");
      print ("\n");
      ptr += sizeof(struct mp_config_processor_entry);
      add_processor(&entry->processor);
      break;
    case MP_CFG_TYPE_BUS:
      print ("Bus entry id: ");
      putx (entry->bus.id);
      print (" type: ");
      for (j=0;j<6;j++) putchar(entry->bus.bus_type[j]);
      print ("\n");
      ptr += sizeof(struct mp_config_bus_entry);
      break;
    case MP_CFG_TYPE_IO_APIC:
      print ("IO APIC id: ");
      putx (entry->IO_APIC.id);
      print (" version: ");
      putx (entry->IO_APIC.version);
      if (entry->IO_APIC.flags & 1) 
        ;
      else
        print (" (disabled)");
      print (" address: ");
      putx (entry->IO_APIC.address);
      print ("\n");
      ptr += sizeof(struct mp_config_IO_APIC_entry);
      break;
    case MP_CFG_TYPE_IO_INT:
#if 0
      print ("IO interrupt type: ");
      putx (entry->IO_int.int_type);
      print ("\n");
#endif
      ptr += sizeof(struct mp_config_interrupt_entry);
      break;
    case MP_CFG_TYPE_LOCAL_INT:
#if 0
      print ("Local interrupt type: ");
      putx (entry->local_int.int_type);
      print ("\n");
#endif
      ptr += sizeof(struct mp_config_interrupt_entry);
      break;
    default:
      print ("Unknown entry type: ");
      putx (*ptr);
      print (" at address: ");
      putx ((DWORD)ptr);
      putchar ('\n');
      return 1;      
    }
  }

  return mp_num_cpus;
}

static BYTE checksum(BYTE *ptr, int length) {
  BYTE sum = 0;
  while (length-- > 0) sum += *ptr++;
  return sum;
}

static int send_ipi(DWORD dest, DWORD v) {
  int to, send_status;

  MP_LAPIC_WRITE(LAPIC_ICR+0x10, dest << 24);
  MP_LAPIC_WRITE(LAPIC_ICR, v);

  to = 0;
  do {
    send_status = MP_LAPIC_READ(LAPIC_ICR) & LAPIC_ICR_STATUS_PEND;
  } while (send_status && (to++ < 1000));

  return (to < 1000);
}


extern BYTE patch_code_start[];
extern BYTE patch_code_end[];
extern BYTE status_code[];
extern BYTE initial_gdt[];

static int boot_cpu(struct mp_config_processor_entry *proc) {
  BYTE apic_id = proc->APIC_id;
  int success = 1;
  volatile int to;
  DWORD bootaddr, accept_status;
  DWORD bios_reset_vector = BIOS_RESET_VECTOR; /* identity mapped */

  /* Set up the boot code for the APs */
#define TEST_BOOTED(x) (*((volatile DWORD *)(x+status_code-patch_code_start)))

  bootaddr = 0x00070000uL;           /* identity mapped */
  TEST_BOOTED(bootaddr) = 0;
  memcpy((BYTE *)bootaddr, patch_code_start, patch_code_end - patch_code_start);

  /* CPU startup sequence */
  /******************************************************************************
   * CMOS_WRITE_BYTE(CMOS_RESET_CODE, CMOS_RESET_JUMP);                         *
   * *((volatile unsigned *) bios_reset_vector) = ((bootaddr & 0xFF000) << 12); *
   ******************************************************************************/

  /* clear APIC error register */
  MP_LAPIC_WRITE(LAPIC_ESR, 0);
  accept_status = MP_LAPIC_READ(LAPIC_ESR);

  /* assert INIT interprocessor interrupt */
  send_ipi(apic_id, LAPIC_ICR_TM_LEVEL | LAPIC_ICR_LEVELASSERT | LAPIC_ICR_DM_INIT);

  /* de-assert INIT IPI */
  send_ipi(apic_id, LAPIC_ICR_TM_LEVEL | LAPIC_ICR_DM_INIT);

  /* Send start-up IPIs if not old version */
  if (proc->APIC_version >= APIC_VER_NEW) {
    int i;
    for (i=1; i <= 2; i++) {
      /* Bochs starts it @ INIT-IPI and the AP goes into p-mode which is
       * bad if I then deliver a STARTUP-IPI because that loads the CS
       * register with 0x7000 which is of course invalid in p-mode.  So
       * I added the test. */
      if (TEST_BOOTED(bootaddr)) break;
      send_ipi(apic_id, LAPIC_ICR_DM_SIPI | ((bootaddr >> 12) & 0xFF));
    }
  }
  
#define LOOPS_TO_WAIT 10000000
  /* Check for successful start */
  to = 0;
  while (!TEST_BOOTED(bootaddr) && to++ < LOOPS_TO_WAIT) {
    asm volatile("pause");
  }
  if (to >= LOOPS_TO_WAIT) {
    print("Processor ");
    putx(apic_id);
    print(" not responding.\n");
    success = 0;
  } else {
    print("Processor ");
    putx(apic_id);
    print(" BOOTED!\n");
  }

  /* cleanup */
  MP_LAPIC_WRITE(LAPIC_ESR, 0);
  accept_status = MP_LAPIC_READ(LAPIC_ESR);

  /************************************************
   * CMOS_WRITE_BYTE(CMOS_RESET_CODE, 0);         *
   * *((volatile DWORD *) bios_reset_vector) = 0; *
   ************************************************/
  
  return success;
}

static void add_processor(struct mp_config_processor_entry *proc) {
  BYTE apic_id = proc->APIC_id;

  if(!(proc->flags & 1)) return; /* disabled processor */
  if(proc->flags & 2) return;    /* bootstrap processor */
  
  if(boot_cpu(proc)) {
    CPU_to_APIC[mp_num_cpus] = apic_id;
    APIC_to_CPU[apic_id] = mp_num_cpus;
    mp_num_cpus++;
  }
}


/* http://forum.osdev.org/viewtopic.php?f=1&t=18648&hilit=smp */
