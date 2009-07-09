/* Based on:
 * http://www.uruk.org/mps/, http://www.osdev.org/, http://www.osdever.net/ 
 * and the Intel Multiprocessing Specification v1.4.
 */
#include "i386.h"
#include "kernel.h"
#include "smp.h"
#include "apic.h"
#include "spinlock.h"
#include "acpi/include/acpi.h"
#include "printf.h"
                    
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
//#define MAX_CPUS          APIC_BROADCAST_ID

#define MP_READ(x)    (*((volatile DWORD *) (x)))
#define MP_WRITE(x,y) (*((volatile DWORD *) (x)) = (y))


#define EBDA_SEG_ADDR       0x40E
#define BIOS_RESET_VECTOR   0x467
#define LAPIC_ADDR_DEFAULT  0xFEE00000uL
#define IOAPIC_ADDR_DEFAULT 0xFEC00000uL
#define CMOS_RESET_CODE     0xF
#define CMOS_RESET_JUMP     0xA
#define CMOS_BASE_MEMORY    0x15

volatile int mp_enabled=0, mp_num_cpus=1;

DWORD mp_LAPIC_addr = LAPIC_ADDR_DEFAULT;
#define MP_LAPIC_READ(x)   (*((volatile DWORD *) (mp_LAPIC_addr+(x))))
#define MP_LAPIC_WRITE(x,y) (*((volatile DWORD *) (mp_LAPIC_addr+(x))) = (y))

DWORD mp_IOAPIC_addr = IOAPIC_ADDR_DEFAULT;
#define MP_IOAPIC_READ(x)   (*((volatile DWORD *) (mp_IOAPIC_addr+(x))))
#define MP_IOAPIC_WRITE(x,y) (*((volatile DWORD *) (mp_IOAPIC_addr+(x))) = (y))

/* Mapping from CPU # to APIC ID */
BYTE CPU_to_APIC[MAX_CPUS];
BYTE APIC_to_CPU[MAX_CPUS];

static int mp_timer_IOAPIC_irq = 0;
static int mp_timer_IOAPIC_id = 0;
static int mp_ISA_bus_id = 0;

static BYTE checksum(BYTE *, int);
static int process_mp_fp(struct mp_fp *);
static int process_mp_config(struct mp_config *);
static int add_processor(struct mp_config_processor_entry *);
int boot_cpu(struct mp_config_processor_entry *);
static struct mp_fp *probe_mp_fp(DWORD, DWORD);
static void smp_setup_LAPIC_timer(void);

#define ACPI_MAX_INIT_TABLES 16
static ACPI_TABLE_DESC TableArray[ACPI_MAX_INIT_TABLES];

/* Returns number of CPUs successfully booted. */
int smp_init(void) {
  struct mp_fp *ptr;
  ACPI_STATUS status;
  status = AcpiInitializeTables(TableArray, ACPI_MAX_INIT_TABLES, TRUE);
  if (status == AE_OK) {
    ACPI_TABLE_MADT *madt;
    ACPI_TABLE_FADT *fadt;
    com1_puts("AcpiInitializeTables OK\n");
    if(AcpiGetTable(ACPI_SIG_MADT, 0, (ACPI_TABLE_HEADER **)&madt) == AE_OK) {
      BYTE *ptr, *lim = (BYTE *)madt + madt->Header.Length;
      com1_puts("madt=");
      com1_putx((unsigned)madt);
      com1_puts(" len=");
      com1_putx(madt->Header.Length);
      com1_puts(" LAPIC=");
      com1_putx(madt->Address);
      com1_putc('\n');
      ptr = (BYTE *)madt + sizeof(ACPI_TABLE_MADT);
      while (ptr < lim) {
        switch(((ACPI_SUBTABLE_HEADER *)ptr)->Type) {
        case ACPI_MADT_TYPE_LOCAL_APIC: {
          ACPI_MADT_LOCAL_APIC *sub = (ACPI_MADT_LOCAL_APIC *)ptr;
          com1_puts("Processor=");
          com1_putx(sub->ProcessorId);
          com1_puts(" APIC ID=");
          com1_putx(sub->ProcessorId);
          if(sub->LapicFlags) com1_puts(" (enabled)");
          com1_putc('\n');
          break;
        }
        case ACPI_MADT_TYPE_IO_APIC: {
          ACPI_MADT_IO_APIC *sub = (ACPI_MADT_IO_APIC *)ptr;
          com1_puts("IO_APIC ID=");
          com1_putx(sub->Id);
          com1_puts(" ADDR=");
          com1_putx(sub->Address);
          com1_puts(" IRQBASE=");
          com1_putx(sub->GlobalIrqBase);
          com1_putc('\n');
          break;
        }
        case ACPI_MADT_TYPE_INTERRUPT_OVERRIDE: {
          ACPI_MADT_INTERRUPT_OVERRIDE *sub = (ACPI_MADT_INTERRUPT_OVERRIDE *)ptr;
          com1_puts("OVERRIDE: BUS=");
          com1_putx(sub->Bus);
          com1_puts(" SourceIRQ=");
          com1_putx(sub->SourceIrq);
          com1_puts(" GlobalIRQ=");
          com1_putx(sub->GlobalIrq);
          com1_putc('\n');
          break;
        }
        default:
          break;
        }
        ptr += ((ACPI_SUBTABLE_HEADER *)ptr)->Length;
      } 
    } else com1_puts("AcpiGetTable MADT FAILED\n");
    if(AcpiGetTable(ACPI_SIG_FADT, 0, (ACPI_TABLE_HEADER **)&fadt) == AE_OK) {
      com1_puts("FADT: ");
      com1_putx((unsigned)fadt);
      com1_putc(' ');
      com1_putx(fadt->BootFlags);
      if(fadt->BootFlags & ACPI_FADT_LEGACY_DEVICES)
        com1_puts(" HAS_LEGACY_DEVICES");
      if(fadt->BootFlags & ACPI_FADT_8042)
        com1_puts(" HAS_KBD_8042");
      if(fadt->BootFlags & ACPI_FADT_NO_VGA)
        com1_puts(" NO_VGA_PROBING");
      com1_putc('\n');
    } else com1_puts("AcpiGetTable FADT FAILED\n");
  } else 
    com1_puts("AcpiInitializeTables FAILED\n");
  
  
  if       ((ptr = probe_mp_fp(0x9F800, 0xA0000)));
  else if  ((ptr = probe_mp_fp(0x0040E, 0x0140E)));
  else if  ((ptr = probe_mp_fp(0xE0000, 0xFFFFF)));
  else return 1;                /* assume uniprocessor */

  mp_num_cpus = process_mp_fp (ptr);

  if (mp_num_cpus > 1) {
    int phys_id, log_dest;
    MP_LAPIC_WRITE(LAPIC_TPR, 0x00);
    MP_LAPIC_WRITE(LAPIC_LVTT, 0x10000); /* disable timer int */
    MP_LAPIC_WRITE(LAPIC_LVTPC, 0x10000); /* disable perf ctr int */
    MP_LAPIC_WRITE(LAPIC_LVT0, 0x08700);  /* enable normal external ints */
    MP_LAPIC_WRITE(LAPIC_LVT1, 0x00400);  /* enable NMI */
    MP_LAPIC_WRITE(LAPIC_LVTE, 0x10000);  /* disable error ints */
    MP_LAPIC_WRITE(LAPIC_SPIV, 0x0010F);  /* enable APIC: spurious vector = 0xF */

    /* be sure: */
    MP_LAPIC_WRITE(LAPIC_LVT1, 0x00400);  /* enable NMI */
    MP_LAPIC_WRITE(LAPIC_LVTE, 0x10000);  /* disable error ints */

    phys_id = (MP_LAPIC_READ(LAPIC_ID) >> 0x18) & 0xF;

    /* setup a logical destination address */
    log_dest = 0x01000000 << phys_id;
    MP_LAPIC_WRITE(LAPIC_LDR, log_dest); /* write to logical destination reg */
    MP_LAPIC_WRITE(LAPIC_DFR, -1);       /* use 'flat model' destination format */

    smp_setup_LAPIC_timer();

    MP_LAPIC_WRITE(LAPIC_LVTT, 0x3e); /* enable LAPIC timer int: vector=0x3e 
                                       * one-shot mode. */
    MP_LAPIC_WRITE(LAPIC_TDCR, 0x0B); /* set LAPIC timer divisor to 1 */
    MP_LAPIC_WRITE(LAPIC_TICR, cpu_bus_freq/100); /* 100Hz */
  }

  return mp_num_cpus;
}

void smp_enable(void) {
  ACPI_STATUS Status;
  int i;

  outb( 0xFF, 0x21 );           /* Mask interrupts in Master/Slave 8259A PIC */
  outb( 0xFF, 0xA1 );

  /* starting at 0x10, 0x12, 0x14, ... write the redirection table entries */
  for(i = 0; i < 24; i++) {
    /* using logical destination mode, mask: 0xFF (all CPUs) */
    IOAPIC_write64(IOAPIC_REDIR + (i * 2), (0xFF00000000000800LL | (0x20 + i)));

    /* using logical destination mode, mask: 0x01 (CPU0 only) */
    //IOAPIC_write64(IOAPIC_REDIR + (i * 2), (0x0100000000000800LL | (0x20 + i)));
  }

  /* need to handle multiple IO-APICs */

  if (mp_timer_IOAPIC_irq != 0) {
    /* some systems send the timer to another IRQ in APIC mode */
    /* map the timer IRQ to vector 0x20 */
    IOAPIC_write64(IOAPIC_REDIR + (mp_timer_IOAPIC_irq * 2), 0xFF00000000000820LL);
    /* disable IRQ0 */
    IOAPIC_write64(IOAPIC_REDIR + 0x00, (0x0000000000010000LL));
  } else {
    /* disable IRQ2 */
    IOAPIC_write64(IOAPIC_REDIR + 0x04, (0x0000000000010000LL));
  }

  outb(0x70, 0x22);             /* Re-direct IMCR to use IO-APIC */
  outb(0x01, 0x23);             /* (for some motherboards) */

  Status = AcpiInitializeSubsystem();
  if(ACPI_FAILURE(Status)) {
    com1_printf("Failed to initialize ACPI.\n");
  }
  
  /* now mp_enabled = 1 is triggered in timer IRQ handler */
  //if (mp_num_cpus > 1) mp_enabled = 1;
}

static struct mp_fp *probe_mp_fp(DWORD start, DWORD end) {
  DWORD i;
  start &= ~0xF;                /* 16-byte aligned */
  for (i=start;i<end;i+=0x10) {
    if (*((volatile DWORD *) i) == MP_FP_SIGNATURE) {
      /* found it */
      return (struct mp_fp *)i;
    }
  }
  return NULL;
}


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

#define putx com1_putx
#define print com1_puts
#define putchar com1_putc
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
      ptr += sizeof(struct mp_config_processor_entry);
      if(add_processor(&entry->processor)) 
        print(" (booted)");
      print ("\n");
      break;
    case MP_CFG_TYPE_BUS:
      print ("Bus entry id: ");
      putx (entry->bus.id);
      print (" type: ");
      for (j=0;j<6;j++) putchar(entry->bus.bus_type[j]);
      print ("\n");
      if (entry->bus.bus_type[0] == 'I' &&
          entry->bus.bus_type[1] == 'S' &&
          entry->bus.bus_type[2] == 'A') {
        mp_ISA_bus_id = entry->bus.id;
      }
      ptr += sizeof(struct mp_config_bus_entry);
      break;
    case MP_CFG_TYPE_IO_APIC:
      print ("IO APIC id: ");
      putx (entry->IO_APIC.id);
      print (" version: ");
      putx (entry->IO_APIC.version);
      if (entry->IO_APIC.flags & 1) 
        mp_IOAPIC_addr = entry->IO_APIC.address;
      else
        print (" (disabled)");
      print (" address: ");
      putx (entry->IO_APIC.address);
      print ("\n");
      ptr += sizeof(struct mp_config_IO_APIC_entry);
      break;
    case MP_CFG_TYPE_IO_INT:
      print ("IO interrupt type: ");
      putx (entry->IO_int.int_type); putchar (' ');
      putx (entry->IO_int.flags); putchar (' ');
      putx (entry->IO_int.source_bus_id); putchar (' ');
      putx (entry->IO_int.source_bus_irq); putchar (' ');
      putx (entry->IO_int.dest_APIC_id); putchar (' ');
      putx (entry->IO_int.dest_APIC_intin); putchar (' ');
      print ("\n");

      if (entry->IO_int.source_bus_id == mp_ISA_bus_id && /* ISA */
          entry->IO_int.source_bus_irq == 0)  { /* timer */
        mp_timer_IOAPIC_id  = entry->IO_int.dest_APIC_id;
        mp_timer_IOAPIC_irq = entry->IO_int.dest_APIC_intin;
      }
          
      ptr += sizeof(struct mp_config_interrupt_entry);
      break;
    case MP_CFG_TYPE_LOCAL_INT:
      print ("Local interrupt type: ");
      putx (entry->local_int.int_type); putchar (' ');
      putx (entry->IO_int.flags); putchar (' ');
      putx (entry->IO_int.source_bus_id); putchar (' ');
      putx (entry->IO_int.source_bus_irq); putchar (' ');
      putx (entry->IO_int.dest_APIC_id); putchar (' ');
      putx (entry->IO_int.dest_APIC_intin); putchar (' ');
      print ("\n");
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
#undef print
#undef putchar
#undef putx

static BYTE checksum(BYTE *ptr, int length) {
  BYTE sum = 0;
  while (length-- > 0) sum += *ptr++;
  return sum;
}

int send_ipi(DWORD dest, DWORD v) {
  int to, send_status;

  asm volatile ("pushfl");
  asm volatile ("cli");

  MP_LAPIC_WRITE(LAPIC_ICR+0x10, dest << 24);
  MP_LAPIC_WRITE(LAPIC_ICR, v);

  to = 0;
  do {
    send_status = MP_LAPIC_READ(LAPIC_ICR) & LAPIC_ICR_STATUS_PEND;
  } while (send_status && (to++ < 1000));

  asm volatile ("popfl");

  return (to < 1000);
}


extern BYTE patch_code_start[];
extern BYTE patch_code_end[];
extern BYTE status_code[];
extern BYTE ap_stack_ptr[];

/* For some reason, if this function is 'static', and -O is on, then
 * qemu fails. */
int boot_cpu(struct mp_config_processor_entry *proc) {
  BYTE apic_id = proc->APIC_id;
  int success = 1;
  volatile int to;
  DWORD bootaddr, accept_status;
  /* DWORD bios_reset_vector = BIOS_RESET_VECTOR; */ /* identity mapped */

  /* Get a page for the AP's C stack */
  unsigned long page_frame = (unsigned long) AllocatePhysicalPage();
  DWORD *virt_addr = MapVirtualPage( page_frame | 3 );

  /* Set up the boot code for the APs */
#define TEST_BOOTED(x) (*((volatile DWORD *)(x+status_code-patch_code_start)))
#define STACK_PTR(x)   (*((volatile DWORD *)(x+ap_stack_ptr-patch_code_start)))

  bootaddr = MP_BOOTADDR;           /* identity mapped */
  memcpy((BYTE *)bootaddr, patch_code_start, patch_code_end - patch_code_start);
  TEST_BOOTED(bootaddr) = 0;
  STACK_PTR(bootaddr) = (DWORD)virt_addr;

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
       * register with MP_BOOTADDR>>4 which is of course invalid in p-mode.  So
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
#if 0
    print("Processor ");
    putx(apic_id);
    print(" not responding.\n");
#endif
    success = 0;
  } else {
#if 0
    print("Processor ");
    putx(apic_id);
    print(" BOOTED!\n");
#endif
  }

  /* cleanup */
  MP_LAPIC_WRITE(LAPIC_ESR, 0);
  accept_status = MP_LAPIC_READ(LAPIC_ESR);

  /* This seems to make more problems: */
  /************************************************
   * CMOS_WRITE_BYTE(CMOS_RESET_CODE, 0);         *
   * *((volatile DWORD *) bios_reset_vector) = 0; *
   ************************************************/
  
  return success;
}

static int add_processor(struct mp_config_processor_entry *proc) {
  BYTE apic_id = proc->APIC_id;

  if(!(proc->flags & 1)) return 0; /* disabled processor */
  if(proc->flags & 2) return 0;    /* bootstrap processor */
  
  if(boot_cpu(proc)) {
    CPU_to_APIC[mp_num_cpus] = apic_id;
    APIC_to_CPU[apic_id] = mp_num_cpus;
    mp_num_cpus++;
    return 1;
  } else return 0;
}

extern volatile unsigned long tick;

static void smp_setup_LAPIC_timer_int(void) {
  tick++;
  
  outb( 0x60, 0x20 );           /* still using PIC */
  
  asm volatile("leave");
  asm volatile("iret");
}

static void smp_LAPIC_timer_irq_handler(void) {
  /* just EOI and ignore it */
  MP_LAPIC_WRITE(LAPIC_EOI, 0); /* send to LAPIC -- this int came from LAPIC */
  asm volatile("leave");
  asm volatile("iret");
}

unsigned long cpu_bus_freq;

/* Use the PIT to find how fast the LAPIC ticks (and correspondingly,
 * the bus speed of the processor) */
static void smp_setup_LAPIC_timer(void) {
  idt_descriptor old_timer, old_3f;
  unsigned long value, start, finish, count;
  
  disable_idt();
  get_idt_descriptor(0x20, &old_timer);
  get_idt_descriptor(0x3f, &old_3f);
  set_idt_descriptor_by_addr(0x20, (void *)&smp_setup_LAPIC_timer_int, 0x3);
  set_idt_descriptor_by_addr(0x3f, (void *)&smp_LAPIC_timer_irq_handler, 0x3);
  
  MP_LAPIC_WRITE(LAPIC_LVTT, 0x3f); /* enable LAPIC timer int: vector=0x3f */
  MP_LAPIC_WRITE(LAPIC_TDCR, 0x0B); /* set LAPIC timer divisor to 1 */
  
  value = tick;
  asm volatile("sti");
  while(tick == value) asm volatile("pause");
  start = tick;
  MP_LAPIC_WRITE(LAPIC_TICR, 0xFFFFFFFF); /* write large value to Initial Count Reg. */
  /* LAPIC begins counting down */
  while(tick == start) asm volatile("pause");
  finish = tick;
  asm volatile("cli");
  
  MP_LAPIC_WRITE(LAPIC_LVTT, 0x10000); /* disable timer int */
  count = MP_LAPIC_READ(LAPIC_TCCR);   /* read the remaining count */
  set_idt_descriptor(0x20, &old_timer);
  set_idt_descriptor(0x3f, &old_3f);
  
  cpu_bus_freq = (0xFFFFFFFF - count) * HZ;
  print("CPU bus freq = ");
  putx(cpu_bus_freq);
  putchar('\n');
  
  enable_idt();
}

QWORD IOAPIC_read64(BYTE reg) {
  DWORD high, low;
  QWORD retval;
  MP_IOAPIC_WRITE(IOAPIC_REGSEL, reg+1);
  high = MP_IOAPIC_READ(IOAPIC_RW);
  MP_IOAPIC_WRITE(IOAPIC_REGSEL, reg);
  low = MP_IOAPIC_READ(IOAPIC_RW);
  retval = (QWORD)high << 32;
  retval |= (QWORD)low;
  return retval;
}

void IOAPIC_write64(BYTE reg, QWORD v) {
  /* First, disable the entry by setting the mask bit */
  MP_IOAPIC_WRITE(IOAPIC_REGSEL, reg);
  MP_IOAPIC_WRITE(IOAPIC_RW, 0x10000);
  /* Write to the upper half */
  MP_IOAPIC_WRITE(IOAPIC_REGSEL, reg+1);
  MP_IOAPIC_WRITE(IOAPIC_RW, (DWORD)(v >> 32));
  /* Write to the lower half */
  MP_IOAPIC_WRITE(IOAPIC_REGSEL, reg);
  MP_IOAPIC_WRITE(IOAPIC_RW, (DWORD)(v & 0xFFFFFFFF));
}


BYTE LAPIC_get_physical_ID(void) {
  return (MP_LAPIC_READ(LAPIC_ID) >> 0x18) & 0xF;
}

void send_eoi (void) {
  if (mp_num_cpus > 1) {
    MP_LAPIC_WRITE(LAPIC_EOI, 0); /* send to LAPIC */
  } else {
    outb( 0x60, 0x20 );         /* send to 8259A PIC */
  }
}

void LAPIC_start_timer(unsigned long count) {
  MP_LAPIC_WRITE(LAPIC_TICR, count); 
}

/* ************************************************** */

void ap_init(void) {
  int phys_id, log_dest;
  /* Setup the LAPIC */

  MP_LAPIC_WRITE(LAPIC_TPR, 0x20); 
  MP_LAPIC_WRITE(LAPIC_LVTT, 0x10000); /* disable timer int */
  MP_LAPIC_WRITE(LAPIC_LVTPC, 0x10000); /* disable perf ctr int */
  MP_LAPIC_WRITE(LAPIC_LVT0, 0x08700);  /* enable normal external ints */
  MP_LAPIC_WRITE(LAPIC_LVT1, 0x00400);  /* enable NMI */
  MP_LAPIC_WRITE(LAPIC_LVTE, 0x10000);  /* disable error ints */
  MP_LAPIC_WRITE(LAPIC_SPIV, 0x0010F);  /* enable APIC: spurious vector = 0xF */

  /* be sure: */
  MP_LAPIC_WRITE(LAPIC_LVT1, 0x00400);  /* enable NMI */
  MP_LAPIC_WRITE(LAPIC_LVTE, 0x10000);  /* disable error ints */

  phys_id = (MP_LAPIC_READ(LAPIC_ID) >> 0x18) & 0xF;

  /* setup a logical destination address */
  log_dest = 0x01000000 << phys_id;
  MP_LAPIC_WRITE(LAPIC_LDR, log_dest); /* write to logical destination reg */
  MP_LAPIC_WRITE(LAPIC_DFR, -1);       /* use 'flat model' destination format */

  /* Wait for all processors to come online, and the system to enter
   * MP mode. */
  while (!mp_enabled) 
    asm volatile("pause");

  MP_LAPIC_WRITE(LAPIC_LVTT, 0x3e); /* enable LAPIC timer int: vector=0x3e 
                                     * one-shot mode. */
  MP_LAPIC_WRITE(LAPIC_TDCR, 0x0B); /* set LAPIC timer divisor to 1 */
  MP_LAPIC_WRITE(LAPIC_TICR, cpu_bus_freq/100); /* 100Hz */

  lock_kernel();

  ltr( dummyTSS_selector );

  /**************************************************
   * print("HELLO WORLD from Physical LAPIC ID: "); *
   * putx(phys_id);                                 *
   * print(" logical mask: ");                      *
   * putx(log_dest);                                *
   * print("\n");                                   *
   **************************************************/

  asm volatile("lidt idt_ptr");

  jmp_gate(idleTSS_selector[phys_id]); /* begin in IDLE task */
  
  panic("AP: unreachable");
}
