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
#include "acpi/include/acmacros.h"
#include "acpi/include/acexcep.h"
#include "printf.h"

#define DEBUG_SMP 1

#define APIC_BROADCAST_ID 0xFF
//#define MAX_CPUS          APIC_BROADCAST_ID

#define MP_READ(x)    (*((volatile DWORD *) (x)))
#define MP_WRITE(x,y) (*((volatile DWORD *) (x)) = (y))

#define LAPIC_ADDR_DEFAULT  0xFEE00000uL
#define IOAPIC_ADDR_DEFAULT 0xFEC00000uL

volatile int mp_enabled=0, mp_num_cpus=1, mp_apic_mode=0;

DWORD mp_LAPIC_addr = LAPIC_ADDR_DEFAULT;
#define MP_LAPIC_READ(x)   (*((volatile DWORD *) (mp_LAPIC_addr+(x))))
#define MP_LAPIC_WRITE(x,y) (*((volatile DWORD *) (mp_LAPIC_addr+(x))) = (y))

#define MAX_IOAPICS MAX_CPUS
DWORD mp_IOAPIC_addr = IOAPIC_ADDR_DEFAULT;
static mp_IOAPIC_info mp_IOAPICs[MAX_IOAPICS];
static int mp_num_IOAPICs = 0;
#define MP_IOAPIC_READ(x)   (*((volatile DWORD *) (mp_IOAPIC_addr+(x))))
#define MP_IOAPIC_WRITE(x,y) (*((volatile DWORD *) (mp_IOAPIC_addr+(x))) = (y))

#define MAX_INT_OVERRIDES 4
static mp_int_override mp_overrides[MAX_INT_OVERRIDES];
static int mp_num_overrides = 0;

/* Mapping from CPU # to APIC ID */
BYTE CPU_to_APIC[MAX_CPUS];
BYTE APIC_to_CPU[MAX_CPUS];

static int mp_ACPI_enabled = 0;

/* I hard-code a check for re-routing of the timer IRQ, but this
 * should probably be folded into a more general interrupt routing
 * system. */
static int mp_timer_IOAPIC_irq = 0;
static int mp_timer_IOAPIC_id = 0;
int mp_ISA_bus_id = 0;

static int process_acpi_tables(void);
static int acpi_add_processor(ACPI_MADT_LOCAL_APIC *);
static int process_mp_fp(struct mp_fp *);
static int process_mp_config(struct mp_config *);
static int add_processor(struct mp_config_processor_entry *);
static struct mp_fp *probe_mp_fp(DWORD, DWORD);
static void smp_setup_LAPIC_timer(void);
int boot_cpu(BYTE, BYTE);

/* ACPICA early initialization requires some static space be set aside
 * for ACPI tables -- and there is no dynamic memory allocation
 * available at this stage, so here it is: */
#define ACPI_MAX_INIT_TABLES 16
static ACPI_TABLE_DESC TableArray[ACPI_MAX_INIT_TABLES];

/* ************************************************** */
/* General initialization for SMP */

/* Returns number of CPUs successfully booted. */
int smp_init(void) {
  struct mp_fp *ptr;
  int phys_id, log_dest;

  mp_apic_mode = 1;

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

  if(process_acpi_tables() <= 0) {
    /* ACPI failed to initialize, try Intel MPS */
  
    if       ((ptr = probe_mp_fp(0x9F800, 0xA0000)));
    else if  ((ptr = probe_mp_fp(0x0040E, 0x0140E)));
    else if  ((ptr = probe_mp_fp(0xE0000, 0xFFFFF)));
    else return 1;                /* assume uniprocessor */

    mp_num_cpus = process_mp_fp (ptr);
  }

  return mp_num_cpus;
}

/* ************************************************** */

extern char const   *AcpiGbl_ExceptionNames_Env[];

ACPI_STATUS 
DisplayOneDevice(ACPI_HANDLE ObjHandle, UINT32 Level, void *Context, void **RetVal) {
  ACPI_STATUS Status;
  ACPI_DEVICE_INFO *Info;
  ACPI_BUFFER Path;
  ACPI_BUFFER Result;
  ACPI_OBJECT Obj;
  char Buffer[256];

  Path.Length = sizeof(Buffer);
  Path.Pointer = Buffer;

  Status = AcpiGetName(ObjHandle, ACPI_FULL_PATHNAME, &Path);
  if (ACPI_SUCCESS(Status)) {
    com1_printf("%s: \n", Path.Pointer);
  }
  Status = AcpiGetObjectInfo(ObjHandle, &Info);
  if (ACPI_SUCCESS(Status)) {
    com1_printf ("    ");
    if(Info->Valid & ACPI_VALID_STA)
      com1_printf (" STA %.8X", Info->CurrentStatus);
    if(Info->Valid & ACPI_VALID_ADR)
      com1_printf (" ADR %.8X", Info->Address);
    if(Info->Valid & ACPI_VALID_HID)
      com1_printf (" HID %s", Info->HardwareId.String);
    if(Info->Valid & ACPI_VALID_UID)
      com1_printf (" UID %s", Info->UniqueId.String);
    if(Info->Valid & ACPI_VALID_CID) 
      com1_printf (" CID");

    ACPI_FREE(Info);
  }

  Result.Length = sizeof(Obj);
  Result.Pointer = &Obj;
  Status = AcpiEvaluateObjectTyped(ObjHandle, "_DDN", NULL, &Result
                                   , ACPI_TYPE_STRING);
  if(ACPI_SUCCESS(Status)) {
    com1_printf(" DDN=%s", Obj.String.Pointer);
  }

  Result.Length = sizeof(Obj);
  Result.Pointer = &Obj;
  Status = AcpiEvaluateObjectTyped(ObjHandle, "_STR", NULL, &Result
                                   , ACPI_TYPE_STRING);
  if(ACPI_SUCCESS(Status)) {
    com1_printf(" STR=%s", Obj.String.Pointer);
  }

  Result.Length = sizeof(Obj);
  Result.Pointer = &Obj;
  Status = AcpiEvaluateObjectTyped(ObjHandle, "_MLS", NULL, &Result
                                   , ACPI_TYPE_STRING);
  if(ACPI_SUCCESS(Status)) {
    com1_printf(" MLS=%s", Obj.String.Pointer);
  }

  com1_printf("\n");


  return AE_OK;
}

static void smp_IOAPIC_setup(void) {
  int i, j;

  outb( 0xFF, 0x21 );  /* Mask interrupts in Master/Slave 8259A PIC */
  outb( 0xFF, 0xA1 );

  /* To get to a consistent state, first disable all IO-APIC
   * redirection entries. */

  for(i=0;i<mp_num_IOAPICs;i++) {
    for(j=0;j<mp_IOAPICs[i].numGSIs;j++) {
      IOAPIC_write64(IOAPIC_REDIR + (i * 2), 0x0000000000010000LL);
    }
  }

  /* Map timer IRQ to vector 0x20 */
  IOAPIC_map_GSI(IRQ_to_GSI(mp_ISA_bus_id, 0), 0x20, 0xFF00000000000800LL);

  outb(0x70, 0x22);             /* Re-direct IMCR to use IO-APIC */
  outb(0x01, 0x23);             /* (for some motherboards) */

}

void smp_enable(void) {
  ACPI_STATUS Status;

  smp_IOAPIC_setup();

  /* Complete the ACPICA initialization sequence */
  if(mp_ACPI_enabled) {
    Status = AcpiInitializeSubsystem();
    if(ACPI_FAILURE(Status)) {
      com1_printf("Failed to initialize ACPI.\n");
    }
    Status = AcpiReallocateRootTable();
    if(ACPI_FAILURE(Status)) {
      com1_printf("Failed: AcpiReallocateRootTable.\n");
    }
    Status = AcpiLoadTables();
    if(ACPI_FAILURE(Status)) {
      com1_printf("Failed: AcpiLoadTables.\n");
    }
    Status = AcpiEnableSubsystem(ACPI_FULL_INITIALIZATION);
    if(ACPI_FAILURE(Status)) {
      com1_printf("Failed: AcpiEnableSubsystem.\n");
    }
    Status = AcpiInitializeObjects(ACPI_FULL_INITIALIZATION);
    if(ACPI_FAILURE(Status)) {
      com1_printf("Failed: AcpiInitializeObjects.\n");
    }
  
    /* Walk the System Bus "\_SB_" and output info about each object
     * found. */
    {
      ACPI_HANDLE SysBusHandle;
      AcpiGetHandle(ACPI_ROOT_OBJECT, ACPI_NS_SYSTEM_BUS, &SysBusHandle);
      AcpiWalkNamespace(ACPI_TYPE_DEVICE, SysBusHandle, INT_MAX, 
                        DisplayOneDevice, NULL, NULL);
    }
  }

  /* The global variable mp_enabled will be incremented in the PIT IRQ
   * handler, this permits the Application Processors to go ahead and
   * complete initialization after the kernel has entered a
   * multi-processing safe state. */
}

void smp_enable_scheduling(void) {
  MP_LAPIC_WRITE(LAPIC_LVTT, 0x3e); /* enable LAPIC timer int: vector=0x3e 
                                     * one-shot mode. */
  MP_LAPIC_WRITE(LAPIC_TDCR, 0x0B); /* set LAPIC timer divisor to 1 */
  MP_LAPIC_WRITE(LAPIC_TICR, cpu_bus_freq/100); /* 100Hz */

  sched_enabled = 1;
}

/*******************************************************************
 * Support for ACPI via ACPICA, the Intel Reference Implementation *
 *******************************************************************/

/* ACPICA supports the notion of "Early Initialization" the very
 * purpose of which is to obtain the ACPI tables and begin the process
 * of booting Application Processors when the rest of the operating
 * system is not ready for normal operation.  This is intended to
 * supersede the usage of the Intel Multiprocessing Specification. */

#define printf com1_printf
static int process_acpi_tables(void) {
  ACPI_STATUS status;
  status = AcpiInitializeTables(TableArray, ACPI_MAX_INIT_TABLES, TRUE);
  if (status == AE_OK) {
    ACPI_TABLE_MADT *madt;
    ACPI_TABLE_FADT *fadt;
    mp_ACPI_enabled = 1;
    mp_ISA_bus_id = 0;
    if(AcpiGetTable(ACPI_SIG_MADT, 0, (ACPI_TABLE_HEADER **)&madt) == AE_OK) {
      /* Multiple APIC Description Table */
      BYTE *ptr, *lim = (BYTE *)madt + madt->Header.Length;
      printf("ACPI OEM: %.6s Compiler: %.4s LAPIC: %p Flags:%s\n", 
             madt->Header.OemId,
             madt->Header.AslCompilerId,
             madt->Address,
             (madt->Flags & ACPI_MADT_PCAT_COMPAT) ? " PCAT_COMPAT" : "");
      ptr = (BYTE *)madt + sizeof(ACPI_TABLE_MADT);
      while (ptr < lim) {
        switch(((ACPI_SUBTABLE_HEADER *)ptr)->Type) {
        case ACPI_MADT_TYPE_LOCAL_APIC: { /* Processor entry */
          ACPI_MADT_LOCAL_APIC *sub = (ACPI_MADT_LOCAL_APIC *)ptr;
          printf("Processor: %X APIC-ID: %X %s",
                 sub->ProcessorId,
                 sub->Id,
                 sub->LapicFlags & 1 ? "(enabled)" : "(disabled)");
          if(acpi_add_processor(sub)) {
            printf(" (booted)");
          }
          printf("\n");
          break;
        }
        case ACPI_MADT_TYPE_IO_APIC: { /* IO-APIC entry */
          ACPI_MADT_IO_APIC *sub = (ACPI_MADT_IO_APIC *)ptr;
          printf("IO-APIC ID: %X Address: %X IRQBase: %X\n",
                 sub->Id,
                 sub->Address,
                 sub->GlobalIrqBase);
          if (mp_num_IOAPICs == MAX_IOAPICS) panic("Too many IO-APICs.");
          mp_IOAPIC_addr = sub->Address;
          mp_timer_IOAPIC_id = sub->Id;
          mp_IOAPICs[mp_num_IOAPICs].id       = sub->Id;
          mp_IOAPICs[mp_num_IOAPICs].address  = sub->Address;
          mp_IOAPICs[mp_num_IOAPICs].startGSI = sub->GlobalIrqBase;
          MP_IOAPIC_WRITE(IOAPIC_REGSEL, 0x1);
          mp_IOAPICs[mp_num_IOAPICs].numGSIs  = 
            APIC_MAXREDIR(MP_IOAPIC_READ(IOAPIC_RW)) + 1;
          mp_num_IOAPICs++;
          break;
        }
        case ACPI_MADT_TYPE_INTERRUPT_OVERRIDE: { /* Interrupt Override entry */
          ACPI_MADT_INTERRUPT_OVERRIDE *sub = (ACPI_MADT_INTERRUPT_OVERRIDE *)ptr;
          printf("Int. Override: Bus: %X SourceIRQ: %X GlobalIRQ: %X\n",
                 sub->Bus,
                 sub->SourceIrq,
                 sub->GlobalIrq);
          if (sub->Bus == 0 && /* spec sez: "'0' meaning ISA" */
              sub->SourceIrq == 0)  { /* timer */
            mp_timer_IOAPIC_irq = sub->GlobalIrq;
          }
          if (mp_num_overrides == MAX_INT_OVERRIDES) 
            panic("Too many interrupt overrides.");
          mp_overrides[mp_num_overrides].src_bus = sub->Bus;
          mp_overrides[mp_num_overrides].src_IRQ = sub->SourceIrq;
          mp_overrides[mp_num_overrides].dest_GSI = sub->GlobalIrq;
          mp_num_overrides++;
          break;
        }
        default:
          printf("MADT sub-entry: %X\n", ((ACPI_SUBTABLE_HEADER *)ptr)->Type);
          break;
        }
        ptr += ((ACPI_SUBTABLE_HEADER *)ptr)->Length;
      } 
    } else {
      printf("AcpiGetTable MADT: FAILED\n");
      return 0;
    }
    if(AcpiGetTable(ACPI_SIG_FADT, 0, (ACPI_TABLE_HEADER **)&fadt) == AE_OK) {
      /* Fixed ACPI Description Table */
      printf("Bootflags: %s %s %s\n",
             (fadt->BootFlags & ACPI_FADT_LEGACY_DEVICES) ? 
             "HAS_LEGACY_DEVICES" : "NO_LEGACY_DEVICES",
             (fadt->BootFlags & ACPI_FADT_8042) ? 
             "HAS_KBD_8042" : "NO_KBD_8042",
             (fadt->BootFlags & ACPI_FADT_NO_VGA) ? 
             "NO_VGA_PROBING" : "VGA_PROBING_OK");
    } else {
      printf("AcpiGetTable FADT: FAILED\n");
      return 0;
    }
  } else return 0;

  return mp_num_cpus;
}
#undef printf

/* A small wrapper around boot_cpu() which does some checks and
 * maintains two small tables. */
static int acpi_add_processor(ACPI_MADT_LOCAL_APIC *ptr) {
  BYTE this_apic_id = LAPIC_get_physical_ID();
  BYTE apic_id = ptr->Id;

  if(!(ptr->LapicFlags & 1)) return 0; /* disabled processor */
  if(this_apic_id == apic_id) return 0; /* bootstrap processor */
  
  if(boot_cpu(apic_id, APIC_VER_NEW)) {
    CPU_to_APIC[mp_num_cpus] = apic_id;
    APIC_to_CPU[apic_id] = mp_num_cpus;
    mp_num_cpus++;
    return 1;
  } else return 0;
}


/* End ACPI support */

/*******************************************************
 * Support for the Intel Multiprocessing Specification *
 *******************************************************/

/* The Intel MPS is a legacy standard which specifies a set of tables
 * populated by the BIOS that describe the system well enough to find
 * and start the Application Processors and IO-APICs. */

/* It is officially superseded by ACPI, but every system still
 * supports it.  It has the benefit of being simpler.  However, it is
 * not as informative, particularly when it comes to distinguishing
 * non-uniform architectures and also hyperthreading vs multicore
 * systems. */

/* Quest will attempt to make use of ACPI first, then fall-back to the
 * Intel MPS.  I do not plan on supporting systems pre-MPS, which
 * means Quest is effectively limited to booting on Pentium-Pro+ most
 * likely. */

/* The Intel MPS tables are allowed to be placed in a variety of
 * memory regions available to the BIOS.  Most commonly found in the
 * ShadowROM area.  This function will search for the 4-byte signature
 * in a given memory region and return a pointer to the so-called
 * "Floating Pointer" table. */
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

/* Once found, the pointer to the table is examined for sanity by this
 * function.  It makes further calls to interpret the information
 * found in the tables and begin SMP initialization. */
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

/* The MP Configuration table is pointed to by the MP Floating Pointer
 * table and contains the information in question that tells us how
 * many processors, IO-APICs, and various bits of useful details about
 * them. */
#define printf com1_printf
static int process_mp_config(struct mp_config *cfg) {
  int i;
  BYTE *ptr;

  /* Sanity checks */
  if (cfg == NULL) {
    printf("MP config pointer is NULL.\n");
    return 1;
  }

  if (cfg->signature != MP_CFG_SIGNATURE) {
    printf("MP config signature invalid.\n");
    return 1;
  }

  if (cfg->specification_revision != 1 && 
      cfg->specification_revision != 4) {
    printf("Unknown MP specification reported by MP config table.\n");
    return 1;
  }

  if(checksum((BYTE *)cfg, cfg->base_table_length) != 0) {
    printf("MP config table failed checksum.\n");
    return 1;
  }

  printf("Manufacturer: %.8s Product: %.12s Local APIC: %.8X\n", 
         cfg->OEM_id, cfg->product_id, cfg->local_APIC);
  if(cfg->local_APIC) mp_LAPIC_addr = cfg->local_APIC;

  /* Check entries */
  ptr = (BYTE *)cfg->entries;
  for(i=0;i<cfg->entry_count;i++) {
    struct mp_config_entry *entry = (struct mp_config_entry*) ptr;
    switch (*ptr) {
    case MP_CFG_TYPE_PROCESSOR: /* Processor entry */
      printf("Processor APIC-id: %X version: %X %s%s", 
              entry->processor.APIC_id,
              entry->processor.APIC_version,
              (entry->processor.flags & 1) ? "(enabled)" : "(disabled)",
              (entry->processor.flags & 2) ? " (bootstrap)" : "");
      ptr += sizeof(struct mp_config_processor_entry);

      if(add_processor(&entry->processor)) /* Try to boot it if necessary */
        printf(" (booted)");
      printf("\n");

      break;

    case MP_CFG_TYPE_BUS:       /* Bus entry, find out which one is ISA */
      printf ("Bus entry-id: %X type: %.6s\n", 
              entry->bus.id,
              entry->bus.bus_type);
      if (entry->bus.bus_type[0] == 'I' &&
          entry->bus.bus_type[1] == 'S' &&
          entry->bus.bus_type[2] == 'A') {
        mp_ISA_bus_id = entry->bus.id;
      }
      ptr += sizeof(struct mp_config_bus_entry);
      break;

    case MP_CFG_TYPE_IO_APIC:   /* IO-APIC entry */
      printf ("IO APIC-id: %X version: %X address: %.8X", 
              entry->IO_APIC.id,
              entry->IO_APIC.version,
              entry->IO_APIC.address);
      if (entry->IO_APIC.flags & 1) 
        mp_IOAPIC_addr = entry->IO_APIC.address;
      else
        printf(" (disabled)");
      printf("\n");

      if (mp_num_IOAPICs == MAX_IOAPICS) panic("Too many IO-APICs.");
      mp_IOAPICs[mp_num_IOAPICs].id       = entry->IO_APIC.id;
      mp_IOAPICs[mp_num_IOAPICs].address  = entry->IO_APIC.address;
      /* going to assume IO-APICs are listed in order */
      if (mp_num_IOAPICs == 0)
        mp_IOAPICs[mp_num_IOAPICs].startGSI = 0;
      else
        mp_IOAPICs[mp_num_IOAPICs].startGSI = 
          mp_IOAPICs[mp_num_IOAPICs-1].startGSI +
          mp_IOAPICs[mp_num_IOAPICs-1].numGSIs;
      
      MP_IOAPIC_WRITE(IOAPIC_REGSEL, 0x1);
      mp_IOAPICs[mp_num_IOAPICs].numGSIs  = 
        APIC_MAXREDIR(MP_IOAPIC_READ(IOAPIC_RW)) + 1;
      mp_num_IOAPICs++;

      ptr += sizeof(struct mp_config_IO_APIC_entry);
      break;

    case MP_CFG_TYPE_IO_INT:    /* IO-Interrupt entry */
      printf("IO interrupt type: %X flags: %X source: (bus: %X irq: %X) dest: (APIC: %X int: %X)\n",
             entry->IO_int.int_type,
             entry->IO_int.flags,
             entry->IO_int.source_bus_id,
             entry->IO_int.source_bus_irq,
             entry->IO_int.dest_APIC_id,
             entry->IO_int.dest_APIC_intin);
      
      /* Check if timer interrupt was re-routed.  It is common for
       * IRQ0 on the PIC to be re-routed to Global System Interrupt 2
       * in APIC-mode.  QEmu does it, for example. */
      if (entry->IO_int.source_bus_id == mp_ISA_bus_id && /* ISA */
          entry->IO_int.source_bus_irq == 0)  { /* timer */
        mp_timer_IOAPIC_id  = entry->IO_int.dest_APIC_id;
        mp_timer_IOAPIC_irq = entry->IO_int.dest_APIC_intin;
      }

      if (entry->IO_int.source_bus_irq != entry->IO_int.dest_APIC_intin) {
        /* not sure if this is the right condition */
        if (mp_num_overrides == MAX_INT_OVERRIDES) 
          panic("Too many interrupt overrides.");
        mp_overrides[mp_num_overrides].src_bus = entry->IO_int.source_bus_id;
        mp_overrides[mp_num_overrides].src_IRQ = entry->IO_int.source_bus_irq;
        mp_overrides[mp_num_overrides].dest_GSI = 
          IOAPIC_lookup(entry->IO_int.dest_APIC_id)->startGSI +
          entry->IO_int.dest_APIC_intin;
        mp_num_overrides++;
      }
      ptr += sizeof(struct mp_config_interrupt_entry);
      break;

    case MP_CFG_TYPE_LOCAL_INT: /* Local-interrupt entry */
      printf("Local interrupt type: %X flags: %X source: (bus: %X irq: %X) dest: (APIC: %X int: %X)\n",
             entry->local_int.int_type,
             entry->local_int.flags,
             entry->local_int.source_bus_id,
             entry->local_int.source_bus_irq,
             entry->local_int.dest_APIC_id,
             entry->local_int.dest_APIC_intin);
      /* It's conceivable that local interrupts could be overriden
       * like IO interrupts, but I have no good examples of it so I
       * will have to defer doing anything about it. */
      ptr += sizeof(struct mp_config_interrupt_entry);
      break;

    default:
      printf("Unknown entry type: %X at address: %p\n", *ptr, ptr);
      return 1;      
    }
  }

  return mp_num_cpus;
}
#undef printf

/* End Intel Multiprocessing Specification implementation */

/* ************************************************** */

/* General SMP initialization functions */

/* Send Interprocessor Interrupt -- prods the Local APIC to deliver an
 * interprocessor interrupt, 'v' specifies the vector but also
 * specifies flags according to the Intel System Programming Manual --
 * also see apic.h and the LAPIC_ICR_* constants. */
int send_ipi(DWORD dest, DWORD v) {
  int timeout, send_status;

  /* It is a bad idea to have interrupts enabled while twiddling the
   * LAPIC. */
  asm volatile ("pushfl");
  asm volatile ("cli");

  MP_LAPIC_WRITE(LAPIC_ICR+0x10, dest << 24);
  MP_LAPIC_WRITE(LAPIC_ICR, v);

  /* Give it a thousand iterations to see if the interrupt was
   * successfully delivered. */
  timeout = 0;
  do {
    send_status = MP_LAPIC_READ(LAPIC_ICR) & LAPIC_ICR_STATUS_PEND;
  } while (send_status && (timeout++ < 1000));

  asm volatile ("popfl");

  return (timeout < 1000);
}


/* A number of symbols defined in the boot-smp.S file: */
extern BYTE patch_code_start[]; /* patch_code is what the AP boots */
extern BYTE patch_code_end[];
extern BYTE status_code[];      /* the AP writes a 1 into here when it's ready */
extern BYTE ap_stack_ptr[];     /* we give the AP a stack pointer through this */

/* For some reason, if this function is 'static', and -O is on, then
 * qemu fails. */
int boot_cpu(BYTE apic_id, BYTE APIC_version) {
  int success = 1;
  volatile int to;
  DWORD bootaddr, accept_status;

  /* Get a page for the AP's C stack */
  unsigned long page_frame = (unsigned long) AllocatePhysicalPage();
  DWORD *virt_addr = MapVirtualPage( page_frame | 3 );

  /* Set up the boot code for the APs */
#define TEST_BOOTED(x) (*((volatile DWORD *)(x+status_code-patch_code_start)))
#define STACK_PTR(x)   (*((volatile DWORD *)(x+ap_stack_ptr-patch_code_start)))

  /* The patch code is memcpyed into the hardcoded address MP_BOOTADDR */
  bootaddr = MP_BOOTADDR;           /* identity mapped */
  memcpy((BYTE *)bootaddr, patch_code_start, patch_code_end - patch_code_start);
  /* The status code is reset to 0 */
  TEST_BOOTED(bootaddr) = 0;
  /* A temporary stack is allocated for the AP to be able to call C code */
  STACK_PTR(bootaddr) = (DWORD)virt_addr;
  /* (FIXME: when does this get de-allocated?) */

  /* CPU startup sequence: officially it is supposed to proceed:
   * ASSERT INIT IPI, DE-ASSERT INIT IPI, STARTUP IPI, STARTUP IPI */

  /* clear APIC error register */
  MP_LAPIC_WRITE(LAPIC_ESR, 0);
  accept_status = MP_LAPIC_READ(LAPIC_ESR);

  /* assert INIT interprocessor interrupt */
  send_ipi(apic_id, LAPIC_ICR_TM_LEVEL | LAPIC_ICR_LEVELASSERT | LAPIC_ICR_DM_INIT);

  /* de-assert INIT IPI */
  send_ipi(apic_id, LAPIC_ICR_TM_LEVEL | LAPIC_ICR_DM_INIT);

  tsc_delay_usec(10000);       /* wait 10 millisec */

  /* Send start-up IPIs if not old version */
  if (APIC_version >= APIC_VER_NEW) {
    int i;
    for (i=1; i <= 2; i++) {
      /* Bochs starts it @ INIT-IPI and the AP goes into p-mode which is
       * bad if I then deliver a STARTUP-IPI because that loads the CS
       * register with MP_BOOTADDR>>4 which is of course invalid in p-mode.  So
       * I added the test. */
      if (TEST_BOOTED(bootaddr)) break;
      send_ipi(apic_id, LAPIC_ICR_DM_SIPI | ((bootaddr >> 12) & 0xFF));
      tsc_delay_usec(200);      /* wait 200 microsec */
    }
  }
  
#define LOOPS_TO_WAIT 10000000
  /* Check for successful start */
  to = 0;
  while (!TEST_BOOTED(bootaddr) && to++ < LOOPS_TO_WAIT) {
    tsc_delay_usec(200);      /* wait 200 microsec */
  }
  if (to >= LOOPS_TO_WAIT) {
    success = 0;
  }

  /* cleanup */
  MP_LAPIC_WRITE(LAPIC_ESR, 0);
  accept_status = MP_LAPIC_READ(LAPIC_ESR);

  return success;
}

/* A small wrapper around boot_cpu() which does some checks and
 * maintains two small tables. */
static int add_processor(struct mp_config_processor_entry *proc) {
  BYTE apic_id = proc->APIC_id;

  if(!(proc->flags & 1)) return 0; /* disabled processor */
  if(proc->flags & 2) return 0;    /* bootstrap processor */
  
  if(boot_cpu(apic_id, proc->APIC_version)) {
    CPU_to_APIC[mp_num_cpus] = apic_id;
    APIC_to_CPU[apic_id] = mp_num_cpus;
    mp_num_cpus++;
    return 1;
  } else return 0;
}


/* ************************************************** */

/* CPU bus frequency measurement code -- the primary usage of this
 * code is for the purpose of programming the Local APIC timer because
 * it operates at the speed of the CPU bus.  So, in order to get an
 * idea of what that means in clock time, we need to compare the
 * frequency of the Local APIC to the frequency of the Programmable
 * Interval Timer. */

/* In essence this boils down to counting how many ticks the Local
 * APIC can speed through for a single interval between PIT
 * interrupts. */

/* NB: The local APIC is perhaps the most finely grained timer
 * commonly available on the PC architecture.  Expect precision on the
 * order of 100 nanoseconds, or better.  In fact, this may cause a
 * problem with integer overflow on the 32-bit architecture. */

extern volatile unsigned long tick; /* defined in interrupt_handler.c */
static void smp_setup_LAPIC_timer_int(void) {
  /* Temporary handler for the PIT-generated timer IRQ */
  tick++;
  
  outb( 0x60, 0x20 );           /* EOI -- still using PIC */

  /* Cheat a little here: the C compiler will insert 
   *   leave
   *   ret
   * but we want to fix the stack up and IRET so I just
   * stick that in right here. */
  asm volatile("leave");
  asm volatile("iret");
}

static void smp_LAPIC_timer_irq_handler(void) {
  /* Temporary handler for the LAPIC-generated timer interrupt */
  /* just EOI and ignore it */
  MP_LAPIC_WRITE(LAPIC_EOI, 0); /* send to LAPIC -- this int came from LAPIC */
  asm volatile("leave");
  asm volatile("iret");
}

/* CPU BUS FREQUENCY -- IN HERTZ */
unsigned long cpu_bus_freq = 0;
QWORD tsc_freq = 0;

/* Use the PIT to find how fast the LAPIC ticks (and correspondingly,
 * the bus speed of the processor) */
static void smp_setup_LAPIC_timer(void) {
  idt_descriptor old_timer, old_3f;
  unsigned long value, start, count;
  DWORD tsc_start_hi, tsc_start_lo, tsc_value_hi, tsc_value_lo;

  /* I want to enable interrupts but I don't want the normal
   * interrupt-handling architecture to kick-in just yet.  Here's an
   * idea: mask off every single entry in the IDT! */
  disable_idt();

  /* Now save the handlers for vectors 0x20 and 0x3F */
  get_idt_descriptor(0x20, &old_timer);
  get_idt_descriptor(0x3f, &old_3f);
  /* And use them for our temporary handlers */
  set_idt_descriptor_by_addr(0x20, (void *)&smp_setup_LAPIC_timer_int, 0x3);
  set_idt_descriptor_by_addr(0x3f, (void *)&smp_LAPIC_timer_irq_handler, 0x3);
  
  MP_LAPIC_WRITE(LAPIC_LVTT, 0x3f); /* enable LAPIC timer int: vector=0x3f */
  MP_LAPIC_WRITE(LAPIC_TDCR, 0x0B); /* set LAPIC timer divisor to 1 */

  /* Timing code: */
  value = tick;
  asm volatile("sti");
  /* Wait until the PIT fires: */
  while(tick == value) asm volatile("pause");
  start = tick;
  MP_LAPIC_WRITE(LAPIC_TICR, 0xFFFFFFFF); /* write large value to Initial Count Reg. */
  asm volatile("rdtsc" : "=a"(tsc_start_lo), "=d"(tsc_start_hi)); /* store timestamp */
  /* LAPIC begins counting down, wait until the PIT fires again: */
  while(tick == start) asm volatile("pause");
  asm volatile("cli");
  asm volatile("rdtsc" : "=a"(tsc_value_lo), "=d"(tsc_value_hi)); /* store timestamp */
  MP_LAPIC_WRITE(LAPIC_LVTT, 0x10000); /* disable timer int */
  count = MP_LAPIC_READ(LAPIC_TCCR);   /* read the remaining count */

  /* Restore the original handlers: */
  set_idt_descriptor(0x20, &old_timer);
  set_idt_descriptor(0x3f, &old_3f);
  
  cpu_bus_freq = (0xFFFFFFFF - count) * HZ;
  printf("CPU bus frequency = %d\n", cpu_bus_freq);
  com1_printf("CPU bus frequency = 0x%X\n", cpu_bus_freq);
  tsc_freq = (((QWORD)tsc_value_hi) << 32) | ((QWORD)tsc_value_lo);
  tsc_freq -= ((QWORD)tsc_start_hi) << 32;
  tsc_freq -= (QWORD)tsc_start_lo;
  tsc_freq *= HZ;
  com1_printf("TSC frequency = 0x%X 0x%X\n", (DWORD)(tsc_freq >> 32), (DWORD)tsc_freq);

  /* Put the IDT back in shape */
  enable_idt();
}

/* End CPU Bus Frequency measurement code */

/* ************************************************** */

/* IOAPIC manipulation: */

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

/* ************************************************** */

/* Some LAPIC utilities */

BYTE LAPIC_get_physical_ID(void) {
  return (MP_LAPIC_READ(LAPIC_ID) >> 0x18) & 0xF;
}

void send_eoi (void) {
  if (mp_apic_mode) {
    MP_LAPIC_WRITE(LAPIC_EOI, 0); /* send to LAPIC */
  } else {
    outb( 0x60, 0x20 );         /* send to 8259A PIC */
  }
}

void LAPIC_start_timer(unsigned long count) {
  MP_LAPIC_WRITE(LAPIC_TICR, count); 
}

/* ************************************************** */

/* Some IO-APIC utilities */
                         
mp_IOAPIC_info *IOAPIC_lookup(BYTE id) {
  int i;
  for(i=0;i<mp_num_IOAPICs;i++) {
    if (mp_IOAPICs[i].id == id) return &mp_IOAPICs[i];
  }
  panic("IOAPIC_lookup failed.");
}

DWORD IRQ_to_GSI(DWORD src_bus, DWORD src_irq) {
  /* this probably only works for ISA at the moment */
  int i;
  for(i=0;i<mp_num_overrides;i++) {
    if(src_bus == mp_overrides[i].src_bus &&
       src_irq == mp_overrides[i].src_IRQ) 
      return mp_overrides[i].dest_GSI;
  }
  if(src_bus != mp_ISA_bus_id)
    panic("IRQ_to_GSI only implemented for ISA bus.");
  /* assume identity-mapped if not overriden */
  return src_irq;
}

int IOAPIC_map_GSI(DWORD GSI, BYTE vector, QWORD flags) {
  int i;
  DWORD old_addr = 0;
  /* First, figure out which IOAPIC */
  for(i=0;i<mp_num_IOAPICs;i++) {
    if(mp_IOAPICs[i].startGSI <= GSI &&
       GSI < mp_IOAPICs[i].startGSI + mp_IOAPICs[i].numGSIs) {
      old_addr = mp_IOAPIC_addr;
      /* set address for macros to use: */
      mp_IOAPIC_addr = mp_IOAPICs[i].address;
      /* set GSI relative to startGSI: */
      GSI -= mp_IOAPICs[i].startGSI;
      break;
    }
  }
  if (!old_addr) return -1;     /* not found */

  IOAPIC_write64(IOAPIC_REDIR + (GSI * 2), flags | vector);
  
  /* clean up */
  mp_IOAPIC_addr = old_addr;
  return GSI;
}


/* ************************************************** */

/* When the AP boots it will first start at the patch code in
 * boot-smp.S.  That code prepares the AP for the jump into C code,
 * namely, this function: */
void ap_init(void) {
  int phys_id, log_dest;

  /* Setup the LAPIC */

  MP_LAPIC_WRITE(LAPIC_TPR, 0x20);      /* task priority = 0x20 */
  MP_LAPIC_WRITE(LAPIC_LVTT, 0x10000);  /* disable timer int */
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

  /* Spin-wait for all processors to come online, and the system to
   * enter MP mode. */
  while (!mp_enabled) 
    asm volatile("pause");

  MP_LAPIC_WRITE(LAPIC_LVTT, 0x3e); /* enable LAPIC timer int: vector=0x3e 
                                     * one-shot mode. */
  MP_LAPIC_WRITE(LAPIC_TDCR, 0x0B); /* set LAPIC timer divisor to 1 */
  MP_LAPIC_WRITE(LAPIC_TICR, cpu_bus_freq/100); /* 100Hz */

  /* The AP is now operating in an SMP environment so the kernel must
   * be locked before any shared resources are utilized.  The dummy
   * TSS is a shared resource. */
  lock_kernel();

  /* Load the dummy TSS so that when the CPU executes jmp_gate it has
   * a place to write the state of the CPU -- even though we don't
   * care about the state and it will be discarded. */
  ltr( dummyTSS_selector );

  asm volatile("lidt idt_ptr"); /* Set the IDT */

  /* The IDLE task runs in kernelspace, therefore it is capable of
   * unlocking the kernel and manually enabling interrupts.  This
   * makes it safe to use lock_kernel() above.  */

  /* If the IDLE task did not run in kernelspace, then we would need a
   * dummy TSS per-processor because it would not be possible to
   * protect the dummy TSS from simultaneous usage by multiple CPUs in
   * this case. */

  jmp_gate(idleTSS_selector[phys_id]); /* begin in IDLE task */
  /* never return */
  
  panic("AP: unreachable");
}
