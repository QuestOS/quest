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

#include "arch/i386.h"
#include "kernel.h"
#include "mem/mem.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "smp/spinlock.h"
#include "drivers/pci/pci.h"
#include "drivers/acpi/acpi.h"
#include "drivers/acpi/acmacros.h"
#include "drivers/acpi/acexcep.h"
#include "util/printf.h"

static int process_acpi_tables (void);
static int acpi_add_processor (ACPI_MADT_LOCAL_APIC *);
static void acpi_parse_srat (ACPI_TABLE_SRAT *);

/*******************************************************************
 * Support for ACPI via ACPICA, the Intel Reference Implementation *
 *******************************************************************/

/* ACPICA supports the notion of "Early Initialization" the very
 * purpose of which is to obtain the ACPI tables and begin the process
 * of booting Application Processors when the rest of the operating
 * system is not ready for normal operation.  This is intended to
 * supersede the usage of the Intel Multiprocessing Specification. */

/* ACPICA early initialization requires some static space be set aside
 * for ACPI tables -- and there is no dynamic memory allocation
 * available at this stage, so here it is: */
#define ACPI_MAX_INIT_TABLES 24
static ACPI_TABLE_DESC TableArray[ACPI_MAX_INIT_TABLES];

uint32
acpi_early_init(void)
{
  return process_acpi_tables ();
}

void
acpi_enable_IOAPIC (void)
{
  ACPI_STATUS Status;
  ACPI_OBJECT arg = { ACPI_TYPE_INTEGER };
  ACPI_OBJECT_LIST arg_list = { 1, &arg };
  arg.Integer.Value = 1;        /* IOAPIC */
  Status = AcpiEvaluateObject (NULL, "\\_PIC", &arg_list, NULL);
  if (ACPI_FAILURE (Status) && Status != AE_NOT_FOUND) {
    com1_printf ("unable to evaluate _PIC(1): %d\n", Status);
  } else
    com1_printf ("ACPI: IOAPIC mode enabled.\n");
}

static u8 acpi_sci_irq = 9, acpi_sci_flags = 0;
static u32
acpi_irq_handler (u8 vec)
{
  extern ACPI_OSD_HANDLER acpi_service_routine;
  extern void *acpi_service_routine_context;

  //logger_printf ("ACPI: IRQ (vec=0x%X) acpi_service_routine=0x%p\n", vec, acpi_service_routine);
  if (acpi_service_routine) {
    acpi_service_routine (acpi_service_routine_context);
  }
  return 0;
}

static UINT32
acpi_power_button (void *ctxt)
{
  ACPI_STATUS AcpiHwWrite (UINT32 Value, ACPI_GENERIC_ADDRESS *Reg);

  logger_printf ("ACPI: acpi_power_button\n");
  printf ("REBOOTING...\n");
  com1_printf ("REBOOTING...\n");
  tsc_delay_usec (100000);      /* avoid race between COM1 and Reboot */
  AcpiHwWrite (AcpiGbl_FADT.ResetValue, &AcpiGbl_FADT.ResetRegister);
  asm volatile ("hlt");
  return 0;
}

static void
acpi_notify_handler (ACPI_HANDLE Device,
                     UINT32 Value,
                     void *Context)
{
  ACPI_BUFFER Path;
  ACPI_STATUS Status;
  logger_printf ("ACPI: acpi_notify_handler (0x%p, 0x%X)\n", Device, Value);
  Status = AcpiGetName (Device, ACPI_FULL_PATHNAME, &Path);
  if (ACPI_SUCCESS (Status)) {
    logger_printf ("    DeviceName=%s\n", Path.Pointer);
  }
}

void
acpi_secondary_init(void)
{
  ACPI_STATUS Status;
  //ACPI_HANDLE SysBusHandle;
  ACPI_STATUS DisplayOneDevice (ACPI_HANDLE, UINT32, void *, void **);
  /* Complete the ACPICA initialization sequence */

  Status = AcpiInitializeSubsystem ();
  if (ACPI_FAILURE (Status)) {
    com1_printf ("Failed to initialize ACPI.\n");
  }
  Status = AcpiReallocateRootTable ();
  if (ACPI_FAILURE (Status)) {
    com1_printf ("Failed: AcpiReallocateRootTable %d.\n", Status);
  }
  Status = AcpiLoadTables ();
  if (ACPI_FAILURE (Status)) {
    com1_printf ("Failed: AcpiLoadTables.\n");
  }
  Status = AcpiEnableSubsystem (ACPI_FULL_INITIALIZATION);
  if (ACPI_FAILURE (Status)) {
    com1_printf ("Failed: AcpiEnableSubsystem.\n");
  }
  Status = AcpiInitializeObjects (ACPI_FULL_INITIALIZATION);
  if (ACPI_FAILURE (Status)) {
    com1_printf ("Failed: AcpiInitializeObjects.\n");
  }

  /* Must enable IOAPIC before checking any PCI routing tables. */
  acpi_enable_IOAPIC ();

  /* Install System Control Interrupt */
  u8 vector = find_unused_vector (MINIMUM_VECTOR_PRIORITY);
  if (vector) {
    u64 flags = IOAPIC_DELIVERY_FIXED | IOAPIC_DESTINATION_LOGICAL;
    u8 gsi = acpi_sci_irq;
    /* SCI defaults to LEVEL/LOW in IO-APIC mode. */
    if ((acpi_sci_flags & ACPI_MADT_POLARITY_MASK) == ACPI_MADT_POLARITY_ACTIVE_HIGH)
      flags |= IOAPIC_POLARITY_HIGH;
    else
      flags |= IOAPIC_POLARITY_LOW;
    if ((acpi_sci_flags & ACPI_MADT_TRIGGER_MASK) == ACPI_MADT_TRIGGER_EDGE)
      flags |= IOAPIC_TRIGGER_EDGE;
    else
      flags |= IOAPIC_TRIGGER_LEVEL;
    if (IOAPIC_map_GSI (gsi, vector, 0x0100000000000000ULL | flags) != -1) {
      set_vector_handler (vector, acpi_irq_handler);
      logger_printf ("ACPI: mapped GSI 0x%X to vector 0x%X (%s, %s)\n",
                     gsi, vector,
                     flags & IOAPIC_TRIGGER_LEVEL ? "level" : "edge",
                     flags & IOAPIC_POLARITY_LOW ? "low" : "high");
    } else
      logger_printf ("ACPI: failed to map GSI\n");
  } else
    logger_printf ("ACPI: failed to find unused vector\n");

  logger_printf ("AcpiEnableEvent returned %d\n",
                 AcpiEnableEvent (ACPI_EVENT_POWER_BUTTON, 0));
  logger_printf ("AcpiInstallFixedEventHandler returned %d\n",
                 AcpiInstallFixedEventHandler (ACPI_EVENT_POWER_BUTTON, acpi_power_button, NULL));
  logger_printf ("AcpiInstallNotifyHandler returned %d\n",
                 AcpiInstallNotifyHandler (ACPI_ROOT_OBJECT, ACPI_SYSTEM_NOTIFY, acpi_notify_handler, NULL));
  logger_printf ("AcpiInstallNotifyHandler returned %d\n",
                 AcpiInstallNotifyHandler (ACPI_ROOT_OBJECT, ACPI_DEVICE_NOTIFY, acpi_notify_handler, NULL));

  extern u8 AcpiGbl_OsiData;
  AcpiGbl_OsiData=0;

  /* Walk the System Bus "\_SB_" and output info about each object
   * found. */

#if 0
  AcpiGetHandle (ACPI_ROOT_OBJECT, ACPI_NS_SYSTEM_BUS, &SysBusHandle);
  AcpiWalkNamespace (ACPI_TYPE_ANY, SysBusHandle, INT_MAX,
                     DisplayOneDevice, NULL, NULL);
#else
  AcpiGetDevices (NULL, DisplayOneDevice, NULL, NULL);
#endif
}

#define printf com1_printf
static int
process_acpi_tables (void)
{
  extern uint32 mp_LAPIC_addr;
  extern uint32 mp_num_IOAPICs;
  extern uint32 mp_IOAPIC_addr;
  extern mp_IOAPIC_info mp_IOAPICs[];
  extern uint32 mp_num_overrides;
  extern mp_int_override mp_overrides[];

  ACPI_STATUS status;

  status = AcpiInitializeTables (TableArray, ACPI_MAX_INIT_TABLES, FALSE);

  if (status == AE_OK) {
    ACPI_TABLE_MADT *madt;
    ACPI_TABLE_FADT *fadt;
    ACPI_TABLE_BOOT *boot;
    ACPI_TABLE_ASF *asf;
    ACPI_TABLE_MCFG *mcfg;
    ACPI_TABLE_HPET *hpet;
    ACPI_TABLE_TCPA *tcpa;
    ACPI_TABLE_SRAT *srat;
    ACPI_TABLE_DMAR *dmar;

    if (AcpiGetTable (ACPI_SIG_FADT, 0, (ACPI_TABLE_HEADER **) & fadt) ==
        AE_OK) {
      /* Fixed ACPI Description Table */
      printf ("Bootflags: %s %s %s\n",
              (fadt->BootFlags & ACPI_FADT_LEGACY_DEVICES) ?
              "HAS_LEGACY_DEVICES" : "NO_LEGACY_DEVICES",
              (fadt->BootFlags & ACPI_FADT_8042) ?
              "HAS_KBD_8042" : "NO_KBD_8042",
              (fadt->BootFlags & ACPI_FADT_NO_VGA) ?
              "NO_VGA_PROBING" : "VGA_PROBING_OK");
      printf ("Flags=0x%X SCI_IRQ=0x%X Pm1aEvt=0x%p Pm1aCtl=0x%p\n",
              fadt->Flags,
              fadt->SciInterrupt,
              fadt->Pm1aEventBlock,
              fadt->Pm1aControlBlock);
      printf ("ResetReg=0x%llX ResetSpace=%d ResetVal=0x%X\n",
              fadt->ResetRegister.Address, fadt->ResetRegister.SpaceId, fadt->ResetValue);
      acpi_sci_irq = fadt->SciInterrupt;
    } else {
      printf ("AcpiGetTable FADT: FAILED\n");
    }

    mp_ISA_bus_id = 0;
    if (AcpiGetTable (ACPI_SIG_MADT, 0, (ACPI_TABLE_HEADER **) & madt) ==
        AE_OK) {
      /* Multiple APIC Description Table */
      uint8 *ptr, *lim = (uint8 *) madt + madt->Header.Length;
      printf ("ACPI OEM: %.6s Compiler: %.4s LAPIC: %p Flags:%s\n",
              madt->Header.OemId,
              madt->Header.AslCompilerId,
              madt->Address,
              (madt->Flags & ACPI_MADT_PCAT_COMPAT) ? " PCAT_COMPAT" : "");
      mp_LAPIC_addr = madt->Address;
      ptr = (uint8 *) madt + sizeof (ACPI_TABLE_MADT);
      while (ptr < lim) {
        switch (((ACPI_SUBTABLE_HEADER *) ptr)->Type) {
        case ACPI_MADT_TYPE_LOCAL_APIC:{
          /* Processor entry */
          ACPI_MADT_LOCAL_APIC *sub = (ACPI_MADT_LOCAL_APIC *) ptr;
          printf ("Processor: %X APIC-ID: %X %s",
                  sub->ProcessorId,
                  sub->Id,
                  sub->LapicFlags & 1 ? "(enabled)" : "(disabled)");
          if (acpi_add_processor (sub)) {
            printf (" (booted)");
          }
          printf ("\n");
          break;
        }
        case ACPI_MADT_TYPE_IO_APIC:{
          /* IO-APIC entry */
          ACPI_MADT_IO_APIC *sub = (ACPI_MADT_IO_APIC *) ptr;
          printf ("IO-APIC ID: %X Address: %X IRQBase: %X\n",
                  sub->Id, sub->Address, sub->GlobalIrqBase);
          if (mp_num_IOAPICs == MAX_IOAPICS)
            panic ("Too many IO-APICs.");
          mp_IOAPIC_addr = sub->Address;
          mp_IOAPICs[mp_num_IOAPICs].id = sub->Id;
          mp_IOAPICs[mp_num_IOAPICs].address = sub->Address;
          mp_IOAPICs[mp_num_IOAPICs].startGSI = sub->GlobalIrqBase;
          mp_IOAPICs[mp_num_IOAPICs].numGSIs = IOAPIC_num_entries();
          mp_num_IOAPICs++;
          break;
        }
        case ACPI_MADT_TYPE_INTERRUPT_OVERRIDE:{
          /* Interrupt Override entry */
          ACPI_MADT_INTERRUPT_OVERRIDE *sub =
            (ACPI_MADT_INTERRUPT_OVERRIDE *) ptr;
          printf ("Int. Override: Bus: %X SourceIRQ: %X GlobalIRQ: %X Flags: %X\n",
                  sub->Bus, sub->SourceIrq, sub->GlobalIrq, sub->IntiFlags);
          if (mp_num_overrides == MAX_INT_OVERRIDES)
            panic ("Too many interrupt overrides.");
          mp_overrides[mp_num_overrides].src_bus = sub->Bus;
          mp_overrides[mp_num_overrides].src_IRQ = sub->SourceIrq;
          mp_overrides[mp_num_overrides].dest_GSI = sub->GlobalIrq;
          mp_num_overrides++;
          /* (special case) SCI interrupt: it can be different in ACPI
           * tables vs Intel MPS tables. */
          if (sub->SourceIrq == acpi_sci_irq) {
            acpi_sci_irq = sub->GlobalIrq;
            acpi_sci_flags = sub->IntiFlags;
          }
          break;
        }
        default:
          printf ("MADT sub-entry: %X\n",
                  ((ACPI_SUBTABLE_HEADER *) ptr)->Type);
          break;
        }
        ptr += ((ACPI_SUBTABLE_HEADER *) ptr)->Length;
      }
    } else {
      printf ("AcpiGetTable MADT: FAILED\n");
      return 0;
    }
    if (AcpiGetTable (ACPI_SIG_BOOT, 0, (ACPI_TABLE_HEADER **) & boot) ==
        AE_OK) {
      /* Simple Boot Information Table */
      printf ("BOOT: CmosIndex=0x%X\n", boot->CmosIndex);
    }
    if (AcpiGetTable (ACPI_SIG_TCPA, 0, (ACPI_TABLE_HEADER **) & tcpa) ==
        AE_OK) {
      /* Trusted Computing Platform Alliance table */
      printf ("TCPA: MaxLog=0x%X Addr: 0x%llX\n",
              tcpa->MaxLogLength,
              tcpa->LogAddress);
    }
    if (AcpiGetTable (ACPI_SIG_HPET, 0, (ACPI_TABLE_HEADER **) & hpet) ==
        AE_OK) {
      /* High Precision Event Timer table */
      printf ("HPET: ID: 0x%X Addr: 0x%p Seq#: 0x%X MinTick: 0x%X Flags: 0x%X\n",
              hpet->Id,
              hpet->Address,
              (uint32) hpet->Sequence,
              (uint32) hpet->MinimumTick, (uint32) hpet->Flags);
    }
    if (AcpiGetTable (ACPI_SIG_MCFG, 0, (ACPI_TABLE_HEADER **) & mcfg) ==
        AE_OK) {
      /* PCI Memory Mapped Configuration table */
      printf ("MCFG: Length: %d Reserved: 0x%llX\n",
              mcfg->Header.Length,
              mcfg->Reserved);
    }
    if (AcpiGetTable (ACPI_SIG_ASF, 0, (ACPI_TABLE_HEADER **) & asf) == AE_OK) {
      /* Alert Standard Format table */
      printf ("ASF: Length: %d\n", mcfg->Header.Length);
    }
    if (AcpiGetTable (ACPI_SIG_SRAT, 0, (ACPI_TABLE_HEADER **) & srat) == AE_OK) {
      /* System Resource Affinity Table */
      printf ("SRAT: Length: %d\n", srat->Header.Length);
      acpi_parse_srat (srat);
    }
    if (AcpiGetTable (ACPI_SIG_DMAR, 0, (ACPI_TABLE_HEADER **) & dmar) == AE_OK) {
      /* DMA Remapping */
      printf ("DMAR: Length: %d Flags: 0x%X Width: %d\n",
              dmar->Header.Length,
              dmar->Flags,
              dmar->Width);
    }
  } else
    return 0;

  return mp_num_cpus;
}

#undef printf

/* A small wrapper around smp_boot_cpu() which does some checks and
 * maintains two small tables. */
static int
acpi_add_processor (ACPI_MADT_LOCAL_APIC * ptr)
{
#ifndef NO_SMP
  uint8 this_apic_id = LAPIC_get_physical_ID ();
  uint8 apic_id = ptr->Id;

  if (!(ptr->LapicFlags & 1))
    return 0;                   /* disabled processor */
  if (this_apic_id == apic_id)
    return 0;                   /* bootstrap processor */

  if (smp_boot_cpu (apic_id, APIC_VER_NEW)) {
    CPU_to_APIC[mp_num_cpus] = apic_id;
    APIC_to_CPU[apic_id] = mp_num_cpus;
    mp_num_cpus++;
    return 1;
  } else
    return 0;
#else
  return 0;
#endif
}

/* ************************************************** */

extern char const *AcpiGbl_ExceptionNames_Env[];

ACPI_STATUS
GetLnkIrq (ACPI_RESOURCE *Resource, void *Context)
{
  pci_irq_t *irq = (pci_irq_t *)Context;
  if (Resource->Type == ACPI_RESOURCE_TYPE_IRQ) {
    irq->gsi = Resource->Data.Irq.Interrupts[0];
    irq->trigger = Resource->Data.Irq.Triggering;
    irq->polarity = Resource->Data.Irq.Polarity;
    return AE_CTRL_TERMINATE;
  }
  return AE_OK;
}

void
GetLnkInfo (char *lnkname, pci_irq_t *irq)
{
  ACPI_HANDLE Handle;
  ACPI_STATUS Status;

  Status = AcpiGetHandle (NULL, lnkname, &Handle);
  if (ACPI_SUCCESS (Status)) {
    AcpiWalkResources (Handle, "_CRS", GetLnkIrq, (void *) irq);
  }
}

#define READ(bus, slot, func, reg, type)                \
  pci_read_##type (pci_addr (bus, slot, func, reg))

ACPI_STATUS
DisplayOneDevice (ACPI_HANDLE ObjHandle, UINT32 Level, void *Context,
                  void **RetVal)
{
  ACPI_STATUS Status;
  ACPI_DEVICE_INFO *Info;
  ACPI_BUFFER Path;
  ACPI_BUFFER Result;
  ACPI_OBJECT Obj;
  char Buffer[256];
  uint8 prt_buf[1024];
  ACPI_BUFFER Prt = { .Length = sizeof (prt_buf), .Pointer = prt_buf };
  ACPI_PCI_ROUTING_TABLE *prtd;
  uint32 addr=0;
  bool pcibus=FALSE;
  u8 busnum;

  Path.Length = sizeof (Buffer);
  Path.Pointer = Buffer;

  Status = AcpiGetName (ObjHandle, ACPI_FULL_PATHNAME, &Path);
  if (ACPI_SUCCESS (Status)) {
    com1_printf ("%s: \n", Path.Pointer);
  }
  Status = AcpiGetObjectInfo (ObjHandle, &Info);
  if (ACPI_SUCCESS (Status)) {
    com1_printf ("    ");
    if (Info->Flags & ACPI_PCI_ROOT_BRIDGE) {
      com1_printf (" PCI_ROOT");
      busnum = 0;
      pcibus = TRUE;
    }
    if (Info->Valid & ACPI_VALID_STA)
      com1_printf (" STA %.8X", Info->CurrentStatus);
    if (Info->Valid & ACPI_VALID_ADR) {
      com1_printf (" ADR %.8X", Info->Address);
      addr = Info->Address >> 16;
    }
    if (Info->Valid & ACPI_VALID_HID)
      com1_printf (" HID %s", Info->HardwareId.String);
    if (Info->Valid & ACPI_VALID_UID)
      com1_printf (" UID %s", Info->UniqueId.String);
    if (Info->Valid & ACPI_VALID_CID)
      com1_printf (" CID");

    ACPI_FREE (Info);
  }

  Result.Length = sizeof (Obj);
  Result.Pointer = &Obj;
  Status =
    AcpiEvaluateObjectTyped (ObjHandle, "_DDN", NULL, &Result,
                             ACPI_TYPE_STRING);
  if (ACPI_SUCCESS (Status)) {
    com1_printf (" DDN=%s", Obj.String.Pointer);
  }

  Result.Length = sizeof (Obj);
  Result.Pointer = &Obj;
  Status =
    AcpiEvaluateObjectTyped (ObjHandle, "_STR", NULL, &Result,
                             ACPI_TYPE_STRING);
  if (ACPI_SUCCESS (Status)) {
    com1_printf (" STR=%s", Obj.String.Pointer);
  }

  Result.Length = sizeof (Obj);
  Result.Pointer = &Obj;
  Status =
    AcpiEvaluateObjectTyped (ObjHandle, "_MLS", NULL, &Result,
                             ACPI_TYPE_STRING);
  if (ACPI_SUCCESS (Status)) {
    com1_printf (" MLS=%s", Obj.String.Pointer);
  }

  Status =
    AcpiEvaluateObjectTyped (ObjHandle, "_BBN", NULL, &Result,
                             ACPI_TYPE_INTEGER);
  if (ACPI_SUCCESS (Status)) {
    com1_printf (" BBN=%d", Obj.Integer.Value);
  } else if (Status != AE_NOT_FOUND)
    com1_printf (" bbnERR=%d", Status);

  Status =
    AcpiEvaluateObjectTyped (ObjHandle, "_PXM", NULL, &Result,
                             ACPI_TYPE_INTEGER);
  if (ACPI_SUCCESS (Status)) {
    com1_printf (" PXM=%d", Obj.Integer.Value);
  } else if (Status != AE_NOT_FOUND)
    com1_printf (" pxmERR=%d", Status);


  com1_printf ("\n");

  for (;;) {
    Status = AcpiGetIrqRoutingTable (ObjHandle, &Prt);
    if (ACPI_FAILURE (Status)) {
      if (Status == AE_BUFFER_OVERFLOW) {
        com1_printf ("AcpiGetIrqRoutingTable failed: BUFFER OVERFLOW\n");
      }
      break;
    } else break;
  }
  if (ACPI_SUCCESS (Status)) {
    int i;

    /* Check if ObjHandle refers to a non-root PCI bus */
    if (READ (0, addr, 0, 0, dword) != 0xFFFFFFFF) {
      u8 hdrtype = READ (0, addr, 0, 0x0E, byte);
      if (hdrtype == 1) {
        /* PCI-to-PCI bridge headerType == 1 */
        busnum = READ (0, addr, 0, 0x19, byte);
        com1_printf ("  bus=0x%.02X\n", busnum);
        pcibus = TRUE;
      }
    }

    for (i=0;i<sizeof(prt_buf);) {
      pci_irq_t irq;
      prtd = (ACPI_PCI_ROUTING_TABLE *)(&prt_buf[i]);
      if (prtd->Length == 0) break;

      if (pcibus) {
        irq.bus = busnum;
        irq.dev = (uint32) ((prtd->Address >> 16) & 0xFF);
        irq.pin = prtd->Pin + 1; /* ACPI numbers pins from 0 */
        irq.gsi = 0;
      }

      if (prtd->Source[0]) {
        com1_printf ("  PRT entry: len=%d pin=%d addr=%p srcidx=0x%x src=%s\n",
                     prtd->Length,
                     prtd->Pin,
                     (uint32)prtd->Address,
                     prtd->SourceIndex,
                     &prtd->Source[0]);
        GetLnkInfo (&prtd->Source[0], &irq);
      } else {
        com1_printf ("  PRT entry: len=%d pin=%d addr=%p fixed IRQ=0x%x\n",
                     prtd->Length,
                     prtd->Pin,
                     (uint32)prtd->Address,
                     prtd->SourceIndex);
        irq.gsi = prtd->SourceIndex;
        irq.polarity = POLARITY_DEFAULT;
        irq.trigger = TRIGGER_DEFAULT;
      }

      if (pcibus && irq.gsi != 0)
        pci_irq_register (&irq);

      i+=prtd->Length;
    }
  }

#if 0
  ACPI_STATUS DisplayResource (ACPI_RESOURCE *Resource, void *Context);
  com1_printf ("  _PRS:\n");
  AcpiWalkResources (ObjHandle, "_PRS", DisplayResource, NULL);
  com1_printf ("  _CRS:\n");
  AcpiWalkResources (ObjHandle, "_CRS", DisplayResource, NULL);
#endif
  return AE_OK;
}

ACPI_STATUS
DisplayResource (ACPI_RESOURCE *Resource, void *Context)
{
  int i;
  com1_printf ("Resource: type=%d ", Resource->Type);
  switch (Resource->Type) {
  case ACPI_RESOURCE_TYPE_IRQ:
    com1_printf ("IRQ dlen=%d trig=%d pol=%d shar=%d cnt=%d\n  int=",
                 Resource->Data.Irq.DescriptorLength,
                 Resource->Data.Irq.Triggering,
                 Resource->Data.Irq.Polarity,
                 Resource->Data.Irq.Sharable,
                 Resource->Data.Irq.InterruptCount);
    for (i=0;i<Resource->Data.Irq.InterruptCount;i++)
      com1_printf ("%.02X ", Resource->Data.Irq.Interrupts[i]);
    com1_printf ("\n");
    break;
  case ACPI_RESOURCE_TYPE_IO:
    com1_printf ("IO decode=0x%x align=0x%x addrlen=%d min=0x%.04X max=0x%.04X\n",
                 Resource->Data.Io.IoDecode,
                 Resource->Data.Io.Alignment,
                 Resource->Data.Io.AddressLength,
                 Resource->Data.Io.Minimum,
                 Resource->Data.Io.Maximum);
    break;
  case ACPI_RESOURCE_TYPE_END_TAG:
    com1_printf ("end_tag\n");
    break;
  case ACPI_RESOURCE_TYPE_ADDRESS16:
    com1_printf ("ADDR16 type=%d min=0x%.04X max=0x%.04X gran=0x%.04X trans=0x%.04X\n",
                 Resource->Data.Address16.ResourceType,
                 Resource->Data.Address16.Minimum,
                 Resource->Data.Address16.Maximum,
                 Resource->Data.Address16.Granularity,
                 Resource->Data.Address16.TranslationOffset);
    break;
  case ACPI_RESOURCE_TYPE_ADDRESS32:
    com1_printf ("ADDR32 type=%d min=0x%.04X max=0x%.04X\n",
                 Resource->Data.Address32.ResourceType,
                 Resource->Data.Address32.Minimum,
                 Resource->Data.Address32.Maximum);
    break;
  default:
    com1_printf ("unhandled\n");
    break;
  }
  return AE_OK;
}

/* ************************************************** */

static void
acpi_parse_srat (ACPI_TABLE_SRAT *srat)
{
  ACPI_SRAT_CPU_AFFINITY *cpu;
  ACPI_SRAT_MEM_AFFINITY *mem;
  ACPI_SRAT_X2APIC_CPU_AFFINITY *x2apic;
  u32 prox;
  ACPI_SUBTABLE_HEADER *sub = (ACPI_SUBTABLE_HEADER *) &srat[1];
  int len = srat->Header.Length;

  len -= sizeof (ACPI_TABLE_SRAT);
  while (len > 0) {
    /* len = size of remaining subtables */
    /* sub = pointer to subtable */
    switch (sub->Type) {
    case ACPI_SRAT_TYPE_CPU_AFFINITY:
      cpu = (ACPI_SRAT_CPU_AFFINITY *) sub;
      prox = cpu->ProximityDomainLo |
        (cpu->ProximityDomainHi[0] << 0x08 |
         cpu->ProximityDomainHi[1] << 0x10 |
         cpu->ProximityDomainHi[2] << 0x18);
      logger_printf ("SRAT: CPU: prox=0x%X apicID=0x%X flags=0x%X%s sapicEID=0x%X\n",
                     prox, cpu->ApicId, cpu->Flags,
                     cpu->Flags & ACPI_SRAT_CPU_ENABLED ? " enabled" : "",
                     cpu->LocalSapicEid);
      break;
    case ACPI_SRAT_TYPE_MEMORY_AFFINITY:
      mem = (ACPI_SRAT_MEM_AFFINITY *) sub;
      logger_printf ("SRAT: MEM: prox=0x%X base=0x%llX len=0x%llX flags=0x%X%s%s%s\n",
                     mem->ProximityDomain, mem->BaseAddress, mem->Length, mem->Flags,
                     mem->Flags & ACPI_SRAT_MEM_ENABLED ? " enabled" : "",
                     mem->Flags & ACPI_SRAT_MEM_HOT_PLUGGABLE ? " hotplug" : "",
                     mem->Flags & ACPI_SRAT_MEM_NON_VOLATILE ? " nonvolatile" : "");
      break;
    case ACPI_SRAT_TYPE_X2APIC_CPU_AFFINITY:
      x2apic = (ACPI_SRAT_X2APIC_CPU_AFFINITY *) sub;
      logger_printf ("SRAT: X2APIC: prox=0x%X apicID=0x%X flags=0x%X%s clockDom=0x%X\n",
                     x2apic->ProximityDomain, x2apic->ApicId, x2apic->Flags,
                     x2apic->Flags & ACPI_SRAT_CPU_ENABLED ? " enabled" : "",
                     x2apic->ClockDomain);
      break;
    default:
      logger_printf ("SRAT: unknown subtype type=%d\n", sub->Type);
      break;
    }
    len -= sub->Length;
    sub = (ACPI_SUBTABLE_HEADER *) &((u8 *) sub)[sub->Length];
  }
}

/* End ACPI support */


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
