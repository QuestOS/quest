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
#define ACPI_MAX_INIT_TABLES 16
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

void
acpi_secondary_init(void)
{
  ACPI_STATUS Status;
  ACPI_HANDLE SysBusHandle;
  ACPI_STATUS DisplayOneDevice (ACPI_HANDLE, UINT32, void *, void **);

  /* Complete the ACPICA initialization sequence */

  Status = AcpiInitializeSubsystem ();
  if (ACPI_FAILURE (Status)) {
    com1_printf ("Failed to initialize ACPI.\n");
  }
  Status = AcpiReallocateRootTable ();
  if (ACPI_FAILURE (Status)) {
    com1_printf ("Failed: AcpiReallocateRootTable.\n");
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

  /* Walk the System Bus "\_SB_" and output info about each object
   * found. */

  AcpiGetHandle (ACPI_ROOT_OBJECT, ACPI_NS_SYSTEM_BUS, &SysBusHandle);
  AcpiWalkNamespace (ACPI_TYPE_ANY, SysBusHandle, INT_MAX,
                     DisplayOneDevice, NULL, NULL);
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

  status = AcpiInitializeTables (TableArray, ACPI_MAX_INIT_TABLES, TRUE);
  if (status == AE_OK) {
    ACPI_TABLE_MADT *madt;
    ACPI_TABLE_FADT *fadt;
    ACPI_TABLE_BOOT *boot;
    ACPI_TABLE_ASF *asf;
    ACPI_TABLE_MCFG *mcfg;
    ACPI_TABLE_HPET *hpet;
    ACPI_TABLE_TCPA *tcpa;

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
    } else {
      printf ("AcpiGetTable FADT: FAILED\n");
    }
    if (AcpiGetTable (ACPI_SIG_BOOT, 0, (ACPI_TABLE_HEADER **) & boot) ==
        AE_OK) {
      /* Simple Boot Information Table */
      printf ("BOOT: CmosIndex=%X\n", boot->CmosIndex);
    }
    if (AcpiGetTable (ACPI_SIG_TCPA, 0, (ACPI_TABLE_HEADER **) & tcpa) ==
        AE_OK) {
      /* Trusted Computing Platform Alliance table */
      printf ("TCPA: MaxLog=%X Addr Upper: %.8X Lower: %.8X\n",
              tcpa->MaxLogLength,
              (uint32) (tcpa->LogAddress >> 32), (uint32) tcpa->LogAddress);
    }
    if (AcpiGetTable (ACPI_SIG_HPET, 0, (ACPI_TABLE_HEADER **) & hpet) ==
        AE_OK) {
      /* High Precision Event Timer table */
      printf ("HPET: ID: %X Addr: %p Seq#: %X MinTick: %X Flags: %X\n",
              hpet->Id,
              hpet->Address,
              (uint32) hpet->Sequence,
              (uint32) hpet->MinimumTick, (uint32) hpet->Flags);
    }
    if (AcpiGetTable (ACPI_SIG_MCFG, 0, (ACPI_TABLE_HEADER **) & mcfg) ==
        AE_OK) {
      /* PCI Memory Mapped Configuration table */
      printf ("MCFG: Length: %X Reserved: %.8X%.8X\n",
              mcfg->Header.Length,
              *((uint32 *) mcfg->Reserved),
              *((uint32 *) (mcfg->Reserved + 4)));
    }
    if (AcpiGetTable (ACPI_SIG_ASF, 0, (ACPI_TABLE_HEADER **) & asf) == AE_OK) {
      /* Alert Standard Format table */
      printf ("ASF: Length: %X\n", mcfg->Header.Length);
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
      pcibus = TRUE;
    }
    if (Info->Valid & ACPI_VALID_STA)
      com1_printf (" STA %.8X", Info->CurrentStatus);
    if (Info->Valid & ACPI_VALID_ADR) {
      com1_printf (" ADR %.8X", Info->Address);
      addr = Info->Address;
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
    com1_printf (" BBN=%x", Obj.Integer.Value);
  }


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
    for (i=0;i<sizeof(prt_buf);) {
      pci_irq_t irq;
      prtd = (ACPI_PCI_ROUTING_TABLE *)(&prt_buf[i]);
      if (prtd->Length == 0) break;

      /* only support bus==0 for now */
      if (addr == 0 && pcibus) {
        irq.bus = 0;
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

      if (addr == 0 && pcibus && irq.gsi != 0)
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
