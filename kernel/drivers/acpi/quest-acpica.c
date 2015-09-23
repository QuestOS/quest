/******************************************************************************
 *
 * Name: quest-acpica.c - Quest interface for ACPICA
 *
 *****************************************************************************/


/******************************************************************************
 *
 * 1. Copyright Notice
 *
 * Some or all of this work - Copyright (c) 1999 - 2009, Intel Corp.
 * All rights reserved.
 *
 * 2. License
 *
 * 2.1. This is your license from Intel Corp. under its intellectual property
 * rights.  You may have additional license terms from the party that provided
 * you this software, covering your right to use that party's intellectual
 * property rights.
 *
 * 2.2. Intel grants, free of charge, to any person ("Licensee") obtaining a
 * copy of the source code appearing in this file ("Covered Code") an
 * irrevocable, perpetual, worldwide license under Intel's copyrights in the
 * base code distributed originally by Intel ("Original Intel Code") to copy,
 * make derivatives, distribute, use and display any portion of the Covered
 * Code in any form, with the right to sublicense such rights; and
 *
 * 2.3. Intel grants Licensee a non-exclusive and non-transferable patent
 * license (with the right to sublicense), under only those claims of Intel
 * patents that are infringed by the Original Intel Code, to make, use, sell,
 * offer to sell, and import the Covered Code and derivative works thereof
 * solely to the minimum extent necessary to exer
 se the above copyright
 * license, and in no event shall the patent license extend to any additions
 * to or modifications of the Original Intel Code.  No other license or right
 * is granted directly or by implication, estoppel or otherwise;
 *
 * The above copyright and patent license is granted only if the following
 * conditions are met:
 *
 * 3. Conditions
 *
 * 3.1. Redistribution of Source with Rights to Further Distribute Source.
 * Redistribution of source code of any substantial portion of the Covered
 * Code or modification with rights to further distribute source must include
 * the above Copyright Notice, the above License, this list of Conditions,
 * and the following Disclaimer and Export Compliance provision.  In addition,
 * Licensee must cause all Covered Code to which Licensee contributes to
 * contain a file documenting the changes Licensee made to create that Covered
 * Code and the date of any change.  Licensee must include in that file the
 * documentation of any changes made by any predecessor Licensee.  Licensee
 * must include a prominent statement that the modification is derived,
 * directly or indirectly, from Original Intel Code.
 *
 * 3.2. Redistribution of Source with no Rights to Further Distribute Source.
 * Redistribution of source code of any substantial portion of the Covered
 * Code or modification without rights to further distribute source must
 * include the following Disclaimer and Export Compliance provision in the
 * documentation and/or other materials provided with distribution.  In
 * addition, Licensee may not authorize further sublicense of source of any
 * portion of the Covered Code, and must include terms to the effect that the
 * license from Licensee to its licensee is limited to the intellectual
 * property embodied in the software Licensee provides to its licensee, and
 * not to intellectual property embodied in modifications its licensee may
 * make.
 *
 * 3.3. Redistribution of Executable. Redistribution in executable form of any
 * substantial portion of the Covered Code or modification must reproduce the
 * above Copyright Notice, and the following Disclaimer and Export Compliance
 * provision in the documentation and/or other materials provided with the
 * distribution.
 *
 * 3.4. Intel retains all right, title, and interest in and to the Original
 * Intel Code.
 *
 * 3.5. Neither the name Intel nor any other trademark owned or controlled by
 * Intel shall be used in advertising or otherwise to promote the sale, use or
 * other dealings in products derived from or relating to the Covered Code
 * without prior written authorization from Intel.
 *
 * 4. Disclaimer and Export Compliance
 *
 * 4.1. INTEL MAKES NO WARRANTY OF ANY KIND REGARDING ANY SOFTWARE PROVIDED
 * HERE.  ANY SOFTWARE ORIGINATING FROM INTEL OR DERIVED FROM INTEL SOFTWARE
 * IS PROVIDED "AS IS," AND INTEL WILL NOT PROVIDE ANY SUPPORT,  ASSISTANCE,
 * INSTALLATION, TRAINING OR OTHER SERVICES.  INTEL WILL NOT PROVIDE ANY
 * UPDATES, ENHANCEMENTS OR EXTENSIONS.  INTEL SPECIFICALLY DISCLAIMS ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT AND FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * 4.2. IN NO EVENT SHALL INTEL HAVE ANY LIABILITY TO LICENSEE, ITS LICENSEES
 * OR ANY OTHER THIRD PARTY, FOR ANY LOST PROFITS, LOST DATA, LOSS OF USE OR
 * COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES, OR FOR ANY INDIRECT,
 * SPECIAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THIS AGREEMENT, UNDER ANY
 * CAUSE OF ACTION OR THEORY OF LIABILITY, AND IRRESPECTIVE OF WHETHER INTEL
 * HAS ADVANCE NOTICE OF THE POSSIBILITY OF SUCH DAMAGES.  THESE LIMITATIONS
 * SHALL APPLY NOTWITHSTANDING THE FAILURE OF THE ESSENTIAL PURPOSE OF ANY
 * LIMITED REMEDY.
 *
 * 4.3. Licensee shall not export, either directly or indirectly, any of this
 * software or system incorporating such software without first obtaining any
 * required license or other approval from the U. S. Department of Commerce or
 * any other agency or department of the United States Government.  In the
 * event Licensee exports any such software from the United States or
 * re-exports any such software from a foreign destination, Licensee shall
 * ensure that the distribution and export/re-export of the software is in
 * compliance with all laws, regulations, orders, or other restrictions of the
 * U.S. Export Administration Regulations. Licensee agrees that neither it nor
 * any of its subsidiaries will export/re-export any technical data, process,
 * software, or service, directly or indirectly, to any country for which the
 * United States government or any agency thereof requires an export license,
 * other governmental approval, or letter of assurance, without first obtaining
 * such license, approval or letter.
 *
 *****************************************************************************/

/*                    The Quest Operating System
 *  Portions Copyright (C) 2005-2012  Richard West, Boston University
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

/* Ported to Quest by: Matthew Danish (md) */

#include"smp/spinlock.h"
#include"smp/semaphore.h"
#include"kernel.h"
#include"drivers/pci/pci.h"
#include"drivers/acpi/acpi.h"
#include"util/printf.h"
#include"mem/mem.h"
#include"smp/smp.h"
#include"smp/apic.h"
#include "sched/sched.h"


#define ACPI_DEBUG
#define ACPI_DEBUG_COM1

#ifdef ACPICA_DEBUG_COM1
#define DLOG_COM1(fmt, ...) com1_printf(fmt,##__VA_ARGS__)
#else
#define DLOG_COM1(fmt,...) ;
#endif

#ifdef ACPICA_DEBUG
#define DLOG(fmt, ...) logger_printf(fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

/*
 * OSL Initialization and shutdown primitives
 */
ACPI_STATUS
AcpiOsInitialize (void)
{
  return AE_OK;
}


ACPI_STATUS
AcpiOsTerminate (void)
{
  return AE_OK;
}



/*
 * ACPI Table interfaces
 */
ACPI_PHYSICAL_ADDRESS
AcpiOsGetRootPointer (void)
{
  ACPI_SIZE addr;
  if (AcpiFindRootPointer (&addr) == AE_OK)
    return (ACPI_PHYSICAL_ADDRESS) addr;
  else
    return 0;
}


ACPI_STATUS
AcpiOsPredefinedOverride (const ACPI_PREDEFINED_NAMES * InitVal,
                          ACPI_STRING * NewVal)
{
  *NewVal = NULL;
  return AE_OK;
}


ACPI_STATUS
AcpiOsTableOverride (ACPI_TABLE_HEADER * ExistingTable,
                     ACPI_TABLE_HEADER ** NewTable)
{
  *NewTable = NULL;
  return AE_OK;
}

ACPI_STATUS
AcpiOsPhysicalTableOverride (ACPI_TABLE_HEADER * ExistingTable,
                             ACPI_PHYSICAL_ADDRESS * NewAddress,
                             UINT32 * NewTableLength)
{
  NewAddress= NULL;
  *NewTableLength = 0;
  return AE_OK;
}

/*
 * Spinlock primitives
 */
ACPI_STATUS
AcpiOsCreateLock (ACPI_SPINLOCK * OutHandle)
{
  *OutHandle = AcpiOsAllocate (sizeof (spinlock));
  spinlock_init (*OutHandle);
  return AE_OK;
}


void
AcpiOsDeleteLock (ACPI_SPINLOCK Handle)
{
  AcpiOsFree (Handle);
  return;
}


ACPI_CPU_FLAGS
AcpiOsAcquireLock (ACPI_SPINLOCK Handle)
{
  spinlock_lock (Handle);
  return 0;
}


void
AcpiOsReleaseLock (ACPI_SPINLOCK Handle, ACPI_CPU_FLAGS Flags)
{
  spinlock_unlock (Handle);
  return;
}



/*
 * Semaphore primitives
 */
ACPI_STATUS
AcpiOsCreateSemaphore (UINT32 MaxUnits,
                       UINT32 InitialUnits, ACPI_SEMAPHORE * OutHandle)
{
  *OutHandle = AcpiOsAllocate (sizeof (semaphore));
  semaphore_init (*OutHandle, MaxUnits, InitialUnits);
  return AE_OK;
}


ACPI_STATUS
AcpiOsDeleteSemaphore (ACPI_SEMAPHORE Handle)
{
  semaphore_destroy (Handle);
  AcpiOsFree (Handle);
  return AE_OK;
}


ACPI_STATUS
AcpiOsWaitSemaphore (ACPI_SEMAPHORE Handle, UINT32 Units, UINT16 Timeout)
{
  semaphore_wait (Handle, Units, Timeout);
  return AE_OK;
}


ACPI_STATUS
AcpiOsSignalSemaphore (ACPI_SEMAPHORE Handle, UINT32 Units)
{
  if (semaphore_signal (Handle, Units) == 0)
    return AE_OK;
  else
    return AE_LIMIT;
}



/*
 * Mutex primitives. May be configured to use semaphores instead via
 * ACPI_MUTEX_TYPE (see platform/acenv.h)
 */
#if (ACPI_MUTEX_TYPE != ACPI_BINARY_SEMAPHORE)

ACPI_STATUS
AcpiOsCreateMutex (ACPI_MUTEX * OutHandle)
{
  return;
}


void
AcpiOsDeleteMutex (ACPI_MUTEX Handle)
{
  return;
}


ACPI_STATUS
AcpiOsAcquireMutex (ACPI_MUTEX Handle, UINT16 Timeout)
{
  return AE_NOT_IMPLEMENTED;
}


void
AcpiOsReleaseMutex (ACPI_MUTEX Handle)
{
  return;
}

#endif


void *
AcpiOsAllocate (ACPI_SIZE Size)
{
  return kmalloc(Size);
}

void
AcpiOsFree (void *Memory)
{
  kfree(Memory);
}

void *
AcpiOsMapMemory (ACPI_PHYSICAL_ADDRESS Where, ACPI_SIZE Length)
{
  ACPI_PHYSICAL_ADDRESS start_frame = Where & (~0xFFF);
  ACPI_PHYSICAL_ADDRESS end_frame = (Where + Length) & (~0xFFF);
  ACPI_SIZE num_frames = ((end_frame - start_frame) >> 12) + 1;
  void *virt = map_contiguous_virtual_pages (start_frame | 3, num_frames);
  if (virt)
    return (void *) ((uint32) virt | (Where & 0xFFF));        /* mask back in the offset */
  else
    return NULL;
}


void
AcpiOsUnmapMemory (void *LogicalAddress, ACPI_SIZE Size)
{
  uint32 start_addr = (uint32) LogicalAddress & (~0xFFF);
  uint32 end_addr = ((uint32) LogicalAddress + Size) & (~0xFFF);
  ACPI_SIZE num_pages = ((end_addr - start_addr) >> 12) + 1;

  return unmap_virtual_pages ((void *) start_addr, num_pages);
}


ACPI_STATUS
AcpiOsGetPhysicalAddress (void *LogicalAddress,
                          ACPI_PHYSICAL_ADDRESS * PhysicalAddress)
{
  *PhysicalAddress = (ACPI_PHYSICAL_ADDRESS) get_phys_addr (LogicalAddress);
  return AE_OK;
}



#if 0
/*
 * Memory/Object Cache
 */
ACPI_STATUS
AcpiOsCreateCache (char *CacheName,
                   UINT16 ObjectSize,
                   UINT16 MaxDepth, ACPI_CACHE_T ** ReturnCache)
{
  return AE_NOT_IMPLEMENTED;
}


ACPI_STATUS
AcpiOsDeleteCache (ACPI_CACHE_T * Cache)
{
  return AE_NOT_IMPLEMENTED;
}


ACPI_STATUS
AcpiOsPurgeCache (ACPI_CACHE_T * Cache)
{
  return AE_NOT_IMPLEMENTED;
}


void *
AcpiOsAcquireObject (ACPI_CACHE_T * Cache)
{
  return NULL;
}


ACPI_STATUS
AcpiOsReleaseObject (ACPI_CACHE_T * Cache, void *Object)
{
  return AE_NOT_IMPLEMENTED;
}

#endif

/*
 * Interrupt handlers
 */

ACPI_OSD_HANDLER acpi_service_routine = NULL;
void *acpi_service_routine_context;

ACPI_STATUS
AcpiOsInstallInterruptHandler (UINT32 InterruptNumber,
                               ACPI_OSD_HANDLER ServiceRoutine, void *Context)
{
  DLOG ("AcpiOsInstallInterruptHandler (0x%X, 0x%p, 0x%p)\n",
                 InterruptNumber, ServiceRoutine, Context);

  acpi_service_routine = ServiceRoutine;
  acpi_service_routine_context = Context;
  //IOAPIC_map_GSI (IRQ_to_GSI (mp_ISA_bus_id, InterruptNumber),
  //                0x29, 0x0100000000000800LL);
  return AE_OK;
}


ACPI_STATUS
AcpiOsRemoveInterruptHandler (UINT32 InterruptNumber,
                              ACPI_OSD_HANDLER ServiceRoutine)
{
  acpi_service_routine = NULL;
  return AE_OK;
}



/*
 * Threads and Scheduling
 */
ACPI_THREAD_ID
AcpiOsGetThreadId (void)
{
  /* ACPICA whines about thread ID==0, even though it calls this
   * function during early boot -- far before anything like "threads"
   * could possibly even begin to exist.  Stupid Intel. */

  /* After whining it defaults to ThreadId=1.  So let's just shut it
   * up here and now. */
  return (str ()->tid) || 1;
}


ACPI_STATUS
AcpiOsExecute (ACPI_EXECUTE_TYPE Type,
               ACPI_OSD_EXEC_CALLBACK Function, void *Context)
{
  return AE_NOT_IMPLEMENTED;
}


void
AcpiOsWaitEventsComplete (void)
{
  return;
}


void
AcpiOsSleep (ACPI_INTEGER Milliseconds)
{
  sched_usleep (Milliseconds * 1000);
}


void
AcpiOsStall (UINT32 Microseconds)
{
  tsc_delay_usec (Microseconds);
}



/*
 * Platform and hardware-independent I/O interfaces
 */
ACPI_STATUS
AcpiOsReadPort (ACPI_IO_ADDRESS Address, UINT32 * Value, UINT32 Width)
{
  switch (Width) {
  case 8:
    *Value = inb (Address);
    break;
  case 16:
    *Value = inw (Address);
    break;
  case 32:
    *Value = inl (Address);
    break;
  }
  //DLOG ("AcpiOsReadPort (0x%.04X, %d) = 0x%X\n", Address, Width, (*Value) & ((1 << Width) - 1));
  return AE_OK;
}


ACPI_STATUS
AcpiOsWritePort (ACPI_IO_ADDRESS Address, UINT32 Value, UINT32 Width)
{
  //DLOG ("AcpiOsWritePort (0x%.04X, 0x%X, %d)\n", Address, (Value) & ((1 << Width) - 1), Width);
  switch (Width) {
  case 8:
    outb ((UINT8) Value, Address);
    break;
  case 16:
    outw ((UINT16) Value, Address);
    break;
  case 32:
    outl (Value, Address);
    break;
  }
  return AE_OK;
}



/*
 * Platform and hardware-independent physical memory interfaces
 */
ACPI_STATUS
AcpiOsReadMemory (ACPI_PHYSICAL_ADDRESS Address, UINT64 * Value, UINT32 Width)
{
  DLOG ("AcpiOsReadMemory (0x%p, %d)\n", Address, Width);
  if (Width != 8 && Width != 16 && Width != 32)
    return AE_BAD_PARAMETER;
  u32 frame = Address & ~(0xFFF);
  u32 offset = Address - frame;
  u8 *virt = map_virtual_page (frame | 3);
  if (!virt) return AE_NO_MEMORY;
  switch (Width) {
  case 8:
    *Value = (UINT64)*((u8 *) (&virt[offset]));
    break;
  case 16:
    *Value = (UINT64)*((u16 *) (&virt[offset]));
    break;
  case 32:
    *Value = (UINT64)*((u32 *) (&virt[offset]));
    break;
  case 64:
    *Value = *((u64 *) (&virt[offset]));
    break;
  }
  unmap_virtual_page (virt);
  return AE_OK;
}


ACPI_STATUS
AcpiOsWriteMemory (ACPI_PHYSICAL_ADDRESS Address, UINT64 Value, UINT32 Width)
{
  DLOG ("AcpiOsWriteMemory (0x%p, 0x%p, %d)\n", Address, Value, Width);
  if (Width != 8 && Width != 16 && Width != 32)
    return AE_BAD_PARAMETER;
  u32 frame = Address & ~(0xFFF);
  u32 offset = Address - frame;
  u8 *virt = map_virtual_page (frame | 3);
  if (!virt) return AE_NO_MEMORY;
  switch (Width) {
  case 8:
    *((u8 *) (&virt[offset])) = (u8) Value;
    break;
  case 16:
    *((u16 *) (&virt[offset])) = (u16) Value;
    break;
  case 32:
    *((u32 *) (&virt[offset])) = (u32) Value;
    break;
  case 64:
    *((u64 *) (&virt[offset])) = Value;
    break;
  }
  unmap_virtual_page (virt);
  return AE_OK;
}



/*
 * Platform and hardware-independent PCI configuration space access
 * Note: Can't use "Register" as a parameter, changed to "Reg" --
 * certain compilers complain.
 */
ACPI_STATUS
AcpiOsReadPciConfiguration (ACPI_PCI_ID * PciId,
                            UINT32 Reg, UINT64 *Value, UINT32 Width)
{
  pci_config_addr a;
  uint8 v8;
  uint16 v16;
  uint32 v32;
  DLOG_COM1
    ("AcpiOsReadPciConfiguration(%.4X:%.4X:%.4X:%.4X, %.8X, ..., %d)",
     PciId->Segment, PciId->Bus, PciId->Device, PciId->Function, Reg, Width);
  pci_config_addr_init (&a, PciId->Bus, PciId->Device, PciId->Function, Reg);
  switch (Width) {
  case 8:
    v8 = pci_read_byte (a);
    *((ACPI_INTEGER *) Value) = (ACPI_INTEGER) v8;
    break;
  case 16:
    v16 = pci_read_word (a);
    *((ACPI_INTEGER *) Value) = (ACPI_INTEGER) v16;
    break;
  case 32:
    v32 = pci_read_dword (a);
    *((ACPI_INTEGER *) Value) = (ACPI_INTEGER) v32;
    break;
  default:
    DLOG_COM1 (" = error\n");
    return AE_BAD_PARAMETER;
  }
  DLOG_COM1 (" = 0x%x\n", *((ACPI_INTEGER *) Value));
  return AE_OK;
}


ACPI_STATUS
AcpiOsWritePciConfiguration (ACPI_PCI_ID * PciId,
                             UINT32 Reg, ACPI_INTEGER Value, UINT32 Width)
{
  pci_config_addr a;
  DLOG_COM1 ("AcpiOsWritePciConfiguration\n");
  pci_config_addr_init (&a, PciId->Bus, PciId->Device, PciId->Function, Reg);
  switch (Width) {
  case 8:
    pci_write_byte (a, (uint8) Value);
    break;
  case 16:
    pci_write_word (a, (uint16) Value);
    break;
  case 32:
    pci_write_dword (a, (uint32) Value);
    break;
  default:
    return AE_BAD_PARAMETER;
  }
  return AE_OK;
}



/*
 * Interim function needed for PCI IRQ routing
 */
void
AcpiOsDerivePciId (ACPI_HANDLE Rhandle,
                   ACPI_HANDLE Chandle, ACPI_PCI_ID ** PciId)
{
  DLOG_COM1 ("AcpiOsDerivePciId\n");
  return;
}



/*
 * Miscellaneous
 */
ACPI_STATUS
AcpiOsValidateInterface (char *Interface)
{
  DLOG ("AcpiOsValidateInterface (%s)\n", Interface);
  return AE_SUPPORT;
}


BOOLEAN
AcpiOsReadable (void *Pointer, ACPI_SIZE Length)
{
  return AE_NOT_IMPLEMENTED;
}


BOOLEAN
AcpiOsWritable (void *Pointer, ACPI_SIZE Length)
{
  return AE_NOT_IMPLEMENTED;
}


UINT64
AcpiOsGetTimer (void)
{
  return AE_NOT_IMPLEMENTED;
}


ACPI_STATUS
AcpiOsSignal (UINT32 Function, void *Info)
{
  return AE_NOT_IMPLEMENTED;
}



/*
 * Debug print routines
 */
void ACPI_INTERNAL_VAR_XFACE
AcpiOsPrintf (const char *Fmt, ...)
{
  va_list Args;
  va_start (Args, Fmt);
  AcpiOsVprintf (Fmt, Args);
  va_end (Args);
}


void
AcpiOsVprintf (const char *Format, va_list Args)
{
  fun_vprintf (com1_putc, Format, Args);
}


void
AcpiOsRedirectOutput (void *Destination)
{
  return;
}



/*
 * Debug input
 */
ACPI_STATUS
AcpiOsGetLine (char *Buffer, UINT32 BufferLength, UINT32 *BytesRead)
{
  return AE_NOT_IMPLEMENTED;
}



/*
 * Directory manipulation
 */
void *
AcpiOsOpenDirectory (char *Pathname,
                     char *WildcardSpec, char RequestedFileType)
{
  return NULL;
}

char *
AcpiOsGetNextFilename (void *DirHandle)
{
  return NULL;
}


void
AcpiOsCloseDirectory (void *DirHandle)
{
  return;
}



/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
