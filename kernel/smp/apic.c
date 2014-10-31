/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
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
#include "util/printf.h"

#define APIC_BROADCAST_ID 0xFF
//#define MAX_CPUS          APIC_BROADCAST_ID

#define LAPIC_ADDR_DEFAULT  0xFEE00000uL
#define IOAPIC_ADDR_DEFAULT 0xFEC00000uL

uint32 mp_LAPIC_addr = LAPIC_ADDR_DEFAULT;
#define MP_LAPIC_READ(x)   (*((volatile uint32 *) (mp_LAPIC_addr+(x))))
#define MP_LAPIC_WRITE(x,y) (*((volatile uint32 *) (mp_LAPIC_addr+(x))) = (y))

uint32 mp_IOAPIC_addr = IOAPIC_ADDR_DEFAULT;
mp_IOAPIC_info mp_IOAPICs[MAX_IOAPICS];
uint32 mp_num_IOAPICs = 0;
#define MP_IOAPIC_READ(x)   (*((volatile uint32 *) (mp_IOAPIC_addr+(x))))
#define MP_IOAPIC_WRITE(x,y) (*((volatile uint32 *) (mp_IOAPIC_addr+(x))) = (y))

/* ************************************************** */
/* Local APIC programming */

/* Send Interprocessor Interrupt -- prods the Local APIC to deliver an
 * interprocessor interrupt, 'v' specifies the vector but also
 * specifies flags according to the Intel System Programming Manual --
 * also see apic.h and the LAPIC_ICR_* constants. */
int
LAPIC_send_ipi (uint32 dest, uint32 v)
{
  int timeout, send_status;

  /* It is a bad idea to have interrupts enabled while twiddling the
   * LAPIC. */
  asm volatile ("pushfl");
  asm volatile ("cli");

  MP_LAPIC_WRITE (LAPIC_ICR + 0x10, dest << 24);
  MP_LAPIC_WRITE (LAPIC_ICR, v);

  /* Give it a thousand iterations to see if the interrupt was
   * successfully delivered. */
  timeout = 0;
  do {
    send_status = MP_LAPIC_READ (LAPIC_ICR) & LAPIC_ICR_STATUS_PEND;
  } while (send_status && (timeout++ < 1000));

  asm volatile ("popfl");

  return (timeout < 1000);
}

void
LAPIC_init(void) 
{
  MP_LAPIC_WRITE (LAPIC_TPR, 0x00);      /* task priority = 0x0 */
  MP_LAPIC_WRITE (LAPIC_LVTT, 0x10000);  /* disable timer int */
  MP_LAPIC_WRITE (LAPIC_LVTPC, 0x10000); /* disable perf ctr int */
  MP_LAPIC_WRITE (LAPIC_LVT0, 0x08700); /* enable normal external ints */
  MP_LAPIC_WRITE (LAPIC_LVT1, 0x00400); /* enable NMI */
  MP_LAPIC_WRITE (LAPIC_LVTE, 0x10000); /* disable error ints */
  MP_LAPIC_WRITE (LAPIC_SPIV, 0x0010F); /* enable APIC: spurious vector = 0xF */

  /* be sure: */
  MP_LAPIC_WRITE (LAPIC_LVT1, 0x00400); /* enable NMI */
  MP_LAPIC_WRITE (LAPIC_LVTE, 0x10000); /* disable error ints */
}

void 
LAPIC_set_task_priority(uint8 prio)
{
  MP_LAPIC_WRITE (LAPIC_TPR, prio);
}

uint32
LAPIC_clear_error(void) 
{
  MP_LAPIC_WRITE (LAPIC_ESR, 0);
  return MP_LAPIC_READ (LAPIC_ESR);
}

uint8
LAPIC_get_physical_ID (void)
{
  if (mp_ISA_PC)
    return 0;
  else
    return (MP_LAPIC_READ (LAPIC_ID) >> 0x18) & 0xF;
}

void
send_eoi (void)
{
  /*
   * TODO: send_eoi should be separated. On processors that do not have IOAPIC
   * but have Local APIC, we need EOI for both 8259A (I/O) and LAPIC (e.g. Local
   * APIC Timer). This happens on Quark in Galileo.
   */
  if (mp_apic_mode) {
    MP_LAPIC_WRITE (LAPIC_EOI, 0);      /* send to LAPIC */
  }
  if (!ioapic_exists) {
    outb (0x20, 0x20);          /* send to 8259A PIC */
  }
}

void
LAPIC_start_timer (uint32 count)
{
  MP_LAPIC_WRITE (LAPIC_TICR, count);
}

void
LAPIC_set_logical_destination(uint32 log_dest) {
  MP_LAPIC_WRITE (LAPIC_LDR, log_dest); /* write to logical destination reg */
  MP_LAPIC_WRITE (LAPIC_DFR, -1);       /* use 'flat model' destination format */
}

void
LAPIC_enable_timer(uint8 vec, bool periodic, uint8 divisor) 
{
  uint8 div_flag = 0xB;         /* default: div-by-1 */

  switch(divisor) {
  case 1: div_flag = 0xB; break;
  case 2: div_flag = 0x0; break;
  case 4: div_flag = 0x1; break;
  case 8: div_flag = 0x2; break;
  case 16: div_flag = 0x3; break;
  case 32: div_flag = 0x8; break;
  case 64: div_flag = 0x9; break;
  case 128: div_flag = 0xA; break;
  default: break;
  }

  if (periodic)
    MP_LAPIC_WRITE (LAPIC_LVTT, (1<<17) | (uint32)vec); 
  else
    MP_LAPIC_WRITE (LAPIC_LVTT, (uint32)vec); 
  MP_LAPIC_WRITE (LAPIC_TDCR, div_flag); 
}

void
LAPIC_disable_timer(void)
{
  MP_LAPIC_WRITE (LAPIC_LVTT, (1<<16));
}


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

extern volatile uint32 tick;    /* defined in interrupt_handler.c */
static void
smp_setup_LAPIC_timer_int (void)
{
  /* Temporary handler for the PIT-generated timer IRQ */
  tick++;

  outb (0x20, 0x20);            /* EOI -- still using PIC */

  /* Cheat a little here: the C compiler will insert 
   *   leave
   *   ret
   * but we want to fix the stack up and IRET so I just
   * stick that in right here. */
  asm volatile ("leave");
  asm volatile ("iret");
}

static void
smp_LAPIC_timer_irq_handler (void)
{
  /* Temporary handler for the LAPIC-generated timer interrupt */
  /* just EOI and ignore it */
  MP_LAPIC_WRITE (LAPIC_EOI, 0);        /* send to LAPIC -- this int came from LAPIC */
  asm volatile ("leave");
  asm volatile ("iret");
}

/* CPU BUS FREQUENCY -- IN HERTZ */
uint32 cpu_bus_freq = 0;
uint64 tsc_freq = 0;

/* Use the PIT to find how fast the LAPIC ticks (and correspondingly,
 * the bus speed of the processor) */
void
LAPIC_measure_timer (void)
{
  idt_descriptor old_timer, old_3f;
  uint32 value, start, count;
  uint32 tsc_start_hi, tsc_start_lo, tsc_value_hi, tsc_value_lo;

  /* I want to enable interrupts but I don't want the normal
   * interrupt-handling architecture to kick-in just yet.  Here's an
   * idea: mask off every single entry in the IDT! */
  /* --YL-- Not a good idea. We had a spurious interrupt on Galileo and it hung the system */
  /* TODO: We should register some dummy handlers here and handle spurious interrupt */
  /* disable_idt (); */

  /* Now save the handlers for vectors 0x20 and 0x3F */
  get_idt_descriptor (0x20, &old_timer);
  get_idt_descriptor (0x3f, &old_3f);
  /* And use them for our temporary handlers */
  set_idt_descriptor_by_addr (0x20, (void *) &smp_setup_LAPIC_timer_int, 0x3);
  set_idt_descriptor_by_addr (0x3f, (void *) &smp_LAPIC_timer_irq_handler, 0x3);

  MP_LAPIC_WRITE (LAPIC_LVTT, 0x3f);    /* enable LAPIC timer int: vector=0x3f */
  MP_LAPIC_WRITE (LAPIC_TDCR, 0x0B);    /* set LAPIC timer divisor to 1 */

  /* Timing code: */
  value = tick;
  asm volatile ("sti");
  /* Wait until the PIT fires: */
  while (tick == value) {
    asm volatile ("pause");
  }
  start = tick;
  MP_LAPIC_WRITE (LAPIC_TICR, 0xFFFFFFFF);      /* write large value to Initial Count Reg. */
  asm volatile ("rdtsc":"=a" (tsc_start_lo), "=d" (tsc_start_hi));      /* store timestamp */
  /* LAPIC begins counting down, wait until the PIT fires again: */
  while (tick == start)
    asm volatile ("pause");
  asm volatile ("cli");
  asm volatile ("rdtsc":"=a" (tsc_value_lo), "=d" (tsc_value_hi));      /* store timestamp */
  MP_LAPIC_WRITE (LAPIC_LVTT, 0x10000); /* disable timer int */
  count = MP_LAPIC_READ (LAPIC_TCCR);   /* read the remaining count */

  /* Restore the original handlers: */
  set_idt_descriptor (0x20, &old_timer);
  set_idt_descriptor (0x3f, &old_3f);

  cpu_bus_freq = (0xFFFFFFFF - count) * HZ;
  com1_printf ("CPU bus frequency = 0x%X\n", cpu_bus_freq);
  tsc_freq = (((uint64) tsc_value_hi) << 32) | ((uint64) tsc_value_lo);
  tsc_freq -= ((uint64) tsc_start_hi) << 32;
  tsc_freq -= (uint64) tsc_start_lo;
  tsc_freq *= HZ;
  com1_printf ("TSC frequency = 0x%X 0x%X\n", (uint32) (tsc_freq >> 32),
               (uint32) tsc_freq);

  /* Put the IDT back in shape */
  enable_idt ();
}

/* End CPU Bus Frequency measurement code */

/* ************************************************** */
/* IO-APIC programming */

void
IOAPIC_init (void)
{
  int i, j;

  outb (0xFF, 0x21);            /* Mask interrupts in Master/Slave 8259A PIC */
  outb (0xFF, 0xA1);

  /* To get to a consistent state, first disable all IO-APIC
   * redirection entries. */

  for (i = 0; i < mp_num_IOAPICs; i++) {
    for (j = 0; j < mp_IOAPICs[i].numGSIs; j++) {
      IOAPIC_write64 (IOAPIC_REDIR + (j * 2), 0x0000000000010000LL);
    }
  }

  /* Map timer IRQ to vector 0x20 */
  IOAPIC_map_GSI (IRQ_to_GSI (mp_ISA_bus_id, 0), 0x20, 0xFF00000000000800LL);

#ifndef NO_IMCR_REDIRECT
  outb (0x70, 0x22);            /* Re-direct IMCR to use IO-APIC */
  outb (0x01, 0x23);            /* (for some motherboards) */
#endif

}

uint64
IOAPIC_read64 (uint8 reg)
{
  uint32 high, low;
  uint64 retval;
  MP_IOAPIC_WRITE (IOAPIC_REGSEL, reg + 1);
  high = MP_IOAPIC_READ (IOAPIC_RW);
  MP_IOAPIC_WRITE (IOAPIC_REGSEL, reg);
  low = MP_IOAPIC_READ (IOAPIC_RW);
  retval = (uint64) high << 32;
  retval |= (uint64) low;
  return retval;
}

void
IOAPIC_write64 (uint8 reg, uint64 v)
{
  /* First, disable the entry by setting the mask bit */
  MP_IOAPIC_WRITE (IOAPIC_REGSEL, reg);
  MP_IOAPIC_WRITE (IOAPIC_RW, 0x10000);
  /* Write to the upper half */
  MP_IOAPIC_WRITE (IOAPIC_REGSEL, reg + 1);
  MP_IOAPIC_WRITE (IOAPIC_RW, (uint32) (v >> 32));
  /* Write to the lower half */
  MP_IOAPIC_WRITE (IOAPIC_REGSEL, reg);
  MP_IOAPIC_WRITE (IOAPIC_RW, (uint32) (v & 0xFFFFFFFF));
}

uint32
IOAPIC_num_entries(void)
{
  MP_IOAPIC_WRITE (IOAPIC_REGSEL, 0x1);
  return APIC_MAXREDIR (MP_IOAPIC_READ (IOAPIC_RW)) + 1;
}

mp_IOAPIC_info *
IOAPIC_lookup (uint8 id)
{
  int i;
  for (i = 0; i < mp_num_IOAPICs; i++) {
    if (mp_IOAPICs[i].id == id)
      return &mp_IOAPICs[i];
  }
  panic ("IOAPIC_lookup failed.");
}

int
IOAPIC_map_GSI (uint32 GSI, uint8 vector, uint64 flags)
{
  int i;
  uint32 old_addr = 0;

  if (mp_ISA_PC)
    return -1;

  /* First, figure out which IOAPIC */
  for (i = 0; i < mp_num_IOAPICs; i++) {
    if (mp_IOAPICs[i].startGSI <= GSI &&
        GSI < mp_IOAPICs[i].startGSI + mp_IOAPICs[i].numGSIs) {
      old_addr = mp_IOAPIC_addr;
      /* set address for macros to use: */
      mp_IOAPIC_addr = mp_IOAPICs[i].address;
      /* set GSI relative to startGSI: */
      GSI -= mp_IOAPICs[i].startGSI;
      break;
    }
  }
  if (!old_addr)
    return -1;                  /* not found */

  IOAPIC_write64 (IOAPIC_REDIR + (GSI * 2), flags | vector);

  /* clean up */
  mp_IOAPIC_addr = old_addr;
  return GSI;
}

int
IOAPIC_get_GSI_mapping (uint32 GSI, uint8 *vector, uint64 *flags)
{
  int i;
  uint32 old_addr = 0;

  if (mp_ISA_PC)
    return -1;

  /* First, figure out which IOAPIC */
  for (i = 0; i < mp_num_IOAPICs; i++) {
    if (mp_IOAPICs[i].startGSI <= GSI &&
        GSI < mp_IOAPICs[i].startGSI + mp_IOAPICs[i].numGSIs) {
      old_addr = mp_IOAPIC_addr;
      /* set address for macros to use: */
      mp_IOAPIC_addr = mp_IOAPICs[i].address;
      /* set GSI relative to startGSI: */
      GSI -= mp_IOAPICs[i].startGSI;
      break;
    }
  }
  if (!old_addr)
    return -1;                  /* not found */

  u64 entry = IOAPIC_read64 (IOAPIC_REDIR + (GSI * 2));

  if (flags)
    *flags = entry & (~0xFFULL);
  if (vector)
    *vector = (u8) (entry & (0xFFULL));

  /* clean up */
  mp_IOAPIC_addr = old_addr;
  return GSI;
}

void
IOAPIC_dump_entries (void)
{
  int i, j;
  u32 old_addr = mp_IOAPIC_addr;
  for (i=0;i<mp_num_IOAPICs;i++) {
    mp_IOAPIC_addr = mp_IOAPICs[i].address;
    for (j=0; j<mp_IOAPICs[i].numGSIs; j++) {
      logger_printf ("IOAPIC[%d][%d]=0x%llX\n",
                     i, j, IOAPIC_read64 (IOAPIC_REDIR + (j * 2)));
    }
  }
  mp_IOAPIC_addr = old_addr;
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
