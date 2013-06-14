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
#include "arch/i386-percpu.h"
#include "kernel.h"
#include "mem/mem.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "smp/spinlock.h"
#include "drivers/acpi/acpi.h"
#include "drivers/acpi/acmacros.h"
#include "drivers/acpi/acexcep.h"
#include "util/printf.h"
#include "util/perfmon.h"
#include "sched/sched.h"

#ifdef USE_VMX
#include "vm/ept.h"
#include "vm/shm.h"
#include "vm/vmx.h"
#include "sched/vcpu.h"

#ifdef USE_LINUX_SANDBOX
#include "vm/linux_boot.h"
#endif

#endif

//#define DEBUG_SMP 1

volatile bool mp_enabled = 0;
uint32 mp_num_cpus = 1;
bool mp_apic_mode = 0;
bool mp_ISA_PC = 0;

mp_int_override mp_overrides[MAX_INT_OVERRIDES];
uint32 mp_num_overrides = 0;

uint32
IRQ_to_GSI (uint32 src_bus, uint32 src_irq)
{
  /* This is for ISA only */
  int i;
  for (i = 0; i < mp_num_overrides; i++) {
    if (src_bus == mp_overrides[i].src_bus &&
        src_irq == mp_overrides[i].src_IRQ)
      return mp_overrides[i].dest_GSI;
  }
  if (src_bus != mp_ISA_bus_id)
    panic ("IRQ_to_GSI only implemented for ISA bus.");
  /* assume identity-mapped if not overriden */
  return src_irq;
}


/* Mapping from CPU # to APIC ID */
uint8 CPU_to_APIC[MAX_CPUS];
uint8 APIC_to_CPU[MAX_CPUS];

bool mp_ACPI_enabled = 0;

/* I hard-code a check for re-routing of the timer IRQ, but this
 * should probably be folded into a more general interrupt routing
 * system. */
uint32 mp_timer_IOAPIC_irq = 0;
uint32 mp_timer_IOAPIC_id = 0;
uint32 mp_ISA_bus_id = 0;


/* ************************************************** */
/* General initialization for SMP */

/* Returns number of CPUs successfully booted. */
int
smp_init (void)
{
  uint32 phys_id, log_dest;
  uint32 intel_mps_init(bool);
  uint32 acpi_early_init(void);
  void LAPIC_measure_timer(void);
  void LAPIC_init(void);

  mp_apic_mode = 1;

  LAPIC_init();
  
  phys_id = get_pcpu_id();

  /* setup a logical destination address */
  log_dest = 0x01000000 << phys_id;
  LAPIC_set_logical_destination(log_dest);

  /* Find out how fast the LAPIC can tick -- and correspondingly the
   * CPU bus frequency.  Also finds the RDTSC frequency. */
  LAPIC_measure_timer ();


#ifdef NO_ACPI
  if (0);
#else
  if ((acpi_early_init()) > 0) {
    /* ACPI succeeded */
    mp_ACPI_enabled = 1;
  }
#endif

#ifdef NO_INTEL_MPS
  else if (0);
#else     
  else if ((intel_mps_init(FALSE)) > 0) {
    /* Intel MPS succeeded */
  }
#endif

  else {
    /* unable to detect any kind of SMP configuration */
    mp_apic_mode = 0;
    com1_printf ("Disabling APIC mode -- assuming ISA bus-only\n");
    mp_ISA_PC = 1;            /* assume no PCI */
    mp_num_cpus = 1;
    return 1;                 /* assume uniprocessor */
  }

  return mp_num_cpus;
}

void
smp_secondary_init (void)
{
  void acpi_secondary_init(void);
  void IOAPIC_init(void);

  if (!mp_ISA_PC)               /* ISA PCs do not have IO APICs */
    IOAPIC_init();

  if(mp_ACPI_enabled)
    acpi_secondary_init();

  /* The global variable mp_enabled will be incremented in the PIT IRQ
   * handler, this permits the Application Processors to go ahead and
   * complete initialization after the kernel has entered a
   * multi-processing safe state. */
}

void
smp_enable_scheduling (void)
{
  LAPIC_enable_timer(0x3e,   /* enable LAPIC timer int: vector=0x3e */
                     FALSE,  /* one-shot mode. */
                     1);     /* set LAPIC timer divisor to 1 */

  LAPIC_start_timer(cpu_bus_freq / QUANTUM_HZ); /* quantum */

  sched_enabled = 1;
}

/* ************************************************** */

/* General SMP initialization functions */


/* A number of symbols defined in the boot-smp.S file: */
extern uint8 patch_code_start[];        /* patch_code is what the AP boots */
extern uint8 patch_code_end[];
extern uint8 status_code[];     /* the AP writes a 1 into here when it's ready */
extern uint8 ap_stack_ptr[];    /* we give the AP a stack pointer through this */

/* For some reason, if this function is 'static', and -O is on, then
 * qemu fails. */
uint32
smp_boot_cpu (uint8 apic_id, uint8 APIC_version)
{
  int success = 1;
  volatile int to;
  uint32 bootaddr;

  /* Get a page for the AP's C stack */
  uint32 page_frame = (uint32) alloc_phys_frame ();
  uint32 *virt_addr = map_virtual_page (page_frame | 3);

  /* Set up the boot code for the APs */
#define TEST_BOOTED(x) (*((volatile uint32 *)(x+status_code-patch_code_start)))
#define STACK_PTR(x)   (*((volatile uint32 *)(x+ap_stack_ptr-patch_code_start)))

  /* The patch code is memcpyed into the hardcoded address MP_BOOTADDR */
  bootaddr = MP_BOOTADDR;       /* identity mapped */
  memcpy ((uint8 *) bootaddr, patch_code_start,
          patch_code_end - patch_code_start);
  /* The status code is reset to 0 */
  TEST_BOOTED (bootaddr) = 0;
  /* A temporary stack is allocated for the AP to be able to call C code */
  STACK_PTR (bootaddr) = (uint32) virt_addr;
  /* (FIXME: when does this get de-allocated?) */

  /* CPU startup sequence: officially it is supposed to proceed:
   * ASSERT INIT IPI, DE-ASSERT INIT IPI, STARTUP IPI, STARTUP IPI */

  /* clear APIC error register */
  LAPIC_clear_error();

  /* assert INIT interprocessor interrupt */
  LAPIC_send_ipi (apic_id,
                  LAPIC_ICR_TM_LEVEL | LAPIC_ICR_LEVELASSERT | LAPIC_ICR_DM_INIT);

  /* de-assert INIT IPI */
  LAPIC_send_ipi (apic_id, LAPIC_ICR_TM_LEVEL | LAPIC_ICR_DM_INIT);

  tsc_delay_usec (10000);       /* wait 10 millisec */

  /* Send start-up IPIs if not old version */
  if (APIC_version >= APIC_VER_NEW) {
    int i;
    for (i = 1; i <= 2; i++) {
      /* Bochs starts it @ INIT-IPI and the AP goes into p-mode which is
       * bad if I then deliver a STARTUP-IPI because that loads the CS
       * register with MP_BOOTADDR>>4 which is of course invalid in p-mode.  So
       * I added the test. */
      if (TEST_BOOTED (bootaddr))
        break;
      LAPIC_send_ipi (apic_id, LAPIC_ICR_DM_SIPI | ((bootaddr >> 12) & 0xFF));
      tsc_delay_usec (200);     /* wait 200 microsec */
    }
  }
#define LOOPS_TO_WAIT 10000000
  /* Check for successful start */
  to = 0;
  while (!TEST_BOOTED (bootaddr) && to++ < LOOPS_TO_WAIT) {
    tsc_delay_usec (200);       /* wait 200 microsec */
  }
  if (to >= LOOPS_TO_WAIT) {
    success = 0;
  }

  /* cleanup */
  LAPIC_clear_error();

  return success;
}


/* ************************************************** */

/* When the AP boots it will first start at the patch code in
 * boot-smp.S.  That code prepares the AP for the jump into C code,
 * namely, this function: */
void
ap_init (void)
{
  int phys_id, log_dest;
  void LAPIC_init(void);

  /* Setup the LAPIC */
  LAPIC_init();
#ifdef USE_VMX
  /* In order to broadcast interrupts */
  LAPIC_set_task_priority(0x00); /* task priority = 0x00 */
#else
  LAPIC_set_task_priority(0x20); /* task priority = 0x20 */
#endif

  phys_id = get_pcpu_id ();

  /* setup a logical destination address */
  log_dest = 0x01000000 << phys_id;
  LAPIC_set_logical_destination(log_dest);

  asm volatile ("lidt idt_ptr");        /* Set the IDT */

  /* Spin-wait for all processors to come online, and the system to
   * enter MP mode. */
  while (!mp_enabled)
    asm volatile ("pause");

  LAPIC_enable_timer(0x3e, FALSE, 1);    /* vector=0x3e, one-shot, divisor=1 */
  LAPIC_start_timer(cpu_bus_freq / QUANTUM_HZ); /* quantum */
  
  /* The AP is now operating in an SMP environment so the kernel must
   * be locked before any shared resources are utilized. */
  lock_kernel ();

  /* Performance monitoring */
  //perfmon_init ();

  /* Initialise the floating-point unit (FPU) */
  initialise_fpu();
  
  /* Load the per-CPU TSS for this AP */
  hw_ltr (cpuTSS_selector[phys_id]);

#ifdef USE_VMX
#ifdef QUESTV_NO_VMX
  /* VMX not enabled. Start VM fork without VM initilization. */
  { void vmx_init_mem (uint32); vmx_init_mem (phys_id); }
#else
  /* Initialize virtual machine per-processor infrastructure */
  { void vmx_processor_init (void); vmx_processor_init (); }
#endif
  spinlock_lock (&(shm->global_lock));

  while (!shm->bsp_booted)
    asm volatile ("pause");
  
  { extern bool vcpu_init (void); vcpu_init (); }
  { extern void net_init (void); net_init (); }
  //if (phys_id == 1) {
  //{ extern bool r8169_register (void); r8169_register (); }
  //{ extern bool netsetup_init (void); netsetup_init (); }
  //{ extern bool vfs_init (void); vfs_init (); }
  //}
  { extern bool migration_init (void); migration_init (); }
  { extern bool logger_init (void); logger_init (); }
#ifdef USE_LINUX_SANDBOX
  if (phys_id == LINUX_SANDBOX)
  { extern bool linux_boot_thread_init (void); linux_boot_thread_init (); }
#endif
  //if (phys_id == 1)
  //{ extern bool hog_thread_init (void); hog_thread_init (); }
  //{ extern bool msgt_bandwidth_init (void); msgt_bandwidth_init (); }
  //{ extern bool udp_bandwidth_init (void); udp_bandwidth_init (); }
  //if (phys_id == 1)
  //{ extern bool ipc_recv_init (void); ipc_recv_init (); }
  //{ extern bool msgt_init (void); msgt_init (); }
  //if (phys_id == 3)
  //{ extern bool stat_thread_init (void); stat_thread_init (); }

  /*
   * For SeQuest, each sandbox kernel will have a shell running
   * using the per-sandbox virtual screen buffer as output.
   */
  /* A hack! Fix the shell tss for sandboxes. */
  //print ("Sandbox ");
  //putx (phys_id);
  //print (" Loading Shell...\n");
  extern task_id shell_tss;
  int mod_num = NR_MODS - 1;
  quest_tss * usr_mod = lookup_TSS (shell_tss);
  //quest_tss * usr_mod = (quest_tss *) ul_tss[mod_num];
  uint32 * plPageDirectory = get_phys_addr (pg_dir[mod_num]);
  uint32 * plPageTable = get_phys_addr (pg_table[mod_num]);
  uint32 * plStackPageTable = get_phys_addr(uls_pg_table[mod_num]);
  //uint32* page_directory_virt = pg_dir[mod_num];
  //uint32* stack_page_table;
  int stack_dir_index, stack_tbl_index;
  void * pStack = get_phys_addr (ul_stack[mod_num]);
  int i = 0;
  uint32 * vpdbr = map_virtual_page ((uint32) get_pdbr () | 3);

  /* Populate ring 3 page directory with kernel mappings */
  memcpy (&pg_dir[mod_num][1023], (void *) (((uint32) vpdbr) + 4092), 4);
  /* LAPIC/IOAPIC mappings */
  memcpy (&pg_dir[mod_num][1019], (void *) (((uint32) vpdbr) + 4076), 4);

  unmap_virtual_page (vpdbr);

  get_pg_dir_and_table_indices ((void *) (USER_STACK_START - 1), &stack_dir_index,
                                &stack_tbl_index);

  /* Populate ring 3 page directory with entries for its private address
     space */
  pg_dir[mod_num][0] = (uint32) plPageTable | 7;
  if(stack_dir_index != 0) {
    pg_dir[mod_num][stack_dir_index] = (uint32) plStackPageTable | 7;
  }

  pg_dir[mod_num][1022] = (uint32) get_phys_addr (kls_pg_table[mod_num]) | 3;
  kls_pg_table[mod_num][0] = (uint32) get_phys_addr (kl_stack[mod_num]) | 3;

  for (i = 0; i < 1024; i++) {
    if (pg_table[mod_num][i]) {
      if (((pg_table[mod_num][i] & 0xFFFFF000) >> 12) <
           ((SANDBOX_KERN_OFFSET >> 12) + 256 /* BIOS */)) {
          pg_table[mod_num][i] =
              ((pg_table[mod_num][i] & 0xFFFFF000) + SANDBOX_KERN_OFFSET * phys_id) |
              (pg_table[mod_num][i] & 0xF);
      }
    }
  }
  map_malloc_paging_structures(pg_dir[mod_num], SANDBOX_KERN_OFFSET * phys_id);
  map_dma_page_tables(pg_dir[mod_num], SANDBOX_KERN_OFFSET * phys_id);

  
  /* Shift stack page */
  if(stack_dir_index != 0) {
    uls_pg_table[mod_num][stack_tbl_index] = (uint32) pStack | 7;
  }
  else {
    pg_table[mod_num][stack_tbl_index] = (uint32) pStack | 7;
  }

  usr_mod->CR3 = (uint32) plPageDirectory;
  usr_mod->ESP = USER_STACK_START - 100;
  usr_mod->EBP = USER_STACK_START - 100;
  usr_mod->sandbox_affinity = phys_id;

  ltr (shell_tss);
  check_copied_threads ();
  /* We don't switch to idle task initially. So, unlock the kernel. */
  unlock_kernel ();
  spinlock_unlock (&(shm->global_lock));

  /* task-switch to shell module */
  asm volatile ("jmp _sw_init_user_task"::"D" (usr_mod));
#else
  /* The IDLE task runs in kernelspace, therefore it is capable of
   * unlocking the kernel and manually enabling interrupts.  This
   * makes it safe to use lock_kernel() above.  */

  /* If the IDLE task did not run in kernelspace, then we would need a
   * dummy TSS per-processor because it would not be possible to
   * protect the dummy TSS from simultaneous usage by multiple CPUs in
   * this case. */

  ltr (idleTSS_selector[phys_id]);

  /* task-switch to IDLE task */
  asm volatile ("jmp _sw_init_task":
                :"D" (lookup_TSS (idleTSS_selector[phys_id])));
#endif

  /* never return */

  panic ("AP: unreachable");
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
