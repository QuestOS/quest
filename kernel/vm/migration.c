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
#include "vm/vmx.h"
#include "vm/ept.h"
#include "kernel.h"
#include "mem/physical.h"
#include "mem/virtual.h"
#include "util/printf.h"
#include "smp/apic.h"
#include "arch/i386.h"
#include "sched/sched.h"
#include "vm/shdr.h"
#include "vm/migration.h"

#define DEBUG_MIGRATION

#ifdef DEBUG_MIGRATION
#define DLOG(fmt,...) DLOG_PREFIX("Migration",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

void *
detach_task (task_id tid)
{
  return NULL;
}

quest_tss *
pull_quest_tss (void * phy_tss)
{
  return NULL;
}

pgdir_t
remote_clone_page_directory (pgdir_t dir)
{
  pgdir_t pgd;
  return dir;
}

void
destroy_task (void * phy_tss)
{
  return;
}

int
request_migration (int sandbox)
{
  int cpu = get_pcpu_id ();
  if (cpu == sandbox) {
    DLOG ("Ignored migration request to local sandbox %d", cpu);
    return 0;
  }
  return LAPIC_send_ipi (0x1 << sandbox,
                         LAPIC_ICR_LEVELASSERT
                         | LAPIC_ICR_DM_LOGICAL
                         | MIGRATION_RECV_REQ_VECTOR);
}

static uint32
receive_migration_request (uint8 vector)
{
  int cpu = get_pcpu_id ();
  DLOG ("Received migration request in sandbox kernel %d!", cpu);
  return 0;
}

/*
 * migration_init should be called by EACH sandbox kernel to setup
 * migration threads and register IPI handlers etc.
 */
bool
migration_init (void)
{
  int cpu = get_pcpu_id ();

  if (vector_used (MIGRATION_RECV_REQ_VECTOR)) {
    DLOG ("Interrupt vector %d has been used in sandbox %d. IPI handler registration failed!",
          MIGRATION_RECV_REQ_VECTOR, cpu);
  } else {
    set_vector_handler (MIGRATION_RECV_REQ_VECTOR, &receive_migration_request);
  }

  DLOG ("Migration subsystem initialized in sandbox kernel %d", cpu);

  return TRUE;
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
