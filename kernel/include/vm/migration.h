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

#ifndef _MIGRATION_H_
#define _MIGRATION_H_

#include <types.h>
#include "sched/proc.h"
#include "mem/virtual.h"

/* We will use some fixed vectors for communications using IPI */
/* Let's start from 240 which is closer to the bottom of the vector table */
#define MIGRATION_RECV_REQ_VECTOR    240
#define MIGRATION_CLEANUP_VECTOR     241

/* The low level infrastructure for process migration includes at least the
 * following 4 functions. An overview of the call graph is shown below. Some
 * inter-sandbox communication mechanism must be implemented to support the
 * asynchronous event notification depicted as horizontal arrows. IPIs will
 * be used for all these events in the current design.
 *
 *  Local Sandbox                  Remote Sandbox
 *       |                               |
 *       V                               |
 *   detach_task                         |
 *       |            IPI                V
 *       +-----------------------> pull_quest_tss
 *                                       |
 *                                       V
 *                           remote_clone_page_directory
 *                    IPI                |
 *       +-------------------------------+
 *       |
 *       V
 *  destroy_task
 *
 * The arguments of these interfaces usually take physical address because
 * different sandboxes have different address spaces.
 */

/* detach_task remove task tid from calling sandbox's local scheduler.
 * The physical address of the corresponding quest_tss will be returned.
 * This physical address will be passed to remote sandox for further
 * migration process.
 */
extern void * detach_task (task_id tid);

/* pull_quest_tss is called by the remote sandbox to create a new tss
 * and set it up for the migrating process.
 * The physical address of the migrating tss is specified with the address
 * of the tss created locally returned.
 */
extern quest_tss * pull_quest_tss (void * phy_tss);

/* remote_clone_page_directory clones the migrating process's address
 * space into the destination sandbox. It is called in the remote
 * sandbox.
 */
extern pgdir_t remote_clone_page_directory (pgdir_t dir);

/* destroy_task is called by the local sandbox to free the whole address
 * space of the migrating process and its other data structures (like
 * quest_tss etc.).
 * This is similar to an exit() system call.
 */
extern void destroy_task (void * phy_tss);

/* Send a migration request from current sandbox to sandbox. The actual
 * request will be retrieved through shared memory.
 * Returns 0 if request sending failed.
 */
extern int request_migration (int sandbox);

#endif

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
