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

#ifndef _FAULT_DETECTION_H_
#define _FAULT_DETECTION_H_


#include <sys/types.h>


/* The following is duplicated in include/vm/fault_detection.h and
   must be the same as it is in there */

typedef enum {
  FDA_REGISTER_PROG = 0,
  FDA_SYNC,
} FAULT_DETECTION_ACTION;

#define FAULT_DETECTION_POOL_SIZE (POOL_SIZE_IN_PAGES * 0x1000)

//#define FAULT_DETECTION_HASH_SIZE (128/32)
#define FAULT_DETECTION_HASH_SIZE (1)

typedef struct {
  uint virtual_page;
  uint hash[FAULT_DETECTION_HASH_SIZE];
} fault_detection_hash_t;

/* This must be the same as fault_detection_hash_dumps_t in libc's
   fault_detection.h */

typedef struct {
  uint count;
  uint num_hashes;
  uint checkpoint_passed;
  fault_detection_hash_t hashes[0];
} fault_detection_hash_dumps_t;

#define FAULT_DETECTION_HASH_COUNT_MAX                                      \
  ((FAULT_DETECTION_POOL_SIZE - sizeof(fault_detection_hash_dumps_t)) / sizeof(fault_detection_hash_t))

typedef struct {
  fault_detection_hash_dumps_t* hash_dumps;
} fault_detection_prog_t;

int fault_detection_register_arbitrator(uint key, uint arbitrated_sandbox,
                                        fault_detection_prog_t* fdp);

int syscall_fault_detection(uint action, uint key, uint arbitrator_sandbox);

int fault_detection_register_program(uint key, uint arbitrator_sandbox);

int fault_detection_sync();


#endif // _FAULT_DETECTION_H_


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
