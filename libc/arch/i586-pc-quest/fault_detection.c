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

#include <fault_detection.h>

int fault_detection_register_program(uint key, uint arbitrator_sandbox)
{
  syscall_fault_detection((unsigned int)FDA_REGISTER_PROG, key, arbitrator_sandbox);
}

int fault_detection_sync()
{
  syscall_fault_detection((unsigned int)FDA_SYNC, 0, 0);
}

int fault_detection_register_arbitrator(uint key, uint arbitrated_sandbox,
                                        fault_detection_prog_t* fdp)
{
  
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
