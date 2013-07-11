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

#ifndef _VSHM_H_
#define _VSHM_H_

#define VSHM_NO_ACCESS        0x0
#define VSHM_READ_ACCESS      0x1
#define VSHM_WRITE_ACCESS     0x2
#define VSHM_EXEC_ACCESS      0x4
#define VSHM_ALL_ACCESS       0x7

#define VSHM_CREATE           0x80000000

inline int vshm_map(uint vshm_key, uint size, uint sandboxes, uint flags, void** addr);


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
