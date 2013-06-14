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

#ifndef _UMSC_H_
#define _UMSC_H_

#include <types.h>

sint umsc_bulk_scsi (uint addr, uint ep_out, uint ep_in,
                     uint8 cmd[16], uint dir, uint8* data,
                     uint data_len, uint maxpkt);
sint umsc_read_sector (uint dev_index, uint32 lba, uint8 *sector, uint len);


int umsc_read_sectors (uint dev_index, uint32 lba, uint8 * buf, uint16 snum);

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
