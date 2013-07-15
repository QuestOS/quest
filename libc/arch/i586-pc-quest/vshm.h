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

#include <stdlib.h>

#define VSHM_NO_ACCESS        0x0
#define VSHM_READ_ACCESS      0x1
#define VSHM_WRITE_ACCESS     0x2
#define VSHM_EXEC_ACCESS      0x4
#define VSHM_ALL_ACCESS       0x7

#define VSHM_CREATE           0x80000000

/* Simpson's four slot asynchronous communication mechanism */
typedef struct {
  unsigned int latest, reading;
  unsigned int slot[2];
  char* data[];
} four_slot_shared_t;

typedef struct {
  size_t element_size;
} four_slot_private_t;

typedef struct {
  four_slot_shared_t* shared;
  four_slot_private_t private;
} vshm_async_channel_t;

int mk_vshm_async_channel(vshm_async_channel_t* vac, unsigned int vshm_key,
                          size_t element_size, unsigned int sandboxes, unsigned int flags);

void vshm_async_channel_write(vshm_async_channel_t* vac, void* item);

void vshm_async_channel_read(vshm_async_channel_t* vac, void* item);

typedef struct
{
  unsigned int start;
  unsigned int end;
  char* data[];
} vshm_circular_buffer_shared_t;

typedef struct
{
  size_t buffer_size;
  size_t element_size;
} vshm_circular_buffer_private_t;

typedef struct {
  vshm_circular_buffer_shared_t* shared;
  vshm_circular_buffer_private_t private;
} vshm_circular_buffer_t;

int mk_vshm_circular_buffer(vshm_circular_buffer_t* vcb, unsigned int vshm_key,
                            size_t buffer_size, size_t element_size, unsigned int sandboxes,
                            unsigned int flags);

int vshm_circular_buffer_insert(vshm_circular_buffer_t* vcb, void* item);

int vshm_circular_buffer_remove(vshm_circular_buffer_t* vcb, void* item);

inline int vshm_map(unsigned int vshm_key, unsigned int size,
                    unsigned int sandboxes, unsigned int flags, void** addr);


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
