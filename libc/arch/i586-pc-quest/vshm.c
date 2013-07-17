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

#include <vshm.h>
#include <string.h>

int mk_vshm_async_channel(vshm_async_channel_t* vac, unsigned int vshm_key,
                          size_t element_size, unsigned int sandboxes, unsigned int flags)
{
  int res = vshm_map(vshm_key, sizeof(four_slot_shared_t) + (element_size * 4), sandboxes, flags,
                     (void**)&vac->shared);
  if(res < 0) return res;
  vac->private.element_size = element_size;
  return 0;
}

#define vac_element_addr(vac, pair, index)      \
  &vac->shared->data[vac->private.element_size * (pair + 2 * index)]

void vshm_async_channel_write(vshm_async_channel_t* vac, void* item)
{
  unsigned int pair = !vac->shared->reading;
  unsigned int index = !vac->shared->slot[pair];
  memcpy(vac_element_addr(vac, pair, index), item, vac->private.element_size);
  vac->shared->slot[pair] = index;
  vac->shared->latest = pair;
}

void vshm_async_channel_read(vshm_async_channel_t* vac, void* item)
{
  unsigned int index;
  unsigned int pair = !! vac->shared->latest;
  vac->shared->reading = pair;
  index = !! vac->shared->slot[pair];
  memcpy(item, vac_element_addr(vac, pair, index), vac->private.element_size);
}

int mk_vshm_circular_buffer(vshm_circular_buffer_t* vcb, unsigned int vshm_key,
                            size_t buffer_size, size_t element_size, unsigned int sandboxes,
                            unsigned int flags)
{
  int res;
 /* increment buffer size by one since we loss an element to avoid
    having to use a lock and keeping track of the number of items*/
  buffer_size++;
  res = vshm_map(vshm_key, sizeof(vshm_circular_buffer_shared_t) + (element_size * buffer_size),
                     sandboxes, flags, (void**)&vcb->shared);
  if(res < 0) return res;
  vcb->private.buffer_size  = buffer_size;
  vcb->private.element_size = element_size;
  return 0;
}

#define vcb_element_addr(vcb, index) &vcb->shared->data[vcb->private.element_size * index]


int vshm_circular_buffer_insert(vshm_circular_buffer_t* vcb, void* item)
{
  unsigned int start = vcb->shared->start;
  unsigned int end   = vcb->shared->end;

  if(end >= vcb->private.buffer_size || start >= vcb->private.buffer_size) return -2;
  
  if(end > start) {
    if(end == (vcb->private.buffer_size-1) && start == 0) return -1;
  }
  else {
    if(end + 1 == start) return -1;
  }
  memcpy(vcb_element_addr(vcb, end), item, vcb->private.element_size);
  end++;
  if(end == vcb->private.buffer_size) {
    end = 0;
  }
  vcb->shared->end = end;
  return 0;
}

int vshm_circular_buffer_remove(vshm_circular_buffer_t* vcb, void* item)
{
  unsigned int start = vcb->shared->start;
  unsigned int end   = vcb->shared->end;

  if(end >= vcb->private.buffer_size || start >= vcb->private.buffer_size) return -2;
  
  if(start == end) {
    /* buffer is empty */
    return -1;
  }
  memcpy(item, vcb_element_addr(vcb, start), vcb->private.element_size);
  start++;
  if(start == vcb->private.buffer_size) {
    start = 0;
  }
  vcb->shared->start = start;
  return 0;
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
