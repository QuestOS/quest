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

#ifndef _USB_GADGET2_H_
#define _USB_GADGET2_H_

#define MAX_EPS 15

typedef struct
{
  unsigned int pipe;
  int transaction_size;
  struct urb* urb;
  int buffer_size;
  int next_to_read;
  int data_available;
  int num_packets;
  uint64_t total_bytes_read;
  uint64_t total_packets;
  uint64_t total_bytes_written;
  uint64_t start_time;
  uint64_t start_data_throughput;
} gadget2_sub_device_info_t;


typedef struct {
  bool initialised;
  unsigned int num_sub_devs;
  int first_dev_num;
  gadget2_sub_device_info_t sub_devs[MAX_EPS];
} gadget2_device_info_t;

#define get_gadget2_device_index(dev, dev_num) (dev_num - dev->first_dev_num)


#define get_gadget2_dev_info(dev) ((gadget2_device_info_t*) (dev)->device_priv)

#define get_gadget2_sub_dev_info(dev, dev_num)                           \
  (&(get_gadget2_dev_info(dev)->sub_devs[(dev_num) -                     \
                                        get_gadget2_dev_info(dev)->first_dev_num]))


#endif // _USB_GADGET2_H_

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
