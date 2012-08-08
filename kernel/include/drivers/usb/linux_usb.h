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

/*
 * This code is heavily based off of Linux's usb.h
 */

#ifndef _LINUX_USB_H_
#define _LINUX_USB_H_

#include "util/list.h"
#include "drivers/usb/usb.h"
#include <arch/i386.h>
#include "smp/spinlock.h"
#include <mem/pow2.h>

struct usb_iso_packet_descriptor {
  unsigned int offset;
  unsigned int length;            /* expected length */
  unsigned int actual_length;
  int status;
  volatile bool data_available;
};

typedef struct usb_iso_packet_descriptor usb_iso_packet_descriptor_t;

struct urb;

typedef void (*usb_complete_t)(struct urb *);

struct _usb_hcd_t;

struct urb {

  /*
   * -- EM -- removing the private data, will add it back as I need it
   */
  
  /* private: usb core and host controller only fields in the urb */
  //struct kref kref;               /* reference count of the URB */
  void *hcpriv;                   /* private data for host controller */
  //atomic_t use_count;             /* concurrent submissions counter */
  //atomic_t reject;                /* submissions will fail */
  //int unlinked;                   /* unlink error code */
  
  volatile bool active; /* Is set to TRUE is submit and set to FALSE
			 * when it is completed or killed */
  
  
  /* public: documented fields in the urb that can be used by drivers */
  struct list_head urb_list;      /* list head for use by the urb's
				   * current owner */
  
  /*
   * -- EM -- removing the anchored api stuff for now.  Don't feel
   * like dealing with it
   */
  //struct list_head anchor_list;   /* the URB may be anchored */
  //struct usb_anchor *anchor;

  
  struct usb_device *dev;         /* (in) pointer to associated device */

  /*
   * -- EM -- removing the struct usb_host_endpoint *ep and will just
   * use the unsigned int pipe for now
   */
  
  //struct usb_host_endpoint *ep;   /* (internal) pointer to endpoint */

  unsigned int pipe;              /* (in) pipe information */
  unsigned int stream_id;         /* (in) stream ID */
  int status;                     /* (return) non-ISO status */
  unsigned int transfer_flags;    /* (in) URB_SHORT_NOT_OK | ...*/
  void *transfer_buffer;          /* (in) associated data buffer */
  
  /* Originally dma_addr_t but we don not have that type */
  phys_addr_t transfer_dma;        /* (in) dma addr for transfer_buffer */

  /*
   * -- EM -- removing the scatterlist stuff for now.  Don't feel like
   * dealing with it
   */
  
  //struct scatterlist *sg;         /* (in) scatter gather buffer list */
  //int num_mapped_sgs;             /* (internal) mapped sg entries */
  //int num_sgs;                    /* (in) number of entries in the sg list */
  
  u32 transfer_buffer_length;     /* (in) data buffer length */
  u32 actual_length;              /* (return) actual transfer length */
  unsigned char *setup_packet;    /* (in) setup packet (control only) */
  
  /* Originally dma_addr_t but we don not have that type */
  phys_addr_t setup_dma;           /* (in) dma addr for setup_packet */

  int start_frame;                /* (modify) start frame (ISO) */
  int number_of_packets;          /* (in) number of ISO packets */
  int interval;                   /* (modify) transfer interval
				   * (INT/ISO) */
  int error_count;                /* (return) number of ISO errors */
  void *context;                  /* (in) context for completion */
  usb_complete_t complete;        /* (in) completion routine */


   /*
   * The Quest URB interface needs to pass the timeout to the USB
   * core/host controller drivers.  This is because we allow USB
   * transactions to occur when interrupts are disabled (specifically
   * within boot) and therefore the host controllers need to know how
   * long to spin until the transaction should be considered a
   * failure.
   */
  int timeout;

  /*
   * Used to mark urb as real-time, which means it will be handled
   * differently than non-real-time USB URBs (which are handled similar to
   * the way Linux handles USB URBS)
   */
  bool realtime;

  /*
   * Used for isochronous real-time urbs.  For USB 1.1 devices it is
   * measured in frames for USB 2.0 devices it is measured in
   * micro-frames
   */
  uint32_t realtime_complete_interval;
  uint32_t interrupt_interval;
  

  /*
   * iso_frame_desc has to be last because we use it is a variable
   * size array
   */
  
  struct usb_iso_packet_descriptor iso_frame_desc[0];
  /* (in) ISO ONLY */
};





#define usb_pipein(pipe)        (!!((pipe) & USB_DIR_IN))
#define usb_pipeout(pipe)       (!usb_pipein(pipe))

#define usb_pipedevice(pipe)    (((pipe) >> 8) & 0x7f)
#define usb_pipeendpoint(pipe)  (((pipe) >> 15) & 0xf)

#define usb_pipetype(pipe)      (((pipe) >> 30) & 3)
#define usb_pipeisoc(pipe)      (usb_pipetype((pipe)) == PIPE_ISOCHRONOUS)
#define usb_pipeint(pipe)       (usb_pipetype((pipe)) == PIPE_INTERRUPT)
#define usb_pipecontrol(pipe)   (usb_pipetype((pipe)) == PIPE_CONTROL)
#define usb_pipebulk(pipe)      (usb_pipetype((pipe)) == PIPE_BULK)

   
struct urb *usb_alloc_urb(int iso_packets, gfp_t mem_flags);

static inline void usb_free_urb(struct urb *urb)
{
  if(urb) pow2_free((uint8_t*)urb);
}

static inline void usb_init_urb(struct urb *urb) {
  memset(urb, 0, sizeof(*urb));
}

#endif // _LINUX_USB_H_
