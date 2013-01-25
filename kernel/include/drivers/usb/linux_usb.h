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

/*
 * URB support, for asynchronous request completions
 */

/*
 * urb->transfer_flags:
 *
 * Note: URB_DIR_IN/OUT is automatically set in usb_submit_urb().
 */
#define URB_SHORT_NOT_OK        0x0001  /* report short reads as errors */
#define URB_ISO_ASAP            0x0002  /* iso-only, urb->start_frame
                                         * ignored */
#define URB_NO_TRANSFER_DMA_MAP 0x0004  /* urb->transfer_dma valid on submit */
#define URB_NO_FSBR             0x0020  /* UHCI-specific */
#define URB_ZERO_PACKET         0x0040  /* Finish bulk OUT with short packet */
#define URB_NO_INTERRUPT        0x0080  /* HINT: no non-error interrupt
                                         * needed */
#define URB_FREE_BUFFER         0x0100  /* Free transfer buffer with the URB */
#define URB_RT_ONE_HANDLER_CALL 0x0200  /* Only need to call the completion handler once
					   per interrupt, not once for each frame/microframe
					   that passed */

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
   * Used for input isochronous real-time urbs. USB 1.1 devices it is
   * measured in frames for USB 2.0 devices it is measured in
   * micro-frames.
   */
  
  uint interrupt_frame_rate;

  /* Used for input non-isochronous real-time urbs.  Specified how
     many bytes should be ready before an interrupt occurs. */
  uint interrupt_byte_rate;

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


#define BitTime(bytecount) (7 * 8 * bytecount / 6) /* with integer truncation */
/* Trying not to use worst-case bit-stuffing
 * of (7/6 * 8 * bytecount) = 9.33 * bytecount */
/* bytecount = data payload byte count */
 
#define NS_TO_US(ns)    ((ns + 500L) / 1000L)
/* convert & round nanoseconds to microseconds */

#define USB2_HOST_DELAY 5       /* nsec, guess */
#define HS_NSECS(bytes) (((55 * 8 * 2083)				\
			  + (2083UL * (3 + BitTime(bytes))))/1000	\
			 + USB2_HOST_DELAY)
#define HS_NSECS_ISO(bytes) (((38 * 8 * 2083)				\
			      + (2083UL * (3 + BitTime(bytes))))/1000	\
			     + USB2_HOST_DELAY)
#define HS_USECS(bytes)         NS_TO_US(HS_NSECS(bytes))
#define HS_USECS_ISO(bytes)     NS_TO_US(HS_NSECS_ISO(bytes))


#endif // _LINUX_USB_H_
