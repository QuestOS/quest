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


/* The following code is modified from the Linux's gadget.h
 * below is the copyright from Linux
 *
 * (C) Copyright 2002-2004 by David Brownell
 * All Rights Reserved.
 *
 * This software is licensed under the GNU GPL version 2.
 */

#ifndef _GADGET_H_
#define _GADGET_H_

#include "drivers/usb/usb.h"

struct usb_ep;

/**
 * struct usb_request - describes one i/o request
 * @buf: Buffer used for data.  Always provide this; some controllers
 *      only use PIO, or don't use DMA for some endpoints.
 * @dma: DMA address corresponding to 'buf'.  If you don't set this
 *      field, and the usb controller needs one, it is responsible
 *      for mapping and unmapping the buffer.
 * @sg: a scatterlist for SG-capable controllers.
 * @num_sgs: number of SG entries
 * @num_mapped_sgs: number of SG entries mapped to DMA (internal)
 * @length: Length of that data
 * @stream_id: The stream id, when USB3.0 bulk streams are being used
 * @no_interrupt: If true, hints that no completion irq is needed.
 *      Helpful sometimes with deep request queues that are handled
 *      directly by DMA controllers.
 * @zero: If true, when writing data, makes the last packet be "short"
 *     by adding a zero length packet as needed;
 * @short_not_ok: When reading data, makes short packets be
 *     treated as errors (queue stops advancing till cleanup).
 * @complete: Function called when request completes, so this request and
 *      its buffer may be re-used.  The function will always be called with
 *      interrupts disabled, and it must not sleep.
 *      Reads terminate with a short packet, or when the buffer fills,
 *      whichever comes first.  When writes terminate, some data bytes
 *      will usually still be in flight (often in a hardware fifo).
 *      Errors (for reads or writes) stop the queue from advancing
 *      until the completion function returns, so that any transfers
 *      invalidated by the error may first be dequeued.
 * @context: For use by the completion callback
 * @list: For use by the gadget driver.
 * @status: Reports completion code, zero or a negative errno.
 *      Normally, faults block the transfer queue from advancing until
 *      the completion callback returns.
 *      Code "-ESHUTDOWN" indicates completion caused by device disconnect,
 *      or when the driver disabled the endpoint.
 * @actual: Reports bytes transferred to/from the buffer.  For reads (OUT
 *      transfers) this may be less than the requested length.  If the
 *      short_not_ok flag is set, short reads are treated as errors
 *      even when status otherwise indicates successful completion.
 *      Note that for writes (IN transfers) some data bytes may still
 *      reside in a device-side FIFO when the request is reported as
 *      complete.
 *
 * These are allocated/freed through the endpoint they're used with.  The
 * hardware's driver can add extra per-request data to the memory it returns,
 * which often avoids separate memory allocations (potential failures),
 * later when the request is queued.
 *
 * Request flags affect request handling, such as whether a zero length
 * packet is written (the "zero" flag), whether a short read should be
 * treated as an error (blocking request queue advance, the "short_not_ok"
 * flag), or hinting that an interrupt is not required (the "no_interrupt"
 * flag, for use with deep request queues).
 *
 * Bulk endpoints can use any size buffers, and can also be used for interrupt
 * transfers. interrupt-only endpoints can be much less functional.
 *
 * NOTE:  this is analogous to 'struct urb' on the host side, except that
 * it's thinner and promotes more pre-allocation.
 */

struct usb_request {
  void                    *buf;
  unsigned                length;
  dma_addr_t              dma;

  struct scatterlist      *sg;
  unsigned                num_sgs;
  unsigned                num_mapped_sgs;

  unsigned                stream_id:16;
  unsigned                no_interrupt:1;
  unsigned                zero:1;
  unsigned                short_not_ok:1;

  void                    (*complete)(struct usb_ep *ep,
                                      struct usb_request *req);
  void                    *context;
  struct list_head        list;

  int                     status;
  unsigned                actual;
};

/*-------------------------------------------------------------------------*/

/* endpoint-specific parts of the api to the usb controller hardware.
 * unlike the urb model, (de)multiplexing layers are not required.
 * (so this api could slash overhead if used on the host side...)
 *
 * note that device side usb controllers commonly differ in how many
 * endpoints they support, as well as their capabilities.
 */
struct usb_ep_ops {
  int (*enable) (struct usb_ep *ep,
                 const USB_EPT_DESC *desc);
  int (*disable) (struct usb_ep *ep);

  struct usb_request *(*alloc_request) (struct usb_ep *ep,
                                        gfp_t gfp_flags);
  void (*free_request) (struct usb_ep *ep, struct usb_request *req);

  int (*queue) (struct usb_ep *ep, struct usb_request *req,
                gfp_t gfp_flags);
  int (*dequeue) (struct usb_ep *ep, struct usb_request *req);

  int (*set_halt) (struct usb_ep *ep, int value);
  int (*set_wedge) (struct usb_ep *ep);

  int (*fifo_status) (struct usb_ep *ep);
  void (*fifo_flush) (struct usb_ep *ep);
};

/**
 * struct usb_ep - device side representation of USB endpoint
 * @name:identifier for the endpoint, such as "ep-a" or "ep9in-bulk"
 * @ops: Function pointers used to access hardware-specific operations.
 * @ep_list:the gadget's ep_list holds all of its endpoints
 * @maxpacket:The maximum packet size used on this endpoint.  The initial
 *      value can sometimes be reduced (hardware allowing), according to
 *      the endpoint descriptor used to configure the endpoint.
 * @max_streams: The maximum number of streams supported
 *      by this EP (0 - 16, actual number is 2^n)
 * @mult: multiplier, 'mult' value for SS Isoc EPs
 * @maxburst: the maximum number of bursts supported by this EP (for usb3)
 * @driver_data:for use by the gadget driver.
 * @address: used to identify the endpoint when finding descriptor that
 *      matches connection speed
 * @desc: endpoint descriptor.  This pointer is set before the endpoint is
 *      enabled and remains valid until the endpoint is disabled.
 * @comp_desc: In case of SuperSpeed support, this is the endpoint companion
 *      descriptor that is used to configure the endpoint
 *
 * the bus controller driver lists all the general purpose endpoints in
 * gadget->ep_list.  the control endpoint (gadget->ep0) is not in that list,
 * and is accessed only in response to a driver setup() callback.
 */
struct usb_ep {
  void                    *driver_data;

  const char              *name;
  const struct usb_ep_ops *ops;
  struct list_head        ep_list;
  unsigned                maxpacket:16;
  unsigned                max_streams:16;
  unsigned                mult:2;
  unsigned                maxburst:5;
  u8                      address;
  const struct USB_EPT_DESC    *desc;
  const struct usb_ss_ep_comp_descriptor  *comp_desc;
};

struct usb_gadget;
struct usb_gadget_driver;

struct usb_gadget_ops {
  int     (*get_frame)(struct usb_gadget *);
  int     (*wakeup)(struct usb_gadget *);
  int     (*set_selfpowered) (struct usb_gadget *, int is_selfpowered);
  int     (*vbus_session) (struct usb_gadget *, int is_active);
  int     (*vbus_draw) (struct usb_gadget *, unsigned mA);
  int     (*pullup) (struct usb_gadget *, int is_on);
  int     (*ioctl)(struct usb_gadget *,
                   unsigned code, unsigned long param);
  //void    (*get_config_params)(struct usb_dcd_config_params *);
  int     (*udc_start)(struct usb_gadget *,
                       struct usb_gadget_driver *);
  int     (*udc_stop)(struct usb_gadget *,
                      struct usb_gadget_driver *);
  
  /* Those two are deprecated */
  int     (*start)(struct usb_gadget_driver *,
                   int (*bind)(struct usb_gadget *));
  int     (*stop)(struct usb_gadget_driver *);
};

struct usb_gadget {
  /* readonly to gadget driver */
  const struct usb_gadget_ops     *ops;
  struct usb_ep                   *ep0;
  struct list_head                ep_list;        /* of usb_ep */
  int           speed;
  int           max_speed;
  unsigned                        sg_supported:1;
  unsigned                        is_otg:1;
  unsigned                        is_a_peripheral:1;
  unsigned                        b_hnp_enable:1;
  unsigned                        a_hnp_support:1;
  unsigned                        a_alt_hnp_support:1;
  const char                      *name;
  //struct device                   dev;
} ;

struct usb_gadget_driver {
  char                    *function;
  int   max_speed;
  void                    (*unbind)(struct usb_gadget *);
  int                     (*setup)(struct usb_gadget *,
                                   const struct usb_dev_req *);
  void                    (*disconnect)(struct usb_gadget *);
  void                    (*suspend)(struct usb_gadget *);
  void                    (*resume)(struct usb_gadget *);

  /* FIXME support safe rmmod */
  //struct device_driver    driver;
};

static inline void
usb_gadget_unmap_request(struct usb_gadget *gadget, struct usb_request *req, int is_in)
{
  com1_printf("IN USB_GADGET_UNMAP_REQUEST WHICH IS A DUMMY FUNCTION");
  panic("IN USB_GADGET_UNMAP_REQUEST WHICH IS A DUMMY FUNCTION");
}


#endif //_GADGET_H_

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
