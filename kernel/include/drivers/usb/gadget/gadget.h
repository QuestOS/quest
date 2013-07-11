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

#define USB_GADGET_DELAYED_STATUS       0x7fff  /* Impossibly large value */
#define CONFIG_USB_GADGET_VBUS_DRAW 500

struct usb_gadget;

static inline int gadget_is_dualspeed(struct usb_gadget *g)  { return 1; }
static inline int gadget_is_superspeed(struct usb_gadget *g) { return 0; }
static inline int gadget_is_otg(struct usb_gadget *g) { return 0; }

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
 * These are allocated/freed through the endpoint they're used with.
 * The hardware's driver can add extra per-request data to the memory
 * it returns, which often avoids separate memory allocations
 * (potential failures), later when the request is queued.
 *
 * Request flags affect request handling, such as whether a zero
 * length packet is written (the "zero" flag), whether a short read
 * should be treated as an error (blocking request queue advance, the
 * "short_not_ok" flag), or hinting that an interrupt is not required
 * (the "no_interrupt" flag, for use with deep request queues).
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
  const USB_EPT_DESC    *desc;
  const struct usb_ss_ep_comp_descriptor  *comp_desc;
};

struct usb_string {
  u8                      id;
  const char              *s;
};

struct usb_gadget_strings {
  u16                     language;       /* 0x0409 for en-us */
  struct usb_string       *strings;
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

#define BITS_PER_BYTE           8
#define DECLARE_BITMAP(name,bits) unsigned long name[BITS_TO_LONGS(bits)]
#define BITS_TO_LONGS(nr)       DIV_ROUND_UP(nr, BITS_PER_BYTE * sizeof(long))



struct usb_function {
  const char                      *name;
  struct usb_gadget_strings       **strings;
  struct usb_descriptor_header    **descriptors;
  struct usb_descriptor_header    **hs_descriptors;
  struct usb_descriptor_header    **ss_descriptors;
  
  struct usb_configuration        *config;
  
  /* REVISIT:  bind() functions can be marked __init, which
   * makes trouble for section mismatch analysis.  See if
   * we can't restructure things to avoid mismatching.
   * Related:  unbind() may kfree() but bind() won't...
   */
  
  /* configuration management:  bind/unbind */
  int                     (*bind)(struct usb_configuration *,
                                  struct usb_function *);
  void                    (*unbind)(struct usb_configuration *,
                                    struct usb_function *);
  
  /* runtime state management */
  int                     (*set_alt)(struct usb_function *,
                                     unsigned interface, unsigned alt);
  int                     (*get_alt)(struct usb_function *,
                                     unsigned interface);
  void                    (*disable)(struct usb_function *);
  int                     (*setup)(struct usb_function *,
                                   const struct usb_ctrlrequest *);
  void                    (*suspend)(struct usb_function *);
  void                    (*resume)(struct usb_function *);

  /* USB 3.0 additions */
  int                     (*get_status)(struct usb_function *);
  int                     (*func_suspend)(struct usb_function *,
                                          u8 suspend_opt);
  /* private: */
  /* internals */
  struct list_head                list;
  DECLARE_BITMAP(endpoints, 32);
};



#define MAX_CONFIG_INTERFACES 20

struct usb_configuration {
  const char                      *label;
  struct usb_gadget_strings       **strings;
  const struct usb_descriptor_header **descriptors;

  /* REVISIT:  bind() functions can be marked __init, which
   * makes trouble for section mismatch analysis.  See if
   * we can't restructure things to avoid mismatching...
   */

  /* configuration management: unbind/setup */
  void                    (*unbind)(struct usb_configuration *);
  int                     (*setup)(struct usb_configuration *,
                                   const struct usb_ctrlrequest *);

  /* fields in the config descriptor */
  u8                      bConfigurationValue;
  u8                      iConfiguration;
  u8                      bmAttributes;
  u8                      bMaxPower;

  /* private: */
  /* internals */
  struct list_head        list;
  struct list_head        functions;
  u8                      next_interface_id;
  unsigned                superspeed:1;
  unsigned                highspeed:1;
  unsigned                fullspeed:1;
  struct usb_function     *interface[MAX_CONFIG_INTERFACES];
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
};




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

static inline int usb_ep_enable(struct usb_ep *ep)
{
  return ep->ops->enable(ep, ep->desc);
}

static inline int usb_ep_queue(struct usb_ep *ep,
                               struct usb_request *req, gfp_t gfp_flags)
{
  return ep->ops->queue(ep, req, gfp_flags);
}

/* Byte order for UTF-16 strings */
enum utf16_endian {
  UTF16_HOST_ENDIAN,
  UTF16_LITTLE_ENDIAN,
  UTF16_BIG_ENDIAN
};

typedef u16 wchar_t;
typedef u32 unicode_t;

#define UNICODE_MAX     0x0010ffff
#define PLANE_SIZE      0x00010000

#define SURROGATE_MASK  0xfffff800
#define SURROGATE_PAIR  0x0000d800
#define SURROGATE_LOW   0x00000400
#define SURROGATE_BITS  0x000003ff

struct utf8_table {
  int     cmask;
  int     cval;
  int     shift;
  long    lmask;
  long    lval;
};

static const struct utf8_table utf8_table[] = {
  {0x80,  0x00,   0*6,    0x7F,           0,         /* 1 byte sequence */},
  {0xE0,  0xC0,   1*6,    0x7FF,          0x80,      /* 2 byte sequence */},
  {0xF0,  0xE0,   2*6,    0xFFFF,         0x800,     /* 3 byte sequence */},
  {0xF8,  0xF0,   3*6,    0x1FFFFF,       0x10000,   /* 4 byte sequence */},
  {0xFC,  0xF8,   4*6,    0x3FFFFFF,      0x200000,  /* 5 byte sequence */},
  {0xFE,  0xFC,   5*6,    0x7FFFFFFF,     0x4000000, /* 6 byte sequence */},
  {0,                                                /* end of table    */}
};

static inline void put_utf16(wchar_t *s, unsigned c, enum utf16_endian endian)
{
  
  /* -- EM -- Breaking the endian check so it compiles */
  switch (endian) {
  default:
    *s = (wchar_t) c;
    break;
    /*
  case UTF16_LITTLE_ENDIAN:
    *s = __cpu_to_le16(c);
    break;
  case UTF16_BIG_ENDIAN: 
    *s = __cpu_to_be16(c);
    break;
    */
  }
}

static int utf8_to_utf32(const u8 *s, int inlen, unicode_t *pu)
{
  unsigned long l;
  int c0, c, nc;
  const struct utf8_table *t;
  
  nc = 0;
  c0 = *s;
  l = c0;
  for (t = utf8_table; t->cmask; t++) {
    nc++;
    if ((c0 & t->cmask) == t->cval) {
      l &= t->lmask;
      if (l < t->lval || l > UNICODE_MAX ||
          (l & SURROGATE_MASK) == SURROGATE_PAIR)
        return -1;
      *pu = (unicode_t) l;
      return nc;
    }
    if (inlen <= nc)
      return -1;
    s++;
    c = (*s ^ 0x80) & 0xFF;
    if (c & 0xC0)
      return -1;
    l = (l << 6) | c;
  }
  return -1;
}

static int utf8s_to_utf16s(const u8 *s, int inlen, enum utf16_endian endian,
                           wchar_t *pwcs, int maxout)
{
  u16 *op;
  int size;
  unicode_t u;

  op = pwcs;
  while (inlen > 0 && maxout > 0 && *s) {
    if (*s & 0x80) {
      size = utf8_to_utf32(s, inlen, &u);
      if (size < 0)
        return -1;
      s += size;
      inlen -= size;

      if (u >= PLANE_SIZE) {
        if (maxout < 2)
          break;
        u -= PLANE_SIZE;
        put_utf16(op++, SURROGATE_PAIR |
                  ((u >> 10) & SURROGATE_BITS),
                  endian);
        put_utf16(op++, SURROGATE_PAIR |
                  SURROGATE_LOW |
                  (u & SURROGATE_BITS),
                  endian);
        maxout -= 2;
      } else {
        put_utf16(op++, u, endian);
        maxout--;
      }
    } else {
      put_utf16(op++, *s++, endian);
      inlen--;
      maxout--;
    }
  }
  return op - pwcs;
}

/**
 * usb_gadget_get_string - fill out a string descriptor 
 * @table: of c strings encoded using UTF-8
 * @id: string id, from low byte of wValue in get string descriptor
 * @buf: at least 256 bytes, must be 16-bit aligned
 *
 * Finds the UTF-8 string matching the ID, and converts it into a
 * string descriptor in utf16-le.
 * Returns length of descriptor (always even) or negative errno
 *
 * If your driver needs stings in multiple languages, you'll probably
 * "switch (wIndex) { ... }"  in your ep0 string descriptor logic,
 * using this routine after choosing which set of UTF-8 strings to use.
 * Note that US-ASCII is a strict subset of UTF-8; any string bytes with
 * the eighth bit set will be multibyte UTF-8 characters, not ISO-8859/1
 * characters (which are also widely used in C strings).
 */
static int
usb_gadget_get_string (struct usb_gadget_strings *table, int id, u8 *buf)
{
        struct usb_string *s;
        int len;

        /* descriptor 0 has the language id */
        if (id == 0) {
                buf [0] = 4;
                buf [1] = USB_DT_STRING;
                buf [2] = (u8) table->language;
                buf [3] = (u8) (table->language >> 8);
                return 4;
        }
        for (s = table->strings; s && s->s; s++)
                if (s->id == id)
                        break;

        /* unrecognized: stall. */
        if (!s || !s->s)
                return -1;

        /* string descriptors have length, tag, then UTF16-LE text */
        len = strlen (s->s) > 126 ? 126 : strlen (s->s);
        //len = min ((size_t) 126, strlen (s->s));
        len = utf8s_to_utf16s((u8*)s->s, len, UTF16_LITTLE_ENDIAN,
                              (wchar_t *) &buf[2], 126);
        if (len < 0)
                return -1;
        buf [0] = (len + 1) * 2;
        buf [1] = USB_DT_STRING;
        return buf [0];
}

enum dma_data_direction {
  DMA_BIDIRECTIONAL = 0,
  DMA_TO_DEVICE = 1,
  DMA_FROM_DEVICE = 2,
  DMA_NONE = 3,
};

/**
 * usb_ep_fifo_flush - flushes contents of a fifo
 * @ep: the endpoint whose fifo is being flushed.
 *
 * This call may be used to flush the "unclaimed data" that may exist in
 * an endpoint fifo after abnormal transaction terminations.  The call
 * must never be used except when endpoint is not being used for any
 * protocol translation.
 */
static inline void usb_ep_fifo_flush(struct usb_ep *ep)
{
  if (ep->ops->fifo_flush)
    ep->ops->fifo_flush(ep);
}

/**
 * usb_descriptor_fillbuf - fill buffer with descriptors
 * @buf: Buffer to be filled
 * @buflen: Size of buf
 * @src: Array of descriptor pointers, terminated by null pointer.
 *
 * Copies descriptors into the buffer, returning the length or a
 * negative error code if they can't all be copied.  Useful when
 * assembling descriptors for an associated set of interfaces used
 * as part of configuring a composite device; or in other cases where
 * sets of descriptors need to be marshaled.
 */
static int
usb_descriptor_fillbuf(void *buf, unsigned buflen,
                       const struct usb_descriptor_header **src)
{
  u8 *dest = buf;

  if (!src)
    return -1;

  /* fill buffer from src[] until null descriptor ptr */
  for (; NULL != *src; src++) {
    unsigned len = (*src)->bLength;

    if (len > buflen)
      return -1;
    memcpy(dest, *src, len);
    buflen -= len;
    dest += len;
  }
  return dest - (u8 *)buf;
}

static int usb_gadget_config_buf(const struct usb_config_descriptor      *config,
                                 void                                    *buf,
                                 unsigned                                length,
                                 const struct usb_descriptor_header      **desc)
{
  struct usb_config_descriptor *cp = buf;
  int len;

  /* config descriptor first */
  if (length < USB_DT_CONFIG_SIZE || !desc)
    return -1;
  *cp = *config;

  /* then interface/endpoint/class/vendor/... */
  len = usb_descriptor_fillbuf(USB_DT_CONFIG_SIZE + (u8*)buf,
                               length - USB_DT_CONFIG_SIZE, desc);
  if (len < 0)
    return len;
  len += USB_DT_CONFIG_SIZE;
  if (len > 0xffff)
    return -1;

  /* patch up the config descriptor */
  cp->bLength = USB_DT_CONFIG_SIZE;
  cp->bDescriptorType = USB_DT_CONFIG;
  cp->wTotalLength = cpu_to_le16(len);
  cp->bmAttributes |= USB_CONFIG_ATT_ONE;
  return len;
}


/**
 * usb_ep_alloc_request - allocate a request object to use with this endpoint
 * @ep:the endpoint to be used with with the request
 * @gfp_flags:GFP_* flags to use
 *
 * Request objects must be allocated with this call, since they normally
 * need controller-specific setup and may even need endpoint-specific
 * resources such as allocation of DMA descriptors.
 * Requests may be submitted with usb_ep_queue(), and receive a single
 * completion callback.  Free requests with usb_ep_free_request(), when
 * they are no longer needed.
 *
 * Returns the request, or null if one could not be allocated.
 */
static inline struct usb_request*
usb_ep_alloc_request(struct usb_ep *ep, gfp_t gfp_flags)
{
  return ep->ops->alloc_request(ep, gfp_flags);
}

/**
 * usb_ep_free_request - frees a request object
 * @ep:the endpoint associated with the request
 * @req:the request being freed
 *
 * Reverses the effect of usb_ep_alloc_request().
 * Caller guarantees the request is not queued, and that it will
 * no longer be requeued (or otherwise used).
 */
static inline void usb_ep_free_request(struct usb_ep *ep,
                                       struct usb_request *req)
{
  ep->ops->free_request(ep, req);
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
