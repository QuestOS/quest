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

/* The following code is modified from the Linux net2280 device driver
 * below is the copyright from Linux
 *
 * Copyright (C) 2003 David Brownell
 * Copyright (C) 2003-2005 PLX Technology, Inc.
 *
 * Modified Seth Levy 2005 PLX Technology, Inc. to provide compatibility
 *      with 2282 chip
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#include <smp/apic.h>
#include <drivers/usb/usb.h>
#include <drivers/usb/linux_usb.h>
#include <util/printf.h>
#include <kernel.h>
#include <drivers/pci/pci.h>
#include <drivers/usb/gadget/net2280.h>
#include <drivers/usb/gadget/gadget.h>
#include <mem/virtual.h>

/*
 * FIFO_MODE 0 == ep-{a,b,c,d} 1K fifo each
 * FIFO_MODE 1 == ep-{a,b} 2K fifo each, ep-{c,d} unavailable
 * FIFO_MODE 2 == ep-a 2K fifo, ep-{b,c} 1K each, ep-d unavailable
 */
#define FIFO_MODE 0

#define DMA_ADDR_INVALID        (~(uint32_t)0)
#define EP_DONTUSE              13      /* nonzero */

static bool use_dma = 1;
static bool use_dma_chaining = 0;

static const char ep0name [] = "ep0";
static const char *const ep_name [] = {
  ep0name,
  "ep-a", "ep-b", "ep-c", "ep-d",
  "ep-e", "ep-f",
};


#define NET2280_ELEMENTS_PER_BITMAP_ENTRY (sizeof(uint32_t) * 8)
#define BITMAP_ALL_USED_MASK 0xFFFFFFFF
#define BITMAP_INDEX_SHIFT 5
#define BITMAP_SUBINDEX_MASK (NET2280_ELEMENTS_PER_BITMAP_ENTRY - 1)

#define NET2280_DMA_POOL_SIZE  64

static net2280_dma_t net2280_dma_pool[NET2280_DMA_POOL_SIZE] ALIGNED(0x1000);
static uint32_t used_net2280_dma_bitmap[(NET2280_DMA_POOL_SIZE + NET2280_ELEMENTS_PER_BITMAP_ENTRY - 1)
                                        / NET2280_ELEMENTS_PER_BITMAP_ENTRY];


#define DEBUG_NET2280
#define DEBUG_NET2280_VERBOSE

#ifdef DEBUG_NET2280
#define DLOG(fmt,...) DLOG_PREFIX("Net2280",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef DEBUG_NET2280_VERBOSE
#define DLOGV(fmt,...) DLOG_PREFIX("Net2280",fmt,##__VA_ARGS__)
#else
#define DLOGV(fmt,...) ;
#endif

struct net2280 net2280_dev;

int default_net2280_usb_gadget_setup(struct usb_gadget* gadget,
                                     const struct usb_dev_req* ctrl_request)
{
  DLOG("In default_net2280_usb_gadget_setup");
  panic("In default_net2280_usb_gadget_setup");
}

struct usb_gadget_driver default_net2280_usb_gadget_driver = 
{
  .function = "NET2280 Test Gadget",
  .max_speed = USB_SPEED_HIGH,
  .unbind = NULL,
  .setup = default_net2280_usb_gadget_setup,
  .disconnect = NULL,
  .suspend = NULL,
  .resume = NULL, 
};


inline void writel(u32 b, volatile void *addr) {
  *(volatile u32 *) addr = b;
}

inline u32 readl(const volatile void *addr)
{
  u32 ret = *(const volatile u32 *)(addr);
  return ret;
}

static void rmb ()
{
  gccmb();
}

static void wmb()
{
  gccmb();
}

static uint32_t
net2280_irq_handler(uint8 vec)
{
  DLOG("In %s", __FUNCTION__);
  panic("IN NET2280 IRQ HANDLER");
}




/*
 * -- EM -- I know the following is ugly, but right now since all the
 * pools are statically declared and that is not the way it will
 * permanently be it does not matter.  Eventually the net2280 memory
 * stuff will use the kernel heap/we will have a more unified dma pool
 * system similar to Linux and this can be redone to be nicer and
 * simpler since all these static pools will go away
 */

#define DECLARE_NET2280_MEM_FUNCTIONS(type)                             \
  inline bool initialise_##type(struct net2280* net2280, type##_t* item); \
  uint32_t allocate_##type##s(struct net2280* net2280, type##_t** items, \
                              uint32_t num_items);                      \
  inline type##_t* allocate_##type(struct net2280* net2280);            \
  void free_##type##s(struct net2280* net2280, type##_t** items,        \
                      uint32_t num_items);                              \
  inline void free_##type(struct net2280* net2280, type##_t* item);

DECLARE_NET2280_MEM_FUNCTIONS(net2280_dma)

#undef DECLARE_NET2280_MEM_FUNCTIONS

#define NET2280_RESOURCE_MEMORY_FUNCS(res)                              \
  uint32_t allocate_##res##s(struct net2280* net2280, res##_t** items, uint32_t num) \
  {                                                                     \
    uint32_t  count       = 0;                                          \
    uint32_t  entry;                                                    \
    uint32_t  i           = net2280->used_##res##_bitmap_size;          \
    uint32_t* used_bitmap = net2280->used_##res##_bitmap;               \
    res##_t*  pool        = net2280->res##_pool;                        \
                                                                        \
    while(i--) {                                                        \
      if(used_bitmap[i] == INT_MAX) continue;                           \
                                                                        \
      entry = NET2280_ELEMENTS_PER_BITMAP_ENTRY;                        \
      while(entry--) {                                                  \
        if(!(used_bitmap[i] & (1 << entry)) ) {                         \
          used_bitmap[i] |= (1 << entry); /* Mark entry as allocated */ \
          items[count] = &pool[i * NET2280_ELEMENTS_PER_BITMAP_ENTRY + entry]; \
          if(!initialise_##res(net2280, items[count])) {                \
            free_##res(net2280, items[count]);                          \
            return count;                                               \
          }                                                             \
          if(++count == num) {                                          \
            return count;                                               \
          }                                                             \
        }                                                               \
      }                                                                 \
    }                                                                   \
    return count;                                                       \
  }                                                                     \
                                                                        \
  inline res##_t* allocate_##res(struct net2280* net2280)               \
  {                                                                     \
    res##_t* temp;                                                      \
    return allocate_##res##s(net2280, &temp, 1) ? temp : NULL;          \
  }                                                                     \
                                                                        \
  void free_##res##s(struct net2280* net2280, res##_t** items, uint32_t num) \
  {                                                                     \
    uint32_t* used_bitmap = net2280->used_##res##_bitmap;               \
    res##_t*    pool        = net2280->res##_pool;                      \
                                                                        \
    while(num--) {                                                      \
      uint32_t index = items[num] - pool;                               \
                                                                        \
      /* Flip the one bit to zero */                                    \
      used_bitmap[index >> BITMAP_INDEX_SHIFT] &=                       \
      ~(1 << (index & BITMAP_SUBINDEX_MASK));                           \
    }                                                                   \
  }                                                                     \
                                                                        \
  inline void free_##res(struct net2280* net2280, res##_t* item)        \
  {                                                                     \
    free_##res##s(net2280, &item, 1);                                   \
  }

inline bool
initialise_net2280_dma(struct net2280* net2280, net2280_dma_t* dma)
{
  memset(dma, 0, sizeof(*dma));
  return TRUE;
}

#define __NET2280_POOL_PHYS_TO_VIRT(dev, phys_addr, pool)                  \
  ((((uint32_t)phys_addr) - ((uint32_t)(dev)->pool ## _phys_addr)) + ((uint32_t)(dev)->pool))

#define __NET2280_POOL_VIRT_TO_PHYS(hcd, virt_addr, pool)                  \
  ((((uint32_t)virt_addr) - ((uint32_t)(dev)->pool)) + ((uint32_t)(dev)->pool ## _phys_addr))

#define NET2280_DMA_VIRT_TO_PHYS(dev, virt_addr)                \
  __NET2280_POOL_VIRT_TO_PHYS(dev, virt_addr, net2280_dma_pool)

#define NET2280_DMA_PHYS_TO_VIRT(dev, phys_addr)                        \
  ((net2280_dma*)__NET2280_POOL_PHYS_TO_VIRT(dev, phys_addr, net2280_dma_pool))

NET2280_RESOURCE_MEMORY_FUNCS(net2280_dma)

#undef NET2280_RESOURCE_MEMORY_FUNCS

#define REG_EP_MAXPKT(dev,num) (((num) + 1) * 0x10 + \
                                (((dev)->gadget.speed == USB_SPEED_HIGH) ? 0 : 1))

static inline void
set_idx_reg (struct net2280_regs *regs, u32 index, u32 value)
{
  writel (index, &regs->idxaddr);
  writel (value, &regs->idxdata);
  /* posted, may not be visible yet */
}

static int net2280_enable (struct usb_ep *_ep,
                    const USB_EPT_DESC *desc)
{
  struct net2280          *dev;
  struct net2280_ep       *ep;
  u32                     max, tmp;
  
 
  ep = container_of (_ep, struct net2280_ep, ep);
  if (!_ep || !desc || ep->desc || _ep->name == ep0name
      || desc->bDescriptorType != USB_TYPE_EPT_DESC)
    return -1;
  dev = ep->dev;
  if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
    return -1;
 
  /* erratum 0119 workaround ties up an endpoint number */
  if ((desc->bEndpointAddress & 0x0f) == EP_DONTUSE)
    return -1;
 
  /* sanity check ep-e/ep-f since their fifos are small */
  max = desc->wMaxPacketSize & 0x1fff;
  if (ep->num > 4 && max > 64)
    return -1;
 
  spinlock_lock(&dev->lock);
  _ep->maxpacket = max & 0x7ff;
  ep->desc = desc;
 
  /* ep_reset() has already been called */
  ep->stopped = 0;
  ep->wedged = 0;
  ep->out_overflow = 0;
 
  /* set speed-dependent max packet; may kick in high bandwidth */
  set_idx_reg (dev->regs, REG_EP_MAXPKT (dev, ep->num), max);
 
  /* FIFO lines can't go to different packets.  PIO is ok, so
   * use it instead of troublesome (non-bulk) multi-packet DMA.
   */
  if (ep->dma && (max % 4) != 0 && use_dma_chaining) {
    DLOG("%s, no dma for maxpacket %d\n",
           ep->ep.name, ep->ep.maxpacket);
    ep->dma = NULL;
  }
 
  /* set type, direction, address; reset fifo counters */
  writel ((1 << FIFO_FLUSH), &ep->regs->ep_stat);
  tmp = (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK);
  if (tmp == USB_ENDPOINT_XFER_INT) {
    /* erratum 0105 workaround prevents hs NYET */
    if (dev->chiprev == 0100
        && dev->gadget.speed == USB_SPEED_HIGH
        && !(desc->bEndpointAddress & USB_DIR_IN))
      writel ((1 << CLEAR_NAK_OUT_PACKETS_MODE),
              &ep->regs->ep_rsp);
  } else if (tmp == USB_ENDPOINT_XFER_BULK) {
    /* catch some particularly blatant driver bugs */
    if ((dev->gadget.speed == USB_SPEED_HIGH
         && max != 512)
        || (dev->gadget.speed == USB_SPEED_FULL
            && max > 64)) {
      spinlock_unlock (&dev->lock);
      return -1;
    }
  }
  ep->is_iso = (tmp == USB_ENDPOINT_XFER_ISOC) ? 1 : 0;
  tmp <<= ENDPOINT_TYPE;
  tmp |= desc->bEndpointAddress;
  tmp |= (4 << ENDPOINT_BYTE_COUNT);      /* default full fifo lines */
  tmp |= 1 << ENDPOINT_ENABLE;
  wmb ();
 
  /* for OUT transfers, block the rx fifo until a read is posted */
  ep->is_in = (tmp & USB_DIR_IN) != 0;
  if (!ep->is_in)
    writel ((1 << SET_NAK_OUT_PACKETS), &ep->regs->ep_rsp);
  else if (!dev->is_2280) {
    /* Added for 2282, Don't use nak packets on an in endpoint,
     * this was ignored on 2280
     */
    writel ((1 << CLEAR_NAK_OUT_PACKETS)
            | (1 << CLEAR_NAK_OUT_PACKETS_MODE), &ep->regs->ep_rsp);
  }
 
  writel (tmp, &ep->regs->ep_cfg);
 
  /* enable irqs */
  if (!ep->dma) {                         /* pio, per-packet */
    tmp = (1 << ep->num) | readl (&dev->regs->pciirqenb0);
    writel (tmp, &dev->regs->pciirqenb0);
 
    tmp = (1 << DATA_PACKET_RECEIVED_INTERRUPT_ENABLE)
      | (1 << DATA_PACKET_TRANSMITTED_INTERRUPT_ENABLE);
    if (dev->is_2280)
      tmp |= readl (&ep->regs->ep_irqenb);
    writel (tmp, &ep->regs->ep_irqenb);
  } else {                                /* dma, per-request */
    tmp = (1 << (8 + ep->num));     /* completion */
    tmp |= readl (&dev->regs->pciirqenb1);
    writel (tmp, &dev->regs->pciirqenb1);
 
    /* for short OUT transfers, dma completions can't
     * advance the queue; do it pio-style, by hand.
     * NOTE erratum 0112 workaround #2
     */
    if ((desc->bEndpointAddress & USB_DIR_IN) == 0) {
      tmp = (1 << SHORT_PACKET_TRANSFERRED_INTERRUPT_ENABLE);
      writel (tmp, &ep->regs->ep_irqenb);
 
      tmp = (1 << ep->num) | readl (&dev->regs->pciirqenb0);
      writel (tmp, &dev->regs->pciirqenb0);
    }
  }
 
  tmp = desc->bEndpointAddress;
  //DLOG("enabled %s (ep%d%s-%s) %s max %04x\n",
  //       _ep->name, tmp & 0x0f, DIR_STRING (tmp),
  //       type_string (desc->bmAttributes),
  //       ep->dma ? "dma" : "pio", max);
 
  /* pci writes may still be posted */
  spinlock_unlock(&dev->lock);
  return 0;
} 


int net2280_disable (struct usb_ep *ep)
{
  DLOG("IN %s which is not implemented", __FUNCTION__);
  panic("In unimplemented net2280 ep op function");
}
  
struct usb_request *net2280_alloc_request (struct usb_ep *ep,
                                           gfp_t gfp_flags)
{
  DLOG("IN %s which is not implemented", __FUNCTION__);
  panic("In unimplemented net2280 ep op function");
}
void net2280_free_request (struct usb_ep *ep, struct usb_request *req)
{
  DLOG("IN %s which is not implemented", __FUNCTION__);
  panic("In unimplemented net2280 ep op function");
}
  
int net2280_queue (struct usb_ep *ep, struct usb_request *req,
                   gfp_t gfp_flags)
{
  DLOG("IN %s which is not implemented", __FUNCTION__);
  panic("In unimplemented net2280 ep op function");
}
int net2280_dequeue (struct usb_ep *ep, struct usb_request *req)
{
  DLOG("IN %s which is not implemented", __FUNCTION__);
  panic("In unimplemented net2280 ep op function");
}
  
int net2280_set_halt (struct usb_ep *ep, int value)
{
  DLOG("IN %s which is not implemented", __FUNCTION__);
  panic("In unimplemented net2280 ep op function");
}
int net2280_set_wedge (struct usb_ep *ep)
{
  DLOG("IN %s which is not implemented", __FUNCTION__);
  panic("In unimplemented net2280 ep op function");
}
  
int net2280_fifo_status (struct usb_ep *ep)
{
  DLOG("IN %s which is not implemented", __FUNCTION__);
  panic("In unimplemented net2280 ep op function");
}
void net2280_fifo_flush (struct usb_ep *ep)
{
  DLOG("IN %s which is not implemented", __FUNCTION__);
  panic("In unimplemented net2280 ep op function");
}

static inline void net2280_led_active (struct net2280 *dev, int is_active)
{
  u32     val = readl (&dev->regs->gpioctl);
  
  // FIXME this LED never seems to turn on.
  if (is_active)
    val |= GPIO2_DATA;
  else
    val &= ~GPIO2_DATA;
  writel (val, &dev->regs->gpioctl);
}

static void ep0_start (struct net2280 *dev)
{
  writel (  (1 << CLEAR_EP_HIDE_STATUS_PHASE)
            | (1 << CLEAR_NAK_OUT_PACKETS)
            | (1 << CLEAR_CONTROL_STATUS_PHASE_HANDSHAKE)
            , &dev->epregs [0].ep_rsp);

  /*
   * hardware optionally handles a bunch of standard requests
   * that the API hides from drivers anyway.  have it do so.
   * endpoint status/features are handled in software, to
   * help pass tests for some dubious behavior.
   */
  writel (  (1 << SET_TEST_MODE)
            | (1 << SET_ADDRESS)
            | (1 << DEVICE_SET_CLEAR_DEVICE_REMOTE_WAKEUP)
            | (1 << GET_DEVICE_STATUS)
            | (1 << GET_INTERFACE_STATUS)
            , &dev->usb->stdrsp);
  writel (  (1 << USB_ROOT_PORT_WAKEUP_ENABLE)
            | (1 << SELF_POWERED_USB_DEVICE)
            | (1 << REMOTE_WAKEUP_SUPPORT)
            | (dev->softconnect << USB_DETECT_ENABLE)
            | (1 << SELF_POWERED_STATUS)
            , &dev->usb->usbctl);

  /* enable irqs so we can see ep0 and general operation  */
  writel (  (1 << SETUP_PACKET_INTERRUPT_ENABLE)
            | (1 << ENDPOINT_0_INTERRUPT_ENABLE)
            , &dev->regs->pciirqenb0);
  writel (  (1 << PCI_INTERRUPT_ENABLE)
            | (1 << PCI_MASTER_ABORT_RECEIVED_INTERRUPT_ENABLE)
            | (1 << PCI_TARGET_ABORT_RECEIVED_INTERRUPT_ENABLE)
            | (1 << PCI_RETRY_ABORT_INTERRUPT_ENABLE)
            | (1 << VBUS_INTERRUPT_ENABLE)
            | (1 << ROOT_PORT_RESET_INTERRUPT_ENABLE)
            | (1 << SUSPEND_REQUEST_CHANGE_INTERRUPT_ENABLE)
            , &dev->regs->pciirqenb1);

  /* don't leave any writes posted */
  (void) readl (&dev->usb->usbctl);
}

static int net2280_start(struct usb_gadget *_gadget,
                         struct usb_gadget_driver *driver)
{
  struct net2280          *dev;
  //int                     retval;
  unsigned                i;

  /* insist on high speed support from the driver, since
   * (dev->usb->xcvrdiag & FORCE_FULL_SPEED_MODE)
   * "must not be used in normal operation"
   */
  if (!driver || driver->max_speed < USB_SPEED_HIGH
      || !driver->setup)
    return -1;

  dev = container_of (_gadget, struct net2280, gadget);

  for (i = 0; i < 7; i++)
    dev->ep [i].irqs = 0;

  /* hook up the driver ... */
  dev->softconnect = 1;
  //driver->driver.bus = NULL;
  dev->driver = driver;
  //dev->gadget.dev.driver = &driver->driver;

  //retval = device_create_file (&dev->pdev->dev, &dev_attr_function);
  //if (retval) goto err_unbind;
  //retval = device_create_file (&dev->pdev->dev, &dev_attr_queues);
  //if (retval) goto err_func;

  /* ... then enable host detection and ep0; and we're ready
   * for set_configuration as well as eventual disconnect.
   */
  net2280_led_active (dev, 1);
  ep0_start (dev);

  /* pci writes may still be posted */
  return 0;

  //err_func:
  //DLOG("Error in net2280_start");
  //panic("Error in net2280_start");
  //return retval;
}


int net2280_get_frame(struct usb_gadget *gadget)
{
  DLOG("In unimplemented net2280_get_frame");
  panic("In unimplemented net2280_get_frame");
  return 0;
}

int net2280_wakeup(struct usb_gadget *gadget)
{
  DLOG("In unimplemented net2280_wakeup");
  panic("In unimplemented net2280_wakeup");
  return 0;
}

int net2280_set_selfpowered(struct usb_gadget *gadget, int is_selfpowered)
{
  DLOG("In unimplemented net2280_set_selfpowered");
  panic("In unimplemented net2280_set_selfpowered");
  return 0;
}

int net2280_pullup(struct usb_gadget *gadget, int is_on)
{
  DLOG("In unimplemented net2280_pullup");
  panic("In unimplemented net2280_pullup");
  return 0;
}

int net2280_stop(struct usb_gadget *_gadget, struct usb_gadget_driver *driver)
{
  DLOG("In unimplemented net2280_stop");
  panic("In unimplemented net2280_stop");
  return 0;
}

static const struct usb_gadget_ops net2280_ops = {
  .get_frame      = net2280_get_frame,
  .wakeup         = net2280_wakeup,
  .set_selfpowered = net2280_set_selfpowered,
  .pullup         = net2280_pullup,
  .udc_start      = net2280_start,
  .udc_stop       = net2280_stop,
};

static const struct usb_ep_ops net2280_ep_ops = {
  .enable         = net2280_enable,
  .disable        = net2280_disable,
  
  .alloc_request  = net2280_alloc_request,
  .free_request   = net2280_free_request,
  
  .queue          = net2280_queue,
  .dequeue        = net2280_dequeue,
  
  .set_halt       = net2280_set_halt,
  .set_wedge      = net2280_set_wedge,
  .fifo_status    = net2280_fifo_status,
  .fifo_flush     = net2280_fifo_flush,
};

static inline void net2280_led_init (struct net2280 *dev)
{
  /* LED3 (green) is on during USB activity. note erratum 0113. */
  writel ((1 << GPIO3_LED_SELECT)
          | (1 << GPIO3_OUTPUT_ENABLE)
          | (1 << GPIO2_OUTPUT_ENABLE)
          | (1 << GPIO1_OUTPUT_ENABLE)
          | (1 << GPIO0_OUTPUT_ENABLE)
          , &dev->regs->gpioctl);
}

static void ep_reset (struct net2280_regs *regs, struct net2280_ep *ep)
{
  u32             tmp;

  ep->desc = NULL;
  INIT_LIST_HEAD (&ep->queue);

  ep->ep.maxpacket = ~0;
  ep->ep.ops = &net2280_ep_ops;

  /* disable the dma, irqs, endpoint... */
  if (ep->dma) {
    writel (0, &ep->dma->dmactl);
    writel (  (1 << DMA_SCATTER_GATHER_DONE_INTERRUPT)
              | (1 << DMA_TRANSACTION_DONE_INTERRUPT)
              | (1 << DMA_ABORT)
              , &ep->dma->dmastat);

    tmp = readl (&regs->pciirqenb0);
    tmp &= ~(1 << ep->num);
    writel (tmp, &regs->pciirqenb0);
  } else {
    tmp = readl (&regs->pciirqenb1);
    tmp &= ~(1 << (8 + ep->num));   /* completion */
    writel (tmp, &regs->pciirqenb1);
  }
  writel (0, &ep->regs->ep_irqenb);

  /* init to our chosen defaults, notably so that we NAK OUT
   * packets until the driver queues a read (+note erratum 0112)
   */
  if (!ep->is_in || ep->dev->is_2280) {
    tmp = (1 << SET_NAK_OUT_PACKETS_MODE)
      | (1 << SET_NAK_OUT_PACKETS)
      | (1 << CLEAR_EP_HIDE_STATUS_PHASE)
      | (1 << CLEAR_INTERRUPT_MODE);
  } else {
    /* added for 2282 */
    tmp = (1 << CLEAR_NAK_OUT_PACKETS_MODE)
      | (1 << CLEAR_NAK_OUT_PACKETS)
      | (1 << CLEAR_EP_HIDE_STATUS_PHASE)
      | (1 << CLEAR_INTERRUPT_MODE);
  }

  if (ep->num != 0) {
    tmp |= (1 << CLEAR_ENDPOINT_TOGGLE)
      | (1 << CLEAR_ENDPOINT_HALT);
  }
  writel (tmp, &ep->regs->ep_rsp);

  /* scrub most status bits, and flush any fifo state */
  if (ep->dev->is_2280)
    tmp = (1 << FIFO_OVERFLOW)
      | (1 << FIFO_UNDERFLOW);
  else
    tmp = 0;

  writel (tmp | (1 << TIMEOUT)
          | (1 << USB_STALL_SENT)
          | (1 << USB_IN_NAK_SENT)
          | (1 << USB_IN_ACK_RCVD)
          | (1 << USB_OUT_PING_NAK_SENT)
          | (1 << USB_OUT_ACK_SENT)
          | (1 << FIFO_FLUSH)
          | (1 << SHORT_PACKET_OUT_DONE_INTERRUPT)
          | (1 << SHORT_PACKET_TRANSFERRED_INTERRUPT)
          | (1 << DATA_PACKET_RECEIVED_INTERRUPT)
          | (1 << DATA_PACKET_TRANSMITTED_INTERRUPT)
          | (1 << DATA_OUT_PING_TOKEN_INTERRUPT)
          | (1 << DATA_IN_TOKEN_INTERRUPT)
          , &ep->regs->ep_stat);

  /* fifo size is handled separately */
}


static inline void spin_stop_dma (struct net2280_dma_regs *dma)
{
  handshake (&dma->dmactl, (1 << DMA_ENABLE), 0, 50);
}

static inline void stop_dma (struct net2280_dma_regs *dma)
{
  writel (readl (&dma->dmactl) & ~(1 << DMA_ENABLE), &dma->dmactl);
  spin_stop_dma (dma);
}

static void set_fifo_mode (struct net2280 *dev, int mode)
{
  /* keeping high bits preserves BAR2 */
  writel ((0xffff << PCI_BASE2_RANGE) | mode, &dev->regs->fifoctl);

  /* always ep-{a,b,e,f} ... maybe not ep-c or ep-d */
  INIT_LIST_HEAD (&dev->gadget.ep_list);
  list_add_tail (&dev->ep [1].ep.ep_list, &dev->gadget.ep_list);
  list_add_tail (&dev->ep [2].ep.ep_list, &dev->gadget.ep_list);
  switch (mode) {
  case 0:
    list_add_tail (&dev->ep [3].ep.ep_list, &dev->gadget.ep_list);
    list_add_tail (&dev->ep [4].ep.ep_list, &dev->gadget.ep_list);
    dev->ep [1].fifo_size = dev->ep [2].fifo_size = 1024;
    break;
  case 1:
    dev->ep [1].fifo_size = dev->ep [2].fifo_size = 2048;
    break;
  case 2:
    list_add_tail (&dev->ep [3].ep.ep_list, &dev->gadget.ep_list);
    dev->ep [1].fifo_size = 2048;
    dev->ep [2].fifo_size = 1024;
    break;
  }
  /* fifo sizes for ep0, ep-c, ep-d, ep-e, and ep-f never change */
  list_add_tail (&dev->ep [5].ep.ep_list, &dev->gadget.ep_list);
  list_add_tail (&dev->ep [6].ep.ep_list, &dev->gadget.ep_list);
}

static void done (struct net2280_ep *ep, struct net2280_request *req, int status)
{
  struct net2280          *dev;
  unsigned                stopped = ep->stopped;

  list_del_init (&req->queue);

  if (req->req.status == -1)
    req->req.status = status;
  else
    status = req->req.status;

  dev = ep->dev;
  if (ep->dma)
    usb_gadget_unmap_request(&dev->gadget, &req->req, ep->is_in);

  if (status && status != -1)
    DLOG("complete %s req %p stat %d len %u/%u\n",
         ep->ep.name, &req->req, status,
         req->req.actual, req->req.length);

  /* don't modify queue heads during completion callback */
  ep->stopped = 1;
  spinlock_unlock (&dev->lock);
  req->req.complete (&ep->ep, &req->req);
  spinlock_lock (&dev->lock);
  ep->stopped = stopped;
}

static inline void
dma_done (struct net2280_ep *ep, struct net2280_request *req,
          u32 dmacount, int status)
{
  req->req.actual = req->req.length - (DMA_BYTE_COUNT_MASK & dmacount);
  done (ep, req, status);
}

static void scan_dma_completions (struct net2280_ep *ep)
{
  /* only look at descriptors that were "naturally" retired,
   * so fifo and list head state won't matter
   */
  while (!list_empty (&ep->queue)) {
    struct net2280_request  *req;
    u32                     tmp;
    
    req = list_entry (ep->queue.next,
                      struct net2280_request, queue);
    if (!req->valid)
      break;
    rmb ();
    tmp = req->td->dmacount;
    if ((tmp & (1 << VALID_BIT)) != 0)
      break;

    /* SHORT_PACKET_TRANSFERRED_INTERRUPT handles "usb-short"
     * cases where DMA must be aborted; this code handles
     * all non-abort DMA completions.
     */
    if (unlikely (req->td->dmadesc == 0)) {
      /* paranoia */
      tmp = readl (&ep->dma->dmacount);
      if (tmp & DMA_BYTE_COUNT_MASK)
        break;
      /* single transfer mode */
      dma_done (ep, req, tmp, 0);
      break;
    } else if (!ep->is_in
               && (req->req.length % ep->ep.maxpacket) != 0) {
      tmp = readl (&ep->regs->ep_stat);

      /* AVOID TROUBLE HERE by not issuing short reads from
       * your gadget driver.  That helps avoids errata 0121,
       * 0122, and 0124; not all cases trigger the warning.
       */
      if ((tmp & (1 << NAK_OUT_PACKETS)) == 0) {
        DLOG ("%s lost packet sync!\n",
              ep->ep.name);
        req->req.status = -1;
      } else if ((tmp = readl (&ep->regs->ep_avail)) != 0) {
        /* fifo gets flushed later */
        ep->out_overflow = 1;
        DLOG ("%s dma, discard %d len %d\n",
              ep->ep.name, tmp,
              req->req.length);
        req->req.status = -1;
      }
    }
    dma_done (ep, req, tmp, 0);
  }
}

static void abort_dma (struct net2280_ep *ep)
{
  /* abort the current transfer */
  if (likely (!list_empty (&ep->queue))) {
    /* FIXME work around errata 0121, 0122, 0124 */
    writel ((1 << DMA_ABORT), &ep->dma->dmastat);
    spin_stop_dma (ep->dma);
  } else
    stop_dma (ep->dma);
  scan_dma_completions (ep);
}

static void usb_reset(struct net2280* dev)
{
  u32     tmp;

  dev->gadget.speed = USB_SPEED_UNKNOWN;
  (void) readl (&dev->usb->usbctl);

  net2280_led_init (dev);

  /* disable automatic responses, and irqs */
  writel (0, &dev->usb->stdrsp);
  writel (0, &dev->regs->pciirqenb0);
  writel (0, &dev->regs->pciirqenb1);

  /* clear old dma and irq state */
  for (tmp = 0; tmp < 4; tmp++) {
    struct net2280_ep       *ep = &dev->ep [tmp + 1];
    
    if (ep->dma)
      abort_dma (ep);
  }
  writel (~0, &dev->regs->irqstat0),
    writel (~(1 << SUSPEND_REQUEST_INTERRUPT), &dev->regs->irqstat1),

    /* reset, and enable pci */
    tmp = readl (&dev->regs->devinit)
    | (1 << PCI_ENABLE)
    | (1 << FIFO_SOFT_RESET)
    | (1 << USB_SOFT_RESET)
    | (1 << M8051_RESET);
  writel (tmp, &dev->regs->devinit);

  /* standard fifo and endpoint allocations */
  set_fifo_mode (dev, (FIFO_MODE <= 2) ? FIFO_MODE : 0);
}


static void usb_reinit(struct net2280 *dev)
{
  u32     tmp;
  int     init_dma;

  /* use_dma changes are ignored till next device re-init */
  init_dma = use_dma;

  /* basic endpoint init */
  for (tmp = 0; tmp < 7; tmp++) {
    struct net2280_ep       *ep = &dev->ep [tmp];

    ep->ep.name = ep_name [tmp];
    ep->dev = dev;
    ep->num = tmp;

    if (tmp > 0 && tmp <= 4) {
      ep->fifo_size = 1024;
      if (init_dma)
        ep->dma = &dev->dma [tmp - 1];
    } else
      ep->fifo_size = 64;
    ep->regs = &dev->epregs [tmp];
    ep_reset (dev->regs, ep);
  }
  dev->ep [0].ep.maxpacket = 64;
  dev->ep [5].ep.maxpacket = 64;
  dev->ep [6].ep.maxpacket = 64;

  dev->gadget.ep0 = &dev->ep [0].ep;
  dev->ep [0].stopped = 0;
  INIT_LIST_HEAD (&dev->gadget.ep0->ep_list);

  /* we want to prevent lowlevel/insecure access from the USB host,
   * but erratum 0119 means this enable bit is ignored
   */
  for (tmp = 0; tmp < 5; tmp++)
    writel (EP_DONTUSE, &dev->dep [tmp].dep_cfg);
}

static bool initialise_net2280(struct net2280* dev, uint32_t base_addr, bool is_2280,
                               net2280_dma_t* net2280_dma_pool,
                               uint32_t net2280_dma_pool_size,
                               uint32_t* used_net2280_dma_bitmap)
{
  int i;
  addr_t base_virt_addr = map_virtual_page(base_addr | 0x3);

  
  dev->gadget.ops = &net2280_ops;
  dev->gadget.max_speed = USB_SPEED_HIGH;
  dev->regs   = (struct net2280_regs *)     (base_virt_addr);
  dev->usb    = (struct net2280_usb_regs *) (base_virt_addr + 0x0080);
  dev->pci    = (struct net2280_pci_regs *) (base_virt_addr + 0x0100);
  dev->dma    = (struct net2280_dma_regs *) (base_virt_addr + 0x0180);
  dev->dep    = (struct net2280_dep_regs *) (base_virt_addr + 0x0200);
  dev->epregs = (struct net2280_ep_regs *)  (base_virt_addr + 0x0300);
  dev->is_2280 = is_2280;

  writel (0, &dev->usb->usbctl);

  usb_reset(dev);
  usb_reinit(dev);

  dev->got_irq = 1;             /* Always have IRQ in Quest implementation */

#define INIT_NET2280_POOL(type)                                         \
    do {                                                                \
    dev->type##_pool = type##_pool;                                     \
    dev->type##_pool_phys_addr = (phys_addr_t)get_phys_addr(type##_pool); \
    dev->type##_pool_size = type##_pool_size;                           \
    dev->used_##type##_bitmap = used_##type##_bitmap;                   \
    memset(type##_pool, 0, type##_pool_size * sizeof(type##_t));        \
    dev->used_##type##_bitmap_size =                                     \
      ((dev)->type##_pool_size + (NET2280_ELEMENTS_PER_BITMAP_ENTRY - 1)) \
      / NET2280_ELEMENTS_PER_BITMAP_ENTRY;                              \
    memset(used_##type##_bitmap, 0,                                     \
           dev->used_##type##_bitmap_size * sizeof(*used_##type##_bitmap)); \
    }                                                                   \
    while(0)

  INIT_NET2280_POOL(net2280_dma);
  
#undef INIT_NET2280_POOL
  
  for (i = 1; i < 5; i++) {
    struct net2280_dma      *td;
    
    td = allocate_net2280_dma(dev);
    if (!td) {
      /* -- EM -- Add proper cleanup code later */
      DLOG("Failed to get dummy net2280_dma");
      panic("Failed to get dummy net2280_dma");
    }
    dev->ep[i].td_dma = NET2280_DMA_VIRT_TO_PHYS(dev, td);
    td->dmacount = 0;       /* not VALID */
    td->dmaaddr = (DMA_ADDR_INVALID);
    td->dmadesc = td->dmaaddr;
    dev->ep [i].dummy = td;
  }


  /* enable lower-overhead pci memory bursts during DMA */
  writel ( (1 << DMA_MEMORY_WRITE_AND_INVALIDATE_ENABLE)
    // 256 write retries may not be enough...
    // | (1 << PCI_RETRY_ABORT_ENABLE)
    | (1 << DMA_READ_MULTIPLE_ENABLE)
    | (1 << DMA_READ_LINE_ENABLE)
    , &dev->pci->pcimstctl);

  return TRUE;
}


static bool
net2280_init(void)
{
  uint i, device_index, irq_pin;
  pci_device net2280_device;
  pci_irq_t irq;
  uint32_t net_base;
  DLOG("In net2280");

  if(mp_ISA_PC) {
    DLOG("Cannot operate without PCI");
    DLOGV("Exiting %s with FALSE", __FUNCTION__);
    return FALSE;
  }
   
  /* Find the Net2280 device on the PCI bus */
  device_index = ~0;
  i = 0;
  
  while (pci_find_device (0x17cc, 0x2280, 0xFF, 0xFF, i, &i)) {
    if (pci_get_device (i, &net2280_device)) {
      device_index = i;
      break;
    }
    else {
      break;
      return FALSE;
    }
  }

  if (!pci_get_device (device_index, &net2280_device)) {
    DLOG ("Unable to get PCI device from PCI subsystem");
    DLOGV("Exiting %s with FALSE", __FUNCTION__);
    return FALSE;
  }

  if (!pci_get_interrupt (device_index, &net2280_dev.irq_line, &irq_pin)) {
    DLOG ("Unable to get IRQ");
    DLOGV("Exiting %s with FALSE", __FUNCTION__);
    return FALSE;
  }

  DLOG("Using IRQ pin=%X", irq_pin);
  DLOG("bus = %d, slot = %d", net2280_device.bus, net2280_device.slot);
  /* Turning this off for right now since enumeration is broken and it
     doesn't work */
  if (pci_irq_find (net2280_device.bus, net2280_device.slot, irq_pin, &irq)) {
    /* use PCI routing table */
    DLOG ("Found PCI routing entry irq.gsi=0x%x", irq.gsi);
    if (! (pci_irq_map_handler (&irq, net2280_irq_handler, 0x01,
                                IOAPIC_DESTINATION_LOGICAL,
                                IOAPIC_DELIVERY_FIXED)) ) {
      DLOG ("Unable to map IRQ handler");
      return FALSE;
    }
    net2280_dev.irq_line = irq.gsi;
  }
  else {
    DLOG("Failed to find PCI routing entry for net2280 device");
    while(1);
    return FALSE;
  }
  
  if (!pci_decode_bar (device_index, 0, &net_base, NULL, NULL)) {
    DLOG ("unable to decode BAR0");
    return FALSE;
  }

  DLOG("net base from BAR0 = 0x%.04X", net_base);
    
  DLOG("device_index = 0x%X", device_index);

  if(!initialise_net2280(&net2280_dev, net_base, TRUE,
                         net2280_dma_pool,
                         sizeof(net2280_dma_pool)/sizeof(net2280_dma_t),
                         used_net2280_dma_bitmap)) {
    DLOG("Failed to initialise net2280 device");
    return FALSE;
  }

  DLOG("first phase initialised");

  DLOG("starting second phase of initialisation");

#if 0
  if(net2280_start(&net2280_dev.gadget, &default_net2280_usb_gadget_driver) < 0) { 
     DLOG("net2280_start failed"); 
     panic("net2280_start failed"); 
  }
#endif

  return TRUE;
}


#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = net2280_init
};

DEF_MODULE (usb___gadget_net2280, "NET2280 driver", &mod_ops, {"usb", "pci"});



/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
