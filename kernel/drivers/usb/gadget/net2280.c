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
#include <mem/physical.h>
#include <mem/virtual.h>
#include <sched/sched.h>
#include <sched/vcpu.h>
#include <vm/migration.h>


//#define DEBUG_NET2280
//#define DEBUG_NET2280_VERBOSE
//#define DEBUG_NET2280_VERY_VERBOSE



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

#ifdef DEBUG_NET2280_VERY_VERBOSE
#define DLOGVV(fmt,...) DLOG_PREFIX("Net2280",fmt,##__VA_ARGS__)
#else
#define DLOGVV(fmt,...) ;
#endif



#define DEBUG(dev, fmt, ...) DLOGVV(fmt, ##__VA_ARGS__)
#define DBG(dev, fmt, ...) DLOGVV(fmt, ##__VA_ARGS__)
#define WARNING(dev, fmt, ...) DLOGVV(fmt, ##__VA_ARGS__)
#define VDEBUG(dev, fmt, ...) DLOGVV(fmt, ##__VA_ARGS__)
#define VDBG(dev, fmt, ...) DLOGVV(fmt, ##__VA_ARGS__)
#define ERROR(dev, fmt, ...) do {DLOG(fmt, ##__VA_ARGS__); panic(fmt); } while(0)


#ifdef DEBUG_NET2280

#  define dump_msg(net2280, /* const char * */ label,                   \
                 /* const u8 * */ buf, /* unsigned */ length) do {      \
    if (length < 512) {                                                 \
      DBG(net2280, "%s, length %u:\n", label, length);                  \
    }                                                                   \
  } while (0)

#else

#  define dump_msg(net2280, /* const char * */ label,                   \
                   /* const u8 * */ buf, /* unsigned */ length) do { } while (0)
#endif


/*
 * FIFO_MODE 0 == ep-{a,b,c,d} 1K fifo each
 * FIFO_MODE 1 == ep-{a,b} 2K fifo each, ep-{c,d} unavailable
 * FIFO_MODE 2 == ep-a 2K fifo, ep-{b,c} 1K each, ep-d unavailable
 */
#define FIFO_MODE 1

#define CHIPREV_1       0x0100
#define CHIPREV_1A      0x0110

#define DMA_ADDR_INVALID        (~(uint32_t)0)
#define EP_DONTUSE              13      /* nonzero */

/* Big enough to hold our biggest descriptor */
#define EP0_BUFSIZE     256
#define DELAYED_STATUS  (EP0_BUFSIZE + 999)     /* An impossibly large value */

static bool use_dma = 0;
static bool use_dma_chaining = 0;

static int net2280_dev_num = -1;

#define cpu_to_le32s(x) do { (void)(x); } while (0)

#define cpu_to_le32(x) ((__force __le32)(u32)(x))

#define __force        __attribute__((force))
#define valid_bit       cpu_to_le32 (1 << VALID_BIT)
#define le16_to_cpu(x) (x)

void net2280_request_complete_callback(struct usb_ep *ep,
                                       struct usb_request *req);

static const char ep0name [] = "ep0";
static const char *const ep_name [] = {
  ep0name,
  "ep-a", "ep-b", "ep-c", "ep-d",
  "ep-e", "ep-f",
};

static inline void put_unaligned_le16(u16 val, u8 *p)
{
  *p++ = val;
  *p++ = val >> 8;
}


static inline void put_unaligned_le32(u32 val, u8 *p)
{
  put_unaligned_le16(val >> 16, p + 2);
  put_unaligned_le16(val, p);
}

static inline void put_unaligned_le64(u64 val, u8 *p)
{
  put_unaligned_le32(val >> 32, p + 4);
  put_unaligned_le32(val, p);
}


#define put_unaligned(val, ptr) ({                      \
      void *__gu_p = (ptr);                             \
      switch (sizeof(*(ptr))) {                         \
      case 1:                                           \
        *(u8 *)__gu_p = (__force u8)(val);              \
        break;                                          \
      case 2:                                           \
        put_unaligned_le16((__force u16)(val), __gu_p); \
        break;                                          \
      case 4:                                           \
        put_unaligned_le32((__force u32)(val), __gu_p); \
        break;                                          \
      case 8:                                           \
        put_unaligned_le64((__force u64)(val), __gu_p); \
        break;                                          \
      default:                                          \
        bad_unaligned_access_size();                    \
        break;                                          \
      }                                                 \
      (void)0; })

#define dma_done_ie     cpu_to_le32 (1 << DMA_DONE_INTERRUPT_ENABLE)

static const u32 dmactl_default =
  (1 << DMA_SCATTER_GATHER_DONE_INTERRUPT)
  | (1 << DMA_CLEAR_COUNT_ENABLE)
  /* erratum 0116 workaround part 1 (use POLLING) */
  | (POLL_100_USEC << DESCRIPTOR_POLLING_RATE)
  | (1 << DMA_VALID_BIT_POLLING_ENABLE)
  | (1 << DMA_VALID_BIT_ENABLE)
  | (1 << DMA_SCATTER_GATHER_ENABLE)
  /* erratum 0116 workaround part 2 (no AUTOSTART) */
  | (1 << DMA_ENABLE);

/* enable_suspend -- When enabled, the driver will respond to
 * USB suspend requests by powering down the NET2280.  Otherwise,
 * USB suspend requests will be ignored.  This is acceptable for
 * self-powered devices
 */
static bool enable_suspend = 0;


#define NET2280_ELEMENTS_PER_BITMAP_ENTRY (sizeof(uint32_t) * 8)
#define BITMAP_ALL_USED_MASK 0xFFFFFFFF
#define BITMAP_INDEX_SHIFT 5
#define BITMAP_SUBINDEX_MASK (NET2280_ELEMENTS_PER_BITMAP_ENTRY - 1)

#define NET2280_DMA_POOL_SIZE  64

static net2280_dma_t net2280_dma_pool[NET2280_DMA_POOL_SIZE] ALIGNED(0x1000);
static uint32_t used_net2280_dma_bitmap[(NET2280_DMA_POOL_SIZE + NET2280_ELEMENTS_PER_BITMAP_ENTRY - 1)
                                        / NET2280_ELEMENTS_PER_BITMAP_ENTRY];

#define INIT_THREAD_STACK_SIZE 1024
uint32_t init_thread_stack[INIT_THREAD_STACK_SIZE];

#ifdef NET2280_MIGRATION_MODE
#define MIGRATION_THREAD_STACK_SIZE 1024
uint32_t migration_thread_stack[INIT_THREAD_STACK_SIZE];
#endif

enum {
  NET2280_STRING_MANUFACTURER = 1,
  NET2280_STRING_PRODUCT,
  NET2280_STRING_SERIAL,
  NET2280_STRING_CONFIG,
  NET2280_STRING_INTERFACE
};

#define DRIVER_VENDOR_NUM       0xabc4          /* NetChip */


#ifdef NET2280_MIGRATION_MODE
#define DRIVER_PRODUCT_NUM      0xabca          /* Linux-USB "Gadget Zero" */
#else
#define DRIVER_PRODUCT_NUM      0xabc7          /* Linux-USB "Gadget Zero" */
#endif
#define CONFIG_VALUE            1

#define udelay sched_usleep


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
      uint32_t* used_bitmap = net2280->used_##res##_bitmap;             \
        res##_t*  pool        = net2280->res##_pool;                    \
                                                                        \
          while(i--) {                                                  \
            if(used_bitmap[i] == INT_MAX) continue;                     \
                                                                        \
            entry = NET2280_ELEMENTS_PER_BITMAP_ENTRY;                  \
            while(entry--) {                                            \
              if(!(used_bitmap[i] & (1 << entry)) ) {                   \
                used_bitmap[i] |= (1 << entry); /* Mark entry as allocated */ \
                items[count] = &pool[i * NET2280_ELEMENTS_PER_BITMAP_ENTRY + entry]; \
                if(!initialise_##res(net2280, items[count])) {          \
                  free_##res(net2280, items[count]);                    \
                    return count;                                       \
                }                                                       \
                if(++count == num) {                                    \
                  return count;                                         \
                }                                                       \
              }                                                         \
            }                                                           \
          }                                                             \
          return count;                                                 \
  }                                                                     \
                                                                        \
  inline res##_t* allocate_##res(struct net2280* net2280)               \
  {                                                                     \
    res##_t* temp;                                                      \
      return allocate_##res##s(net2280, &temp, 1) ? temp : NULL;        \
  }                                                                     \
                                                                        \
  void free_##res##s(struct net2280* net2280, res##_t** items, uint32_t num) \
  {                                                                     \
    uint32_t* used_bitmap = net2280->used_##res##_bitmap;               \
      res##_t*    pool        = net2280->res##_pool;                    \
                                                                        \
        while(num--) {                                                  \
          uint32_t index = items[num] - pool;                           \
                                                                        \
          /* Flip the one bit to zero */                                \
          used_bitmap[index >> BITMAP_INDEX_SHIFT] &=                   \
            ~(1 << (index & BITMAP_SUBINDEX_MASK));                     \
        }                                                               \
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

#define __NET2280_POOL_PHYS_TO_VIRT(dev, phys_addr, pool)               \
  ((((uint32_t)phys_addr) - ((uint32_t)(dev)->pool ## _phys_addr)) + ((uint32_t)(dev)->pool))

#define __NET2280_POOL_VIRT_TO_PHYS(dev, virt_addr, pool)               \
  ((((uint32_t)virt_addr) - ((uint32_t)(dev)->pool)) + ((uint32_t)(dev)->pool ## _phys_addr))

#define NET2280_DMA_VIRT_TO_PHYS(dev, virt_addr)                        \
  __NET2280_POOL_VIRT_TO_PHYS((dev), (virt_addr), net2280_dma_pool)

#define NET2280_DMA_PHYS_TO_VIRT(dev, phys_addr)                        \
  ((net2280_dma*)__NET2280_POOL_PHYS_TO_VIRT((dev), (phys_addr), net2280_dma_pool))

NET2280_RESOURCE_MEMORY_FUNCS(net2280_dma)

#undef NET2280_RESOURCE_MEMORY_FUNCS

typedef void (*net2280_routine_t)(struct net2280 *);

struct net2280 net2280_dev;

static const char net2280_string_interface[] = "Temp USB Device";

/* Static strings, in UTF-8 (for simplicity we use only ASCII characters) */
static struct usb_string                net2280_strings[] = {
  {NET2280_STRING_INTERFACE,          net2280_string_interface},
  {}
};

static struct usb_gadget_strings        net2280_stringtab = {
  .language       = 0x0409,               /* en-us */
  .strings        = net2280_strings,
};

static struct usb_qualifier_descriptor dev_qualifier = {
  .bLength =              sizeof dev_qualifier,
  .bDescriptorType =      USB_DT_DEVICE_QUALIFIER,
  
  .bcdUSB =               cpu_to_le16(0x0200),
  .bDeviceClass =         USB_CLASS_PER_INTERFACE,
  
  .bNumConfigurations =   1,
};

static struct usb_config_descriptor
config_desc = {
  .bLength =              sizeof config_desc,
  .bDescriptorType =      USB_DT_CONFIG,
  
  /* wTotalLength computed by usb_gadget_config_buf() */
  .bNumInterfaces =       1,
  .bConfigurationValue =  CONFIG_VALUE,
  .iConfiguration =       NET2280_STRING_CONFIG,
  .bmAttributes =         USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
  .bMaxPower =            CONFIG_USB_GADGET_VBUS_DRAW / 2,
};

static struct usb_device_descriptor device_desc = {
  .bLength =              sizeof device_desc,
  .bDescriptorType =      USB_DT_DEVICE,
  
  .bcdUSB =               cpu_to_le16(0x0200),
  .bDeviceClass =         USB_CLASS_VENDOR_SPEC,
  
  .idVendor =             cpu_to_le16(DRIVER_VENDOR_NUM),
  .idProduct =            cpu_to_le16(DRIVER_PRODUCT_NUM),
  .bNumConfigurations =   2,
};

static struct usb_interface_descriptor loopback_intf = {
  .bLength =              sizeof loopback_intf,
  .bDescriptorType =      USB_DT_INTERFACE,
  
  .bNumEndpoints =        2,
  .bInterfaceClass =      USB_CLASS_VENDOR_SPEC,
  /* .iInterface = DYNAMIC */
};

/* full speed support: */



static struct usb_endpoint_descriptor fs_loop_source_desc = {
  .bLength =              USB_DT_ENDPOINT_SIZE,
  .bDescriptorType =      USB_DT_ENDPOINT,
  .wMaxPacketSize =       cpu_to_le16(PACKET_SIZE),
  
  .bEndpointAddress =     USB_DIR_IN | 1,
  .bmAttributes =         NET2280_INTERFACE_TYPE,
  .bInterval = INTERVAL,
};

static struct usb_endpoint_descriptor fs_loop_sink_desc = {
  .bLength =              USB_DT_ENDPOINT_SIZE,
  .bDescriptorType =      USB_DT_ENDPOINT,
  .wMaxPacketSize =       cpu_to_le16(PACKET_SIZE),

  .bEndpointAddress =     USB_DIR_OUT | 1,
  .bmAttributes =         NET2280_INTERFACE_TYPE,
  .bInterval = INTERVAL,
};

static struct usb_descriptor_header *net2280_fs_function[] = {
  (struct usb_descriptor_header *) &loopback_intf,
  (struct usb_descriptor_header *) &fs_loop_sink_desc,
  (struct usb_descriptor_header *) &fs_loop_source_desc,
  NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor hs_out_ept_desc = {
  .bLength =              USB_DT_ENDPOINT_SIZE,
  .bDescriptorType =      USB_DT_ENDPOINT,

  .bEndpointAddress =     USB_DIR_OUT | 1,
  .bmAttributes =         NET2280_INTERFACE_TYPE,
  .wMaxPacketSize =       cpu_to_le16(PACKET_SIZE),
  .bInterval = INTERVAL,
};

static struct usb_endpoint_descriptor hs_in_ept_desc = {
  .bLength =              USB_DT_ENDPOINT_SIZE,
  .bDescriptorType =      USB_DT_ENDPOINT,

  .bEndpointAddress =     USB_DIR_IN | 1,
  .bmAttributes =         NET2280_INTERFACE_TYPE,
  .wMaxPacketSize =       cpu_to_le16(PACKET_SIZE),
  .bInterval = INTERVAL,
};

/* -- EM -- There is a bug right now in copying over the configuration
   descriptor to the host, it keeps missing the first element
   (i.e. the interface descriptor), so the really really ugly hack I
   am doing is putting a dummy value in the first spot */
static struct usb_descriptor_header *net2280_hs_function[] = {
  (struct usb_descriptor_header *) 1,
  (struct usb_descriptor_header *) &loopback_intf,
  (struct usb_descriptor_header *) &hs_out_ept_desc,
  (struct usb_descriptor_header *) &hs_in_ept_desc,
  NULL,
};


static int net2280_write(USB_DEVICE_INFO* device, int dev_num, char* buf, int data_len)
{
#ifdef NET2280_MIGRATION_MODE
  DLOG("Called write in migration mode");
  return -1;
#else
  struct usb_request* req = usb_ep_alloc_request(net2280_dev.in_ep, 0);
  if(req == NULL) {
    /* -- EM -- Not doing all the necessary cleanup */
    DLOG("Failed to allocate request");
    return -1; 
  }
  
  req->buf = kmalloc(req->length = data_len);
  
  if(req->buf == NULL) {
    /* -- EM -- Not doing all the necessary cleanup */
    DLOG("Failed to allocate buffer");
    return -1;
  }

  memcpy(req->buf, buf, data_len);
  req->dma = (dma_addr_t)NULL;
  req->complete = net2280_request_complete_callback;

  if(usb_ep_queue(net2280_dev.in_ep, req, 0) < 0) {
    DLOG("Failed to queue request");
    return -1;
  }

  return data_len;
#endif
}



static void print_table_bitmap(uint32* bitmap)
{
#ifdef DEBUG_NET2280_VERBOSE
  int i;
  DLOGV("\n\n\n");
  for(i = 0; i < 32; i +=4) {
    DLOGV("0x%08X 0x%08X 0x%08X 0x%08X", bitmap[i+3], bitmap[i+2], bitmap[i+1], bitmap[i]);
  }
  DLOGV("\n\n\n");
#endif //DEBUG_NET2280_VERBOSE
}

void net2280_request_complete_callback(struct usb_ep *ep,
                                       struct usb_request *req)
{
  
#ifdef NET2280_MIGRATION_MODE
  static int pages_received = 0;
  int status = req->status;
  int i, j;

  DLOGV("In %s", __FUNCTION__);
  if(status < 0) {
    DLOG("Status is -1 in net2280 migration callback");
    panic("Status is -1 in net2280 migration callback");
    
    return;
  }
  
  if(req == net2280_dev.tss_request) {
    if(req->actual != req->length) {
      DLOG("Didn't get entire tss");
      panic("Didn't get entire tss");
    }
    quest_tss *pTSS = req->buf;
    int i;
#ifdef DEBUG_NET2280
    u64 tsc;
#endif
    pTSS->next_tss = pTSS->prev_tss = NULL;
    pTSS->tid += 200;           /* -- EM -- Hack right now to make
                                   sure the tid is different than the
                                   tid of the original task */
    pTSS->machine_affinity = 0;
    pTSS->CR3 = (u32)net2280_dev.new_cr3;
    
    for(i = 0; i < 1024; ++i) {
      if(net2280_dev.kernel_specific_pages[i]) {
        net2280_dev.mig_task_pd_table[i].raw = net2280_dev.kernel_specific_pages[i];
      }
    }
#ifdef DEBUG_NET2280
    RDTSC(tsc);
    DLOG("Migration end time = 0x%016llX", tsc);
#endif
    attach_task(pTSS, TRUE, 0);
    DLOG("Task migrated");
    return;
  }
  
  if(req == net2280_dev.bitmap_request) {
    /* We got a bitmap request */
    
    uint partial_bitmap = req->actual % (TABLE_BITMAP_SIZE * 4);
    
    DLOGV("Got bitmap request\n\n\n\n\n\n\n\n\n");
    net2280_dev.bitmap_request_in_flight = FALSE;
    net2280_dev.num_bitmaps_received = req->actual / (TABLE_BITMAP_SIZE * 4);

    
    if(partial_bitmap) {
      DLOG("Received a partial bitmap");
      panic("Received a partial bitmap");
    }

    if(net2280_dev.num_bitmaps_received == 0) {
      DLOG("Did not receive any bitmaps");
      panic("Did not receive any bitmaps");
    }
    
    
    if(net2280_dev.migration_in_processes) {
      net2280_dev.current_pt_counter = 0;
    }
    else {
      phys_addr_t phys_addr;
      memset(net2280_dev.frames_per_table, 0, MAX_NUM_BITMAPS);
      net2280_dev.migration_in_processes = TRUE;
      net2280_dev.current_pt_counter = 1;
      /* Copy over the page directory bitmap as we will need it for later */
      memcpy(net2280_dev.pd_bitmap, req->buf, (TABLE_BITMAP_SIZE * 4));
      phys_addr = alloc_phys_frame();
      if(phys_addr == 0xFFFFFFFF) {
        DLOG("Failed to allocate physical frame for page directory");
        panic("Failed to allocate physical frame for page directory");
      }
      net2280_dev.new_cr3 = phys_addr;
      net2280_dev.mig_task_pd_table = map_virtual_page(phys_addr | 3);
      if(net2280_dev.mig_task_pd_table == NULL) {
        DLOG("Failed to map page directory table");
        panic("Failed to map page directory table");
      }
      memset(net2280_dev.mig_task_pd_table, 0, 4096);
      net2280_dev.next_pd_entry = 0;
      net2280_dev.next_pt_entry = 0;
      net2280_dev.next_usb_request = 0;
      net2280_dev.page_requests_in_queue = 0;
      net2280_dev.current_page_table = NULL;
      net2280_dev.all_page_requests_in_queue = FALSE;
    }
    
    memcpy(net2280_dev.current_pt_bitmaps, req->buf,
           TABLE_BITMAP_SIZE * 4 * net2280_dev.num_bitmaps_received);
    
    for(i = net2280_dev.current_pt_counter; i < net2280_dev.num_bitmaps_received; ++i) {
      print_table_bitmap(net2280_dev.current_pt_bitmaps[i]);
      net2280_dev.frames_per_table[i] = 0;
      for(j = 0; j < TABLE_BITMAP_SIZE; ++j) {
        net2280_dev.frames_per_table[i] += popcount(net2280_dev.current_pt_bitmaps[i][j]);
      }
      DLOGV("Frames for table %d = %u", i, net2280_dev.frames_per_table[i]);
    }
  }
  else {
    if(req->actual != PAGE_SIZE) {
      DLOG("Got a short page, req->actual = %u", req->actual);
      panic("Got a short page");
    }
    pages_received++;
    net2280_dev.page_requests_in_queue--;
    DLOGV("Pages received = %d", pages_received);
  }


  DLOGV("net2280_dev.page_requests_in_queue = %d", net2280_dev.page_requests_in_queue);
  while( (net2280_dev.page_requests_in_queue < NET2280_MAX_PAGE_REQUESTS) &&
         (!net2280_dev.all_page_requests_in_queue) ) {
    struct usb_request* req;
    phys_addr_t page_phys;

    DLOGV("net2280_dev.page_requests_in_queue = %d", net2280_dev.page_requests_in_queue);
    
    if(net2280_dev.next_pt_entry == 0 && net2280_dev.next_pd_entry != 1024) {
      /* We need to map a page table onto the page directory */
      
      if(net2280_dev.current_page_table) {
        unmap_virtual_page(net2280_dev.current_page_table);
        DLOGV("unmapping current page table");
        net2280_dev.current_page_table = NULL;
      }

      while((!BITMAP_TST(net2280_dev.pd_bitmap, net2280_dev.next_pd_entry)) &&
            (net2280_dev.next_pd_entry < 1024)) {
        net2280_dev.next_pd_entry++;
      }
      DLOG("At %d net2280_dev.next_pd_entry = %u", __LINE__, net2280_dev.next_pd_entry);
      if(net2280_dev.next_pd_entry == 1024) {
        net2280_dev.all_page_requests_in_queue = TRUE;
        if(usb_ep_queue(net2280_dev.out_ep, net2280_dev.tss_request, 0) < 0) {
          DLOG("Failed to queue tss request");
          panic("Failed to queue tss request");
        }
        break;
      }
      else {
        
        if(net2280_dev.current_pt_counter == net2280_dev.num_bitmaps_received) {
          if(!net2280_dev.bitmap_request_in_flight) {
            if(usb_ep_queue(net2280_dev.out_ep, net2280_dev.bitmap_request, 0) < 0) {
              DLOG("Failed to queue bitmap request");
              panic("Failed to queue bitmap request");
            }
            net2280_dev.bitmap_request_in_flight = TRUE;
          }
          break;
        }
        
        phys_addr_t table_phys = alloc_phys_frame();
        if(table_phys == 0xFFFFFFFF) {
          DLOG("Failed to allocate page table for migration");
          panic("Failed to allocate page table for migration");
        }
        
        net2280_dev.current_page_table = map_virtual_page(table_phys | 3);
        if(net2280_dev.current_page_table == NULL) {
          DLOG("Failed to map page table");
          panic("Failed to map page table");
        }
        memset(net2280_dev.current_page_table, 0, 4096);

        net2280_dev.kernel_only_area = net2280_dev.next_pd_entry == PGDIR_KERNEL_STACK;

        net2280_dev.mig_task_pd_table[net2280_dev.next_pd_entry].raw =
          (0xFFFFF000 & table_phys) | (net2280_dev.kernel_only_area ? 3 : 7);
        
        net2280_dev.next_pd_entry++;
      }
    }
    
    while((!BITMAP_TST(net2280_dev.current_pt_bitmaps[net2280_dev.current_pt_counter],
                       net2280_dev.next_pt_entry)) &&
          (net2280_dev.next_pt_entry < 1024)) {
      net2280_dev.next_pt_entry++;
    }
    
    if(net2280_dev.next_pt_entry == 1024) {
      /* Reached the end of the current table, loop back to set new
         table up */
      net2280_dev.next_pt_entry = 0;
      net2280_dev.current_pt_counter++;
      continue;
    }

    req = net2280_dev.page_requests[net2280_dev.next_usb_request++];
    if(net2280_dev.next_usb_request == NET2280_MAX_PAGE_REQUESTS) {
      net2280_dev.next_usb_request = 0;
    }
    page_phys = alloc_phys_frame();
    if(page_phys == 0xFFFFFFFF) {
      DLOG("Failed to allocate frame for migration");
      panic("Failed to allocate frame for migration");
    }

    if(req->buf) {
      unmap_virtual_page(req->buf);
      req->buf = 0;
    }

    if(net2280_dev.next_pt_entry >= 1024) {
      DLOG("net2280_dev.next_pt_entry = %d out of range", net2280_dev.next_pt_entry);
      panic("net2280_dev.next_pt_entry out of range");
    }

    DLOGV("net2280_dev.next_pt_entry = %u", net2280_dev.next_pt_entry);
    net2280_dev.current_page_table[net2280_dev.next_pt_entry].raw =
      (0xFFFFF000 & page_phys) | (net2280_dev.kernel_only_area ? 3 : 7);
    
    req->buf = map_virtual_page(page_phys | 3);
    if(req->buf == NULL) {
      DLOG("Failed to map virtual page");
      panic("Failed to map virtual page");
    }

    if(usb_ep_queue(net2280_dev.out_ep, req, 0) < 0) {
      DLOG("Failed to queue request for page");
      panic("Failed to queue request for page");
    }
    
    net2280_dev.page_requests_in_queue++;
    net2280_dev.next_pt_entry++;
  }
  

  return;
    
#else
  int status = req->status;

  if(status < 0) {
    DLOG("status < 0 in %s", __FUNCTION__);
    panic("status < 0 in  net2280_request_complete_callback");
    return;
  }

  if(ep == net2280_dev.out_ep) {
    DLOG("In %s for out ep", __FUNCTION__);
    net2280_dev.out_requests_with_data[net2280_dev.next_out_request_insert_index++] = req;
    if(net2280_dev.next_out_request_insert_index == NET2280_NUM_OUT_REQS) {
      net2280_dev.next_out_request_insert_index = 0;
    }
  }
  else {
    DLOG("In %s for in ep");
    kfree(req->buf);
    usb_ep_free_request(ep, req);
  }
#endif
}

static int net2280_open(USB_DEVICE_INFO* device, int dev_num)
{
#ifdef NET2280_MIGRATION_MODE
  
  int i;
  list_head_t* current_ep_list = &net2280_dev.gadget.ep_list;
  struct usb_request* req;
  net2280_dev.migration_in_processes = FALSE;
  net2280_dev.out_ep = list_entry(current_ep_list->next, struct usb_ep, ep_list);
  current_ep_list = current_ep_list->next;
  net2280_dev.in_ep = list_entry(current_ep_list->next, struct usb_ep, ep_list);
  net2280_dev.in_ep->driver_data = &net2280_dev;
  net2280_dev.in_ep->desc = &hs_in_ept_desc;

  if(usb_ep_enable(net2280_dev.in_ep) < 0) {
    DLOG("Failed to enable in endpoint");
    return -1;
  }
  
  net2280_dev.out_ep->driver_data = &net2280_dev;
  net2280_dev.out_ep->desc = &hs_out_ept_desc;
  if(usb_ep_enable(net2280_dev.out_ep) < 0) {
    DLOG("Failed to enable out endpoint");
    return -1;
  }

  for(i = 0; i < NET2280_MAX_PAGE_REQUESTS; ++i) {
    req = net2280_dev.page_requests[i]
      = usb_ep_alloc_request(net2280_dev.out_ep, 0);
    if(req == NULL) {
      /* -- EM -- Not doing all the necessary cleanup */
      DLOG("Failed to allocate request");
      return -1; 
    }
    req->length = PAGE_SIZE;
    
    req->buf = NULL;
    req->dma = (dma_addr_t)NULL;
    
    req->complete = net2280_request_complete_callback;
  }

  
  req = net2280_dev.bitmap_request = usb_ep_alloc_request(net2280_dev.out_ep, 0);
  if(req == NULL) {
    /* -- EM -- Not doing all the necessary cleanup */
    DLOG("Failed to allocate request");
    return -1; 
  }
  
  req->buf = kmalloc(req->length = hs_out_ept_desc.wMaxPacketSize);
  
  if(req->buf == NULL) {
    /* -- EM -- Not doing all the necessary cleanup */
    DLOG("Failed to allocate buffer");
    panic("Failed to allocate buffer");
    return -1;
  }
  req->dma = (dma_addr_t)NULL;
  
  req->complete = net2280_request_complete_callback;
  
  if(usb_ep_queue(net2280_dev.out_ep, req, 0) < 0) {
    DLOG("Failed to queue request");
    panic("Failed to queue request");
    
    return -1;
  }

  req = net2280_dev.tss_request = usb_ep_alloc_request(net2280_dev.out_ep, 0);
  if(req == NULL) {
    /* -- EM -- Not doing all the necessary cleanup */
    DLOG("Failed to allocate request");
    return -1; 
  }
  
  req->buf = kmalloc(req->length = sizeof(quest_tss));
  
  if(req->buf == NULL) {
    /* -- EM -- Not doing all the necessary cleanup */
    DLOG("Failed to allocate buffer");
    panic("Failed to allocate buffer");
    return -1;
  }
  req->dma = (dma_addr_t)NULL;
  
  req->complete = net2280_request_complete_callback;
  
    
  return 0;
  
#else
  int i;
  list_head_t* current_ep_list = &net2280_dev.gadget.ep_list;
  net2280_dev.out_ep = list_entry(current_ep_list->next, struct usb_ep, ep_list);
  current_ep_list = current_ep_list->next;
  net2280_dev.in_ep = list_entry(current_ep_list->next, struct usb_ep, ep_list);
  net2280_dev.in_ep->driver_data = &net2280_dev;
  net2280_dev.in_ep->desc = &hs_in_ept_desc;

  if(usb_ep_enable(net2280_dev.in_ep) < 0) {
    DLOG("Failed to enable in endpoint");
    return -1;
  }
  
  memset(net2280_dev.out_requests_with_data, 0, sizeof(struct usb_request*) * NET2280_NUM_OUT_REQS);
  net2280_dev.next_out_request_to_read = 0;
  net2280_dev.next_out_request_insert_index = 0;
  net2280_dev.out_ep->driver_data = &net2280_dev;
  net2280_dev.out_ep->desc = &hs_out_ept_desc;
  if(usb_ep_enable(net2280_dev.out_ep) < 0) {
    DLOG("Failed to enable out endpoint");
    return -1;
  }
  

  for(i = 0; i < NET2280_NUM_OUT_REQS; ++i) {
    struct usb_request* req = net2280_dev.out_requests[i]
      = usb_ep_alloc_request(net2280_dev.out_ep, 0);
    if(req == NULL) {
      /* -- EM -- Not doing all the necessary cleanup */
      DLOG("Failed to allocate request");
      return -1; 
    }
    req->buf = kmalloc(req->length = (hs_out_ept_desc.wMaxPacketSize*2));
    
    if(req->buf == NULL) {
      /* -- EM -- Not doing all the necessary cleanup */
      DLOG("Failed to allocate buffer");
      return -1;
    }
    req->dma = (dma_addr_t)NULL;
    
    
    req->complete = net2280_request_complete_callback;

    if(usb_ep_queue(net2280_dev.out_ep, req, 0) < 0) {
      DLOG("Failed to queue request");
      return -1;
    }
  }

  return 0;
#endif
}

static int net2280_read(USB_DEVICE_INFO* device, int dev_num, char* buf, int data_len)
{
#ifdef NET2280_MIGRATION_MODE
  DLOG("Called read in migration mode");
  return -1;
#else
  int data_returned = 0;
  while(data_returned < data_len) {
    struct usb_request* req = net2280_dev.out_requests_with_data[net2280_dev.next_out_request_to_read];
    
    if(req != NULL) {
      if(req->actual > (data_len - data_returned)) {
        break;
      }
      DLOG("Memcpying %d bytes of data from request at index %d",
           req->actual, net2280_dev.next_out_request_to_read);
      memcpy(&buf[data_returned], req->buf, req->actual);
      data_returned += req->actual;
      if(usb_ep_queue(net2280_dev.out_ep, req, 0) < 0) {
        DLOG("Failed to queue request");
        panic("Failed to queue request");
        return -1;
      }
      DLOG("Returned from ep queue");
      net2280_dev.out_requests_with_data[net2280_dev.next_out_request_to_read] = NULL;
      net2280_dev.next_out_request_to_read++;
      if(net2280_dev.next_out_request_to_read == NET2280_NUM_OUT_REQS) {
        net2280_dev.next_out_request_to_read = 0;
      }
      DLOG("net2280_dev.next_out_request_to_read = %d", net2280_dev.next_out_request_to_read);
    }
    else {
      break;
    }
  }
  
  return data_returned;
#endif
}

static USB_DRIVER net2280_driver = {
  .open = net2280_open,
  .read = net2280_read,
  .write = net2280_write
};


static void handle_ep_small (struct net2280_ep *ep);

static inline void set_fifo_bytecount (struct net2280_ep *ep, unsigned count);

static inline void
dma_done (struct net2280_ep *ep, struct net2280_request *req,
          u32 dmacount, int status);

static void usb_reset(struct net2280* dev);

static void usb_reinit(struct net2280 *dev);

static void start_dma (struct net2280_ep *ep, struct net2280_request *req);

static void nuke (struct net2280_ep *ep);

static void done (struct net2280_ep *ep, struct net2280_request *req, int status);

static void abort_dma (struct net2280_ep *ep);

static inline void prefetch(void *a __attribute__((unused))) { }

static inline u32 get_unaligned_le32(const u8 *p)
{
  return p[0] | p[1] << 8 | p[2] << 16 | p[3] << 24;
}

static inline u64 get_unaligned_le64(const u8 *p)
{
  return (u64)get_unaligned_le32(p + 4) << 32 | get_unaligned_le32(p);
}

static inline u16 get_unaligned_le16(const u8 *p)
{
  return p[0] | p[1] << 8;
}


#define bad_unaligned_access_size() panic("Bad unaligned access size")

#define get_unaligned(ptr) ((__force typeof(*(ptr)))({                             \
        __builtin_choose_expr(sizeof(*(ptr)) == 1, *(ptr),                         \
        __builtin_choose_expr(sizeof(*(ptr)) == 2, get_unaligned_le16((u8*)(ptr)), \
        __builtin_choose_expr(sizeof(*(ptr)) == 4, get_unaligned_le32((u8*)(ptr)), \
        __builtin_choose_expr(sizeof(*(ptr)) == 8, get_unaligned_le64((u8*)(ptr)), \
       bad_unaligned_access_size()))));                                            \
      }))

static inline void writeb(u8 b, volatile void *addr)
{
  *(volatile u8 __force *) addr = b;
}

inline void writel(u32 b, volatile void *addr) {
  *(volatile u32 *) addr = b;
}

static void rmb ()
{
  gccmb();
}

static void wmb()
{
  gccmb();
}

inline u32 readl(const volatile void *addr)
{
  u32 ret = *(const volatile u32 *)(addr);
  return ret;
}


#ifdef DEBUG_NET2280
static inline void assert_out_naking (struct net2280_ep *ep, const char *where)
{
  u32     tmp = readl (&ep->regs->ep_stat);
  
  if ((tmp & (1 << NAK_OUT_PACKETS)) == 0) {
    DEBUG (ep->dev, "%s %s %08x !NAK\n",
           ep->ep.name, where, tmp);
    writel ((1 << SET_NAK_OUT_PACKETS),
            &ep->regs->ep_rsp);
  }
}
#define ASSERT_OUT_NAKING(ep) assert_out_naking(ep,__func__)
#else
#define ASSERT_OUT_NAKING(ep) do {} while (0)
#endif

#define min(x,y) ( x < y ? x : y)


/* work around erratum 0106: PCI and USB race over the OUT fifo.
 * caller guarantees chiprev 0100, out endpoint is NAKing, and
 * there's no real data in the fifo.
 *
 * NOTE:  also used in cases where that erratum doesn't apply:
 * where the host wrote "too much" data to us.
 */
static void out_flush (struct net2280_ep *ep)
{
  u32     *statp;
  u32     tmp;

  ASSERT_OUT_NAKING (ep);
  
  statp = &ep->regs->ep_stat;
  writel (  (1 << DATA_OUT_PING_TOKEN_INTERRUPT)
            | (1 << DATA_PACKET_RECEIVED_INTERRUPT)
            , statp);
  writel ((1 << FIFO_FLUSH), statp);
  gccmb();
  tmp = readl (statp);
  if (tmp & (1 << DATA_OUT_PING_TOKEN_INTERRUPT)
      /* high speed did bulk NYET; fifo isn't filling */
      && ep->dev->gadget.speed == USB_SPEED_FULL) {
    unsigned        usec;

    usec = 50;              /* 64 byte bulk/interrupt */
    handshake (statp, (1 << USB_OUT_PING_NAK_SENT),
               (1 << USB_OUT_PING_NAK_SENT), usec);
    /* NAK done; now CLEAR_NAK_OUT_PACKETS is safe */
  }
}

/*
 * Config descriptors must agree with the code that sets
 * configurations and with code managing interfaces and their
 * altsettings.  They must also handle different speeds and
 * other-speed requests.
 */
static int populate_config_buf(struct usb_gadget *gadget,
                               u8 *buf, u8 type, unsigned index)
{
  uint                   speed = gadget->speed;
  int                                     len;
  const struct usb_descriptor_header      **function;

  DLOGV("index = %u", index);
  //if (index > 0) 
  //  return -1;

  if (gadget_is_dualspeed(gadget) && type == USB_DT_OTHER_SPEED_CONFIG)
    speed = (USB_SPEED_FULL + USB_SPEED_HIGH) - speed;
  function = gadget_is_dualspeed(gadget) && speed == USB_SPEED_HIGH
    ? (const struct usb_descriptor_header **)net2280_hs_function
    : (const struct usb_descriptor_header **)net2280_fs_function;

  /* for now, don't advertise srp-only devices */
  if (!gadget_is_otg(gadget))
    function++;

  len = usb_gadget_config_buf(&config_desc, buf, EP0_BUFSIZE, function);
  ((struct usb_config_descriptor *) buf)->bDescriptorType = type;
  return len;
}

static inline void start_out_naking (struct net2280_ep *ep)
{
  /* NOTE:  hardware races lurk here, and PING protocol issues */
  writel ((1 << SET_NAK_OUT_PACKETS), &ep->regs->ep_rsp);
  /* synch with device */
  readl (&ep->regs->ep_rsp);
}

/* unload packet(s) from the fifo we use for usb OUT transfers.
 * returns true iff the request completed, because of short packet
 * or the request buffer having filled with full packets.
 *
 * for ep-a..ep-d this will read multiple packets out when they
 * have been accepted.
 */
static int
read_fifo (struct net2280_ep *ep, struct net2280_request *req)
{
  struct net2280_ep_regs *regs = ep->regs;
  u8                      *buf = req->req.buf + req->req.actual;
  unsigned                count, tmp, is_short;
  unsigned                cleanup = 0, prevent = 0;
  
  /* erratum 0106 ... packets coming in during fifo reads might
   * be incompletely rejected.  not all cases have workarounds.
   */
  if (ep->dev->chiprev == 0x0100
      && ep->dev->gadget.speed == USB_SPEED_FULL) {
    udelay (1);
    tmp = readl (&ep->regs->ep_stat);
    if ((tmp & (1 << NAK_OUT_PACKETS)))
      cleanup = 1;
    else if ((tmp & (1 << FIFO_FULL))) {
      start_out_naking (ep);
      prevent = 1;
    }
    /* else: hope we don't see the problem */
  }

  /* never overflow the rx buffer. the fifo reads packets until
   * it sees a short one; we might not be ready for them all.
   */
  //prefetchw (buf);
  count = readl (&regs->ep_avail);
  if (unlikely (count == 0)) {
    udelay (1);
    tmp = readl (&ep->regs->ep_stat);
    count = readl (&regs->ep_avail);
    /* handled that data already? */
    if (count == 0 && (tmp & (1 << NAK_OUT_PACKETS)) == 0)
      return 0;
  }

  tmp = req->req.length - req->req.actual;
  if (count > tmp) {
    /* as with DMA, data overflow gets flushed */
    if ((tmp % ep->ep.maxpacket) != 0) {
      ERROR (ep->dev,
             "%s out fifo %d bytes, expected %d\n",
             ep->ep.name, count, tmp);
      req->req.status = -1;
      cleanup = 1;
      /* NAK_OUT_PACKETS will be set, so flushing is safe;
       * the next read will start with the next packet
       */
    } /* else it's a ZLP, no worries */
    count = tmp;
  }
  req->req.actual += count;

  is_short = (count == 0) || ((count % ep->ep.maxpacket) != 0);

  VDEBUG (ep->dev, "read %s fifo (OUT) %d bytes%s%s%s req %p %d/%d\n",
          ep->ep.name, count, is_short ? " (short)" : "",
          cleanup ? " flush" : "", prevent ? " nak" : "",
          req, req->req.actual, req->req.length);

  while (count >= 4) {
    tmp = readl (&regs->ep_data);
    cpu_to_le32s (&tmp);
    put_unaligned (tmp, (u32 *)buf);
    buf += 4;
    count -= 4;
  }
  if (count) {
    tmp = readl (&regs->ep_data);
    /* LE conversion is implicit here: */
    do {
      *buf++ = (u8) tmp;
      tmp >>= 8;
    } while (--count);
  }
  if (cleanup)
    out_flush (ep);
  if (prevent) {
    writel ((1 << CLEAR_NAK_OUT_PACKETS), &ep->regs->ep_rsp);
    (void) readl (&ep->regs->ep_rsp);
  }

  return is_short || ((req->req.actual == req->req.length)
                      && !req->req.zero);
}

static inline void set_halt (struct net2280_ep *ep)
{
  /* ep0 and bulk/intr endpoints */
  writel (  (1 << CLEAR_CONTROL_STATUS_PHASE_HANDSHAKE)
            /* set NAK_OUT for erratum 0114 */
            | ((ep->dev->chiprev == CHIPREV_1) << SET_NAK_OUT_PACKETS)
            | (1 << SET_ENDPOINT_HALT)
            , &ep->regs->ep_rsp);
}

static inline void allow_status (struct net2280_ep *ep)
{
  /* ep0 only */
  writel (  (1 << CLEAR_CONTROL_STATUS_PHASE_HANDSHAKE)
            | (1 << CLEAR_NAK_OUT_PACKETS)
            | (1 << CLEAR_NAK_OUT_PACKETS_MODE)
            , &ep->regs->ep_rsp);
  ep->stopped = 1;
}

static void ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
  struct net2280          *net2280 = ep->driver_data;
  
  if (req->actual > 0)
    dump_msg(net2280, net2280->ep0req_name, req->buf, req->actual);
  if (req->status || req->actual != req->length)
    DBG(net2280, "%s --> %d, %u/%u\n", __func__,
        req->status, req->actual, req->length);
  if (req->status == -1)         // Request was cancelled
    usb_ep_fifo_flush(ep);
  
  if (req->status == 0 && req->context)
    ((net2280_routine_t) (req->context))(net2280);
}

static inline void clear_halt (struct net2280_ep *ep)
{
  /* ep0 and bulk/intr endpoints */
  writel (  (1 << CLEAR_ENDPOINT_HALT)
            | (1 << CLEAR_ENDPOINT_TOGGLE)
            /* unless the gadget driver left a short packet in the
             * fifo, this reverses the erratum 0114 workaround.
             */
            | ((ep->dev->chiprev == CHIPREV_1) << CLEAR_NAK_OUT_PACKETS)
            , &ep->regs->ep_rsp);
}

static void
stop_activity (struct net2280 *dev, struct usb_gadget_driver *driver);

static struct net2280_ep*
get_ep_by_addr (struct net2280 *dev, u16 wIndex)
{
  struct net2280_ep       *ep;
  
  if ((wIndex & USB_ENDPOINT_NUMBER_MASK) == 0)
    return &dev->ep [0];
  list_for_each_entry (ep, &dev->gadget.ep_list, ep.ep_list) {
    u8      bEndpointAddress;

    if (!ep->desc)
      continue;
    bEndpointAddress = ep->desc->bEndpointAddress;
    if ((wIndex ^ bEndpointAddress) & USB_DIR_IN)
      continue;
    if ((wIndex & 0x0f) == (bEndpointAddress & 0x0f))
      return ep;
  }
  return NULL;
}

/* dequeue ALL requests */
static void nuke (struct net2280_ep *ep)
{
  struct net2280_request  *req;
  
  /* called with spinlock held */
  ep->stopped = 1;
  if (ep->dma)
    abort_dma (ep);
  while (!list_empty (&ep->queue)) {
    req = list_entry (ep->queue.next,
                      struct net2280_request,
                      queue);
    done (ep, req, -1);
  }
}

/* load a packet into the fifo we use for usb IN transfers.
 * works for all endpoints.
 *
 * NOTE: pio with ep-a..ep-d could stuff multiple packets into the fifo
 * at a time, but this code is simpler because it knows it only writes
 * one packet.  ep-a..ep-d should use dma instead.
 */
static void
write_fifo (struct net2280_ep *ep, struct usb_request *req)
{
  struct net2280_ep_regs   *regs = ep->regs;
  u8                      *buf;
  u32                     tmp;
  unsigned                count, total;

  /* INVARIANT:  fifo is currently empty. (testable) */

  if (req) {
    buf = req->buf + req->actual;
    prefetch (buf);
    total = req->length - req->actual;
  } else {
    total = 0;
    buf = NULL;
  }

  /* write just one packet at a time */
  count = ep->ep.maxpacket;
  if (count > total)      /* min() cannot be used on a bitfield */
    count = total;

  VDEBUG (ep->dev, "write %s fifo (IN) %d bytes%s req %p\n",
          ep->ep.name, count,
          (count != ep->ep.maxpacket) ? " (short)" : "",
          req);
  while (count >= 4) {
    /* NOTE be careful if you try to align these. fifo lines
     * should normally be full (4 bytes) and successive partial
     * lines are ok only in certain cases.
     */
    tmp = get_unaligned ((u32 *)buf);
    cpu_to_le32s (&tmp);
    writel (tmp, &regs->ep_data);
    buf += 4;
    count -= 4;
  }

  /* last fifo entry is "short" unless we wrote a full packet.
   * also explicitly validate last word in (periodic) transfers
   * when maxpacket is not a multiple of 4 bytes.
   */
  if (count || total < ep->ep.maxpacket) {
    tmp = count ? get_unaligned ((u32 *)buf) : count;
    cpu_to_le32s (&tmp);
    set_fifo_bytecount (ep, count & 0x03);
    writel (tmp, &regs->ep_data);
  }

  /* pci writes may still be posted */
}

/* fill out dma descriptor to match a given request */
static void
fill_dma_desc (struct net2280_ep *ep, struct net2280_request *req, int valid)
{
  struct net2280_dma      *td = req->td;
  u32                     dmacount = req->req.length;

  /* don't let DMA continue after a short OUT packet,
   * so overruns can't affect the next transfer.
   * in case of overruns on max-size packets, we can't
   * stop the fifo from filling but we can flush it.
   */
  if (ep->is_in)
    dmacount |= (1 << DMA_DIRECTION);
  if ((!ep->is_in && (dmacount % ep->ep.maxpacket) != 0)
      || ep->dev->is_2280)
    dmacount |= (1 << END_OF_CHAIN);

  req->valid = valid;
  if (valid)
    dmacount |= (1 << VALID_BIT);
  if (likely(!req->req.no_interrupt || !use_dma_chaining))
    dmacount |= (1 << DMA_DONE_INTERRUPT_ENABLE);

  /* td->dmadesc = previously set by caller */
  td->dmaaddr = cpu_to_le32 (req->req.dma);

  /* 2280 may be polling VALID_BIT through ep->dma->dmadesc */
  wmb ();
  td->dmacount = cpu_to_le32(dmacount);
}

static inline void stop_out_naking (struct net2280_ep *ep)
{
  u32     tmp;
  
  tmp = readl (&ep->regs->ep_stat);
  if ((tmp & (1 << NAK_OUT_PACKETS)) != 0)
    writel ((1 << CLEAR_NAK_OUT_PACKETS), &ep->regs->ep_rsp);
}

static void start_queue (struct net2280_ep *ep, u32 dmactl, u32 td_dma)
{
  struct net2280_dma_regs *dma = ep->dma;
  unsigned int tmp = (1 << VALID_BIT) | (ep->is_in << DMA_DIRECTION);

  if (ep->dev->is_2280)
    tmp |= (1 << END_OF_CHAIN);

  writel (tmp, &dma->dmacount);
  writel (readl (&dma->dmastat), &dma->dmastat);

  writel (td_dma, &dma->dmadesc);
  writel (dmactl, &dma->dmactl);

  /* erratum 0116 workaround part 3:  pci arbiter away from net2280 */
  (void) readl (&ep->dev->pci->pcimstctl);

  writel ((1 << DMA_START), &dma->dmastat);

  if (!ep->is_in)
    stop_out_naking (ep);
}

static void raise_exception(struct net2280* net2280, enum net2280_state new_state)
{
  DLOG("IN %s which is not implemented", __FUNCTION__);
  panic("In unimplemented net2280.c raise_exception");
}

static inline void set_fifo_bytecount (struct net2280_ep *ep, unsigned count)
{
  writeb (count, 2 + (u8 *) &ep->regs->ep_cfg);
}


/* Ep0 standard request handlers.  These always run in_irq. */

static int standard_setup_req(struct net2280 *net2280,
                              const struct usb_ctrlrequest *ctrl)
{
  struct usb_request      *req = net2280->ep0req;
  int                     value = -1;
  u16                     w_index = le16_to_cpu(ctrl->wIndex);
  u16                     w_value = le16_to_cpu(ctrl->wValue);

  /* Usually this just stores reply data in the pre-allocated ep0 buffer,
   * but config change events will also reconfigure hardware. */
  switch (ctrl->bRequest) {

  case USB_REQ_GET_DESCRIPTOR:
    if (ctrl->bmRequestType != (USB_DIR_IN | USB_TYPE_STANDARD |
                                USB_RECIP_DEVICE))
      break;
    switch (w_value >> 8) {

    case USB_DT_DEVICE:
      VDBG(net2280, "get device descriptor\n");
      device_desc.bMaxPacketSize0 = net2280->ep0->maxpacket;
      value = sizeof device_desc;
      memcpy(req->buf, &device_desc, value);
      break;
    case USB_DT_DEVICE_QUALIFIER:
      VDBG(net2280, "get device qualifier\n");
      if (!gadget_is_dualspeed(&net2280->gadget) ||
          net2280->gadget.speed == USB_SPEED_SUPER)
        break;
      /*
       * Assume ep0 uses the same maxpacket value for both
       * speeds
       */
      dev_qualifier.bMaxPacketSize0 = net2280->ep0->maxpacket;
      value = sizeof dev_qualifier;
      memcpy(req->buf, &dev_qualifier, value);
      break;

    case USB_DT_OTHER_SPEED_CONFIG:
      VDBG(net2280, "get other-speed config descriptor\n");
      if (!gadget_is_dualspeed(&net2280->gadget) ||
          net2280->gadget.speed == USB_SPEED_SUPER)
        break;
      goto get_config;
    case USB_DT_CONFIG:
      VDBG(net2280, "get configuration descriptor\n");
    get_config:
      value = populate_config_buf(&net2280->gadget,
                                  req->buf,
                                  w_value >> 8,
                                  w_value & 0xff);
      DLOGV("value = %d at %d", value, __LINE__);
      break;

    case USB_DT_STRING:
      VDBG(net2280, "get string descriptor\n");

      /* wIndex == language code */
      value = usb_gadget_get_string(&net2280_stringtab,
                                    w_value & 0xff, req->buf);
      break;

    case USB_DT_BOS:
      VDBG(net2280, "get bos descriptor\n");
      panic("Unsupported get request for bos");
    }

    break;

    /* One config, two speeds */
  case USB_REQ_SET_CONFIGURATION:
    if (ctrl->bmRequestType != (USB_DIR_OUT | USB_TYPE_STANDARD |
                                USB_RECIP_DEVICE))
      break;
    VDBG(net2280, "set configuration\n");
    if (w_value == CONFIG_VALUE || w_value == 0) {
      net2280->new_config = w_value;

      /* Raise an exception to wipe out previous transaction
       * state (queued bufs, etc) and set the new config. */
      
      /* -- EM -- We are going to be nice to the net2280 and never
         call set configuration except the first time during
         initialization, we normally we reset some state that I don't
         want to both with right now so this is just a dummy call */
      
      //raise_exception(net2280, NET2280_STATE_CONFIG_CHANGE);
      //value = DELAYED_STATUS;
      value = 1;
    }
    break;
  case USB_REQ_GET_CONFIGURATION:
    if (ctrl->bmRequestType != (USB_DIR_IN | USB_TYPE_STANDARD |
                                USB_RECIP_DEVICE))
      break;
    VDBG(net2280, "get configuration\n");
    *(u8 *) req->buf = net2280->config;
    value = 1;
    break;

  case USB_REQ_SET_INTERFACE:
    if (ctrl->bmRequestType != (USB_DIR_OUT| USB_TYPE_STANDARD |
                                USB_RECIP_INTERFACE))
      break;
    if (net2280->config && w_index == 0) {

      /* Raise an exception to wipe out previous transaction
       * state (queued bufs, etc) and install the new
       * interface altsetting. */
      raise_exception(net2280, NET2280_STATE_INTERFACE_CHANGE);
      value = DELAYED_STATUS;
    }
    break;
  case USB_REQ_GET_INTERFACE:
    if (ctrl->bmRequestType != (USB_DIR_IN | USB_TYPE_STANDARD |
                                USB_RECIP_INTERFACE))
      break;
    if (!net2280->config)
      break;
    if (w_index != 0) {
      value = -1;
      break;
    }
    VDBG(net2280, "get interface\n");
    *(u8 *) req->buf = 0;
    value = 1;
    break;

  default:
    VDBG(net2280,
         "unknown control req %02x.%02x v%04x i%04x l%u\n",
         ctrl->bmRequestType, ctrl->bRequest,
         w_value, w_index, le16_to_cpu(ctrl->wLength));
  }

  return value;
}

static int ep0_queue(struct net2280 *net2280)
{
  int     rc;
  
  rc = usb_ep_queue(net2280->ep0, net2280->ep0req, 0);
  if (rc != 0 && rc != -1) {
    
    /* We can't do much more than wait for a reset */
    WARNING(net2280, "error in submission: %s --> %d\n",
            net2280->ep0->name, rc);
  }
  return rc;
}

/* Based on the setup callback in Linux's
   drivers/usb/gadget/file_storage.c */

int default_net2280_usb_gadget_setup(struct usb_gadget* gadget,
                                     const struct usb_dev_req* ctrl)
{
  struct net2280          *net2280 = &net2280_dev;
  int                     rc;
  int                     w_length = le16_to_cpu(ctrl->wLength);

  ++net2280->ep0_req_tag;             // Record arrival of a new request
  net2280->ep0req->context = NULL;
  net2280->ep0req->length = 0;
  dump_msg(net2280, "ep0-setup", (u8 *) ctrl, sizeof(*ctrl));

  if ((ctrl->bmRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS) {
    /* -- EM -- Fix this later */
    DLOG("Shouldn't get class specific requests for net2880");
    panic("Shouldn't get class specific requests for net2880");
  }
  else {
    rc = standard_setup_req(net2280, ctrl);
    DLOGV("rc = %d at line %d", rc, __LINE__);
  }
  
  /* Respond with data/status or defer until later? */
  if (rc >= 0 && rc != DELAYED_STATUS) {
    rc = min(rc, w_length);
    net2280->ep0req->length = rc;
    net2280->ep0req->zero = rc < w_length;
    net2280->ep0req_name = (ctrl->bmRequestType & USB_DIR_IN ?
                            "ep0-in" : "ep0-out");
    rc = ep0_queue(net2280);
    DLOGV("rc = %d at line %d", rc, __LINE__);
  }

  /* Device either stalls (rc < 0) or reports success */
  return rc;
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

static void scan_dma_completions (struct net2280_ep *ep);
static void ep0_start (struct net2280 *dev);

static inline void spin_stop_dma (struct net2280_dma_regs *dma)
{
  handshake (&dma->dmactl, (1 << DMA_ENABLE), 0, 50);
}

static inline void stop_dma (struct net2280_dma_regs *dma)
{
  writel (readl (&dma->dmactl) & ~(1 << DMA_ENABLE), &dma->dmactl);
  spin_stop_dma (dma);
}

static void restart_dma (struct net2280_ep *ep)
{
  struct net2280_request  *req;
  u32                     dmactl = dmactl_default;

  if (ep->stopped)
    return;
  req = list_entry (ep->queue.next, struct net2280_request, queue);

  if (!use_dma_chaining) {
    start_dma (ep, req);
    return;
  }

  /* the 2280 will be processing the queue unless queue hiccups after
   * the previous transfer:
   *  IN:   wanted automagic zlp, head doesn't (or vice versa)
   *        DMA_FIFO_VALIDATE doesn't init from dma descriptors.
   *  OUT:  was "usb-short", we must restart.
   */
  if (ep->is_in && !req->valid) {
    struct net2280_request  *entry, *prev = NULL;
    int                     reqmode, done = 0;

    DEBUG (ep->dev, "%s dma hiccup td %p\n", ep->ep.name, req->td);
    ep->in_fifo_validate = likely (req->req.zero
                                   || (req->req.length % ep->ep.maxpacket) != 0);
    if (ep->in_fifo_validate)
      dmactl |= (1 << DMA_FIFO_VALIDATE);
    list_for_each_entry (entry, &ep->queue, queue) {
      __le32          dmacount;

      if (entry == req)
        continue;
      dmacount = entry->td->dmacount;
      if (!done) {
        reqmode = likely (entry->req.zero
                          || (entry->req.length
                              % ep->ep.maxpacket) != 0);
        if (reqmode == ep->in_fifo_validate) {
          entry->valid = 1;
          dmacount |= valid_bit;
          entry->td->dmacount = dmacount;
          prev = entry;
          continue;
        } else {
          /* force a hiccup */
          prev->td->dmacount |= dma_done_ie;
          done = 1;
        }
      }

      /* walk the rest of the queue so unlinks behave */
      entry->valid = 0;
      dmacount &= ~valid_bit;
      entry->td->dmacount = dmacount;
      prev = entry;
    }
  }

  writel (0, &ep->dma->dmactl);
  start_queue (ep, dmactl, req->td_dma);
}

static inline
void net2280_led_speed (struct net2280 *dev, int speed)
{
  u32     val = readl (&dev->regs->gpioctl);
  switch (speed) {
  case USB_SPEED_HIGH:            /* green */
    val &= ~(1 << GPIO0_DATA);
    val |= (1 << GPIO1_DATA);
    break;
  case USB_SPEED_FULL:            /* red */
    val &= ~(1 << GPIO1_DATA);
    val |= (1 << GPIO0_DATA);
    break;
  default:                        /* (off/black) */
    val &= ~((1 << GPIO1_DATA) | (1 << GPIO0_DATA));
    break;
  }
  writel (val, &dev->regs->gpioctl);
}

static void
done (struct net2280_ep *ep, struct net2280_request *req, int status)
{
  struct net2280          *dev;
#if defined(NET2280_IO_VCPU) || defined(NET2280_MAIN_VCPU)
  u32 flags;
#endif
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
    VDEBUG (dev, "complete %s req %p stat %d len %u/%u\n",
            ep->ep.name, &req->req, status,
            req->req.actual, req->req.length);
  
#if defined(NET2280_IO_VCPU) || defined(NET2280_MAIN_VCPU)
  
  if(&req->req != dev->ep0req) {
    spinlock_lock_irq_save(&dev->completion_list_lock, flags);
    list_add_tail(&req->completion_chain, &dev->completion_list);
    spinlock_unlock_irq_restore(&dev->completion_list_lock, flags);
    /* -- EM -- hack: use VCPU2's period */

#ifdef NET2280_IO_VCPU
    iovcpu_job_wakeup (dev->iovcpu, vcpu_lookup (2)->T);
#else
    wakeup(dev->iovcpu);
#endif
    
  }
  else {
#endif
    /* don't modify queue heads during completion callback */
    ep->stopped = 1;
    spinlock_unlock (&dev->lock);
    req->req.complete (&ep->ep, &req->req);
    spinlock_lock (&dev->lock);
    ep->stopped = stopped;
#if defined(NET2280_IO_VCPU) || defined(NET2280_MAIN_VCPU)
  }
#endif
}

static void handle_stat0_irqs (struct net2280 *dev, u32 stat)
{
  struct net2280_ep       *ep;
  u32                     num, scratch;

  /* most of these don't need individual acks */
  stat &= ~(1 << INTA_ASSERTED);
  if (!stat)
    return;
  DEBUG (dev, "irqstat0 %04x\n", stat);

  /* starting a control request? */
  if (unlikely (stat & (1 << SETUP_PACKET_INTERRUPT))) {
    union {
      u32                     raw [2];
      struct usb_ctrlrequest  r;
    } u;
    int                             tmp;
    struct net2280_request          *req;

    if (dev->gadget.speed == USB_SPEED_UNKNOWN) {
      if (readl (&dev->usb->usbstat) & (1 << HIGH_SPEED))
        dev->gadget.speed = USB_SPEED_HIGH;
      else
        dev->gadget.speed = USB_SPEED_FULL;
      net2280_led_speed (dev, dev->gadget.speed);
      DLOG("%s\n", usb_speed_string(dev->gadget.speed));
    }

    ep = &dev->ep [0];
    ep->irqs++;

    /* make sure any leftover request state is cleared */
    stat &= ~(1 << ENDPOINT_0_INTERRUPT);
    while (!list_empty (&ep->queue)) {
      req = list_entry (ep->queue.next,
                        struct net2280_request, queue);
      done (ep, req, (req->req.actual == req->req.length)
            ? 0 : -1);
    }
    ep->stopped = 0;
    dev->protocol_stall = 0;

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
            | (1 << SHORT_PACKET_OUT_DONE_INTERRUPT)
            | (1 << SHORT_PACKET_TRANSFERRED_INTERRUPT)
            | (1 << DATA_PACKET_RECEIVED_INTERRUPT)
            | (1 << DATA_PACKET_TRANSMITTED_INTERRUPT)
            | (1 << DATA_OUT_PING_TOKEN_INTERRUPT)
            | (1 << DATA_IN_TOKEN_INTERRUPT)
            , &ep->regs->ep_stat);
    u.raw [0] = readl (&dev->usb->setup0123);
    u.raw [1] = readl (&dev->usb->setup4567);

    cpu_to_le32s (&u.raw [0]);
    cpu_to_le32s (&u.raw [1]);

    tmp = 0;

#define w_value         le16_to_cpu(u.r.wValue)
#define w_index         le16_to_cpu(u.r.wIndex)
#define w_length        le16_to_cpu(u.r.wLength)

    /* ack the irq */
    writel (1 << SETUP_PACKET_INTERRUPT, &dev->regs->irqstat0);
    stat ^= (1 << SETUP_PACKET_INTERRUPT);

    /* watch control traffic at the token level, and force
     * synchronization before letting the status stage happen.
     * FIXME ignore tokens we'll NAK, until driver responds.
     * that'll mean a lot less irqs for some drivers.
     */
    ep->is_in = (u.r.bmRequestType & USB_DIR_IN) != 0;
    if (ep->is_in) {
      scratch = (1 << DATA_PACKET_TRANSMITTED_INTERRUPT)
        | (1 << DATA_OUT_PING_TOKEN_INTERRUPT)
        | (1 << DATA_IN_TOKEN_INTERRUPT);
      stop_out_naking (ep);
    } else
      scratch = (1 << DATA_PACKET_RECEIVED_INTERRUPT)
        | (1 << DATA_OUT_PING_TOKEN_INTERRUPT)
        | (1 << DATA_IN_TOKEN_INTERRUPT);
    writel (scratch, &dev->epregs [0].ep_irqenb);

    /* we made the hardware handle most lowlevel requests;
     * everything else goes uplevel to the gadget code.
     */
    ep->responded = 1;
    switch (u.r.bRequest) {
    case USB_REQ_GET_STATUS: {
      struct net2280_ep       *e;
      __le32                  status;

      /* hw handles device and interface status */
      if (u.r.bmRequestType != (USB_DIR_IN|USB_RECIP_ENDPOINT))
        goto delegate;
      if ((e = get_ep_by_addr (dev, w_index)) == 0
          || w_length > 2)
        goto do_stall;

      if (readl (&e->regs->ep_rsp)
          & (1 << SET_ENDPOINT_HALT))
        status = cpu_to_le32 (1);
      else
        status = cpu_to_le32 (0);

      /* don't bother with a request object! */
      writel (0, &dev->epregs [0].ep_irqenb);
      set_fifo_bytecount (ep, w_length);
      writel ((__force u32)status, &dev->epregs [0].ep_data);
      allow_status (ep);
      DLOGV("%s stat %02x\n", ep->ep.name, status);
      goto next_endpoints;
    }
      break;
    case USB_REQ_CLEAR_FEATURE: {
      struct net2280_ep       *e;

      /* hw handles device features */
      if (u.r.bmRequestType != USB_RECIP_ENDPOINT)
        goto delegate;
      if (w_value != USB_ENDPOINT_HALT
          || w_length != 0)
        goto do_stall;
      if ((e = get_ep_by_addr (dev, w_index)) == 0)
        goto do_stall;
      if (e->wedged) {
        VDEBUG(dev, "%s wedged, halt not cleared\n",
               ep->ep.name);
      } else {
        VDEBUG(dev, "%s clear halt\n", ep->ep.name);
        clear_halt(e);
      }
      allow_status (ep);
      goto next_endpoints;
    }
      break;
    case USB_REQ_SET_FEATURE: {
      struct net2280_ep       *e;

      /* hw handles device features */
      if (u.r.bmRequestType != USB_RECIP_ENDPOINT)
        goto delegate;
      if (w_value != USB_ENDPOINT_HALT
          || w_length != 0)
        goto do_stall;
      if ((e = get_ep_by_addr (dev, w_index)) == 0)
        goto do_stall;
      if (e->ep.name == ep0name)
        goto do_stall;
      set_halt (e);
      allow_status (ep);
      VDEBUG (dev, "%s set halt\n", ep->ep.name);
      goto next_endpoints;
    }
      break;
    default:
    delegate:
      VDEBUG (dev, "setup %02x.%02x v%04x i%04x l%04x "
              "ep_cfg %08x\n",
              u.r.bmRequestType, u.r.bRequest,
              w_value, w_index, w_length,
              readl (&ep->regs->ep_cfg));
      ep->responded = 0;
      spinlock_unlock (&dev->lock);
      tmp = dev->driver->setup (&dev->gadget, &u.r);
      DLOGV("tmp = %d at line %d", tmp, __LINE__);
      spinlock_lock (&dev->lock);
    }

    /* stall ep0 on error */
    if (tmp < 0) {
      DLOG("Stalling because tmp < 0");
      
    do_stall:
      VDEBUG (dev, "req %02x.%02x protocol STALL; stat %d\n",
              u.r.bmRequestType, u.r.bRequest, tmp);
      dev->protocol_stall = 1;
    }

    /* some in/out token irq should follow; maybe stall then.
     * driver must queue a request (even zlp) or halt ep0
     * before the host times out.
     */
  }

#undef  w_value
#undef  w_index
#undef  w_length

 next_endpoints:
  /* endpoint data irq ? */
  scratch = stat & 0x7f;
  stat &= ~0x7f;
  for (num = 0; scratch; num++) {
    u32             t;

    /* do this endpoint's FIFO and queue need tending? */
    t = 1 << num;
    if ((scratch & t) == 0)
      continue;
    scratch ^= t;

    ep = &dev->ep [num];
    handle_ep_small (ep);
  }

  if (stat)
    DEBUG (dev, "unhandled irqstat0 %08x\n", stat);
}

#define DMA_INTERRUPTS (                                \
                        (1 << DMA_D_INTERRUPT)          \
                        | (1 << DMA_C_INTERRUPT)        \
                        | (1 << DMA_B_INTERRUPT)        \
                        | (1 << DMA_A_INTERRUPT))
#define PCI_ERROR_INTERRUPTS (                                          \
                              (1 << PCI_MASTER_ABORT_RECEIVED_INTERRUPT) \
                              | (1 << PCI_TARGET_ABORT_RECEIVED_INTERRUPT) \
                              | (1 << PCI_RETRY_ABORT_INTERRUPT))

static void handle_stat1_irqs (struct net2280 *dev, u32 stat)
{
  struct net2280_ep       *ep;
  u32                     tmp, num, mask, scratch;

  /* after disconnect there's nothing else to do! */
  tmp = (1 << VBUS_INTERRUPT) | (1 << ROOT_PORT_RESET_INTERRUPT);
  mask = (1 << HIGH_SPEED) | (1 << FULL_SPEED);

  /* VBUS disconnect is indicated by VBUS_PIN and VBUS_INTERRUPT set.
   * Root Port Reset is indicated by ROOT_PORT_RESET_INTERRUPT set and
   * both HIGH_SPEED and FULL_SPEED clear (as ROOT_PORT_RESET_INTERRUPT
   * only indicates a change in the reset state).
   */
  if (stat & tmp) {
    writel (tmp, &dev->regs->irqstat1);
    if ((((stat & (1 << ROOT_PORT_RESET_INTERRUPT))
          && ((readl (&dev->usb->usbstat) & mask)
              == 0))
         || ((readl (&dev->usb->usbctl)
              & (1 << VBUS_PIN)) == 0)
         ) && ( dev->gadget.speed != USB_SPEED_UNKNOWN)) {
      stop_activity (dev, dev->driver);
      ep0_start (dev);
      return;
    }
    stat &= ~tmp;

    /* vBUS can bounce ... one of many reasons to ignore the
     * notion of hotplug events on bus connect/disconnect!
     */
    if (!stat)
      return;
  }

  /* NOTE: chip stays in PCI D0 state for now, but it could
   * enter D1 to save more power
   */
  tmp = (1 << SUSPEND_REQUEST_CHANGE_INTERRUPT);
  if (stat & tmp) {
    writel (tmp, &dev->regs->irqstat1);
    if (stat & (1 << SUSPEND_REQUEST_INTERRUPT)) {
      if (dev->driver->suspend)
        dev->driver->suspend (&dev->gadget);
      if (!enable_suspend)
        stat &= ~(1 << SUSPEND_REQUEST_INTERRUPT);
    } else {
      if (dev->driver->resume)
        dev->driver->resume (&dev->gadget);
      /* at high speed, note erratum 0133 */
    }
    stat &= ~tmp;
  }

  /* clear any other status/irqs */
  if (stat)
    writel (stat, &dev->regs->irqstat1);

  /* some status we can just ignore */
  if (dev->is_2280)
    stat &= ~((1 << CONTROL_STATUS_INTERRUPT)
              | (1 << SUSPEND_REQUEST_INTERRUPT)
              | (1 << RESUME_INTERRUPT)
              | (1 << SOF_INTERRUPT));
  else
    stat &= ~((1 << CONTROL_STATUS_INTERRUPT)
              | (1 << RESUME_INTERRUPT)
              | (1 << SOF_DOWN_INTERRUPT)
              | (1 << SOF_INTERRUPT));

  if (!stat)
    return;
  // DLOG("irqstat1 %08x\n", stat);

  /* DMA status, for ep-{a,b,c,d} */
  scratch = stat & DMA_INTERRUPTS;
  stat &= ~DMA_INTERRUPTS;
  scratch >>= 9;
  for (num = 0; scratch; num++) {
    struct net2280_dma_regs *dma;

    tmp = 1 << num;
    if ((tmp & scratch) == 0)
      continue;
    scratch ^= tmp;

    ep = &dev->ep [num + 1];
    dma = ep->dma;

    if (!dma)
      continue;

    /* clear ep's dma status */
    tmp = readl (&dma->dmastat);
    writel (tmp, &dma->dmastat);

    /* chaining should stop on abort, short OUT from fifo,
     * or (stat0 codepath) short OUT transfer.
     */
    if (!use_dma_chaining) {
      if ((tmp & (1 << DMA_TRANSACTION_DONE_INTERRUPT))
          == 0) {
        DLOG("%s no xact done? %08x\n",
             ep->ep.name, tmp);
        continue;
      }
      stop_dma (ep->dma);
    }

    /* OUT transfers terminate when the data from the
     * host is in our memory.  Process whatever's done.
     * On this path, we know transfer's last packet wasn't
     * less than req->length. NAK_OUT_PACKETS may be set,
     * or the FIFO may already be holding new packets.
     *
     * IN transfers can linger in the FIFO for a very
     * long time ... we ignore that for now, accounting
     * precisely (like PIO does) needs per-packet irqs
     */
    scan_dma_completions (ep);

    /* disable dma on inactive queues; else maybe restart */
    if (list_empty (&ep->queue)) {
      if (use_dma_chaining)
        stop_dma (ep->dma);
    } else {
      tmp = readl (&dma->dmactl);
      if (!use_dma_chaining
          || (tmp & (1 << DMA_ENABLE)) == 0)
        restart_dma (ep);
      else if (ep->is_in && use_dma_chaining) {
        struct net2280_request  *req;
        __le32                  dmacount;

        /* the descriptor at the head of the chain
         * may still have VALID_BIT clear; that's
         * used to trigger changing DMA_FIFO_VALIDATE
         * (affects automagic zlp writes).
         */
        req = list_entry (ep->queue.next,
                          struct net2280_request, queue);
        dmacount = req->td->dmacount;
        dmacount &= cpu_to_le32 (
                                 (1 << VALID_BIT)
                                 | DMA_BYTE_COUNT_MASK);
        if (dmacount && (dmacount & valid_bit) == 0)
          restart_dma (ep);
      }
    }
    ep->irqs++;
  }

  /* NOTE:  there are other PCI errors we might usefully notice.
   * if they appear very often, here's where to try recovering.
   */
  if (stat & PCI_ERROR_INTERRUPTS) {
    ERROR (dev, "pci dma error; stat %08x\n", stat);
    stat &= ~PCI_ERROR_INTERRUPTS;
    /* these are fatal errors, but "maybe" they won't
     * happen again ...
     */
    stop_activity (dev, dev->driver);
    ep0_start (dev);
    stat = 0;
  }

  if (stat)
    DLOG("unhandled irqstat1 %08x\n", stat);
}

static uint32_t
net2280_irq_handler(uint8 vec)
{
  lock_kernel();
  /* shared interrupt, not ours */
  if (!(readl(&net2280_dev.regs->irqstat0) & (1 << INTA_ASSERTED))) {
    unlock_kernel();
    return 0;
  }
  
  spinlock_lock (&net2280_dev.lock);
  
  /* handle disconnect, dma, and more */
  handle_stat1_irqs(&net2280_dev, readl (&net2280_dev.regs->irqstat1));
  
  /* control requests and PIO */
  handle_stat0_irqs(&net2280_dev, readl (&net2280_dev.regs->irqstat0));
  
  spinlock_unlock (&net2280_dev.lock);
  unlock_kernel();
  return 1;
}


#define REG_EP_MAXPKT(dev,num) (((num) + 1) * 0x10 +                    \
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
      || desc->bDescriptorType != USB_TYPE_EPT_DESC) {
    DLOGV("net2280_enable failed at line %d", __LINE__);
    return -1;
  }
  dev = ep->dev;
  if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
    DLOG("net2280_enable failed at line %d", __LINE__);
    DLOGV("dev->driver = 0x%p", dev->driver);
    if(dev->driver) {
      DLOGV("dev->gadget.speed = 0x%X", dev->gadget.speed);
    }
    return -1;
  }
 
  /* erratum 0119 workaround ties up an endpoint number */
  if ((desc->bEndpointAddress & 0x0f) == EP_DONTUSE) {
    DLOG("net2280_enable failed at line %d", __LINE__);
    return -1;
  }
 
  /* sanity check ep-e/ep-f since their fifos are small */
  max = desc->wMaxPacketSize & 0x1fff;
  if (ep->num > 4 && max > 64) {
    DLOG("net2280_enable failed at line %d", __LINE__);
    return -1;
  }
 
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
      DLOG("net2280_enable failed at line %d", __LINE__);
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

static void start_dma (struct net2280_ep *ep, struct net2280_request *req)
{
  u32                     tmp;
  struct net2280_dma_regs *dma = ep->dma;

  /* FIXME can't use DMA for ZLPs */

  /* on this path we "know" there's no dma active (yet) */
  //WARN_ON (readl (&dma->dmactl) & (1 << DMA_ENABLE));
  writel (0, &ep->dma->dmactl);

  /* previous OUT packet might have been short */
  if (!ep->is_in && ((tmp = readl (&ep->regs->ep_stat))
                     & (1 << NAK_OUT_PACKETS)) != 0) {
    writel ((1 << SHORT_PACKET_TRANSFERRED_INTERRUPT),
            &ep->regs->ep_stat);

    tmp = readl (&ep->regs->ep_avail);
    if (tmp) {
      writel (readl (&dma->dmastat), &dma->dmastat);

      /* transfer all/some fifo data */
      writel (req->req.dma, &dma->dmaaddr);
      tmp = min (tmp, req->req.length);

      /* dma irq, faking scatterlist status */
      req->td->dmacount = cpu_to_le32 (req->req.length - tmp);
      writel ((1 << DMA_DONE_INTERRUPT_ENABLE)
              | tmp, &dma->dmacount);
      req->td->dmadesc = 0;
      req->valid = 1;

      writel ((1 << DMA_ENABLE), &dma->dmactl);
      writel ((1 << DMA_START), &dma->dmastat);
      return;
    }
  }

  tmp = dmactl_default;

  /* force packet boundaries between dma requests, but prevent the
   * controller from automagically writing a last "short" packet
   * (zero length) unless the driver explicitly said to do that.
   */
  if (ep->is_in) {
    if (likely ((req->req.length % ep->ep.maxpacket) != 0
                || req->req.zero)) {
      tmp |= (1 << DMA_FIFO_VALIDATE);
      ep->in_fifo_validate = 1;
    } else
      ep->in_fifo_validate = 0;
  }

  /* init req->td, pointing to the current dummy */
  req->td->dmadesc = cpu_to_le32 (ep->td_dma);
  fill_dma_desc (ep, req, 1);

  if (!use_dma_chaining)
    req->td->dmacount |= cpu_to_le32 (1 << END_OF_CHAIN);

  start_queue (ep, tmp, req->td_dma);
}


/* handle ep0, ep-e, ep-f with 64 byte packets: packet per irq.
 * also works for dma-capable endpoints, in pio mode or just
 * to manually advance the queue after short OUT transfers.
 */
static void handle_ep_small (struct net2280_ep *ep)
{
  struct net2280_request  *req;
  u32                     t;
  /* 0 error, 1 mid-data, 2 done */
  int                     mode = 1;

  if (!list_empty (&ep->queue))
    req = list_entry (ep->queue.next,
                      struct net2280_request, queue);
  else
    req = NULL;

  /* ack all, and handle what we care about */
  t = readl (&ep->regs->ep_stat);
  ep->irqs++;
#if 0
  VDEBUG (ep->dev, "%s ack ep_stat %08x, req %p\n",
          ep->ep.name, t, req ? &req->req : 0);
#endif
  if (!ep->is_in || ep->dev->is_2280)
    writel (t & ~(1 << NAK_OUT_PACKETS), &ep->regs->ep_stat);
  else
    /* Added for 2282 */
    writel (t, &ep->regs->ep_stat);

  /* for ep0, monitor token irqs to catch data stage length errors
   * and to synchronize on status.
   *
   * also, to defer reporting of protocol stalls ... here's where
   * data or status first appears, handling stalls here should never
   * cause trouble on the host side..
   *
   * control requests could be slightly faster without token synch for
   * status, but status can jam up that way.
   */
  if (unlikely (ep->num == 0)) {
    if (ep->is_in) {
      /* status; stop NAKing */
      if (t & (1 << DATA_OUT_PING_TOKEN_INTERRUPT)) {
        if (ep->dev->protocol_stall) {
          ep->stopped = 1;
          set_halt (ep);
        }
        if (!req)
          allow_status (ep);
        mode = 2;
        /* reply to extra IN data tokens with a zlp */
      } else if (t & (1 << DATA_IN_TOKEN_INTERRUPT)) {
        if (ep->dev->protocol_stall) {
          ep->stopped = 1;
          set_halt (ep);
          mode = 2;
        } else if (ep->responded &&
                   !req && !ep->stopped)
          write_fifo (ep, NULL);
      }
    } else {
      /* status; stop NAKing */
      if (t & (1 << DATA_IN_TOKEN_INTERRUPT)) {
        if (ep->dev->protocol_stall) {
          ep->stopped = 1;
          set_halt (ep);
        }
        mode = 2;
        /* an extra OUT token is an error */
      } else if (((t & (1 << DATA_OUT_PING_TOKEN_INTERRUPT))
                  && req
                  && req->req.actual == req->req.length)
                 || (ep->responded && !req)) {
        ep->dev->protocol_stall = 1;
        set_halt (ep);
        ep->stopped = 1;
        if (req)
          done (ep, req, -1);
        req = NULL;
      }
    }
  }

  if (unlikely (!req))
    return;

  /* manual DMA queue advance after short OUT */
  if (likely (ep->dma != 0)) {
    if (t & (1 << SHORT_PACKET_TRANSFERRED_INTERRUPT)) {
      u32     count;
      int     stopped = ep->stopped;

      /* TRANSFERRED works around OUT_DONE erratum 0112.
       * we expect (N <= maxpacket) bytes; host wrote M.
       * iff (M < N) we won't ever see a DMA interrupt.
       */
      ep->stopped = 1;
      for (count = 0; ; t = readl (&ep->regs->ep_stat)) {

        /* any preceding dma transfers must finish.
         * dma handles (M >= N), may empty the queue
         */
        scan_dma_completions (ep);
        if (unlikely (list_empty (&ep->queue)
                      || ep->out_overflow)) {
          req = NULL;
          break;
        }
        req = list_entry (ep->queue.next,
                          struct net2280_request, queue);

        /* here either (M < N), a "real" short rx;
         * or (M == N) and the queue didn't empty
         */
        if (likely (t & (1 << FIFO_EMPTY))) {
          count = readl (&ep->dma->dmacount);
          count &= DMA_BYTE_COUNT_MASK;
          if (readl (&ep->dma->dmadesc)
              != req->td_dma)
            req = NULL;
          break;
        }
        udelay(1);
      }

      /* stop DMA, leave ep NAKing */
      writel ((1 << DMA_ABORT), &ep->dma->dmastat);
      spin_stop_dma (ep->dma);

      if (likely (req)) {
        req->td->dmacount = 0;
        t = readl (&ep->regs->ep_avail);
        dma_done (ep, req, count,
                  (ep->out_overflow || t)
                  ? -1 : 0);
      }

      /* also flush to prevent erratum 0106 trouble */
      if (unlikely (ep->out_overflow
                    || (ep->dev->chiprev == 0x0100
                        && ep->dev->gadget.speed
                        == USB_SPEED_FULL))) {
        out_flush (ep);
        ep->out_overflow = 0;
      }

      /* (re)start dma if needed, stop NAKing */
      ep->stopped = stopped;
      if (!list_empty (&ep->queue))
        restart_dma (ep);
    } else
      DEBUG (ep->dev, "%s dma ep_stat %08x ??\n",
             ep->ep.name, t);
    return;

    /* data packet(s) received (in the fifo, OUT) */
  } else if (t & (1 << DATA_PACKET_RECEIVED_INTERRUPT)) {
    if (read_fifo (ep, req) && ep->num != 0)
      mode = 2;

    /* data packet(s) transmitted (IN) */
  } else if (t & (1 << DATA_PACKET_TRANSMITTED_INTERRUPT)) {
    unsigned        len;

    len = req->req.length - req->req.actual;
    if (len > ep->ep.maxpacket)
      len = ep->ep.maxpacket;
    req->req.actual += len;

    /* if we wrote it all, we're usually done */
    if (req->req.actual == req->req.length) {
      if (ep->num == 0) {
        /* send zlps until the status stage */
      } else if (!req->req.zero || len != ep->ep.maxpacket)
        mode = 2;
    }

    /* there was nothing to do ...  */
  } else if (mode == 1)
    return;

  /* done */
  if (mode == 2) {
    /* stream endpoints often resubmit/unlink in completion */
    done (ep, req, 0);

    /* maybe advance queue to next request */
    if (ep->num == 0) {
      /* NOTE:  net2280 could let gadget driver start the
       * status stage later. since not all controllers let
       * them control that, the api doesn't (yet) allow it.
       */
      if (!ep->stopped)
        allow_status (ep);
      req = NULL;
    } else {
      if (!list_empty (&ep->queue) && !ep->stopped)
        req = list_entry (ep->queue.next,
                          struct net2280_request, queue);
      else
        req = NULL;
      if (req && !ep->is_in)
        stop_out_naking (ep);
    }
  }

  /* is there a buffer for the next packet?
   * for best streaming performance, make sure there is one.
   */
  if (req && !ep->stopped) {

    /* load IN fifo with next packet (may be zlp) */
    if (t & (1 << DATA_PACKET_TRANSMITTED_INTERRUPT))
      write_fifo (ep, &req->req);
  }
}


int net2280_disable (struct usb_ep *ep)
{
  DLOG("IN %s which is not implemented", __FUNCTION__);
  panic("In unimplemented net2280 ep op function");
}

static struct usb_request *
net2280_alloc_request (struct usb_ep *_ep, gfp_t gfp_flags)
{
  struct net2280_ep       *ep;
  struct net2280_request  *req;

  if (!_ep)
    return NULL;
  ep = container_of (_ep, struct net2280_ep, ep);

  req = kzalloc(sizeof(*req));
  if (!req)
    return NULL;

  req->req.dma = DMA_ADDR_INVALID;
  INIT_LIST_HEAD (&req->queue);
  INIT_LIST_HEAD (&req->completion_chain);
  /* this dma descriptor may be swapped with the previous dummy */
  if (ep->dma) {
    struct net2280_dma      *td;

    //td = pci_pool_alloc (ep->dev->requests, gfp_flags,
    //                     &req->td_dma);

    td = allocate_net2280_dma(&net2280_dev);
    
    if (!td) {
      kfree (req);
      return NULL;
    }


    req->td_dma = NET2280_DMA_VIRT_TO_PHYS(&net2280_dev, td);
    
    td->dmacount = 0;       /* not VALID */
    td->dmaaddr = cpu_to_le32 (DMA_ADDR_INVALID);
    td->dmadesc = td->dmaaddr;
    req->td = td;
  }
  req->ep = ep;
  return &req->req;
}


static void
net2280_free_request (struct usb_ep *_ep, struct usb_request *_req)
{
  //struct net2280_ep       *ep;
  struct net2280_request  *req;
  
  //ep = container_of (_ep, struct net2280_ep, ep);
  if (!_ep || !_req)
    return;
  
  req = container_of (_req, struct net2280_request, req);
  //WARN_ON (!list_empty (&req->queue));
  if (req->td) {
    free_net2280_dma(&net2280_dev, req->td);
    //pci_pool_free (ep->dev->requests, req->td, req->td_dma);
  }
  kfree (req);
}

static inline void
queue_dma (struct net2280_ep *ep, struct net2280_request *req, int valid)
{
  struct net2280_dma      *end;
  dma_addr_t              tmp;
  
  /* swap new dummy for old, link; fill and maybe activate */
  end = ep->dummy;
  ep->dummy = req->td;
  req->td = end;

  tmp = ep->td_dma;
  ep->td_dma = req->td_dma;
  req->td_dma = tmp;

  end->dmadesc = cpu_to_le32 (ep->td_dma);

  fill_dma_desc (ep, req, valid);
}
  
static int
net2280_queue (struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
  struct net2280_request  *req;
  struct net2280_ep       *ep;
  struct net2280          *dev;
  u32                     flags;

  /* we always require a cpu-view buffer, so that we can
   * always use pio (as fallback or whatever).
   */
  req = container_of (_req, struct net2280_request, req);
  if (!_req || !_req->complete || !_req->buf
      || !list_empty (&req->queue)) {
    DLOG("Returning -1 at line %d", __LINE__);
    return -1;
  }
  if (_req->length > (~0 & DMA_BYTE_COUNT_MASK)) {
    DLOG("Returning -1 at line %d", __LINE__);
    return -1;
  }
  ep = container_of (_ep, struct net2280_ep, ep);
  if (!_ep || (!ep->desc && ep->num != 0)) {
    DLOGV("ep->desc = 0x%p", ep->desc);
    DLOGV("ep->num = %d", ep->num);
    DLOG("Returning -1 at line %d", __LINE__);
    return -1;
  }
  dev = ep->dev;
  if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
    DLOG("Returning -1 at line %d", __LINE__);
    return -1;
  }

  /* FIXME implement PIO fallback for ZLPs with DMA */
  if (ep->dma && _req->length == 0) {
    return -1;
  }

  /* set up dma mapping in case the caller didn't */
  if (ep->dma) {
    int ret;
    DLOG("About to call usb_gadget_map_request which isn't implemented");
    panic("About to call usb_gadget_map_request which isn't implemented");
    //
    //ret = usb_gadget_map_request(&dev->gadget, _req,
    //                             ep->is_in);
    if (ret)
      return ret;
  }

#if 0
  VDEBUG (dev, "%s queue req %p, len %d buf %p\n",
          _ep->name, _req, _req->length, _req->buf);
#endif

  spinlock_lock_irq_save (&dev->lock, flags);
  
  _req->status = -1;
  _req->actual = 0;

  /* kickstart this i/o queue? */
  if (list_empty (&ep->queue) && !ep->stopped) {
    /* use DMA if the endpoint supports it, else pio */
    if (ep->dma)
      start_dma (ep, req);
    else {
      /* maybe there's no control data, just status ack */
      if (ep->num == 0 && _req->length == 0) {
        allow_status (ep);
        done (ep, req, 0);
        VDEBUG (dev, "%s status ack\n", ep->ep.name);
        goto done;
      }

      /* PIO ... stuff the fifo, or unblock it.  */
      if (ep->is_in)
        write_fifo (ep, _req);
      else if (list_empty (&ep->queue)) {
        u32     s;

        /* OUT FIFO might have packet(s) buffered */
        s = readl (&ep->regs->ep_stat);
        if ((s & (1 << FIFO_EMPTY)) == 0) {
          /* note: _req->short_not_ok is ignored here since PIO
           * _always_ stops queue advance here, and _req->status
           * doesn't change for short reads (only _req->actual)
           */
          if (read_fifo (ep, req)) {
            done (ep, req, 0);
            if (ep->num == 0)
              allow_status (ep);
            /* don't queue it */
            req = NULL;
          } else
            s = readl (&ep->regs->ep_stat);
        }

        /* don't NAK, let the fifo fill */
        if (req && (s & (1 << NAK_OUT_PACKETS)))
          writel ((1 << CLEAR_NAK_OUT_PACKETS),
                  &ep->regs->ep_rsp);
      }
    }

  } else if (ep->dma) {
    int     valid = 1;

    if (ep->is_in) {
      int     expect;

      /* preventing magic zlps is per-engine state, not
       * per-transfer; irq logic must recover hiccups.
       */
      expect = likely (req->req.zero
                       || (req->req.length % ep->ep.maxpacket) != 0);
      if (expect != ep->in_fifo_validate)
        valid = 0;
    }
    queue_dma (ep, req, valid);

  } /* else the irq handler advances the queue. */

  ep->responded = 1;
  if (req)
    list_add_tail (&req->queue, &ep->queue);
 done:
  spinlock_unlock_irq_restore (&dev->lock, flags);
  
  /* pci writes may still be posted */
  return 0;
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

static void
stop_activity (struct net2280 *dev, struct usb_gadget_driver *driver)
{
  int                     i;
  
  /* don't disconnect if it's not connected */
  if (dev->gadget.speed == USB_SPEED_UNKNOWN)
    driver = NULL;
  
  /* stop hardware; prevent new request submissions;
   * and kill any outstanding requests.
   */
  usb_reset (dev);
  for (i = 0; i < 7; i++)
    nuke (&dev->ep [i]);
  
  usb_reinit (dev);
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

#if defined(NET2280_IO_VCPU) || defined(NET2280_MAIN_VCPU)
  INIT_LIST_HEAD(&dev->completion_list);
  spinlock_init(&dev->completion_list_lock);
#endif
  spinlock_init(&dev->lock);

  dev->gadget.ops = &net2280_ops;
  dev->gadget.max_speed = USB_SPEED_HIGH;
  dev->regs   = (struct net2280_regs     *) (base_virt_addr);
  dev->usb    = (struct net2280_usb_regs *) (base_virt_addr + 0x0080);
  dev->pci    = (struct net2280_pci_regs *) (base_virt_addr + 0x0100);
  dev->dma    = (struct net2280_dma_regs *) (base_virt_addr + 0x0180);
  dev->dep    = (struct net2280_dep_regs *) (base_virt_addr + 0x0200);
  dev->epregs = (struct net2280_ep_regs  *) (base_virt_addr + 0x0300);
  dev->is_2280 = is_2280;

  writel (0, &dev->usb->usbctl);

  usb_reset(dev);
  usb_reinit(dev);

  dev->got_irq = 1;

#define INIT_NET2280_POOL(type)                                         \
  do {                                                                  \
    dev->type##_pool = type##_pool;                                     \
    dev->type##_pool_phys_addr = (phys_addr_t)get_phys_addr(type##_pool); \
    dev->type##_pool_size = type##_pool_size;                           \
    dev->used_##type##_bitmap = used_##type##_bitmap;                   \
    memset(type##_pool, 0, type##_pool_size * sizeof(type##_t));        \
    dev->used_##type##_bitmap_size =                                    \
    ((dev)->type##_pool_size + (NET2280_ELEMENTS_PER_BITMAP_ENTRY - 1)) \
      / NET2280_ELEMENTS_PER_BITMAP_ENTRY;                              \
    memset(used_##type##_bitmap, 0,                                     \
    dev->used_##type##_bitmap_size * sizeof(*used_##type##_bitmap));    \
  }                                                                     \
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


  /* -- EM -- Add stuff for pseudo gadget driver to work */

  
  dev->ep0 = dev->gadget.ep0;
  dev->ep0->driver_data = dev;
  dev->ep0req = usb_ep_alloc_request(dev->ep0, 0);

  if (!dev->ep0req) {
    DLOG("Failed to allocate ep 0");
    panic("Failed to allocate ep 0");
  }
  dev->ep0req->buf = kmalloc(EP0_BUFSIZE);
  if (!dev->ep0req->buf) {
    DLOG("Failed to allocate buffer");
    panic("Failed to allocate buffer");
  }
  dev->ep0req->complete = ep0_complete;
  
  return TRUE;
}

#ifdef NET2280_MIGRATION_MODE

static void net2280_migration_thread()
{
  phys_addr_t cr3;
  pgdir_entry_t* pd_table;
  pgdir_entry_t* pd_table_original;
  int i;
  USB_DEVICE_INFO* device_info = usb_get_device(net2280_dev_num);
  if(device_info == NULL) {
    DLOG("device info for migration device could not be found");
    panic("device info for migration device could not be found");
  }

  /* -- EM -- Spin until device has been enumerated by host*/
  while(net2280_dev.gadget.speed == USB_SPEED_UNKNOWN) {
    sched_usleep(1000000 * 1);
  }

  memset(net2280_dev.kernel_specific_pages, 0, sizeof(net2280_dev.kernel_specific_pages));
  
  cr3 = (phys_addr_t)get_pdbr();
  pd_table_original = pd_table = map_virtual_page(cr3 | 3);
  if(pd_table_original == NULL) {
    DLOG("Failed to map pd table");
    panic("Failed to map pd table");
  }

  for(i = 0; i < 1024; ++i, ++pd_table) {
    if(pd_table->flags.present && !pd_table->flags.supervisor && i != PGDIR_KERNEL_STACK) {
      net2280_dev.kernel_specific_pages[i] = pd_table->raw;
    }
  }

  unmap_virtual_page(pd_table_original);
  
  if(net2280_open(device_info, net2280_dev_num) < 0) {
    DLOG("net2280 open failed");
    panic("net2280 open failed");
  }

  return;
  DLOG("Spinning in migration thread");
  while(1) sched_usleep(10000000 * 30);
}

#endif


#if defined(NET2280_IO_VCPU) || defined(NET2280_MAIN_VCPU)
static void net2280_bh_thread(struct net2280* dev)
{
  //unlock_kernel();
  //sti();
  u32 flags;
  
  while(1) {
    while(1) {
      struct net2280_request *req;
      spinlock_lock_irq_save(&dev->completion_list_lock, flags);
      
      if(!list_empty(&dev->completion_list)) {
        req = list_entry(dev->completion_list.next, struct net2280_request, completion_chain);
        list_del(dev->completion_list.next);
      }
      else {
        req = NULL;
      }
      
      spinlock_unlock_irq_restore(&dev->completion_list_lock, flags);
      
      if(req) {
        unsigned stopped = req->ep->stopped;
        req->ep->stopped = 1;

        /* -- EM -- Ugly hack right now because of lot of the stuff
           the completion functions call assume that the kernel lock
           is acquired, at least for now we will be preempted in
           between calls to complete */
        
        //cli();
        //lock_kernel();
        req->req.complete(&req->ep->ep, &req->req);
        //unlock_kernel();
        //sti();
        req->ep->stopped = stopped;
      }
      else {
        break;
      }
    }
    
    //cli();
    //lock_kernel();
#ifdef NET2280_IO_VCPU
    iovcpu_job_completion();
#else
    schedule();
#endif
    //unlock_kernel();
    //sti();
        
    
  }
}
#endif

static void net2280_init_thread(struct net2280* net2280_dev)
{
  USB_DEVICE_INFO* temp;
  
  if(net2280_start(&net2280_dev->gadget, &default_net2280_usb_gadget_driver) < 0) { 
    DLOG("net2280_start failed"); 
    panic("net2280_start failed");
  }

  temp = kmalloc(sizeof(USB_DEVICE_INFO));
  
  if(temp == NULL) {
    DLOG("Failed to malloc temp driver info");
    return;
  }

#if defined(NET2280_IO_VCPU) || defined(NET2280_MAIN_VCPU)
  net2280_dev->iovcpu =
    create_kernel_thread_args ((u32) net2280_bh_thread,
                               (u32) &net2280_dev->bh_stack[NET2280_IOC_BH_THREAD_STACK_SIZE-1],
                               "Net2280 Bottom Half", FALSE, 1, net2280_dev);
  DLOG("net2280_dev->iovcpu = 0x%X", net2280_dev->iovcpu->tid);
#  ifdef NET2280_IO_VCPU
  set_iovcpu (net2280_dev->iovcpu, IOVCPU_CLASS_USB);
#  endif
#endif

  net2280_dev_num = usb_register_device(temp, &net2280_driver,
#ifdef NET2280_MIGRATION_MODE
                                        "net2280_migration"
#else
                                        "net2280_communication"
#endif
                                        );
  
  if(net2280_dev_num < 0) {
    DLOG("Failed to register net2280 device");
    panic("Failed to register net2280 device");
  }

  DLOG("net2280_start done, device number = %d", net2280_dev_num);

#ifdef NET2280_MIGRATION_MODE
  
  create_kernel_thread_args((u32)net2280_migration_thread,
                            (u32) &migration_thread_stack[MIGRATION_THREAD_STACK_SIZE - 1],
                            "Net2280 Migration Thread",
                            TRUE, 0);  
#endif
}


static bool net2280_init(void)
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
    if (! (pci_irq_map_handler (&irq, net2280_irq_handler, get_logical_dest_addr (0),
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

  /* -- EM -- Right now since we don't have a namespace the gadget
     driver is piggy backing on hacked namespace we have for usb
     devices */
  
  
  

  

  DLOG("first phase initialisation done");

  DLOG("starting second phase of initialisation thread");

  create_kernel_thread_args((u32)net2280_init_thread,
                            (u32) &init_thread_stack[INIT_THREAD_STACK_SIZE - 1],
                            "Net2280 Init Thread",
                            TRUE, 1, &net2280_dev);  
  
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
