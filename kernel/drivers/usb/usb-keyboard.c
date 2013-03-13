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


#include <mem/virtual.h>
#include <drivers/usb/usb.h>
#include <util/printf.h>
#include <kernel.h>
#include <drivers/usb/usb-keyboard.h>
#include <sched/sched.h>

static usb_keyboard_dev_t* keyboard_dev = NULL;

//#define DEBUG_USB_KEYBOARD
//#define DEBUG_USB_KEYBOARD_VERBOSE

#ifdef DEBUG_USB_KEYBOARD
#define DLOG(fmt,...) DLOG_PREFIX("usb-keyboard",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef DEBUG_USB_KEYBOARD_VERBOSE
#define DLOGV(fmt,...) DLOG_PREFIX("usb-keyboard",fmt,##__VA_ARGS__)
#else
#define DLOGV(fmt, ...) ;
#endif

static const unsigned char usb_kbd_keycode[256] = {
  0,  0,  0,  0, 30, 48, 46, 32, 18, 33, 34, 35, 23, 36, 37, 38,
  50, 49, 24, 25, 16, 19, 31, 20, 22, 47, 17, 45, 21, 44,  2,  3,
  4,  5,  6,  7,  8,  9, 10, 11, 28,  1, 14, 15, 57, 12, 13, 26,
  27, 43, 43, 39, 40, 41, 51, 52, 53, 58, 59, 60, 61, 62, 63, 64,
  65, 66, 67, 68, 87, 88, 99, 70,119,110,102,104,111,107,109,106,
  105,108,103, 69, 98, 55, 74, 78, 96, 79, 80, 81, 75, 76, 77, 71,
  72, 73, 82, 83, 86,127,116,117,183,184,185,186,187,188,189,190,
  191,192,193,194,134,138,130,132,128,129,131,137,133,135,136,113,
  115,114,  0,  0,  0,121,  0, 89, 93,124, 92, 94, 95,  0,  0,  0,
  122,123, 90, 91, 85,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  29, 42, 56,125, 97, 54,100,126,164,166,165,163,161,115,114,113,
  150,158,159,128,136,177,178,176,142,152,173,140
};

void *memscan(void *addr, int c, size_t size)
{
  unsigned char *p = addr;
  
  while (size) {
    if (*p == c)
      return (void *)p;
    p++;
    size--;
  }
  return (void *)p;
}


static void report_key(usb_keyboard_dev_t* dev, uint code, uint press)
{
  new_keyboard_code(press ? code : 0x80 | code, TRUE);
}


static void usb_keyboard_callback(struct urb* urb)
{
  int bytes_freed;
  int new_bytes = usb_rt_update_data(urb, 8);
  static int next_bytes = 0;
  static int bytes_available = 0;
  int i;
  uint8 new[8];

  if(new_bytes != 8) {
    DLOG("Did not get eight bytes");
    panic("Did not get eight bytes");
  }

  bytes_available += new_bytes;
  memcpy(new, &keyboard_dev->buffer[next_bytes], 8);
  next_bytes += 8;
  if(next_bytes > USB_KEYBOARD_BUFFER_SIZE) {
    next_bytes -= USB_KEYBOARD_BUFFER_SIZE;
  }
  

  

  //for (i = 0; i < 8; i++)
  //  input_report_key(keyboard_dev, usb_kbd_keycode[i + 224], (new[0] >> i) & 1);
  
  for (i = 2; i < 8; i++) {
    
    if (keyboard_dev->old[i] > 3 && memscan(new + 2, keyboard_dev->old[i], 6) == new + 8) {
      if (usb_kbd_keycode[keyboard_dev->old[i]])
	report_key(keyboard_dev, usb_kbd_keycode[keyboard_dev->old[i]], 0);
      else
	DLOG("Unknown key (scancode %#x) released.\n", keyboard_dev->old[i]);
    }
    
    if (new[i] > 3 && memscan(keyboard_dev->old + 2, new[i], 6) == keyboard_dev->old + 8) {
      if (usb_kbd_keycode[new[i]])
	report_key(keyboard_dev, usb_kbd_keycode[new[i]], 1);
      else
	DLOG("Unknown key (scancode %#x) released.\n", new[i]);
    }
  }
  
  memcpy(keyboard_dev->old, new, 8);
  
  bytes_freed = usb_rt_free_data(urb, bytes_available);
  bytes_available = bytes_available - bytes_freed;
  
  
}

uint32_t temp_stack[1024];

static void keyboard_setup_thread()
{
  int res;
  uint pipe = usb_create_pipe(keyboard_dev->dev, &keyboard_dev->int_ep);

  DLOG("In %s", __FUNCTION__);
  DLOG("pipe = 0x%X", pipe);
  DLOG("pipe endpoint = %d", usb_pipeendpoint(pipe));
  DLOG("pipe addr = %d", usb_pipedevice(pipe));
  

  DLOG("keyboard_dev->int_ep.bInterval = %u", keyboard_dev->int_ep.bInterval);
  //keyboard_dev->int_ep.bInterval = keyboard_dev->int_ep.bInterval / 2;
  usb_fill_rt_int_urb(keyboard_dev->urb, keyboard_dev->dev, pipe, &keyboard_dev->buffer,
                      //(keyboard_dev->int_ep.wMaxPacketSize > 8
                      // ? 8 : keyboard_dev->int_ep.wMaxPacketSize),
                      USB_KEYBOARD_BUFFER_SIZE,
                      usb_keyboard_callback, NULL,
                      keyboard_dev->int_ep.bInterval,
                      //1,
                      8);
  while(1) {
    res = usb_submit_urb(keyboard_dev->urb, 0);
    DLOG("res = %d", res);
    if(res < 0) {
        if(res == -1) {
          /* Straight out failure */
          /* -- EM -- Normally just give up for keyboard but panic for now */
          DLOG("Failed to submit urb for keyboard");
          panic("Failed to submit urb for keyboard");
          break;
        }
        else {
          /* Try to submit again later */
          sched_usleep(500 * 1000);
        }
    }
    else {
      DLOG("Submitted urb for keyboard");
      break;
    }
  }
}

static bool
keyboard_probe(USB_DEVICE_INFO *dev, USB_CFG_DESC *cfg, USB_IF_DESC *ifd)
{
  USB_EPT_DESC* int_ep;
  char* temp;
  
  if(keyboard_dev) {
    DLOG("Only 1 usb keyboard allowed right now");
    return FALSE;
  }



  if(ifd->bInterfaceClass != 0x3 ||
     ifd->bInterfaceSubClass != 0x1 ||
     ifd->bInterfaceProtocol != 0x1) return FALSE;

  keyboard_dev = kmalloc(sizeof(usb_keyboard_dev_t));

  init_usb_keyboard_dev(keyboard_dev);
  
  if(keyboard_dev == NULL) {
    DLOG("Failed to allocate keyboard dev");
    panic("Failed to allocate keyboard dev");
  }
  temp = kmalloc(cfg->wTotalLength);
  if(temp == NULL) {
    DLOG("Failed to allocate temp");
    panic("Failed to allocate temp");
  }

  usb_get_descriptor(dev, USB_TYPE_CFG_DESC, 0, 0, cfg->wTotalLength, (addr_t)temp);

  print_all_descriptors(temp, cfg->wTotalLength);
  DLOG("ifd->bAlternateSetting = %d", ifd->bAlternateSetting);
  int_ep = get_next_desc(USB_TYPE_EPT_DESC, temp, cfg->wTotalLength);
  
  print_ept_desc_info(int_ep);
  if(int_ep == NULL || (usb_get_endpoint_transfer_type(int_ep) != PIPE_INTERRUPT)) {
    DLOG("Failed to find interrupt endpoint");
    kfree(temp);
    return FALSE;
  }

  DLOG("cfg->bConfigurationValue = 0x%X", cfg->bConfigurationValue);
  if(usb_set_configuration (dev, cfg->bConfigurationValue) < 0) {
    DLOG("Failed to set configuration");
    return FALSE;
  }
  
  uint pipe = usb_create_pipe(dev, int_ep);
  if(!usb_pipein(pipe)) {
    DLOG("keyboard endpoint should be an input endpoint");
    panic("keyboard endpoint should be an input endpoint");
  }

  
  //input_register_device(keyboard_dev);
  
  keyboard_dev->dev = dev;
  keyboard_dev->int_ep = *int_ep;
  dev->ep_in[int_ep->bEndpointAddress & 0xF].desc = *int_ep;
  keyboard_dev->urb = usb_alloc_urb(0, 0);
  if(keyboard_dev->urb == NULL) {
    DLOG("Failed to allocate urb");
    panic("Failed to allocate urb");
  }
  
  create_kernel_thread_args((u32)keyboard_setup_thread,
                            (u32) &temp_stack[1023],
                            "USB Keyboard",
                            TRUE, 0);

  return TRUE;
}


#include "module/header.h"

USB_DRIVER_INIT(keyboard, "USB Keyboard Driver", keyboard_probe, NULL, NULL, NULL, NULL)


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
