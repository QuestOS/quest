#ifndef _LINUX_USB_H_
#define _LINUX_USB_H_

struct usb_iso_packet_descriptor {
  unsigned int offset;
  unsigned int length;            /* expected length */
  unsigned int actual_length;
  int status;
};

typedef struct usb_iso_packet_descriptor usb_iso_packet_descriptor_t;


#endif // _LINUX_USB_H_
