#ifndef _UVC_H_
#define _UVC_H_

#include <types.h>

#define TYPE_IA_DESC    0x0B

/*
 * UVC_IA_DESC : Standard Video Interface Collection IAD
 *
 * Reference :
 *     USB Device Class Definition for Video Devices
 *     Revision 1.1, Page 48
 */
struct uvc_ia_desc
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bFirstInterface;
  uint8_t bInterfaceCount;
  uint8_t bFunctionClass;
  uint8_t bFunctionSubClass;
  uint8_t bFunctionProtocol;
  uint8_t iFunction;
} __attribute__ ((packed));

typedef struct uvc_ia_desc UVC_IA_DESC;

struct uvc_csvc_if_hdr_desc
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubType;
  uint16_t bcdUVC;
  uint16_t wTotalLength;
  uint32_t dwClockFrequency;
  uint8_t bInCollection;
  uint8_t baInterface1;
} __attribute__ ((packed));

typedef struct uvc_csvc_if_hdr_desc UVC_CSVC_IF_HDR_DESC;

extern int uvc_read (void *, int);
extern int uvc_init (void);

#endif

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
