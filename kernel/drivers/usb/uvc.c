/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
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

#include <drivers/usb/usb.h>
#include <drivers/usb/uhci.h>
#include <drivers/usb/uvc.h>
#include <arch/i386.h>
#include <util/printf.h>
#include <kernel.h>

#define DEBUG_UVC

#ifdef DEBUG_UVC

#define DLOG(fmt,...) DLOG_PREFIX("uvc",fmt,##__VA_ARGS__)

#define printf com1_printf
#define print com1_puts
#define putx  com1_putx
#define putchar com1_putc

#else

#define DLOG(fmt,...) ;
#endif

struct iso_data_source
{
  uint8_t endp;
  uint16_t max_packet;
  uint32_t sample_size;
};

typedef struct iso_data_source ISO_DATA_SRC;

static ISO_DATA_SRC iso_src;
static uint8_t frame_buf[38400];
static uint8_t jpeg_frame[38400];
//static uint32_t uvc_test_stack[1024];

bool usb_uvc_driver_init (void);
static bool uvc_probe (USB_DEVICE_INFO *, USB_CFG_DESC *, USB_IF_DESC *);
static bool uvc_init(USB_DEVICE_INFO *, USB_CFG_DESC *);
static int video_probe_controls (USB_DEVICE_INFO *, uint8_t, uint8_t,
    UVC_VS_CTL_PAR_BLOCK *);
static int video_commit_controls (USB_DEVICE_INFO *, uint8_t, uint8_t,
    UVC_VS_CTL_PAR_BLOCK *);
static int video_ctl_error_code (USB_DEVICE_INFO *, uint8_t, uint8_t);
static void para_block_dump (UVC_VS_CTL_PAR_BLOCK *);
int uvc_get_frame (USB_DEVICE_INFO *, ISO_DATA_SRC *, uint8_t *,
    uint32_t, uint8_t *, uint32_t *);
static void uvc_test (void);
static int uvc_device_cfg (USB_DEVICE_INFO *, USB_CFG_DESC *,
    ISO_DATA_SRC *);

static USB_DEVICE_INFO gdev; // --??-- Remove after test

static void
uvc_test (void)
{
  DLOG("UVC Test starts!");
  uint32_t frm_len;
  int i = 0;
  uint32_t *dump = (uint32_t*)jpeg_frame;

  memset(frame_buf, 0, 38400);
  memset(jpeg_frame, 0, 38400);
  uvc_get_frame (&gdev, &iso_src, frame_buf, 384, jpeg_frame, &frm_len);

  DLOG("Getting the first frame : %d bytes", frm_len);

  DLOG("Getting the second frame : %d bytes", frm_len);
  memset(frame_buf, 0, 38400);
  memset(jpeg_frame, 0, 38400);
  uvc_get_frame (&gdev, &iso_src, frame_buf, 384, jpeg_frame, &frm_len);

  DLOG("Getting the third frame : %d bytes", frm_len);
  memset(frame_buf, 0, 38400);
  memset(jpeg_frame, 0, 38400);
  uvc_get_frame (&gdev, &iso_src, frame_buf, 384, jpeg_frame, &frm_len);

  DLOG("Dumping the frame:");

  for(i = 1; i <= (frm_len / 4 + 1); i++) {
    putx(*dump);
    dump++;
    if ((i % 6) == 0) putchar('\n');
    if ((i % 96) == 0) putchar('\n');
  }

  //for (;;)
  //  delay (1000);
}

int
uvc_get_frame (
    USB_DEVICE_INFO * dev,
    ISO_DATA_SRC * iso_src,
    uint8_t * buf,
    uint32_t transfer_len,
    uint8_t * frame,
    uint32_t * frm_len)
{
  int status = 0, i = 0, j = 0, act_len = 0;
  uint8_t * index;
  int counter = 0, header_len = 0;

  //DLOG("In uvc_get_frame!");
  *frm_len = 0;

  for (i = 0; i < 1024; i++) {
    index = buf + i * transfer_len;

    status += uhci_isochronous_transfer (dev->address, iso_src->endp,
        (addr_t) index, transfer_len, &act_len, i, DIR_IN, 0);

    header_len = *(index);
    *frm_len += (act_len - header_len);

#if 0
    DLOG("Actual length received : %d, header : %d, frame : %d",
        act_len, header_len, *frm_len);
#endif

    for (j = 0; j < (act_len - header_len); j++) {
      *(frame + counter) = *(index + header_len + j);
      counter ++;
    }

    /* Now, we only retreive one frame. Check EOF in the header */
    if ((*(index + 1)) & 0x02) {
      //DLOG("End of frame reached!");
      break;
    }
  }

  return status;
}

static bool
uvc_init (USB_DEVICE_INFO * dev, USB_CFG_DESC * cfg)
{
  USB_SPD_CFG_DESC *scfgd;
  UVC_VS_CTL_PAR_BLOCK par;
  uint8_t tmp[1300];

  DLOG("Configuring UVC device ...");

  gdev = *dev; // --??-- Remove after test

  if (uvc_device_cfg(dev, cfg, &iso_src) == -1) {
    DLOG("Device Configuration Failed!");
    return FALSE;
  }

  /* Set data source info manually for now */
  iso_src.endp = 1;
  iso_src.max_packet = 512;
  iso_src.sample_size = 0;

  memset(tmp, 0, 1300);
  memset(&par, 0, sizeof(UVC_VS_CTL_PAR_BLOCK));

  DLOG("Now, getting other speed configuration.");
  if (usb_get_descriptor(dev, USB_TYPE_SPD_CFG_DESC, 0, 0, 9, (addr_t)tmp)) {
    DLOG("Other Speed Configuration is not presented.");
  } else {
    scfgd = (USB_SPD_CFG_DESC *)tmp;
    DLOG("Other Speed Configuration:");
    DLOG("  bLength : 0x%x  bDescriptorType : 0x%x  wTotalLength : 0x%x",
        scfgd->bLength, scfgd->bDescriptorType, scfgd->wTotalLength);
    DLOG("  bNumInterfaces : 0x%x  bConfigurationValue : 0x%x",
        scfgd->bNumInterfaces, scfgd->bConfigurationValue);
  }

  /* There is only one configuration in Logitech Webcam Pro 9000 */
  DLOG("Set configuration to %d.", cfg->bConfigurationValue);
  usb_set_configuration(dev, cfg->bConfigurationValue);

  /* Now, negotiate with VS interface for streaming parameters */

  /* Manually set parameters */
  par.bmHint = 1; // Frame Interval Fixed
  par.bFormatIndex = 2;
  par.bFrameIndex = 2;
  par.dwFrameInterval = 0x61A80; // 25FPS
  //par.dwFrameInterval = 0xF4240; // 10FPS
  //par.wCompQuality = 5000; // 1 - 10000, with 1 the lowest
  //par.dwMaxPayloadTransferSize = 512;

  if (video_probe_controls (dev, SET_CUR, 1, &par)) {
    DLOG("Initial negotiation failed during probe");
  }

  memset(&par, 0, sizeof(UVC_VS_CTL_PAR_BLOCK));

  if (video_probe_controls (dev, GET_CUR, 1, &par)) {
    DLOG("Getting current state during probe failed");
  }
  para_block_dump (&par);

  if (video_commit_controls (dev, SET_CUR, 1, &par)) {
    DLOG("Setting device state during probe failed");
  }
  memset(&par, 0, sizeof(UVC_VS_CTL_PAR_BLOCK));

  if (video_commit_controls (dev, GET_CUR, 1, &par)) {
    DLOG("Setting device state during probe failed");
  }
  para_block_dump (&par);

  /* Select Alternate Setting 2 for interface 1 (Std VS interface) */
  /* This is for an isochronous endpoint with MaxPacketSize 384 */
  DLOG("Select Alternate Setting 2 for VS interface");
  if (usb_set_interface(dev, 2, 1)) {
    DLOG("Cannot configure interface setting for Std VS interface");
    return FALSE;
  }

  delay(1000);
  delay(1000);
  delay(1000);
  delay(1000);
  delay(1000);
  delay(1000);

  return TRUE;
}

static bool
uvc_probe (USB_DEVICE_INFO *dev, USB_CFG_DESC *cfg, USB_IF_DESC *ifd)
{
  /* Avoid multi-entrance in uhci_enumerate(), should be removed soon */
  static int entrance = 0;
  if (entrance) return FALSE;

  /* For now, we only support device with multi video interface
   * collections. This is ugly. But let's make Logitech Webcam Pro
   * 9000 work first.
   */
  if(!(dev->devd.bDeviceClass == 0xEF) ||
     !(dev->devd.bDeviceSubClass == 0x02) ||
     !(dev->devd.bDeviceProtocol == 0x01)) {
    return FALSE;
  }

#if 0
  /* Dumping UVC device descriptors */
  desc_dump (dev, cfg);
#endif

  if (uvc_init(dev, cfg) == FALSE) {
    DLOG("Device configuration failed!");
    return FALSE;
  }

  //start_kernel_thread((uint)uvc_test, (uint)&uvc_test_stack[1023]);
  //uvc_test ();

#if 0
  int i = 0;
  uint32_t *dump = (uint32_t*)frame_buf;
  memset(frame_buf, 0, 38400);

  uvc_get_frame (dev, &iso_src, (addr_t) frame_buf, 384);

  for(i = 1; i <= 9600; i++) {
    putx(*dump);
    dump++;
    if ((i % 6) == 0) putchar('\n');
    if ((i % 96) == 0) putchar('\n');
  }
#endif

  /* Avoid multi-entrance in uhci_enumerate(), should be removed soon */
  entrance++;

  return TRUE;
}

static int
video_ctl_error_code (
    USB_DEVICE_INFO * dev,
    uint8_t request,
    uint8_t interface)
{
  USB_DEV_REQ setup_req;
  uint8_t code, status;

  setup_req.bmRequestType = 0xA1;
  setup_req.bRequest = request;
  setup_req.wValue = VC_REQUEST_ERROR_CODE_CONTROL << 8;
  setup_req.wIndex = interface;
  setup_req.wLength = 1;

  status = usb_control_transfer (dev, (addr_t) & setup_req,
      sizeof (USB_DEV_REQ), (addr_t) & code, 1);

  if (status) {DLOG("Getting error code failed!"); return 0xFF;}

  return code;
}

static int
video_probe_controls (
    USB_DEVICE_INFO * dev,
    uint8_t request,
    uint8_t interface,
    UVC_VS_CTL_PAR_BLOCK * par)
{
  USB_DEV_REQ setup_req;
  if (request == SET_CUR)
    setup_req.bmRequestType = 0x21;
  else
    setup_req.bmRequestType = 0xA1;
  setup_req.bRequest = request;
  setup_req.wValue = VS_PROBE_CONTROL << 8;
  setup_req.wIndex = interface;
  //setup_req.wLength = sizeof(UVC_VS_CTL_PAR_BLOCK);
  setup_req.wLength = 26; // Why? It should be 34! Odd...

  return usb_control_transfer (dev, (addr_t) & setup_req,
      sizeof (USB_DEV_REQ), (addr_t) par, setup_req.wLength);
}

static int
video_commit_controls (
    USB_DEVICE_INFO * dev,
    uint8_t request,
    uint8_t interface,
    UVC_VS_CTL_PAR_BLOCK * par)
{
  USB_DEV_REQ setup_req;
  if (request == SET_CUR)
    setup_req.bmRequestType = 0x21;
  else
    setup_req.bmRequestType = 0xA1;
  setup_req.bRequest = request;
  setup_req.wValue = VS_COMMIT_CONTROL << 8;
  setup_req.wIndex = interface;
  //setup_req.wLength = sizeof(UVC_VS_CTL_PAR_BLOCK);
  setup_req.wLength = 26; // Why? It should be 34! Odd...

  return usb_control_transfer (dev, (addr_t) & setup_req,
      sizeof (USB_DEV_REQ), (addr_t) par, setup_req.wLength);
}

static USB_DRIVER uvc_driver = {
  .probe = uvc_probe
};

bool
usb_uvc_driver_init (void)
{
  return usb_register_driver (&uvc_driver);
}

static void
para_block_dump (UVC_VS_CTL_PAR_BLOCK * par)
{
  DLOG ("Parameter Block Dump: ");
  DLOG ("  bmHint : 0x%x", par->bmHint);
  DLOG ("  bFormatIndex : 0x%x", par->bFormatIndex);
  DLOG ("  bFrameIndex : 0x%x", par->bFrameIndex);
  DLOG ("  dwFrameInterval : 0x%x", par->dwFrameInterval);
  DLOG ("  wKeyFrameRate : 0x%x", par->wKeyFrameRate);
  DLOG ("  wPFrameRate : 0x%x", par->wPFrameRate);
  DLOG ("  wCompQuality : 0x%x", par->wCompQuality);
  DLOG ("  wCompWindowSize : 0x%x", par->wCompWindowSize);
  DLOG ("  wDelay : 0x%x", par->wDelay);
  DLOG ("  dwMaxVideoFrameSize : 0x%x", par->dwMaxVideoFrameSize);
  DLOG ("  dwMaxPayloadTransferSize : 0x%x", par->dwMaxPayloadTransferSize);
  DLOG ("  dwClockFrequency : 0x%x", par->dwClockFrequency);
  DLOG ("  bmFramingInfo : 0x%x", par->bmFramingInfo);
  DLOG ("  bPreferedVersion : 0x%x", par->bPreferedVersion);
  DLOG ("  bMinVersion : 0x%x", par->bMinVersion);
  DLOG ("  bMaxVersion : 0x%x", par->bMaxVersion);
}

static int
uvc_device_cfg (
    USB_DEVICE_INFO *dev,
    USB_CFG_DESC *cfg,
    ISO_DATA_SRC * src)
{
  /* Dump all the descriptors retreived from UVC device */
  uint8_t conf[1300];
  int index = 0, tmp_index = 0;

  USB_DEV_DESC *desc;
  UVC_IA_DESC *iad;
  USB_IF_DESC *vcifd, *vsifd;
  UVC_CSVC_IF_HDR_DESC *csvcifd;
  UVC_CSVS_IF_HDR_DESC *csvsifd;
  UVC_IN_TERM_DESC *intd;
  UVC_OUT_TERM_DESC *outd;
  UVC_MJPEG_FORMAT_DESC *jpegfd;
  UVC_MJPEG_FRAME_DESC *jpegrd;
  USB_EPT_DESC *videoept;

  typedef struct uvc_desc_idx {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
  } UVC_DESC_IDX;

  UVC_DESC_IDX * desc_idx;

  desc = &(dev->devd);
  usb_get_descriptor(dev, USB_TYPE_CFG_DESC, 0, 0, cfg->wTotalLength, (addr_t)conf);

  /* Parsing all UVC specific descriptors */
  /* Get the first IAD */
  DLOG("Getting IAD ...");
  index += cfg->bLength;
  iad = (UVC_IA_DESC*)(&conf[index]);

  if ((iad->bDescriptorType != USB_TYPE_IA_DESC) ||
      (iad->bFunctionClass != CC_VIDEO) ||
      (iad->bFunctionSubClass != SC_VIDEO_INTERFACE_COLLECTION)) {
    DLOG("IAD not found after configuration descriptor.");
    return -1;
  } else {
    DLOG("Found Video Interface Collection");
    DLOG("  bFirstInterface : 0x%x", iad->bFirstInterface);
    DLOG("  bInterfaceCount : 0x%x", iad->bInterfaceCount);
  }

  /* Get First VC Interface Descriptor */
  DLOG("Getting VC Interface Descriptor ...");
  while (index < cfg->wTotalLength) {
    desc_idx = (UVC_DESC_IDX *)(&conf[index]);
    if (desc_idx->bDescriptorType != USB_TYPE_IF_DESC)
      index += desc_idx->bLength;
    else
      break;
  }

  if (index >= cfg->wTotalLength) {
    DLOG("Descriptor parsing error!");
    return -1;
  }

  vcifd = (USB_IF_DESC *)(&conf[index]);

  if ((vcifd->bDescriptorType != USB_TYPE_IF_DESC) ||
      (vcifd->bInterfaceClass != CC_VIDEO) ||
      (vcifd->bInterfaceSubClass != SC_VIDEOCONTROL)) {
    DLOG("Standard VC Interface Descriptor cannot be found");
    return -1;
  } else {
    DLOG("Found Standard VC Interface");
    DLOG("  bInterfaceNumber : 0x%x", vcifd->bInterfaceNumber);
    DLOG("  bAlternateSetting : 0x%x", vcifd->bAlternateSetting);
    if (vcifd->bNumEndpoints)
      DLOG("  Status interrupt endpoint is present");
    else
      DLOG("  Status interrupt endpoint is not present");
  }

  /* Get Class-specific VC Interface header descriptor */
  DLOG("Getting Class-specific VC Interface Descriptor ...");
  while (index < cfg->wTotalLength) {
    desc_idx = (UVC_DESC_IDX *)(&conf[index]);
    if ((desc_idx->bDescriptorType != CS_INTERFACE) ||
        (desc_idx->bDescriptorSubType != VC_HEADER))
      index += desc_idx->bLength;
    else
      break;
  }

  if (index >= cfg->wTotalLength) {
    DLOG("Descriptor parsing error!");
    return -1;
  }

  csvcifd = (UVC_CSVC_IF_HDR_DESC *)(&conf[index]);

  DLOG("Found Class-specific VC Interface Header");
  DLOG("  wTotalLength : 0x%x", csvcifd->wTotalLength);
  DLOG("  bInCollection : 0x%x", csvcifd->bInCollection);

  /* Get Input and Output Terminals */
  DLOG("Getting Input and Output Terminals ...");
  tmp_index = index;
  index += csvcifd->wTotalLength;
  while (tmp_index < index) {
    desc_idx = (UVC_DESC_IDX *)(&conf[tmp_index]);

    if ((desc_idx->bDescriptorType == CS_INTERFACE) &&
        (desc_idx->bDescriptorSubType == VC_INPUT_TERMINAL)) {
      intd = (UVC_IN_TERM_DESC *)(&conf[tmp_index]);
      DLOG("Found Input Terminal");
      DLOG("  bTerminalID : 0x%x", intd->bTerminalID);
      DLOG("  wTerminalType : 0x%x", intd->wTerminalType);
      DLOG("  bAssocTerminal : 0x%x", intd->bAssocTerminal);
    }

    if ((desc_idx->bDescriptorType == CS_INTERFACE) &&
        (desc_idx->bDescriptorSubType == VC_OUTPUT_TERMINAL)) {
      outd = (UVC_OUT_TERM_DESC *)(&conf[tmp_index]);
      DLOG("Found Output Terminal");
      DLOG("  bTerminalID : 0x%x", outd->bTerminalID);
      DLOG("  wTerminalType : 0x%x", outd->wTerminalType);
      DLOG("  bAssocTerminal : 0x%x", outd->bAssocTerminal);
      DLOG("  bSourceID : 0x%x", outd->bSourceID);
    }

    /* Multiple Processing or Extension Units may exist here */

    tmp_index += desc_idx->bLength;
  }

  /* We may have Standard and Class-specific Interrupt Endpoint
   * Descriptors after the Class-specific VC Interface Descriptor.
   * Just ignore them for now!
   */

  /* Get Standard VS Interface */
  DLOG("Getting Standard VS Interface ...");
  while (index < cfg->wTotalLength) {
    desc_idx = (UVC_DESC_IDX *)(&conf[index]);
    if (desc_idx->bDescriptorType != USB_TYPE_IF_DESC)
      index += desc_idx->bLength;
    else
      break;
  }

  if (index >= cfg->wTotalLength) {
    DLOG("Descriptor parsing error!");
    return -1;
  }

  vsifd = (USB_IF_DESC *)(&conf[index]);

  if ((vsifd->bDescriptorType != USB_TYPE_IF_DESC) ||
      (vsifd->bInterfaceClass != CC_VIDEO) ||
      (vsifd->bInterfaceSubClass != SC_VIDEOSTREAMING)) {
    DLOG("Standard VS Interface Descriptor cannot be found");
    return -1;
  } else {
    DLOG("Found Standard VS Interface");
    DLOG("  bInterfaceNumber : 0x%x", vsifd->bInterfaceNumber);
    DLOG("  bAlternateSetting : 0x%x", vsifd->bAlternateSetting);
    DLOG("  bNumEndpoints : 0x%x", vsifd->bNumEndpoints);
  }

  /* Get Class-Specific VS Interface Input Header Descriptor */
  DLOG("Getting Class-Specific VS Interface Input Header ...");
  while (index < cfg->wTotalLength) {
    desc_idx = (UVC_DESC_IDX *)(&conf[index]);
    if ((desc_idx->bDescriptorType != CS_INTERFACE) ||
        (desc_idx->bDescriptorSubType != VS_INPUT_HEADER))
      index += desc_idx->bLength;
    else
      break;
  }

  if (index >= cfg->wTotalLength) {
    DLOG("Descriptor parsing error!");
    return -1;
  }

  csvsifd = (UVC_CSVS_IF_HDR_DESC *)(&conf[index]);

  DLOG("Found Class-specific VS Interface Input Header");
  DLOG("  bNumFormats : 0x%x", csvsifd->bNumFormats);
  DLOG("  wTotalLength : 0x%x", csvsifd->wTotalLength);
  DLOG("  bEndpointAddress : 0x%x", csvsifd->bEndpointAddress);
  DLOG("  bTerminalLink : 0x%x", csvsifd->bTerminalLink);

  /* Try to get MJPEG format and frame info */
  DLOG("Getting MJPEG Format and Frame Info ...");
  tmp_index = index;
  index += csvsifd->wTotalLength;

  while (tmp_index < index) {
    desc_idx = (UVC_DESC_IDX *)(&conf[tmp_index]);
    if ((desc_idx->bDescriptorType != CS_INTERFACE) ||
        (desc_idx->bDescriptorSubType != VS_FORMAT_MJPEG)) {
      tmp_index += desc_idx->bLength;
    } else break;
  }

  jpegfd = (UVC_MJPEG_FORMAT_DESC *)(&conf[tmp_index]);

  if (tmp_index >= index) {
    DLOG("Motion-JPEG format is not supported");
    return -1;
  } else {
    DLOG("Motion-JPEG format is supported");
    DLOG("  bFormatIndex : 0x%x", jpegfd->bFormatIndex);
    DLOG("  bNumFrameDescriptors : 0x%x", jpegfd->bNumFrameDescriptors);
    DLOG("  bDefaultFrameIndex : 0x%x", jpegfd->bDefaultFrameIndex);
    DLOG("  bCopyProtect : 0x%x", jpegfd->bCopyProtect);
  }

  tmp_index += jpegfd->bLength;

  /* Get all the frame descriptors pertaining to the format */
  while (tmp_index < index) {
    desc_idx = (UVC_DESC_IDX *)(&conf[tmp_index]);
    if ((desc_idx->bDescriptorType == CS_INTERFACE) &&
        (desc_idx->bDescriptorSubType == VS_FRAME_MJPEG)) {
      jpegrd = (UVC_MJPEG_FRAME_DESC *)(&conf[tmp_index]);

      DLOG("Found Motion-JPEG Frame Descriptor");
      DLOG("  bFrameIndex : 0x%x", jpegrd->bFrameIndex);
      DLOG("  The Minimum Bit Rate : %d", jpegrd->dwMinBitRate);
      DLOG("  The Maximum Bit Rate : %d", jpegrd->dwMaxBitRate);
      DLOG("  Maximum Frame Buffer : %d bytes",
          jpegrd->dwMaxVideoFrameBufferSize);
      DLOG("  The default interval : %d ns",
          jpegrd->dwDefaultFrameInterval * 100);
      DLOG("  The supported resolution : %d x %d",
          jpegrd->wWidth, jpegrd->wHeight);
    }

    tmp_index += desc_idx->bLength;
  }

  /* We could have parsed the Still Image Frame descriptor
   * and Color Matching Descriptor here. But for simplicity,
   * just skip them.
   */

  /* Get all the alternate settings for the VS interface above */
  DLOG("Getting possible alternate settings for VS interface ...");
  while (index < cfg->wTotalLength) {
    desc_idx = (UVC_DESC_IDX *)(&conf[index]);

    if (desc_idx->bDescriptorType == USB_TYPE_IA_DESC) {
      DLOG("Reach the end of the current VS interface");
      break;
    }

    if (desc_idx->bDescriptorType == USB_TYPE_IF_DESC) {
      vsifd = (USB_IF_DESC *)(&conf[index]);

      DLOG("Found Standard VS Interface");
      DLOG("  bInterfaceNumber : 0x%x", vsifd->bInterfaceNumber);
      DLOG("  bAlternateSetting : 0x%x", vsifd->bAlternateSetting);
      DLOG("  bNumEndpoints : 0x%x", vsifd->bNumEndpoints);
    }

    if (desc_idx->bDescriptorType == USB_TYPE_EPT_DESC) {
      videoept = (USB_EPT_DESC *)(&conf[index]);

      DLOG("Found Video Data Endpoint");
      DLOG("  bEndpointAddress : 0x%x", videoept->bEndpointAddress);
      DLOG("  bmAttributes : 0x%x", videoept->bmAttributes);
      if ((videoept->bmAttributes & 0xFC) == 1)
        DLOG("  This is an isochronous end point"); 
      if ((videoept->bmAttributes & 0xFC) == 2)
        DLOG("  This is a bulk end point"); 
      DLOG("  The max packet size is : %d bytes",
          videoept->wMaxPacketSize);
      DLOG("  bInterval : 0x%x", videoept->bInterval);
    }

    index += desc_idx->bLength;
  }

  DLOG("UVC device configuration finished");

  return 0;
}

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
