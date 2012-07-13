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

#include <arch/i386.h>
#include <drivers/usb/usb.h>
#include <drivers/usb/uvc.h>
#include <util/printf.h>
#include <kernel.h>
#include <sched/sched.h>

//#define DEBUG_UVC
//#define DEBUG_UVC_VERBOSE

#ifdef DEBUG_UVC

#define DLOG(fmt,...) DLOG_PREFIX("uvc",fmt,##__VA_ARGS__)

#define printf com1_printf
#define print com1_puts
#define putx  com1_putx
#define putchar com1_putc

#else

#define DLOG(fmt,...) ;
#endif

#ifdef DEBUG_UVC_VERBOSE

#define DLOGV(fmt,...) DLOG_PREFIX("uvc",fmt,##__VA_ARGS__)

#else

#define DLOGV(fmt, ...) ;

#endif

#define BUF_SIZE        (403600 * 1)
#define PACKET_SIZE     192

#define GET_FRAME_NUM_PACKETS ((BUF_SIZE / PACKET_SIZE) - 1)

unsigned int get_frame_actual_lens[GET_FRAME_NUM_PACKETS];
int          get_frame_statuses   [GET_FRAME_NUM_PACKETS];

struct iso_data_source
{
  uint8_t endp;
  uint16_t max_packet;
  uint32_t sample_size;
};

typedef struct iso_data_source ISO_DATA_SRC;

static ISO_DATA_SRC iso_src;
static uint8_t frame_buf[BUF_SIZE];
static uint8_t jpeg_frame[BUF_SIZE];
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


static int uvc_read(USB_DEVICE_INFO* device, char* buf, int data_len)
{
  
  uint32_t frm_len;
  int i = 0;
  uint32_t *dump = (uint32_t*)jpeg_frame;

  DLOG("%s called\n   buf = 0x%X\n   data_len = %d", __FUNCTION__, buf, data_len);

  memset(frame_buf, 0, BUF_SIZE);
  memset(jpeg_frame, 0, BUF_SIZE);
  uvc_get_frame (&gdev, &iso_src, frame_buf, PACKET_SIZE, jpeg_frame, &frm_len);
  DLOG("Getting a frame : %d bytes", frm_len);

#if 1
  DLOG("Dumping the frame:");
  
  for(i = 1; i <= (frm_len / 4 + 1); i++) {
    putx(*dump);
    dump++;
    if ((i % 6) == 0) putchar('\n');
    if ((i % 96) == 0) putchar('\n');
  }
  putchar('\n');
  putchar('\n');
  putchar('\n');
  putchar('\n');
#endif
  return -1;
  
}

static int uvc_write(USB_DEVICE_INFO* device, char* buf, int data_len)
{
  UVC_VS_CTL_PAR_BLOCK par;
  //DLOG("%s called", __FUNCTION__);
  memset(&par, 0, sizeof(UVC_VS_CTL_PAR_BLOCK));
  if (video_probe_controls (device, GET_CUR, 1, &par)) {
    DLOG("Getting current state during probe failed");
  }
  DLOG("done with write about to dump par");
  para_block_dump (&par);
  return -1;
}

static USB_DRIVER uvc_driver = {
  .probe = uvc_probe,
  .read = uvc_read,
  .write = uvc_write
};


static void
uvc_test (void)
{
  DLOG("UVC Test starts!");
  uint32_t frm_len;
  int i = 0;
  uint32_t *dump = (uint32_t*)jpeg_frame;

  memset(frame_buf, 0, BUF_SIZE);
  memset(jpeg_frame, 0, BUF_SIZE);
  uvc_get_frame (&gdev, &iso_src, frame_buf, PACKET_SIZE, jpeg_frame, &frm_len);
  DLOG("Getting the first frame : %d bytes", frm_len);
  
  memset(frame_buf, 0, BUF_SIZE);
  memset(jpeg_frame, 0, BUF_SIZE);
  uvc_get_frame (&gdev, &iso_src, frame_buf, PACKET_SIZE, jpeg_frame, &frm_len);
  DLOG("Getting the second frame : %d bytes", frm_len);

  memset(frame_buf, 0, BUF_SIZE);
  memset(jpeg_frame, 0, BUF_SIZE);
  uvc_get_frame (&gdev, &iso_src, frame_buf, PACKET_SIZE, jpeg_frame, &frm_len);
  DLOG("Getting the third frame : %d bytes", frm_len);

  DLOG("Dumping the frame:");

  for(i = 1; i <= (frm_len / 4 + 1); i++) {
    putx(*dump);
    dump++;
    if ((i % 6) == 0) putchar('\n');
    if ((i % 96) == 0) putchar('\n');
  }

  
  for (;;)
    delay (1000);
}

int
uvc_get_frame (USB_DEVICE_INFO * dev,
               ISO_DATA_SRC * iso_src,
               uint8_t * buf,
               uint32_t transfer_len,
               uint8_t * frame,
               uint32_t * frm_len)
{

  int status = 0, i = 0, j = 0;
  int counter = 0, header_len = 0;
  uint8_t header_bitfield;
  bool saving_data = FALSE;
  //DLOG("uvc_get_frame called");
    
  DLOG("In uvc_get_frame!");
  *frm_len = 0;
  DLOG("iso_src->endp = %d", iso_src->endp);
  if(usb_isochronous_msg(dev, usb_rcvisocpipe(dev, iso_src->endp),
                         buf, transfer_len, GET_FRAME_NUM_PACKETS,
                         get_frame_actual_lens,
                         get_frame_statuses,
                         USB_DEFAULT_ISOCHRONOUS_MSG_TIMEOUT) < 0) {
    DLOG("uvc_get_frame: usb_isochronous_transfer failed");
    return -1;
  }
  /*
  if(usb_isochronous_transfer(dev, iso_src->endp, transfer_len, USB_DIR_IN,
                              buf, iso_packets, GET_FRAME_NUM_PACKETS) < 0) {
 
  }
  */
  
  
  
  //DLOG("GET_FRAME_NUM_PACKETS = %d", GET_FRAME_NUM_PACKETS);
  
  for(i = 0; i < GET_FRAME_NUM_PACKETS; ++i) {
    //DLOG("loop iteration %d", i);
    //DLOG("packet %d virt addr = 0x%p", i, &(iso_packets[i]));
    if(get_frame_actual_lens[i] == 0) {
      //DLOG("%d: received zero length frame", i);
    }
    else {
      header_len = *(buf + (i * transfer_len));
      
      if(saving_data) {
        if(get_frame_actual_lens[i] > header_len) {
          *frm_len += (get_frame_actual_lens[i] - header_len);
          
          
          if(get_frame_actual_lens[i] > PACKET_SIZE ||
             get_frame_actual_lens[i] < 0) {
            DLOG("error with transaction %d", i);
            while(1);
          }
          for (j = 0; j < (get_frame_actual_lens[i] - header_len); j++) {
            *(frame + counter) = *(buf + (i * transfer_len) + header_len + j);
            counter++;
          }
        }
        
        /* Now, we only retrieve one frame. Check EOF in the header */
      }
      if ((*(buf + (i * transfer_len) + 1)) & 0x02) {
        DLOG("End of frame reached!");
        if(saving_data) {
          DLOG("frm_len = %d", *frm_len);
          break;
        }
        else {
          saving_data = TRUE;
        }
      }
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
  
  
  if (uvc_device_cfg(dev, cfg, &iso_src) == -1) {
    DLOG("Device Configuration Failed!");
    return FALSE;
  }

  /* Set data source info manually for now */
  iso_src.endp = 1;
  iso_src.max_packet = PACKET_SIZE;
  iso_src.sample_size = 0;
  dev->ep_in[1].desc.wMaxPacketSize = PACKET_SIZE;

  
  memcpy(&gdev, dev, sizeof(USB_DEVICE_INFO)); // --??-- Remove after test

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
  //DLOG("Set configuration to %d.", cfg->bConfigurationValue);
  usb_set_configuration(dev, cfg->bConfigurationValue);

  /* Now, negotiate with VS interface for streaming parameters */

  /* Manually set parameters */
  par.bmHint = 1; // Frame Interval Fixed
  par.bFormatIndex = 2;
  par.bFrameIndex = 2;
  //par.dwFrameInterval = 0x61A80; // 25FPS
  par.dwFrameInterval = 0xF4240; // 10FPS
  //par.wCompQuality = 5000; // 1 - 10000, with 1 the lowest
  //par.dwMaxPayloadTransferSize = PACKET_SIZE;
  
  para_block_dump (&par);
  
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


  /* Select Alternate Setting 1 for interface 1 (Std VS interface) */
  /* This is for an isochronous endpoint with MaxPacketSize 192 */
  DLOG("Select Alternate Setting 1 for VS interface");
  if (usb_set_interface(dev, 1, 1)) {
    DLOG("Cannot configure interface setting for Std VS interface");
    return FALSE;
  }

  delay(6000); /* -- EM -- Why is this here? I'm guessing because of
                * there is a delay that is required but this should be
                * done via checking for the control error whether the
                * last control transaction is complete, unless it is
                * asynchronous in which case the interrupt endpoint
                * should be checked
                */

  return TRUE;
}

static bool
uvc_probe (USB_DEVICE_INFO *dev, USB_CFG_DESC *cfg, USB_IF_DESC *ifd)
{
  /* For now, we only support device with multi video interface
   * collections. This is ugly. But let's make Logitech Webcam Pro
   * 9000 work first.
   */
  if(!(dev->devd.bDeviceClass == 0xEF) ||
     !(dev->devd.bDeviceSubClass == 0x02) ||
     !(dev->devd.bDeviceProtocol == 0x01)) {
    return FALSE;
  }

  if(((bool)dev->device_data)) {
    return TRUE;
  }

  usb_register_device(dev, &uvc_driver);
  dev->device_data = TRUE;

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

  status = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0), request, 0xA1,
                           VC_REQUEST_ERROR_CODE_CONTROL << 8, interface,
                           &code, 1, USB_DEFAULT_CONTROL_MSG_TIMEOUT);

  if (status) {DLOG("Getting error code failed!"); return 0xFF;}

  return code;
}

static int
video_probe_controls (USB_DEVICE_INFO * dev,
                      uint8_t request,
                      uint8_t interface,
                      UVC_VS_CTL_PAR_BLOCK * par)
{
  return usb_control_msg(dev,
                         request == SET_CUR ?
                         usb_sndctrlpipe(dev, 0) : usb_rcvctrlpipe(dev, 0),
                         request,
                         request == SET_CUR ? 0x21 : 0xA1,
                         VS_PROBE_CONTROL << 8, interface, par, 26,
                         USB_DEFAULT_CONTROL_MSG_TIMEOUT);

}

static int
video_commit_controls (USB_DEVICE_INFO * dev,
                       uint8_t request,
                       uint8_t interface,
                       UVC_VS_CTL_PAR_BLOCK * par)
{
   return usb_control_msg(dev,
                          request == SET_CUR ?
                          usb_sndctrlpipe(dev, 0) : usb_rcvctrlpipe(dev, 0),
                          request,
                          request == SET_CUR ? 0x21 : 0xA1,
                          VS_COMMIT_CONTROL << 8, interface, par, 26,
                          USB_DEFAULT_CONTROL_MSG_TIMEOUT);
}





bool
usb_uvc_driver_init (void)
{
  return usb_register_driver (&uvc_driver);
}

static void
para_block_dump (UVC_VS_CTL_PAR_BLOCK * par)
{
  DLOGV ("Parameter Block Dump: ");
  DLOGV ("  bmHint : 0x%x", par->bmHint);
  DLOGV ("  bFormatIndex : 0x%x", par->bFormatIndex);
  DLOGV ("  bFrameIndex : 0x%x", par->bFrameIndex);
  DLOGV ("  dwFrameInterval : 0x%x", par->dwFrameInterval);
  DLOGV ("  wKeyFrameRate : 0x%x", par->wKeyFrameRate);
  DLOGV ("  wPFrameRate : 0x%x", par->wPFrameRate);
  DLOGV ("  wCompQuality : 0x%x", par->wCompQuality);
  DLOGV ("  wCompWindowSize : 0x%x", par->wCompWindowSize);
  DLOGV ("  wDelay : 0x%x", par->wDelay);
  DLOGV ("  dwMaxVideoFrameSize : 0x%x", par->dwMaxVideoFrameSize);
  DLOGV ("  dwMaxPayloadTransferSize : 0x%x", par->dwMaxPayloadTransferSize);
  DLOGV ("  dwClockFrequency : 0x%x", par->dwClockFrequency);
  DLOGV ("  bmFramingInfo : 0x%x", par->bmFramingInfo);
  DLOGV ("  bPreferedVersion : 0x%x", par->bPreferedVersion);
  DLOGV ("  bMinVersion : 0x%x", par->bMinVersion);
  DLOGV ("  bMaxVersion : 0x%x", par->bMaxVersion);
}

static uint8_t conf[3000];

static int
uvc_device_cfg (
                USB_DEVICE_INFO *dev,
                USB_CFG_DESC *cfg,
                ISO_DATA_SRC * src)
{
  /* Dump all the descriptors retreived from UVC device */
  
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
  if(cfg->wTotalLength > sizeof(conf)) {
    DLOG("cfg->wTotalLength > sizeof(conf)");
    panic("cfg->wTotalLength > sizeof(conf)");
  }
  DLOGV("cfg->wTotalLength = %d", cfg->wTotalLength);
  usb_get_descriptor(dev, USB_TYPE_CFG_DESC, 0, 0, cfg->wTotalLength, (addr_t)conf);

  DLOGV("Primary descriptor");
  DLOGV("  bLength : 0x%x  bDescriptorType : 0x%x  wTotalLength : 0x%x",
        cfg->bLength, cfg->bDescriptorType, cfg->wTotalLength);
  DLOGV("  bNumInterfaces : 0x%x  bConfigurationValue : 0x%x",
        cfg->bNumInterfaces, cfg->bConfigurationValue);
  
  /* Parsing all UVC specific descriptors */
  /* Get the first IAD */
  DLOGV("Getting IAD ...");
  index += cfg->bLength;
  iad = (UVC_IA_DESC*)(&conf[index]);

  if ((iad->bDescriptorType != USB_TYPE_IA_DESC) ||
      (iad->bFunctionClass != CC_VIDEO) ||
      (iad->bFunctionSubClass != SC_VIDEO_INTERFACE_COLLECTION)) {
    DLOGV("IAD not found after configuration descriptor.");
    return -1;
  } else {
    DLOGV("Found Video Interface Collection");
    DLOGV("  bFirstInterface : 0x%x", iad->bFirstInterface);
    DLOGV("  bInterfaceCount : 0x%x", iad->bInterfaceCount);
  }

  /* Get First VC Interface Descriptor */
  DLOGV("Getting VC Interface Descriptor ...");
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
    DLOGV("Standard VC Interface Descriptor cannot be found");
    return -1;
  } else {
    DLOGV("Found Standard VC Interface");
    DLOGV("  bInterfaceNumber : 0x%x", vcifd->bInterfaceNumber);
    DLOGV("  bAlternateSetting : 0x%x", vcifd->bAlternateSetting);
    if (vcifd->bNumEndpoints) {
      DLOGV("  Status interrupt endpoint is present");
    }
    else {
      DLOGV("  Status interrupt endpoint is not present");
    }
  }

  /* Get Class-specific VC Interface header descriptor */
  DLOGV("Getting Class-specific VC Interface Descriptor ...");
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

  DLOGV("Found Class-specific VC Interface Header");
  DLOGV("  wTotalLength : 0x%x", csvcifd->wTotalLength);
  DLOGV("  bInCollection : 0x%x", csvcifd->bInCollection);

  /* Get Input and Output Terminals */
  DLOGV("Getting Input and Output Terminals ...");
  tmp_index = index;
  index += csvcifd->wTotalLength;
  DLOGV("csvcifd->wTotalLength = %d", csvcifd->wTotalLength);
  
  while (tmp_index < index) {
    desc_idx = (UVC_DESC_IDX *)(&conf[tmp_index]);
    if ((desc_idx->bDescriptorType == CS_INTERFACE) &&
        (desc_idx->bDescriptorSubType == VC_INPUT_TERMINAL)) {
      intd = (UVC_IN_TERM_DESC *)(&conf[tmp_index]);
      DLOGV("Found Input Terminal");
      DLOGV("  bTerminalID : 0x%x", intd->bTerminalID);
      DLOGV("  wTerminalType : 0x%x", intd->wTerminalType);
      DLOGV("  bAssocTerminal : 0x%x", intd->bAssocTerminal);
    }

    if ((desc_idx->bDescriptorType == CS_INTERFACE) &&
        (desc_idx->bDescriptorSubType == VC_OUTPUT_TERMINAL)) {
      outd = (UVC_OUT_TERM_DESC *)(&conf[tmp_index]);
      DLOGV("Found Output Terminal");
      DLOGV("  bTerminalID : 0x%x", outd->bTerminalID);
      DLOGV("  wTerminalType : 0x%x", outd->wTerminalType);
      DLOGV("  bAssocTerminal : 0x%x", outd->bAssocTerminal);
      DLOGV("  bSourceID : 0x%x", outd->bSourceID);
    }

    /* Multiple Processing or Extension Units may exist here */

    tmp_index += desc_idx->bLength;
  }

  /* We may have Standard and Class-specific Interrupt Endpoint
   * Descriptors after the Class-specific VC Interface Descriptor.
   * Just ignore them for now!
   */

  /* Get Standard VS Interface */
  DLOGV("Getting Standard VS Interface ...");
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
    DLOGV("Standard VS Interface Descriptor cannot be found");
    return -1;
  } else {
    DLOGV("Found Standard VS Interface");
    DLOGV("  bInterfaceNumber : 0x%x", vsifd->bInterfaceNumber);
    DLOGV("  bAlternateSetting : 0x%x", vsifd->bAlternateSetting);
    DLOGV("  bNumEndpoints : 0x%x", vsifd->bNumEndpoints);
  }

  /* Get Class-Specific VS Interface Input Header Descriptor */
  DLOGV("Getting Class-Specific VS Interface Input Header ...");
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

  DLOGV("Found Class-specific VS Interface Input Header");
  DLOGV("  bNumFormats : 0x%x", csvsifd->bNumFormats);
  DLOGV("  wTotalLength : 0x%x", csvsifd->wTotalLength);
  DLOGV("  bEndpointAddress : 0x%x", csvsifd->bEndpointAddress);
  DLOGV("  bTerminalLink : 0x%x", csvsifd->bTerminalLink);

  /* Try to get MJPEG format and frame info */
  DLOGV("Getting MJPEG Format and Frame Info ...");
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
    DLOGV("Motion-JPEG format is not supported");
    return -1;
  } else {
    DLOGV("Motion-JPEG format is supported");
    DLOGV("  bFormatIndex : 0x%x", jpegfd->bFormatIndex);
    DLOGV("  bNumFrameDescriptors : 0x%x", jpegfd->bNumFrameDescriptors);
    DLOGV("  bDefaultFrameIndex : 0x%x", jpegfd->bDefaultFrameIndex);
    DLOGV("  bCopyProtect : 0x%x", jpegfd->bCopyProtect);
  }

  tmp_index += jpegfd->bLength;

  /* Get all the frame descriptors pertaining to the format */
  while (tmp_index < index) {
    desc_idx = (UVC_DESC_IDX *)(&conf[tmp_index]);
    if ((desc_idx->bDescriptorType == CS_INTERFACE) &&
        (desc_idx->bDescriptorSubType == VS_FRAME_MJPEG)) {
      jpegrd = (UVC_MJPEG_FRAME_DESC *)(&conf[tmp_index]);

      DLOGV("Found Motion-JPEG Frame Descriptor");
      DLOGV("  bFrameIndex : 0x%x", jpegrd->bFrameIndex);
      DLOGV("  The Minimum Bit Rate : %d", jpegrd->dwMinBitRate);
      DLOGV("  The Maximum Bit Rate : %d", jpegrd->dwMaxBitRate);
      DLOGV("  Maximum Frame Buffer : %d bytes",
            jpegrd->dwMaxVideoFrameBufferSize);
      DLOGV("  The default interval : %d ns",
            jpegrd->dwDefaultFrameInterval * 100);
      DLOGV("  The supported resolution : %d x %d",
            jpegrd->wWidth, jpegrd->wHeight);
    }

    tmp_index += desc_idx->bLength;
  }

  /* We could have parsed the Still Image Frame descriptor
   * and Color Matching Descriptor here. But for simplicity,
   * just skip them.
   */

  /* Get all the alternate settings for the VS interface above */
  DLOGV("Getting possible alternate settings for VS interface ...");
  while (index < cfg->wTotalLength) {
    desc_idx = (UVC_DESC_IDX *)(&conf[index]);

    if (desc_idx->bDescriptorType == USB_TYPE_IA_DESC) {
      DLOGV("Reach the end of the current VS interface");
      break;
    }

    if (desc_idx->bDescriptorType == USB_TYPE_IF_DESC) {
      vsifd = (USB_IF_DESC *)(&conf[index]);

      DLOGV("Found Standard VS Interface");
      DLOGV("  bInterfaceNumber : 0x%x", vsifd->bInterfaceNumber);
      DLOGV("  bAlternateSetting : 0x%x", vsifd->bAlternateSetting);
      DLOGV("  bNumEndpoints : 0x%x", vsifd->bNumEndpoints);
    }

    if (desc_idx->bDescriptorType == USB_TYPE_EPT_DESC) {
      videoept = (USB_EPT_DESC *)(&conf[index]);

      DLOGV("Found Video Data Endpoint");
      DLOGV("  bEndpointAddress : 0x%x", videoept->bEndpointAddress);
      DLOGV("  bmAttributes : 0x%x", videoept->bmAttributes);
      if ((videoept->bmAttributes & 0xFC) == 1)
        DLOGV("  This is an isochronous end point"); 
      if ((videoept->bmAttributes & 0xFC) == 2)
        DLOGV("  This is a bulk end point"); 
      DLOGV("  The max packet size is : %d bytes",
            videoept->wMaxPacketSize);
      DLOGV("  bInterval : 0x%x", videoept->bInterval);
    }

    index += desc_idx->bLength;
  }

  DLOGV("UVC device configuration finished");

  return 0;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = usb_uvc_driver_init
};

DEF_MODULE (usb___uvc, "USB video driver", &mod_ops, {"usb"});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
