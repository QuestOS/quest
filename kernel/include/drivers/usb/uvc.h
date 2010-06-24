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

#ifndef _UVC_H_
#define _UVC_H_

#include <types.h>

#define USB_TYPE_IA_DESC    0x0B

/* Video Class-Specific Request Codes */
#define RC_UNDEFINED    0x00
#define SET_CUR         0x01
#define GET_CUR         0x81
#define GET_MIN         0x82
#define GET_MAX         0x83
#define GET_RES         0x84
#define GET_LEN         0x85
#define GET_INFO        0x86
#define GET_DEF         0x87

/* VideoControl Interface Control Selectors */
#define VC_CONTROL_UNDEFINED             0x00
#define VC_VIDEO_POWER_MODE_CONTROL      0x01
#define VC_REQUEST_ERROR_CODE_CONTROL    0x02

/* Terminal Control Selectors */
#define TE_CONTROL_UNDEFINED    0x00

/* Selector Unit Control Selector */
#define SU_CONTROL_UNDEFINED       0x00
#define SU_INPUT_SELECT_CONTROL    0x01

/* Camera Terminal Control Selectors */
#define CT_CONTROL_UNDEFINED                 0x00
#define CT_SCANNING_MODE_CONTROL             0x01
#define CT_AE_MODE_CONTROL                   0x02
#define CT_AE_PRIORITY_CONTROL               0x03
#define CT_EXPOSURE_TIME_ABSOLUTE_CONTROL    0x04
#define CT_EXPOSURE_TIME_RELATIVE_CONTROL    0x05
#define CT_FOCUS_ABSOLUTE_CONTROL            0x06
#define CT_FOCUS_RELATIVE_CONTROL            0x07
#define CT_FOCUS_AUTO_CONTROL                0x08
#define CT_IRIS_ABSOLUTE_CONTROL             0x09
#define CT_IRIS_RELATIVE_CONTROL             0x0A
#define CT_ZOOM_ABSOLUTE_CONTROL             0x0B
#define CT_ZOOM_RELATIVE_CONTROL             0x0C
#define CT_PANTILT_ABSOLUTE_CONTROL          0x0D
#define CT_PANTILT_RELATIVE_CONTROL          0x0E
#define CT_ROLL_ABSOLUTE_CONTROL             0x0F
#define CT_ROLL_RELATIVE_CONTROL             0x10
#define CT_PRIVACY_CONTROL                   0x11

/* Processing Unit Control Selectors */
#define PU_CONTROL_UNDEFINED                         0x00
#define PU_BACKLIGHT_COMPENSATION_CONTROL            0x01
#define PU_BRIGHTNESS_CONTROL                        0x02
#define PU_CONTRAST_CONTROL                          0x03
#define PU_GAIN_CONTROL                              0x04
#define PU_POWER_LINE_FREQUENCY_CONTROL              0x05
#define PU_HUE_CONTROL                               0x06
#define PU_SATURATION_CONTROL                        0x07
#define PU_SHARPNESS_CONTROL                         0x08
#define PU_GAMMA_CONTROL                             0x09
#define PU_WHITE_BALANCE_TEMPERATURE_CONTROL         0x0A
#define PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL    0x0B
#define PU_WHITE_BALANCE_COMPONENT_CONTROL           0x0C
#define PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL      0x0D
#define PU_DIGITAL_MULTIPLIER_CONTROL                0x0E
#define PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL          0x0F
#define PU_HUE_AUTO_CONTROL                          0x10
#define PU_ANALOG_VIDEO_STANDARD_CONTROL             0x11
#define PU_ANALOG_LOCK_STATUS_CONTROL                0x12

/* Extension Unit Control Selectors */
#define XU_CONTROL_UNDEFINED    0x00

/* VideoStreaming Interface Control Selectors */
#define VS_CONTROL_UNDEFINED               0x00
#define VS_PROBE_CONTROL                   0x01
#define VS_COMMIT_CONTROL                  0x02
#define VS_STILL_PROBE_CONTROL             0x03
#define VS_STILL_COMMIT_CONTROL            0x04
#define VS_STILL_IMAGE_TRIGGER_CONTROL     0x05
#define VS_STREAM_ERROR_CODE_CONTROL       0x06
#define VS_GENERATE_KEY_FRAME_CONTROL      0x07
#define VS_UPDATE_FRAME_SEGMENT_CONTROL    0x08
#define VS_SYNCH_DELAY_CONTROL             0x09

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

/*
 * UVC_VS_CTL_PAR_BLOCK : Parameter block of VS interface control request
 * 
 * Reference :
 *     USB Device Class Definition for Video Devices
 *     Revision 1.1, Page 103
 */
struct uvc_vs_ctl_par_block
{
  uint16_t bmHint;
  uint8_t bFormatIndex;
  uint8_t bFrameIndex;
  uint32_t dwFrameInterval;
  uint16_t wKeyFrameRate;
  uint16_t wPFrameRate;
  uint16_t wCompQuality;
  uint16_t wCompWindowSize;
  uint16_t wDelay;
  uint32_t dwMaxVideoFrameSize;
  uint32_t dwMaxPayloadTransferSize;
  uint32_t dwClockFrequency;
  uint8_t bmFramingInfo;
  uint8_t bPreferedVersion;
  uint8_t bMinVersion;
  uint8_t bMaxVersion;
} __attribute__ ((packed));

typedef struct uvc_vs_ctl_par_block UVC_VS_CTL_PAR_BLOCK;

extern bool usb_uvc_driver_init (void);

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
