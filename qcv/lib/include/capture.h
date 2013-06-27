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

#ifndef _QCV_CAPTURE_H_
#define _QCV_CAPTURE_H_


#include <libavutil/opt.h>
#include <libavcodec/avcodec.h>
#include <libavutil/channel_layout.h>
#include <libavutil/common.h>
#include <libavutil/imgutils.h>
#include <libavutil/mathematics.h>
#include <libavutil/samplefmt.h>
#include <libswscale/swscale.h>

#include "frame.h"

typedef enum {
  CAPTURE_SOURCE_CAMERA,
  CAPTURE_SOURCE_FILE,
} capture_source_t;

typedef struct {
  int source_fd;
  capture_source_t source;
  union {
    struct {
      unsigned char* uncompressed_frame;
      size_t uncompressed_frame_buf_len;
      size_t uncompressed_frame_len;
    } source_camera;
    struct {
      AVCodec *av_codec;
      AVCodecContext *av_codec_context;
      AVPacket *av_pkt;
      AVFrame *av_frame;
      unsigned char* buf;
      size_t buf_size;
      BOOL done;
    } source_file;
  };
} qcv_capture_t;

#define CAMERA_UNCOMPRSSED_FRAME_BUF_LEN ((size_t)40960)

int qcv_capture_from_camera(qcv_capture_t* capture, int index);
int qcv_capture_from_file(qcv_capture_t* capture, char * const filename);
int qcv_grab_frame(qcv_capture_t* capture);
int qcv_retrieve_frame(qcv_capture_t* capture, qcv_frame_t* frame);
int qcv_query_frame(qcv_capture_t* capture, qcv_frame_t* frame);

#endif // _QCV_CAPTURE_H_


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
