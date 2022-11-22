/****************************************************************************
 * arch/sim/src/sim/sim_video.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arch/board/board.h>
#include <nuttx/config.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imgdata.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include "sim_video_host.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HOST_VIDEO_DEV_PATH  CONFIG_HOST_VIDEO_DEV_PATH
#define ENQUEUE(q, addr, size) (q).addr[q.num] = addr; \
                               (q).size[q.num++] = size;

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  imgdata_capture_t capture_cb;
  uint32_t buf_size;
  uintptr_t next_buf;
  bool is_streaming;
} video_priv_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* video image sensor operations */

static bool sim_video_is_available(void);
static int sim_video_init(void);
static int sim_video_uninit(void);
static FAR const char *sim_video_get_driver_name(void);
static int sim_video_validate_frame_setting(imgsensor_stream_type_t type,
                                         uint8_t nr_datafmt,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval);
static int sim_video_start_capture(imgsensor_stream_type_t type,
                                uint8_t nr_datafmt,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval);
static int sim_video_stop_capture(imgsensor_stream_type_t type);
static int sim_video_get_supported_value(uint32_t id,
                                     FAR imgsensor_supported_value_t *value);
static int sim_video_get_value(uint32_t id, uint32_t size,
                            FAR imgsensor_value_t *value);
static int sim_video_set_value(uint32_t id, uint32_t size,
                            imgsensor_value_t value);

/* video image data operations */

static int sim_video_data_init(void);
static int sim_video_data_uninit(void);
static int sim_video_data_validate_frame_setting
             (uint8_t nr_datafmt,
              imgdata_format_t *datafmt,
              imgdata_interval_t *interval);
static int sim_video_data_start_capture
             (uint8_t nr_datafmt,
              imgdata_format_t *datafmt,
              imgdata_interval_t *interval,
              imgdata_capture_t callback);
static int sim_video_data_stop_capture(void);
static int sim_video_data_validate_buf(uint8_t *addr, uint32_t size);
static int sim_video_data_set_buf(uint8_t *addr, uint32_t size);

static uint32_t imgsensor_fmt_to_v4l2(uint32_t pixelformat);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imgsensor_ops_s g_sim_video_ops =
{
  .is_available             = sim_video_is_available,
  .init                     = sim_video_init,
  .uninit                   = sim_video_uninit,
  .get_driver_name          = sim_video_get_driver_name,
  .validate_frame_setting   = sim_video_validate_frame_setting,
  .start_capture            = sim_video_start_capture,
  .stop_capture             = sim_video_stop_capture,
  .get_supported_value      = sim_video_get_supported_value,
  .get_value                = sim_video_get_value,
  .set_value                = sim_video_set_value,
};

static struct imgdata_ops_s g_sim_video_data_ops =
  {
    .init                   = sim_video_data_init,
    .uninit                 = sim_video_data_uninit,
    .validate_buf           = sim_video_data_validate_buf,
    .set_buf                = sim_video_data_set_buf,
    .validate_frame_setting = sim_video_data_validate_frame_setting,
    .start_capture          = sim_video_data_start_capture,
    .stop_capture           = sim_video_data_stop_capture,
  };

static video_priv_t priv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool sim_video_is_available(void)
{
  return video_host_is_available(HOST_VIDEO_DEV_PATH);
}

static int sim_video_init(void)
{
  return video_host_init(HOST_VIDEO_DEV_PATH);
}

static int sim_video_uninit(void)
{
  return video_host_uninit();
}

static FAR const char *sim_video_get_driver_name(void)
{
  return "V4L2 NuttX Sim Driver";
}

static int sim_video_get_supported_value(uint32_t id,
                                     FAR imgsensor_supported_value_t *value)
{
  return 0;
}

static int sim_video_get_value(uint32_t id, uint32_t size,
                            FAR imgsensor_value_t *value)
{
  return 0;
}

static int sim_video_set_value(uint32_t id, uint32_t size,
                            imgsensor_value_t value)
{
  return 0;
}

static int validate_format(imgsensor_stream_type_t type, int nr_fmt,
    FAR imgsensor_format_t *fmt, FAR imgsensor_interval_t *interval)
{
  if (nr_fmt > 1)
    {
      return -ENOTSUP;
    }

  uint32_t v4l2_fmt = imgsensor_fmt_to_v4l2(fmt->pixelformat);
  if (type == IMGSENSOR_STREAM_TYPE_VIDEO)
    {
      return video_host_try_fmt(fmt->width, fmt->height,
        v4l2_fmt, interval->denominator, interval->numerator);
    }
  else
    {
      return video_host_try_fmt(fmt->width, fmt->height, v4l2_fmt, 0, 0);
    }
}

static int sim_video_validate_frame_setting(imgsensor_stream_type_t type,
                                         uint8_t nr_fmt,
                                         FAR imgsensor_format_t *fmt,
                                         FAR imgsensor_interval_t *interval)
{
  return validate_format(type, nr_fmt, fmt, interval);
}

static int sim_video_start_capture(imgsensor_stream_type_t type,
                                uint8_t nr_fmt,
                                FAR imgsensor_format_t *fmt,
                                FAR imgsensor_interval_t *interval)
{
  return video_host_set_fmt(fmt[IMGDATA_FMT_MAIN].width,
    fmt[IMGDATA_FMT_MAIN].height,
    imgsensor_fmt_to_v4l2(fmt[IMGDATA_FMT_MAIN].pixelformat),
    interval->denominator, interval->numerator);
}

static int sim_video_stop_capture(imgsensor_stream_type_t type)
{
  return 0;
}

static int sim_video_data_init()
{
  memset(&priv, 0, sizeof(priv));
  return video_host_data_init();
}

static int sim_video_data_uninit()
{
  return 0;
}

static int sim_video_data_validate_frame_setting
             (uint8_t nr_datafmt,
              imgdata_format_t *datafmt,
              imgdata_interval_t *interval)
{
  return 0;
}

static int sim_video_data_start_capture
             (uint8_t nr_datafmt,
              imgdata_format_t *datafmt,
              imgdata_interval_t *interval,
              imgdata_capture_t callback)
{
  int ret;

  priv.capture_cb = callback;
  ret = video_host_start_capture();
  if (ret < 0)
    {
      return ret;
    }

  priv.is_streaming = true;
  return 0;
}

static int sim_video_data_stop_capture()
{
  priv.is_streaming = false;
  return video_host_stop_capture();
}

static int sim_video_data_validate_buf(uint8_t *addr, uint32_t size)
{
  if (!addr || (uintptr_t)(addr) & 0x1f)
    {
      return -EINVAL;
    }

  return 0;
}

static int sim_video_data_set_buf(uint8_t *addr, uint32_t size)
{
  priv.next_buf = (uintptr_t)addr;
  priv.buf_size = size;
  return 0;
}

static uint32_t imgsensor_fmt_to_v4l2(uint32_t pixelformat)
{
  uint32_t fourcc;
  switch (pixelformat)
    {
      case IMGSENSOR_PIX_FMT_YUV420P:
        fourcc = V4L2_PIX_FMT_YUV420;
        break;

      case IMGSENSOR_PIX_FMT_YUYV:
        fourcc = V4L2_PIX_FMT_YUYV;
        break;

      case IMGSENSOR_PIX_FMT_JPEG_WITH_SUBIMG:
        fourcc = V4L2_PIX_FMT_JPEG;
        break;

      case IMGSENSOR_PIX_FMT_JPEG:
        fourcc = V4L2_PIX_FMT_JPEG;
        break;

      case IMGSENSOR_PIX_FMT_RGB565:
        fourcc = V4L2_PIX_FMT_RGB565;
        break;

      case IMGSENSOR_PIX_FMT_UYVY:
        fourcc = V4L2_PIX_FMT_UYVY;
        break;

      default:

      /* Unsupported format */

        fourcc = 0;
    }

  return fourcc;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sim_video_initialize(void)
{
  imgsensor_register(&g_sim_video_ops);
  imgdata_register(&g_sim_video_data_ops);
  return 0;
}

int sim_video_uninitialize(void)
{
  return 0;
}

void sim_video_loop(void)
{
  int ret;

  if (priv.is_streaming && priv.next_buf)
    {
      ret = video_host_dq_buf((uint8_t *)priv.next_buf, priv.buf_size);
      if (ret > 0)
        {
          priv.capture_cb(0, ret);
        }
    }
}
