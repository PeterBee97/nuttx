/****************************************************************************
 * arch/sim/src/sim/posix/sim_video_host.c
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

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_REQBUFS 3
#define WARN(fmt, ...) \
        syslog(LOG_WARNING, "sim_video_host: " fmt "\n", ##__VA_ARGS__)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  int fd;
  struct v4l2_requestbuffers reqbuf;
  void *addrs[MAX_REQBUFS];
  size_t buflen[MAX_REQBUFS];
} video_host_dev_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static video_host_dev_t priv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int xioctl(int fd, int request, void *arg)
{
  int r;
  do
    {
      r = ioctl(fd, request, arg);
    }
  while (-1 == r && EINTR == errno);

  return r;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool video_host_is_available(const char *video_host_dev_path)
{
  if (access(video_host_dev_path, F_OK) == 0)
    {
      return true;
    }

  return false;
}

int video_host_init(const char *video_host_dev_path)
{
  int fd = open(video_host_dev_path, O_RDWR | O_NONBLOCK);
  if (fd < 0)
    {
      perror(video_host_dev_path);
      return -errno;
    }

  memset(&priv, 0, sizeof(priv));
  priv.fd = fd;
  return 0;
}

int video_host_dq_buf(uint8_t *addr, uint32_t size)
{
  struct v4l2_buffer buf;

  memset(&buf, 0, sizeof(buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  /* Dequeue a buffer */

  if (-1 == xioctl(priv.fd, VIDIOC_DQBUF, &buf))
    {
      switch (errno)
        {
          case EAGAIN:

            /* No buffer in the outgoing queue */

            return 0;
          case EIO:

            /* fall through */

          default:
            perror("VIDIOC_DQBUF");
            return -errno;
        }
    }

  if (size > buf.bytesused)
    {
      size = buf.bytesused;
    }

  memcpy(addr, priv.addrs[buf.index], size);
  if (-1 == ioctl(priv.fd, VIDIOC_QBUF, &buf))
    {
      perror("VIDIOC_QBUF");
      return -errno;
    }

  return size;
}

int video_host_uninit(void)
{
  close(priv.fd);
  return 0;
}

int video_host_data_init(void)
{
  return 0;
}

int video_host_start_capture(void)
{
  struct v4l2_buffer buf;
  int i;

  /* VIDIOC_REQBUFS initiate user pointer I/O */

  priv.reqbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  priv.reqbuf.memory = V4L2_MEMORY_MMAP;
  priv.reqbuf.count  = MAX_REQBUFS;

  if (-1 == xioctl(priv.fd, VIDIOC_REQBUFS, &priv.reqbuf))
    {
      perror("VIDIOC_REQBUFS");
      return -errno;
    }

  if (priv.reqbuf.count < 2)
    {
      errno = ENOMEM;
      perror("Not enough buffers");
      return -ENOMEM;
    }

  for (i = 0; i < priv.reqbuf.count; i++)
    {
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (-1 == xioctl(priv.fd, VIDIOC_QUERYBUF, &buf))
        {
          perror("VIDIOC_QUERYBUF");
          return -errno;
        }

      priv.addrs[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                           MAP_SHARED, priv.fd, buf.m.offset);
      priv.buflen[i] = buf.length;
      if (priv.addrs[i] == MAP_FAILED)
        {
          perror("Mmap failed");
          return -errno;
        }

      if (-1 == xioctl(priv.fd, VIDIOC_QBUF, &buf))
        {
          perror("VIDIOC_QBUF");
          return -errno;
        }
    }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(priv.fd, VIDIOC_STREAMON, &type))
    {
      perror("VIDIOC_STREAMON");
      return -errno;
    }

  return 0;
}

int video_host_stop_capture(void)
{
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  int i;

  if (-1 == xioctl(priv.fd, VIDIOC_STREAMOFF, &type))
    {
      perror("VIDIOC_STREAMOFF");
      return -errno;
    }

  for (i = 0; i < priv.reqbuf.count; i++)
    {
      munmap(priv.addrs[i], priv.buflen[i]);
    }

  return 0;
}

int video_host_set_fmt(uint16_t width, uint16_t height, uint32_t fmt,
    uint32_t denom, uint32_t numer)
{
  struct v4l2_format v4l2_fmt =
    {
      0
    };

  v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  v4l2_fmt.fmt.pix.width = width;
  v4l2_fmt.fmt.pix.height = height;
  v4l2_fmt.fmt.pix.pixelformat = fmt;
  v4l2_fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (-1 == xioctl(priv.fd, VIDIOC_S_FMT, &v4l2_fmt))
    {
      perror("VIDIOC_S_FMT");
      return -errno;
    }

  struct v4l2_streamparm streamparm =
    {
      0
    };

  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(priv.fd, VIDIOC_G_PARM, &streamparm))
    {
      perror("VIDIOC_G_PARM");
      return -errno;
    }

  streamparm.parm.capture.capturemode |= V4L2_CAP_TIMEPERFRAME;
  streamparm.parm.capture.timeperframe.numerator = numer;
  streamparm.parm.capture.timeperframe.denominator = denom;
  if (-1 == xioctl(priv.fd, VIDIOC_S_PARM, &streamparm))
    {
      perror("VIDIOC_S_PARM");
      return -errno;
    }

  return 0;
}

int video_host_try_fmt(uint16_t width, uint16_t height, uint32_t fmt,
    uint32_t denom, uint32_t numer)
{
  struct v4l2_format v4l2_fmt =
    {
      0
    };

  v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  v4l2_fmt.fmt.pix.width = width;
  v4l2_fmt.fmt.pix.height = height;
  v4l2_fmt.fmt.pix.pixelformat = fmt;
  v4l2_fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (-1 == xioctl(priv.fd, VIDIOC_TRY_FMT, &v4l2_fmt))
    {
      perror("VIDIOC_TRY_FMT");
      return -errno;
    }

  if (v4l2_fmt.fmt.pix.pixelformat != fmt)
    {
      WARN("Pixel format not supported");
      return -EINVAL;
    }

  struct v4l2_frmivalenum v4l2_frmival =
    {
      0
    };

  v4l2_frmival.width = width;
  v4l2_frmival.height = height;
  v4l2_frmival.pixel_format = fmt;

  /* Need not check frame interval for STILL type */

  if (!denom)
    {
      while (xioctl(priv.fd, VIDIOC_ENUM_FRAMEINTERVALS,
          &v4l2_frmival) == 0)
        {
          if (v4l2_frmival.type == V4L2_FRMSIZE_TYPE_DISCRETE &&
              v4l2_frmival.discrete.denominator == denom &&
              v4l2_frmival.discrete.numerator == numer)
            {
              return 0;
            }

          v4l2_frmival.index++;
        }

        WARN("Invalid frame interval, fallback to default");
    }

  return 0;
}
