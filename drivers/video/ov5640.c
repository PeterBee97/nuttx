/****************************************************************************
 * drivers/video/ov5640.c
 *
 * Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its
 * affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/ov5640.h>
#include <nuttx/video/video.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OV5640_I2C_ADDRESS       0x3c
#define OV5640_I2C_FREQUENCY     I2C_SPEED_STANDARD

#define OV5640_REG_CHIP_ID_HIGH  0x300a
#define OV5640_REG_CHIP_ID_LOW   0x300b
#define OV5640_CHIP_ID_HIGH      0x56
#define OV5640_CHIP_ID_LOW       0x40
#define OV5640_CHIP_ID_LOW_MASK  0xf0

#define OV5640_REG_SYSTEM_CTRL0   0x3008
#define OV5640_REG_STREAM_CTRL    0x4202
#define OV5640_STREAM_ON          0x00
#define OV5640_STREAM_OFF         0x0f

#define OV5640_REG_DELAY          0xfffe
#define OV5640_REG_END            0xffff

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ov5640_reg_s
{
  uint16_t address;
  uint8_t value;
};

struct ov5640_dev_s
{
  struct imgsensor_s sensor;
  FAR struct i2c_master_s *i2c;
  bool streaming;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool ov5640_is_available(FAR struct imgsensor_s *sensor);
static int ov5640_init(FAR struct imgsensor_s *sensor);
static int ov5640_uninit(FAR struct imgsensor_s *sensor);
static FAR const char *
ov5640_get_driver_name(FAR struct imgsensor_s *sensor);
static int ov5640_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval);
static int ov5640_start_capture(FAR struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval);
static int ov5640_stop_capture(FAR struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* VGA YUV422 MIPI sensor configuration derived from the Renesas reference
 * project.  This table contains OV5640 register data only; controller and
 * board operations are implemented through NuttX interfaces.
 */

static const struct ov5640_reg_s g_ov5640_init_regs[] =
{
  {0x3103, 0x11},
  {OV5640_REG_SYSTEM_CTRL0, 0x82},
  {OV5640_REG_DELAY, 255},
  {OV5640_REG_SYSTEM_CTRL0, 0x42},
  {0x3103, 0x03},
  {OV5640_REG_DELAY, 5},
  {0x3017, 0x00},
  {0x3018, 0x00},
  {0x3034, 0x18},
  {0x3037, 0x13},
  {0x302c, 0xc2},
  {0x3108, 0x01},
  {0x3630, 0x2e},
  {0x3632, 0xe2},
  {0x3633, 0x23},
  {0x3621, 0xe0},
  {0x3704, 0xa0},
  {0x3703, 0x5a},
  {0x3715, 0x78},
  {0x3717, 0x01},
  {0x370b, 0x60},
  {0x3705, 0x1a},
  {0x3905, 0x02},
  {0x3906, 0x10},
  {0x3901, 0x0a},
  {0x3731, 0x12},
  {0x3600, 0x08},
  {0x3601, 0x33},
  {0x302d, 0x60},
  {0x3620, 0x52},
  {0x371b, 0x20},
  {0x471c, 0x50},
  {0x3a18, 0x00},
  {0x3a19, 0x80},
  {0x3635, 0x1c},
  {0x3634, 0x40},
  {0x3622, 0x01},
  {0x3c04, 0x28},
  {0x3c05, 0x98},
  {0x3c06, 0x00},
  {0x3c07, 0x08},
  {0x3c08, 0x00},
  {0x3c09, 0x1c},
  {0x3c0a, 0x9c},
  {0x3c0b, 0x40},
  {0x3820, 0x41},
  {0x3821, 0x01},
  {0x3800, 0x00},
  {0x3801, 0x20},
  {0x3802, 0x00},
  {0x3803, 0x16},
  {0x3804, 0x0a},
  {0x3805, 0x20},
  {0x3806, 0x07},
  {0x3807, 0x96},
  {0x3810, 0x00},
  {0x3811, 0x10},
  {0x3812, 0x00},
  {0x3813, 0x10},
  {0x3808, 0x02},
  {0x3809, 0x80},
  {0x380a, 0x01},
  {0x380b, 0xe0},
  {0x3814, 0x31},
  {0x3815, 0x31},
  {0x3708, 0x64},
  {0x4001, 0x02},
  {0x4005, 0x1a},
  {0x3000, 0x00},
  {0x3002, 0x1c},
  {0x3004, 0xff},
  {0x3006, 0xc3},
  {0x300e, 0x44},
  {0x302e, 0x08},
  {0x4300, 0x32},
  {0x501f, 0x00},
  {0x4407, 0x04},
  {0x5000, 0xa7},
  {0x3406, 0x01},
  {0x3400, 0x06},
  {0x3401, 0x80},
  {0x3402, 0x04},
  {0x3403, 0x00},
  {0x3404, 0x06},
  {0x3405, 0x00},
  {0x5180, 0xff},
  {0x5181, 0xf2},
  {0x5182, 0x00},
  {0x5183, 0x14},
  {0x5184, 0x25},
  {0x5185, 0x24},
  {0x5186, 0x16},
  {0x5187, 0x16},
  {0x5188, 0x16},
  {0x5189, 0x62},
  {0x518a, 0x62},
  {0x518b, 0xf0},
  {0x518c, 0xb2},
  {0x518d, 0x50},
  {0x518e, 0x30},
  {0x518f, 0x30},
  {0x5190, 0x50},
  {0x5191, 0xf8},
  {0x5192, 0x04},
  {0x5193, 0x70},
  {0x5194, 0xf0},
  {0x5195, 0xf0},
  {0x5196, 0x03},
  {0x5197, 0x01},
  {0x5198, 0x04},
  {0x5199, 0x12},
  {0x519a, 0x04},
  {0x519b, 0x00},
  {0x519c, 0x06},
  {0x519d, 0x82},
  {0x519e, 0x38},
  {0x5381, 0x1e},
  {0x5382, 0x5b},
  {0x5383, 0x14},
  {0x5384, 0x06},
  {0x5385, 0x82},
  {0x5386, 0x88},
  {0x5387, 0x7c},
  {0x5388, 0x60},
  {0x5389, 0x1c},
  {0x538a, 0x01},
  {0x538b, 0x98},
  {0x5300, 0x08},
  {0x5301, 0x30},
  {0x5302, 0x5f},
  {0x5303, 0x10},
  {0x5304, 0x08},
  {0x5305, 0x30},
  {0x5306, 0x28},
  {0x5307, 0x38},
  {0x5309, 0x08},
  {0x530a, 0x30},
  {0x530b, 0x04},
  {0x530c, 0x06},
  {0x5480, 0x01},
  {0x5481, 0x06},
  {0x5482, 0x12},
  {0x5483, 0x24},
  {0x5484, 0x4a},
  {0x5485, 0x58},
  {0x5486, 0x65},
  {0x5487, 0x72},
  {0x5488, 0x7d},
  {0x5489, 0x88},
  {0x548a, 0x92},
  {0x548b, 0xa3},
  {0x548c, 0xb2},
  {0x548d, 0xc8},
  {0x548e, 0xdd},
  {0x548f, 0xf0},
  {0x5490, 0x15},
  {0x5580, 0x06},
  {0x5583, 0x40},
  {0x5584, 0x20},
  {0x5589, 0x10},
  {0x558a, 0x00},
  {0x558b, 0xf8},
  {0x5585, 0x00},
  {0x5586, 0x20},
  {0x5587, 0x00},
  {0x5588, 0x01},
  {0x501d, 0x40},
  {0x5000, 0xa7},
  {0x5800, 0x20},
  {0x5801, 0x19},
  {0x5802, 0x17},
  {0x5803, 0x16},
  {0x5804, 0x18},
  {0x5805, 0x21},
  {0x5806, 0x0f},
  {0x5807, 0x0a},
  {0x5808, 0x07},
  {0x5809, 0x07},
  {0x580a, 0x0a},
  {0x580b, 0x0c},
  {0x580c, 0x0a},
  {0x580d, 0x03},
  {0x580e, 0x01},
  {0x580f, 0x01},
  {0x5810, 0x03},
  {0x5811, 0x09},
  {0x5812, 0x0a},
  {0x5813, 0x03},
  {0x5814, 0x01},
  {0x5815, 0x01},
  {0x5816, 0x03},
  {0x5817, 0x08},
  {0x5818, 0x10},
  {0x5819, 0x0a},
  {0x581a, 0x06},
  {0x581b, 0x06},
  {0x581c, 0x08},
  {0x581d, 0x0e},
  {0x581e, 0x22},
  {0x581f, 0x18},
  {0x5820, 0x13},
  {0x5821, 0x12},
  {0x5822, 0x16},
  {0x5823, 0x1e},
  {0x5824, 0x64},
  {0x5825, 0x2a},
  {0x5826, 0x2c},
  {0x5827, 0x2a},
  {0x5828, 0x46},
  {0x5829, 0x2a},
  {0x582a, 0x26},
  {0x582b, 0x24},
  {0x582c, 0x26},
  {0x582d, 0x2a},
  {0x582e, 0x28},
  {0x582f, 0x42},
  {0x5830, 0x40},
  {0x5831, 0x42},
  {0x5832, 0x08},
  {0x5833, 0x28},
  {0x5834, 0x26},
  {0x5835, 0x24},
  {0x5836, 0x26},
  {0x5837, 0x2a},
  {0x5838, 0x44},
  {0x5839, 0x4a},
  {0x583a, 0x2c},
  {0x583b, 0x2a},
  {0x583c, 0x46},
  {0x583d, 0xce},
  {0x5688, 0x22},
  {0x5689, 0x22},
  {0x568a, 0x42},
  {0x568b, 0x24},
  {0x568c, 0x42},
  {0x568d, 0x24},
  {0x568e, 0x22},
  {0x568f, 0x22},
  {0x5025, 0x00},
  {0x3a0f, 0x40},
  {0x3a10, 0x30},
  {0x3a1b, 0x40},
  {0x3a1e, 0x30},
  {0x3a11, 0x71},
  {0x3a1f, 0x20},
  {0x4800, 0x24},
  {0x3007, 0xfb},
  {0x380c, 0x08},
  {0x380d, 0x50},
  {0x380e, 0x04},
  {0x380f, 0xb0},
  {0x3c01, 0xb4},
  {0x3c00, 0x04},
  {0x3a08, 0x00},
  {0x3a09, 0x93},
  {0x3a0e, 0x06},
  {0x3a0a, 0x00},
  {0x3a0b, 0x7b},
  {0x3a0d, 0x08},
  {0x3a00, 0x3c},
  {0x3a02, 0x05},
  {0x3a03, 0x44},
  {0x3a14, 0x05},
  {0x3a15, 0x44},
  {0x3618, 0x00},
  {0x3612, 0x29},
  {0x3708, 0x64},
  {0x3709, 0x52},
  {0x370c, 0x03},
  {0x4001, 0x02},
  {0x4004, 0x02},
  {0x4005, 0x1a},
  {0x4713, 0x03},
  {0x460b, 0x35},
  {0x460c, 0x22},
  {0x4837, 0x0a},
  {0x3824, 0x01},
  {0x5001, 0xa3},
  {OV5640_REG_SYSTEM_CTRL0, 0x02},
  {OV5640_REG_DELAY, 5},
  {0x503d, 0x00},
  {0x4741, 0x00},
  {OV5640_REG_STREAM_CTRL, OV5640_STREAM_OFF},
  {OV5640_REG_END, 0x00},
};

static const struct imgsensor_ops_s g_ov5640_ops =
{
  .is_available = ov5640_is_available,
  .init = ov5640_init,
  .uninit = ov5640_uninit,
  .get_driver_name = ov5640_get_driver_name,
  .validate_frame_setting = ov5640_validate_frame_setting,
  .start_capture = ov5640_start_capture,
  .stop_capture = ov5640_stop_capture,
};

static const struct v4l2_fmtdesc g_ov5640_fmtdescs[] =
{
  {
    .pixelformat = V4L2_PIX_FMT_YUYV,
    .description = "YUV 4:2:2 (YUYV)",
  },
};

static const struct v4l2_frmsizeenum g_ov5640_frmsizes[] =
{
  {
    .type = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete =
      {
        .width = VIDEO_HSIZE_QVGA,
        .height = VIDEO_VSIZE_QVGA,
      },
  },
  {
    .type = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete =
      {
        .width = VIDEO_HSIZE_VGA,
        .height = VIDEO_VSIZE_VGA,
      },
  },
};

static const struct v4l2_frmivalenum g_ov5640_frmintervals[] =
{
  {
    .type = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete =
      {
        .numerator = 1,
        .denominator = 30,
      },
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ov5640_putreg(FAR struct ov5640_dev_s *priv,
                         uint16_t address, uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t buffer[3];

  buffer[0] = address >> 8;
  buffer[1] = address;
  buffer[2] = value;

  msg.frequency = OV5640_I2C_FREQUENCY;
  msg.addr = OV5640_I2C_ADDRESS;
  msg.flags = 0;
  msg.buffer = buffer;
  msg.length = sizeof(buffer);
  return I2C_TRANSFER(priv->i2c, &msg, 1);
}

static int ov5640_getreg(FAR struct ov5640_dev_s *priv,
                         uint16_t address, FAR uint8_t *value)
{
  struct i2c_config_s config =
  {
    .frequency = OV5640_I2C_FREQUENCY,
    .address   = OV5640_I2C_ADDRESS,
    .addrlen   = 7,
  };

  uint8_t buffer[2];

  buffer[0] = address >> 8;
  buffer[1] = address;
  return i2c_writeread(priv->i2c, &config, buffer, sizeof(buffer),
                       value, 1);
}

static int ov5640_putreglist(FAR struct ov5640_dev_s *priv,
                             FAR const struct ov5640_reg_s *regs)
{
  int ret;

  while (regs->address != OV5640_REG_END)
    {
      if (regs->address == OV5640_REG_DELAY)
        {
          up_mdelay(regs->value);
        }
      else
        {
          ret = ov5640_putreg(priv, regs->address, regs->value);
          if (ret < 0)
            {
              return ret;
            }
        }

      regs++;
    }

  return OK;
}

static int ov5640_set_size(FAR struct ov5640_dev_s *priv,
                           uint16_t width, uint16_t height)
{
  int ret;

  ret = ov5640_putreg(priv, 0x3808, width >> 8);
  if (ret >= 0)
    {
      ret = ov5640_putreg(priv, 0x3809, width);
    }

  if (ret >= 0)
    {
      ret = ov5640_putreg(priv, 0x380a, height >> 8);
    }

  if (ret >= 0)
    {
      ret = ov5640_putreg(priv, 0x380b, height);
    }

  return ret;
}

static bool ov5640_valid_format(uint8_t nr_datafmts,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval)
{
  FAR imgsensor_format_t *fmt;

  if (nr_datafmts != 1 || datafmts == NULL || interval == NULL)
    {
      return false;
    }

  fmt = &datafmts[IMGSENSOR_FMT_MAIN];
  if (fmt->pixelformat != IMGSENSOR_PIX_FMT_YUYV ||
      interval->numerator != 1 || interval->denominator != 30)
    {
      return false;
    }

  return (fmt->width == VIDEO_HSIZE_QVGA &&
          fmt->height == VIDEO_VSIZE_QVGA) ||
         (fmt->width == VIDEO_HSIZE_VGA &&
          fmt->height == VIDEO_VSIZE_VGA);
}

static bool ov5640_is_available(FAR struct imgsensor_s *sensor)
{
  FAR struct ov5640_dev_s *priv = (FAR struct ov5640_dev_s *)sensor;
  uint8_t id_high;
  uint8_t id_low;

  if (ov5640_getreg(priv, OV5640_REG_CHIP_ID_HIGH, &id_high) < 0 ||
      ov5640_getreg(priv, OV5640_REG_CHIP_ID_LOW, &id_low) < 0)
    {
      return false;
    }

  return id_high == OV5640_CHIP_ID_HIGH &&
         (id_low & OV5640_CHIP_ID_LOW_MASK) == OV5640_CHIP_ID_LOW;
}

static int ov5640_init(FAR struct imgsensor_s *sensor)
{
  FAR struct ov5640_dev_s *priv = (FAR struct ov5640_dev_s *)sensor;
  int ret;

  ret = ov5640_putreglist(priv, g_ov5640_init_regs);
  if (ret >= 0)
    {
      priv->streaming = false;
    }

  return ret;
}

static int ov5640_uninit(FAR struct imgsensor_s *sensor)
{
  FAR struct ov5640_dev_s *priv = (FAR struct ov5640_dev_s *)sensor;
  int ret;

  ret = ov5640_putreg(priv, OV5640_REG_STREAM_CTRL,
                      OV5640_STREAM_OFF);
  up_mdelay(40);
  if (ret >= 0)
    {
      priv->streaming = false;
    }

  return ret;
}

static FAR const char *
ov5640_get_driver_name(FAR struct imgsensor_s *sensor)
{
  (void)sensor;
  return "OV5640";
}

static int ov5640_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval)
{
  if (type != IMGSENSOR_STREAM_TYPE_VIDEO)
    {
      return -EINVAL;
    }

  return ov5640_valid_format(nr_datafmts, datafmts, interval) ?
         OK : -EINVAL;
}

static int ov5640_start_capture(FAR struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval)
{
  FAR struct ov5640_dev_s *priv = (FAR struct ov5640_dev_s *)sensor;
  FAR imgsensor_format_t *fmt;
  uint8_t value;
  int ret;

  if (type != IMGSENSOR_STREAM_TYPE_VIDEO ||
      !ov5640_valid_format(nr_datafmts, datafmts, interval))
    {
      return -EINVAL;
    }

  if (priv->streaming)
    {
      return -EBUSY;
    }

  fmt = &datafmts[IMGSENSOR_FMT_MAIN];
  ret = ov5640_set_size(priv, fmt->width, fmt->height);
  if (ret >= 0)
    {
      ret = ov5640_putreg(priv, 0x3035, 0x12);
    }

  if (ret >= 0)
    {
      ret = ov5640_putreg(priv, 0x3036, 0x8c);
    }

  if (ret >= 0)
    {
      ret = ov5640_putreg(priv, 0x3037, 0x13);
    }

  if (ret >= 0)
    {
      ret = ov5640_putreg(priv, 0x3108, 0x01);
    }

  if (ret >= 0)
    {
      ret = ov5640_putreg(priv, 0x3406, 0x00);
    }

  if (ret >= 0)
    {
      ret = ov5640_getreg(priv, 0x4814, &value);
    }

  if (ret >= 0)
    {
      ret = ov5640_putreg(priv, 0x4814, value & 0x3f);
    }

  if (ret >= 0)
    {
      ret = ov5640_putreg(priv, OV5640_REG_STREAM_CTRL,
                          OV5640_STREAM_ON);
    }

  if (ret >= 0)
    {
      priv->streaming = true;
    }

  return ret;
}

static int ov5640_stop_capture(FAR struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type)
{
  FAR struct ov5640_dev_s *priv = (FAR struct ov5640_dev_s *)sensor;
  int ret;

  if (type != IMGSENSOR_STREAM_TYPE_VIDEO)
    {
      return -EINVAL;
    }

  ret = ov5640_putreg(priv, OV5640_REG_STREAM_CTRL, OV5640_STREAM_OFF);
  if (ret >= 0)
    {
      priv->streaming = false;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov5640_initialize
 ****************************************************************************/

FAR struct imgsensor_s *
ov5640_initialize(FAR struct i2c_master_s *i2c)
{
  FAR struct ov5640_dev_s *priv;

  if (i2c == NULL)
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->i2c = i2c;
  priv->sensor.ops = &g_ov5640_ops;
  priv->sensor.fmtdescs_num = nitems(g_ov5640_fmtdescs);
  priv->sensor.fmtdescs = g_ov5640_fmtdescs;
  priv->sensor.frmsizes_num = nitems(g_ov5640_frmsizes);
  priv->sensor.frmsizes = g_ov5640_frmsizes;
  priv->sensor.frmintervals_num = nitems(g_ov5640_frmintervals);
  priv->sensor.frmintervals = g_ov5640_frmintervals;
  return &priv->sensor;
}
