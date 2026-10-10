/****************************************************************************
 * arch/arm/src/ra8p1/ra_camera.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/config.h>

#include <sys/time.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/video/imgdata.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/ra8p1_camera.h"
#include "hardware/ra8p1_system.h"
#include "ra_camera.h"
#include "ra_clockconfig.h"
#include "ra_icu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_CAMERA_WIDTH_MAX          640
#define RA_CAMERA_HEIGHT_MAX         480
#define RA_CAMERA_BYTES_PER_PIXEL    2
#define RA_CAMERA_BUFFER_ALIGN       128
#define RA_CAMERA_IRQ_PRIORITY       12
#define RA_CAMERA_REGISTER_TIMEOUT   1000000
#define RA_CAMERA_START_DELAY_READS  10
#define RA_CAMERA_VIRTUAL_CHANNELS   16

#define RA_CAMERA_PHY_TIM1           74999u
#define RA_CAMERA_PHY_TIM2           ((37u << 16) | (62u << 8) | 9u)
#define RA_CAMERA_PHY_TIM3           ((24u << 8) | 6u)
#define RA_CAMERA_PHY_TIM4           ((7u << 24) | (20u << 16) | \
                                      (1u << 8) | 28u)
#define RA_CAMERA_PHY_TIM5           ((12u << 16) | (8u << 8) | 19u)
#define RA_CAMERA_PHY_TIM6           7u

#define RA_CAMERA_CSI_MCT0           (MIPI_CSI_MCT0_LANES(2) | \
                                      MIPI_CSI_MCT0_ERR_FRAME | \
                                      MIPI_CSI_MCT0_RESERVED_PACKET | \
                                      MIPI_CSI_MCT0_GENERIC_RULE)
#define RA_CAMERA_CSI_MCT2           (MIPI_CSI_MCT2_FRRCLK(10) | \
                                      MIPI_CSI_MCT2_FRRSKW(10))

#define RA_CAMERA_VIN_MC             (VIN_MC_COLOR_BYPASS | \
                                      VIN_MC_INTERLACE_BOTH | \
                                      VIN_MC_INPUT_YUV422_8 | \
                                      VIN_MC_YUV444_INTERPOLATE)
#define RA_CAMERA_VIN_DMR            (VIN_DMR_ALPHA_BIT | \
                                      VIN_DMR_BYTE_SWAP | \
                                      VIN_DMR_ALPHA8(0xaa))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_camera_s
{
  struct imgdata_s data;
  spinlock_t lock;
  imgdata_capture_t callback;
  FAR void *callback_arg;
  FAR uint8_t *buffer;
  uint32_t buffer_size;
  uint32_t buffer_generation;
  uintptr_t dma_base;
  size_t dma_size;
  uint16_t width;
  uint16_t height;
  bool dma_allocated;
  bool initialized;
  bool capturing;
  bool waiting_buffer;
  bool link_started;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra_camera_init(FAR struct imgdata_s *data);
static int ra_camera_uninit(FAR struct imgdata_s *data);
static int ra_camera_set_buf(FAR struct imgdata_s *data,
                             uint8_t nr_datafmts,
                             FAR imgdata_format_t *datafmts,
                             FAR uint8_t *addr, uint32_t size);
static int ra_camera_validate_frame_setting(FAR struct imgdata_s *data,
                             uint8_t nr_datafmts,
                             FAR imgdata_format_t *datafmts,
                             FAR imgdata_interval_t *interval);
static int ra_camera_start_capture(FAR struct imgdata_s *data,
                             uint8_t nr_datafmts,
                             FAR imgdata_format_t *datafmts,
                             FAR imgdata_interval_t *interval,
                             imgdata_capture_t callback,
                             FAR void *arg);
static int ra_camera_stop_capture(FAR struct imgdata_s *data);
static FAR void *ra_camera_alloc(FAR struct imgdata_s *data,
                                 uint32_t align_size, uint32_t size);
static void ra_camera_free(FAR struct imgdata_s *data, FAR void *addr);

static int ra_camera_vin_status_interrupt(int irq, FAR void *context,
                                          FAR void *arg);
static int ra_camera_vin_error_interrupt(int irq, FAR void *context,
                                         FAR void *arg);
static int ra_camera_csi_dl_interrupt(int irq, FAR void *context,
                                      FAR void *arg);
static int ra_camera_csi_vc_interrupt(int irq, FAR void *context,
                                      FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imgdata_ops_s g_camera_ops =
{
  .init = ra_camera_init,
  .uninit = ra_camera_uninit,
  .set_buf = ra_camera_set_buf,
  .validate_frame_setting = ra_camera_validate_frame_setting,
  .start_capture = ra_camera_start_capture,
  .stop_capture = ra_camera_stop_capture,
  .alloc = ra_camera_alloc,
  .free = ra_camera_free,
};

static struct ra_camera_s g_camera =
{
  .data =
    {
      .ops = &g_camera_ops,
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ra_camera_wait(uintptr_t address, uint32_t mask,
                          uint32_t expected)
{
  unsigned int timeout;

  for (timeout = 0; timeout < RA_CAMERA_REGISTER_TIMEOUT; timeout++)
    {
      if ((getreg32(address) & mask) == expected)
        {
          return OK;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

static void ra_camera_module_enable(bool enable)
{
  ra_module_control(RA8P1_MSTPCRC, MSTPCRC_MIPI_CSI, enable);
}

static int ra_camera_phy_enable(void)
{
  uint32_t reference;
  int ret;

  reference = BOARD_PCLKA_FREQUENCY / 1000000 - 1;
  putreg32(0, RA8P1_MIPI_PHY_DPHYMDC);
  putreg32(reference, RA8P1_MIPI_PHY_DPHYREFCR);
  putreg32(MIPI_PHY_PWR_ENABLE, RA8P1_MIPI_PHY_DPHYPWRCR);

  ret = ra_camera_wait(RA8P1_MIPI_PHY_DPHYSFR,
                       MIPI_PHY_PWR_STABLE, MIPI_PHY_PWR_STABLE);
  if (ret < 0)
    {
      putreg32(0, RA8P1_MIPI_PHY_DPHYPWRCR);
      return ret;
    }

  /* These D-PHY timings match the board SDK's two-lane OV5640 profile. */

  putreg32(RA_CAMERA_PHY_TIM1, RA8P1_MIPI_PHY_DPHYTIM1);
  putreg32(RA_CAMERA_PHY_TIM2, RA8P1_MIPI_PHY_DPHYTIM2);
  putreg32(RA_CAMERA_PHY_TIM3, RA8P1_MIPI_PHY_DPHYTIM3);
  putreg32(RA_CAMERA_PHY_TIM4, RA8P1_MIPI_PHY_DPHYTIM4);
  putreg32(RA_CAMERA_PHY_TIM5, RA8P1_MIPI_PHY_DPHYTIM5);
  putreg32(RA_CAMERA_PHY_TIM6, RA8P1_MIPI_PHY_DPHYTIM6);
  putreg32(MIPI_PHY_ENABLE, RA8P1_MIPI_PHY_DPHYOCR);
  return OK;
}

static void ra_camera_phy_disable(void)
{
  putreg32(0, RA8P1_MIPI_PHY_DPHYOCR);
  putreg32(MIPI_PHY_PLL_STOP, RA8P1_MIPI_PHY_DPHYPLOCR);
  putreg32(0, RA8P1_MIPI_PHY_DPHYPWRCR);
}

static void ra_camera_csi_clear(void)
{
  uint32_t status;
  unsigned int channel;

  status = getreg32(RA8P1_MIPI_CSI_RXST);
  putreg32(status & MIPI_CSI_RX_DETECTED,
           RA8P1_MIPI_CSI_RXSC);

  status = getreg32(RA8P1_MIPI_CSI_DLST0);
  putreg32(status & ~MIPI_CSI_DL_ULPS_MASK,
           RA8P1_MIPI_CSI_DLSC0);
  status = getreg32(RA8P1_MIPI_CSI_DLST1);
  putreg32(status & ~MIPI_CSI_DL_ULPS_MASK,
           RA8P1_MIPI_CSI_DLSC1);

  for (channel = 0; channel < RA_CAMERA_VIRTUAL_CHANNELS; channel++)
    {
      status = getreg32(RA8P1_MIPI_CSI_VCST(channel));
      putreg32(status, RA8P1_MIPI_CSI_VCSC(channel));
    }

  status = getreg32(RA8P1_MIPI_CSI_PMST);
  putreg32(status & 0xff, RA8P1_MIPI_CSI_PMSC);
  status = getreg32(RA8P1_MIPI_CSI_GSST);
  putreg32(status & (1u << 4), RA8P1_MIPI_CSI_GSSC);
}

static int ra_camera_csi_configure(void)
{
  unsigned int channel;
  int ret;

  putreg32(0, RA8P1_MIPI_CSI_MCT3);
  ret = ra_camera_wait(RA8P1_MIPI_CSI_RTST,
                       MIPI_CSI_VIDEO_RESET, 0);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(RA_CAMERA_CSI_MCT0, RA8P1_MIPI_CSI_MCT0);
  putreg32(RA_CAMERA_CSI_MCT2, RA8P1_MIPI_CSI_MCT2);
  putreg32(0, RA8P1_MIPI_CSI_EPCT);
  putreg32(0, RA8P1_MIPI_CSI_EMCT);
  putreg32(MIPI_CSI_DATA_YUV422_8, RA8P1_MIPI_CSI_DTEL);
  putreg32(0, RA8P1_MIPI_CSI_DTEH);
  putreg32(0, RA8P1_MIPI_CSI_GSCT);

  putreg32(0, RA8P1_MIPI_CSI_RXIE);
  putreg32(MIPI_CSI_DL_IRQ_MASK, RA8P1_MIPI_CSI_DLIE0);
  putreg32(MIPI_CSI_DL_IRQ_MASK, RA8P1_MIPI_CSI_DLIE1);
  for (channel = 0; channel < RA_CAMERA_VIRTUAL_CHANNELS; channel++)
    {
      putreg32(channel == 0 ? MIPI_CSI_VC_IRQ_MASK : 0,
               RA8P1_MIPI_CSI_VCIE(channel));
    }

  putreg32(0, RA8P1_MIPI_CSI_PMIE);
  putreg32(0, RA8P1_MIPI_CSI_GSIE);
  ra_camera_csi_clear();
  return OK;
}

static int ra_camera_csi_reset(FAR struct ra_camera_s *priv)
{
  int ret;

  putreg32(0, RA8P1_MIPI_CSI_MCT3);
  putreg32(MIPI_CSI_VIDEO_RESET, RA8P1_MIPI_CSI_RTCT);
  ret = ra_camera_wait(RA8P1_MIPI_CSI_RTST,
                       MIPI_CSI_VIDEO_RESET, 0);
  ra_camera_csi_clear();
  priv->link_started = false;
  return ret;
}

static void ra_camera_vin_configure(FAR struct ra_camera_s *priv)
{
  uintptr_t buffer = (uintptr_t)priv->buffer;

  putreg32(0, RA8P1_VIN_FC);
  putreg32(RA_CAMERA_VIN_MC & ~(VIN_MC_START | VIN_MC_ENABLE),
           RA8P1_VIN_MC);
  putreg32(VIN_CSI_IFMD_YUV422_8 | VIN_CSI_IFMD_ZERO_EXTEND,
           RA8P1_VIN_CSI_IFMD);
  putreg32(VIN_CSIFLD_ENABLE | VIN_CSIFLD_EVEN_BIT0,
           RA8P1_VIN_CSIFLD);
  putreg32(RA_CAMERA_VIN_DMR, RA8P1_VIN_DMR);
  putreg32(0, RA8P1_VIN_SLPRC);
  putreg32(priv->height - 1, RA8P1_VIN_ELPRC);
  putreg32(0, RA8P1_VIN_SPPRC);
  putreg32(priv->width - 1, RA8P1_VIN_EPPRC);
  putreg32(priv->width, RA8P1_VIN_IS);
  putreg32(buffer, RA8P1_VIN_MB1);
  putreg32(buffer, RA8P1_VIN_MB2);
  putreg32(buffer, RA8P1_VIN_MB3);
  putreg32(0, RA8P1_VIN_UVAOF);
  putreg32(VIN_UDS_BWIDTH_64, RA8P1_VIN_UDS_PASS_BWIDTH);
  putreg32(((uint32_t)priv->width << 16) | priv->height,
           RA8P1_VIN_UDS_CLIP_SIZE);
  putreg32(VIN_UDS_ADVANCED, RA8P1_VIN_UDS_CTRL);
  putreg32(0, RA8P1_VIN_SI);
  putreg32(VIN_INT_ENABLE_MASK, RA8P1_VIN_IE);
  modifyreg32(RA8P1_VIN_MC, 0, VIN_MC_COLOR_BYPASS);
}

static void ra_camera_vin_stop(void)
{
  modifyreg32(RA8P1_VIN_FC, VIN_FC_CONTINUOUS, 0);
  modifyreg32(RA8P1_VIN_MC, VIN_MC_ENABLE, 0);
  UP_DSB();
}

static int ra_camera_vin_start(FAR struct ra_camera_s *priv)
{
  unsigned int cycle;

  ra_camera_vin_configure(priv);
  modifyreg32(RA8P1_VIN_MC, 0, VIN_MC_START);
  for (cycle = 0; cycle < RA_CAMERA_START_DELAY_READS; cycle++)
    {
      (void)getreg32(RA8P1_VIN_MC);
    }

  putreg32(VIN_UDS_SCALE_ONE, RA8P1_VIN_UDS_SCALE);
  modifyreg32(RA8P1_VIN_MC, 0, VIN_MC_ENABLE);
  modifyreg32(RA8P1_VIN_FC, 0, VIN_FC_CONTINUOUS);

  if (!priv->link_started)
    {
      modifyreg32(RA8P1_MIPI_CSI_MCT3, 0, MIPI_CSI_RX_ENABLE);
      priv->link_started = true;
    }

  return OK;
}

static int ra_camera_irq_attach(FAR struct ra_camera_s *priv)
{
  int ret;

  ret = irq_attach(RA_IRQ_VIN_STATUS, ra_camera_vin_status_interrupt,
                   priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = irq_attach(RA_IRQ_VIN_ERROR, ra_camera_vin_error_interrupt,
                   priv);
  if (ret < 0)
    {
      irq_detach(RA_IRQ_VIN_STATUS);
      return ret;
    }

  ret = irq_attach(RA_IRQ_MIPICSI_DL, ra_camera_csi_dl_interrupt, priv);
  if (ret < 0)
    {
      irq_detach(RA_IRQ_VIN_ERROR);
      irq_detach(RA_IRQ_VIN_STATUS);
      return ret;
    }

  ret = irq_attach(RA_IRQ_MIPICSI_VC, ra_camera_csi_vc_interrupt, priv);
  if (ret < 0)
    {
      irq_detach(RA_IRQ_MIPICSI_DL);
      irq_detach(RA_IRQ_VIN_ERROR);
      irq_detach(RA_IRQ_VIN_STATUS);
      return ret;
    }

#ifdef CONFIG_ARCH_IRQPRIO
  up_prioritize_irq(RA_IRQ_VIN_STATUS, RA_CAMERA_IRQ_PRIORITY << 4);
  up_prioritize_irq(RA_IRQ_VIN_ERROR, RA_CAMERA_IRQ_PRIORITY << 4);
  up_prioritize_irq(RA_IRQ_MIPICSI_DL, RA_CAMERA_IRQ_PRIORITY << 4);
  up_prioritize_irq(RA_IRQ_MIPICSI_VC, RA_CAMERA_IRQ_PRIORITY << 4);
#endif

  ra_clear_ir(RA_IRQ_VIN_STATUS);
  ra_clear_ir(RA_IRQ_VIN_ERROR);
  ra_clear_ir(RA_IRQ_MIPICSI_DL);
  ra_clear_ir(RA_IRQ_MIPICSI_VC);
  up_enable_irq(RA_IRQ_VIN_STATUS);
  up_enable_irq(RA_IRQ_VIN_ERROR);
  up_enable_irq(RA_IRQ_MIPICSI_DL);
  up_enable_irq(RA_IRQ_MIPICSI_VC);
  return OK;
}

static void ra_camera_irq_detach(void)
{
  up_disable_irq(RA_IRQ_VIN_STATUS);
  up_disable_irq(RA_IRQ_VIN_ERROR);
  up_disable_irq(RA_IRQ_MIPICSI_DL);
  up_disable_irq(RA_IRQ_MIPICSI_VC);
  irq_detach(RA_IRQ_VIN_STATUS);
  irq_detach(RA_IRQ_VIN_ERROR);
  irq_detach(RA_IRQ_MIPICSI_DL);
  irq_detach(RA_IRQ_MIPICSI_VC);
}

static int ra_camera_hw_initialize(FAR struct ra_camera_s *priv)
{
  int ret;

  ra_camera_module_enable(true);
  ret = ra_camera_phy_enable();
  if (ret < 0)
    {
      ra_camera_module_enable(false);
      return ret;
    }

  ret = ra_camera_csi_configure();
  if (ret < 0)
    {
      ra_camera_phy_disable();
      ra_camera_module_enable(false);
      return ret;
    }

  putreg32(0, RA8P1_VIN_FC);
  putreg32(RA_CAMERA_VIN_MC, RA8P1_VIN_MC);
  putreg32(VIN_INT_FIFO_OVERFLOW | VIN_INT_AXI_ERROR |
           VIN_INT_FIELD2, RA8P1_VIN_INTS);

  ret = ra_camera_irq_attach(priv);
  if (ret < 0)
    {
      ra_camera_phy_disable();
      ra_camera_module_enable(false);
    }

  return ret;
}

static void ra_camera_hw_uninitialize(FAR struct ra_camera_s *priv)
{
  ra_camera_irq_detach();
  ra_camera_vin_stop();
  putreg32(0, RA8P1_VIN_IE);
  (void)ra_camera_csi_reset(priv);
  putreg32(0, RA8P1_MIPI_CSI_DLIE0);
  putreg32(0, RA8P1_MIPI_CSI_DLIE1);
  putreg32(0, RA8P1_MIPI_CSI_VCIE(0));
  ra_camera_phy_disable();
  ra_camera_module_enable(false);
}

static bool ra_camera_valid_format(uint8_t nr_datafmts,
                                   FAR imgdata_format_t *datafmts)
{
  FAR imgdata_format_t *fmt;

  if (nr_datafmts != 1 || datafmts == NULL)
    {
      return false;
    }

  fmt = &datafmts[IMGDATA_FMT_MAIN];
  if (fmt->pixelformat != IMGDATA_PIX_FMT_YUYV)
    {
      return false;
    }

  return (fmt->width == RA_CAMERA_WIDTH_MAX &&
          fmt->height == RA_CAMERA_HEIGHT_MAX) ||
         (fmt->width == RA_CAMERA_WIDTH_MAX / 2 &&
          fmt->height == RA_CAMERA_HEIGHT_MAX / 2);
}

static bool ra_camera_valid_interval(FAR imgdata_interval_t *interval)
{
  return interval != NULL && interval->numerator == 1 &&
         interval->denominator == 30;
}

static void ra_camera_complete(FAR struct ra_camera_s *priv,
                               bool error)
{
  imgdata_capture_t callback;
  FAR void *callback_arg;
  struct timeval timestamp;
  irqstate_t flags;
  uint32_t generation;
  uint32_t size;
  bool restart;

  ra_camera_vin_stop();
  if (error)
    {
      (void)ra_camera_csi_reset(priv);
    }

  flags = spin_lock_irqsave(&priv->lock);
  if (!priv->capturing || priv->callback == NULL)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return;
    }

  callback = priv->callback;
  callback_arg = priv->callback_arg;
  generation = priv->buffer_generation;
  size = priv->buffer_size;
  priv->waiting_buffer = true;
  spin_unlock_irqrestore(&priv->lock, flags);

  gettimeofday(&timestamp, NULL);
  callback(error ? 1 : 0, error ? 0 : size, &timestamp, callback_arg);

  flags = spin_lock_irqsave(&priv->lock);
  restart = priv->capturing && !priv->waiting_buffer &&
            priv->buffer_generation != generation;
  if (!restart && priv->capturing)
    {
      priv->capturing = false;
      priv->callback = NULL;
      priv->callback_arg = NULL;
      priv->waiting_buffer = false;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  if (restart && ra_camera_vin_start(priv) < 0)
    {
      flags = spin_lock_irqsave(&priv->lock);
      priv->capturing = false;
      priv->callback = NULL;
      priv->callback_arg = NULL;
      spin_unlock_irqrestore(&priv->lock, flags);
    }
}

static int ra_camera_init(FAR struct imgdata_s *data)
{
  FAR struct ra_camera_s *priv = (FAR struct ra_camera_s *)data;
  irqstate_t flags;
  int ret;

  if (priv->initialized)
    {
      return -EBUSY;
    }

  spin_lock_init(&priv->lock);
  flags = spin_lock_irqsave(&priv->lock);
  priv->callback = NULL;
  priv->callback_arg = NULL;
  priv->buffer = NULL;
  priv->buffer_size = 0;
  priv->buffer_generation = 0;
  priv->capturing = false;
  priv->waiting_buffer = false;
  priv->link_started = false;
  spin_unlock_irqrestore(&priv->lock, flags);

  ret = ra_camera_hw_initialize(priv);
  if (ret < 0)
    {
      return ret;
    }

  flags = spin_lock_irqsave(&priv->lock);
  priv->initialized = true;
  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

static int ra_camera_uninit(FAR struct imgdata_s *data)
{
  FAR struct ra_camera_s *priv = (FAR struct ra_camera_s *)data;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  if (!priv->initialized)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return -EPERM;
    }

  priv->capturing = false;
  priv->waiting_buffer = false;
  priv->callback = NULL;
  priv->callback_arg = NULL;
  priv->initialized = false;
  spin_unlock_irqrestore(&priv->lock, flags);

  ra_camera_hw_uninitialize(priv);
  return OK;
}

static int ra_camera_set_buf(FAR struct imgdata_s *data,
                             uint8_t nr_datafmts,
                             FAR imgdata_format_t *datafmts,
                             FAR uint8_t *addr, uint32_t size)
{
  FAR struct ra_camera_s *priv = (FAR struct ra_camera_s *)data;
  FAR imgdata_format_t *fmt;
  irqstate_t flags;
  uint32_t required;

  if (!ra_camera_valid_format(nr_datafmts, datafmts) || addr == NULL ||
      ((uintptr_t)addr & (RA_CAMERA_BUFFER_ALIGN - 1)) != 0 ||
      (uintptr_t)addr < priv->dma_base ||
      (uintptr_t)addr + size < (uintptr_t)addr ||
      (uintptr_t)addr + size > priv->dma_base + priv->dma_size)
    {
      return -EINVAL;
    }

  fmt = &datafmts[IMGDATA_FMT_MAIN];
  required = fmt->width * fmt->height * RA_CAMERA_BYTES_PER_PIXEL;
  if (size < required)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&priv->lock);
  if (priv->capturing && !priv->waiting_buffer)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return -EBUSY;
    }

  priv->buffer = addr;
  priv->buffer_size = required;
  priv->width = fmt->width;
  priv->height = fmt->height;
  priv->buffer_generation++;
  priv->waiting_buffer = false;
  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

static int ra_camera_validate_frame_setting(FAR struct imgdata_s *data,
                             uint8_t nr_datafmts,
                             FAR imgdata_format_t *datafmts,
                             FAR imgdata_interval_t *interval)
{
  (void)data;

  if (!ra_camera_valid_format(nr_datafmts, datafmts) ||
      !ra_camera_valid_interval(interval))
    {
      return -EINVAL;
    }

  return OK;
}

static int ra_camera_start_capture(FAR struct imgdata_s *data,
                             uint8_t nr_datafmts,
                             FAR imgdata_format_t *datafmts,
                             FAR imgdata_interval_t *interval,
                             imgdata_capture_t callback,
                             FAR void *arg)
{
  FAR struct ra_camera_s *priv = (FAR struct ra_camera_s *)data;
  irqstate_t flags;
  int ret;

  if (!ra_camera_valid_format(nr_datafmts, datafmts) ||
      !ra_camera_valid_interval(interval) || callback == NULL)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&priv->lock);
  if (!priv->initialized || priv->capturing || priv->buffer == NULL)
    {
      ret = !priv->initialized ? -EPERM :
            priv->capturing ? -EBUSY : -EINVAL;
      spin_unlock_irqrestore(&priv->lock, flags);
      return ret;
    }

  priv->callback = callback;
  priv->callback_arg = arg;
  priv->capturing = true;
  priv->waiting_buffer = false;
  spin_unlock_irqrestore(&priv->lock, flags);

  ret = ra_camera_vin_start(priv);
  if (ret < 0)
    {
      flags = spin_lock_irqsave(&priv->lock);
      priv->capturing = false;
      priv->callback = NULL;
      priv->callback_arg = NULL;
      spin_unlock_irqrestore(&priv->lock, flags);
    }

  return ret;
}

static int ra_camera_stop_capture(FAR struct imgdata_s *data)
{
  FAR struct ra_camera_s *priv = (FAR struct ra_camera_s *)data;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  priv->capturing = false;
  priv->waiting_buffer = false;
  priv->callback = NULL;
  priv->callback_arg = NULL;
  spin_unlock_irqrestore(&priv->lock, flags);

  ra_camera_vin_stop();
  return OK;
}

static FAR void *ra_camera_alloc(FAR struct imgdata_s *data,
                                 uint32_t align_size, uint32_t size)
{
  FAR struct ra_camera_s *priv = (FAR struct ra_camera_s *)data;
  FAR void *buffer = NULL;
  irqstate_t flags;

  if (align_size > RA_CAMERA_BUFFER_ALIGN)
    {
      return NULL;
    }

  flags = spin_lock_irqsave(&priv->lock);
  if (!priv->dma_allocated && size <= priv->dma_size)
    {
      priv->dma_allocated = true;
      buffer = (FAR void *)priv->dma_base;
    }

  spin_unlock_irqrestore(&priv->lock, flags);
  return buffer;
}

static void ra_camera_free(FAR struct imgdata_s *data, FAR void *addr)
{
  FAR struct ra_camera_s *priv = (FAR struct ra_camera_s *)data;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  if (!priv->capturing && (uintptr_t)addr == priv->dma_base)
    {
      priv->dma_allocated = false;
      priv->buffer = NULL;
      priv->buffer_size = 0;
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

static int ra_camera_vin_interrupt(FAR struct ra_camera_s *priv,
                                   int irq, bool forced_error)
{
  uint32_t status;
  bool error;

  status = getreg32(RA8P1_VIN_INTS);
  putreg32(status | VIN_INT_FIFO_OVERFLOW | VIN_INT_AXI_ERROR |
           VIN_INT_FIELD2, RA8P1_VIN_INTS);
  ra_clear_ir(irq);

  error = forced_error;
  if (error || (status & VIN_INT_FRAME_COMPLETE) != 0)
    {
      ra_camera_complete(priv, error);
    }

  return OK;
}

static int ra_camera_vin_status_interrupt(int irq, FAR void *context,
                                          FAR void *arg)
{
  return ra_camera_vin_interrupt(arg, irq, false);
}

static int ra_camera_vin_error_interrupt(int irq, FAR void *context,
                                         FAR void *arg)
{
  return ra_camera_vin_interrupt(arg, irq, true);
}

static int ra_camera_csi_dl_interrupt(int irq, FAR void *context,
                                      FAR void *arg)
{
  FAR struct ra_camera_s *priv = arg;
  uint32_t lane0;
  uint32_t lane1;

  lane0 = getreg32(RA8P1_MIPI_CSI_DLST0);
  lane1 = getreg32(RA8P1_MIPI_CSI_DLST1);
  putreg32(lane0 & ~MIPI_CSI_DL_ULPS_MASK,
           RA8P1_MIPI_CSI_DLSC0);
  putreg32(lane1 & ~MIPI_CSI_DL_ULPS_MASK,
           RA8P1_MIPI_CSI_DLSC1);
  ra_clear_ir(irq);

  if (((lane0 | lane1) & MIPI_CSI_DL_ERROR_MASK) != 0)
    {
      ra_camera_complete(priv, true);
    }

  return OK;
}

static int ra_camera_csi_vc_interrupt(int irq, FAR void *context,
                                      FAR void *arg)
{
  FAR struct ra_camera_s *priv = arg;
  uint32_t status;

  status = getreg32(RA8P1_MIPI_CSI_VCST(0));
  putreg32(status, RA8P1_MIPI_CSI_VCSC(0));
  ra_clear_ir(irq);

  if ((status & MIPI_CSI_VC_ERROR_MASK) != 0)
    {
      ra_camera_complete(priv, true);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_camera_initialize
 *
 * Description:
 *   Return the singleton image-data lower-half backed by a board-provided
 *   DMA region.
 *
 ****************************************************************************/

FAR struct imgdata_s *ra_camera_initialize(uintptr_t dma_base,
                                           size_t dma_size)
{
  if (dma_base == 0 || dma_size == 0 ||
      dma_base + dma_size < dma_base)
    {
      return NULL;
    }

  if (g_camera.dma_size != 0)
    {
      return g_camera.dma_base == dma_base &&
             g_camera.dma_size == dma_size ? &g_camera.data : NULL;
    }

  g_camera.dma_base = dma_base;
  g_camera.dma_size = dma_size;
  g_camera.dma_allocated = false;
  return &g_camera.data;
}
