/****************************************************************************
 * arch/arm/src/ra8p1/ra_sdhi.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>

#include <debug.h>

#include <arch/board/board.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "hardware/ra8p1_sdhi.h"
#include "hardware/ra8p1_system.h"
#include "ra_clockconfig.h"
#include "ra_icu.h"
#include "ra_sdhi.h"

#ifdef CONFIG_RA8P1_SDHI0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_SDHI_COMMAND_TIMEOUT_MS  100
#define RA_SDHI_BUSY_TIMEOUT_MS     5000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_sdhi_dev_s
{
  struct sdio_dev_s dev;
  uintptr_t base;
  uint32_t response[4];
  FAR uint8_t *buffer;
  size_t remaining;
  unsigned int blocklen;
  unsigned int nblocks;
  uint32_t timeout;
  int cmdresult;
  sdio_eventset_t waitevents;
  sdio_eventset_t wkupevent;
  sdio_eventset_t cbevents;
  bool receive;
  bool appcmd;
  bool initialized;

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
  worker_t callback;
  FAR void *cbarg;
  struct work_s cbwork;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ra_sdhi_reset(FAR struct sdio_dev_s *dev);
static sdio_capset_t ra_sdhi_capabilities(FAR struct sdio_dev_s *dev);
static sdio_statset_t ra_sdhi_status(FAR struct sdio_dev_s *dev);
static void ra_sdhi_widebus(FAR struct sdio_dev_s *dev, bool enable);
static void ra_sdhi_clock(FAR struct sdio_dev_s *dev,
                          enum sdio_clock_e rate);
static int ra_sdhi_attach(FAR struct sdio_dev_s *dev);
static int ra_sdhi_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t arg);
#ifdef CONFIG_SDIO_BLOCKSETUP
static void ra_sdhi_blocksetup(FAR struct sdio_dev_s *dev,
                               unsigned int blocklen,
                               unsigned int nblocks);
#endif
static int ra_sdhi_recvsetup(FAR struct sdio_dev_s *dev,
                             FAR uint8_t *buffer, size_t nbytes);
static int ra_sdhi_sendsetup(FAR struct sdio_dev_s *dev,
                             FAR const uint8_t *buffer, size_t nbytes);
static int ra_sdhi_cancel(FAR struct sdio_dev_s *dev);
static int ra_sdhi_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd);
static int ra_sdhi_recv_r1(FAR struct sdio_dev_s *dev, uint32_t cmd,
                           FAR uint32_t *r1);
static int ra_sdhi_recv_r2(FAR struct sdio_dev_s *dev, uint32_t cmd,
                           FAR uint32_t r2[4]);
static int ra_sdhi_recv_r3(FAR struct sdio_dev_s *dev, uint32_t cmd,
                           FAR uint32_t *r3);
static void ra_sdhi_waitenable(FAR struct sdio_dev_s *dev,
                               sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t ra_sdhi_eventwait(FAR struct sdio_dev_s *dev);
static void ra_sdhi_callbackenable(FAR struct sdio_dev_s *dev,
                                   sdio_eventset_t eventset);
#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
static int ra_sdhi_registercallback(FAR struct sdio_dev_s *dev,
                                    worker_t callback, FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ra_sdhi_dev_s g_sdhi0 =
{
  .dev =
  {
    .mutex            = NXMUTEX_INITIALIZER,
    .reset            = ra_sdhi_reset,
    .capabilities     = ra_sdhi_capabilities,
    .status           = ra_sdhi_status,
    .widebus          = ra_sdhi_widebus,
    .clock            = ra_sdhi_clock,
    .attach           = ra_sdhi_attach,
    .sendcmd          = ra_sdhi_sendcmd,
#ifdef CONFIG_SDIO_BLOCKSETUP
    .blocksetup       = ra_sdhi_blocksetup,
#endif
    .recvsetup        = ra_sdhi_recvsetup,
    .sendsetup        = ra_sdhi_sendsetup,
    .cancel           = ra_sdhi_cancel,
    .waitresponse     = ra_sdhi_waitresponse,
    .recv_r1          = ra_sdhi_recv_r1,
    .recv_r2          = ra_sdhi_recv_r2,
    .recv_r3          = ra_sdhi_recv_r3,
    .recv_r4          = ra_sdhi_recv_r3,
    .recv_r5          = ra_sdhi_recv_r1,
    .recv_r6          = ra_sdhi_recv_r1,
    .recv_r7          = ra_sdhi_recv_r3,
    .waitenable       = ra_sdhi_waitenable,
    .eventwait        = ra_sdhi_eventwait,
    .callbackenable   = ra_sdhi_callbackenable,
#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
    .registercallback = ra_sdhi_registercallback,
#endif
    .gotextcsd        = NULL,
  },
  .base               = RA8P1_SDHI0_BASE,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t ra_sdhi_getreg(FAR struct ra_sdhi_dev_s *priv,
                                      unsigned int offset)
{
  return getreg32(priv->base + offset);
}

static inline void ra_sdhi_putreg(FAR struct ra_sdhi_dev_s *priv,
                                  unsigned int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static bool ra_sdhi_timedout(clock_t start, uint32_t timeout)
{
  clock_t ticks = MSEC2TICK(timeout);

  if (timeout == 0)
    {
      return true;
    }

  if (ticks < 1)
    {
      ticks = 1;
    }

  return clock_systime_ticks() - start >= ticks;
}

static int ra_sdhi_waitidle(FAR struct ra_sdhi_dev_s *priv)
{
  clock_t start = clock_systime_ticks();

  while ((ra_sdhi_getreg(priv, RA8P1_SDHI_SD_INFO2_OFFSET) &
          SDHI_INFO2_IDLE_MASK) != SDHI_INFO2_IDLE)
    {
      if (ra_sdhi_timedout(start, RA_SDHI_BUSY_TIMEOUT_MS))
        {
          return -ETIMEDOUT;
        }

      up_udelay(1);
    }

  return OK;
}

static int ra_sdhi_setclock(FAR struct ra_sdhi_dev_s *priv,
                            uint32_t frequency)
{
  uint32_t setting = SDHI_CLOCK_DIV_BYPASS;
  unsigned int shift;
  clock_t start;

  for (shift = 0; shift <= 9; shift++)
    {
      if ((BOARD_PCLKB_FREQUENCY >> shift) <= frequency)
        {
          setting = shift == 0 ? SDHI_CLOCK_DIV_BYPASS :
                    (1u << shift) >> 2;
          break;
        }
    }

  if (shift > 9)
    {
      return -EINVAL;
    }

  start = clock_systime_ticks();
  while ((ra_sdhi_getreg(priv, RA8P1_SDHI_SD_INFO2_OFFSET) &
          SDHI_INFO2_CLOCK_READY) == 0)
    {
      if (ra_sdhi_timedout(start, RA_SDHI_COMMAND_TIMEOUT_MS))
        {
          return -ETIMEDOUT;
        }
    }

  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_CLK_CTRL_OFFSET,
                 setting | SDHI_CLOCK_CONTROL_MASK);
  return OK;
}

static uint32_t ra_sdhi_command(uint32_t cmd, bool appcmd)
{
  uint32_t command;

  command = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  if (appcmd)
    {
      command |= SDHI_CMD_ACMD;
    }

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
      case MMCSD_NO_RESPONSE:
        command |= SDHI_CMD_RESPONSE_NONE;
        break;

      case MMCSD_R1B_RESPONSE:
        command |= SDHI_CMD_RESPONSE_R1B;
        break;

      case MMCSD_R2_RESPONSE:
        command |= SDHI_CMD_RESPONSE_R2;
        break;

      case MMCSD_R3_RESPONSE:
      case MMCSD_R4_RESPONSE:
        command |= SDHI_CMD_RESPONSE_R3;
        break;

      default:
        command |= SDHI_CMD_RESPONSE_R1;
        break;
    }

  if ((cmd & MMCSD_DATAXFR_MASK) != MMCSD_NODATAXFR)
    {
      command |= SDHI_CMD_DATA;
      if ((cmd & MMCSD_WRXFR) == 0)
        {
          command |= SDHI_CMD_DATA_READ;
        }

      if ((cmd & MMCSD_MULTIBLOCK) != 0)
        {
          command |= SDHI_CMD_MULTI | SDHI_CMD_MULTI_NO_AUTOSTOP;
        }
    }

  return command;
}

static int ra_sdhi_commandwait(FAR struct ra_sdhi_dev_s *priv,
                               uint32_t cmd)
{
  uint32_t info1;
  uint32_t info2;
  uint32_t errors;
  clock_t start = clock_systime_ticks();

  if ((cmd & MMCSD_RESPONSE_MASK) == MMCSD_NO_RESPONSE)
    {
      while ((ra_sdhi_getreg(priv, RA8P1_SDHI_SD_INFO2_OFFSET) &
              SDHI_INFO2_CMD_BUSY) != 0)
        {
          if (ra_sdhi_timedout(start, RA_SDHI_COMMAND_TIMEOUT_MS))
            {
              return -ETIMEDOUT;
            }
        }

      return OK;
    }

  for (; ; )
    {
      info1 = ra_sdhi_getreg(priv, RA8P1_SDHI_SD_INFO1_OFFSET);
      info2 = ra_sdhi_getreg(priv, RA8P1_SDHI_SD_INFO2_OFFSET);
      errors = info2 & SDHI_INFO2_ERROR_MASK;

      if ((cmd & MMCSD_RESPONSE_MASK) == MMCSD_R3_RESPONSE ||
          (cmd & MMCSD_RESPONSE_MASK) == MMCSD_R4_RESPONSE)
        {
          errors &= ~SDHI_INFO2_CRC_ERROR;
        }

      if (errors != 0)
        {
          return (errors & (SDHI_INFO2_RESPONSE_TIMEOUT |
                            SDHI_INFO2_DATA_TIMEOUT)) != 0 ?
                 -ETIMEDOUT : -EIO;
        }

      if ((info1 & SDHI_INFO1_RESPONSE_END) != 0)
        {
          ra_sdhi_putreg(priv, RA8P1_SDHI_SD_INFO1_OFFSET,
                         ~SDHI_INFO1_RESPONSE_END);
          ra_sdhi_putreg(priv, RA8P1_SDHI_SD_INFO2_OFFSET,
                         ~SDHI_INFO2_CRC_ERROR);
          return OK;
        }

      if (ra_sdhi_timedout(start, RA_SDHI_COMMAND_TIMEOUT_MS))
        {
          return -ETIMEDOUT;
        }
    }
}

static void ra_sdhi_saveresponse(FAR struct ra_sdhi_dev_s *priv,
                                  uint32_t cmd)
{
  uint32_t rsp10;
  uint32_t rsp32;
  uint32_t rsp54;
  uint32_t rsp76;

  memset(priv->response, 0, sizeof(priv->response));
  if ((cmd & MMCSD_RESPONSE_MASK) == MMCSD_R2_RESPONSE)
    {
      rsp10 = ra_sdhi_getreg(priv, RA8P1_SDHI_SD_RSP10_OFFSET);
      rsp32 = ra_sdhi_getreg(priv, RA8P1_SDHI_SD_RSP32_OFFSET);
      rsp54 = ra_sdhi_getreg(priv, RA8P1_SDHI_SD_RSP54_OFFSET);
      rsp76 = ra_sdhi_getreg(priv, RA8P1_SDHI_SD_RSP76_OFFSET);

      priv->response[0] = (rsp76 << 8) | (rsp54 >> 24);
      priv->response[1] = (rsp54 << 8) | (rsp32 >> 24);
      priv->response[2] = (rsp32 << 8) | (rsp10 >> 24);
      priv->response[3] = rsp10 << 8;
    }
  else if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_NO_RESPONSE)
    {
      priv->response[0] =
        ra_sdhi_getreg(priv, RA8P1_SDHI_SD_RSP10_OFFSET);
    }
}

static int ra_sdhi_transfer(FAR struct ra_sdhi_dev_s *priv)
{
  uint32_t ready;
  uint32_t info1;
  uint32_t info2;
  uint32_t errors;
  uint32_t word;
  unsigned int block;
  unsigned int offset;
  size_t copylen;
  clock_t start = clock_systime_ticks();

  ready = priv->receive ? SDHI_INFO2_BUFFER_READ_ENABLE :
                          SDHI_INFO2_BUFFER_WRITE_ENABLE;

  for (block = 0; block < priv->nblocks; block++)
    {
      for (; ; )
        {
          info2 = ra_sdhi_getreg(priv, RA8P1_SDHI_SD_INFO2_OFFSET);
          errors = info2 & SDHI_INFO2_ERROR_MASK;
          if (errors != 0)
            {
              return (errors & SDHI_INFO2_DATA_TIMEOUT) != 0 ?
                     -ETIMEDOUT : -EIO;
            }

          if ((info2 & ready) != 0)
            {
              break;
            }

          if (ra_sdhi_timedout(start, priv->timeout))
            {
              return -ETIMEDOUT;
            }
        }

      for (offset = 0; offset < priv->blocklen; offset += sizeof(word))
        {
          if (priv->receive)
            {
              word = ra_sdhi_getreg(priv, RA8P1_SDHI_SD_BUF0_OFFSET);
              copylen = priv->blocklen - offset;
              if (copylen > sizeof(word))
                {
                  copylen = sizeof(word);
                }

              memcpy(priv->buffer + offset, &word, copylen);
            }
          else
            {
              word = 0;
              copylen = priv->blocklen - offset;
              if (copylen > sizeof(word))
                {
                  copylen = sizeof(word);
                }

              memcpy(&word, priv->buffer + offset, copylen);
              ra_sdhi_putreg(priv, RA8P1_SDHI_SD_BUF0_OFFSET, word);
            }
        }

      priv->buffer += priv->blocklen;
      priv->remaining -= priv->blocklen;
      ra_sdhi_putreg(priv, RA8P1_SDHI_SD_INFO2_OFFSET, ~ready);
    }

  for (; ; )
    {
      info1 = ra_sdhi_getreg(priv, RA8P1_SDHI_SD_INFO1_OFFSET);
      info2 = ra_sdhi_getreg(priv, RA8P1_SDHI_SD_INFO2_OFFSET);
      errors = info2 & SDHI_INFO2_ERROR_MASK;

      if (errors != 0)
        {
          return (errors & SDHI_INFO2_DATA_TIMEOUT) != 0 ?
                 -ETIMEDOUT : -EIO;
        }

      if ((info1 & SDHI_INFO1_ACCESS_END) != 0)
        {
          ra_sdhi_putreg(priv, RA8P1_SDHI_SD_INFO1_OFFSET,
                         ~SDHI_INFO1_ACCESS_END);
          return OK;
        }

      if (ra_sdhi_timedout(start, priv->timeout))
        {
          return -ETIMEDOUT;
        }
    }
}

static void ra_sdhi_callback(FAR struct ra_sdhi_dev_s *priv)
{
#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
  sdio_statset_t status;
  bool notify;

  if (priv->callback == NULL)
    {
      return;
    }

  status = ra_sdhi_status(&priv->dev);
  notify = ((status & SDIO_STATUS_PRESENT) != 0 &&
            (priv->cbevents & SDIOMEDIA_INSERTED) != 0) ||
           ((status & SDIO_STATUS_PRESENT) == 0 &&
            (priv->cbevents & SDIOMEDIA_EJECTED) != 0);
  if (!notify)
    {
      return;
    }

  priv->cbevents = 0;
  if (up_interrupt_context())
    {
      work_queue(HPWORK, &priv->cbwork, priv->callback,
                 priv->cbarg, 0);
    }
  else
    {
      priv->callback(priv->cbarg);
    }
#endif
}

static int ra_sdhi_cardinterrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ra_sdhi_dev_s *priv = arg;
  uint32_t info1;

  info1 = ra_sdhi_getreg(priv, RA8P1_SDHI_SD_INFO1_OFFSET);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_INFO1_OFFSET,
                 ~(info1 & SDHI_INFO1_CARD_EVENTS));
  ra_clear_ir(irq);
  ra_sdhi_callback(priv);
  return OK;
}

static void ra_sdhi_reset(FAR struct sdio_dev_s *dev)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;

  ra_module_control(RA8P1_MSTPCRC, MSTPCRC_SDHI0, true);

  ra_sdhi_putreg(priv, RA8P1_SDHI_SOFT_RST_OFFSET, 0);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SOFT_RST_OFFSET, 1);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_INFO1_OFFSET, 0);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_INFO2_OFFSET, 0);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_INFO1_MASK_OFFSET,
                 SDHI_INFO1_MASK_CARD_DETECT);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_INFO2_MASK_OFFSET,
                 SDHI_INFO2_MASK_ALL);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_CLK_CTRL_OFFSET, 0x20);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_OPTION_OFFSET,
                 SDHI_OPTION_DEFAULT | SDHI_OPTION_WIDTH_1BIT);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SDIO_MODE_OFFSET, 0);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SDIO_MASK_OFFSET, 0xffff);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_DMAEN_OFFSET, 0);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SDIF_MODE_OFFSET, 0);
  ra_sdhi_putreg(priv, RA8P1_SDHI_EXT_SWAP_OFFSET, 0);

  priv->buffer = NULL;
  priv->remaining = 0;
  priv->waitevents = 0;
  priv->wkupevent = 0;
  priv->cbevents = 0;
  priv->cmdresult = OK;
  priv->appcmd = false;
}

static sdio_capset_t ra_sdhi_capabilities(FAR struct sdio_dev_s *dev)
{
  (void)dev;

  /* The controller requires sendsetup() before the write command.  NuttX
   * uses DMABEFOREWRITE to describe that ordering for both DMA and PIO
   * lower-halves.
   */

  return SDIO_CAPS_DMABEFOREWRITE | SDIO_CAPS_4BIT;
}

static sdio_statset_t ra_sdhi_status(FAR struct sdio_dev_s *dev)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;
  uint32_t info1;

  info1 = ra_sdhi_getreg(priv, RA8P1_SDHI_SD_INFO1_OFFSET);
  return (info1 & SDHI_INFO1_CARD_PRESENT) != 0 ?
         SDIO_STATUS_PRESENT : 0;
}

static void ra_sdhi_widebus(FAR struct sdio_dev_s *dev, bool enable)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;

  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_OPTION_OFFSET,
                 SDHI_OPTION_DEFAULT |
                 (enable ? SDHI_OPTION_WIDTH_4BIT :
                           SDHI_OPTION_WIDTH_1BIT));
}

static void ra_sdhi_clock(FAR struct sdio_dev_s *dev,
                          enum sdio_clock_e rate)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;
  int ret = OK;

  switch (rate)
    {
      case CLOCK_SDIO_DISABLED:
        modifyreg32(priv->base + RA8P1_SDHI_SD_CLK_CTRL_OFFSET,
                    SDHI_CLOCK_CONTROL_MASK, 0);
        break;

      case CLOCK_IDMODE:
        ret = ra_sdhi_setclock(priv, 400000);
        break;

      case CLOCK_SD_TRANSFER_4BIT_HS:
        ret = ra_sdhi_setclock(priv, 50000000);
        break;

      default:
        ret = ra_sdhi_setclock(priv, 25000000);
        break;
    }

  if (ret < 0)
    {
      mcerr("ERROR: Failed to set SDHI clock: %d\n", ret);
    }
}

static int ra_sdhi_attach(FAR struct sdio_dev_s *dev)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;
  int ret;

  ret = irq_attach(RA_IRQ_SDHI0_CARD, ra_sdhi_cardinterrupt, priv);
  if (ret == OK)
    {
      up_enable_irq(RA_IRQ_SDHI0_CARD);
    }

  return ret;
}

static int ra_sdhi_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t arg)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;
  uint32_t command;
  uint32_t cmdidx;
  bool data;
  int ret;

  ret = ra_sdhi_waitidle(priv);
  if (ret < 0)
    {
      priv->cmdresult = ret;
      return OK;
    }

  data = (cmd & MMCSD_DATAXFR_MASK) != MMCSD_NODATAXFR;
  if (data &&
      (priv->buffer == NULL || priv->blocklen == 0 || priv->nblocks == 0 ||
       priv->remaining != (size_t)priv->blocklen * priv->nblocks))
    {
      priv->cmdresult = -EINVAL;
      return OK;
    }

  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_INFO1_OFFSET, 0);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_INFO2_OFFSET, 0);

  if (data)
    {
      if (priv->nblocks > 1)
        {
          ra_sdhi_putreg(priv, RA8P1_SDHI_SD_STOP_OFFSET,
                         SDHI_STOP_BLOCK_COUNT);
          ra_sdhi_putreg(priv, RA8P1_SDHI_SD_SECCNT_OFFSET,
                         priv->nblocks);
        }
      else
        {
          ra_sdhi_putreg(priv, RA8P1_SDHI_SD_STOP_OFFSET, 0);
        }

      ra_sdhi_putreg(priv, RA8P1_SDHI_SD_SIZE_OFFSET, priv->blocklen);
    }

  command = ra_sdhi_command(cmd, priv->appcmd);
  priv->appcmd = false;

  modifyreg32(priv->base + RA8P1_SDHI_SD_CLK_CTRL_OFFSET, 0,
              SDHI_CLOCK_CONTROL_MASK);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_ARG_OFFSET, arg & UINT16_MAX);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_ARG1_OFFSET, arg >> 16);
  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_CMD_OFFSET, command);

  priv->cmdresult = ra_sdhi_commandwait(priv, cmd);
  if (priv->cmdresult == OK)
    {
      ra_sdhi_saveresponse(priv, cmd);
    }

  cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  if (cmdidx == SD_CMDIDX55 && priv->cmdresult == OK)
    {
      priv->appcmd = true;
    }

  if (data)
    {
      ret = priv->cmdresult == OK ? ra_sdhi_transfer(priv) :
                                    priv->cmdresult;
      priv->wkupevent = ret == OK ? SDIOWAIT_TRANSFERDONE :
                         ret == -ETIMEDOUT ? SDIOWAIT_TIMEOUT :
                                            SDIOWAIT_ERROR;
      priv->buffer = NULL;
      priv->remaining = 0;
    }

  return OK;
}

#ifdef CONFIG_SDIO_BLOCKSETUP
static void ra_sdhi_blocksetup(FAR struct sdio_dev_s *dev,
                               unsigned int blocklen,
                               unsigned int nblocks)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;

  priv->blocklen = blocklen;
  priv->nblocks = nblocks;
}
#endif

static int ra_sdhi_recvsetup(FAR struct sdio_dev_s *dev,
                             FAR uint8_t *buffer, size_t nbytes)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;

  if (buffer == NULL || nbytes == 0)
    {
      return -EINVAL;
    }

  priv->buffer = buffer;
  priv->remaining = nbytes;
  priv->receive = true;
  return OK;
}

static int ra_sdhi_sendsetup(FAR struct sdio_dev_s *dev,
                             FAR const uint8_t *buffer, size_t nbytes)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;

  if (buffer == NULL || nbytes == 0)
    {
      return -EINVAL;
    }

  priv->buffer = (FAR uint8_t *)buffer;
  priv->remaining = nbytes;
  priv->receive = false;
  return OK;
}

static int ra_sdhi_cancel(FAR struct sdio_dev_s *dev)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;

  priv->buffer = NULL;
  priv->remaining = 0;
  priv->wkupevent = SDIOWAIT_ERROR;
  ra_sdhi_putreg(priv, RA8P1_SDHI_SD_STOP_OFFSET, 1);
  return OK;
}

static int ra_sdhi_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;

  (void)cmd;
  return priv->cmdresult;
}

static int ra_sdhi_recv_r1(FAR struct sdio_dev_s *dev, uint32_t cmd,
                           FAR uint32_t *r1)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;

  if (r1 == NULL)
    {
      return -EINVAL;
    }

  (void)cmd;
  *r1 = priv->response[0];
  return priv->cmdresult;
}

static int ra_sdhi_recv_r2(FAR struct sdio_dev_s *dev, uint32_t cmd,
                           FAR uint32_t r2[4])
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;

  if (r2 == NULL)
    {
      return -EINVAL;
    }

  (void)cmd;
  memcpy(r2, priv->response, sizeof(priv->response));
  return priv->cmdresult;
}

static int ra_sdhi_recv_r3(FAR struct sdio_dev_s *dev, uint32_t cmd,
                           FAR uint32_t *r3)
{
  return ra_sdhi_recv_r1(dev, cmd, r3);
}

static void ra_sdhi_waitenable(FAR struct sdio_dev_s *dev,
                               sdio_eventset_t eventset, uint32_t timeout)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;

  priv->waitevents = eventset;
  priv->wkupevent = 0;
  priv->timeout = timeout;
}

static sdio_eventset_t ra_sdhi_eventwait(FAR struct sdio_dev_s *dev)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;
  sdio_eventset_t event;

  event = priv->wkupevent & priv->waitevents;
  if (event == 0)
    {
      event = (priv->waitevents & SDIOWAIT_TIMEOUT) != 0 ?
              SDIOWAIT_TIMEOUT : SDIOWAIT_ERROR;
    }

  priv->waitevents = 0;
  priv->wkupevent = 0;
  return event;
}

static void ra_sdhi_callbackenable(FAR struct sdio_dev_s *dev,
                                   sdio_eventset_t eventset)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;

  priv->cbevents = eventset;
  ra_sdhi_callback(priv);
}

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
static int ra_sdhi_registercallback(FAR struct sdio_dev_s *dev,
                                    worker_t callback, FAR void *arg)
{
  FAR struct ra_sdhi_dev_s *priv = (FAR struct ra_sdhi_dev_s *)dev;

  priv->cbevents = 0;
  priv->callback = callback;
  priv->cbarg = arg;
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sdio_initialize
 ****************************************************************************/

struct sdio_dev_s *sdio_initialize(int slotno)
{
  if (slotno != 0)
    {
      return NULL;
    }

  if (!g_sdhi0.initialized)
    {
      ra_sdhi_reset(&g_sdhi0.dev);
      g_sdhi0.initialized = true;
    }

  return &g_sdhi0.dev;
}

#endif /* CONFIG_RA8P1_SDHI0 */
