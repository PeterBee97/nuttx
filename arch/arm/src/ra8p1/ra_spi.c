/****************************************************************************
 * arch/arm/src/ra8p1/ra_spi.c
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

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/ra8p1_spi.h"
#include "hardware/ra8p1_system.h"
#include "ra_clockconfig.h"
#include "ra_spi.h"

#ifdef CONFIG_RA8P1_SPI1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_SPI_TIMEOUT_MS 100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_spi_dev_s
{
  struct spi_dev_s dev;
  mutex_t lock;
  uintptr_t base;
  uint32_t frequency;
  enum spi_mode_e mode;
  uint8_t nbits;
  uint8_t brdv;
  bool initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra_spi_lock(FAR struct spi_dev_s *dev, bool lock);
static void ra_spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                          bool selected);
static uint32_t ra_spi_setfrequency(FAR struct spi_dev_s *dev,
                                    uint32_t frequency);
#ifdef CONFIG_SPI_DELAY_CONTROL
static int ra_spi_setdelay(FAR struct spi_dev_s *dev, uint32_t startdelay,
                           uint32_t stopdelay, uint32_t csdelay,
                           uint32_t ifdelay);
#endif
static void ra_spi_setmode(FAR struct spi_dev_s *dev,
                           enum spi_mode_e mode);
static void ra_spi_setbits(FAR struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int ra_spi_hwfeatures(FAR struct spi_dev_s *dev,
                             spi_hwfeatures_t features);
#endif
static uint8_t ra_spi_status(FAR struct spi_dev_s *dev, uint32_t devid);
static uint32_t ra_spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static void ra_spi_exchange(FAR struct spi_dev_s *dev,
                            FAR const void *txbuffer, FAR void *rxbuffer,
                            size_t nwords);
#else
static void ra_spi_sndblock(FAR struct spi_dev_s *dev,
                            FAR const void *buffer, size_t nwords);
static void ra_spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                             size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
  .lock             = ra_spi_lock,
  .select           = ra_spi_select,
  .setfrequency     = ra_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay         = ra_spi_setdelay,
#endif
  .setmode          = ra_spi_setmode,
  .setbits          = ra_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = ra_spi_hwfeatures,
#endif
  .status           = ra_spi_status,
  .send             = ra_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = ra_spi_exchange,
#else
  .sndblock         = ra_spi_sndblock,
  .recvblock        = ra_spi_recvblock,
#endif
  .registercallback = NULL,
};

static struct ra_spi_dev_s g_spi1 =
{
  .dev =
  {
    .ops = &g_spiops,
  },
  .lock = NXMUTEX_INITIALIZER,
  .base = RA8P1_SPI1_BASE,
  .frequency = 1000000,
  .mode = SPIDEV_MODE0,
  .nbits = 8,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t ra_spi_getreg(FAR struct ra_spi_dev_s *priv,
                                     unsigned int offset)
{
  return getreg32(priv->base + offset);
}

static inline void ra_spi_putreg(FAR struct ra_spi_dev_s *priv,
                                 unsigned int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static bool ra_spi_timedout(clock_t start)
{
  clock_t ticks = MSEC2TICK(RA_SPI_TIMEOUT_MS);

  if (ticks < 1)
    {
      ticks = 1;
    }

  return clock_systime_ticks() - start >= ticks;
}

static int ra_spi_wait(FAR struct ra_spi_dev_s *priv, uint32_t mask)
{
  clock_t start = clock_systime_ticks();
  uint32_t status;

  for (; ; )
    {
      status = ra_spi_getreg(priv, RA8P1_SPI_SPSR_OFFSET);
      if ((status & SPI_SPSR_ERROR_MASK) != 0)
        {
          ra_spi_putreg(priv, RA8P1_SPI_SPSRC_OFFSET,
                        SPI_SPSRC_CLEAR_ALL);
          return -EIO;
        }

      if ((status & mask) != 0)
        {
          return OK;
        }

      if (ra_spi_timedout(start))
        {
          return -ETIMEDOUT;
        }
    }
}

static void ra_spi_configure(FAR struct ra_spi_dev_s *priv)
{
  uint32_t command;
  uint32_t control;

  control = SPI_SPCR_MASTER | SPI_SPCR_CLOCK_AUTOSTOP;
  command = ((uint32_t)priv->brdv << SPI_SPCMD_BRDV_SHIFT) |
            ((uint32_t)(priv->nbits - 1) << SPI_SPCMD_SPB_SHIFT);

  if (priv->mode == SPIDEV_MODE1 || priv->mode == SPIDEV_MODE3)
    {
      command |= SPI_SPCMD_CPHA;
    }

  if (priv->mode == SPIDEV_MODE2 || priv->mode == SPIDEV_MODE3)
    {
      command |= SPI_SPCMD_CPOL;
    }

  ra_spi_putreg(priv, RA8P1_SPI_SPCR_OFFSET,
                control & ~SPI_SPCR_MASTER);
  (void)ra_spi_getreg(priv, RA8P1_SPI_SPCR_OFFSET);
  ra_spi_putreg(priv, RA8P1_SPI_SPCR_OFFSET, control);
  ra_spi_putreg(priv, RA8P1_SPI_SPCMD0_OFFSET, command);
}

static int ra_spi_transfer(FAR struct ra_spi_dev_s *priv,
                           FAR const void *txbuffer, FAR void *rxbuffer,
                           size_t nwords)
{
  FAR const uint8_t *tx8 = txbuffer;
  FAR const uint16_t *tx16 = txbuffer;
  FAR const uint32_t *tx32 = txbuffer;
  FAR uint8_t *rx8 = rxbuffer;
  FAR uint16_t *rx16 = rxbuffer;
  FAR uint32_t *rx32 = rxbuffer;
  uint32_t txword;
  uint32_t rxword;
  size_t i;
  int ret;

  if (nwords == 0)
    {
      return OK;
    }

  if (ra_spi_getreg(priv, RA8P1_SPI_SPPSR_OFFSET) != 0)
    {
      return -EBUSY;
    }

  ra_spi_putreg(priv, RA8P1_SPI_SPFCR_OFFSET, SPI_SPFCR_RESET);
  ra_spi_putreg(priv, RA8P1_SPI_SPSRC_OFFSET, SPI_SPSRC_CLEAR_ALL);
  modifyreg32(priv->base + RA8P1_SPI_SPCR_OFFSET, 0, SPI_SPCR_ENABLE);

  for (i = 0; i < nwords; i++)
    {
      if (txbuffer == NULL)
        {
          txword = UINT32_MAX;
        }
      else if (priv->nbits <= 8)
        {
          txword = tx8[i];
        }
      else if (priv->nbits <= 16)
        {
          txword = tx16[i];
        }
      else
        {
          txword = tx32[i];
        }

      ret = ra_spi_wait(priv, SPI_SPSR_TX_EMPTY);
      if (ret < 0)
        {
          goto errout;
        }

      ra_spi_putreg(priv, RA8P1_SPI_SPDR_OFFSET, txword);
      ra_spi_putreg(priv, RA8P1_SPI_SPSRC_OFFSET, SPI_SPSRC_CLEAR_TX);

      ret = ra_spi_wait(priv, SPI_SPSR_RX_READY);
      if (ret < 0)
        {
          goto errout;
        }

      rxword = ra_spi_getreg(priv, RA8P1_SPI_SPDR_OFFSET);
      ra_spi_putreg(priv, RA8P1_SPI_SPSRC_OFFSET, SPI_SPSRC_CLEAR_RX);

      if (rxbuffer != NULL)
        {
          if (priv->nbits <= 8)
            {
              rx8[i] = rxword;
            }
          else if (priv->nbits <= 16)
            {
              rx16[i] = rxword;
            }
          else
            {
              rx32[i] = rxword;
            }
        }
    }

  ret = ra_spi_wait(priv, SPI_SPSR_COMM_END);

errout:
  modifyreg32(priv->base + RA8P1_SPI_SPCR_OFFSET, SPI_SPCR_ENABLE, 0);
  ra_spi_putreg(priv, RA8P1_SPI_SPSRC_OFFSET, SPI_SPSRC_CLEAR_ALL);
  return ret;
}

static int ra_spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct ra_spi_dev_s *priv = (FAR struct ra_spi_dev_s *)dev;

  return lock ? nxmutex_lock(&priv->lock) : nxmutex_unlock(&priv->lock);
}

static void ra_spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  ra_spi1select(dev, devid, selected);
}

static uint32_t ra_spi_setfrequency(FAR struct spi_dev_s *dev,
                                    uint32_t frequency)
{
  FAR struct ra_spi_dev_s *priv = (FAR struct ra_spi_dev_s *)dev;
  uint32_t divider;
  uint32_t factor;
  uint32_t spbr;
  uint32_t regval;
  uint8_t brdv;

  if (frequency == 0)
    {
      return priv->frequency;
    }

  divider = (BOARD_PCLKA_FREQUENCY + frequency - 1) / frequency;
  if (divider < 2)
    {
      divider = 2;
    }

  for (brdv = 0; brdv < 4; brdv++)
    {
      factor = 2u << brdv;
      spbr = (divider + factor - 1) / factor;
      if (spbr <= 256)
        {
          break;
        }
    }

  if (brdv >= 4)
    {
      brdv = 3;
      spbr = 256;
    }

  spbr--;
  priv->brdv = brdv;
  priv->frequency = BOARD_PCLKA_FREQUENCY /
                    ((spbr + 1) * (2u << brdv));

  modifyreg32(priv->base + RA8P1_SPI_SPCR_OFFSET, SPI_SPCR_ENABLE, 0);
  regval = ra_spi_getreg(priv, RA8P1_SPI_SPCR3_OFFSET);
  regval &= ~SPI_SPCR3_SPBR_MASK;
  regval |= spbr << SPI_SPCR3_SPBR_SHIFT;
  ra_spi_putreg(priv, RA8P1_SPI_SPCR3_OFFSET, regval);
  ra_spi_configure(priv);
  return priv->frequency;
}

#ifdef CONFIG_SPI_DELAY_CONTROL
static int ra_spi_setdelay(FAR struct spi_dev_s *dev, uint32_t startdelay,
                           uint32_t stopdelay, uint32_t csdelay,
                           uint32_t ifdelay)
{
  (void)dev;
  (void)startdelay;
  (void)stopdelay;
  (void)csdelay;
  (void)ifdelay;
  return -ENOSYS;
}
#endif

static void ra_spi_setmode(FAR struct spi_dev_s *dev,
                           enum spi_mode_e mode)
{
  FAR struct ra_spi_dev_s *priv = (FAR struct ra_spi_dev_s *)dev;

  if (mode > SPIDEV_MODE3 || mode == priv->mode)
    {
      return;
    }

  priv->mode = mode;
  modifyreg32(priv->base + RA8P1_SPI_SPCR_OFFSET, SPI_SPCR_ENABLE, 0);
  ra_spi_configure(priv);
}

static void ra_spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct ra_spi_dev_s *priv = (FAR struct ra_spi_dev_s *)dev;

  if (nbits < 4 || nbits > 32 || nbits == priv->nbits)
    {
      return;
    }

  priv->nbits = nbits;
  modifyreg32(priv->base + RA8P1_SPI_SPCR_OFFSET, SPI_SPCR_ENABLE, 0);
  ra_spi_configure(priv);
}

#ifdef CONFIG_SPI_HWFEATURES
static int ra_spi_hwfeatures(FAR struct spi_dev_s *dev,
                             spi_hwfeatures_t features)
{
  (void)dev;
  return features == 0 ? OK : -ENOSYS;
}
#endif

static uint8_t ra_spi_status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return ra_spi1status(dev, devid);
}

static uint32_t ra_spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct ra_spi_dev_s *priv = (FAR struct ra_spi_dev_s *)dev;
  uint32_t rd = 0;

  if (ra_spi_transfer(priv, &wd, &rd, 1) < 0)
    {
      spierr("ERROR: SPI1 word transfer failed\n");
    }

  return rd;
}

#ifdef CONFIG_SPI_EXCHANGE
static void ra_spi_exchange(FAR struct spi_dev_s *dev,
                            FAR const void *txbuffer, FAR void *rxbuffer,
                            size_t nwords)
{
  FAR struct ra_spi_dev_s *priv = (FAR struct ra_spi_dev_s *)dev;

  if (ra_spi_transfer(priv, txbuffer, rxbuffer, nwords) < 0)
    {
      spierr("ERROR: SPI1 block transfer failed\n");
    }
}
#else
static void ra_spi_sndblock(FAR struct spi_dev_s *dev,
                            FAR const void *buffer, size_t nwords)
{
  FAR struct ra_spi_dev_s *priv = (FAR struct ra_spi_dev_s *)dev;

  if (ra_spi_transfer(priv, buffer, NULL, nwords) < 0)
    {
      spierr("ERROR: SPI1 block transfer failed\n");
    }
}

static void ra_spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                             size_t nwords)
{
  FAR struct ra_spi_dev_s *priv = (FAR struct ra_spi_dev_s *)dev;

  if (ra_spi_transfer(priv, NULL, buffer, nwords) < 0)
    {
      spierr("ERROR: SPI1 block transfer failed\n");
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_spibus_initialize
 ****************************************************************************/

struct spi_dev_s *ra_spibus_initialize(int bus)
{
  int ret;

  if (bus != 1)
    {
      return NULL;
    }

  ret = nxmutex_lock(&g_spi1.lock);
  if (ret < 0)
    {
      return NULL;
    }

  if (!g_spi1.initialized)
    {
      ra_module_control(RA8P1_MSTPCRB, MSTPCRB_SPI1, true);
      ra_spi_putreg(&g_spi1, RA8P1_SPI_SPCR_OFFSET, 0);
      ra_spi_putreg(&g_spi1, RA8P1_SPI_SPSRC_OFFSET,
                    SPI_SPSRC_CLEAR_ALL);
      ra_spi_putreg(&g_spi1, RA8P1_SPI_SPDECR_OFFSET, 0);
      ra_spi_putreg(&g_spi1, RA8P1_SPI_SPCR2_OFFSET, 0);
      ra_spi_putreg(&g_spi1, RA8P1_SPI_SPDCR_OFFSET, 0);
      ra_spi_putreg(&g_spi1, RA8P1_SPI_SPDCR2_OFFSET, 0);
      ra_spi_setfrequency(&g_spi1.dev, g_spi1.frequency);
      g_spi1.initialized = true;
    }

  nxmutex_unlock(&g_spi1.lock);
  return &g_spi1.dev;
}

#endif /* CONFIG_RA8P1_SPI1 */
