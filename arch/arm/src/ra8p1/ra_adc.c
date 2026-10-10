/****************************************************************************
 * arch/arm/src/ra8p1/ra_adc.c
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

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "arm_internal.h"
#include "hardware/ra8p1_adc.h"
#include "hardware/ra8p1_pinmux.h"
#include "hardware/ra8p1_system.h"
#include "ra_adc.h"
#include "ra_clockconfig.h"
#include "ra_pinmux.h"

#ifdef CONFIG_RA8P1_ADC_B

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_ADC_WAIT_RETRIES      1000
#define RA8P1_ADC_SAMPLE_STATES     95
#define RA8P1_ADC_CONVERT_STATES    5
#define RA8P1_ADC_SYNC_STATES       100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_adc_s
{
  FAR const struct adc_callback_s *callback;
  uint8_t channel;
  uint8_t port;
  uint8_t pin;
  bool configured;
  bool initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra_adc_bind(FAR struct adc_dev_s *dev,
                       FAR const struct adc_callback_s *callback);
static void ra_adc_reset(FAR struct adc_dev_s *dev);
static int ra_adc_setup(FAR struct adc_dev_s *dev);
static void ra_adc_shutdown(FAR struct adc_dev_s *dev);
static void ra_adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int ra_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                        unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_ra_adc_ops =
{
  .ao_bind     = ra_adc_bind,
  .ao_reset    = ra_adc_reset,
  .ao_setup    = ra_adc_setup,
  .ao_shutdown = ra_adc_shutdown,
  .ao_rxint    = ra_adc_rxint,
  .ao_ioctl    = ra_adc_ioctl,
};

static struct ra_adc_s g_ra_adc_priv;

static struct adc_dev_s g_ra_adc_dev =
{
  .ad_ops  = &g_ra_adc_ops,
  .ad_priv = &g_ra_adc_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ra_adc_wait(uintptr_t address, uint32_t mask, uint32_t value)
{
  int retry;

  for (retry = 0; retry < RA8P1_ADC_WAIT_RETRIES; retry++)
    {
      if ((getreg32(address) & mask) == value)
        {
          return OK;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

static int ra_adc_calibrate(void)
{
  int ret;

  putreg32(RA8P1_ADC_CALEND_UNIT0 | RA8P1_ADC_CALEND_UNIT1,
           RA8P1_ADC_ADCALENDSCR);

  putreg32(RA8P1_ADC_CALIBRATE_UNIT0, RA8P1_ADC_ADCALSTR);
  ret = ra_adc_wait(RA8P1_ADC_ADCALENDSR, RA8P1_ADC_CALEND_UNIT0,
                    RA8P1_ADC_CALEND_UNIT0);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(RA8P1_ADC_CALEND_UNIT0, RA8P1_ADC_ADCALENDSCR);

  putreg32(RA8P1_ADC_CALIBRATE_UNIT1, RA8P1_ADC_ADCALSTR);
  ret = ra_adc_wait(RA8P1_ADC_ADCALENDSR, RA8P1_ADC_CALEND_UNIT1,
                    RA8P1_ADC_CALEND_UNIT1);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(RA8P1_ADC_CALEND_UNIT1, RA8P1_ADC_ADCALENDSCR);
  return OK;
}

static int ra_adc_hwsetup(FAR struct ra_adc_s *priv)
{
  uint32_t regval;
  int ret;

  ra_module_control(RA8P1_MSTPCRD, MSTPCRD_ADC, true);
  ra_pin_configure(priv->port, priv->pin, RA8P1_PFS_ANALOG);

  putreg32(0, RA8P1_ADC_ADCLKENR);
  ret = ra_adc_wait(RA8P1_ADC_ADCLKSR, RA8P1_ADC_ADCLK_ENABLE, 0);
  if (ret < 0)
    {
      goto errout;
    }

  putreg32(0, RA8P1_ADC_ADCLKCR);
  putreg32(RA8P1_ADC_ADCLK_ENABLE, RA8P1_ADC_ADCLKENR);
  ret = ra_adc_wait(RA8P1_ADC_ADCLKSR, RA8P1_ADC_ADCLK_ENABLE,
                    RA8P1_ADC_ADCLK_ENABLE);
  if (ret < 0)
    {
      goto errout;
    }

  /* Match the SDK SAR timing while using single-scan mode for NuttX
   * software triggers.
   */

  regval = RA8P1_ADC_SYNC_STATES | RA8P1_ADC_ADSYCR_DISABLE0 |
           RA8P1_ADC_ADSYCR_DISABLE1;
  putreg32(regval, RA8P1_ADC_ADSYCR);
  putreg32(0, RA8P1_ADC_ADMDR);
  putreg32(RA8P1_ADC_ADGSPCR_SAR0 | RA8P1_ADC_ADGSPCR_SAR1,
           RA8P1_ADC_ADGSPCR);
  putreg32(0, RA8P1_ADC_ADERINTCR);
  putreg32(0, RA8P1_ADC_ADOVFINTCR);
  putreg32(0, RA8P1_ADC_ADCALINTCR);
  putreg32(0, RA8P1_ADC_ADINTCR);

  regval = RA8P1_ADC_SAMPLE_STATES |
           (RA8P1_ADC_SAMPLE_STATES << 16);
  putreg32(regval, RA8P1_ADC_ADSSTR0);

  regval = RA8P1_ADC_CONVERT_STATES |
           (RA8P1_ADC_CONVERT_STATES << 8);
  putreg32(regval, RA8P1_ADC_ADCNVSTR);

  regval = RA8P1_ADC_SAMPLE_STATES |
           (RA8P1_ADC_CONVERT_STATES << 16);
  putreg32(regval, RA8P1_ADC_ADCALSTCR);

  putreg32(RA8P1_ADC_ADCHCR_GROUP0 | (priv->channel << 8),
           RA8P1_ADC_ADCHCR(0));
  putreg32(0, RA8P1_ADC_ADDOPCRA(0));
  putreg32(0, RA8P1_ADC_ADDOPCRB(0));
  putreg32(RA8P1_ADC_ADDOPCRC_UNSIGNED, RA8P1_ADC_ADDOPCRC(0));

  putreg32(1, RA8P1_ADC_ADERSCR);
  putreg32(1, RA8P1_ADC_ADOVFERSCR);
  ret = ra_adc_calibrate();
  if (ret < 0)
    {
      goto errout;
    }

  putreg32(RA8P1_ADC_GROUP0, RA8P1_ADC_ADSCANENDSCR);
  putreg32(RA8P1_ADC_GROUP0, RA8P1_ADC_ADSGER);

  priv->initialized = true;
  return OK;

errout:
  putreg32(0, RA8P1_ADC_ADCLKENR);
  ra_module_control(RA8P1_MSTPCRD, MSTPCRD_ADC, false);
  return ret;
}

static int ra_adc_trigger(FAR struct adc_dev_s *dev)
{
  FAR struct ra_adc_s *priv = dev->ad_priv;
  uint32_t result;
  int ret;

  if (!priv->initialized || priv->callback == NULL ||
      priv->callback->au_receive == NULL)
    {
      return -EIO;
    }

  putreg32(RA8P1_ADC_GROUP0, RA8P1_ADC_ADSCANENDSCR);
  putreg32(RA8P1_ADC_GROUP0, RA8P1_ADC_ADSYSTR);

  ret = ra_adc_wait(RA8P1_ADC_ADSCANENDSR, RA8P1_ADC_GROUP0,
                    RA8P1_ADC_GROUP0);
  if (ret < 0)
    {
      return ret;
    }

  result = getreg32(RA8P1_ADC_ADDR(0));
  putreg32(RA8P1_ADC_GROUP0, RA8P1_ADC_ADSCANENDSCR);

  if ((result & RA8P1_ADC_ADDR_ERROR) != 0)
    {
      return -EIO;
    }

  return priv->callback->au_receive(dev, priv->channel,
                                    result & RA8P1_ADC_ADDR_DATA_MASK);
}

static int ra_adc_bind(FAR struct adc_dev_s *dev,
                       FAR const struct adc_callback_s *callback)
{
  FAR struct ra_adc_s *priv = dev->ad_priv;

  priv->callback = callback;
  return OK;
}

static void ra_adc_reset(FAR struct adc_dev_s *dev)
{
  ra_adc_shutdown(dev);
}

static int ra_adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct ra_adc_s *priv = dev->ad_priv;

  if (priv->initialized)
    {
      return OK;
    }

  return ra_adc_hwsetup(priv);
}

static void ra_adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct ra_adc_s *priv = dev->ad_priv;

  if (priv->initialized)
    {
      putreg32(RA8P1_ADC_ADSTOP_UNIT0, RA8P1_ADC_ADSTOPR);
      putreg32(0, RA8P1_ADC_ADCLKENR);
      ra_module_control(RA8P1_MSTPCRD, MSTPCRD_ADC, false);
      priv->initialized = false;
    }
}

static void ra_adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  (void)dev;
  (void)enable;
}

static int ra_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                        unsigned long arg)
{
  (void)arg;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        return ra_adc_trigger(dev);

      case ANIOC_GET_NCHANNELS:
        return 1;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_adc_initialize
 ****************************************************************************/

FAR struct adc_dev_s *ra_adc_initialize(uint8_t channel, uint8_t port,
                                        uint8_t pin)
{
  if (channel != 0)
    {
      return NULL;
    }

  if (g_ra_adc_priv.configured)
    {
      return g_ra_adc_priv.channel == channel &&
             g_ra_adc_priv.port == port &&
             g_ra_adc_priv.pin == pin ? &g_ra_adc_dev : NULL;
    }

  g_ra_adc_priv.channel = channel;
  g_ra_adc_priv.port = port;
  g_ra_adc_priv.pin = pin;
  g_ra_adc_priv.configured = true;
  g_ra_adc_priv.initialized = false;
  return &g_ra_adc_dev;
}

#endif /* CONFIG_RA8P1_ADC_B */
