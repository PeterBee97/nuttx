/****************************************************************************
 * arch/arm/src/ra8p1/ra_pwm.c
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

#include <sys/param.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/timers/pwm.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/ra8p1_gpt.h"
#include "hardware/ra8p1_system.h"
#include "ra_pinmux.h"
#include "ra_pwm.h"

#ifdef CONFIG_RA8P1_GPT_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_PWM_OUTPUT_A          (1u << 0)
#define RA_PWM_OUTPUT_B          (1u << 1)
#define RA_PWM_GROUP_MSTP        (1u << 27)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_pwm_prescaler_s
{
  uint16_t divisor;
  uint8_t tpcs;
};

/* ops must remain first for pwm_lowerhalf_s cast compatibility. */

struct ra_pwm_s
{
  FAR const struct pwm_ops_s *ops;
  uintptr_t base;
  uint32_t pin_a;
  uint32_t pin_b;
  uint8_t channel;
  uint8_t outputs;
  bool setup;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra_pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int ra_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);
static int ra_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                        FAR const struct pwm_info_s *info);
static int ra_pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int ra_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                        unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pwm_ops_s g_ra_pwm_ops =
{
  .setup = ra_pwm_setup,
  .shutdown = ra_pwm_shutdown,
  .start = ra_pwm_start,
  .stop = ra_pwm_stop,
  .ioctl = ra_pwm_ioctl,
};

static const struct ra_pwm_prescaler_s g_prescalers[] =
{
  {    1, 0 },
  {    2, 1 },
  {    4, 2 },
  {    8, 3 },
  {   16, 4 },
  {   32, 5 },
  {   64, 6 },
  {  256, 7 },
  { 1024, 8 },
};

#ifdef CONFIG_RA8P1_GPT6_PWM
static struct ra_pwm_s g_pwm6 =
{
  .ops = &g_ra_pwm_ops,
  .base = RA8P1_GPT6_BASE,
  .pin_a = BOARD_GPT6_OUTPUTA_PIN,
  .channel = 6,
  .outputs = RA_PWM_OUTPUT_A,
};
#endif

#ifdef CONFIG_RA8P1_GPT7_PWM
static struct ra_pwm_s g_pwm7 =
{
  .ops = &g_ra_pwm_ops,
  .base = RA8P1_GPT7_BASE,
  .pin_b = BOARD_GPT7_OUTPUTB_PIN,
  .channel = 7,
  .outputs = RA_PWM_OUTPUT_B,
};
#endif

#ifdef CONFIG_RA8P1_GPT9_PWM
static struct ra_pwm_s g_pwm9 =
{
  .ops = &g_ra_pwm_ops,
  .base = RA8P1_GPT9_BASE,
  .pin_b = BOARD_GPT9_OUTPUTB_PIN,
  .channel = 9,
  .outputs = RA_PWM_OUTPUT_B,
};
#endif

#ifdef CONFIG_RA8P1_GPT10_PWM
static struct ra_pwm_s g_pwm10 =
{
  .ops = &g_ra_pwm_ops,
  .base = RA8P1_GPT10_BASE,
  .pin_a = BOARD_GPT10_OUTPUTA_PIN,
  .pin_b = BOARD_GPT10_OUTPUTB_PIN,
  .channel = 10,
  .outputs = RA_PWM_OUTPUT_A | RA_PWM_OUTPUT_B,
};
#endif

/* GPT channels 4 through 9 share one module-stop bit. */

static uint8_t g_group_users;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void ra_pwm_putreg(FAR struct ra_pwm_s *priv,
                                 uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static void ra_pwm_write_enable(FAR struct ra_pwm_s *priv, bool enable)
{
  ra_pwm_putreg(priv, RA8P1_GPT_GTWP_OFFSET,
                enable ? GPT_GTWP_ENABLE : GPT_GTWP_DISABLE);
}

static uint32_t ra_pwm_mstp_mask(FAR struct ra_pwm_s *priv)
{
  if (priv->channel >= 4 && priv->channel <= 9)
    {
      return RA_PWM_GROUP_MSTP;
    }

  return 1u << (31 - priv->channel);
}

static void ra_pwm_module_control(FAR struct ra_pwm_s *priv, bool enable)
{
  irqstate_t flags;
  uint32_t mstp;
  bool update = false;

  flags = enter_critical_section();

  if (enable && !priv->setup)
    {
      if (priv->channel >= 4 && priv->channel <= 9)
        {
          update = g_group_users++ == 0;
        }
      else
        {
          update = true;
        }

      priv->setup = true;
    }
  else if (!enable && priv->setup)
    {
      if (priv->channel >= 4 && priv->channel <= 9)
        {
          update = --g_group_users == 0;
        }
      else
        {
          update = true;
        }

      priv->setup = false;
    }

  if (update)
    {
      mstp = ra_pwm_mstp_mask(priv);

      if (enable)
        {
          modifyreg32(RA8P1_MSTPCRE, mstp, 0);
          (void)getreg32(RA8P1_MSTPCRE);
        }
      else
        {
          modifyreg32(RA8P1_MSTPCRE, 0, mstp);
        }
    }

  leave_critical_section(flags);

  if (enable && update)
    {
      up_udelay(30);
    }
}

static uint32_t ra_pwm_gtior(FAR struct ra_pwm_s *priv)
{
  uint32_t gtior = 0;

  if ((priv->outputs & RA_PWM_OUTPUT_A) != 0)
    {
      gtior |= GPT_GTIOR_GTIOA_PWM | GPT_GTIOR_OAE;
    }

  if ((priv->outputs & RA_PWM_OUTPUT_B) != 0)
    {
      gtior |= GPT_GTIOR_GTIOB_PWM | GPT_GTIOR_OBE;
    }

  return gtior;
}

static void ra_pwm_set_duty(FAR struct ra_pwm_s *priv, uint32_t duty)
{
  if ((priv->outputs & RA_PWM_OUTPUT_A) != 0)
    {
      ra_pwm_putreg(priv, RA8P1_GPT_GTCCRA_OFFSET, duty - 1);
    }

  if ((priv->outputs & RA_PWM_OUTPUT_B) != 0)
    {
      ra_pwm_putreg(priv, RA8P1_GPT_GTCCRB_OFFSET, duty - 1);
    }
}

static void ra_pwm_hw_stop(FAR struct ra_pwm_s *priv)
{
  ra_pwm_write_enable(priv, false);
  ra_pwm_putreg(priv, RA8P1_GPT_GTSTP_OFFSET,
                GPT_CHANNEL(priv->channel));
  ra_pwm_write_enable(priv, true);
}

static int ra_pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct ra_pwm_s *priv = (FAR struct ra_pwm_s *)dev;
  uint32_t period = BOARD_GPT_FREQUENCY / 1000;

  ra_pwm_module_control(priv, true);

  if ((priv->outputs & RA_PWM_OUTPUT_A) != 0)
    {
      ra_pinmux_configure(priv->pin_a);
    }

  if ((priv->outputs & RA_PWM_OUTPUT_B) != 0)
    {
      ra_pinmux_configure(priv->pin_b);
    }

  ra_pwm_write_enable(priv, false);
  ra_pwm_putreg(priv, RA8P1_GPT_GTSTP_OFFSET,
                GPT_CHANNEL(priv->channel));
  ra_pwm_putreg(priv, RA8P1_GPT_GTCR_OFFSET, 0);
  ra_pwm_putreg(priv, RA8P1_GPT_GTSSR_OFFSET,
                GPT_GROUP_SOFTWARE_UPDATE);
  ra_pwm_putreg(priv, RA8P1_GPT_GTPSR_OFFSET,
                GPT_GROUP_SOFTWARE_UPDATE);
  ra_pwm_putreg(priv, RA8P1_GPT_GTCSR_OFFSET,
                GPT_GROUP_SOFTWARE_UPDATE);
  ra_pwm_putreg(priv, RA8P1_GPT_GTCNT_OFFSET, 0);
  ra_pwm_putreg(priv, RA8P1_GPT_GTPR_OFFSET, period - 1);
  ra_pwm_set_duty(priv, period / 2);
  ra_pwm_putreg(priv, RA8P1_GPT_GTIOR_OFFSET, ra_pwm_gtior(priv));
  ra_pwm_putreg(priv, RA8P1_GPT_GTUDDTYC_OFFSET, 3);
  ra_pwm_putreg(priv, RA8P1_GPT_GTUDDTYC_OFFSET, 1);
  ra_pwm_putreg(priv, RA8P1_GPT_GTCLR_OFFSET,
                GPT_CHANNEL(priv->channel));
  ra_pwm_write_enable(priv, true);

  return OK;
}

static int ra_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct ra_pwm_s *priv = (FAR struct ra_pwm_s *)dev;

  ra_pwm_hw_stop(priv);
  ra_pwm_write_enable(priv, false);
  ra_pwm_putreg(priv, RA8P1_GPT_GTIOR_OFFSET, 0);
  ra_pwm_write_enable(priv, true);
  ra_pwm_module_control(priv, false);
  return OK;
}

static int ra_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                        FAR const struct pwm_info_s *info)
{
  FAR struct ra_pwm_s *priv = (FAR struct ra_pwm_s *)dev;
  uint64_t maxcount;
  uint64_t period;
  uint64_t duty;
  unsigned int i;

  if (info == NULL || info->frequency == 0)
    {
      return -EINVAL;
    }

  maxcount = UINT32_MAX;

  for (i = 0; i < nitems(g_prescalers); i++)
    {
      period = BOARD_GPT_FREQUENCY /
               ((uint64_t)g_prescalers[i].divisor * info->frequency);
      if (period >= 2 && period <= maxcount)
        {
          break;
        }
    }

  if (i == nitems(g_prescalers))
    {
      return -ERANGE;
    }

  duty = (period * info->channels[0].duty) >> 16;
  if (duty == 0)
    {
      duty = 1;
    }
  else if (duty >= period)
    {
      duty = period - 1;
    }

  ra_pwm_hw_stop(priv);
  ra_pwm_write_enable(priv, false);
  ra_pwm_putreg(priv, RA8P1_GPT_GTCR_OFFSET,
                (uint32_t)g_prescalers[i].tpcs << GPT_GTCR_TPCS_SHIFT);
  ra_pwm_putreg(priv, RA8P1_GPT_GTCNT_OFFSET, 0);
  ra_pwm_putreg(priv, RA8P1_GPT_GTPR_OFFSET, (uint32_t)period - 1);
  ra_pwm_set_duty(priv, (uint32_t)duty);
  ra_pwm_putreg(priv, RA8P1_GPT_GTCLR_OFFSET,
                GPT_CHANNEL(priv->channel));
  ra_pwm_write_enable(priv, true);
  ra_pwm_putreg(priv, RA8P1_GPT_GTSTR_OFFSET,
                GPT_CHANNEL(priv->channel));
  return OK;
}

static int ra_pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct ra_pwm_s *priv = (FAR struct ra_pwm_s *)dev;

  ra_pwm_hw_stop(priv);
  return OK;
}

static int ra_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                        unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_pwm_initialize
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *ra_pwm_initialize(int channel)
{
  switch (channel)
    {
#ifdef CONFIG_RA8P1_GPT6_PWM
      case 6:
        return (FAR struct pwm_lowerhalf_s *)&g_pwm6;
#endif

#ifdef CONFIG_RA8P1_GPT7_PWM
      case 7:
        return (FAR struct pwm_lowerhalf_s *)&g_pwm7;
#endif

#ifdef CONFIG_RA8P1_GPT9_PWM
      case 9:
        return (FAR struct pwm_lowerhalf_s *)&g_pwm9;
#endif

#ifdef CONFIG_RA8P1_GPT10_PWM
      case 10:
        return (FAR struct pwm_lowerhalf_s *)&g_pwm10;
#endif

      default:
        return NULL;
    }
}

#endif /* CONFIG_RA8P1_GPT_PWM */
