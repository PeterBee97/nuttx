/****************************************************************************
 * arch/arm/src/ra8p1/ra_wdt.c
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

#include <nuttx/irq.h>
#include <nuttx/timers/watchdog.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/ra8p1_wdt.h"
#include "ra_wdt.h"

#ifdef CONFIG_RA8P1_WDT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_WDT_CLOCK_DIVIDER  8192u
#define RA8P1_WDT_TIMEOUT_CLOCKS 16384u
#define RA8P1_WDT_TIMEOUT_MS \
  ((uint32_t)(((uint64_t)RA8P1_WDT_TIMEOUT_CLOCKS * \
               RA8P1_WDT_CLOCK_DIVIDER * 1000) / \
              BOARD_PCLKB_FREQUENCY))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ops must remain first for watchdog_lowerhalf_s cast compatibility. */

struct ra_wdt_s
{
  FAR const struct watchdog_ops_s *ops;
  uint32_t timeout;
  bool started;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra_wdt_start(FAR struct watchdog_lowerhalf_s *lower);
static int ra_wdt_stop(FAR struct watchdog_lowerhalf_s *lower);
static int ra_wdt_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int ra_wdt_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                            FAR struct watchdog_status_s *status);
static int ra_wdt_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                             uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct watchdog_ops_s g_ra_wdt_ops =
{
  .start      = ra_wdt_start,
  .stop       = ra_wdt_stop,
  .keepalive  = ra_wdt_keepalive,
  .getstatus  = ra_wdt_getstatus,
  .settimeout = ra_wdt_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

static struct ra_wdt_s g_ra_wdt =
{
  .ops = &g_ra_wdt_ops,
  .timeout = RA8P1_WDT_TIMEOUT_MS,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ra_wdt_refresh(void)
{
  putreg8(RA8P1_WDT_REFRESH_FIRST, RA8P1_WDT_WDTRR);
  putreg8(RA8P1_WDT_REFRESH_SECOND, RA8P1_WDT_WDTRR);
}

static int ra_wdt_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct ra_wdt_s *priv = (FAR struct ra_wdt_s *)lower;
  irqstate_t flags;

  if (!priv->started)
    {
      flags = enter_critical_section();
      ra_wdt_refresh();
      priv->started = true;
      leave_critical_section(flags);
    }

  return OK;
}

static int ra_wdt_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct ra_wdt_s *priv = (FAR struct ra_wdt_s *)lower;

  /* RA8P1 uses register-start mode.  Once refreshed, the watchdog cannot
   * be stopped until reset.
   */

  return priv->started ? -ENOSYS : OK;
}

static int ra_wdt_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct ra_wdt_s *priv = (FAR struct ra_wdt_s *)lower;
  irqstate_t flags;

  if (!priv->started)
    {
      return -EIO;
    }

  flags = enter_critical_section();
  ra_wdt_refresh();
  leave_critical_section(flags);
  return OK;
}

static int ra_wdt_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                            FAR struct watchdog_status_s *status)
{
  FAR struct ra_wdt_s *priv = (FAR struct ra_wdt_s *)lower;
  uint32_t counter;
  uint32_t timeleft;

  status->flags = WDFLAGS_RESET;
  status->timeout = priv->timeout;

  if (!priv->started)
    {
      status->timeleft = priv->timeout;
      return OK;
    }

  status->flags |= WDFLAGS_ACTIVE;
  counter = getreg16(RA8P1_WDT_WDTSR) & RA8P1_WDT_COUNTER_MASK;
  timeleft = (uint32_t)(((uint64_t)counter * RA8P1_WDT_CLOCK_DIVIDER *
                         1000) / BOARD_PCLKB_FREQUENCY);
  status->timeleft = timeleft > priv->timeout ? priv->timeout : timeleft;
  return OK;
}

static int ra_wdt_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                             uint32_t timeout)
{
  FAR struct ra_wdt_s *priv = (FAR struct ra_wdt_s *)lower;

  if (timeout != RA8P1_WDT_TIMEOUT_MS)
    {
      return -ERANGE;
    }

  if (priv->started)
    {
      return ra_wdt_keepalive(lower);
    }

  priv->timeout = timeout;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_wdt_initialize
 ****************************************************************************/

FAR struct watchdog_lowerhalf_s *ra_wdt_initialize(void)
{
  uint16_t control;

  control = RA8P1_WDT_TIMEOUT_16384 | RA8P1_WDT_CLOCK_DIV_8192 |
            RA8P1_WDT_WINDOW_END_0 | RA8P1_WDT_WINDOW_START_100;

  putreg8(RA8P1_WDT_RESET_ENABLE, RA8P1_WDT_WDTRCR);
  putreg16(control, RA8P1_WDT_WDTCR);
#ifdef CONFIG_RA8P1_WDT_STOP_IN_SLEEP
  putreg8(RA8P1_WDT_SLEEP_STOP, RA8P1_WDT_WDTCSTPR);
#else
  putreg8(0, RA8P1_WDT_WDTCSTPR);
#endif

  g_ra_wdt.timeout = RA8P1_WDT_TIMEOUT_MS;
  g_ra_wdt.started = false;
  return (FAR struct watchdog_lowerhalf_s *)&g_ra_wdt;
}

#endif /* CONFIG_RA8P1_WDT */
