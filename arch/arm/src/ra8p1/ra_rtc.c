/****************************************************************************
 * arch/arm/src/ra8p1/ra_rtc.c
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
#include <string.h>
#include <errno.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/arch_rtc.h>
#include <nuttx/timers/rtc.h>

#include "arm_internal.h"
#include "hardware/ra8p1_rtc.h"
#include "ra_rtc.h"

#ifdef CONFIG_RA8P1_RTC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_RTC_SYNC_RETRIES       1000
#define RA8P1_RTC_READ_RETRIES       3
#define RA8P1_RTC_HOUR_MASK          0x3f
#define RA8P1_RTC_WEEK_MASK          0x07
#define RA8P1_RTC_YEAR_OFFSET        100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_rtc_lowerhalf_s
{
  /* lower must remain first for rtc_lowerhalf_s cast compatibility. */

  struct rtc_lowerhalf_s lower;
  spinlock_t lock;
  bool havesettime;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
                         FAR struct rtc_time *rtctime);
static int ra_rtc_settime(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct rtc_time *rtctime);
static bool ra_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtc_ops_s g_ra_rtc_ops =
{
  .rdtime      = ra_rtc_rdtime,
  .settime     = ra_rtc_settime,
  .havesettime = ra_rtc_havesettime,
};

static struct ra_rtc_lowerhalf_s g_ra_rtc =
{
  .lower =
    {
      .ops = &g_ra_rtc_ops,
    },
  .lock = SP_UNLOCKED,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t ra_rtc_dec2bcd(uint8_t value)
{
  return (uint8_t)(((value / 10) << 4) | (value % 10));
}

static int ra_rtc_bcd2dec(uint8_t value, uint8_t maximum,
                          FAR int *result)
{
  uint8_t ones = value & 0x0f;
  uint8_t tens = value >> 4;
  int decoded;

  if (ones > 9 || tens > 9)
    {
      return -EINVAL;
    }

  decoded = tens * 10 + ones;
  if (decoded > maximum)
    {
      return -EINVAL;
    }

  *result = decoded;
  return OK;
}

static bool ra_rtc_timevalid(FAR const struct tm *time)
{
  bool leap;
  int days;
  int year;

  if (time->tm_sec < 0 || time->tm_sec > 59 ||
      time->tm_min < 0 || time->tm_min > 59 ||
      time->tm_hour < 0 || time->tm_hour > 23 ||
      time->tm_mon < 0 || time->tm_mon > 11 ||
      time->tm_year < RA8P1_RTC_YEAR_OFFSET ||
      time->tm_year > RA8P1_RTC_YEAR_OFFSET + 99 ||
      time->tm_wday < 0 || time->tm_wday > 6)
    {
      return false;
    }

  year = time->tm_year + 1900;
  leap = clock_isleapyear(year);
  days = clock_daysbeforemonth(time->tm_mon + 1, leap) -
         clock_daysbeforemonth(time->tm_mon, leap);

  return time->tm_mday >= 1 && time->tm_mday <= days;
}

static int ra_rtc_wait8(uintptr_t address, uint8_t mask, uint8_t value)
{
  int retry;

  for (retry = 0; retry < RA8P1_RTC_SYNC_RETRIES; retry++)
    {
      if ((getreg8(address) & mask) == value)
        {
          return OK;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

static int ra_rtc_start(bool start)
{
  uint8_t regval;

  regval = getreg8(RA8P1_RTC_RCR2);
  if (start)
    {
      regval |= RA8P1_RTC_RCR2_START;
    }
  else
    {
      regval &= ~RA8P1_RTC_RCR2_START;
    }

  putreg8(regval, RA8P1_RTC_RCR2);
  return ra_rtc_wait8(RA8P1_RTC_RCR2, RA8P1_RTC_RCR2_START,
                      start ? RA8P1_RTC_RCR2_START : 0);
}

static int ra_rtc_read_locked(FAR struct tm *time)
{
  uint8_t second;
  uint8_t second_check;
  uint8_t minute;
  uint8_t hour;
  uint8_t weekday;
  uint8_t day;
  uint8_t month;
  uint8_t year;
  int retry;
  int ret;

  for (retry = 0; retry < RA8P1_RTC_READ_RETRIES; retry++)
    {
      second  = getreg8(RA8P1_RTC_RSECCNT);
      minute  = getreg8(RA8P1_RTC_RMINCNT);
      hour    = getreg8(RA8P1_RTC_RHRCNT);
      weekday = getreg8(RA8P1_RTC_RWKCNT);
      day     = getreg8(RA8P1_RTC_RDAYCNT);
      month   = getreg8(RA8P1_RTC_RMONCNT);
      year    = (uint8_t)getreg16(RA8P1_RTC_RYRCNT);
      second_check = getreg8(RA8P1_RTC_RSECCNT);

      if (second == second_check)
        {
          break;
        }
    }

  if (retry >= RA8P1_RTC_READ_RETRIES)
    {
      return -EAGAIN;
    }

  memset(time, 0, sizeof(*time));

  ret = ra_rtc_bcd2dec(second, 59, &time->tm_sec);
  if (ret < 0)
    {
      return ret;
    }

  ret = ra_rtc_bcd2dec(minute, 59, &time->tm_min);
  if (ret < 0)
    {
      return ret;
    }

  ret = ra_rtc_bcd2dec(hour & RA8P1_RTC_HOUR_MASK, 23, &time->tm_hour);
  if (ret < 0)
    {
      return ret;
    }

  ret = ra_rtc_bcd2dec(weekday & RA8P1_RTC_WEEK_MASK, 6,
                       &time->tm_wday);
  if (ret < 0)
    {
      return ret;
    }

  ret = ra_rtc_bcd2dec(day, 31, &time->tm_mday);
  if (ret < 0)
    {
      return ret;
    }

  ret = ra_rtc_bcd2dec(month, 12, &time->tm_mon);
  if (ret < 0)
    {
      return ret;
    }

  time->tm_mon--;

  ret = ra_rtc_bcd2dec(year, 99, &time->tm_year);
  if (ret < 0)
    {
      return ret;
    }

  time->tm_year += RA8P1_RTC_YEAR_OFFSET;
  return ra_rtc_timevalid(time) ? OK : -EINVAL;
}

static int ra_rtc_write_locked(FAR const struct tm *time)
{
  int ret;

  if (!ra_rtc_timevalid(time))
    {
      return -EINVAL;
    }

  ret = ra_rtc_start(false);
  if (ret < 0)
    {
      return ret;
    }

  putreg8(ra_rtc_dec2bcd((uint8_t)time->tm_sec),
          RA8P1_RTC_RSECCNT);
  putreg8(ra_rtc_dec2bcd((uint8_t)time->tm_min),
          RA8P1_RTC_RMINCNT);
  putreg8(ra_rtc_dec2bcd((uint8_t)time->tm_hour),
          RA8P1_RTC_RHRCNT);
  putreg8(ra_rtc_dec2bcd((uint8_t)time->tm_wday),
          RA8P1_RTC_RWKCNT);
  putreg8(ra_rtc_dec2bcd((uint8_t)time->tm_mday),
          RA8P1_RTC_RDAYCNT);
  putreg8(ra_rtc_dec2bcd((uint8_t)(time->tm_mon + 1)),
          RA8P1_RTC_RMONCNT);
  putreg16(ra_rtc_dec2bcd((uint8_t)(time->tm_year -
                                    RA8P1_RTC_YEAR_OFFSET)),
           RA8P1_RTC_RYRCNT);

  return ra_rtc_start(true);
}

static int ra_rtc_configure_locked(void)
{
  struct tm initial =
  {
    .tm_mday = 1,
    .tm_year = RA8P1_RTC_YEAR_OFFSET,
    .tm_wday = 6,
  };

  int ret;

  putreg8(RA8P1_RTC_RCR4_RCKSEL_LOCO, RA8P1_RTC_RCR4);
  up_udelay(200);

  ret = ra_rtc_start(false);
  if (ret < 0)
    {
      return ret;
    }

  putreg16(0, RA8P1_RTC_RFRH);
  putreg16(RA8P1_RTC_RFRL_LOCO, RA8P1_RTC_RFRL);

  putreg8(0, RA8P1_RTC_RCR2);
  ret = ra_rtc_wait8(RA8P1_RTC_RCR2, RA8P1_RTC_RCR2_CNTMD, 0);
  if (ret < 0)
    {
      return ret;
    }

  putreg8(RA8P1_RTC_RCR2_RESET, RA8P1_RTC_RCR2);
  ret = ra_rtc_wait8(RA8P1_RTC_RCR2, RA8P1_RTC_RCR2_RESET, 0);
  if (ret < 0)
    {
      return ret;
    }

  putreg8(0, RA8P1_RTC_RCR1);
  ret = ra_rtc_wait8(RA8P1_RTC_RCR1, UINT8_MAX, 0);
  if (ret < 0)
    {
      return ret;
    }

  putreg8(RA8P1_RTC_RCR2_HR24, RA8P1_RTC_RCR2);
  ret = ra_rtc_wait8(RA8P1_RTC_RCR2, RA8P1_RTC_RCR2_HR24,
                     RA8P1_RTC_RCR2_HR24);
  if (ret < 0)
    {
      return ret;
    }

  return ra_rtc_write_locked(&initial);
}

static bool ra_rtc_isrunning(void)
{
  uint8_t rcr2;

  if ((getreg8(RA8P1_RTC_RCR4) & RA8P1_RTC_RCR4_RCKSEL_LOCO) == 0 ||
      getreg16(RA8P1_RTC_RFRL) != RA8P1_RTC_RFRL_LOCO)
    {
      return false;
    }

  rcr2 = getreg8(RA8P1_RTC_RCR2);
  return (rcr2 & (RA8P1_RTC_RCR2_CNTMD | RA8P1_RTC_RCR2_HR24 |
                  RA8P1_RTC_RCR2_START)) ==
         (RA8P1_RTC_RCR2_HR24 | RA8P1_RTC_RCR2_START);
}

static int ra_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
                         FAR struct rtc_time *rtctime)
{
  FAR struct ra_rtc_lowerhalf_s *priv =
    (FAR struct ra_rtc_lowerhalf_s *)lower;
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave(&priv->lock);
  ret = ra_rtc_read_locked((FAR struct tm *)rtctime);
  spin_unlock_irqrestore(&priv->lock, flags);
  return ret;
}

static int ra_rtc_settime(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct rtc_time *rtctime)
{
  FAR struct ra_rtc_lowerhalf_s *priv =
    (FAR struct ra_rtc_lowerhalf_s *)lower;
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave(&priv->lock);
  ret = ra_rtc_write_locked((FAR const struct tm *)rtctime);
  if (ret == OK)
    {
      priv->havesettime = true;
    }

  spin_unlock_irqrestore(&priv->lock, flags);
  return ret;
}

static bool ra_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  FAR struct ra_rtc_lowerhalf_s *priv =
    (FAR struct ra_rtc_lowerhalf_s *)lower;
  irqstate_t flags;
  bool havesettime;

  flags = spin_lock_irqsave(&priv->lock);
  havesettime = priv->havesettime;
  spin_unlock_irqrestore(&priv->lock, flags);
  return havesettime;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 ****************************************************************************/

int up_rtc_initialize(void)
{
  struct tm time;
  irqstate_t flags;
  bool preserved;
  int ret;

  flags = spin_lock_irqsave(&g_ra_rtc.lock);

  putreg8(0, RA8P1_RTC_RCR1);
  ret = ra_rtc_wait8(RA8P1_RTC_RCR1, UINT8_MAX, 0);
  preserved = ret == OK && ra_rtc_isrunning() &&
              ra_rtc_read_locked(&time) == OK;

  if (!preserved)
    {
      ret = ra_rtc_configure_locked();
    }

  g_ra_rtc.havesettime = preserved;
  spin_unlock_irqrestore(&g_ra_rtc.lock, flags);

  if (ret < 0)
    {
      return ret;
    }

  up_rtc_set_lowerhalf(&g_ra_rtc.lower, false);
  return OK;
}

/****************************************************************************
 * Name: ra_rtc_lowerhalf
 ****************************************************************************/

FAR struct rtc_lowerhalf_s *ra_rtc_lowerhalf(void)
{
  return &g_ra_rtc.lower;
}

#endif /* CONFIG_RA8P1_RTC */
