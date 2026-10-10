/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_pwm.c
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
#include <stdint.h>
#include <errno.h>

#include <nuttx/timers/pwm.h>

#include "ra_pwm.h"

#if defined(CONFIG_TITAN_MINI_PWM6) || \
    defined(CONFIG_TITAN_MINI_PWM7) || \
    defined(CONFIG_TITAN_MINI_PWM9) || \
    defined(CONFIG_TITAN_MINI_PWM10)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_pwm_channels[] =
{
#ifdef CONFIG_TITAN_MINI_PWM6
  6,
#endif
#ifdef CONFIG_TITAN_MINI_PWM7
  7,
#endif
#ifdef CONFIG_TITAN_MINI_PWM9
  9,
#endif
#ifdef CONFIG_TITAN_MINI_PWM10
  10,
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8p1_pwm_initialize
 ****************************************************************************/

int ra8p1_pwm_initialize(void)
{
  FAR struct pwm_lowerhalf_s *lower;
  char path[] = "/dev/pwm0";
  unsigned int i;
  int ret;

  for (i = 0; i < nitems(g_pwm_channels); i++)
    {
      lower = ra_pwm_initialize(g_pwm_channels[i]);
      if (lower == NULL)
        {
          return -ENODEV;
        }

      path[8] = '0' + i;
      ret = pwm_register(path, lower);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

#endif
