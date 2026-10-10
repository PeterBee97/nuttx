/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_wdt.c
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

#include <errno.h>

#include <nuttx/timers/watchdog.h>

#include "ra_wdt.h"

#ifdef CONFIG_TITAN_MINI_WDT

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8p1_wdt_initialize
 ****************************************************************************/

int ra8p1_wdt_initialize(void)
{
  FAR struct watchdog_lowerhalf_s *lower;

  lower = ra_wdt_initialize();
  if (lower == NULL)
    {
      return -ENODEV;
    }

  if (watchdog_register("/dev/watchdog0", lower) == NULL)
    {
      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_TITAN_MINI_WDT */
