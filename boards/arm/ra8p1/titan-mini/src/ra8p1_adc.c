/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_adc.c
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

#include <nuttx/analog/adc.h>

#include <arch/board/board.h>

#include "ra_adc.h"

#ifdef CONFIG_TITAN_MINI_ADC

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8p1_adc_initialize
 ****************************************************************************/

int ra8p1_adc_initialize(void)
{
  FAR struct adc_dev_s *adc;

  adc = ra_adc_initialize(BOARD_ADC0_CHANNEL, BOARD_ADC0_PORT,
                          BOARD_ADC0_PIN);
  if (adc == NULL)
    {
      return -ENODEV;
    }

  return adc_register("/dev/adc0", adc);
}

#endif /* CONFIG_TITAN_MINI_ADC */
