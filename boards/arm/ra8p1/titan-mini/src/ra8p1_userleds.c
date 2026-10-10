/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_userleds.c
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

#include <nuttx/board.h>

#include <arch/board/board.h>

#include "ra_gpio.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra8p1_led_s
{
  uint8_t port;
  uint8_t pin;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ra8p1_led_s g_leds[BOARD_NLEDS] =
{
  { BOARD_LED_GREEN_PORT, BOARD_LED_GREEN_PIN },
  { BOARD_LED_RED_PORT, BOARD_LED_RED_PIN },
  { BOARD_LED_BLUE_PORT, BOARD_LED_BLUE_PIN },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  unsigned int i;

  for (i = 0; i < BOARD_NLEDS; i++)
    {
      ra_gpio_configure_output(g_leds[i].port, g_leds[i].pin, true);
    }

  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if ((unsigned int)led < BOARD_NLEDS)
    {
      ra_gpio_write(g_leds[led].port, g_leds[led].pin, !ledon);
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  unsigned int i;

  for (i = 0; i < BOARD_NLEDS; i++)
    {
      ra_gpio_write(g_leds[i].port, g_leds[i].pin,
                    (ledset & (1u << i)) == 0);
    }
}

#endif /* !CONFIG_ARCH_LEDS */
