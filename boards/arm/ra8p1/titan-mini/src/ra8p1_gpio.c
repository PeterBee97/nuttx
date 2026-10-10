/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_gpio.c
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
#include <syslog.h>

#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "ra_gpio.h"

#ifdef CONFIG_TITAN_MINI_GPIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TITAN_GPIO_OUTPUTS  3
#define TITAN_GPIO_INPUTS   1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra8p1_gpio_pin_s
{
  struct gpio_dev_s gpio;
  uint8_t port;
  uint8_t pin;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra8p1_gpio_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int ra8p1_gpio_write(FAR struct gpio_dev_s *dev, bool value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s g_input_ops =
{
  .go_read   = ra8p1_gpio_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};

static const struct gpio_operations_s g_output_ops =
{
  .go_read   = ra8p1_gpio_read,
  .go_write  = ra8p1_gpio_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

static struct ra8p1_gpio_pin_s g_outputs[TITAN_GPIO_OUTPUTS] =
{
  {
    .port = BOARD_LED_GREEN_PORT,
    .pin = BOARD_LED_GREEN_PIN,
  },
  {
    .port = BOARD_LED_RED_PORT,
    .pin = BOARD_LED_RED_PIN,
  },
  {
    .port = BOARD_LED_BLUE_PORT,
    .pin = BOARD_LED_BLUE_PIN,
  },
};

static struct ra8p1_gpio_pin_s g_inputs[TITAN_GPIO_INPUTS] =
{
  {
    .port = BOARD_BUTTON_USER_PORT,
    .pin = BOARD_BUTTON_USER_PIN,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ra8p1_gpio_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct ra8p1_gpio_pin_s *pin = (FAR struct ra8p1_gpio_pin_s *)dev;

  if (value == NULL)
    {
      return -EINVAL;
    }

  *value = ra_gpio_read(pin->port, pin->pin);
  return OK;
}

static int ra8p1_gpio_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct ra8p1_gpio_pin_s *pin = (FAR struct ra8p1_gpio_pin_s *)dev;

  ra_gpio_write(pin->port, pin->pin, value);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8p1_gpio_initialize
 ****************************************************************************/

int ra8p1_gpio_initialize(void)
{
  unsigned int i;
  int ret;

  for (i = 0; i < TITAN_GPIO_OUTPUTS; i++)
    {
      g_outputs[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_outputs[i].gpio.gp_ops = &g_output_ops;

      ra_gpio_configure_output(g_outputs[i].port, g_outputs[i].pin, true);
      ret = gpio_pin_register(&g_outputs[i].gpio, i);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register GPIO output %u: %d\n",
                 i, ret);
          return ret;
        }
    }

  for (i = 0; i < TITAN_GPIO_INPUTS; i++)
    {
      g_inputs[i].gpio.gp_pintype = GPIO_INPUT_PIN_PULLUP;
      g_inputs[i].gpio.gp_ops = &g_input_ops;

      ra_gpio_configure_input(g_inputs[i].port, g_inputs[i].pin,
                              true, false);
      ret = gpio_pin_register(&g_inputs[i].gpio, TITAN_GPIO_OUTPUTS + i);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register GPIO input %u: %d\n",
                 i, ret);
          return ret;
        }
    }

  return OK;
}

#endif /* CONFIG_TITAN_MINI_GPIO */
