/****************************************************************************
 * arch/arm/src/ra8p1/ra_gpio.c
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

#include "arm_internal.h"
#include "hardware/ra8p1_gpio.h"
#include "hardware/ra8p1_pinmux.h"
#include "ra_gpio.h"
#include "ra_pinmux.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_gpio_configure_input
 ****************************************************************************/

void ra_gpio_configure_input(uint8_t port, uint8_t pin, bool pullup,
                             bool interrupt)
{
  uint32_t pfs = 0;

  if (pullup)
    {
      pfs |= RA8P1_PFS_PCR;
    }

  if (interrupt)
    {
      pfs |= RA8P1_PFS_ISEL;
    }

  ra_pin_configure(port, pin, pfs);
}

/****************************************************************************
 * Name: ra_gpio_configure_output
 ****************************************************************************/

void ra_gpio_configure_output(uint8_t port, uint8_t pin, bool value)
{
  uint32_t pfs = RA8P1_PFS_PDR;

  if (value)
    {
      pfs |= RA8P1_PFS_PODR;
    }

  ra_pin_configure(port, pin, pfs);
}

/****************************************************************************
 * Name: ra_gpio_read
 ****************************************************************************/

bool ra_gpio_read(uint8_t port, uint8_t pin)
{
  return (getreg32(RA8P1_PFS(port, pin)) & RA8P1_PFS_PIDR) != 0;
}

/****************************************************************************
 * Name: ra_gpio_write
 ****************************************************************************/

void ra_gpio_write(uint8_t port, uint8_t pin, bool value)
{
  putreg16(1u << pin, value ? RA8P1_PORT_POSR(port) :
                             RA8P1_PORT_PORR(port));
}
