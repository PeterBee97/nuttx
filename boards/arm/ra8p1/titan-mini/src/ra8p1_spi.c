/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_spi.c
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

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include <arch/board/board.h>

#include "ra_gpio.h"
#include "ra_pinmux.h"
#include "ra_spi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_spi1select
 ****************************************************************************/

void ra_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  (void)dev;
  (void)devid;
  ra_gpio_write(BOARD_SPI1_CS_PORT, BOARD_SPI1_CS_PIN, !selected);
}

/****************************************************************************
 * Name: ra_spi1status
 ****************************************************************************/

uint8_t ra_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  (void)dev;
  (void)devid;
  return 0;
}

/****************************************************************************
 * Name: ra8p1_spi_initialize
 ****************************************************************************/

int ra8p1_spi_initialize(void)
{
  FAR struct spi_dev_s *spi;

  ra_pinmux_configure(BOARD_SPI1_RSPCK_PIN);
  ra_pinmux_configure(BOARD_SPI1_MOSI_PIN);
  ra_pinmux_configure(BOARD_SPI1_MISO_PIN);
  ra_gpio_configure_output(BOARD_SPI1_CS_PORT, BOARD_SPI1_CS_PIN, true);

  spi = ra_spibus_initialize(1);
  if (spi == NULL)
    {
      return -ENODEV;
    }

  return spi_register(spi, 1);
}
