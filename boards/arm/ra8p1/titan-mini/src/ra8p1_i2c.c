/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_i2c.c
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

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "ra_i2c.h"
#include "ra_pinmux.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ra8p1_i2c_register(int bus, uint32_t sda_pin, uint32_t scl_pin)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  ra_pinmux_configure(sda_pin);
  ra_pinmux_configure(scl_pin);

  i2c = ra_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize IIC%d\n", bus);
      return -ENODEV;
    }

  ret = i2c_register(i2c, bus);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/i2c%d: %d\n",
             bus, ret);
      ra_i2cbus_uninitialize(i2c);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8p1_i2c_initialize
 ****************************************************************************/

int ra8p1_i2c_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_RA8P1_I2C0
  ret = ra8p1_i2c_register(0, BOARD_IIC0_SDA_PIN, BOARD_IIC0_SCL_PIN);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RA8P1_I2C1
  ret = ra8p1_i2c_register(1, BOARD_IIC1_SDA_PIN, BOARD_IIC1_SCL_PIN);
#endif

  return ret;
}
