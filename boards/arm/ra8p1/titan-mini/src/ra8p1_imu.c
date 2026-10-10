/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_imu.c
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
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lsm6ds3trc.h>

#include "ra_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TITAN_IMU_I2C_BUS      1
#define TITAN_IMU_I2C_ADDRESS  0x6a

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lsm6ds3trc_config_s g_imu_config =
{
  .int_pin = LSM6DS3TRC_INT1,
  .attach = NULL,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8p1_imu_initialize
 ****************************************************************************/

int ra8p1_imu_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = ra_i2cbus_initialize(TITAN_IMU_I2C_BUS);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  ret = lsm6ds3trc_register(i2c, TITAN_IMU_I2C_ADDRESS, 0,
                            &g_imu_config);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register LSM6DS3TR-C: %d\n",
             ret);
      ra_i2cbus_uninitialize(i2c);
      return ret;
    }

  syslog(LOG_INFO, "LSM6DS3TR-C IMU registered on IIC1\n");
  return OK;
}
