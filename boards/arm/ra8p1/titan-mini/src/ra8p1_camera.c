/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_camera.c
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
#include <nuttx/signal.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/ov5640.h>
#include <nuttx/video/v4l2_cap.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/ra8p1_gpt.h"
#include "hardware/ra8p1_system.h"
#include "ra_camera.h"
#include "ra_clockconfig.h"
#include "ra_gpio.h"
#include "ra_i2c.h"
#include "ra_pinmux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ra8p1_camera_clock_start(void)
{
  ra_pinmux_configure(BOARD_CAMERA_XCLK_PIN);

  ra_module_control(RA8P1_MSTPCRE, MSTPCRE_GPT11, true);

  putreg32(GPT_GTWP_DISABLE, RA8P1_GPT11_GTWP);
  putreg32(0, RA8P1_GPT11_GTCR);
  putreg32(GPT_GROUP_SOFTWARE_UPDATE, RA8P1_GPT11_GTSSR);
  putreg32(GPT_GROUP_SOFTWARE_UPDATE, RA8P1_GPT11_GTPSR);
  putreg32(GPT_GROUP_SOFTWARE_UPDATE, RA8P1_GPT11_GTCSR);
  putreg32(0, RA8P1_GPT11_GTCNT);
  putreg32(11, RA8P1_GPT11_GTPR);
  putreg32(5, RA8P1_GPT11_GTCCRB);
  putreg32(GPT_GTIOR_GTIOB_PWM | GPT_GTIOR_OBE, RA8P1_GPT11_GTIOR);
  putreg32(3, RA8P1_GPT11_GTUDDTYC);
  putreg32(1, RA8P1_GPT11_GTUDDTYC);
  putreg32(GPT_CHANNEL(11), RA8P1_GPT11_GTCLR);
  putreg32(GPT_GTWP_ENABLE, RA8P1_GPT11_GTWP);
  putreg32(GPT_CHANNEL(11), RA8P1_GPT11_GTSTR);
}

static void ra8p1_camera_powerup(void)
{
  ra8p1_camera_clock_start();

  ra_gpio_configure_output(BOARD_CAMERA_PWDN_PORT,
                           BOARD_CAMERA_PWDN_PIN, true);
  ra_gpio_configure_output(BOARD_CAMERA_RESET_PORT,
                           BOARD_CAMERA_RESET_PIN, false);
  nxsig_usleep(10000);

  ra_gpio_write(BOARD_CAMERA_PWDN_PORT, BOARD_CAMERA_PWDN_PIN, false);
  nxsig_usleep(10000);
  ra_gpio_write(BOARD_CAMERA_RESET_PORT, BOARD_CAMERA_RESET_PIN, true);
  nxsig_usleep(10000);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8p1_camera_initialize
 ****************************************************************************/

int ra8p1_camera_initialize(void)
{
  FAR struct imgsensor_s *sensor;
  FAR struct imgdata_s *imgdata;
  FAR struct i2c_master_s *i2c;
  int ret;

  ra8p1_camera_powerup();

  imgdata = ra_camera_initialize(BOARD_SDRAM_BASE, BOARD_SDRAM_DMA_SIZE);
  if (imgdata == NULL)
    {
      return -ENODEV;
    }

  i2c = ra_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  sensor = ov5640_initialize(i2c);
  if (sensor == NULL)
    {
      ra_i2cbus_uninitialize(i2c);
      return -ENOMEM;
    }

  imgdata_register(imgdata);
  ret = imgsensor_register(sensor);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register OV5640: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "OV5640 camera drivers registered\n");
  return OK;
}
