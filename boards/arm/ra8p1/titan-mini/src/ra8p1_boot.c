/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_boot.c
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

#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_TITAN_MINI_USERLEDS
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_TITAN_MINI_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#include <arch/board/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_late_initialize
 ****************************************************************************/

void board_late_initialize(void)
{
#if defined(CONFIG_TITAN_MINI_USERLEDS) || \
    defined(CONFIG_TITAN_MINI_BUTTONS) || \
    defined(CONFIG_TITAN_MINI_GPIO) || \
    defined(CONFIG_TITAN_MINI_PWM6) || \
    defined(CONFIG_TITAN_MINI_PWM7) || \
    defined(CONFIG_TITAN_MINI_PWM9) || \
    defined(CONFIG_TITAN_MINI_PWM10) || \
    defined(CONFIG_TITAN_MINI_OSPI) || \
    defined(CONFIG_TITAN_MINI_SDHI) || \
    defined(CONFIG_TITAN_MINI_SPI1) || \
    defined(CONFIG_TITAN_MINI_CANFD0) || \
    defined(CONFIG_TITAN_MINI_USBFS_CDC) || \
    defined(CONFIG_TITAN_MINI_AUDIO) || \
    defined(CONFIG_TITAN_MINI_ADC) || \
    defined(CONFIG_TITAN_MINI_RTC) || \
    defined(CONFIG_TITAN_MINI_WDT) || \
    defined(CONFIG_TITAN_MINI_ETHOSU55) || \
    defined(CONFIG_RA8P1_I2C) || \
    defined(CONFIG_TITAN_MINI_IMU) || \
    defined(CONFIG_TITAN_MINI_CAMERA) || \
    defined(CONFIG_TITAN_MINI_PDM)
  int ret;
#endif

#ifdef CONFIG_TITAN_MINI_USERLEDS
  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register user LEDs: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_GPIO
  ret = ra8p1_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register GPIO nodes: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_BUTTONS
  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register USER button: %d\n", ret);
    }
#endif

#if defined(CONFIG_TITAN_MINI_PWM6) || \
    defined(CONFIG_TITAN_MINI_PWM7) || \
    defined(CONFIG_TITAN_MINI_PWM9) || \
    defined(CONFIG_TITAN_MINI_PWM10)
  ret = ra8p1_pwm_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register GPT PWM: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_OSPI
  ret = ra8p1_ospi_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register OSPI flash: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_SDHI
  ret = ra8p1_sdhi_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDHI0: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_SPI1
  ret = ra8p1_spi_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI1: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_CANFD0
  ret = ra8p1_canfd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register CANFD0: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_USBFS_CDC
  ret = ra8p1_usb_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USBFS CDC: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_ADC
  ret = ra8p1_adc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register ADC: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_RTC
  ret = ra8p1_rtc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register RTC: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_WDT
  ret = ra8p1_wdt_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register WDT: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_ETHOSU55
  ret = ra8p1_ethosu_initialize("/dev/ethosu0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register Ethos-U55: %d\n", ret);
    }
#endif

#ifdef CONFIG_RA8P1_I2C
  ret = ra8p1_i2c_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register I2C buses: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_AUDIO
  ret = ra8p1_audio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize audio: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_IMU
  ret = ra8p1_imu_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize IMU: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_CAMERA
  ret = ra8p1_camera_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize camera: %d\n", ret);
    }
#endif

#ifdef CONFIG_TITAN_MINI_PDM
  ret = ra8p1_pdm_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize PDM: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  nx_mount(NULL, "/proc", "procfs", 0, NULL);
#endif
}

/****************************************************************************
 * Name: board_app_initialize
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  return OK;
}
