# SPDX-License-Identifier: Apache-2.0

set(SRCS ra8p1_boot.c)
if(CONFIG_TITAN_MINI_USERLEDS)
  list(APPEND SRCS ra8p1_userleds.c)
endif()

if(CONFIG_TITAN_MINI_BUTTONS)
  list(APPEND SRCS ra8p1_buttons.c)
endif()

if(CONFIG_TITAN_MINI_GPIO)
  list(APPEND SRCS ra8p1_gpio.c)
endif()

if(CONFIG_TITAN_MINI_PWM6
   OR CONFIG_TITAN_MINI_PWM7
   OR CONFIG_TITAN_MINI_PWM9
   OR CONFIG_TITAN_MINI_PWM10)
  list(APPEND SRCS ra8p1_pwm.c)
endif()

if(CONFIG_TITAN_MINI_OSPI)
  list(APPEND SRCS ra8p1_ospi.c)
endif()

if(CONFIG_TITAN_MINI_SDHI)
  list(APPEND SRCS ra8p1_sdhi.c)
endif()

if(CONFIG_TITAN_MINI_SPI1)
  list(APPEND SRCS ra8p1_spi.c)
endif()

if(CONFIG_TITAN_MINI_CANFD0)
  list(APPEND SRCS ra8p1_canfd.c)
endif()

if(CONFIG_TITAN_MINI_USBFS_CDC)
  list(APPEND SRCS ra8p1_usb.c)
endif()

if(CONFIG_TITAN_MINI_AUDIO)
  list(APPEND SRCS ra8p1_audio.c)
endif()

if(CONFIG_TITAN_MINI_ADC)
  list(APPEND SRCS ra8p1_adc.c)
endif()

if(CONFIG_TITAN_MINI_RTC)
  list(APPEND SRCS ra8p1_rtc.c)
endif()

if(CONFIG_TITAN_MINI_WDT)
  list(APPEND SRCS ra8p1_wdt.c)
endif()

if(CONFIG_RA8P1_SDRAM)
  list(APPEND SRCS ra8p1_sdram.c)
endif()

if(CONFIG_RA8P1_I2C)
  list(APPEND SRCS ra8p1_i2c.c)
endif()

if(CONFIG_TITAN_MINI_IMU)
  list(APPEND SRCS ra8p1_imu.c)
endif()

if(CONFIG_TITAN_MINI_PDM)
  list(APPEND SRCS ra8p1_pdm.c)
endif()

if(CONFIG_TITAN_MINI_CAMERA)
  list(APPEND SRCS ra8p1_camera.c)
endif()

if(CONFIG_RA8P1_ETHERNET)
  list(APPEND SRCS ra8p1_ethernet.c)
endif()

target_sources(board PRIVATE ${SRCS})
if(CONFIG_TITAN_MINI_VOICE_DEMO_AUTOSTART)
  nuttx_add_romfs(
    NAME
    etc
    MOUNTPOINT
    etc
    RCRAWS
    etc/init.d/rcS
    PATH
    ${CMAKE_CURRENT_BINARY_DIR}/etc)
  target_link_libraries(board PRIVATE romfs_etc)
endif()

set_property(GLOBAL PROPERTY LD_SCRIPT
                             "${NUTTX_BOARD_DIR}/scripts/titan-mini.ld")
