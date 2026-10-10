# SPDX-License-Identifier: Apache-2.0

set(SRCS
    ra_start.c
    ra_clockconfig.c
    ra_irq.c
    ra_icu.c
    ra_gpio.c
    ra_pinmux.c
    ra_allocateheap.c
    ra_timerisr.c
    ra_serial.c)

if(CONFIG_RA8P1_SDRAM)
  list(APPEND SRCS ra_sdram.c)
endif()

if(CONFIG_RA8P1_I2C)
  list(APPEND SRCS ra_i2c.c)
endif()

if(CONFIG_RA8P1_OSPI_B)
  list(APPEND SRCS ra_ospi.c)
endif()

if(CONFIG_RA8P1_SDHI0)
  list(APPEND SRCS ra_sdhi.c)
endif()

if(CONFIG_RA8P1_SPI1)
  list(APPEND SRCS ra_spi.c)
endif()

if(CONFIG_RA8P1_CANFD0)
  list(APPEND SRCS ra_canfd.c)
endif()

if(CONFIG_RA8P1_GPT_PWM)
  list(APPEND SRCS ra_pwm.c)
endif()

if(CONFIG_RA8P1_ADC_B)
  list(APPEND SRCS ra_adc.c)
endif()

if(CONFIG_RA8P1_RTC)
  list(APPEND SRCS ra_rtc.c)
endif()

if(CONFIG_RA8P1_WDT)
  list(APPEND SRCS ra_wdt.c)
endif()

if(CONFIG_RA8P1_USBFS)
  list(APPEND SRCS ra_usbdev.c)
endif()

if(CONFIG_RA8P1_SSI0)
  list(APPEND SRCS ra_i2s.c)
endif()

if(CONFIG_RA8P1_ETHOSU55)
  target_include_directories(arch PRIVATE ${NUTTX_DIR}/drivers/aie/ethosu)
  list(APPEND SRCS ra_ethosu.c)
endif()

if(CONFIG_RA8P1_ETHERNET)
  list(APPEND SRCS ra_ethernet.c)
endif()

if(CONFIG_RA8P1_CAMERA)
  list(APPEND SRCS ra_camera.c)
endif()

target_sources(arch PRIVATE ${SRCS})
