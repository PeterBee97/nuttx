# SPDX-License-Identifier: Apache-2.0

include $(TOPDIR)/Make.defs

CSRCS = ra8p1_boot.c
ifeq ($(CONFIG_TITAN_MINI_USERLEDS),y)
CSRCS += ra8p1_userleds.c
endif
ifeq ($(CONFIG_TITAN_MINI_BUTTONS),y)
CSRCS += ra8p1_buttons.c
endif
ifeq ($(CONFIG_TITAN_MINI_GPIO),y)
CSRCS += ra8p1_gpio.c
endif
ifneq ($(filter y,$(CONFIG_TITAN_MINI_PWM6) $(CONFIG_TITAN_MINI_PWM7) \
                     $(CONFIG_TITAN_MINI_PWM9) $(CONFIG_TITAN_MINI_PWM10)),)
CSRCS += ra8p1_pwm.c
endif
ifeq ($(CONFIG_TITAN_MINI_OSPI),y)
CSRCS += ra8p1_ospi.c
endif
ifeq ($(CONFIG_TITAN_MINI_SDHI),y)
CSRCS += ra8p1_sdhi.c
endif
ifeq ($(CONFIG_TITAN_MINI_SPI1),y)
CSRCS += ra8p1_spi.c
endif
ifeq ($(CONFIG_TITAN_MINI_CANFD0),y)
CSRCS += ra8p1_canfd.c
endif
ifeq ($(CONFIG_TITAN_MINI_USBFS_CDC),y)
CSRCS += ra8p1_usb.c
endif
ifeq ($(CONFIG_TITAN_MINI_AUDIO),y)
CSRCS += ra8p1_audio.c
endif
ifeq ($(CONFIG_TITAN_MINI_ADC),y)
CSRCS += ra8p1_adc.c
endif
ifeq ($(CONFIG_TITAN_MINI_RTC),y)
CSRCS += ra8p1_rtc.c
endif
ifeq ($(CONFIG_TITAN_MINI_WDT),y)
CSRCS += ra8p1_wdt.c
endif
ifeq ($(CONFIG_RA8P1_SDRAM),y)
CSRCS += ra8p1_sdram.c
endif
ifeq ($(CONFIG_RA8P1_I2C),y)
CSRCS += ra8p1_i2c.c
endif
ifeq ($(CONFIG_TITAN_MINI_IMU),y)
CSRCS += ra8p1_imu.c
endif
ifeq ($(CONFIG_TITAN_MINI_PDM),y)
CSRCS += ra8p1_pdm.c
endif
ifeq ($(CONFIG_TITAN_MINI_CAMERA),y)
CSRCS += ra8p1_camera.c
endif
ifeq ($(CONFIG_RA8P1_ETHERNET),y)
CSRCS += ra8p1_ethernet.c
endif

ifeq ($(CONFIG_TITAN_MINI_VOICE_DEMO_AUTOSTART),y)
RCRAWS += etc/init.d/rcS
endif

include $(TOPDIR)/boards/Board.mk
