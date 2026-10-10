/****************************************************************************
 * boards/arm/ra8p1/titan-mini/include/board.h
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

#ifndef __BOARDS_ARM_RA8P1_TITAN_MINI_INCLUDE_BOARD_H
#define __BOARDS_ARM_RA8P1_TITAN_MINI_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOARD_XTAL_FREQUENCY  24000000
#define BOARD_CPU_FREQUENCY   1000000000
#define BOARD_SCI_FREQUENCY   100000000
#define BOARD_PCLKA_FREQUENCY 125000000
#define BOARD_PCLKB_FREQUENCY 62500000
#define BOARD_CANFD_FREQUENCY 80000000

/* The NSH console uses SCI1 on the Raspberry Pi compatible header. */

#define BOARD_SCI1_RX_PIN      RA8P1_PINMUX(7, 6, 5)
#define BOARD_SCI1_TX_PIN      RA8P1_PINMUX(7, 7, 5)

/* SCI2 is routed to PMOD1 through SW4. */

#define BOARD_SCI2_TX_PIN      RA8P1_PINMUX(8, 1, 4)
#define BOARD_SCI2_RX_PIN      RA8P1_PINMUX(8, 2, 4)

/* The onboard RGB LED is active low. */

#define BOARD_LED_GREEN_PORT    1
#define BOARD_LED_GREEN_PIN     8
#define BOARD_LED_RED_PORT      1
#define BOARD_LED_RED_PIN       9
#define BOARD_LED_BLUE_PORT     1
#define BOARD_LED_BLUE_PIN      10

#define BOARD_LED_GREEN         0
#define BOARD_LED_RED           1
#define BOARD_LED_BLUE          2
#define BOARD_NLEDS             3

#define BOARD_LED_GREEN_BIT     (1 << BOARD_LED_GREEN)
#define BOARD_LED_RED_BIT       (1 << BOARD_LED_RED)
#define BOARD_LED_BLUE_BIT      (1 << BOARD_LED_BLUE)

/* The USER/BOOT button on P2.01 is active low. */

#define BOARD_BUTTON_USER_PORT  2
#define BOARD_BUTTON_USER_PIN   1
#define BUTTON_USER             0
#define BUTTON_USER_BIT         (1 << BUTTON_USER)
#define NUM_BUTTONS             1

/* IIC0 is routed to the OV5640 control bus. */

#define BOARD_IIC0_SDA_PIN     (RA8P1_PINMUX(4, 9, 7) | \
                                RA8P1_PINMUX_DRIVE_MID)
#define BOARD_IIC0_SCL_PIN     (RA8P1_PINMUX(4, 10, 7) | \
                                RA8P1_PINMUX_DRIVE_MID)

/* IIC1 is shared by the onboard LSM6DS3TR-C and ES8156. */

#define BOARD_IIC1_SDA_PIN     (RA8P1_PINMUX(5, 11, 7) | \
                                RA8P1_PINMUX_DRIVE_MID)
#define BOARD_IIC1_SCL_PIN     (RA8P1_PINMUX(5, 12, 7) | \
                                RA8P1_PINMUX_DRIVE_MID)

/* OV5640 PWDN is active high and RESETB is active low. */

#define BOARD_CAMERA_PWDN_PORT   7
#define BOARD_CAMERA_PWDN_PIN    10
#define BOARD_CAMERA_RESET_PORT  11
#define BOARD_CAMERA_RESET_PIN   0
#define BOARD_CAMERA_XCLK_PIN    RA8P1_PINMUX(5, 0, 3)

/* The two microphones share PDM2 data and clock. */

#define BOARD_PDM_DATA_PIN       RA8P1_PINMUX(5, 2, 27)
#define BOARD_PDM_CLOCK_PIN      RA8P1_PINMUX(8, 12, 27)

/* PWM outputs configured by the Titan Mini SDK. */

#define BOARD_GPT_FREQUENCY      300000000
#define BOARD_GPT6_OUTPUTA_PIN   RA8P1_PINMUX(6, 1, 3)
#define BOARD_GPT7_OUTPUTB_PIN   RA8P1_PINMUX(3, 3, 3)
#define BOARD_GPT9_OUTPUTB_PIN   RA8P1_PINMUX(4, 10, 3)
#define BOARD_GPT10_OUTPUTA_PIN  RA8P1_PINMUX(1, 9, 3)
#define BOARD_GPT10_OUTPUTB_PIN  RA8P1_PINMUX(1, 8, 3)

/* W25Q64JVSSIQ on OSPI0 chip select 1. */

#define BOARD_OSPI0_SIO0_PIN     RA8P1_PINMUX(1, 0, 28)
#define BOARD_OSPI0_SIO1_PIN     RA8P1_PINMUX(8, 3, 28)
#define BOARD_OSPI0_SIO2_PIN     RA8P1_PINMUX(1, 3, 28)
#define BOARD_OSPI0_SIO3_PIN     RA8P1_PINMUX(1, 1, 28)
#define BOARD_OSPI0_SCLK_PIN     RA8P1_PINMUX(8, 8, 28)
#define BOARD_OSPI0_CS1_PIN      RA8P1_PINMUX(1, 4, 28)
#define BOARD_OSPI_FLASH_SIZE    0x00800000u
#define BOARD_OSPI_FS_OFFSET     0x00200000u
#define BOARD_OSPI_FS_SIZE       0x00100000u

/* Four-bit microSD slot on SDHI0.  The CD signal is active low at the
 * socket and is interpreted by the SDHI peripheral.
 */

#define BOARD_SDHI0_DAT0_PIN      RA8P1_PINMUX(13, 3, 21)
#define BOARD_SDHI0_DAT1_PIN      RA8P1_PINMUX(13, 2, 21)
#define BOARD_SDHI0_DAT2_PIN      RA8P1_PINMUX(13, 1, 21)
#define BOARD_SDHI0_DAT3_PIN      RA8P1_PINMUX(1, 11, 21)
#define BOARD_SDHI0_CMD_PIN       RA8P1_PINMUX(13, 4, 21)
#define BOARD_SDHI0_CLK_PIN       RA8P1_PINMUX(13, 5, 21)
#define BOARD_SDHI0_CD_PIN        RA8P1_PINMUX(13, 7, 21)

/* SPI1 uses a software-controlled version of the SDK SSL2 signal. */

#define BOARD_SPI1_RSPCK_PIN       RA8P1_PINMUX(1, 2, 6)
#define BOARD_SPI1_MOSI_PIN        (RA8P1_PINMUX(7, 8, 6) | \
                                    RA8P1_PINMUX_DRIVE_HIGH)
#define BOARD_SPI1_MISO_PIN        (RA8P1_PINMUX(7, 9, 6) | \
                                    RA8P1_PINMUX_DRIVE_MID)
#define BOARD_SPI1_CS_PORT         1
#define BOARD_SPI1_CS_PIN          5

/* CANFD0 is connected to an always-enabled TJA1042 transceiver. */

#define BOARD_CANFD0_RX_PIN        RA8P1_PINMUX(3, 11, 16)
#define BOARD_CANFD0_TX_PIN        RA8P1_PINMUX(3, 12, 16)

/* SSI0 drives the onboard ES8156 codec. */

#define BOARD_AUDIO_BCLK_PIN       RA8P1_PINMUX(4, 3, 18)
#define BOARD_AUDIO_WCLK_PIN       RA8P1_PINMUX(4, 4, 18)
#define BOARD_AUDIO_DATA_PIN       RA8P1_PINMUX(4, 5, 18)
#define BOARD_AUDIO_MCLK_PIN       RA8P1_PINMUX(13, 6, 3)

/* USBFS device port. */

#define BOARD_USBFS_VBUS_PIN       (RA8P1_PINMUX(4, 7, 19) | \
                                    RA8P1_PINMUX_DRIVE_HIGH)
#define BOARD_USBFS_DP_PIN         (RA8P1_PINMUX(8, 14, 19) | \
                                    RA8P1_PINMUX_DRIVE_HIGH)
#define BOARD_USBFS_DM_PIN         (RA8P1_PINMUX(8, 15, 19) | \
                                    RA8P1_PINMUX_DRIVE_HIGH)

/* ADC_B0 channel 0 is routed to P0.00 on pin 11 of the 40-pin header. */

#define BOARD_ADC0_CHANNEL        0
#define BOARD_ADC0_PORT           0
#define BOARD_ADC0_PIN            0

/* W9825G6KH-6: 32 MiB, 16-bit SDR SDRAM.  Reserve the first 2 MiB for
 * three 640x480x2 camera DMA buffers and alignment.
 */

#define BOARD_SDRAM_BASE         0x68000000u
#define BOARD_SDRAM_SIZE         0x02000000u
#define BOARD_SDRAM_DMA_SIZE     0x00200000u
#define BOARD_SDRAM_HEAP_BASE    (BOARD_SDRAM_BASE + \
                                  BOARD_SDRAM_DMA_SIZE)
#define BOARD_SDRAM_HEAP_SIZE    (BOARD_SDRAM_SIZE - \
                                  BOARD_SDRAM_DMA_SIZE)

#define BOARD_SDRAM_TRAS         6
#define BOARD_SDRAM_TRCD         3
#define BOARD_SDRAM_TRP          3
#define BOARD_SDRAM_TWR          2
#define BOARD_SDRAM_TCL          3
#define BOARD_SDRAM_TRFC         937
#define BOARD_SDRAM_TREFW        10
#define BOARD_SDRAM_INIT_ARFI    10
#define BOARD_SDRAM_INIT_ARFC    8
#define BOARD_SDRAM_INIT_PRC     3
#define BOARD_SDRAM_ADDR_SHIFT   1
#define BOARD_SDRAM_ENDIAN_MODE  0
#define BOARD_SDRAM_ACCESS_MODE  1
#define BOARD_SDRAM_BUS_WIDTH    0

#define LED_STARTED       0
#define LED_HEAPALLOCATE  1
#define LED_IRQSENABLED   2
#define LED_STACKCREATED  3
#define LED_INIRQ         4
#define LED_SIGNAL        5
#define LED_ASSERTION     6
#define LED_PANIC         7

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_RA8P1_I2C
int ra8p1_i2c_initialize(void);
#endif

#ifdef CONFIG_TITAN_MINI_ETHOSU55
int ra8p1_ethosu_initialize(FAR const char *devpath);
#endif

#ifdef CONFIG_TITAN_MINI_PDM
int ra8p1_pdm_initialize(void);
#endif

#ifdef CONFIG_TITAN_MINI_IMU
int ra8p1_imu_initialize(void);
#endif

#ifdef CONFIG_TITAN_MINI_CAMERA
int ra8p1_camera_initialize(void);
#endif

#ifdef CONFIG_TITAN_MINI_GPIO
int ra8p1_gpio_initialize(void);
#endif

#if defined(CONFIG_TITAN_MINI_PWM6) || \
    defined(CONFIG_TITAN_MINI_PWM7) || \
    defined(CONFIG_TITAN_MINI_PWM9) || \
    defined(CONFIG_TITAN_MINI_PWM10)
int ra8p1_pwm_initialize(void);
#endif

#ifdef CONFIG_TITAN_MINI_OSPI
int ra8p1_ospi_initialize(void);
#endif

#ifdef CONFIG_TITAN_MINI_SDHI
int ra8p1_sdhi_initialize(void);
#endif

#ifdef CONFIG_TITAN_MINI_SPI1
int ra8p1_spi_initialize(void);
#endif

#ifdef CONFIG_TITAN_MINI_CANFD0
int ra8p1_canfd_initialize(void);
#endif

#ifdef CONFIG_TITAN_MINI_USBFS_CDC
int ra8p1_usb_initialize(void);
#endif

#ifdef CONFIG_TITAN_MINI_AUDIO
int ra8p1_audio_initialize(void);
#endif

#ifdef CONFIG_TITAN_MINI_ADC
int ra8p1_adc_initialize(void);
#endif

#ifdef CONFIG_TITAN_MINI_RTC
int ra8p1_rtc_initialize(void);
#endif

#ifdef CONFIG_TITAN_MINI_WDT
int ra8p1_wdt_initialize(void);
#endif

#endif /* __BOARDS_ARM_RA8P1_TITAN_MINI_INCLUDE_BOARD_H */
