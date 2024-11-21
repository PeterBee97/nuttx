/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_pdm_pio.c
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

#include <stdint.h>
#include <stdbool.h>

#include <arch/board/board.h>

#include "rp23xx_pdm_pio.h"
#include "rp23xx_pio.h"
#include "rp23xx_pio_instructions.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_RP23XX_PDM_PIO
  #define CONFIG_RP23XX_PDM_PIO     0
#endif

#ifndef  CONFIG_RP23XX_PDM_PIO_SM
  #define CONFIG_RP23XX_PDM_PIO_SM  0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp23xx_pdm_pio_config
{
  const rp23xx_pio_program_t program;
  uint32_t entry;
  uint32_t wrap_target;
  uint32_t wrap;
  uint32_t clocks;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PIO program for 16bit stereo PDM transfer */

static const uint16_t pio_program_pdm_16s[] = {
            /*     .wrap_target                      */
    0xe02e, /*  0: set    x, 14           side 0     */
    0x4001, /*  1: in     pins, 1         side 0     */
    0x51cf, /*  2: in     isr, 15         side 1 [1] */
    0x5001, /*  3: in     pins, 1         side 1     */
    0x40d0, /*  4: in     isr, 16         side 0     */
    0x0041, /*  5: jmp    x--, 1          side 0     */
    0x4001, /*  6: in     pins, 1         side 0     */
    0x51cf, /*  7: in     isr, 15         side 1 [1] */
    0x5001, /*  8: in     pins, 1         side 1     */
    0x8000, /*  9: push   noblock         side 0     */
            /*     .wrap                             */
};

static const uint16_t pio_program_pdm_16s_noclk[] = {
            /*     .wrap_target                      */
    0xe02e, /*  0: set    x, 14                      */
    0x4001, /*  1: in     pins, 1                    */
    0x41cf, /*  2: in     isr, 15                [1] */
    0x4001, /*  3: in     pins, 1                    */
    0x40d0, /*  4: in     isr, 16                    */
    0x0041, /*  5: jmp    x--, 1                     */
    0x4001, /*  6: in     pins, 1                    */
    0x41cf, /*  7: in     isr, 15                [1] */
    0x4001, /*  8: in     pins, 1                    */
    0x8000, /*  9: push   noblock                    */
            /*     .wrap                             */
};

/* PIO program for 16bit mono PDM transfer */

static const uint16_t pio_program_pdm_16m[] =
  {
            /*     .wrap_target                       */

    0x80a0, /*  0: pull   block           side 0      */
    0x6870, /*  1: out    null, 16        side 1      */
    0xa847, /*  2: mov    y, osr          side 1      */
    0x6101, /*  3: out    pins, 1         side 0 [1]  */
    0xe92d, /*  4: set    x, 13           side 1 [1]  */
    0x6101, /*  5: out    pins, 1         side 0 [1]  */
    0x0945, /*  6: jmp    x--, 5          side 1 [1]  */
    0x7101, /*  7: out    pins, 1         side 2 [1]  */
    0xb9e2, /*  8: mov    osr, y          side 3 [1]  */
    0x7101, /*  9: out    pins, 1         side 2 [1]  */
    0xf92d, /* 10: set    x, 13           side 3 [1]  */
    0x7101, /* 11: out    pins, 1         side 2 [1]  */
    0x194b, /* 12: jmp    x--, 11         side 3 [1]  */
    0x6001, /* 13: out    pins, 1         side 0      */

            /*     .wrap                              */
  };

/* PIO program for 8bit stereo PDM transfer */

static const uint16_t pio_program_pdm_8s[] =
  {
            /*     .wrap_target                       */

    0x80a0, /*  0: pull   block           side 0      */
    0x6078, /*  1: out    null, 24        side 0      */
    0xa9ef, /*  2: mov    osr, !osr       side 1 [1]  */
    0x6101, /*  3: out    pins, 1         side 0 [1]  */
    0xa8ef, /*  4: mov    osr, !osr       side 1      */
    0xe826, /*  5: set    x, 6            side 1      */
    0x6101, /*  6: out    pins, 1         side 0 [1]  */
    0x0946, /*  7: jmp    x--, 6          side 1 [1]  */
    0xe100, /*  8: set    pins, 0         side 0 [1]  */
    0xe925, /*  9: set    x, 5            side 1 [1]  */
    0xa142, /* 10: nop                    side 0 [1]  */
    0x094a, /* 11: jmp    x--, 10         side 1 [1]  */
    0x90a0, /* 12: pull   block           side 2      */
    0x7078, /* 13: out    null, 24        side 2      */
    0xb9ef, /* 14: mov    osr, !osr       side 3 [1]  */
    0x7101, /* 15: out    pins, 1         side 2 [1]  */
    0xb8ef, /* 16: mov    osr, !osr       side 3      */
    0xf826, /* 17: set    x, 6            side 3      */
    0x7101, /* 18: out    pins, 1         side 2 [1]  */
    0x1952, /* 19: jmp    x--, 18         side 3 [1]  */
    0xf100, /* 20: set    pins, 0         side 2 [1]  */
    0xf925, /* 21: set    x, 5            side 3 [1]  */
    0xb142, /* 22: nop                    side 2 [1]  */
    0x1956, /* 23: jmp    x--, 22         side 3 [1]  */

            /*     .wrap                              */
  };

/* PIO program for 8bit mono PDM transfer */

static const uint16_t pio_program_pdm_8m[] =
  {
            /*     .wrap_target                       */

    0x80a0, /*  0: pull   block           side 0      */
    0x6078, /*  1: out    null, 24        side 0      */
    0xa8ef, /*  2: mov    osr, !osr       side 1      */
    0xa847, /*  3: mov    y, osr          side 1      */
    0x6101, /*  4: out    pins, 1         side 0 [1]  */
    0xa8ef, /*  5: mov    osr, !osr       side 1      */
    0xe826, /*  6: set    x, 6            side 1      */
    0x6101, /*  7: out    pins, 1         side 0 [1]  */
    0x0947, /*  8: jmp    x--, 7          side 1 [1]  */
    0xe100, /*  9: set    pins, 0         side 0 [1]  */
    0xe925, /* 10: set    x, 5            side 1 [1]  */
    0xa142, /* 11: nop                    side 0 [1]  */
    0x094b, /* 12: jmp    x--, 11         side 1 [1]  */
    0xb142, /* 13: nop                    side 2 [1]  */
    0xb9e2, /* 14: mov    osr, y          side 3 [1]  */
    0x7101, /* 15: out    pins, 1         side 2 [1]  */
    0xb8ef, /* 16: mov    osr, !osr       side 3      */
    0xf826, /* 17: set    x, 6            side 3      */
    0x7101, /* 18: out    pins, 1         side 2 [1]  */
    0x1952, /* 19: jmp    x--, 18         side 3 [1]  */
    0xf100, /* 20: set    pins, 0         side 2 [1]  */
    0xf925, /* 21: set    x, 5            side 3 [1]  */
    0xb142, /* 22: nop                    side 2 [1]  */
    0x1956, /* 23: jmp    x--, 22         side 3 [1]  */

            /*     .wrap                              */
  };

/* PIO configuration table */

static const struct rp23xx_pdm_pio_config g_pio_pdm_configs[] =
  {
    [RP23XX_PDM_PIO_16BIT_STEREO] =
      {
        {
          pio_program_pdm_16s,
          sizeof(pio_program_pdm_16s) / sizeof(uint16_t),
          -1
        },
        0, 0, 9,
        64 * 3 * 2
      },

    [RP23XX_PDM_PIO_16BIT_MONO] =
      {
        {
          pio_program_pdm_16m,
          sizeof(pio_program_pdm_16m) / sizeof(uint16_t),
          -1
        },
        0, 0, 13,
        16 * 2 * 4
      },

    [RP23XX_PDM_PIO_8BIT_STEREO] =
      {
        {
          pio_program_pdm_8s,
          sizeof(pio_program_pdm_8s) / sizeof(uint16_t),
          -1
        },
        0, 0, 23,
        16 * 2 * 4
      },

    [RP23XX_PDM_PIO_8BIT_MONO] =
      {
        {
          pio_program_pdm_8m,
          sizeof(pio_program_pdm_8m) / sizeof(uint16_t),
          -1
        },
        0, 0, 23,
        16 * 2 * 4
      }
  };

static const uint32_t g_pdm_pio = CONFIG_RP23XX_PDM_PIO;
static const uint32_t g_pdm_pio_sm = CONFIG_RP23XX_PDM_PIO_SM;

/* PIO PDM status */

static int g_pio_current_mode = -1;
static uint32_t g_pio_current_samplerate;
static uint32_t g_pio_current_offset;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static float get_clkdiv(int mode, uint32_t samplerate)
{
  float div = (float)BOARD_SYS_FREQ /
              (samplerate * g_pio_pdm_configs[mode].clocks);

  return div;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_pdm_pio_configure
 *
 * Description:
 *   Configure RP23XX PIO for PDM
 *
 ****************************************************************************/

int rp23xx_pdm_pio_configure(int mode, uint32_t samplerate)
{
  const struct rp23xx_pdm_pio_config *conf;
  rp23xx_pio_sm_config sm_config;

  uint32_t data_pin = CONFIG_RP23XX_PDM_DATA;
  uint32_t clk_pin = CONFIG_RP23XX_PDM_CLOCK;

  /* Check parameters */

  if (mode < 0 || mode >= RP23XX_PDM_PIO_MAX_MODE ||
      samplerate == 0)
    {
      return -1;
    }

  if (mode == g_pio_current_mode)
    {
      if (samplerate == g_pio_current_samplerate)
        {
          return 0;
        }
      else
        {
          /* Only changing the sampling rate */

          rp23xx_pio_sm_set_clkdiv(g_pdm_pio, g_pdm_pio_sm,
                                   get_clkdiv(mode, samplerate));
          rp23xx_pio_sm_clkdiv_restart(g_pdm_pio, g_pdm_pio_sm);
          return 0;
        }
    }

  if (g_pio_current_mode < 0)
    {
      /* Claim to use PIO state machine for PDM */

      rp23xx_pio_sm_claim(g_pdm_pio, g_pdm_pio_sm);
    }
  else
    {
      /* Remove existing PIO program to change the PDM mode */

      rp23xx_pio_remove_program(CONFIG_RP23XX_PDM_PIO,
                          &g_pio_pdm_configs[g_pio_current_mode].program,
                          g_pio_current_offset);
    }

  /* Program the PIO */

  conf = &g_pio_pdm_configs[mode];
  g_pio_current_offset = rp23xx_pio_add_program(CONFIG_RP23XX_PDM_PIO,
                          &conf->program);
  g_pio_current_mode = mode;

  rp23xx_pio_gpio_init(g_pdm_pio, clk_pin);
  rp23xx_pio_gpio_init(g_pdm_pio, data_pin);
  rp23xx_pio_sm_set_consecutive_pindirs(g_pdm_pio, g_pdm_pio_sm,
                                        data_pin, 1, false);
  rp23xx_pio_sm_set_consecutive_pindirs(g_pdm_pio, g_pdm_pio_sm,
                                        clk_pin, 1, true);

  /* Configure the state machine */

  sm_config = rp23xx_pio_get_default_sm_config();
  rp23xx_sm_config_set_wrap(&sm_config,
                            g_pio_current_offset + conf->wrap_target,
                            g_pio_current_offset + conf->wrap);
  rp23xx_sm_config_set_sideset(&sm_config, 1, false, false);
  rp23xx_sm_config_set_sideset_pins(&sm_config, clk_pin);
  rp23xx_sm_config_set_in_pins(&sm_config, data_pin);
  rp23xx_sm_config_set_in_shift(&sm_config, true, false, 32);
  rp23xx_sm_config_set_fifo_join(&sm_config, RP23XX_PIO_FIFO_JOIN_RX);
  rp23xx_sm_config_set_clkdiv(&sm_config, get_clkdiv(mode, samplerate));
  rp23xx_pio_sm_init(g_pdm_pio, g_pdm_pio_sm,
                     g_pio_current_offset, &sm_config);
  rp23xx_pio_sm_exec(g_pdm_pio, g_pdm_pio_sm,
                     pio_encode_jmp(g_pio_current_offset + conf->entry));

  return 0;
}

/****************************************************************************
 * Name: rp23xx_pdm_pio_enable
 *
 * Description:
 *   Set enable PDM transfer
 *
 ****************************************************************************/

void rp23xx_pdm_pio_enable(bool enable)
{
  rp23xx_pio_sm_set_enabled(g_pdm_pio, g_pdm_pio_sm, enable);
}

/****************************************************************************
 * Name: rp23xx_pdm_pio_getdmaaddr
 *
 * Description:
 *   Get DMA peripheral address for PDM transfer
 *
 ****************************************************************************/

uintptr_t rp23xx_pdm_pio_getdmaaddr(void)
{
  return RP23XX_PIO_RXF(g_pdm_pio, g_pdm_pio_sm);
}

/****************************************************************************
 * Name: rp23xx_pdm_pio_getdmaaddr
 *
 * Description:
 *   Get DMA peripheral address for PDM transfer
 *
 ****************************************************************************/

uint8_t rp23xx_pdm_pio_getdreq(void)
{
  return RP23XX_DMA_DREQ_PIO0_RX0 + g_pdm_pio_sm + g_pdm_pio * 8;
}
