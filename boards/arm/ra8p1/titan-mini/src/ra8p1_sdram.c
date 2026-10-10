/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_sdram.c
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

#include <sys/param.h>
#include <errno.h>
#include <stdint.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "ra_pinmux.h"
#include "ra_sdram.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TITAN_BUS_PIN(port, pin) \
  (RA8P1_PINMUX(port, pin, 11) | RA8P1_PINMUX_DRIVE_HIGH)
#define TITAN_BUS_HS_PIN(port, pin) \
  (RA8P1_PINMUX(port, pin, 11) | RA8P1_PINMUX_DRIVE_HS_HIGH)

#define TITAN_SDRAM_TEST_PATTERN 0x5a000000u

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_sdram_pins[] =
{
  TITAN_BUS_PIN(1, 12),
  TITAN_BUS_PIN(1, 13),
  TITAN_BUS_PIN(1, 14),
  TITAN_BUS_PIN(1, 15),
  TITAN_BUS_PIN(3, 0),
  TITAN_BUS_PIN(3, 1),
  TITAN_BUS_PIN(3, 2),
  TITAN_BUS_PIN(5, 3),
  TITAN_BUS_PIN(5, 4),
  TITAN_BUS_PIN(5, 5),
  TITAN_BUS_PIN(5, 6),
  TITAN_BUS_PIN(5, 7),
  TITAN_BUS_PIN(5, 8),
  TITAN_BUS_PIN(5, 9),
  TITAN_BUS_PIN(5, 10),
  TITAN_BUS_PIN(6, 8),
  TITAN_BUS_PIN(6, 9),
  TITAN_BUS_PIN(6, 10),
  TITAN_BUS_PIN(6, 11),
  TITAN_BUS_PIN(6, 12),
  TITAN_BUS_PIN(6, 13),
  TITAN_BUS_PIN(6, 14),
  TITAN_BUS_PIN(8, 13),
  TITAN_BUS_PIN(10, 0),
  TITAN_BUS_PIN(10, 1),
  TITAN_BUS_PIN(10, 2),
  TITAN_BUS_PIN(10, 3),
  TITAN_BUS_PIN(10, 4),
  TITAN_BUS_PIN(10, 5),
  TITAN_BUS_PIN(10, 6),
  TITAN_BUS_PIN(10, 8),
  TITAN_BUS_PIN(10, 9),
  TITAN_BUS_PIN(10, 10),
  TITAN_BUS_PIN(10, 11),
  TITAN_BUS_PIN(10, 12),
  TITAN_BUS_PIN(10, 13),
  TITAN_BUS_PIN(10, 14),
  TITAN_BUS_HS_PIN(10, 15),
  TITAN_BUS_PIN(13, 0)
};

/* Probe both ends and power-of-two boundaries across the 32 MiB device. */

static const uint32_t g_test_offsets[] =
{
  0x00000000,
  0x00100000,
  0x00200000,
  0x00400000,
  0x00800000,
  0x01000000,
  0x01fffffc
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ra8p1_sdram_test(void)
{
  FAR volatile uint32_t *address;
  unsigned int i;

  for (i = 0; i < nitems(g_test_offsets); i++)
    {
      address = (FAR volatile uint32_t *)(BOARD_SDRAM_BASE +
                                          g_test_offsets[i]);
      *address = TITAN_SDRAM_TEST_PATTERN | i;
    }

  UP_DSB();

  for (i = 0; i < nitems(g_test_offsets); i++)
    {
      address = (FAR volatile uint32_t *)(BOARD_SDRAM_BASE +
                                          g_test_offsets[i]);
      if (*address != (TITAN_SDRAM_TEST_PATTERN | i))
        {
          return -EIO;
        }
    }

  for (i = 0; i < nitems(g_test_offsets); i++)
    {
      address = (FAR volatile uint32_t *)(BOARD_SDRAM_BASE +
                                          g_test_offsets[i]);
      *address = 0;
    }

  UP_DSB();
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_board_sdram_initialize
 ****************************************************************************/

int ra_board_sdram_initialize(void)
{
  unsigned int i;
  int ret;

  for (i = 0; i < nitems(g_sdram_pins); i++)
    {
      ra_pinmux_configure(g_sdram_pins[i]);
    }

  ret = ra_sdram_initialize();
  if (ret < 0)
    {
      return ret;
    }

  return ra8p1_sdram_test();
}
