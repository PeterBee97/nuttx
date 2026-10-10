/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_ethernet.c
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
#include <string.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/ra8p1_memorymap.h"
#include "ra_ethernet.h"
#include "ra_pinmux.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ra_eth_config_s g_ethcfg =
{
  .port = 1,
  .mdio_port = 0,
  .phy_addr = 1,
  .etha_mask = 3
};

static const uint32_t g_ethpins[] =
{
  RA8P1_PINMUX(2, 6, 24) | RA8P1_PINMUX_DRIVE_HIGH,
  RA8P1_PINMUX(3, 4, 24) | RA8P1_PINMUX_DRIVE_HIGH,
  RA8P1_PINMUX(3, 5, 24) | RA8P1_PINMUX_DRIVE_HIGH,
  RA8P1_PINMUX(3, 6, 24) | RA8P1_PINMUX_DRIVE_HIGH,
  RA8P1_PINMUX(3, 7, 24) | RA8P1_PINMUX_DRIVE_HIGH,
  RA8P1_PINMUX(3, 9, 24) | RA8P1_PINMUX_DRIVE_MID,
  RA8P1_PINMUX(3, 10, 24) | RA8P1_PINMUX_DRIVE_HIGH,
  RA8P1_PINMUX(9, 5, 24) | RA8P1_PINMUX_DRIVE_HIGH,
  RA8P1_PINMUX(9, 6, 24) | RA8P1_PINMUX_DRIVE_HIGH,
  RA8P1_PINMUX(9, 7, 24) | RA8P1_PINMUX_DRIVE_HIGH,
  RA8P1_PINMUX(9, 8, 24) | RA8P1_PINMUX_DRIVE_HIGH,
  RA8P1_PINMUX(9, 9, 24) | RA8P1_PINMUX_DRIVE_HIGH,
  RA8P1_PINMUX(12, 11, 24) | RA8P1_PINMUX_DRIVE_HIGH,
  RA8P1_PINMUX(12, 12, 24) | RA8P1_PINMUX_DRIVE_HIGH
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_board_ethernet_config
 ****************************************************************************/

FAR const struct ra_eth_config_s *ra_board_ethernet_config(void)
{
  unsigned int i;

  for (i = 0; i < nitems(g_ethpins); i++)
    {
      ra_pinmux_configure(g_ethpins[i]);
    }

  /* Derive a stable locally administered address from the 128-bit MCU UID. */

  memset(g_ethcfg.mac, 0, sizeof(g_ethcfg.mac));
  g_ethcfg.mac[0] = 0x02;
  for (i = 0; i < RA8P1_UID_SIZE; i++)
    {
      g_ethcfg.mac[1 + i % (IFHWADDRLEN - 1)] ^=
        getreg8(RA8P1_UID_BASE + i);
    }

  return &g_ethcfg;
}
