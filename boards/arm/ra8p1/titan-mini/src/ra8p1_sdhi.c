/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_sdhi.c
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

#include <sys/param.h>
#include <errno.h>

#include <nuttx/mmcsd.h>
#include <nuttx/sdio.h>

#include <arch/board/board.h>

#include "ra_pinmux.h"
#include "ra_sdhi.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_sdhi_pins[] =
{
  BOARD_SDHI0_DAT0_PIN,
  BOARD_SDHI0_DAT1_PIN,
  BOARD_SDHI0_DAT2_PIN,
  BOARD_SDHI0_DAT3_PIN,
  BOARD_SDHI0_CMD_PIN,
  BOARD_SDHI0_CLK_PIN,
  BOARD_SDHI0_CD_PIN
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8p1_sdhi_initialize
 ****************************************************************************/

int ra8p1_sdhi_initialize(void)
{
  FAR struct sdio_dev_s *sdio;
  unsigned int i;

  for (i = 0; i < nitems(g_sdhi_pins); i++)
    {
      ra_pinmux_configure(g_sdhi_pins[i]);
    }

  sdio = sdio_initialize(0);
  if (sdio == NULL)
    {
      return -ENODEV;
    }

  return mmcsd_slotinitialize(0, sdio);
}
