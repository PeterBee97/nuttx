/****************************************************************************
 * arch/arm/src/ra8p1/ra_sdram.c
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
#include <stdbool.h>
#include <stdint.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/ra8p1_sdram.h"
#include "hardware/ra8p1_system.h"
#include "ra_sdram.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SDRAM_POLL_LIMIT           10000000
#define SDRAM_MODE_WRITE_SINGLE    (1u << 9)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_sdram_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ra_sdram_wait(uint8_t mask)
{
  unsigned int timeout;

  for (timeout = SDRAM_POLL_LIMIT; timeout > 0; timeout--)
    {
      if ((getreg8(RA8P1_SDRAMC_SDSR) & mask) == 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_sdram_initialize
 *
 * Description:
 *   Initialize the RA8P1 SDRAM controller and the attached SDR SDRAM.
 *
 ****************************************************************************/

int ra_sdram_initialize(void)
{
  uint32_t timing;
  uint16_t refresh;
  uint16_t init;
  int ret;

  if (g_sdram_initialized)
    {
      return OK;
    }

  ret = ra_sdram_wait(0xff);
  if (ret < 0)
    {
      return ret;
    }

  init = ((BOARD_SDRAM_INIT_ARFI - 3) << SDRAMC_SDIR_ARFI_SHIFT) |
         (BOARD_SDRAM_INIT_ARFC << SDRAMC_SDIR_ARFC_SHIFT) |
         ((BOARD_SDRAM_INIT_PRC - 3) << SDRAMC_SDIR_PRC_SHIFT);
  putreg16(init, RA8P1_SDRAMC_SDIR);
  putreg8(BOARD_SDRAM_BUS_WIDTH << 4, RA8P1_SDRAMC_SDCCR);

  putreg16(SYSTEM_PRCR_KEY | SYSTEM_PRCR_PRC0, RA8P1_SYSTEM_PRCR);
  putreg8(1, RA8P1_SYSTEM_SDCKOCR);
  putreg16(SYSTEM_PRCR_KEY, RA8P1_SYSTEM_PRCR);

  putreg8(1, RA8P1_SDRAMC_SDICR);
  ret = ra_sdram_wait(SDRAMC_SDSR_INIST);
  if (ret < 0)
    {
      return ret;
    }

  putreg8(BOARD_SDRAM_ACCESS_MODE, RA8P1_SDRAMC_SDAMOD);
  putreg8(BOARD_SDRAM_ENDIAN_MODE, RA8P1_SDRAMC_SDCMOD);
  ret = ra_sdram_wait(0xff);
  if (ret < 0)
    {
      return ret;
    }

  putreg16(SDRAM_MODE_WRITE_SINGLE | (BOARD_SDRAM_TCL << 4),
           RA8P1_SDRAMC_SDMOD);
  ret = ra_sdram_wait(SDRAMC_SDSR_MRSST);
  if (ret < 0)
    {
      return ret;
    }

  timing = ((BOARD_SDRAM_TRAS - 1) << SDRAMC_SDTR_RAS_SHIFT) |
           ((BOARD_SDRAM_TRCD - 1) << SDRAMC_SDTR_RCD_SHIFT) |
           ((BOARD_SDRAM_TRP - 1) << SDRAMC_SDTR_RP_SHIFT) |
           ((BOARD_SDRAM_TWR - 1) << SDRAMC_SDTR_WR_SHIFT) |
           (BOARD_SDRAM_TCL << SDRAMC_SDTR_CL_SHIFT);
  putreg32(timing, RA8P1_SDRAMC_SDTR);
  putreg8(BOARD_SDRAM_ADDR_SHIFT, RA8P1_SDRAMC_SDADR);

  refresh = ((BOARD_SDRAM_TREFW - 1) << SDRAMC_SDRFCR_REFW_SHIFT) |
            ((BOARD_SDRAM_TRFC - 1) << SDRAMC_SDRFCR_RFC_SHIFT);
  putreg16(refresh, RA8P1_SDRAMC_SDRFCR);
  putreg8(1, RA8P1_SDRAMC_SDRFEN);
  putreg8(SDRAMC_SDCCR_EXENB | (BOARD_SDRAM_BUS_WIDTH << 4),
          RA8P1_SDRAMC_SDCCR);
  UP_DSB();

  g_sdram_initialized = true;
  return OK;
}
