/****************************************************************************
 * arch/arm/src/ra8p1/ra_pinmux.c
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

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/ra8p1_pinmux.h"
#include "ra_pinmux.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_pin_configure
 *
 * Description:
 *   Atomically update one pin-function-select register.
 *
 ****************************************************************************/

void ra_pin_configure(uint8_t port, uint8_t pin, uint32_t pfs)
{
  irqstate_t flags;

  flags = up_irq_save();
  putreg8(0, RA8P1_PWPRS);
  putreg8(RA8P1_PWPRS_PFSWE, RA8P1_PWPRS);
  putreg32(pfs, RA8P1_PFS(port, pin));
  putreg8(0, RA8P1_PWPRS);
  putreg8(RA8P1_PWPRS_BOWI, RA8P1_PWPRS);
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: ra_pinmux_configure
 *
 * Description:
 *   Select a peripheral function for one RA8P1 pin.
 *
 ****************************************************************************/

void ra_pinmux_configure(uint32_t config)
{
  unsigned int port = RA8P1_PINMUX_PORT(config);
  unsigned int pin = RA8P1_PINMUX_PIN(config);
  unsigned int psel = RA8P1_PINMUX_PSEL(config);
  uint32_t pfs = RA8P1_PFS_PSEL(psel) | RA8P1_PFS_PMR;

  if ((config & RA8P1_PINMUX_DRIVE_MID) != 0)
    {
      pfs |= RA8P1_PFS_DRIVE_MID;
    }
  else if ((config & RA8P1_PINMUX_DRIVE_HIGH) != 0)
    {
      pfs |= RA8P1_PFS_DRIVE_HIGH;
    }
  else if ((config & RA8P1_PINMUX_DRIVE_HS_HIGH) != 0)
    {
      pfs |= RA8P1_PFS_DRIVE_HS_HIGH;
    }

  ra_pin_configure(port, pin, pfs);
}
