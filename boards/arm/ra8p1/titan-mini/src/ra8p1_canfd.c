/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_canfd.c
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

#include <errno.h>

#include <nuttx/can/can.h>

#include <arch/board/board.h>

#include "ra_canfd.h"
#include "ra_pinmux.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ra8p1_canfd_initialize(void)
{
  FAR struct can_dev_s *dev;

  ra_pinmux_configure(BOARD_CANFD0_RX_PIN);
  ra_pinmux_configure(BOARD_CANFD0_TX_PIN);

  dev = ra_canfd_initialize(0);
  if (dev == NULL)
    {
      return -ENODEV;
    }

  return can_register("/dev/can0", dev);
}
