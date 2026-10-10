/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_wdt.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_WDT_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_WDT_WDTRR        (RA8P1_WDT_BASE + 0x0000)
#define RA8P1_WDT_WDTCR        (RA8P1_WDT_BASE + 0x0002)
#define RA8P1_WDT_WDTSR        (RA8P1_WDT_BASE + 0x0004)
#define RA8P1_WDT_WDTRCR       (RA8P1_WDT_BASE + 0x0006)
#define RA8P1_WDT_WDTCSTPR     (RA8P1_WDT_BASE + 0x0008)

#define RA8P1_WDT_TIMEOUT_16384    3u
#define RA8P1_WDT_CLOCK_DIV_8192   (8u << 4)
#define RA8P1_WDT_WINDOW_END_0     (3u << 8)
#define RA8P1_WDT_WINDOW_START_100 (3u << 12)
#define RA8P1_WDT_RESET_ENABLE     (1u << 7)
#define RA8P1_WDT_SLEEP_STOP       (1u << 7)
#define RA8P1_WDT_COUNTER_MASK     0x3fffu

#define RA8P1_WDT_REFRESH_FIRST    0x00u
#define RA8P1_WDT_REFRESH_SECOND   0xffu

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_WDT_H */
