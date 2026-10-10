/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_rtc.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_RTC_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register addresses */

#define RA8P1_RTC_R64CNT       (RA8P1_RTC_BASE + 0x0000)
#define RA8P1_RTC_RSECCNT      (RA8P1_RTC_BASE + 0x0002)
#define RA8P1_RTC_RMINCNT      (RA8P1_RTC_BASE + 0x0004)
#define RA8P1_RTC_RHRCNT       (RA8P1_RTC_BASE + 0x0006)
#define RA8P1_RTC_RWKCNT       (RA8P1_RTC_BASE + 0x0008)
#define RA8P1_RTC_RDAYCNT      (RA8P1_RTC_BASE + 0x000a)
#define RA8P1_RTC_RMONCNT      (RA8P1_RTC_BASE + 0x000c)
#define RA8P1_RTC_RYRCNT       (RA8P1_RTC_BASE + 0x000e)
#define RA8P1_RTC_RCR1         (RA8P1_RTC_BASE + 0x0022)
#define RA8P1_RTC_RCR2         (RA8P1_RTC_BASE + 0x0024)
#define RA8P1_RTC_RCR4         (RA8P1_RTC_BASE + 0x0028)
#define RA8P1_RTC_RFRH         (RA8P1_RTC_BASE + 0x002a)
#define RA8P1_RTC_RFRL         (RA8P1_RTC_BASE + 0x002c)

/* RTC control register 2 */

#define RA8P1_RTC_RCR2_START         (1u << 0)
#define RA8P1_RTC_RCR2_RESET         (1u << 1)
#define RA8P1_RTC_RCR2_HR24          (1u << 6)
#define RA8P1_RTC_RCR2_CNTMD         (1u << 7)

/* RTC control register 4 */

#define RA8P1_RTC_RCR4_RCKSEL_LOCO   (1u << 0)

/* The SDK configures the nominal 32.768 kHz LOCO with this comparison
 * value.
 */

#define RA8P1_RTC_RFRL_LOCO          255u

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_RTC_H */
