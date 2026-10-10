/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_adc.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_ADC_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register addresses */

#define RA8P1_ADC_ADCLKENR       (RA8P1_ADC_B0_BASE + 0x0000)
#define RA8P1_ADC_ADCLKSR        (RA8P1_ADC_B0_BASE + 0x0004)
#define RA8P1_ADC_ADCLKCR        (RA8P1_ADC_B0_BASE + 0x0008)
#define RA8P1_ADC_ADSYCR         (RA8P1_ADC_B0_BASE + 0x000c)
#define RA8P1_ADC_ADERINTCR      (RA8P1_ADC_B0_BASE + 0x0020)
#define RA8P1_ADC_ADOVFINTCR     (RA8P1_ADC_B0_BASE + 0x0024)
#define RA8P1_ADC_ADCALINTCR     (RA8P1_ADC_B0_BASE + 0x0028)
#define RA8P1_ADC_ADMDR          (RA8P1_ADC_B0_BASE + 0x0040)
#define RA8P1_ADC_ADGSPCR        (RA8P1_ADC_B0_BASE + 0x0044)
#define RA8P1_ADC_ADSGER         (RA8P1_ADC_B0_BASE + 0x0048)
#define RA8P1_ADC_ADINTCR        (RA8P1_ADC_B0_BASE + 0x005c)
#define RA8P1_ADC_ADSSTR0        (RA8P1_ADC_B0_BASE + 0x0240)
#define RA8P1_ADC_ADCNVSTR       (RA8P1_ADC_B0_BASE + 0x0260)
#define RA8P1_ADC_ADCALSTCR      (RA8P1_ADC_B0_BASE + 0x0264)
#define RA8P1_ADC_ADCHCR(n)      (RA8P1_ADC_B0_BASE + 0x0600 + \
                                  ((n) << 4))
#define RA8P1_ADC_ADDOPCRA(n)    (RA8P1_ADC_B0_BASE + 0x0604 + \
                                  ((n) << 4))
#define RA8P1_ADC_ADDOPCRB(n)    (RA8P1_ADC_B0_BASE + 0x0608 + \
                                  ((n) << 4))
#define RA8P1_ADC_ADDOPCRC(n)    (RA8P1_ADC_B0_BASE + 0x060c + \
                                  ((n) << 4))
#define RA8P1_ADC_ADCALSTR       (RA8P1_ADC_B0_BASE + 0x0c00)
#define RA8P1_ADC_ADSYSTR        (RA8P1_ADC_B0_BASE + 0x0c10)
#define RA8P1_ADC_ADSTOPR        (RA8P1_ADC_B0_BASE + 0x0c60)
#define RA8P1_ADC_ADSR           (RA8P1_ADC_B0_BASE + 0x0c80)
#define RA8P1_ADC_ADERSCR        (RA8P1_ADC_B0_BASE + 0x0c8c)
#define RA8P1_ADC_ADCALENDSR     (RA8P1_ADC_B0_BASE + 0x0c98)
#define RA8P1_ADC_ADCALENDSCR    (RA8P1_ADC_B0_BASE + 0x0c9c)
#define RA8P1_ADC_ADOVFERSCR     (RA8P1_ADC_B0_BASE + 0x0cb4)
#define RA8P1_ADC_ADSCANENDSR    (RA8P1_ADC_B0_BASE + 0x0d50)
#define RA8P1_ADC_ADSCANENDSCR   (RA8P1_ADC_B0_BASE + 0x0d54)
#define RA8P1_ADC_ADDR(n)        (RA8P1_ADC_B0_BASE + 0x2000 + \
                                  ((n) << 2))

/* Register fields */

#define RA8P1_ADC_ADCLK_ENABLE       (1u << 0)
#define RA8P1_ADC_ADSYCR_DISABLE0    (1u << 16)
#define RA8P1_ADC_ADSYCR_DISABLE1    (1u << 17)
#define RA8P1_ADC_ADGSPCR_SAR0       0x00000007u
#define RA8P1_ADC_ADGSPCR_SAR1       0x00000700u
#define RA8P1_ADC_GROUP0             (1u << 0)
#define RA8P1_ADC_ADCHCR_GROUP0      1u
#define RA8P1_ADC_ADDOPCRC_UNSIGNED  (1u << 20)
#define RA8P1_ADC_CALIBRATE_UNIT0     0x00000003u
#define RA8P1_ADC_CALIBRATE_UNIT1     0x00000300u
#define RA8P1_ADC_CALEND_UNIT0        (1u << 0)
#define RA8P1_ADC_CALEND_UNIT1        (1u << 1)
#define RA8P1_ADC_ADSTOP_UNIT0       (1u << 0)
#define RA8P1_ADC_ADDR_ERROR         (1u << 31)
#define RA8P1_ADC_ADDR_DATA_MASK     0x0000ffffu

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_ADC_H */
