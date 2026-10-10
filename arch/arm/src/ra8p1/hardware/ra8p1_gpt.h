/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_gpt.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_GPT_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_GPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_GPT_GTWP_OFFSET       0x0000
#define RA8P1_GPT_GTSTR_OFFSET      0x0004
#define RA8P1_GPT_GTSTP_OFFSET      0x0008
#define RA8P1_GPT_GTCLR_OFFSET      0x000c
#define RA8P1_GPT_GTSSR_OFFSET      0x0010
#define RA8P1_GPT_GTPSR_OFFSET      0x0014
#define RA8P1_GPT_GTCSR_OFFSET      0x0018
#define RA8P1_GPT_GTCR_OFFSET       0x002c
#define RA8P1_GPT_GTUDDTYC_OFFSET   0x0030
#define RA8P1_GPT_GTIOR_OFFSET      0x0034
#define RA8P1_GPT_GTCNT_OFFSET      0x0048
#define RA8P1_GPT_GTCCRA_OFFSET     0x004c
#define RA8P1_GPT_GTCCRB_OFFSET     0x0050
#define RA8P1_GPT_GTPR_OFFSET       0x0064

#define RA8P1_GPT_REG(base, reg)     ((base) + \
                                      RA8P1_GPT_ ## reg ## _OFFSET)

#define RA8P1_GPT11_GTWP            (RA8P1_GPT11_BASE + \
                                      RA8P1_GPT_GTWP_OFFSET)
#define RA8P1_GPT11_GTSTR           (RA8P1_GPT11_BASE + \
                                      RA8P1_GPT_GTSTR_OFFSET)
#define RA8P1_GPT11_GTCLR           (RA8P1_GPT11_BASE + \
                                      RA8P1_GPT_GTCLR_OFFSET)
#define RA8P1_GPT11_GTSSR           (RA8P1_GPT11_BASE + \
                                      RA8P1_GPT_GTSSR_OFFSET)
#define RA8P1_GPT11_GTPSR           (RA8P1_GPT11_BASE + \
                                      RA8P1_GPT_GTPSR_OFFSET)
#define RA8P1_GPT11_GTCSR           (RA8P1_GPT11_BASE + \
                                      RA8P1_GPT_GTCSR_OFFSET)
#define RA8P1_GPT11_GTCR            (RA8P1_GPT11_BASE + \
                                      RA8P1_GPT_GTCR_OFFSET)
#define RA8P1_GPT11_GTUDDTYC        (RA8P1_GPT11_BASE + \
                                      RA8P1_GPT_GTUDDTYC_OFFSET)
#define RA8P1_GPT11_GTIOR           (RA8P1_GPT11_BASE + \
                                      RA8P1_GPT_GTIOR_OFFSET)
#define RA8P1_GPT11_GTCNT           (RA8P1_GPT11_BASE + \
                                      RA8P1_GPT_GTCNT_OFFSET)
#define RA8P1_GPT11_GTCCRB          (RA8P1_GPT11_BASE + \
                                      RA8P1_GPT_GTCCRB_OFFSET)
#define RA8P1_GPT11_GTPR            (RA8P1_GPT11_BASE + \
                                      RA8P1_GPT_GTPR_OFFSET)

#define GPT_GTWP_DISABLE            0xa500u
#define GPT_GTWP_ENABLE             0xa501u
#define GPT_GROUP_SOFTWARE_UPDATE   0x80000000u
#define GPT_GTCR_TPCS_SHIFT         23
#define GPT_GTCR_TPCS_MASK          (0x0fu << GPT_GTCR_TPCS_SHIFT)
#define GPT_GTIOR_GTIOA_PWM         0x09u
#define GPT_GTIOR_OAE               (1u << 8)
#define GPT_GTIOR_GTIOB_PWM         (0x09u << 16)
#define GPT_GTIOR_OBE               (1u << 24)
#define GPT_CHANNEL(n)              (1u << (n))

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_GPT_H */
