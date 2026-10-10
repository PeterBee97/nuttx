/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_i2c.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_I2C_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets */

#define RA8P1_IIC_ICCR1_OFFSET    0x0000
#define RA8P1_IIC_ICCR2_OFFSET    0x0001
#define RA8P1_IIC_ICMR1_OFFSET    0x0002
#define RA8P1_IIC_ICMR2_OFFSET    0x0003
#define RA8P1_IIC_ICMR3_OFFSET    0x0004
#define RA8P1_IIC_ICFER_OFFSET    0x0005
#define RA8P1_IIC_ICSER_OFFSET    0x0006
#define RA8P1_IIC_ICIER_OFFSET    0x0007
#define RA8P1_IIC_ICSR1_OFFSET    0x0008
#define RA8P1_IIC_ICSR2_OFFSET    0x0009
#define RA8P1_IIC_ICBRL_OFFSET    0x0010
#define RA8P1_IIC_ICBRH_OFFSET    0x0011
#define RA8P1_IIC_ICDRT_OFFSET    0x0012
#define RA8P1_IIC_ICDRR_OFFSET    0x0013

/* IIC0 registers */

#define RA8P1_IIC0_ICCR1          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICCR1_OFFSET)
#define RA8P1_IIC0_ICCR2          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICCR2_OFFSET)
#define RA8P1_IIC0_ICMR1          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICMR1_OFFSET)
#define RA8P1_IIC0_ICMR2          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICMR2_OFFSET)
#define RA8P1_IIC0_ICMR3          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICMR3_OFFSET)
#define RA8P1_IIC0_ICFER          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICFER_OFFSET)
#define RA8P1_IIC0_ICSER          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICSER_OFFSET)
#define RA8P1_IIC0_ICIER          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICIER_OFFSET)
#define RA8P1_IIC0_ICSR1          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICSR1_OFFSET)
#define RA8P1_IIC0_ICSR2          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICSR2_OFFSET)
#define RA8P1_IIC0_ICBRL          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICBRL_OFFSET)
#define RA8P1_IIC0_ICBRH          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICBRH_OFFSET)
#define RA8P1_IIC0_ICDRT          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICDRT_OFFSET)
#define RA8P1_IIC0_ICDRR          (RA8P1_IIC0_BASE + \
                                    RA8P1_IIC_ICDRR_OFFSET)

/* I2C Bus Control Register 1 */

#define IIC_ICCR1_ICE             (1u << 7)
#define IIC_ICCR1_IICRST          (1u << 6)
#define IIC_ICCR1_OUTPUT_HIGH     0x1fu

/* I2C Bus Control Register 2 */

#define IIC_ICCR2_BBSY            (1u << 7)
#define IIC_ICCR2_MST             (1u << 6)
#define IIC_ICCR2_TRS             (1u << 5)
#define IIC_ICCR2_SP              (1u << 3)
#define IIC_ICCR2_RS              (1u << 2)
#define IIC_ICCR2_ST              (1u << 1)

/* I2C Bus Mode Registers */

#define IIC_ICMR1_CKS_SHIFT       4
#define IIC_ICMR1_BCWP            (1u << 3)
#define IIC_ICMR2_TMOH            (1u << 2)
#define IIC_ICMR2_TMOL            (1u << 1)
#define IIC_ICMR2_TMOS            (1u << 0)
#define IIC_ICMR3_WAIT            (1u << 6)
#define IIC_ICMR3_ACKWP           (1u << 4)
#define IIC_ICMR3_ACKBT           (1u << 3)

/* I2C Bus Function Enable Register */

#define IIC_ICFER_SCLE            (1u << 6)
#define IIC_ICFER_NFE             (1u << 5)
#define IIC_ICFER_NACKE           (1u << 4)
#define IIC_ICFER_NALE            (1u << 2)
#define IIC_ICFER_MALE            (1u << 1)
#define IIC_ICFER_TMOE            (1u << 0)
#define IIC_ICFER_MASTER_INIT     0x77u

/* I2C Bus Status Register 2 */

#define IIC_ICSR2_TDRE            (1u << 7)
#define IIC_ICSR2_TEND            (1u << 6)
#define IIC_ICSR2_RDRF            (1u << 5)
#define IIC_ICSR2_NACKF           (1u << 4)
#define IIC_ICSR2_STOP            (1u << 3)
#define IIC_ICSR2_START           (1u << 2)
#define IIC_ICSR2_AL              (1u << 1)
#define IIC_ICSR2_TMOF            (1u << 0)
#define IIC_ICSR2_ERRORS          (IIC_ICSR2_NACKF | IIC_ICSR2_AL | \
                                   IIC_ICSR2_TMOF)
#define IIC_ICSR2_EVENTS          (IIC_ICSR2_NACKF | IIC_ICSR2_STOP | \
                                   IIC_ICSR2_START | IIC_ICSR2_AL | \
                                   IIC_ICSR2_TMOF)

/* Bit-rate registers */

#define IIC_ICBR_RESERVED         0xe0u

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_I2C_H */
