/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_sci.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SCI_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets */

#define RA8P1_SCI_RDR_OFFSET       0x0000
#define RA8P1_SCI_TDR_OFFSET       0x0004
#define RA8P1_SCI_CCR0_OFFSET      0x0008
#define RA8P1_SCI_CCR1_OFFSET      0x000c
#define RA8P1_SCI_CCR2_OFFSET      0x0010
#define RA8P1_SCI_CCR3_OFFSET      0x0014
#define RA8P1_SCI_CSR_OFFSET       0x0048
#define RA8P1_SCI_CFCLR_OFFSET     0x0068

/* SCI1 register addresses */

#define RA8P1_SCI1_RDR             (RA8P1_SCI1_BASE + \
                                     RA8P1_SCI_RDR_OFFSET)
#define RA8P1_SCI1_TDR             (RA8P1_SCI1_BASE + \
                                     RA8P1_SCI_TDR_OFFSET)
#define RA8P1_SCI1_CCR0            (RA8P1_SCI1_BASE + \
                                     RA8P1_SCI_CCR0_OFFSET)
#define RA8P1_SCI1_CCR1            (RA8P1_SCI1_BASE + \
                                     RA8P1_SCI_CCR1_OFFSET)
#define RA8P1_SCI1_CCR2            (RA8P1_SCI1_BASE + \
                                     RA8P1_SCI_CCR2_OFFSET)
#define RA8P1_SCI1_CCR3            (RA8P1_SCI1_BASE + \
                                     RA8P1_SCI_CCR3_OFFSET)
#define RA8P1_SCI1_CSR             (RA8P1_SCI1_BASE + \
                                     RA8P1_SCI_CSR_OFFSET)
#define RA8P1_SCI1_CFCLR           (RA8P1_SCI1_BASE + \
                                     RA8P1_SCI_CFCLR_OFFSET)

/* Common control register 0 */

#define SCI_CCR0_RE                (1u << 0)
#define SCI_CCR0_TE                (1u << 4)
#define SCI_CCR0_RIE               (1u << 16)
#define SCI_CCR0_TIE               (1u << 20)

/* Common control register 1 */

#define SCI_CCR1_SPB2DT            (1u << 4)
#define SCI_CCR1_SPB2IO            (1u << 5)

/* Common control register 2 */

#define SCI_CCR2_BGDM              (1u << 4)
#define SCI_CCR2_BRR_SHIFT         8

/* Common control register 3 */

#define SCI_CCR3_CHR_SHIFT         8
#define SCI_CCR3_LSBF              (1u << 12)

/* Common status and flag clear registers */

#define SCI_CSR_TDRE               (1u << 29)
#define SCI_CSR_TEND               (1u << 30)
#define SCI_CSR_RDRF               (1u << 31)

#define SCI_CFCLR_ORERC            (1u << 24)
#define SCI_CFCLR_PERC             (1u << 27)
#define SCI_CFCLR_FERC             (1u << 28)

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SCI_H */
