/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_pdm.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_PDM_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_PDM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Common registers */

#define RA8P1_PDM_PDCSTRTR         (RA8P1_PDM_BASE + 0x0000)
#define RA8P1_PDM_PDCSTPTR         (RA8P1_PDM_BASE + 0x0004)
#define RA8P1_PDM_PDCSR            (RA8P1_PDM_BASE + 0x0010)

/* Channel registers */

#define RA8P1_PDM_CH_BASE(n)       (RA8P1_PDM_BASE + 0x0100 + \
                                    ((n) << 8))
#define RA8P1_PDM_PDICR(n)         (RA8P1_PDM_CH_BASE(n) + 0x000c)
#define RA8P1_PDM_PDSDCR(n)        (RA8P1_PDM_CH_BASE(n) + 0x0010)
#define RA8P1_PDM_PDSR(n)          (RA8P1_PDM_CH_BASE(n) + 0x0014)
#define RA8P1_PDM_PDSCR(n)         (RA8P1_PDM_CH_BASE(n) + 0x0018)
#define RA8P1_PDM_PDMDSR(n)        (RA8P1_PDM_CH_BASE(n) + 0x0020)
#define RA8P1_PDM_PDSFCR(n)        (RA8P1_PDM_CH_BASE(n) + 0x0024)
#define RA8P1_PDM_PDHFCS0R(n)      (RA8P1_PDM_CH_BASE(n) + 0x0028)
#define RA8P1_PDM_PDHFCK1R(n)      (RA8P1_PDM_CH_BASE(n) + 0x002c)
#define RA8P1_PDM_PDHFCHR(n, i)    (RA8P1_PDM_CH_BASE(n) + 0x0030 + \
                                    ((i) << 2))
#define RA8P1_PDM_PDCFCHR(n, i)    (RA8P1_PDM_CH_BASE(n) + 0x0038 + \
                                    ((i) << 2))
#define RA8P1_PDM_PDLFCH010R(n)    (RA8P1_PDM_CH_BASE(n) + 0x0064)
#define RA8P1_PDM_PDLFCH1R(n, i)   (RA8P1_PDM_CH_BASE(n) + 0x0068 + \
                                    ((i) << 2))
#define RA8P1_PDM_PDDBCR(n)        (RA8P1_PDM_CH_BASE(n) + 0x00c0)
#define RA8P1_PDM_PDSCTSR(n)       (RA8P1_PDM_CH_BASE(n) + 0x00c4)
#define RA8P1_PDM_PDOVLTR(n)       (RA8P1_PDM_CH_BASE(n) + 0x00c8)
#define RA8P1_PDM_PDOVUTR(n)       (RA8P1_PDM_CH_BASE(n) + 0x00cc)
#define RA8P1_PDM_PDDRCR(n)        (RA8P1_PDM_CH_BASE(n) + 0x00e0)
#define RA8P1_PDM_PDDCR(n)         (RA8P1_PDM_CH_BASE(n) + 0x00e4)
#define RA8P1_PDM_PDDRR(n)         (RA8P1_PDM_CH_BASE(n) + 0x00e8)
#define RA8P1_PDM_PDDSR(n)         (RA8P1_PDM_CH_BASE(n) + 0x00ec)

/* Common register fields */

#define PDM_CHANNEL(n)             (1u << (n))

/* Channel register fields */

#define PDM_PDICR_IDRE             (1u << 2)

#define PDM_PDSDCR_SCDE            (1u << 16)
#define PDM_PDSDCR_OVLDE           (1u << 17)
#define PDM_PDSDCR_OVUDE           (1u << 18)
#define PDM_PDSDCR_BFOWDE          (1u << 27)

#define PDM_PDSR_STATE             (1u << 0)
#define PDM_PDSR_SCDF              (1u << 16)
#define PDM_PDSR_OVLDF             (1u << 17)
#define PDM_PDSR_OVUDF             (1u << 18)
#define PDM_PDSR_BFOWDF            (1u << 27)
#define PDM_PDSR_ERROR_MASK        (PDM_PDSR_SCDF | PDM_PDSR_OVLDF | \
                                    PDM_PDSR_OVUDF | PDM_PDSR_BFOWDF)

#define PDM_PDSCR_CLEAR_MASK       ((1u << 1) | (1u << 16) | \
                                    (1u << 17) | (1u << 18) | \
                                    (1u << 27))

#define PDM_PDMDSR_INPUT_FALL      (1u << 0)
#define PDM_PDMDSR_SINC_ORDER_4    (4u << 4)
#define PDM_PDMDSR_PCM_16_0_14     (12u << 28)

#define PDM_PDSFCR_SINCDEC(n)      ((uint32_t)(n) << 16)
#define PDM_PDSFCR_SINCRNG(n)      ((uint32_t)(n) << 24)

#define PDM_PDDBCR_THRESHOLD_16    4u
#define PDM_PDDRCR_DATRE           (1u << 0)
#define PDM_PDDCR_DATC             (1u << 0)
#define PDM_PDDSR_DATNUM_MASK      0xffu

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_PDM_H */
