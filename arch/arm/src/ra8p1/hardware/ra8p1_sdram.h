/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_sdram.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SDRAM_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SDRAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_SDRAMC_BASE         (RA8P1_BUS_BASE + 0x0c00)

#define RA8P1_SDRAMC_SDCCR        (RA8P1_SDRAMC_BASE + 0x0000)
#define RA8P1_SDRAMC_SDCMOD       (RA8P1_SDRAMC_BASE + 0x0001)
#define RA8P1_SDRAMC_SDAMOD       (RA8P1_SDRAMC_BASE + 0x0002)
#define RA8P1_SDRAMC_SDSELF       (RA8P1_SDRAMC_BASE + 0x0010)
#define RA8P1_SDRAMC_SDRFCR       (RA8P1_SDRAMC_BASE + 0x0014)
#define RA8P1_SDRAMC_SDRFEN       (RA8P1_SDRAMC_BASE + 0x0016)
#define RA8P1_SDRAMC_SDICR        (RA8P1_SDRAMC_BASE + 0x0020)
#define RA8P1_SDRAMC_SDIR         (RA8P1_SDRAMC_BASE + 0x0024)
#define RA8P1_SDRAMC_SDADR        (RA8P1_SDRAMC_BASE + 0x0040)
#define RA8P1_SDRAMC_SDTR         (RA8P1_SDRAMC_BASE + 0x0044)
#define RA8P1_SDRAMC_SDMOD        (RA8P1_SDRAMC_BASE + 0x0048)
#define RA8P1_SDRAMC_SDSR         (RA8P1_SDRAMC_BASE + 0x0050)

#define SDRAMC_SDCCR_EXENB        (1u << 0)
#define SDRAMC_SDSR_MRSST         (1u << 0)
#define SDRAMC_SDSR_INIST         (1u << 3)

#define SDRAMC_SDIR_ARFI_SHIFT    0
#define SDRAMC_SDIR_ARFC_SHIFT    4
#define SDRAMC_SDIR_PRC_SHIFT     8

#define SDRAMC_SDTR_CL_SHIFT      0
#define SDRAMC_SDTR_WR_SHIFT      8
#define SDRAMC_SDTR_RP_SHIFT      9
#define SDRAMC_SDTR_RCD_SHIFT     12
#define SDRAMC_SDTR_RAS_SHIFT     16

#define SDRAMC_SDRFCR_RFC_SHIFT   0
#define SDRAMC_SDRFCR_REFW_SHIFT  12

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SDRAM_H */
