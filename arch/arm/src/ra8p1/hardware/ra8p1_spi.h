/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_spi.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SPI_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets */

#define RA8P1_SPI_SPDR_OFFSET       0x0000
#define RA8P1_SPI_SPDECR_OFFSET     0x0004
#define RA8P1_SPI_SPCR_OFFSET       0x0008
#define RA8P1_SPI_SPCR2_OFFSET      0x000c
#define RA8P1_SPI_SPCR3_OFFSET      0x0010
#define RA8P1_SPI_SPCMD0_OFFSET     0x0014
#define RA8P1_SPI_SPDCR_OFFSET      0x0040
#define RA8P1_SPI_SPDCR2_OFFSET     0x0044
#define RA8P1_SPI_SPSR_OFFSET       0x0050
#define RA8P1_SPI_SPTFSR_OFFSET     0x0058
#define RA8P1_SPI_SPRFSR_OFFSET     0x005c
#define RA8P1_SPI_SPPSR_OFFSET      0x0060
#define RA8P1_SPI_SPSRC_OFFSET      0x0068
#define RA8P1_SPI_SPFCR_OFFSET      0x006c

/* Control registers */

#define SPI_SPCR_ENABLE             (1u << 0)
#define SPI_SPCR_CLOCK_AUTOSTOP     (1u << 12)
#define SPI_SPCR_MASTER             (1u << 30)

#define SPI_SPCR3_SPBR_SHIFT        8
#define SPI_SPCR3_SPBR_MASK         (0xffu << SPI_SPCR3_SPBR_SHIFT)

#define SPI_SPCMD_CPHA              (1u << 0)
#define SPI_SPCMD_CPOL              (1u << 1)
#define SPI_SPCMD_BRDV_SHIFT        2
#define SPI_SPCMD_BRDV_MASK         (3u << SPI_SPCMD_BRDV_SHIFT)
#define SPI_SPCMD_LSBF              (1u << 12)
#define SPI_SPCMD_SPB_SHIFT         16
#define SPI_SPCMD_SPB_MASK          (0x1fu << SPI_SPCMD_SPB_SHIFT)

/* Status registers */

#define SPI_SPSR_RX_READY           (1u << 31)
#define SPI_SPSR_COMM_END           (1u << 30)
#define SPI_SPSR_TX_EMPTY           (1u << 29)
#define SPI_SPSR_UNDERRUN           (1u << 28)
#define SPI_SPSR_PARITY_ERROR       (1u << 27)
#define SPI_SPSR_MODE_FAULT         (1u << 26)
#define SPI_SPSR_OVERRUN            (1u << 24)
#define SPI_SPSR_DATA_READY         (1u << 23)
#define SPI_SPSR_ERROR_MASK         (SPI_SPSR_UNDERRUN | \
                                     SPI_SPSR_PARITY_ERROR | \
                                     SPI_SPSR_MODE_FAULT | \
                                     SPI_SPSR_OVERRUN)

#define SPI_SPSRC_CLEAR_ALL         0xfd800000u
#define SPI_SPSRC_CLEAR_RX          SPI_SPSR_RX_READY
#define SPI_SPSRC_CLEAR_END         SPI_SPSR_COMM_END
#define SPI_SPSRC_CLEAR_TX          SPI_SPSR_TX_EMPTY
#define SPI_SPFCR_RESET             (1u << 0)

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SPI_H */
