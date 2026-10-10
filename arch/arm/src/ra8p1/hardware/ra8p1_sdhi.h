/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_sdhi.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SDHI_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SDHI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets */

#define RA8P1_SDHI_SD_CMD_OFFSET         0x0000
#define RA8P1_SDHI_SD_ARG_OFFSET         0x0008
#define RA8P1_SDHI_SD_ARG1_OFFSET        0x000c
#define RA8P1_SDHI_SD_STOP_OFFSET        0x0010
#define RA8P1_SDHI_SD_SECCNT_OFFSET      0x0014
#define RA8P1_SDHI_SD_RSP10_OFFSET       0x0018
#define RA8P1_SDHI_SD_RSP32_OFFSET       0x0020
#define RA8P1_SDHI_SD_RSP54_OFFSET       0x0028
#define RA8P1_SDHI_SD_RSP76_OFFSET       0x0030
#define RA8P1_SDHI_SD_INFO1_OFFSET       0x0038
#define RA8P1_SDHI_SD_INFO2_OFFSET       0x003c
#define RA8P1_SDHI_SD_INFO1_MASK_OFFSET  0x0040
#define RA8P1_SDHI_SD_INFO2_MASK_OFFSET  0x0044
#define RA8P1_SDHI_SD_CLK_CTRL_OFFSET    0x0048
#define RA8P1_SDHI_SD_SIZE_OFFSET        0x004c
#define RA8P1_SDHI_SD_OPTION_OFFSET      0x0050
#define RA8P1_SDHI_SD_BUF0_OFFSET        0x0060
#define RA8P1_SDHI_SDIO_MODE_OFFSET      0x0068
#define RA8P1_SDHI_SDIO_INFO1_OFFSET     0x006c
#define RA8P1_SDHI_SDIO_MASK_OFFSET      0x0070
#define RA8P1_SDHI_SD_DMAEN_OFFSET       0x01b0
#define RA8P1_SDHI_SOFT_RST_OFFSET       0x01c0
#define RA8P1_SDHI_SDIF_MODE_OFFSET      0x01cc
#define RA8P1_SDHI_EXT_SWAP_OFFSET       0x01e0

/* Register addresses */

#define RA8P1_SDHI0_SD_CMD        (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_CMD_OFFSET)
#define RA8P1_SDHI0_SD_ARG        (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_ARG_OFFSET)
#define RA8P1_SDHI0_SD_ARG1       (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_ARG1_OFFSET)
#define RA8P1_SDHI0_SD_STOP       (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_STOP_OFFSET)
#define RA8P1_SDHI0_SD_SECCNT     (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_SECCNT_OFFSET)
#define RA8P1_SDHI0_SD_RSP10      (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_RSP10_OFFSET)
#define RA8P1_SDHI0_SD_RSP32      (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_RSP32_OFFSET)
#define RA8P1_SDHI0_SD_RSP54      (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_RSP54_OFFSET)
#define RA8P1_SDHI0_SD_RSP76      (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_RSP76_OFFSET)
#define RA8P1_SDHI0_SD_INFO1      (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_INFO1_OFFSET)
#define RA8P1_SDHI0_SD_INFO2      (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_INFO2_OFFSET)
#define RA8P1_SDHI0_SD_INFO1_MASK (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_INFO1_MASK_OFFSET)
#define RA8P1_SDHI0_SD_INFO2_MASK (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_INFO2_MASK_OFFSET)
#define RA8P1_SDHI0_SD_CLK_CTRL   (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_CLK_CTRL_OFFSET)
#define RA8P1_SDHI0_SD_SIZE       (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_SIZE_OFFSET)
#define RA8P1_SDHI0_SD_OPTION     (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_OPTION_OFFSET)
#define RA8P1_SDHI0_SD_BUF0       (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_BUF0_OFFSET)
#define RA8P1_SDHI0_SDIO_MODE     (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SDIO_MODE_OFFSET)
#define RA8P1_SDHI0_SDIO_INFO1    (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SDIO_INFO1_OFFSET)
#define RA8P1_SDHI0_SDIO_MASK     (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SDIO_MASK_OFFSET)
#define RA8P1_SDHI0_SD_DMAEN      (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SD_DMAEN_OFFSET)
#define RA8P1_SDHI0_SOFT_RST      (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SOFT_RST_OFFSET)
#define RA8P1_SDHI0_SDIF_MODE     (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_SDIF_MODE_OFFSET)
#define RA8P1_SDHI0_EXT_SWAP      (RA8P1_SDHI0_BASE + \
                                    RA8P1_SDHI_EXT_SWAP_OFFSET)

/* Command register */

#define SDHI_CMD_INDEX_MASK            0x3f
#define SDHI_CMD_ACMD                  (1u << 6)
#define SDHI_CMD_RESPONSE_NONE         (0u << 8)
#define SDHI_CMD_RESPONSE_R1           (4u << 8)
#define SDHI_CMD_RESPONSE_R1B          (5u << 8)
#define SDHI_CMD_RESPONSE_R2           (6u << 8)
#define SDHI_CMD_RESPONSE_R3           (7u << 8)
#define SDHI_CMD_DATA                  (1u << 11)
#define SDHI_CMD_DATA_READ             (1u << 12)
#define SDHI_CMD_MULTI                 (1u << 13)
#define SDHI_CMD_MULTI_NO_AUTOSTOP     (1u << 14)

/* Status and mask registers */

#define SDHI_INFO1_RESPONSE_END        (1u << 0)
#define SDHI_INFO1_ACCESS_END          (1u << 2)
#define SDHI_INFO1_CARD_REMOVED        (1u << 3)
#define SDHI_INFO1_CARD_INSERTED       (1u << 4)
#define SDHI_INFO1_CARD_PRESENT        (1u << 5)
#define SDHI_INFO1_CARD_EVENTS         (SDHI_INFO1_CARD_REMOVED | \
                                        SDHI_INFO1_CARD_INSERTED)
#define SDHI_INFO1_MASK_CARD_DETECT    0x00000305u
#define SDHI_INFO1_MASK_ALL            0x0000031du

#define SDHI_INFO2_CMD_ERROR           (1u << 0)
#define SDHI_INFO2_CRC_ERROR           (1u << 1)
#define SDHI_INFO2_END_ERROR           (1u << 2)
#define SDHI_INFO2_DATA_TIMEOUT        (1u << 3)
#define SDHI_INFO2_ILLEGAL_WRITE       (1u << 4)
#define SDHI_INFO2_ILLEGAL_READ        (1u << 5)
#define SDHI_INFO2_RESPONSE_TIMEOUT    (1u << 6)
#define SDHI_INFO2_DAT0                (1u << 7)
#define SDHI_INFO2_BUFFER_READ_ENABLE  (1u << 8)
#define SDHI_INFO2_BUFFER_WRITE_ENABLE (1u << 9)
#define SDHI_INFO2_CLOCK_READY         (1u << 13)
#define SDHI_INFO2_CMD_BUSY            (1u << 14)
#define SDHI_INFO2_ILLEGAL_ACCESS      (1u << 15)
#define SDHI_INFO2_ERROR_MASK          (SDHI_INFO2_CMD_ERROR | \
                                        SDHI_INFO2_CRC_ERROR | \
                                        SDHI_INFO2_END_ERROR | \
                                        SDHI_INFO2_DATA_TIMEOUT | \
                                        SDHI_INFO2_ILLEGAL_WRITE | \
                                        SDHI_INFO2_ILLEGAL_READ | \
                                        SDHI_INFO2_RESPONSE_TIMEOUT | \
                                        SDHI_INFO2_ILLEGAL_ACCESS)
#define SDHI_INFO2_IDLE_MASK           (SDHI_INFO2_CMD_BUSY | \
                                        SDHI_INFO2_DAT0)
#define SDHI_INFO2_IDLE                SDHI_INFO2_DAT0
#define SDHI_INFO2_MASK_ALL            0x0000837fu

/* Transfer and clock control */

#define SDHI_STOP_BLOCK_COUNT          (1u << 8)
#define SDHI_CLOCK_ENABLE              (1u << 8)
#define SDHI_CLOCK_AUTO_ENABLE         (1u << 9)
#define SDHI_CLOCK_CONTROL_MASK        (SDHI_CLOCK_ENABLE | \
                                        SDHI_CLOCK_AUTO_ENABLE)
#define SDHI_CLOCK_DIV_BYPASS          0xffu
#define SDHI_OPTION_DEFAULT            0x40e0u
#define SDHI_OPTION_WIDTH_1BIT         (4u << 13)
#define SDHI_OPTION_WIDTH_4BIT         (0u << 13)

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SDHI_H */
