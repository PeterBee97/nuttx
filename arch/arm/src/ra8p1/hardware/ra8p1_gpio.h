/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_gpio.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_GPIO_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_PORT_STRIDE         0x20
#define RA8P1_PORT_POSR_OFFSET    0x08
#define RA8P1_PORT_PORR_OFFSET    0x0a

#define RA8P1_PORT_POSR(port)     (RA8P1_PORT_BASE + \
                                   (port) * RA8P1_PORT_STRIDE + \
                                   RA8P1_PORT_POSR_OFFSET)
#define RA8P1_PORT_PORR(port)     (RA8P1_PORT_BASE + \
                                   (port) * RA8P1_PORT_STRIDE + \
                                   RA8P1_PORT_PORR_OFFSET)

#define RA8P1_PFS_PODR            (1u << 0)
#define RA8P1_PFS_PIDR            (1u << 1)
#define RA8P1_PFS_PDR             (1u << 2)
#define RA8P1_PFS_PCR             (1u << 4)
#define RA8P1_PFS_ISEL            (1u << 14)

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_GPIO_H */
