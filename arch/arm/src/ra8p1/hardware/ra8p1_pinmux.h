/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_pinmux.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_PINMUX_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_PINMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_PWPRS               (RA8P1_PMISC_BASE + 0x0014)
#define RA8P1_PFS(port, pin)       (RA8P1_PFS_BASE + ((port) << 6) + \
                                    ((pin) << 2))

#define RA8P1_PWPRS_BOWI          (1u << 7)
#define RA8P1_PWPRS_PFSWE         (1u << 6)

#define RA8P1_PFS_PSEL(n)         ((n) << 24)
#define RA8P1_PFS_PMR             (1u << 16)
#define RA8P1_PFS_ANALOG          (1u << 15)
#define RA8P1_PFS_DRIVE_MID       0x00000400u
#define RA8P1_PFS_DRIVE_HIGH      0x00000c00u
#define RA8P1_PFS_DRIVE_HS_HIGH   0x00000800u

#define RA8P1_PINMUX_PORT(cfg)     (((cfg) >> 24) & 0xff)
#define RA8P1_PINMUX_PIN(cfg)      (((cfg) >> 16) & 0xff)
#define RA8P1_PINMUX_PSEL(cfg)     ((cfg) & 0xff)

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_PINMUX_H */
