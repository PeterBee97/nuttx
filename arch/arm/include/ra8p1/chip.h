/****************************************************************************
 * arch/arm/include/ra8p1/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_RA8P1_CHIP_H
#define __ARCH_ARM_INCLUDE_RA8P1_CHIP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NVIC_SYSH_PRIORITY_MIN      0xf0
#define NVIC_SYSH_PRIORITY_DEFAULT  0x80
#define NVIC_SYSH_PRIORITY_MAX      0x00
#define NVIC_SYSH_PRIORITY_STEP     0x10

#define RA8P1_PINMUX(port, pin, psel) \
  (((port) << 24) | ((pin) << 16) | (psel))
#define RA8P1_PINMUX_DRIVE_MID         (1 << 10)
#define RA8P1_PINMUX_DRIVE_HIGH        (1 << 8)
#define RA8P1_PINMUX_DRIVE_HS_HIGH     (1 << 9)

#endif /* __ARCH_ARM_INCLUDE_RA8P1_CHIP_H */
