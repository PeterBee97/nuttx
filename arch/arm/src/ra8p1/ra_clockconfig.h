/****************************************************************************
 * arch/arm/src/ra8p1/ra_clockconfig.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_RA_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_RA8P1_RA_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_CPUCLK_FREQUENCY BOARD_CPU_FREQUENCY
#define RA_SCICLK_FREQUENCY BOARD_SCI_FREQUENCY

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void ra_clockconfig(void);
void ra_lowsetup(void);
void ra_module_control(uintptr_t reg, uint32_t mask, bool enable);

#endif /* __ARCH_ARM_SRC_RA8P1_RA_CLOCKCONFIG_H */
