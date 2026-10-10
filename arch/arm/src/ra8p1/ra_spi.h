/****************************************************************************
 * arch/arm/src/ra8p1/ra_spi.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_RA_SPI_H
#define __ARCH_ARM_SRC_RA8P1_RA_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

struct spi_dev_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct spi_dev_s *ra_spibus_initialize(int bus);

#ifdef CONFIG_RA8P1_SPI1
void ra_spi1select(struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t ra_spi1status(struct spi_dev_s *dev, uint32_t devid);
#endif

#endif /* __ARCH_ARM_SRC_RA8P1_RA_SPI_H */
