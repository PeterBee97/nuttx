/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_icu.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_ICU_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_ICU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_ICU_IELSR_BASE       (RA8P1_ICU_BASE + 0x6300)
#define RA8P1_ICU_IELSR(n)         (RA8P1_ICU_IELSR_BASE + ((n) << 2))
#define RA8P1_ICU_IRQCR(n)         (RA8P1_ICU_BASE + (n))
#define ICU_IELSR_IR               (1u << 16)

#define ICU_EVENT_IRQ4             0x0005
#define ICU_EVENT_NPU_IRQ          0x0067
#define ICU_EVENT_SDHIMMC0_CARD    0x00ad
#define ICU_EVENT_PDM_DAT0         0x00bf
#define ICU_EVENT_PDM_DAT2         0x00c1
#define ICU_EVENT_USBFS_FIFO_0     0x0098
#define ICU_EVENT_USBFS_FIFO_1     0x0099
#define ICU_EVENT_USBFS_INT        0x009a
#define ICU_EVENT_USBFS_RESUME     0x009b
#define ICU_EVENT_SSI0_TXI         0x00b3
#define ICU_EVENT_SSI0_INT         0x00b6
#define ICU_EVENT_ETHA0_EAEI       0x029d
#define ICU_EVENT_ETHA1_EAEI       0x029e
#define ICU_EVENT_GWCA0_GWDI       0x02a6
#define ICU_EVENT_CAN_RXF           0x0322
#define ICU_EVENT_CAN_GLERR         0x0323
#define ICU_EVENT_CAN0_TX           0x032c
#define ICU_EVENT_CAN0_CHERR        0x032d
#define ICU_EVENT_SCI1_RXI         0x02cb
#define ICU_EVENT_SCI1_TXI         0x02cc
#define ICU_EVENT_SCI1_TEI         0x02cd
#define ICU_EVENT_SCI1_ERI         0x02ce
#define ICU_EVENT_SCI2_RXI         0x02d2
#define ICU_EVENT_SCI2_TXI         0x02d3
#define ICU_EVENT_SCI2_TEI         0x02d4
#define ICU_EVENT_SCI2_ERI         0x02d5
#define ICU_EVENT_MIPICSI_RX       0x038f
#define ICU_EVENT_MIPICSI_DL       0x0390
#define ICU_EVENT_MIPICSI_VC       0x0391
#define ICU_EVENT_MIPICSI_PM       0x0392
#define ICU_EVENT_MIPICSI_GST      0x0393
#define ICU_EVENT_VIN_STATUS       0x0395
#define ICU_EVENT_VIN_ERROR        0x0396

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_ICU_H */
