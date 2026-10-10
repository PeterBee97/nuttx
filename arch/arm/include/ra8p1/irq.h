/****************************************************************************
 * arch/arm/include/ra8p1/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_RA8P1_IRQ_H
#define __ARCH_ARM_INCLUDE_RA8P1_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/ra8p1/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_IRQ_RESERVED     0
#define RA_IRQ_NMI          2
#define RA_IRQ_HARDFAULT    3
#define RA_IRQ_MEMFAULT     4
#define RA_IRQ_BUSFAULT     5
#define RA_IRQ_USAGEFAULT   6
#define RA_IRQ_SVCALL       11
#define RA_IRQ_DBGMONITOR   12
#define RA_IRQ_PENDSV       14
#define RA_IRQ_SYSTICK      15
#define RA_IRQ_FIRST       16
#define RA_IRQ_NEXTINT     96
#define NR_IRQS            (RA_IRQ_FIRST + RA_IRQ_NEXTINT)

/* ICU slots are allocated here; peripheral event numbers are in ra_icu.c. */

#define SCI1_RXI           (RA_IRQ_FIRST + 0)
#define SCI1_TXI           (RA_IRQ_FIRST + 1)
#define SCI1_TEI           (RA_IRQ_FIRST + 2)
#define SCI1_ERI           (RA_IRQ_FIRST + 3)
#define RA_IRQ_GWDI0       (RA_IRQ_FIRST + 4)
#define RA_IRQ_EAEI0       (RA_IRQ_FIRST + 5)
#define RA_IRQ_EAEI1       (RA_IRQ_FIRST + 6)
#define RA_IRQ_PDM_DAT0    (RA_IRQ_FIRST + 7)
#define RA_IRQ_PDM_DAT2    (RA_IRQ_FIRST + 8)
#define RA_IRQ_VIN_STATUS  (RA_IRQ_FIRST + 9)
#define RA_IRQ_VIN_ERROR   (RA_IRQ_FIRST + 10)
#define RA_IRQ_MIPICSI_RX  (RA_IRQ_FIRST + 11)
#define RA_IRQ_MIPICSI_DL  (RA_IRQ_FIRST + 12)
#define RA_IRQ_MIPICSI_VC  (RA_IRQ_FIRST + 13)
#define RA_IRQ_MIPICSI_PM  (RA_IRQ_FIRST + 14)
#define RA_IRQ_MIPICSI_GST (RA_IRQ_FIRST + 15)
#define RA_IRQ_ICU_IRQ4     (RA_IRQ_FIRST + 16)
#define RA_IRQ_SDHI0_CARD   (RA_IRQ_FIRST + 17)
#define RA_IRQ_CAN0_TX      (RA_IRQ_FIRST + 18)
#define RA_IRQ_CAN0_CHERR   (RA_IRQ_FIRST + 19)
#define RA_IRQ_CAN_GLERR    (RA_IRQ_FIRST + 20)
#define RA_IRQ_CAN_RXF      (RA_IRQ_FIRST + 21)
#define RA_IRQ_USBFS_INT    (RA_IRQ_FIRST + 22)
#define RA_IRQ_USBFS_RESUME (RA_IRQ_FIRST + 23)
#define RA_IRQ_SSI0_TXI     (RA_IRQ_FIRST + 24)
#define RA_IRQ_SSI0_INT     (RA_IRQ_FIRST + 25)
#define SCI2_RXI            (RA_IRQ_FIRST + 26)
#define SCI2_TXI            (RA_IRQ_FIRST + 27)
#define SCI2_TEI            (RA_IRQ_FIRST + 28)
#define SCI2_ERI            (RA_IRQ_FIRST + 29)
#define RA_IRQ_NPU          (RA_IRQ_FIRST + 30)

#endif /* __ARCH_ARM_INCLUDE_RA8P1_IRQ_H */
