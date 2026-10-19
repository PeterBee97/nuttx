/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_system.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SYSTEM_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SYSTEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System registers */

#define RA8P1_SYSTEM_VSCR         (RA8P1_SYSTEM_BASE + 0x0014)
#define RA8P1_SYSTEM_SCKDIVCR     (RA8P1_SYSTEM_BASE + 0x0020)
#define RA8P1_SYSTEM_SCKDIVCR2    (RA8P1_SYSTEM_BASE + 0x0024)
#define RA8P1_SYSTEM_SCKSCR       (RA8P1_SYSTEM_BASE + 0x0026)
#define RA8P1_SYSTEM_PLLCR        (RA8P1_SYSTEM_BASE + 0x002a)
#define RA8P1_SYSTEM_MOSCCR       (RA8P1_SYSTEM_BASE + 0x0032)
#define RA8P1_SYSTEM_OSCSF        (RA8P1_SYSTEM_BASE + 0x003c)
#define RA8P1_SYSTEM_PLL2CR       (RA8P1_SYSTEM_BASE + 0x004a)
#define RA8P1_SYSTEM_PLLCCR2      (RA8P1_SYSTEM_BASE + 0x004c)
#define RA8P1_SYSTEM_PLL2CCR2     (RA8P1_SYSTEM_BASE + 0x004e)
#define RA8P1_SYSTEM_SDCKOCR      (RA8P1_SYSTEM_BASE + 0x0053)
#define RA8P1_SYSTEM_SCICKDIVCR   (RA8P1_SYSTEM_BASE + 0x0054)
#define RA8P1_SYSTEM_SCICKCR      (RA8P1_SYSTEM_BASE + 0x0055)
#define RA8P1_SYSTEM_ADCCKDIVCR   (RA8P1_SYSTEM_BASE + 0x005a)
#define RA8P1_SYSTEM_ADCCKCR      (RA8P1_SYSTEM_BASE + 0x005b)
#define RA8P1_SYSTEM_GPTCKDIVCR   (RA8P1_SYSTEM_BASE + 0x005c)
#define RA8P1_SYSTEM_GPTCKCR      (RA8P1_SYSTEM_BASE + 0x005d)
#define RA8P1_SYSTEM_LCDCKDIVCR   (RA8P1_SYSTEM_BASE + 0x005e)
#define RA8P1_SYSTEM_LCDCKCR      (RA8P1_SYSTEM_BASE + 0x005f)
#define RA8P1_SYSTEM_USBCKDIVCR   (RA8P1_SYSTEM_BASE + 0x006c)
#define RA8P1_SYSTEM_OCTACKDIVCR  (RA8P1_SYSTEM_BASE + 0x006d)
#define RA8P1_SYSTEM_CANFDCKDIVCR (RA8P1_SYSTEM_BASE + 0x006e)
#define RA8P1_SYSTEM_USBCKCR      (RA8P1_SYSTEM_BASE + 0x0074)
#define RA8P1_SYSTEM_OCTACKCR     (RA8P1_SYSTEM_BASE + 0x0075)
#define RA8P1_SYSTEM_CANFDCKCR    (RA8P1_SYSTEM_BASE + 0x0076)
#define RA8P1_SYSTEM_MOSCWTCR     (RA8P1_SYSTEM_BASE + 0x00a2)
#define RA8P1_SYSTEM_PLLCCR       (RA8P1_SYSTEM_BASE + 0x00ac)
#define RA8P1_SYSTEM_PLL2CCR      (RA8P1_SYSTEM_BASE + 0x00c8)
#define RA8P1_SYSTEM_ESWCKDIVCR   (RA8P1_SYSTEM_BASE + 0x00d5)
#define RA8P1_SYSTEM_ESWPCKDIVCR  (RA8P1_SYSTEM_BASE + 0x00d6)
#define RA8P1_SYSTEM_ETHPCKDIVCR  (RA8P1_SYSTEM_BASE + 0x00d8)
#define RA8P1_SYSTEM_ESWCKCR      (RA8P1_SYSTEM_BASE + 0x00db)
#define RA8P1_SYSTEM_ESWPCKCR     (RA8P1_SYSTEM_BASE + 0x00dc)
#define RA8P1_SYSTEM_ETHPCKCR     (RA8P1_SYSTEM_BASE + 0x00de)
#define RA8P1_SYSTEM_PRCR         (RA8P1_SYSTEM_BASE + 0x03fa)
#define RA8P1_SYSTEM_MOMCR        (RA8P1_SYSTEM_BASE + 0x0a50)
#define RA8P1_SYSTEM_PDCTRGD      (RA8P1_SYSTEM_BASE + 0x0110)
#define RA8P1_SYSTEM_PDCTRNPU     (RA8P1_SYSTEM_BASE + 0x0114)
#define RA8P1_SYSTEM_PDCTRESWM    (RA8P1_SYSTEM_BASE + 0x0118)
#define RA8P1_SYSTEM_PGCSAR       (RA8P1_SYSTEM_BASE + 0x03d8)

/* MRAM registers */

#define RA8P1_SRAM_PRCR           0x40002000u
#define RA8P1_SRAM_WTSC           0x40002008u
#define SRAM_PRCR_KEY            0xa500u
#define SRAM_PRCR_WRITE          0x0001u
#define SRAM_WTSC_WAIT           0x01u

#define RA8P1_MRAM_PREFETCH       (RA8P1_MRAM_BASE + 0x0000)
#define RA8P1_MRAM_CODE_FREQ      (RA8P1_MRAM_BASE + 0x0004)
#define RA8P1_MRAM_EXTRA_FREQ     (RA8P1_MRAM_BASE + 0x0008)

/* Module-stop registers */

#define RA8P1_MSTPCRA             (RA8P1_MSTP_BASE + 0x0000)
#define RA8P1_MSTPCRB             (RA8P1_MSTP_BASE + 0x0004)
#define RA8P1_MSTPCRC             (RA8P1_MSTP_BASE + 0x0008)
#define RA8P1_MSTPCRD             (RA8P1_MSTP_BASE + 0x000c)
#define RA8P1_MSTPCRE             (RA8P1_MSTP_BASE + 0x0010)
#define RA8P1_PSCU_PSARB          (RA8P1_PSCU_BASE + 0x0004)
#define RA8P1_PSCU_PSARC          (RA8P1_PSCU_BASE + 0x0008)
#define RA8P1_PSCU_PSARD          (RA8P1_PSCU_BASE + 0x000c)
#define RA8P1_PSCU_PSARE          (RA8P1_PSCU_BASE + 0x0010)
#define RA8P1_CPUCFG              0xe001e000u

/* System register bits */

#define SYSTEM_VSCR_VSCM_MASK     0x07u
#define SYSTEM_VSCR_VSCM_HIGH     0x01u
#define SYSTEM_VSCR_TRANSITION    (1u << 4)

#define SYSTEM_OSCSF_MOSC         (1u << 3)
#define SYSTEM_OSCSF_PLL1         (1u << 5)
#define SYSTEM_OSCSF_PLL2         (1u << 6)

#define SYSTEM_PRCR_KEY           0xa500u
#define SYSTEM_PRCR_PRC0          (1u << 0)
#define SYSTEM_PRCR_PRC1          (1u << 1)
#define SYSTEM_PRCR_PRC3          (1u << 3)
#define SYSTEM_PRCR_PRC4          (1u << 4)

#define SYSTEM_PDCTRGD_PDCSF      (1u << 6)
#define SYSTEM_PDCTRGD_PDPGSF     (1u << 7)
#define SYSTEM_PDCTRNPU_STATUS    ((1u << 6) | (1u << 7))
#define SYSTEM_PDCTRNPU_DOWN      (1u << 7)
#define SYSTEM_PDCTRESWM_STATUS   ((1u << 6) | (1u << 7))
#define SYSTEM_PDCTRESWM_DOWN     (1u << 7)

#define MSTPCRA_NPU               (1u << 16)
#define MSTPCRB_USBFS0            (1u << 11)
#define MSTPCRB_IIC0              (1u << 9)
#define MSTPCRB_IIC1              (1u << 8)
#define MSTPCRB_OSPI0             (1u << 16)
#define MSTPCRB_SPI1              (1u << 18)
#define MSTPCRB_SCI1              (1u << 30)
#define MSTPCRB_SCI2              (1u << 29)
#define MSTPCRC_SSI0              (1u << 8)
#define MSTPCRC_SDHI0             (1u << 12)
#define MSTPCRC_MIPI_CSI          (1u << 17)
#define MSTPCRC_PDM               (1u << 24)
#define MSTPCRC_CANFD0            (1u << 27)
#define MSTPCRC_ETHPHY            (1u << 28)
#define MSTPCRC_ESWM              (1u << 30)
#define MSTPCRD_ADC               (1u << 21)
#define MSTPCRE_GPT2              (1u << 29)
#define MSTPCRE_GPT11             (1u << 20)
#define CPUCFG_TCM_MASK           0x00003000u

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SYSTEM_H */
