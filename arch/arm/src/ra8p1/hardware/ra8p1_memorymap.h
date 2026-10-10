/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_memorymap.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_MEMORYMAP_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_UID_BASE         0x02f07b00u
#define RA8P1_UID_SIZE         16
#define RA8P1_BUS_BASE         0x40003000u
#define RA8P1_ICU_BASE         0x40006000u
#define RA8P1_SYSTEM_BASE      0x4001e000u
#define RA8P1_MRAM_BASE        0x4013c000u
#define RA8P1_ETHOSU_BASE      0x40140000u
#define RA8P1_RTC_BASE         0x40202000u
#define RA8P1_WDT_BASE         0x40202600u
#define RA8P1_MSTP_BASE        0x40203000u
#define RA8P1_PSCU_BASE        0x40204000u
#define RA8P1_USBFS_BASE       0x40250000u
#define RA8P1_SDHI0_BASE       0x40252000u
#define RA8P1_PDM_BASE         0x40256000u
#define RA8P1_SSI0_BASE        0x4025d000u
#define RA8P1_IIC0_BASE        0x4025e000u
#define RA8P1_IIC1_BASE        0x4025e100u
#define RA8P1_OSPI0_BASE       0x40268000u
#define RA8P1_GPT0_BASE        0x40322000u
#define RA8P1_GPT2_BASE        0x40322200u
#define RA8P1_GPT6_BASE        0x40322600u
#define RA8P1_GPT7_BASE        0x40322700u
#define RA8P1_GPT9_BASE        0x40322900u
#define RA8P1_GPT10_BASE       0x40322a00u
#define RA8P1_GPT11_BASE       0x40322b00u
#define RA8P1_ADC_B0_BASE      0x40338000u
#define RA8P1_MIPI_PHY_BASE    0x40346c00u
#define RA8P1_MIPI_CSI_BASE    0x40347000u
#define RA8P1_VIN_BASE         0x40347400u
#define RA8P1_SCI1_BASE        0x40358100u
#define RA8P1_SCI2_BASE        0x40358200u
#define RA8P1_SPI0_BASE        0x4035c000u
#define RA8P1_SPI1_BASE        0x4035c100u
#define RA8P1_CANFD_BASE       0x40380000u
#define RA8P1_MFWD_BASE        0x403c0000u
#define RA8P1_ESWM_BASE        0x403c8000u
#define RA8P1_COMA_BASE        0x403c9000u
#define RA8P1_ETHA0_BASE       0x403ca000u
#define RA8P1_RMAC0_BASE       0x403cb000u
#define RA8P1_ETHA1_BASE       0x403cc000u
#define RA8P1_RMAC1_BASE       0x403cd000u
#define RA8P1_GWCA0_BASE       0x403ce000u
#define RA8P1_PORT_BASE        0x40400000u
#define RA8P1_PFS_BASE         0x40400800u
#define RA8P1_PMISC_BASE       0x40400d00u
#define RA8P1_SDRAM_BASE       0x68000000u
#define RA8P1_SDRAM_SIZE       0x02000000u
#define RA8P1_OSPI_CS1_BASE    0x90000000u
#define RA8P1_OSPI_CS1_SIZE    0x10000000u

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_MEMORYMAP_H */
