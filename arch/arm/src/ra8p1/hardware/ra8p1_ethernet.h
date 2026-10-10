/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_ethernet.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_ETHERNET_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Common agent */

#define RA8P1_COMA_RRC                 (RA8P1_COMA_BASE + 0x0004)
#define RA8P1_COMA_RCEC                (RA8P1_COMA_BASE + 0x0008)
#define RA8P1_COMA_CABPIRM             (RA8P1_COMA_BASE + 0x0140)

#define COMA_RRC_RESET                 (1u << 0)
#define COMA_RCEC_AGENT_ALL            0x7fu
#define COMA_RCEC_ENABLE               (1u << 16)
#define COMA_CABPIRM_INITIALIZE        (1u << 0)
#define COMA_CABPIRM_READY             (1u << 1)

/* Ethernet switch media interface */

#define RA8P1_ESWM_MIIRR               (RA8P1_ESWM_BASE + 0x19400)
#define RA8P1_ESWM_MIICR(n)            (RA8P1_ESWM_BASE + 0x19404 + \
                                        ((n) << 2))
#define RA8P1_PMISC_PFENET             (RA8P1_PMISC_BASE + 0x0000)

#define ESWM_MIIRR_RGMII_RESET(n)      (1u << (n))
#define ESWM_MIICR_RGMII               1u
#define ESWM_MIICR_TXC_DELAY           (1u << 12)
#define PMISC_PFENET_PHYMODE(n)        (1u << (4 + (n)))

/* Ethernet agents */

#define RA8P1_ETHA_BASE(n)             (RA8P1_ETHA0_BASE + ((n) << 13))
#define RA8P1_ETHA_EAMC(n)             (RA8P1_ETHA_BASE(n) + 0x0000)
#define RA8P1_ETHA_EAMS(n)             (RA8P1_ETHA_BASE(n) + 0x0004)
#define RA8P1_ETHA_EATPEC(n)           (RA8P1_ETHA_BASE(n) + 0x0020)
#define RA8P1_ETHA_EATDQDC(n, q)       (RA8P1_ETHA_BASE(n) + 0x0060 + \
                                        ((q) << 2))

#define ETHA_MODE_MASK                 0x3u
#define ETHA_MODE_DISABLE              1u
#define ETHA_MODE_CONFIG               2u
#define ETHA_MODE_OPERATION            3u
#define ETHA_QUEUE_DEPTH_MASK          0x7ffu

/* Gateway CPU agent */

#define RA8P1_GWCA_GWMC                (RA8P1_GWCA0_BASE + 0x0000)
#define RA8P1_GWCA_GWMS                (RA8P1_GWCA0_BASE + 0x0004)
#define RA8P1_GWCA_GWDCBAC0            (RA8P1_GWCA0_BASE + 0x0194)
#define RA8P1_GWCA_GWDCBAC1            (RA8P1_GWCA0_BASE + 0x0198)
#define RA8P1_GWCA_GWTRC0              (RA8P1_GWCA0_BASE + 0x0200)
#define RA8P1_GWCA_GWARIRM             (RA8P1_GWCA0_BASE + 0x0380)
#define RA8P1_GWCA_GWDCC(q)            (RA8P1_GWCA0_BASE + 0x0400 + \
                                        ((q) << 2))
#define RA8P1_GWCA_GWDIS0              (RA8P1_GWCA0_BASE + 0x1100)
#define RA8P1_GWCA_GWDIE0              (RA8P1_GWCA0_BASE + 0x1104)
#define RA8P1_GWCA_GWDID0              (RA8P1_GWCA0_BASE + 0x1108)
#define RA8P1_GWCA_GWEIS20             (RA8P1_GWCA0_BASE + 0x1200)
#define RA8P1_GWCA_GWEIE20             (RA8P1_GWCA0_BASE + 0x1204)
#define RA8P1_GWCA_GWEID20             (RA8P1_GWCA0_BASE + 0x1208)

#define GWCA_MODE_MASK                 0x3u
#define GWCA_MODE_DISABLE              1u
#define GWCA_MODE_CONFIG               2u
#define GWCA_MODE_OPERATION            3u
#define GWCA_GWARIRM_INITIALIZE        (1u << 0)
#define GWCA_GWARIRM_READY             (1u << 1)
#define GWCA_GWDCC_EXTENDED            (1u << 8)
#define GWCA_GWDCC_TX_QUEUE            (1u << 11)
#define GWCA_GWDCC_RELOAD              (1u << 24)

/* Multi-port forwarding */

#define RA8P1_MFWD_FWPC1(n)            (RA8P1_MFWD_BASE + 0x0104 + \
                                        ((n) << 4))
#define RA8P1_MFWD_FWPBFC(n)           (RA8P1_MFWD_BASE + 0x4a00 + \
                                        ((n) << 4))
#define RA8P1_MFWD_FWPBFCSDC(n)        (RA8P1_MFWD_BASE + 0x4a04 + \
                                        ((n) << 4))

#define MFWD_FWPC1_DIRECT               (1u << 0)
#define MFWD_FWPBFC_DEST_MASK           0x7fu
#define MFWD_FWPBFC_CPU                 (1u << 2)
#define MFWD_FWPBFCSDC_MASK             0x7fu

/* RMAC */

#define RA8P1_RMAC_BASE(n)              (RA8P1_RMAC0_BASE + ((n) << 13))
#define RA8P1_RMAC_MPSM(n)              (RA8P1_RMAC_BASE(n) + 0x0000)
#define RA8P1_RMAC_MPIC(n)              (RA8P1_RMAC_BASE(n) + 0x0004)
#define RA8P1_RMAC_MIOC(n)              (RA8P1_RMAC_BASE(n) + 0x0010)
#define RA8P1_RMAC_MRGC(n)              (RA8P1_RMAC_BASE(n) + 0x0080)
#define RA8P1_RMAC_MRMAC0(n)            (RA8P1_RMAC_BASE(n) + 0x0084)
#define RA8P1_RMAC_MRMAC1(n)            (RA8P1_RMAC_BASE(n) + 0x0088)
#define RA8P1_RMAC_MRAFC(n)             (RA8P1_RMAC_BASE(n) + 0x008c)

#define RMAC_MPSM_EXECUTE               (1u << 0)
#define RMAC_MPSM_PHY_SHIFT             3
#define RMAC_MPSM_REG_SHIFT             8
#define RMAC_MPSM_OPCODE_SHIFT          13
#define RMAC_MPSM_DATA_SHIFT            16
#define RMAC_MPSM_OPCODE_WRITE          1u
#define RMAC_MPSM_OPCODE_READ           2u
#define RMAC_MPSM_DATA_MASK             0xffffu

#define RMAC_MPIC_RGMII                 2u
#define RMAC_MPIC_SPEED_SHIFT           3
#define RMAC_MPIC_MDC_SHIFT             16
#define RMAC_MPIC_MDC_DIV               49u

#define RMAC_MIOC_RECOVERY_CLOCK        1u
#define RMAC_MRGC_MAGIC_PACKET          (1u << 3)
#define RMAC_MRAFC_MULTICAST_E          (1u << 1)
#define RMAC_MRAFC_BROADCAST_E          (1u << 2)
#define RMAC_MRAFC_MULTICAST_P          (1u << 17)
#define RMAC_MRAFC_BROADCAST_P          (1u << 18)

/* GWCA descriptor fields */

#define GWCA_DESC_SIZE_MASK             0xfffu
#define GWCA_DESC_ERROR_MASK            (7u << 16)
#define GWCA_DESC_INTERRUPT             (1u << 19)
#define GWCA_DESC_TYPE_SHIFT            20
#define GWCA_DESC_TYPE_MASK             (15u << GWCA_DESC_TYPE_SHIFT)
#define GWCA_DESC_TYPE_FEMPTY           4u
#define GWCA_DESC_TYPE_FSINGLE          8u
#define GWCA_DESC_TYPE_LEMPTY           12u
#define GWCA_DESC_TYPE_LINKFIX          0u
#define GWCA_DESC_INFO1_DIRECT          (1u << 2)
#define GWCA_DESC_INFO2_DV(n)           (1u << (16 + (n)))

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_ETHERNET_H */
