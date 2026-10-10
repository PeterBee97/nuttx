/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_usbfs.h
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

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_USBFS_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_USBFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets */

#define RA_USBFS_SYSCFG_OFFSET       0x0000
#define RA_USBFS_BUSWAIT_OFFSET      0x0002
#define RA_USBFS_SYSSTS0_OFFSET      0x0004
#define RA_USBFS_DVSTCTR0_OFFSET     0x0008
#define RA_USBFS_CFIFO_OFFSET        0x0014
#define RA_USBFS_D0FIFO_OFFSET       0x0018
#define RA_USBFS_D1FIFO_OFFSET       0x001c
#define RA_USBFS_CFIFOSEL_OFFSET     0x0020
#define RA_USBFS_CFIFOCTR_OFFSET     0x0022
#define RA_USBFS_D0FIFOSEL_OFFSET    0x0028
#define RA_USBFS_D0FIFOCTR_OFFSET    0x002a
#define RA_USBFS_D1FIFOSEL_OFFSET    0x002c
#define RA_USBFS_D1FIFOCTR_OFFSET    0x002e
#define RA_USBFS_INTENB0_OFFSET      0x0030
#define RA_USBFS_INTENB1_OFFSET      0x0032
#define RA_USBFS_BRDYENB_OFFSET      0x0036
#define RA_USBFS_NRDYENB_OFFSET      0x0038
#define RA_USBFS_BEMPENB_OFFSET      0x003a
#define RA_USBFS_SOFCFG_OFFSET       0x003c
#define RA_USBFS_INTSTS0_OFFSET      0x0040
#define RA_USBFS_INTSTS1_OFFSET      0x0042
#define RA_USBFS_BRDYSTS_OFFSET      0x0046
#define RA_USBFS_NRDYSTS_OFFSET      0x0048
#define RA_USBFS_BEMPSTS_OFFSET      0x004a
#define RA_USBFS_FRMNUM_OFFSET       0x004c
#define RA_USBFS_USBADDR_OFFSET      0x0050
#define RA_USBFS_USBREQ_OFFSET       0x0054
#define RA_USBFS_USBVAL_OFFSET       0x0056
#define RA_USBFS_USBINDX_OFFSET      0x0058
#define RA_USBFS_USBLENG_OFFSET      0x005a
#define RA_USBFS_DCPCFG_OFFSET       0x005c
#define RA_USBFS_DCPMAXP_OFFSET      0x005e
#define RA_USBFS_DCPCTR_OFFSET       0x0060
#define RA_USBFS_PIPESEL_OFFSET      0x0064
#define RA_USBFS_PIPECFG_OFFSET      0x0068
#define RA_USBFS_PIPEMAXP_OFFSET     0x006c
#define RA_USBFS_PIPEPERI_OFFSET     0x006e
#define RA_USBFS_PIPECTR_OFFSET(n)   (0x0070 + (((n) - 1) << 1))
#define RA_USBFS_PIPETRE_OFFSET(n)   (0x0090 + (((n) - 1) << 2))
#define RA_USBFS_PIPETRN_OFFSET(n)   (0x0092 + (((n) - 1) << 2))
#define RA_USBFS_PHYSLEW_OFFSET      0x00f0
#define RA_USBFS_DPUSR0R_FS_OFFSET   0x0400

/* Register addresses */

#define RA_USBFS_SYSCFG       (RA8P1_USBFS_BASE + RA_USBFS_SYSCFG_OFFSET)
#define RA_USBFS_BUSWAIT      (RA8P1_USBFS_BASE + RA_USBFS_BUSWAIT_OFFSET)
#define RA_USBFS_SYSSTS0      (RA8P1_USBFS_BASE + RA_USBFS_SYSSTS0_OFFSET)
#define RA_USBFS_DVSTCTR0     (RA8P1_USBFS_BASE + RA_USBFS_DVSTCTR0_OFFSET)
#define RA_USBFS_CFIFO        (RA8P1_USBFS_BASE + RA_USBFS_CFIFO_OFFSET)
#define RA_USBFS_D0FIFO       (RA8P1_USBFS_BASE + RA_USBFS_D0FIFO_OFFSET)
#define RA_USBFS_D1FIFO       (RA8P1_USBFS_BASE + RA_USBFS_D1FIFO_OFFSET)
#define RA_USBFS_CFIFOSEL     (RA8P1_USBFS_BASE + RA_USBFS_CFIFOSEL_OFFSET)
#define RA_USBFS_CFIFOCTR     (RA8P1_USBFS_BASE + RA_USBFS_CFIFOCTR_OFFSET)
#define RA_USBFS_D0FIFOSEL    (RA8P1_USBFS_BASE + RA_USBFS_D0FIFOSEL_OFFSET)
#define RA_USBFS_D0FIFOCTR    (RA8P1_USBFS_BASE + RA_USBFS_D0FIFOCTR_OFFSET)
#define RA_USBFS_D1FIFOSEL    (RA8P1_USBFS_BASE + RA_USBFS_D1FIFOSEL_OFFSET)
#define RA_USBFS_D1FIFOCTR    (RA8P1_USBFS_BASE + RA_USBFS_D1FIFOCTR_OFFSET)
#define RA_USBFS_INTENB0      (RA8P1_USBFS_BASE + RA_USBFS_INTENB0_OFFSET)
#define RA_USBFS_INTENB1      (RA8P1_USBFS_BASE + RA_USBFS_INTENB1_OFFSET)
#define RA_USBFS_BRDYENB      (RA8P1_USBFS_BASE + RA_USBFS_BRDYENB_OFFSET)
#define RA_USBFS_NRDYENB      (RA8P1_USBFS_BASE + RA_USBFS_NRDYENB_OFFSET)
#define RA_USBFS_BEMPENB      (RA8P1_USBFS_BASE + RA_USBFS_BEMPENB_OFFSET)
#define RA_USBFS_SOFCFG       (RA8P1_USBFS_BASE + RA_USBFS_SOFCFG_OFFSET)
#define RA_USBFS_INTSTS0      (RA8P1_USBFS_BASE + RA_USBFS_INTSTS0_OFFSET)
#define RA_USBFS_INTSTS1      (RA8P1_USBFS_BASE + RA_USBFS_INTSTS1_OFFSET)
#define RA_USBFS_BRDYSTS      (RA8P1_USBFS_BASE + RA_USBFS_BRDYSTS_OFFSET)
#define RA_USBFS_NRDYSTS      (RA8P1_USBFS_BASE + RA_USBFS_NRDYSTS_OFFSET)
#define RA_USBFS_BEMPSTS      (RA8P1_USBFS_BASE + RA_USBFS_BEMPSTS_OFFSET)
#define RA_USBFS_FRMNUM       (RA8P1_USBFS_BASE + RA_USBFS_FRMNUM_OFFSET)
#define RA_USBFS_USBADDR      (RA8P1_USBFS_BASE + RA_USBFS_USBADDR_OFFSET)
#define RA_USBFS_USBREQ       (RA8P1_USBFS_BASE + RA_USBFS_USBREQ_OFFSET)
#define RA_USBFS_USBVAL       (RA8P1_USBFS_BASE + RA_USBFS_USBVAL_OFFSET)
#define RA_USBFS_USBINDX      (RA8P1_USBFS_BASE + RA_USBFS_USBINDX_OFFSET)
#define RA_USBFS_USBLENG      (RA8P1_USBFS_BASE + RA_USBFS_USBLENG_OFFSET)
#define RA_USBFS_DCPCFG       (RA8P1_USBFS_BASE + RA_USBFS_DCPCFG_OFFSET)
#define RA_USBFS_DCPMAXP      (RA8P1_USBFS_BASE + RA_USBFS_DCPMAXP_OFFSET)
#define RA_USBFS_DCPCTR       (RA8P1_USBFS_BASE + RA_USBFS_DCPCTR_OFFSET)
#define RA_USBFS_PIPESEL      (RA8P1_USBFS_BASE + RA_USBFS_PIPESEL_OFFSET)
#define RA_USBFS_PIPECFG      (RA8P1_USBFS_BASE + RA_USBFS_PIPECFG_OFFSET)
#define RA_USBFS_PIPEMAXP     (RA8P1_USBFS_BASE + RA_USBFS_PIPEMAXP_OFFSET)
#define RA_USBFS_PIPEPERI     (RA8P1_USBFS_BASE + RA_USBFS_PIPEPERI_OFFSET)
#define RA_USBFS_PIPECTR(n) \
  (RA8P1_USBFS_BASE + RA_USBFS_PIPECTR_OFFSET(n))
#define RA_USBFS_PIPETRE(n) \
  (RA8P1_USBFS_BASE + RA_USBFS_PIPETRE_OFFSET(n))
#define RA_USBFS_PIPETRN(n) \
  (RA8P1_USBFS_BASE + RA_USBFS_PIPETRN_OFFSET(n))
#define RA_USBFS_PHYSLEW      (RA8P1_USBFS_BASE + RA_USBFS_PHYSLEW_OFFSET)
#define RA_USBFS_DPUSR0R_FS \
  (RA8P1_USBFS_BASE + RA_USBFS_DPUSR0R_FS_OFFSET)

/* SYSCFG */

#define USBFS_SYSCFG_USBE              (1u << 0)
#define USBFS_SYSCFG_DMRPU             (1u << 3)
#define USBFS_SYSCFG_DPRPU             (1u << 4)
#define USBFS_SYSCFG_DRPD              (1u << 5)
#define USBFS_SYSCFG_DCFM              (1u << 6)
#define USBFS_SYSCFG_CNEN              (1u << 8)
#define USBFS_SYSCFG_SCKE              (1u << 10)

/* DVSTCTR0 */

#define USBFS_DVSTCTR0_RHST_MASK       (7u << 0)
#define USBFS_DVSTCTR0_RHST_LS         (1u << 0)
#define USBFS_DVSTCTR0_RHST_FS         (2u << 0)
#define USBFS_DVSTCTR0_UACT            (1u << 4)
#define USBFS_DVSTCTR0_RESUME          (1u << 5)
#define USBFS_DVSTCTR0_USBRST          (1u << 6)
#define USBFS_DVSTCTR0_RWUPE           (1u << 7)
#define USBFS_DVSTCTR0_WKUP            (1u << 8)

/* FIFO select and control */

#define USBFS_FIFOSEL_CURPIPE_MASK      (15u << 0)
#define USBFS_CFIFOSEL_ISEL             (1u << 5)
#define USBFS_FIFOSEL_BIGEND            (1u << 8)
#define USBFS_FIFOSEL_MBW_MASK          (3u << 10)
#define USBFS_FIFOSEL_MBW_8             (0u << 10)
#define USBFS_FIFOSEL_MBW_16            (1u << 10)
#define USBFS_FIFOSEL_MBW_32            (2u << 10)
#define USBFS_FIFOSEL_REW               (1u << 14)
#define USBFS_FIFOSEL_RCNT              (1u << 15)

#define USBFS_FIFOCTR_DTLN_MASK         (0x0fffu << 0)
#define USBFS_FIFOCTR_FRDY              (1u << 13)
#define USBFS_FIFOCTR_BCLR              (1u << 14)
#define USBFS_FIFOCTR_BVAL              (1u << 15)

/* Main interrupt status and enable bits */

#define USBFS_INT_BRDY                  (1u << 8)
#define USBFS_INT_NRDY                  (1u << 9)
#define USBFS_INT_BEMP                  (1u << 10)
#define USBFS_INT_CTRT                  (1u << 11)
#define USBFS_INT_DVST                  (1u << 12)
#define USBFS_INT_SOFR                  (1u << 13)
#define USBFS_INT_RESM                  (1u << 14)
#define USBFS_INT_VBINT                 (1u << 15)

#define USBFS_INTSTS0_CTSQ_MASK         (7u << 0)
#define USBFS_INTSTS0_CTSQ_IDLE         (0u << 0)
#define USBFS_INTSTS0_CTSQ_RDDS         (1u << 0)
#define USBFS_INTSTS0_CTSQ_RDSS         (2u << 0)
#define USBFS_INTSTS0_CTSQ_WRDS         (3u << 0)
#define USBFS_INTSTS0_CTSQ_WRSS         (4u << 0)
#define USBFS_INTSTS0_CTSQ_WRND         (5u << 0)
#define USBFS_INTSTS0_CTSQ_SQER         (6u << 0)
#define USBFS_INTSTS0_VALID             (1u << 3)
#define USBFS_INTSTS0_DVSQ_MASK         (7u << 4)
#define USBFS_INTSTS0_DVSQ_POWERED      (0u << 4)
#define USBFS_INTSTS0_DVSQ_DEFAULT      (1u << 4)
#define USBFS_INTSTS0_DVSQ_ADDRESS      (2u << 4)
#define USBFS_INTSTS0_DVSQ_CONFIGURED   (3u << 4)
#define USBFS_INTSTS0_DVSQ_SUSPENDED    (4u << 4)
#define USBFS_INTSTS0_VBSTS             (1u << 7)

/* Frame number */

#define USBFS_FRMNUM_MASK               0x07ffu

/* Pipe configuration */

#define USBFS_PIPECFG_EPNUM_MASK        (15u << 0)
#define USBFS_PIPECFG_DIR_IN            (1u << 4)
#define USBFS_PIPECFG_SHTNAK            (1u << 7)
#define USBFS_PIPECFG_DBLB              (1u << 9)
#define USBFS_PIPECFG_BFRE              (1u << 10)
#define USBFS_PIPECFG_TYPE_MASK         (3u << 14)
#define USBFS_PIPECFG_TYPE_BULK         (1u << 14)
#define USBFS_PIPECFG_TYPE_INT          (2u << 14)
#define USBFS_PIPECFG_TYPE_ISOC         (3u << 14)

#define USBFS_PIPEMAXP_MASK             0x01ffu
#define USBFS_DCPMAXP_MASK              0x007fu
#define USBFS_PIPEPERI_IITV_MASK        (7u << 0)

/* DCPCTR and PIPEnCTR */

#define USBFS_PIPECTR_PID_MASK          (3u << 0)
#define USBFS_PIPECTR_PID_NAK           (0u << 0)
#define USBFS_PIPECTR_PID_BUF           (1u << 0)
#define USBFS_PIPECTR_PID_STALL         (2u << 0)
#define USBFS_DCPCTR_CCPL               (1u << 2)
#define USBFS_PIPECTR_PBUSY             (1u << 5)
#define USBFS_PIPECTR_SQMON             (1u << 6)
#define USBFS_PIPECTR_SQSET             (1u << 7)
#define USBFS_PIPECTR_SQCLR             (1u << 8)
#define USBFS_PIPECTR_ACLRM             (1u << 9)
#define USBFS_PIPECTR_INBUFM            (1u << 14)
#define USBFS_PIPECTR_BSTS              (1u << 15)

/* PIPEnTRE */

#define USBFS_PIPETRE_TRCLR             (1u << 8)
#define USBFS_PIPETRE_TRENB             (1u << 9)

/* Deep-standby transceiver control */

#define USBFS_DPUSR0R_FS_FIXPHY0        (1u << 4)

/* Pipe helpers */

#define USBFS_PIPE_BIT(n)               (1u << (n))
#define USBFS_PIPE_MASK                 0x03ffu

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_USBFS_H */
