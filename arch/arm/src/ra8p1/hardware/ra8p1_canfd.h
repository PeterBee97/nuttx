/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_canfd.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_CANFD_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_CANFD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_CANFD_NCFG                 (RA8P1_CANFD_BASE + 0x0000)
#define RA8P1_CANFD_CTR                  (RA8P1_CANFD_BASE + 0x0004)
#define RA8P1_CANFD_STS                  (RA8P1_CANFD_BASE + 0x0008)
#define RA8P1_CANFD_ERFL                 (RA8P1_CANFD_BASE + 0x000c)
#define RA8P1_CANFD_GCFG                 (RA8P1_CANFD_BASE + 0x0014)
#define RA8P1_CANFD_GCTR                 (RA8P1_CANFD_BASE + 0x0018)
#define RA8P1_CANFD_GSTS                 (RA8P1_CANFD_BASE + 0x001c)
#define RA8P1_CANFD_GERFL                (RA8P1_CANFD_BASE + 0x0020)
#define RA8P1_CANFD_GAFLECTR             (RA8P1_CANFD_BASE + 0x0028)
#define RA8P1_CANFD_GAFLCFG0             (RA8P1_CANFD_BASE + 0x002c)
#define RA8P1_CANFD_RMNB                 (RA8P1_CANFD_BASE + 0x0030)
#define RA8P1_CANFD_RFCC(n)              (RA8P1_CANFD_BASE + 0x003c + \
                                         ((n) << 2))
#define RA8P1_CANFD_RFSTS(n)             (RA8P1_CANFD_BASE + 0x0044 + \
                                         ((n) << 2))
#define RA8P1_CANFD_RFPCTR(n)            (RA8P1_CANFD_BASE + 0x004c + \
                                         ((n) << 2))
#define RA8P1_CANFD_FESTS                (RA8P1_CANFD_BASE + 0x0060)
#define RA8P1_CANFD_FMSTS                (RA8P1_CANFD_BASE + 0x0068)
#define RA8P1_CANFD_RFISTS               (RA8P1_CANFD_BASE + 0x006c)
#define RA8P1_CANFD_TMC(n)               (RA8P1_CANFD_BASE + 0x0070 + (n))
#define RA8P1_CANFD_TMSTS(n)             (RA8P1_CANFD_BASE + 0x0074 + (n))
#define RA8P1_CANFD_TMTCSTS              (RA8P1_CANFD_BASE + 0x0080)
#define RA8P1_CANFD_TMTASTS              (RA8P1_CANFD_BASE + 0x0084)
#define RA8P1_CANFD_TMIEC                (RA8P1_CANFD_BASE + 0x0088)
#define RA8P1_CANFD_GTINTSTS0            (RA8P1_CANFD_BASE + 0x00a4)
#define RA8P1_CANFD_GFDCFG               (RA8P1_CANFD_BASE + 0x00b0)
#define RA8P1_CANFD_DCFG                 (RA8P1_CANFD_BASE + 0x0100)
#define RA8P1_CANFD_FDCFG                (RA8P1_CANFD_BASE + 0x0104)

#define RA8P1_CANFD_GAFL_ID(n)           (RA8P1_CANFD_BASE + 0x0120 + \
                                         ((n) << 4))
#define RA8P1_CANFD_GAFL_MASK(n)         (RA8P1_CANFD_BASE + 0x0124 + \
                                         ((n) << 4))
#define RA8P1_CANFD_GAFL_P0(n)           (RA8P1_CANFD_BASE + 0x0128 + \
                                         ((n) << 4))
#define RA8P1_CANFD_GAFL_P1(n)           (RA8P1_CANFD_BASE + 0x012c + \
                                         ((n) << 4))

#define RA8P1_CANFD_RF_ID(n)             (RA8P1_CANFD_BASE + 0x0520 + \
                                         ((n) * 0x4c))
#define RA8P1_CANFD_RF_PTR(n)            (RA8P1_CANFD_RF_ID(n) + 0x04)
#define RA8P1_CANFD_RF_FDSTS(n)          (RA8P1_CANFD_RF_ID(n) + 0x08)
#define RA8P1_CANFD_RF_DATA(n)           (RA8P1_CANFD_RF_ID(n) + 0x0c)

#define RA8P1_CANFD_TM_ID(n)             (RA8P1_CANFD_BASE + 0x0604 + \
                                         ((n) * 0x4c))
#define RA8P1_CANFD_TM_PTR(n)            (RA8P1_CANFD_TM_ID(n) + 0x04)
#define RA8P1_CANFD_TM_FDCTR(n)          (RA8P1_CANFD_TM_ID(n) + 0x08)
#define RA8P1_CANFD_TM_DATA(n)           (RA8P1_CANFD_TM_ID(n) + 0x0c)

#define CANFD_MODE_MASK                  0x07u
#define CANFD_MODE_OPERATION             0x00u
#define CANFD_MODE_RESET                 0x01u
#define CANFD_MODE_HALT                  0x02u

#define CANFD_GSTS_RAMINIT               (1u << 3)

#define CANFD_CTR_RETURN_BUSOFF          (1u << 3)
#define CANFD_CTR_ERROR_IRQS             (0x1ffu << 8)
#define CANFD_CTR_TEST_SHIFT             24
#define CANFD_TEST_DISABLED              0u
#define CANFD_TEST_LISTEN                3u
#define CANFD_TEST_LOOPBACK_INTERNAL     7u

#define CANFD_STS_ERROR_PASSIVE          (1u << 3)
#define CANFD_STS_BUSOFF                 (1u << 4)
#define CANFD_STS_REC_SHIFT              16
#define CANFD_STS_TEC_SHIFT              24

#define CANFD_ERFL_BUS                   (1u << 0)
#define CANFD_ERFL_WARNING               (1u << 1)
#define CANFD_ERFL_PASSIVE               (1u << 2)
#define CANFD_ERFL_BUSOFF                (1u << 3)
#define CANFD_ERFL_RECOVERY              (1u << 4)
#define CANFD_ERFL_OVERLOAD              (1u << 5)
#define CANFD_ERFL_BUSLOCK               (1u << 6)
#define CANFD_ERFL_ARBLOST               (1u << 7)
#define CANFD_ERFL_STUFF                 (1u << 8)
#define CANFD_ERFL_FORM                  (1u << 9)
#define CANFD_ERFL_ACK                   (1u << 10)
#define CANFD_ERFL_CRC                   (1u << 11)
#define CANFD_ERFL_BIT1                  (1u << 12)
#define CANFD_ERFL_BIT0                  (1u << 13)
#define CANFD_ERFL_ACKDEL                (1u << 14)
#define CANFD_ERFL_ALL                   0x7fffu

#define CANFD_GERFL_DLC                  (1u << 0)
#define CANFD_GERFL_MESSAGE_LOST         (1u << 1)
#define CANFD_GERFL_PAYLOAD_OVERFLOW     (1u << 3)
#define CANFD_GERFL_ECC                  (1u << 16)

#define CANFD_GAFLECTR_ACCESS            (1u << 8)
#define CANFD_GAFLCFG0_RULES(n)          ((uint32_t)(n) << 16)
#define CANFD_GAFL_ID_LOOPBACK           (1u << 29)
#define CANFD_GAFL_P1_FIFO0              (1u << 0)

#define CANFD_RFCC_ENABLE                (1u << 0)
#define CANFD_RFCC_IRQ_ENABLE            (1u << 1)
#define CANFD_RFCC_PAYLOAD_64            (7u << 4)
#define CANFD_RFCC_DEPTH_16              (3u << 8)
#define CANFD_RFCC_IRQ_EVERY_FRAME       (1u << 12)
#define CANFD_RFCC_IRQ_HALF_FULL         (3u << 13)
#define CANFD_RFCC_FIFO0_CONFIG          (CANFD_RFCC_IRQ_ENABLE | \
                                         CANFD_RFCC_PAYLOAD_64 | \
                                         CANFD_RFCC_DEPTH_16 | \
                                         CANFD_RFCC_IRQ_EVERY_FRAME | \
                                         CANFD_RFCC_IRQ_HALF_FULL)
#define CANFD_RFCC_FIFO0                 (CANFD_RFCC_FIFO0_CONFIG | \
                                         CANFD_RFCC_ENABLE)

#define CANFD_RFSTS_EMPTY                (1u << 0)
#define CANFD_RFSTS_MESSAGE_LOST         (1u << 2)
#define CANFD_RFSTS_IRQ                  (1u << 3)

#define CANFD_TMSTS_RESULT_MASK          (3u << 1)
#define CANFD_TMSTS_REQUEST              (1u << 3)
#define CANFD_GTINT_TX_COMPLETE          (1u << 0)
#define CANFD_GTINT_TX_ABORT             (1u << 1)

#define CANFD_ID_MASK                    0x1fffffffu
#define CANFD_ID_RTR                     (1u << 30)
#define CANFD_ID_EXTENDED                (1u << 31)
#define CANFD_DLC_SHIFT                  28
#define CANFD_FD_ESI                     (1u << 0)
#define CANFD_FD_BRS                     (1u << 1)
#define CANFD_FD_FRAME                   (1u << 2)

#define CANFD_NCFG_BRP(n)                (((uint32_t)(n) - 1) << 0)
#define CANFD_NCFG_SJW(n)                (((uint32_t)(n) - 1) << 10)
#define CANFD_NCFG_TSEG1(n)              (((uint32_t)(n) - 1) << 17)
#define CANFD_NCFG_TSEG2(n)              (((uint32_t)(n) - 1) << 25)

#define CANFD_DCFG_BRP(n)                (((uint32_t)(n) - 1) << 0)
#define CANFD_DCFG_TSEG1(n)              (((uint32_t)(n) - 1) << 8)
#define CANFD_DCFG_TSEG2(n)              (((uint32_t)(n) - 1) << 16)
#define CANFD_DCFG_SJW(n)                (((uint32_t)(n) - 1) << 24)

#define CANFD_FDCFG_TDCE                 (1u << 9)
#define CANFD_FDCFG_ESIC                 (1u << 10)
#define CANFD_FDCFG_TDCO(n)              ((uint32_t)(n) << 16)

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_CANFD_H */
