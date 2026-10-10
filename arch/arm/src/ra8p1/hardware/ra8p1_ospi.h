/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_ospi.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_OSPI_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_OSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_OSPI_WRAPCFG                (RA8P1_OSPI0_BASE + 0x0000)
#define RA8P1_OSPI_BMCFGCH(n)             (RA8P1_OSPI0_BASE + 0x0008 + \
                                          ((n) << 2))
#define RA8P1_OSPI_CMCFG0(n)              (RA8P1_OSPI0_BASE + 0x0010 + \
                                          ((n) << 4))
#define RA8P1_OSPI_CMCFG1(n)              (RA8P1_OSPI0_BASE + 0x0014 + \
                                          ((n) << 4))
#define RA8P1_OSPI_CMCFG2(n)              (RA8P1_OSPI0_BASE + 0x0018 + \
                                          ((n) << 4))
#define RA8P1_OSPI_LIOCFGCS(n)            (RA8P1_OSPI0_BASE + 0x0050 + \
                                          ((n) << 2))
#define RA8P1_OSPI_BMCTL0                  (RA8P1_OSPI0_BASE + 0x0060)
#define RA8P1_OSPI_BMCTL1                  (RA8P1_OSPI0_BASE + 0x0064)
#define RA8P1_OSPI_CDCTL0                  (RA8P1_OSPI0_BASE + 0x0070)
#define RA8P1_OSPI_CDTBUF(n)               (RA8P1_OSPI0_BASE + 0x0080 + \
                                          ((n) << 4))
#define RA8P1_OSPI_CDABUF(n)               (RA8P1_OSPI0_BASE + 0x0084 + \
                                          ((n) << 4))
#define RA8P1_OSPI_CDD0BUF(n)              (RA8P1_OSPI0_BASE + 0x0088 + \
                                          ((n) << 4))
#define RA8P1_OSPI_CDD1BUF(n)              (RA8P1_OSPI0_BASE + 0x008c + \
                                          ((n) << 4))
#define RA8P1_OSPI_COMSTT                  (RA8P1_OSPI0_BASE + 0x0184)
#define RA8P1_OSPI_INTS                    (RA8P1_OSPI0_BASE + 0x0190)
#define RA8P1_OSPI_INTC                    (RA8P1_OSPI0_BASE + 0x0194)

#define OSPI_WRAPCFG_DSSFTCS1(n)           ((uint32_t)(n) << 24)

#define OSPI_BMCFGCH_COMBINE_64            ((1u << 7) | (15u << 8))

#define OSPI_CMCFG0_ADDRESS_3_BYTES        (2u << 2)
#define OSPI_CMCFG_COMMAND_1_BYTE(n)       ((uint32_t)(n) << 8)

#define OSPI_LIOCFGCS_CSMIN_2_CLOCKS       (1u << 16)

#define OSPI_BMCTL0_CS1_ACCESS             ((3u << 2) | (3u << 6))
#define OSPI_BMCTL1_PUSH_ALL               (3u << 8)

#define OSPI_CDCTL0_TRREQ                  (1u << 0)
#define OSPI_CDCTL0_CS1                    (1u << 3)

#define OSPI_CDT_COMMAND_SIZE(n)           ((uint32_t)(n) << 0)
#define OSPI_CDT_ADDRESS_SIZE(n)           ((uint32_t)(n) << 2)
#define OSPI_CDT_DATA_SIZE(n)              ((uint32_t)(n) << 5)
#define OSPI_CDT_DUMMY_CYCLES(n)           ((uint32_t)(n) << 9)
#define OSPI_CDT_WRITE                     (1u << 15)
#define OSPI_CDT_COMMAND_1_BYTE(n)         ((uint32_t)(n) << 24)

#define OSPI_COMSTT_MEM_ACCESS             (1u << 0)
#define OSPI_COMSTT_WRITE_BUFFER           (1u << 6)
#define OSPI_COMSTT_BUSY                   (OSPI_COMSTT_MEM_ACCESS | \
                                           OSPI_COMSTT_WRITE_BUFFER)

#define OSPI_INTS_COMMAND_COMPLETE         (1u << 0)
#define OSPI_INTS_ERRORS                   ((1u << 3) | (1u << 5) | \
                                           (1u << 9) | (1u << 16) | \
                                           (1u << 18) | (1u << 20))

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_OSPI_H */
