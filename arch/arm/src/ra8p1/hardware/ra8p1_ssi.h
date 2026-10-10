/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_ssi.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SSI_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_SSI_SSICR                  (RA8P1_SSI0_BASE + 0x0000)
#define RA8P1_SSI_SSISR                  (RA8P1_SSI0_BASE + 0x0004)
#define RA8P1_SSI_SSIFCR                 (RA8P1_SSI0_BASE + 0x0010)
#define RA8P1_SSI_SSIFSR                 (RA8P1_SSI0_BASE + 0x0014)
#define RA8P1_SSI_SSIFTDR                (RA8P1_SSI0_BASE + 0x0018)
#define RA8P1_SSI_SSIOFR                 (RA8P1_SSI0_BASE + 0x0020)
#define RA8P1_SSI_SSISCR                 (RA8P1_SSI0_BASE + 0x0024)

#define SSI_SSICR_CKS_INTERNAL           (1u << 30)
#define SSI_SSICR_TUIEN                  (1u << 29)
#define SSI_SSICR_IIEN                   (1u << 25)
#define SSI_SSICR_DWL_16                 (1u << 19)
#define SSI_SSICR_SWL_16                 (1u << 16)
#define SSI_SSICR_MST                    (1u << 14)
#define SSI_SSICR_CKDV_DIV12             (9u << 4)
#define SSI_SSICR_TEN                    (1u << 1)
#define SSI_SSICR_REN                    (1u << 0)
#define SSI_SSICR_TXRX_IRQS              (0x1fu << 25)

#define SSI_SSISR_TUIRQ                  (1u << 29)
#define SSI_SSISR_TOIRQ                  (1u << 28)
#define SSI_SSISR_RUIRQ                  (1u << 27)
#define SSI_SSISR_ROIRQ                  (1u << 26)
#define SSI_SSISR_IIRQ                   (1u << 25)
#define SSI_SSISR_ERRORS                 (SSI_SSISR_TUIRQ | SSI_SSISR_TOIRQ | \
                                         SSI_SSISR_RUIRQ | SSI_SSISR_ROIRQ)

#define SSI_SSIFCR_AUCKE                 (1u << 31)
#define SSI_SSIFCR_SSIRST                (1u << 16)
#define SSI_SSIFCR_TIE                   (1u << 3)
#define SSI_SSIFCR_RIE                   (1u << 2)
#define SSI_SSIFCR_TFRST                 (1u << 1)
#define SSI_SSIFCR_RFRST                 (1u << 0)
#define SSI_SSIFCR_FIFO_RESET            (SSI_SSIFCR_TFRST | \
                                         SSI_SSIFCR_RFRST)

#define SSI_SSIFSR_TDC_SHIFT             24
#define SSI_SSIFSR_TDC_MASK              (0x3fu << SSI_SSIFSR_TDC_SHIFT)
#define SSI_SSIFSR_TDE                   (1u << 16)
#define SSI_SSIFSR_RDF                   (1u << 0)
#define SSI_SSIFSR_TDE_CLEAR             SSI_SSIFSR_RDF

#define SSI_SSIOFR_LRCONT                (1u << 8)

#define SSI_SSISCR_TDES(n)               ((uint32_t)(n) << 8)
#define SSI_SSISCR_RDFS(n)               ((uint32_t)(n) << 0)

#define SSI_FIFO_STAGES                  32u

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_SSI_H */
