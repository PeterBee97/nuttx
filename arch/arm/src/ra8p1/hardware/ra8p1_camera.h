/****************************************************************************
 * arch/arm/src/ra8p1/hardware/ra8p1_camera.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_CAMERA_H
#define __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_CAMERA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ra8p1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MIPI D-PHY registers */

#define RA8P1_MIPI_PHY_DPHYREFCR       (RA8P1_MIPI_PHY_BASE + 0x0000)
#define RA8P1_MIPI_PHY_DPHYPLFCR       (RA8P1_MIPI_PHY_BASE + 0x0004)
#define RA8P1_MIPI_PHY_DPHYPLOCR       (RA8P1_MIPI_PHY_BASE + 0x0008)
#define RA8P1_MIPI_PHY_DPHYESCCR       (RA8P1_MIPI_PHY_BASE + 0x000c)
#define RA8P1_MIPI_PHY_DPHYPWRCR       (RA8P1_MIPI_PHY_BASE + 0x0010)
#define RA8P1_MIPI_PHY_DPHYSFR         (RA8P1_MIPI_PHY_BASE + 0x001c)
#define RA8P1_MIPI_PHY_DPHYOCR         (RA8P1_MIPI_PHY_BASE + 0x0020)
#define RA8P1_MIPI_PHY_DPHYTIM1        (RA8P1_MIPI_PHY_BASE + 0x0024)
#define RA8P1_MIPI_PHY_DPHYTIM2        (RA8P1_MIPI_PHY_BASE + 0x0028)
#define RA8P1_MIPI_PHY_DPHYTIM3        (RA8P1_MIPI_PHY_BASE + 0x002c)
#define RA8P1_MIPI_PHY_DPHYTIM4        (RA8P1_MIPI_PHY_BASE + 0x0030)
#define RA8P1_MIPI_PHY_DPHYTIM5        (RA8P1_MIPI_PHY_BASE + 0x0034)
#define RA8P1_MIPI_PHY_DPHYTIM6        (RA8P1_MIPI_PHY_BASE + 0x0038)
#define RA8P1_MIPI_PHY_DPHYMDC         (RA8P1_MIPI_PHY_BASE + 0x0048)

#define MIPI_PHY_PWR_ENABLE            (1u << 0)
#define MIPI_PHY_PWR_STABLE            (1u << 0)
#define MIPI_PHY_PLL_STOP              (1u << 0)
#define MIPI_PHY_ENABLE                (1u << 0)

/* MIPI CSI-2 receiver registers */

#define RA8P1_MIPI_CSI_MCT0            (RA8P1_MIPI_CSI_BASE + 0x0010)
#define RA8P1_MIPI_CSI_MCT2            (RA8P1_MIPI_CSI_BASE + 0x0018)
#define RA8P1_MIPI_CSI_MCT3            (RA8P1_MIPI_CSI_BASE + 0x001c)
#define RA8P1_MIPI_CSI_RTCT            (RA8P1_MIPI_CSI_BASE + 0x0028)
#define RA8P1_MIPI_CSI_RTST            (RA8P1_MIPI_CSI_BASE + 0x002c)
#define RA8P1_MIPI_CSI_EPCT            (RA8P1_MIPI_CSI_BASE + 0x0040)
#define RA8P1_MIPI_CSI_EMCT            (RA8P1_MIPI_CSI_BASE + 0x0044)
#define RA8P1_MIPI_CSI_MIST            (RA8P1_MIPI_CSI_BASE + 0x0050)
#define RA8P1_MIPI_CSI_DTEL            (RA8P1_MIPI_CSI_BASE + 0x0060)
#define RA8P1_MIPI_CSI_DTEH            (RA8P1_MIPI_CSI_BASE + 0x0064)
#define RA8P1_MIPI_CSI_RXST            (RA8P1_MIPI_CSI_BASE + 0x0070)
#define RA8P1_MIPI_CSI_RXSC            (RA8P1_MIPI_CSI_BASE + 0x0074)
#define RA8P1_MIPI_CSI_RXIE            (RA8P1_MIPI_CSI_BASE + 0x0078)
#define RA8P1_MIPI_CSI_DLST0           (RA8P1_MIPI_CSI_BASE + 0x0080)
#define RA8P1_MIPI_CSI_DLSC0           (RA8P1_MIPI_CSI_BASE + 0x0084)
#define RA8P1_MIPI_CSI_DLIE0           (RA8P1_MIPI_CSI_BASE + 0x0088)
#define RA8P1_MIPI_CSI_DLST1           (RA8P1_MIPI_CSI_BASE + 0x0090)
#define RA8P1_MIPI_CSI_DLSC1           (RA8P1_MIPI_CSI_BASE + 0x0094)
#define RA8P1_MIPI_CSI_DLIE1           (RA8P1_MIPI_CSI_BASE + 0x0098)
#define RA8P1_MIPI_CSI_VCST(n)         (RA8P1_MIPI_CSI_BASE + 0x0100 + \
                                        ((n) << 4))
#define RA8P1_MIPI_CSI_VCSC(n)         (RA8P1_MIPI_CSI_BASE + 0x0104 + \
                                        ((n) << 4))
#define RA8P1_MIPI_CSI_VCIE(n)         (RA8P1_MIPI_CSI_BASE + 0x0108 + \
                                        ((n) << 4))
#define RA8P1_MIPI_CSI_PMST            (RA8P1_MIPI_CSI_BASE + 0x0200)
#define RA8P1_MIPI_CSI_PMSC            (RA8P1_MIPI_CSI_BASE + 0x0204)
#define RA8P1_MIPI_CSI_PMIE            (RA8P1_MIPI_CSI_BASE + 0x0208)
#define RA8P1_MIPI_CSI_GSCT            (RA8P1_MIPI_CSI_BASE + 0x0280)
#define RA8P1_MIPI_CSI_GSST            (RA8P1_MIPI_CSI_BASE + 0x0284)
#define RA8P1_MIPI_CSI_GSSC            (RA8P1_MIPI_CSI_BASE + 0x0288)
#define RA8P1_MIPI_CSI_GSIE            (RA8P1_MIPI_CSI_BASE + 0x028c)

#define MIPI_CSI_MCT0_LANES(n)         ((uint32_t)(n) << 0)
#define MIPI_CSI_MCT0_ERR_FRAME        (1u << 17)
#define MIPI_CSI_MCT0_RESERVED_PACKET  (1u << 19)
#define MIPI_CSI_MCT0_GENERIC_RULE     (1u << 20)
#define MIPI_CSI_MCT2_FRRCLK(n)        ((uint32_t)(n) << 0)
#define MIPI_CSI_MCT2_FRRSKW(n)        ((uint32_t)(n) << 16)
#define MIPI_CSI_RX_ENABLE             (1u << 0)
#define MIPI_CSI_VIDEO_RESET           (1u << 0)
#define MIPI_CSI_RX_ACTIVE             (1u << 16)
#define MIPI_CSI_RX_DETECTED           (1u << 17)
#define MIPI_CSI_RX_DETECTED_CLEAR     (1u << 17)
#define MIPI_CSI_DATA_YUV422_8         (1u << 30)

#define MIPI_CSI_DL_ERROR_MASK         0x0000000fu
#define MIPI_CSI_DL_ULPS_MASK          (1u << 24)
#define MIPI_CSI_DL_IRQ_MASK           ((1u << 2) | (1u << 3) | \
                                        (1u << 16) | (1u << 17))
#define MIPI_CSI_VC_ERROR_MASK         0x0001031fu
#define MIPI_CSI_VC_CORRECTED          (1u << 5)
#define MIPI_CSI_VC_IRQ_MASK           (MIPI_CSI_VC_ERROR_MASK | \
                                        MIPI_CSI_VC_CORRECTED)

/* VIN registers */

#define RA8P1_VIN_MC                   (RA8P1_VIN_BASE + 0x0000)
#define RA8P1_VIN_MS                   (RA8P1_VIN_BASE + 0x0004)
#define RA8P1_VIN_FC                   (RA8P1_VIN_BASE + 0x0008)
#define RA8P1_VIN_SLPRC                (RA8P1_VIN_BASE + 0x000c)
#define RA8P1_VIN_ELPRC                (RA8P1_VIN_BASE + 0x0010)
#define RA8P1_VIN_SPPRC                (RA8P1_VIN_BASE + 0x0014)
#define RA8P1_VIN_EPPRC                (RA8P1_VIN_BASE + 0x0018)
#define RA8P1_VIN_CSI_IFMD             (RA8P1_VIN_BASE + 0x0020)
#define RA8P1_VIN_CSIFLD               (RA8P1_VIN_BASE + 0x0024)
#define RA8P1_VIN_IS                   (RA8P1_VIN_BASE + 0x002c)
#define RA8P1_VIN_MB1                  (RA8P1_VIN_BASE + 0x0030)
#define RA8P1_VIN_MB2                  (RA8P1_VIN_BASE + 0x0034)
#define RA8P1_VIN_MB3                  (RA8P1_VIN_BASE + 0x0038)
#define RA8P1_VIN_LC                   (RA8P1_VIN_BASE + 0x003c)
#define RA8P1_VIN_IE                   (RA8P1_VIN_BASE + 0x0040)
#define RA8P1_VIN_INTS                 (RA8P1_VIN_BASE + 0x0044)
#define RA8P1_VIN_SI                   (RA8P1_VIN_BASE + 0x0048)
#define RA8P1_VIN_MTCSTOP              (RA8P1_VIN_BASE + 0x0054)
#define RA8P1_VIN_DMR                  (RA8P1_VIN_BASE + 0x0058)
#define RA8P1_VIN_UVAOF                (RA8P1_VIN_BASE + 0x0060)
#define RA8P1_VIN_UDS_CTRL             (RA8P1_VIN_BASE + 0x0080)
#define RA8P1_VIN_UDS_SCALE            (RA8P1_VIN_BASE + 0x0084)
#define RA8P1_VIN_UDS_PASS_BWIDTH      (RA8P1_VIN_BASE + 0x0090)
#define RA8P1_VIN_UDS_CLIP_SIZE        (RA8P1_VIN_BASE + 0x00a4)

#define VIN_MC_ENABLE                  (1u << 0)
#define VIN_MC_COLOR_BYPASS            (1u << 1)
#define VIN_MC_INTERLACE_BOTH          (1u << 3)
#define VIN_MC_INPUT_YUV422_8          (1u << 16)
#define VIN_MC_START                   (1u << 22)
#define VIN_MC_YUV444_INTERPOLATE      (1u << 25)
#define VIN_MC_SCALE_ENABLE            (1u << 26)
#define VIN_FC_CONTINUOUS              (1u << 1)
#define VIN_CSI_IFMD_YUV422_8          (0x1eu << 8)
#define VIN_CSI_IFMD_ZERO_EXTEND       (1u << 25)
#define VIN_CSIFLD_ENABLE              (1u << 0)
#define VIN_CSIFLD_EVEN_BIT0           (1u << 4)
#define VIN_DMR_ALPHA_BIT              (1u << 2)
#define VIN_DMR_BYTE_SWAP              (1u << 4)
#define VIN_DMR_ALPHA8(n)              ((uint32_t)(n) << 24)
#define VIN_UDS_ADVANCED               (1u << 28)
#define VIN_UDS_SCALE_ONE              0x10001000u
#define VIN_UDS_BWIDTH_64              0x00400040u

#define VIN_MS_CAPTURE_ACTIVE          (1u << 0)
#define VIN_MS_BUFFER_SHIFT            19
#define VIN_MS_BUFFER_MASK             (3u << VIN_MS_BUFFER_SHIFT)
#define VIN_MTCSTOP_ACK                (1u << 1)
#define VIN_MTCSTOP_OUTSTANDING_MASK   (0x3fu << 16)

#define VIN_INT_FIFO_OVERFLOW          (1u << 0)
#define VIN_INT_FRAME_COMPLETE         (1u << 5)
#define VIN_INT_PRECLIP_H              (1u << 8)
#define VIN_INT_PRECLIP_V              (1u << 9)
#define VIN_INT_RESPONSE_OVERFLOW      (1u << 14)
#define VIN_INT_AXI_ERROR              (1u << 15)
#define VIN_INT_FIELD2                 (1u << 31)
#define VIN_INT_ERRORS                 (VIN_INT_FIFO_OVERFLOW | \
                                        VIN_INT_PRECLIP_H | \
                                        VIN_INT_PRECLIP_V | \
                                        VIN_INT_RESPONSE_OVERFLOW | \
                                        VIN_INT_AXI_ERROR)
#define VIN_INT_ENABLE_MASK            (VIN_INT_FRAME_COMPLETE | \
                                        VIN_INT_ERRORS)

#endif /* __ARCH_ARM_SRC_RA8P1_HARDWARE_RA8P1_CAMERA_H */
