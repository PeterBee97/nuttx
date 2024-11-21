/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_pdm_pio.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_PDM_PIO_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_PDM_PIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RP23XX_PDM_PIO_16BIT_STEREO         0
#define RP23XX_PDM_PIO_16BIT_MONO           1
#define RP23XX_PDM_PIO_8BIT_STEREO          2
#define RP23XX_PDM_PIO_8BIT_MONO            3
#define RP23XX_PDM_PIO_MAX_MODE             4

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_pdm_pio_configure
 *
 * Description:
 *   Configure RP23XX PIO for PDM
 *
 ****************************************************************************/

int rp23xx_pdm_pio_configure(int mode, uint32_t samplerate);

/****************************************************************************
 * Name: rp23xx_pdm_pio_enable
 *
 * Description:
 *   Set enable PDM transfer
 *
 ****************************************************************************/

void rp23xx_pdm_pio_enable(bool enable);

/****************************************************************************
 * Name: rp23xx_pdm_pio_getdmaaddr
 *
 * Description:
 *   Get DMA peripheral address for PDM transfer
 *
 ****************************************************************************/

uintptr_t rp23xx_pdm_pio_getdmaaddr(void);

/****************************************************************************
 * Name: rp23xx_pdm_pio_getdmaaddr
 *
 * Description:
 *   Get DREQ number for PDM transfer
 *
 ****************************************************************************/

uint8_t rp23xx_pdm_pio_getdreq(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_PDM_PIO_H */
