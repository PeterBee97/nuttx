/****************************************************************************
 * arch/arm/src/ra8p1/ra_i2s.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RA8P1_RA_I2S_H
#define __ARCH_ARM_SRC_RA8P1_RA_I2S_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/audio/i2s.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_RA8P1_SSI0
FAR struct i2s_dev_s *ra_i2s_initialize(int port);
#ifdef CONFIG_RA8P1_SSI0_REFERENCE
/* Called with interrupts disabled. Time uses perf_gettime() ticks. */

int16_t ra_i2s_reference(uint32_t time);
#endif
#endif

#endif /* __ARCH_ARM_SRC_RA8P1_RA_I2S_H */
