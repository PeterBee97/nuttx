/****************************************************************************
 * arch/arm/src/ra8p1/ra_ethosu.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RA8P1_RA_ETHOSU_H
#define __ARCH_ARM_SRC_RA8P1_RA_ETHOSU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int ra8p1_ethosu_initialize(FAR const char *devpath);

#endif /* __ARCH_ARM_SRC_RA8P1_RA_ETHOSU_H */
