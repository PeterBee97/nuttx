/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * NuttX configuration for the Arm Ethos-U core driver.
 */

#ifndef __DRIVERS_AIE_ETHOSU_ETHOSU_CONFIG_H
#define __DRIVERS_AIE_ETHOSU_ETHOSU_CONFIG_H

#include <nuttx/config.h>

#define ETHOSU55 1
#define ETHOSU_ARCH u55
#define ETHOSU_SEMAPHORE_WAIT_INFERENCE \
  CONFIG_AI_ENGINE_ETHOSU_TIMEOUT_MS

#ifdef CONFIG_AI_ENGINE_ETHOSU_DEBUG
#  define ETHOSU_LOG_SEVERITY 4
#else
#  define ETHOSU_LOG_SEVERITY 1
#endif

#endif /* __DRIVERS_AIE_ETHOSU_ETHOSU_CONFIG_H */
