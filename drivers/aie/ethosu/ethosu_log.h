/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * NuttX logging adapter for the Arm Ethos-U core driver.
 */

#ifndef __DRIVERS_AIE_ETHOSU_ETHOSU_LOG_H
#define __DRIVERS_AIE_ETHOSU_ETHOSU_LOG_H

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/

#include <stdio.h>

/******************************************************************************
 * Defines
 ******************************************************************************/

// Log severity levels
#define ETHOSU_LOG_DISABLED    0
#define ETHOSU_LOG_ERR         1
#define ETHOSU_LOG_WARN        2
#define ETHOSU_LOG_INFO        3
#define ETHOSU_LOG_DEBUG       4

#include "ethosu_config.h"

// Define default log severity
#ifndef ETHOSU_LOG_SEVERITY
#define ETHOSU_LOG_SEVERITY    ETHOSU_LOG_DISABLED
#endif

#if ETHOSU_LOG_SEVERITY >= ETHOSU_LOG_ERR
#define LOG(f, ...)    (void)printf(f, ## __VA_ARGS__)
#else
#define LOG(f, ...)
#endif

#if ETHOSU_LOG_SEVERITY >= ETHOSU_LOG_ERR
#define LOG_ERR(f, ...)    (void)printf("Err: " f "\n", ## __VA_ARGS__)
#else
#define LOG_ERR(f, ...)
#endif

#if ETHOSU_LOG_SEVERITY >= ETHOSU_LOG_WARN
#define LOG_WARN(f, ...)    (void)printf("Warn: " f "\n", ## __VA_ARGS__)
#else
#define LOG_WARN(f, ...)
#endif

#if ETHOSU_LOG_SEVERITY >= ETHOSU_LOG_INFO
#define LOG_INFO(f, ...)    (void)printf("Info: " f "\n", ## __VA_ARGS__)
#else
#define LOG_INFO(f, ...)
#endif

#if ETHOSU_LOG_SEVERITY >= ETHOSU_LOG_DEBUG
#define LOG_DEBUG(f, ...)    (void)printf("Debug: " f "\n", ## __VA_ARGS__)
#else
#define LOG_DEBUG(f, ...)
#endif

#endif /* __DRIVERS_AIE_ETHOSU_ETHOSU_LOG_H */
