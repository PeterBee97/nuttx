/****************************************************************************
 * include/nuttx/aie/ethosu.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_AIE_ETHOSU_H
#define __INCLUDE_NUTTX_AIE_ETHOSU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/fs/ioctl.h>

#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Ethos-U extensions to the AI engine character driver.
 *
 * ETHOSUIOC_GET_INFO  - Read struct ethosu_info_s.
 * ETHOSUIOC_INVOKE    - Execute struct ethosu_invoke_s.
 * ETHOSUIOC_GET_STATS - Read struct ethosu_stats_s for the last invocation.
 */

#define ETHOSU_MAX_BASE_ADDR 8

#define ETHOSUIOC_GET_INFO _DIOC(0x0080)
#define ETHOSUIOC_INVOKE   _DIOC(0x0081)
#define ETHOSUIOC_GET_STATS _DIOC(0x0082)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ethosu_info_s
{
  uint8_t driver_major;          /* Core-driver major version */
  uint8_t driver_minor;          /* Core-driver minor version */
  uint8_t driver_patch;          /* Core-driver patch version */
  uint8_t reserved;              /* Must be zero */
  uint32_t version_status;       /* Hardware version status */
  uint32_t version_minor;        /* Hardware minor version */
  uint32_t version_major;        /* Hardware major version */
  uint32_t product_major;        /* Product major revision */
  uint32_t arch_patch_rev;       /* Architecture patch revision */
  uint32_t arch_minor_rev;       /* Architecture minor revision */
  uint32_t arch_major_rev;       /* Architecture major revision */
  uint32_t macs_per_cc;          /* Encoded MACs per clock cycle */
  uint32_t cmd_stream_version;   /* Supported command-stream version */
  uint32_t custom_dma;           /* Nonzero when custom DMA is present */
};

struct ethosu_invoke_s
{
  FAR const void *custom_data;   /* Vela COP1 command stream */
  uint32_t custom_data_size;     /* Command-stream size in bytes */
  uint32_t num_base_addr;        /* Number of valid base-address entries */

  /* DMA base addresses and their accessible region sizes. */

  uint64_t base_addr[ETHOSU_MAX_BASE_ADDR];
  size_t base_addr_size[ETHOSU_MAX_BASE_ADDR];
  uint64_t pmu_cycles;           /* Output: invocation PMU cycle count */
};

struct ethosu_stats_s
{
  uint64_t pmu_cycles;           /* Last invocation PMU cycle count */
  int32_t result;                /* Last enum ethosu_job_result result */
};

#endif /* __INCLUDE_NUTTX_AIE_ETHOSU_H */
