/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_ospi.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <sys/stat.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

#include <arch/board/board.h>

#include "ra_ospi.h"
#include "ra_pinmux.h"

#ifdef CONFIG_TITAN_MINI_OSPI

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_ospi_pins[] =
{
  BOARD_OSPI0_SIO0_PIN,
  BOARD_OSPI0_SIO1_PIN,
  BOARD_OSPI0_SIO2_PIN,
  BOARD_OSPI0_SIO3_PIN,
  BOARD_OSPI0_SCLK_PIN,
  BOARD_OSPI0_CS1_PIN,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8p1_ospi_initialize
 ****************************************************************************/

int ra8p1_ospi_initialize(void)
{
  FAR struct qspi_dev_s *qspi;
  FAR struct mtd_dev_s *mtd;
#ifdef CONFIG_TITAN_MINI_OSPI_FILESYSTEM
  FAR struct mtd_dev_s *partition;
  struct mtd_geometry_s geometry;
#endif
  unsigned int i;
  int ret;

  for (i = 0; i < nitems(g_ospi_pins); i++)
    {
      ra_pinmux_configure(g_ospi_pins[i]);
    }

  qspi = ra_ospi_initialize(0, 1);
  if (qspi == NULL)
    {
      return -ENODEV;
    }

  mtd = w25qxxxjv_initialize(qspi, false);
  if (mtd == NULL)
    {
      return -ENODEV;
    }

  ret = register_mtddriver("/dev/ospi0", mtd, 0755, NULL);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_TITAN_MINI_OSPI_FILESYSTEM
  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY,
                  (unsigned long)(uintptr_t)&geometry);
  if (ret < 0 || geometry.blocksize == 0 ||
      BOARD_OSPI_FS_OFFSET % geometry.blocksize != 0 ||
      BOARD_OSPI_FS_SIZE % geometry.blocksize != 0)
    {
      return -EINVAL;
    }

  partition = mtd_partition(mtd,
                            BOARD_OSPI_FS_OFFSET / geometry.blocksize,
                            BOARD_OSPI_FS_SIZE / geometry.blocksize);
  if (partition == NULL)
    {
      return -ENODEV;
    }

  ret = register_mtddriver("/dev/ospi_fs", partition, 0755, NULL);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_TITAN_MINI_OSPI_AUTOMOUNT
  ret = mkdir("/fal", 0755);
  if (ret < 0 && errno != EEXIST)
    {
      return -errno;
    }

  ret = nx_mount("/dev/ospi_fs", "/fal", "littlefs", 0, NULL);
#endif
#endif

  return ret;
}

#endif /* CONFIG_TITAN_MINI_OSPI */
