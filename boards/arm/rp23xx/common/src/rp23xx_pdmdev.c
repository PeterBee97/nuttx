/****************************************************************************
 * boards/arm/rp23xx/common/src/rp23xx_pdmdev.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/pdm.h>
#include <nuttx/audio/audio_pdm.h>
#include <nuttx/audio/pcm.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "rp23xx_pdm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_pdmdev_initialize
 *
 * Description:
 *   Initialize pdm driver and register the /dev/audio/pcm0 device.
 *
 ****************************************************************************/

int board_pdmdev_initialize(int port)
{
  struct audio_lowerhalf_s *audio_pdm;
  struct audio_lowerhalf_s *pcm;
  struct pdm_dev_s *pdm;
  char devname[12];
  int ret;

  ainfo("Initializing PDM\n");

  pdm = rp23xx_pdmbus_initialize(port);

#ifdef CONFIG_AUDIO_PDMCHAR
  pdmchar_register(pdm, 0);
#endif

  audio_pdm = audio_pdm_initialize(pdm, false);

  if (!audio_pdm)
    {
      auderr("ERROR: Failed to initialize PDM\n");
      return -ENODEV;
    }

  snprintf(devname, 12, "pcm%dc", port);

  ret = audio_register(devname, audio_pdm);

  if (ret < 0)
    {
      auderr("ERROR: Failed to register /dev/%s device: %d\n", devname, ret);
    }

  return 0;
}
