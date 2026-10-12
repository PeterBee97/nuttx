/****************************************************************************
 * include/nuttx/audio/pdm.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_PDM_H
#define __INCLUDE_NUTTX_AUDIO_PDM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Read-only snapshot. Counters reset when capture is opened. */

#define PDMIOC_GETSTATS _PDMIOC(1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct pdm_stats_s
{
  uint32_t frames;
  uint32_t overruns;
  uint32_t desyncs;
  uint32_t hwerrors;
  uint32_t fifoerrors;
  uint32_t levelerrors;
  uint32_t shorterrors;
  uint32_t buffered;
  uint32_t highwater;
};

#endif /* __INCLUDE_NUTTX_AUDIO_PDM_H */
