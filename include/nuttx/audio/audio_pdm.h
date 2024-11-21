/****************************************************************************
 * include/nuttx/audio/audio_pdm.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_AUDIO_PDM_H
#define __INCLUDE_NUTTX_AUDIO_AUDIO_PDM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_AUDIO_PDM

#include <nuttx/audio/audio.h>
#include <nuttx/audio/pdm.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct audio_lowerhalf_s *audio_pdm_initialize(FAR struct pdm_dev_s *pdm,
                                                   bool playback);

#endif /* CONFIG_AUDIO_PDM */

#endif
