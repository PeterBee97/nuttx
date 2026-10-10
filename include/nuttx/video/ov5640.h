/****************************************************************************
 * include/nuttx/video/ov5640.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_OV5640_H
#define __INCLUDE_NUTTX_VIDEO_OV5640_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgsensor.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ov5640_initialize
 *
 * Description:
 *   Allocate and initialize an OV5640 image-sensor interface.
 *
 * Input Parameters:
 *   i2c - I2C controller connected to the sensor.
 *
 * Returned Value:
 *   A pointer to the image-sensor interface on success; NULL on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_VIDEO_OV5640
FAR struct imgsensor_s *
ov5640_initialize(FAR struct i2c_master_s *i2c);
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_OV5640_H */
