/****************************************************************************
 * arch/arm/src/ra8p1/ra_start.c
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

#include <stdint.h>

#include <nuttx/cache.h>
#include <nuttx/init.h>

#include "arm_internal.h"
#include "nvic.h"
#include "ra_clockconfig.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

const uintptr_t g_idle_topstack =
  (uintptr_t)_ebss + CONFIG_IDLETHREAD_STACKSIZE;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   Reset entry point for the RA8P1 CPU0.
 *
 ****************************************************************************/

void __start(void)
{
  const uint32_t *src;
  uint32_t *dest;

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss;
       )
    {
      *dest++ = 0;
    }

  for (src = (const uint32_t *)_eronly, dest = (uint32_t *)_sdata;
       dest < (uint32_t *)_edata;
       )
    {
      *dest++ = *src++;
    }

  ra_clockconfig();
  arm_fpuconfig();

  putreg32(NVIC_CFGCON_STKALIGN | NVIC_CFGCON_NONBASETHRDENA,
           NVIC_CFGCON);

#ifdef CONFIG_ARMV8M_ICACHE
  up_enable_icache();
  modifyreg32(NVIC_CFGCON, 0, NVIC_CFGCON_BP | NVIC_CFGCON_LOB);
#endif

#ifdef CONFIG_ARMV8M_DCACHE
  up_enable_dcache();
#endif

  UP_DSB();
  UP_ISB();

  ra_lowsetup();
  arm_lowputs("RA8P1 NuttX boot\r\n");

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif

  nx_start();

  for (; ; )
    {
    }
}
