/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_buttons.c
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

#include <stdint.h>
#include <errno.h>

#include <nuttx/irq.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/ra8p1_icu.h"
#include "ra_gpio.h"
#include "ra_icu.h"

#ifdef CONFIG_ARCH_BUTTONS

#ifdef CONFIG_ARCH_IRQBUTTONS
/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra8p1_button_irq_s
{
  xcpt_t handler;
  FAR void *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ra8p1_button_irq_s g_button_irq;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
static int ra8p1_button_interrupt(int irq, FAR void *context, FAR void *arg)
{
  xcpt_t handler = g_button_irq.handler;

  ra_clear_ir(RA_IRQ_ICU_IRQ4);
  return handler == NULL ? OK : handler(irq, context, g_button_irq.arg);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
#ifdef CONFIG_ARCH_IRQBUTTONS
  putreg32(0, RA8P1_ICU_IELSR(RA_IRQ_ICU_IRQ4 - RA_IRQ_FIRST));
  putreg8(0, RA8P1_ICU_IRQCR(4));
  ra_icu_set_event(RA_IRQ_ICU_IRQ4, ICU_EVENT_IRQ4);
#endif

  ra_gpio_configure_input(BOARD_BUTTON_USER_PORT, BOARD_BUTTON_USER_PIN,
                          true,
#ifdef CONFIG_ARCH_IRQBUTTONS
                          true
#else
                          false
#endif
                          );
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  return ra_gpio_read(BOARD_BUTTON_USER_PORT, BOARD_BUTTON_USER_PIN) ?
         0 : BUTTON_USER_BIT;
}

/****************************************************************************
 * Name: board_button_irq
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  irqstate_t flags;
  int ret;

  if (id != BUTTON_USER)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  up_disable_irq(RA_IRQ_ICU_IRQ4);

  if (irqhandler == NULL)
    {
      g_button_irq.handler = NULL;
      g_button_irq.arg = NULL;
      ret = irq_detach(RA_IRQ_ICU_IRQ4);
    }
  else
    {
      g_button_irq.handler = irqhandler;
      g_button_irq.arg = arg;
      ret = irq_attach(RA_IRQ_ICU_IRQ4, ra8p1_button_interrupt, NULL);
      if (ret >= 0)
        {
          ra_clear_ir(RA_IRQ_ICU_IRQ4);
          up_enable_irq(RA_IRQ_ICU_IRQ4);
        }
    }

  leave_critical_section(flags);
  return ret;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
