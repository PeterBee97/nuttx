/****************************************************************************
 * arch/arm/src/ra8p1/ra_icu.c
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

#include <assert.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/ra8p1_icu.h"
#include "ra_icu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_icu_set_event
 ****************************************************************************/

void ra_icu_set_event(int irq, uint32_t event)
{
  DEBUGASSERT(irq >= RA_IRQ_FIRST && irq < NR_IRQS);
  putreg32(event, RA8P1_ICU_IELSR(irq - RA_IRQ_FIRST));
}

/****************************************************************************
 * Name: ra_attach_icu
 ****************************************************************************/

void ra_attach_icu(void)
{
  int i;

  for (i = 0; i < RA_IRQ_NEXTINT; i++)
    {
      putreg32(0, RA8P1_ICU_IELSR(i));
    }

#ifdef CONFIG_RA_SCI1_UART
  ra_icu_set_event(SCI1_RXI, ICU_EVENT_SCI1_RXI);
  ra_icu_set_event(SCI1_TXI, ICU_EVENT_SCI1_TXI);
  ra_icu_set_event(SCI1_ERI, ICU_EVENT_SCI1_ERI);
#endif

#ifdef CONFIG_RA_SCI2_UART
  ra_icu_set_event(SCI2_RXI, ICU_EVENT_SCI2_RXI);
  ra_icu_set_event(SCI2_TXI, ICU_EVENT_SCI2_TXI);
  ra_icu_set_event(SCI2_ERI, ICU_EVENT_SCI2_ERI);
#endif

#ifdef CONFIG_RA8P1_ETHERNET
  ra_icu_set_event(RA_IRQ_GWDI0, ICU_EVENT_GWCA0_GWDI);
#endif

#ifdef CONFIG_RA8P1_SDHI0
  ra_icu_set_event(RA_IRQ_SDHI0_CARD, ICU_EVENT_SDHIMMC0_CARD);
#endif

#ifdef CONFIG_RA8P1_CANFD0
  ra_icu_set_event(RA_IRQ_CAN0_TX, ICU_EVENT_CAN0_TX);
#  ifdef CONFIG_CAN_ERRORS
  ra_icu_set_event(RA_IRQ_CAN0_CHERR, ICU_EVENT_CAN0_CHERR);
  ra_icu_set_event(RA_IRQ_CAN_GLERR, ICU_EVENT_CAN_GLERR);
#  endif
  ra_icu_set_event(RA_IRQ_CAN_RXF, ICU_EVENT_CAN_RXF);
#endif

#ifdef CONFIG_RA8P1_USBFS
  ra_icu_set_event(RA_IRQ_USBFS_INT, ICU_EVENT_USBFS_INT);
#endif

#ifdef CONFIG_RA8P1_SSI0
  ra_icu_set_event(RA_IRQ_SSI0_TXI, ICU_EVENT_SSI0_TXI);
  ra_icu_set_event(RA_IRQ_SSI0_INT, ICU_EVENT_SSI0_INT);
#endif

#ifdef CONFIG_RA8P1_ETHOSU55
  ra_icu_set_event(RA_IRQ_NPU, ICU_EVENT_NPU_IRQ);
#endif

#ifdef CONFIG_RA8P1_CAMERA
  ra_icu_set_event(RA_IRQ_VIN_STATUS, ICU_EVENT_VIN_STATUS);
  ra_icu_set_event(RA_IRQ_VIN_ERROR, ICU_EVENT_VIN_ERROR);
  ra_icu_set_event(RA_IRQ_MIPICSI_DL, ICU_EVENT_MIPICSI_DL);
  ra_icu_set_event(RA_IRQ_MIPICSI_VC, ICU_EVENT_MIPICSI_VC);
#endif
}

/****************************************************************************
 * Name: ra_clear_ir
 ****************************************************************************/

void ra_clear_ir(int irq)
{
  uintptr_t regaddr;

  DEBUGASSERT(irq >= RA_IRQ_FIRST && irq < NR_IRQS);
  regaddr = RA8P1_ICU_IELSR(irq - RA_IRQ_FIRST);
  modifyreg32(regaddr, ICU_IELSR_IR, 0);
  (void)getreg32(regaddr);
}
