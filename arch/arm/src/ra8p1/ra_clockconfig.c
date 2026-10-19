/****************************************************************************
 * arch/arm/src/ra8p1/ra_clockconfig.c
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

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/ra8p1_system.h"
#include "ra_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PERIPHERAL_CLOCK_STOP      (1u << 6)
#define PERIPHERAL_CLOCK_BUSY      (1u << 7)
#define PERIPHERAL_CLOCK_ENABLE    (1u << 7)

/* Clock-tree values match the RA8P1 reference 1 GHz operating profile. */

#define RA_MAIN_OSC_MODE           0x0a
#define RA_MAIN_OSC_WAIT           9
#define RA_PLL1_CONFIG             (((250u - 1) << 8) | 2u)
#define RA_PLL1_DIVIDERS           ((4u << 8) | (5u << 4) | 1u)
#define RA_PLL2_CONFIG             (((300u - 1) << 8) | 2u)
#define RA_PLL2_DIVIDERS           ((4u << 8) | (2u << 4) | 3u)

#define RA_MRAM_CODE_MHZ           250u
#define RA_MRAM_CODE_COMMAND       0x1e000000u
#define RA_MRAM_EXTRA_MHZ          125u
#define RA_MRAM_EXTRA_COMMAND      0xe1000000u

#define RA_SYSTEM_DIVIDERS         0x32233432u
#define RA_SYSTEM_DIVIDERS2        0x2120u
#define RA_SYSTEM_CLOCK_SOURCE     5u
#define RA_PLL_SETTLE_LOOPS        150000

#define RA_SCICLK_SOURCE           8
#define RA_SCICLK_DIVISOR          2
#define RA_OSPICLK_SOURCE          7
#define RA_OSPICLK_DIVISOR         5
#define RA_ADCCLK_SOURCE           10
#define RA_ADCCLK_DIVISOR          4
#define RA_CANFDCLK_SOURCE         10
#define RA_CANFDCLK_DIVISOR        3
#define RA_USBCLK_SOURCE           10
#define RA_USBCLK_DIVISOR          7
#define RA_GPTCLK_SOURCE           6
#define RA_GPTCLK_DIVISOR          1
#define RA_LCDCLK_SOURCE           10
#define RA_LCDCLK_DIVISOR          2
#define RA_ESWCLK_SOURCE           5
#define RA_ESWCLK_DIVISOR          2
#define RA_ESWPCLK_SOURCE          5
#define RA_ESWPCLK_DIVISOR         1
#define RA_ETHPCLK_SOURCE          9
#define RA_ETHPCLK_DIVISOR         9

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ra_peripheral_clock(uintptr_t control, uintptr_t divider,
                                uint8_t source, uint8_t divisor)
{
  modifyreg8(control, 0, PERIPHERAL_CLOCK_STOP);
  while ((getreg8(control) & PERIPHERAL_CLOCK_BUSY) == 0)
    {
    }

  putreg8(divisor, divider);
  putreg8(source | PERIPHERAL_CLOCK_ENABLE | PERIPHERAL_CLOCK_STOP,
          control);
  modifyreg8(control, PERIPHERAL_CLOCK_STOP, 0);
  while ((getreg8(control) & PERIPHERAL_CLOCK_BUSY) != 0)
    {
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_module_control
 *
 * Description:
 *   Start or stop one peripheral module through its MSTP register.  Module
 *   starts are read back and given the hardware-required stabilization
 *   delay before the caller accesses peripheral registers.
 *
 ****************************************************************************/

void ra_module_control(uintptr_t reg, uint32_t mask, bool enable)
{
  irqstate_t flags;

  flags = up_irq_save();
  if (enable)
    {
      modifyreg32(reg, mask, 0);
      (void)getreg32(reg);
    }
  else
    {
      modifyreg32(reg, 0, mask);
    }

  up_irq_restore(flags);

  if (enable)
    {
      up_udelay(30);
    }
}

/****************************************************************************
 * Name: ra_clockconfig
 *
 * Description:
 *   Configure the RA8P1 clock tree from the board's 24 MHz main oscillator.
 *
 ****************************************************************************/

void ra_clockconfig(void)
{
  unsigned int delay;

  putreg16(SYSTEM_PRCR_KEY | SYSTEM_PRCR_PRC0 | SYSTEM_PRCR_PRC1 |
           SYSTEM_PRCR_PRC4,
           RA8P1_SYSTEM_PRCR);

  /* NuttX is a flat secure image.  Keep every peripheral in the secure
   * attribution domain required by the RA8P1 secure flat startup.
   */

  putreg32(0, RA8P1_PSCU_PSARB);
  putreg32(0, RA8P1_PSCU_PSARC);
  putreg32(0, RA8P1_PSCU_PSARD);
  putreg32(0, RA8P1_PSCU_PSARE);

  putreg8(RA_MAIN_OSC_MODE, RA8P1_SYSTEM_MOMCR);
  putreg8(RA_MAIN_OSC_WAIT, RA8P1_SYSTEM_MOSCWTCR);
  putreg8(0, RA8P1_SYSTEM_MOSCCR);
  while ((getreg8(RA8P1_SYSTEM_OSCSF) & SYSTEM_OSCSF_MOSC) == 0)
    {
    }

  putreg32(RA_PLL1_CONFIG, RA8P1_SYSTEM_PLLCCR);
  putreg16(RA_PLL1_DIVIDERS, RA8P1_SYSTEM_PLLCCR2);

  /* Voltage scaling must complete while cache and TCM are disabled. */

  modifyreg32(RA8P1_CPUCFG, CPUCFG_TCM_MASK, 0);
  UP_DSB();
  UP_ISB();
  modifyreg8(RA8P1_SYSTEM_VSCR, SYSTEM_VSCR_VSCM_MASK,
             SYSTEM_VSCR_VSCM_HIGH);
  while ((getreg8(RA8P1_SYSTEM_VSCR) & SYSTEM_VSCR_TRANSITION) != 0)
    {
    }

  modifyreg32(RA8P1_CPUCFG, 0, CPUCFG_TCM_MASK);
  UP_DSB();
  UP_ISB();
  putreg8(0, RA8P1_SYSTEM_PLLCR);

#if defined(CONFIG_RA8P1_ETHERNET) || defined(CONFIG_RA8P1_GPT11) || \
    defined(CONFIG_RA8P1_GPT_PWM) || defined(CONFIG_RA8P1_ADC_B) || \
    defined(CONFIG_RA8P1_CANFD0) || defined(CONFIG_RA8P1_USBFS) || \
    defined(CONFIG_RA8P1_SSI0)
  putreg32(RA_PLL2_CONFIG, RA8P1_SYSTEM_PLL2CCR);
  putreg16(RA_PLL2_DIVIDERS, RA8P1_SYSTEM_PLL2CCR2);
  putreg8(0, RA8P1_SYSTEM_PLL2CR);
  while ((getreg8(RA8P1_SYSTEM_OSCSF) & SYSTEM_OSCSF_PLL2) == 0)
    {
    }
#endif

  while ((getreg8(RA8P1_SYSTEM_OSCSF) & SYSTEM_OSCSF_PLL1) == 0)
    {
    }

  putreg8(0, RA8P1_MRAM_PREFETCH);
  getreg8(RA8P1_MRAM_PREFETCH);
  getreg8(RA8P1_MRAM_PREFETCH);
  getreg8(RA8P1_MRAM_PREFETCH);
  while (getreg32(RA8P1_MRAM_CODE_FREQ) != RA_MRAM_CODE_MHZ)
    {
      putreg32(RA_MRAM_CODE_COMMAND | RA_MRAM_CODE_MHZ,
               RA8P1_MRAM_CODE_FREQ);
    }

  while (getreg32(RA8P1_MRAM_EXTRA_FREQ) != RA_MRAM_EXTRA_MHZ)
    {
      putreg32(RA_MRAM_EXTRA_COMMAND | RA_MRAM_EXTRA_MHZ,
               RA8P1_MRAM_EXTRA_FREQ);
    }

  putreg32(RA_SYSTEM_DIVIDERS, RA8P1_SYSTEM_SCKDIVCR);
  putreg16(RA_SYSTEM_DIVIDERS2, RA8P1_SYSTEM_SCKDIVCR2);

#ifdef CONFIG_RA8P1_SRAM_WAIT
  /* SRAM requires a wait cycle at the selected 250 MHz ICLK.  Configure
   * it before switching to the high-frequency clock source.
   */

  putreg16(SRAM_PRCR_KEY | SRAM_PRCR_WRITE, RA8P1_SRAM_PRCR);
  putreg8(SRAM_WTSC_WAIT, RA8P1_SRAM_WTSC);
  (void)getreg8(RA8P1_SRAM_WTSC);
  putreg16(SRAM_PRCR_KEY, RA8P1_SRAM_PRCR);
#endif

  putreg8(RA_SYSTEM_CLOCK_SOURCE, RA8P1_SYSTEM_SCKSCR);

  /* Allow the PLL supply to settle for at least 150 us at 1 GHz. */

  for (delay = 0; delay < RA_PLL_SETTLE_LOOPS; delay++)
    {
      __asm__ volatile ("nop");
    }

  putreg8(1, RA8P1_MRAM_PREFETCH);
  ra_peripheral_clock(RA8P1_SYSTEM_SCICKCR,
                      RA8P1_SYSTEM_SCICKDIVCR,
                      RA_SCICLK_SOURCE, RA_SCICLK_DIVISOR);

#ifdef CONFIG_RA8P1_OSPI_B
  ra_peripheral_clock(RA8P1_SYSTEM_OCTACKCR,
                      RA8P1_SYSTEM_OCTACKDIVCR,
                      RA_OSPICLK_SOURCE, RA_OSPICLK_DIVISOR);
#endif

#ifdef CONFIG_RA8P1_ADC_B
  ra_peripheral_clock(RA8P1_SYSTEM_ADCCKCR,
                      RA8P1_SYSTEM_ADCCKDIVCR,
                      RA_ADCCLK_SOURCE, RA_ADCCLK_DIVISOR);
#endif

#ifdef CONFIG_RA8P1_CANFD0
  ra_peripheral_clock(RA8P1_SYSTEM_CANFDCKCR,
                      RA8P1_SYSTEM_CANFDCKDIVCR,
                      RA_CANFDCLK_SOURCE, RA_CANFDCLK_DIVISOR);
#endif

#ifdef CONFIG_RA8P1_USBFS
  ra_peripheral_clock(RA8P1_SYSTEM_USBCKCR,
                      RA8P1_SYSTEM_USBCKDIVCR,
                      RA_USBCLK_SOURCE, RA_USBCLK_DIVISOR);
#endif

#if defined(CONFIG_RA8P1_GPT11) || defined(CONFIG_RA8P1_GPT_PWM) || \
    defined(CONFIG_RA8P1_SSI0)
  ra_peripheral_clock(RA8P1_SYSTEM_GPTCKCR,
                      RA8P1_SYSTEM_GPTCKDIVCR,
                      RA_GPTCLK_SOURCE, RA_GPTCLK_DIVISOR);
#endif

#ifdef CONFIG_RA8P1_CAMERA
  ra_peripheral_clock(RA8P1_SYSTEM_LCDCKCR,
                      RA8P1_SYSTEM_LCDCKDIVCR,
                      RA_LCDCLK_SOURCE, RA_LCDCLK_DIVISOR);
#endif

#ifdef CONFIG_RA8P1_ETHERNET
  ra_peripheral_clock(RA8P1_SYSTEM_ESWCKCR,
                      RA8P1_SYSTEM_ESWCKDIVCR,
                      RA_ESWCLK_SOURCE, RA_ESWCLK_DIVISOR);
  ra_peripheral_clock(RA8P1_SYSTEM_ESWPCKCR,
                      RA8P1_SYSTEM_ESWPCKDIVCR,
                      RA_ESWPCLK_SOURCE, RA_ESWPCLK_DIVISOR);
  ra_peripheral_clock(RA8P1_SYSTEM_ETHPCKCR,
                      RA8P1_SYSTEM_ETHPCKDIVCR,
                      RA_ETHPCLK_SOURCE, RA_ETHPCLK_DIVISOR);
#endif

#ifdef CONFIG_RA8P1_CAMERA
  if (getreg8(RA8P1_SYSTEM_PDCTRGD) != 0)
    {
      while ((getreg8(RA8P1_SYSTEM_PDCTRGD) &
              (SYSTEM_PDCTRGD_PDCSF | SYSTEM_PDCTRGD_PDPGSF)) !=
             SYSTEM_PDCTRGD_PDPGSF)
        {
        }

      putreg8(0, RA8P1_SYSTEM_PDCTRGD);
      while ((getreg8(RA8P1_SYSTEM_PDCTRGD) &
              (SYSTEM_PDCTRGD_PDCSF | SYSTEM_PDCTRGD_PDPGSF)) != 0)
        {
        }
    }
#endif

  putreg16(SYSTEM_PRCR_KEY, RA8P1_SYSTEM_PRCR);
}
