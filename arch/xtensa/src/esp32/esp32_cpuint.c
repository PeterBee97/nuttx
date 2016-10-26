/****************************************************************************
 * arch/xtensa/src/esp32/esp32_irq.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "xtensa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32_INTSET(n)  ((1 << (n)) - 1)
#define ESP32_LEVEL_SET  ESP32_INTSET(ESP32_CPUINT_NLEVELPERIPHS)
#define ESP32_EDGE_SET   ESP32_INTSET(ESP32_CPUINT_NEDGEPERIPHS)

/* Mapping Peripheral IDs to map register addresses
 *
 * PERIPHERAL ID                  DPORT REGISTER OFFSET
 * MNEMONIC                   VAL PRO CPU APP CPU
 * -------------------------- --- ------- -------
 * ESP32_PERIPH_MAC            0  0x104   0x218
 * ESP32_PERIPH_MAC_NMI        1  0x108   0x21c
 * ESP32_PERIPH_BB             2  0x10c   0x220
 * ESP32_PERIPH_BB_MAC         3  0x110   0x224
 * ESP32_PERIPH_BT_BB          4  0x114   0x228
 * ESP32_PERIPH_BT_BB_NMI      5  0x118   0x22c
 * ESP32_PERIPH_RWBT_IRQ       6  0x11c   0x230
 * ESP32_PERIPH_RWBLE_IRQ      7  0x120   0x234
 * ESP32_PERIPH_RWBT_NMI       8  0x124   0x238
 * ESP32_PERIPH_RWBLE_NMI      9  0x128   0x23c
 * ESP32_PERIPH_SLC0           10 0x12c   0x240
 * ESP32_PERIPH_SLC1           11 0x130   0x244
 * ESP32_PERIPH_UHCI0          12 0x134   0x248
 * ESP32_PERIPH_UHCI1          13 0x138   0x24c
 * ESP32_PERIPH_TG_T0_LEVEL    14 0x13c   0x250
 * ESP32_PERIPH_TG_T1_LEVEL    15 0x140   0x254
 * ESP32_PERIPH_TG_WDT_LEVEL   16 0x144   0x258
 * ESP32_PERIPH_TG_LACT_LEVEL  17 0x148   0x25c
 * ESP32_PERIPH_TG1_T0_LEVEL   18 0x14c   0x260
 * ESP32_PERIPH_TG1_T1_LEVEL   19 0x150   0x264
 * ESP32_PERIPH_TG1_WDT_LEVEL  20 0x154   0x268
 * ESP32_PERIPH_G1_LACT_LEVEL  21 0x158   0x26c
 * ESP32_PERIPH_CPU_GPIO       22 0x15c   0x270
 * ESP32_PERIPH_CPU_NMI        23 0x160   0x274
 * ESP32_PERIPH_CPU_CPU0       24 0x164   0x278
 * ESP32_PERIPH_CPU_CPU1       25 0x168   0x27c
 * ESP32_PERIPH_CPU_CPU2       26 0x16c   0x280
 * ESP32_PERIPH_CPU_CPU3       27 0x170   0x284
 * ESP32_PERIPH_SPI0           28 0x174   0x288
 * ESP32_PERIPH_SPI1           29 0x178   0x28c
 * ESP32_PERIPH_SPI2           30 0x17c   0x290
 * ESP32_PERIPH_SPI3           31 0x180   0x294
 * ESP32_PERIPH_I2S0           32 0x184   0x298
 * ESP32_PERIPH_I2S1           33 0x188   0x29c
 * ESP32_PERIPH_UART           34 0x18c   0x2a0
 * ESP32_PERIPH_UART1          35 0x190   0x2a4
 * ESP32_PERIPH_UART2          36 0x194   0x2a8
 * ESP32_PERIPH_SDIO_HOST      37 0x198   0x2ac
 * ESP32_PERIPH_EMAC           38 0x19c   0x2b0
 * ESP32_PERIPH_PWM0           39 0x1a0   0x2b4
 * ESP32_PERIPH_PWM1           40 0x1a4   0x2b8
 * ESP32_PERIPH_PWM2           41 0x1a8   0x2bc
 * ESP32_PERIPH_PWM3           42 0x1ac   0x2c0
 * ESP32_PERIPH_LEDC           43 0x1b0   0x2c4
 * ESP32_PERIPH_EFUSE          44 0x1b4   0x2c8
 * ESP32_PERIPH_CAN            45 0x1b8   0x2cc
 * ESP32_PERIPH_RTC_CORE       46 0x1bc   0x2d0
 * ESP32_PERIPH_RMT            47 0x1c0   0x2d4
 * ESP32_PERIPH_PCNT           48 0x1c4   0x2d8
 * ESP32_PERIPH_I2C_EXT0       49 0x1c8   0x2dc
 * ESP32_PERIPH_I2C_EXT1       50 0x1cc   0x2e0
 * ESP32_PERIPH_RSA            51 0x1d0   0x2e4
 * ESP32_PERIPH_SPI1_DMA       52 0x1d4   0x2e8
 * ESP32_PERIPH_SPI2_DMA       53 0x1d8   0x2ec
 * ESP32_PERIPH_SPI3_DMA       54 0x1dc   0x2f0
 * ESP32_PERIPH_WDG            55 0x1e0   0x2f4
 * ESP32_PERIPH_TIMER1         56 0x1e4   0x2f8
 * ESP32_PERIPH_TIMER2         57 0x1e8   0x2fc
 * ESP32_PERIPH_TG_T0_EDGE     58 0x1ec   0x300
 * ESP32_PERIPH_TG_T1_EDGE     59 0x1f0   0x304
 * ESP32_PERIPH_TG_WDT_EDGE    60 0x1F4   0x308
 * ESP32_PERIPH_TG_LACT_EDGE   61 0x1F8   0x30c
 * ESP32_PERIPH_TG1_T0_EDGE    62 0x1fc   0x310
 * ESP32_PERIPH_TG1_T1_EDGE    63 0x200   0x314
 * ESP32_PERIPH_TG1_WDT_EDGE   64 0x204   0x318
 * ESP32_PERIPH_TG1_LACT_EDGE  65 0x208   0x31c
 * ESP32_PERIPH_MMU_IA         66 0x20c   0x320
 * ESP32_PERIPH_MPU_IA         67 0x210   0x324
 * ESP32_PERIPH_CACHE_IA       68 0x214   0x328
 */

#define DPORT_PRO_MAP_REGADDR(n) (DR_REG_DPORT_BASE + 0x104 + ((n) << 2))
#define DPORT_APP_MAP_REGADDR(n) (DR_REG_DPORT_BASE + 0x218 + ((n) << 2))

/* CPU interrupts can be detached from any peripheral source by setting the
 * map register to an internal CPU interrupt (6, 7, 11, 15, 16, or 29).
 */

#define NO_CPUINT  ESP32_CPUINT_TIMER0

/* Priority range is 1-5 */

#define ESP32_MIN_PRIORITY     1
#define ESP32_MAX_PRIORITY     5
#define ESP32_PRIO_INDEX(p)    ((p) - ESP32_MIN_PRIORITY)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* g_intenable[] is a shadow copy of the per-CPU INTENABLE register
 * content.
 */

#ifdef CONFIG_SMP

static uint32_t g_intenable[CONFIG_SMP_NCPUS];

#else

static uint32_t g_intenable[1];

#endif

/* Bitsets for free, unallocated CPU interrupts */

static uint32_t g_level_ints = ESP32_LEVEL_SET;
static uint32_t g_edge_ints  = ESP32_EDGE_SET;

/* Bitsets for each interrupt priority 1-5 */

static uint32_t g_priority[5] =
{
  ESP32_INTPRI1_MASK,
  ESP32_INTPRI2_MASK,
  ESP32_INTPRI3_MASK,
  ESP32_INTPRI4_MASK,
  ESP32_INTPRI5_MASK
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the CPU interrupt specified by 'cpuint'
 *
 ****************************************************************************/

void up_disable_irq(int cpuint)
{
#ifdef CONFIG_SMP
  int cpu;
#endif

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32_CPUINT_MAX);

#ifdef CONFIG_SMP
  cpu = up_cpu_index();
  (void)xtensa_disable_cpuint(&g_intenable[cpu], (1ul << cpuint));
#else
  (void)xtensa_disable_cpuint(&g_intenable[0], (1ul << cpuint));
#endif
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Ensable the CPU interrupt specified by 'cpuint'
 *
 ****************************************************************************/

void up_enable_irq(int cpuint)
{
#ifdef CONFIG_SMP
  int cpu;
#endif

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32_CPUINT_MAX);

#ifdef CONFIG_SMP
  cpu = up_cpu_index();
  (void)xtensa_enable_cpuint(&g_intenable[cpu], (1ul << cpuint));
#else
  (void)xtensa_enable_cpuint(&g_intenable[0], (1ul << cpuint));
#endif
}

/****************************************************************************
 * Name:  esp32_alloc_levelint
 *
 * Description:
 *   Allocate a level CPU interrupt
 *
 * Input Parameters:
 *   priority - Priority of the CPU interrupt (1-5)
 *
 * Returned Value:
 *   On success, the allocated level-sensitive, CPU interrupt numbr is
 *   returned.  A negated errno is returned on failure.  The only possible
 *   failure is that all level-sensitive CPU interrupts have already been
 *   allocated.
 *
 ****************************************************************************/

int esp32_alloc_levelint(int priority)
{
  irqstate_t flags;
  uint32_t mask;
  uint32_t intset;
  int cpuint;
  int ret = -ENOMEM;

  DEBUGASSERT(priority >= ESP32_MIN_PRIORITY && priority <= ESP32_MAX_PRIORITY)

  /* Check if there are any level CPU interrupts available */

  flags = enter_critical_section();

  intset = g_level_ints & g_priority[ESP32_PRIO_INDEX(priority)] & ESP32_LEVEL_SET;
  if (intset != 0)
    {
      /* Skip over initial zeroes as quickly in groups of 8 bits. */

      for (cpuint = 0, mask = 0xff;
           cpuint <= ESP32_CPUINT_MAX && (intset & mask) == 0;
           cpuint += 8, mask <<= 8);

      /* Search for an unallocated CPU interrupt number in the remaining intset. */

      for (; cpuint <= ESP32_CPUINT_MAX && intset != 0; cpuint++)
        {
          /* If the bit corresponding to the CPU interrupt is '1', then
           * that CPU interrupt is available.
           */

          mask = (1ul << cpuint);
          if ((intset & mask) != 0)
            {
              /* Got it! */

              g_level_ints &= ~mask;
              ret = cpuint;
              break;
            }

          /* Clear the bit in intset so that we may exit the loop sooner */

          intset &= ~mask;
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name:  esp32_free_levelint
 *
 * Description:
 *   Free a previoulsy allocated level CPU interrupt
 *
 * Input Parameters:
 *   The CPU interrupt number to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_free_levelint(int cpuint)
{
  irqstate_t flags;
  uint32_t mask;

  DEBUGASSERT(cpuint >= 0 && cpuint < ESP32_CPUINT_NLEVELPERIPHS);

  /* Mark the CPU interrupt as available */

  mask  = (1ul << cpuint);
  flags = enter_critical_section();
  DEBUGASSERT((g_level_ints & mask) == 0);
  g_level_ints |= mask;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name:  esp32_alloc_edgeint
 *
 * Description:
 *   Allocate an edge CPU interrupt
 *
 * Input Parameters:
 *   priority - Priority of the CPU interrupt (1-5)
 *
 * Returned Value:
 *   On success, the allocated edge-sensitive, CPU interrupt numbr is
 *   returned.  A negated errno is returned on failure.  The only possible
 *   failure is that all edge-sensitive CPU interrupts have already been
 *   allocated.
 *
 ****************************************************************************/

int esp32_alloc_edgeint(int priority)
{
  irqstate_t flags;
  uint32_t mask;
  uint32_t intset;
  int cpuint;
  int ret = -ENOMEM;

  DEBUGASSERT(priority >= ESP32_MIN_PRIORITY && priority <= ESP32_MAX_PRIORITY)

  /* Check if there are any level CPU interrupts available */

  flags = enter_critical_section();

  intset = g_edge_ints & g_priority[ESP32_PRIO_INDEX(priority)] & ESP32_EDGE_SET;
  if (intset != 0)
    {
      /* Skip over initial zeroes as quickly in groups of 8 bits. */

      for (cpuint = 0, mask = 0xff;
           cpuint <= ESP32_CPUINT_MAX && (intset & mask) == 0;
           cpuint += 8, mask <<= 8);

      /* Search for an unallocated CPU interrupt number in the remaining intset. */

      for (; cpuint <= ESP32_CPUINT_MAX && intset != 0; cpuint++)
        {
          /* If the bit corresponding to the CPU interrupt is '1', then
           * that CPU interrupt is available.
           */

          mask = (1ul << cpuint);
          if ((intset & mask) != 0)
            {
              /* Got it! */

              g_edge_ints &= ~mask;
              ret = cpuint;
              break;
            }

          /* Clear the bit in intset so that we may exit the loop sooner */

          intset &= ~mask;
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name:  esp32_free_edgeint
 *
 * Description:
 *   Free a previoulsy allocated edge CPU interrupt
 *
 * Input Parameters:
 *   The CPU interrupt number to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_free_edgeint(int cpuint)
{
  irqstate_t flags;
  uint32_t mask;

  DEBUGASSERT(cpuint >= 0 && cpuint < ESP32_CPUINT_NEDGEPERIPHS);

  /* Mark the CPU interrupt as available */

  mask  = (1ul << cpuint);
  flags = enter_critical_section();
  DEBUGASSERT((g_edge_ints & mask) == 0);
  g_edge_ints |= mask;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name:  esp32_attach_peripheral
 *
 * Description:
 *   Attach a peripheral interupt to a CPU interrupt.
 *
 * Input Parameters:
 *   cpu      - The CPU to receive the interrupt 0=PRO CPU 1=APP CPU
 *   periphid - The peripheral number from ira.h to be assigned.
 *   cpuint   - The CPU interrupt to receive the peripheral interrupt
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_attach_peripheral(int cpu, int periphid, int cpuint)
{
  uintptr_t regaddr;

  DEBUGASSERT(periphid >= 0 && periphid < NR_PERIPHERALS);
  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32_CPUINT_MAX);
#ifdef CONFIG_SMP
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);

  if (cpu != 0)
    {
      regaddr = DPORT_APP_MAP_REGADDR(periphid);
    }
  else
#endif
    {
      regaddr = DPORT_PRO_MAP_REGADDR(periphid);
    }

  putreg(cpuint, regaddr);
}

/****************************************************************************
 * Name:  esp32_detach_peripheral
 *
 * Description:
 *   Detach a peripheral interupt from a CPU interrupt.
 *
 * Input Parameters:
 *   cpu    - The CPU to receive the interrupt 0=PRO CPU 1=APP CPU
 *   periphid - The peripheral number from ira.h to be assigned.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_detach_peripheral(int cpu, int periphid)
{
  uintptr_t regaddr;

  DEBUGASSERT(periphid >= 0 && periphid < NR_PERIPHERALS);
#ifdef CONFIG_SMP
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);

  if (cpu != 0)
    {
      regaddr = DPORT_APP_MAP_REGADDR(periphid);
    }
  else
#endif
    {
      regaddr = DPORT_PRO_MAP_REGADDR(periphid);
    }

  putreg(NO_CPUINT, regaddr);
}
