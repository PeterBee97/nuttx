/****************************************************************************
 * arch/arm/src/ra8p1/ra_serial.c
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

#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/ra8p1_sci.h"
#include "hardware/ra8p1_system.h"
#include "ra_clockconfig.h"
#include "ra_icu.h"
#include "ra_pinmux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_SCI_REG(priv, offset) ((priv)->base + (offset))

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(CONFIG_RA_SCI1_UART) || defined(CONFIG_RA_SCI2_UART)

struct ra_uart_s
{
  uintptr_t base;
  int irqs[3];
  uint32_t rxpin;
  uint32_t txpin;
  uint32_t mstp;
  uint32_t baud;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RA_SCI1_UART
static struct ra_uart_s g_uart1priv =
{
  .base  = RA8P1_SCI1_BASE,
  .irqs  =
    {
      SCI1_RXI,
      SCI1_TXI,
      SCI1_ERI,
    },
  .rxpin = BOARD_SCI1_RX_PIN,
  .txpin = BOARD_SCI1_TX_PIN,
  .mstp  = MSTPCRB_SCI1,
  .baud  = CONFIG_SCI1_BAUD,
};

static char g_uart1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_SCI1_TXBUFSIZE];
#endif

#ifdef CONFIG_RA_SCI2_UART
static struct ra_uart_s g_uart2priv =
{
  .base  = RA8P1_SCI2_BASE,
  .irqs  =
    {
      SCI2_RXI,
      SCI2_TXI,
      SCI2_ERI,
    },
  .rxpin = BOARD_SCI2_RX_PIN,
  .txpin = BOARD_SCI2_TX_PIN,
  .mstp  = MSTPCRB_SCI2,
  .baud  = CONFIG_SCI2_BAUD,
};

static char g_uart2rxbuffer[CONFIG_SCI2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_SCI2_TXBUFSIZE];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ra_rxint(struct uart_dev_s *dev, bool enable)
{
  FAR struct ra_uart_s *priv = dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  modifyreg32(RA_SCI_REG(priv, RA8P1_SCI_CCR0_OFFSET), SCI_CCR0_RIE,
              enable ? SCI_CCR0_RIE : 0);
  leave_critical_section(flags);
}

static bool ra_rxavailable(struct uart_dev_s *dev)
{
  FAR struct ra_uart_s *priv = dev->priv;

  return (getreg32(RA_SCI_REG(priv, RA8P1_SCI_CSR_OFFSET)) &
          SCI_CSR_RDRF) != 0;
}

static int ra_receive(struct uart_dev_s *dev, unsigned int *status)
{
  FAR struct ra_uart_s *priv = dev->priv;

  *status = getreg32(RA_SCI_REG(priv, RA8P1_SCI_CSR_OFFSET));
  return getreg32(RA_SCI_REG(priv, RA8P1_SCI_RDR_OFFSET)) & 0xff;
}

static bool ra_txready(struct uart_dev_s *dev)
{
  FAR struct ra_uart_s *priv = dev->priv;

  return (getreg32(RA_SCI_REG(priv, RA8P1_SCI_CSR_OFFSET)) &
          SCI_CSR_TDRE) != 0;
}

static bool ra_txempty(struct uart_dev_s *dev)
{
  FAR struct ra_uart_s *priv = dev->priv;

  return (getreg32(RA_SCI_REG(priv, RA8P1_SCI_CSR_OFFSET)) &
          SCI_CSR_TEND) != 0;
}

static void ra_send(struct uart_dev_s *dev, int ch)
{
  FAR struct ra_uart_s *priv = dev->priv;

  putreg32(ch & 0xff, RA_SCI_REG(priv, RA8P1_SCI_TDR_OFFSET));
}

static void ra_txint(struct uart_dev_s *dev, bool enable)
{
  FAR struct ra_uart_s *priv = dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  modifyreg32(RA_SCI_REG(priv, RA8P1_SCI_CCR0_OFFSET), SCI_CCR0_TIE,
              enable ? SCI_CCR0_TIE : 0);
  if (enable)
    {
      uart_xmitchars(dev);
    }

  leave_critical_section(flags);
}

static int ra_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = arg;
  FAR struct ra_uart_s *priv = dev->priv;

  ra_clear_ir(irq);
  if (irq == priv->irqs[0])
    {
      uart_recvchars(dev);
    }
  else if (irq == priv->irqs[1])
    {
      uart_xmitchars(dev);
    }
  else
    {
      putreg32(SCI_CFCLR_ORERC | SCI_CFCLR_PERC | SCI_CFCLR_FERC,
               RA_SCI_REG(priv, RA8P1_SCI_CFCLR_OFFSET));
    }

  return OK;
}

static int ra_attach(struct uart_dev_s *dev)
{
  FAR struct ra_uart_s *priv = dev->priv;
  unsigned int i;
  int ret;

  for (i = 0; i < nitems(priv->irqs); i++)
    {
      ret = irq_attach(priv->irqs[i], ra_interrupt, dev);
      if (ret < 0)
        {
          while (i > 0)
            {
              i--;
              up_disable_irq(priv->irqs[i]);
              irq_detach(priv->irqs[i]);
            }

          return ret;
        }

      up_enable_irq(priv->irqs[i]);
    }

  return OK;
}

static void ra_detach(struct uart_dev_s *dev)
{
  FAR struct ra_uart_s *priv = dev->priv;
  unsigned int i;

  for (i = 0; i < nitems(priv->irqs); i++)
    {
      up_disable_irq(priv->irqs[i]);
      irq_detach(priv->irqs[i]);
    }
}

static int ra_setup(struct uart_dev_s *dev)
{
  FAR struct ra_uart_s *priv = dev->priv;
  uint32_t divisor;

  ra_pinmux_configure(priv->rxpin);
  ra_pinmux_configure(priv->txpin);

  ra_module_control(RA8P1_MSTPCRB, priv->mstp, true);

  putreg32(0, RA_SCI_REG(priv, RA8P1_SCI_CCR0_OFFSET));
  putreg32(SCI_CCR1_SPB2DT | SCI_CCR1_SPB2IO,
           RA_SCI_REG(priv, RA8P1_SCI_CCR1_OFFSET));
  putreg32((2u << SCI_CCR3_CHR_SHIFT) | SCI_CCR3_LSBF,
           RA_SCI_REG(priv, RA8P1_SCI_CCR3_OFFSET));

  /* Asynchronous, 16 clocks per bit, no baud modulation. */

  divisor = (RA_SCICLK_FREQUENCY + 8 * priv->baud) /
            (16 * priv->baud) - 1;
  putreg32(SCI_CCR2_BGDM | (divisor << SCI_CCR2_BRR_SHIFT),
           RA_SCI_REG(priv, RA8P1_SCI_CCR2_OFFSET));
  putreg32(SCI_CFCLR_ORERC | SCI_CFCLR_PERC | SCI_CFCLR_FERC,
           RA_SCI_REG(priv, RA8P1_SCI_CFCLR_OFFSET));
  putreg32(SCI_CCR0_RE | SCI_CCR0_TE,
           RA_SCI_REG(priv, RA8P1_SCI_CCR0_OFFSET));
  return OK;
}

static void ra_shutdown(struct uart_dev_s *dev)
{
  FAR struct ra_uart_s *priv = dev->priv;

  putreg32(0, RA_SCI_REG(priv, RA8P1_SCI_CCR0_OFFSET));
  ra_module_control(RA8P1_MSTPCRB, priv->mstp, false);
}

static int ra_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

static const struct uart_ops_s g_uart_ops =
{
  .setup       = ra_setup,
  .shutdown    = ra_shutdown,
  .attach      = ra_attach,
  .detach      = ra_detach,
  .ioctl       = ra_ioctl,
  .receive     = ra_receive,
  .rxint       = ra_rxint,
  .rxavailable = ra_rxavailable,
  .send        = ra_send,
  .txint       = ra_txint,
  .txready     = ra_txready,
  .txempty     = ra_txempty
};

#ifdef CONFIG_RA_SCI1_UART
static uart_dev_t g_uart1 =
{
  .recv =
    {
      .size   = sizeof(g_uart1rxbuffer),
      .buffer = g_uart1rxbuffer,
    },

  .xmit =
    {
      .size   = sizeof(g_uart1txbuffer),
      .buffer = g_uart1txbuffer,
    },

  .ops       = &g_uart_ops,
  .priv      = &g_uart1priv,
#if defined(CONFIG_RA_SCI1_UART) && defined(CONFIG_SCI1_SERIAL_CONSOLE)
  .isconsole = true,
#else
  .isconsole = false,
#endif
};
#endif

#ifdef CONFIG_RA_SCI2_UART
static uart_dev_t g_uart2 =
{
  .recv =
    {
      .size   = sizeof(g_uart2rxbuffer),
      .buffer = g_uart2rxbuffer,
    },

  .xmit =
    {
      .size   = sizeof(g_uart2txbuffer),
      .buffer = g_uart2txbuffer,
    },

  .ops       = &g_uart_ops,
  .priv      = &g_uart2priv,
#if defined(CONFIG_RA_SCI2_UART) && defined(CONFIG_SCI2_SERIAL_CONSOLE)
  .isconsole = true,
#else
  .isconsole = false,
#endif
};
#endif

#if defined(CONFIG_RA_SCI1_UART) && defined(CONFIG_SCI1_SERIAL_CONSOLE)
#  define RA_CONSOLE_DEV g_uart1
#elif defined(CONFIG_RA_SCI2_UART) && defined(CONFIG_SCI2_SERIAL_CONSOLE)
#  define RA_CONSOLE_DEV g_uart2
#endif

#endif /* CONFIG_RA_SCI1_UART || CONFIG_RA_SCI2_UART */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_lowsetup
 ****************************************************************************/

void ra_lowsetup(void)
{
#ifdef RA_CONSOLE_DEV
  ra_setup(&RA_CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: arm_lowputc
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef RA_CONSOLE_DEV
  irqstate_t flags;

  flags = up_irq_save();
  while (!ra_txready(&RA_CONSOLE_DEV))
    {
    }

  ra_send(&RA_CONSOLE_DEV, ch);
  up_irq_restore(flags);
#else
  (void)ch;
#endif
}

/****************************************************************************
 * Name: up_putc
 ****************************************************************************/

void up_putc(int ch)
{
  arm_lowputc(ch);
}

/****************************************************************************
 * Name: arm_earlyserialinit
 ****************************************************************************/

void arm_earlyserialinit(void)
{
#ifdef RA_CONSOLE_DEV
  ra_rxint(&RA_CONSOLE_DEV, false);
  ra_txint(&RA_CONSOLE_DEV, false);
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 ****************************************************************************/

void arm_serialinit(void)
{
#if defined(CONFIG_RA_SCI1_UART) && defined(CONFIG_SCI1_SERIAL_CONSOLE)
  uart_register("/dev/console", &g_uart1);
  uart_register("/dev/ttyS0", &g_uart1);
#ifdef CONFIG_RA_SCI2_UART
  uart_register("/dev/ttyS1", &g_uart2);
#endif
#elif defined(CONFIG_RA_SCI2_UART) && defined(CONFIG_SCI2_SERIAL_CONSOLE)
  uart_register("/dev/console", &g_uart2);
  uart_register("/dev/ttyS0", &g_uart2);
#  ifdef CONFIG_RA_SCI1_UART
  uart_register("/dev/ttyS1", &g_uart1);
#  endif
#else
#  ifdef CONFIG_RA_SCI1_UART
  uart_register("/dev/ttyS0", &g_uart1);
#    ifdef CONFIG_RA_SCI2_UART
  uart_register("/dev/ttyS1", &g_uart2);
#    endif
#  elif defined(CONFIG_RA_SCI2_UART)
  uart_register("/dev/ttyS0", &g_uart2);
#  endif
#endif
}
