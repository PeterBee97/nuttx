/****************************************************************************
 * arch/arm/src/ra8p1/ra_ethernet.c
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

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <debug.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/net/gmii.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "arm_internal.h"
#include "hardware/ra8p1_ethernet.h"
#include "hardware/ra8p1_system.h"
#include "ra_clockconfig.h"
#include "ra_ethernet.h"
#include "ra_icu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_BUFSIZE                    1536
#define RA_MAX_FRAME                  1514
#define RA_MIN_FRAME                  60
#define RA_LINK_TICKS                 MSEC2TICK(500)
#define RA_TX_TIMEOUT                 SEC2TICK(5)
#define RA_REGISTER_TIMEOUT           100000

#define RA_TX_QUEUE                   0
#define RA_RX_QUEUE_BASE              1
#define RA_RX_QUEUES                  2
#define RA_QUEUE_COUNT                3
#define RA_LINK_COUNT                 4
#define RA_TX_DESCRIPTORS             2
#define RA_RX_DESCRIPTORS             4
#define RA_RX_DATA_DESCRIPTORS        3

#define RA_TX_IRQ                     (1u << RA_TX_QUEUE)
#define RA_RX_IRQ(q)                  (1u << (RA_RX_QUEUE_BASE + (q)))
#define RA_QUEUE_IRQ_MASK             ((1u << RA_QUEUE_COUNT) - 1)

#define RA_PHY_ADVERTISE_ALL \
  (MII_ADVERTISE_8023 | MII_ADVERTISE_10BASETXHALF | \
   MII_ADVERTISE_10BASETXFULL | MII_ADVERTISE_100BASETXHALF | \
   MII_ADVERTISE_100BASETXFULL)
#define RA_PHY_1000_CONTROL_ALL \
  (GMII_1000BTCR_1000BASETHALF | GMII_1000BTCR_1000BASETFULL)
#define RA_PHY_1000_STATUS_ALL \
  (GMII_1000BTSR_LP1000BASETHALF | GMII_1000BTSR_LP1000BASETFULL)
#define RA_PHY_LPA_100 \
  (MII_LPA_100BASETXHALF | MII_LPA_100BASETXFULL)
#define RA_PHY_LPA_10 \
  (MII_LPA_10BASETXHALF | MII_LPA_10BASETXFULL)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_desc_s
{
  volatile uint32_t control;
  volatile uint32_t pointer;
  volatile uint32_t info1;
  volatile uint32_t info2;
};

struct ra_link_desc_s
{
  volatile uint32_t control;
  volatile uint32_t pointer;
};

struct ra_eth_s
{
  struct net_driver_s dev;
  struct work_s irqwork;
  struct work_s linkwork;
  FAR const struct ra_eth_config_s *cfg;
  bool ifup;
  bool link;
  bool hwopen;
  bool irqattached;
  volatile bool txbusy;
  volatile int8_t rxcurrent;
  volatile bool rxready[RA_RX_QUEUES];
  volatile uint32_t irqstatus;
  volatile uint32_t errstatus;
  uint8_t linkspeed;
  uint8_t rxpos[RA_RX_QUEUES];
  uint8_t rxscan;
  clock_t txtime;
  struct ra_link_desc_s links[RA_LINK_COUNT] aligned_data(32);
  struct ra_desc_s txdesc[RA_TX_DESCRIPTORS] aligned_data(32);
  struct ra_desc_s rxdesc[RA_RX_QUEUES][RA_RX_DESCRIPTORS]
    aligned_data(32);
  uint8_t rxbuf[RA_RX_QUEUES][RA_RX_DATA_DESCRIPTORS][RA_BUFSIZE]
    aligned_data(32);
  uint8_t pktbuf[RA_BUFSIZE] aligned_data(32);
  uint8_t txbuf[RA_BUFSIZE] aligned_data(32);
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ra_network_work(void *arg);
static void ra_link_work(void *arg);
static int ra_ifdown(struct net_driver_s *dev);
static int ra_ifup(struct net_driver_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ra_eth_s g_eth;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ra_waitreg(uintptr_t addr, uint32_t mask, uint32_t value)
{
  unsigned int timeout;

  for (timeout = 0; timeout < RA_REGISTER_TIMEOUT; timeout++)
    {
      if ((getreg32(addr) & mask) == value)
        {
          return 0;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

static int ra_waitreg8(uintptr_t addr, uint8_t mask, uint8_t value)
{
  unsigned int timeout;

  for (timeout = 0; timeout < RA_REGISTER_TIMEOUT; timeout++)
    {
      if ((getreg8(addr) & mask) == value)
        {
          return 0;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

static uint8_t ra_desc_type(const struct ra_desc_s *desc)
{
  return (desc->control & GWCA_DESC_TYPE_MASK) >>
         GWCA_DESC_TYPE_SHIFT;
}

static void ra_desc_reset(struct ra_desc_s *desc, uint8_t type,
                          uintptr_t buffer, unsigned int size,
                          bool interrupt)
{
  desc->pointer = buffer;
  desc->info1 = 0;
  desc->info2 = 0;
  UP_DSB();
  desc->control = (size & GWCA_DESC_SIZE_MASK) |
                  (interrupt ? GWCA_DESC_INTERRUPT : 0) |
                  ((uint32_t)type << GWCA_DESC_TYPE_SHIFT);
}

static void ra_schedule(struct ra_eth_s *priv)
{
  work_queue(LPWORK, &priv->irqwork, ra_network_work, priv, 0);
}

static int ra_gwca_mode(uint32_t mode)
{
  putreg32(mode, RA8P1_GWCA_GWMC);
  return ra_waitreg(RA8P1_GWCA_GWMS, GWCA_MODE_MASK, mode);
}

static int ra_etha_mode(unsigned int port, uint32_t mode)
{
  int ret;

  putreg32(mode, RA8P1_ETHA_EAMC(port));
  ret = ra_waitreg(RA8P1_ETHA_EAMS(port), ETHA_MODE_MASK, mode);
  if (ret < 0)
    {
      putreg32(RMAC_MIOC_RECOVERY_CLOCK, RA8P1_RMAC_MIOC(port));
      putreg32(mode, RA8P1_ETHA_EAMC(port));
      ret = ra_waitreg(RA8P1_ETHA_EAMS(port), ETHA_MODE_MASK, mode);
      putreg32(0, RA8P1_RMAC_MIOC(port));
    }

  return ret;
}

static int ra_powerup(void)
{
  irqstate_t flags;
  uint8_t power;
  int ret = 0;

  flags = enter_critical_section();
  putreg16(SYSTEM_PRCR_KEY | SYSTEM_PRCR_PRC1 | SYSTEM_PRCR_PRC3,
           RA8P1_SYSTEM_PRCR);

  power = getreg8(RA8P1_SYSTEM_PDCTRESWM);
  if ((power & SYSTEM_PDCTRESWM_STATUS) == SYSTEM_PDCTRESWM_DOWN)
    {
      putreg8(0, RA8P1_SYSTEM_PDCTRESWM);
    }

  putreg16(SYSTEM_PRCR_KEY, RA8P1_SYSTEM_PRCR);
  leave_critical_section(flags);

  if ((power & SYSTEM_PDCTRESWM_STATUS) == SYSTEM_PDCTRESWM_DOWN)
    {
      ret = ra_waitreg8(RA8P1_SYSTEM_PDCTRESWM,
                        SYSTEM_PDCTRESWM_STATUS, 0);
      if (ret < 0)
        {
          return ret;
        }
    }

  ra_module_control(RA8P1_MSTPCRC,
                    MSTPCRC_ETHPHY | MSTPCRC_ESWM, true);
  return 0;
}

static uint32_t ra_mpic(unsigned int speed)
{
  return RMAC_MPIC_RGMII |
         (speed << RMAC_MPIC_SPEED_SHIFT) |
         (RMAC_MPIC_MDC_DIV << RMAC_MPIC_MDC_SHIFT);
}

static int ra_mdio_write(struct ra_eth_s *priv, unsigned int reg,
                         uint16_t data)
{
  uint32_t value;
  uintptr_t addr = RA8P1_RMAC_MPSM(priv->cfg->mdio_port);
  int ret;

  ret = ra_waitreg(addr, RMAC_MPSM_EXECUTE, 0);
  if (ret < 0)
    {
      return ret;
    }

  value = ((uint32_t)priv->cfg->phy_addr << RMAC_MPSM_PHY_SHIFT) |
          (reg << RMAC_MPSM_REG_SHIFT) |
          (RMAC_MPSM_OPCODE_WRITE << RMAC_MPSM_OPCODE_SHIFT) |
          ((uint32_t)data << RMAC_MPSM_DATA_SHIFT);
  putreg32(value, addr);
  putreg32(value | RMAC_MPSM_EXECUTE, addr);
  return ra_waitreg(addr, RMAC_MPSM_EXECUTE, 0);
}

static int ra_mdio_read(struct ra_eth_s *priv, unsigned int reg,
                        uint16_t *data)
{
  uint32_t value;
  uintptr_t addr = RA8P1_RMAC_MPSM(priv->cfg->mdio_port);
  int ret;

  ret = ra_waitreg(addr, RMAC_MPSM_EXECUTE, 0);
  if (ret < 0)
    {
      return ret;
    }

  value = ((uint32_t)priv->cfg->phy_addr << RMAC_MPSM_PHY_SHIFT) |
          (reg << RMAC_MPSM_REG_SHIFT) |
          (RMAC_MPSM_OPCODE_READ << RMAC_MPSM_OPCODE_SHIFT);
  putreg32(value, addr);
  putreg32(value | RMAC_MPSM_EXECUTE, addr);
  ret = ra_waitreg(addr, RMAC_MPSM_EXECUTE, 0);
  if (ret == 0)
    {
      *data = (getreg32(addr) >> RMAC_MPSM_DATA_SHIFT) &
              RMAC_MPSM_DATA_MASK;
    }

  return ret;
}

static int ra_phy_initialize(struct ra_eth_s *priv)
{
  uint16_t value;
  unsigned int timeout;
  int ret;

  ret = ra_mdio_write(priv, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      return ret;
    }

  for (timeout = 0; timeout < 1000; timeout++)
    {
      ret = ra_mdio_read(priv, MII_MCR, &value);
      if (ret < 0 || (value & MII_MCR_RESET) == 0)
        {
          break;
        }

      up_udelay(100);
    }

  if (ret < 0)
    {
      return ret;
    }

  if ((value & MII_MCR_RESET) != 0)
    {
      return -ETIMEDOUT;
    }

  ret = ra_mdio_write(priv, MII_ADVERTISE, RA_PHY_ADVERTISE_ALL);
  if (ret == 0)
    {
      ret = ra_mdio_write(priv, GMII_1000BTCR,
                          RA_PHY_1000_CONTROL_ALL);
    }

  if (ret == 0)
    {
      ret = ra_mdio_write(priv, MII_MCR,
                          MII_MCR_ANENABLE | MII_MCR_ANRESTART);
    }

  return ret;
}

static int ra_phy_link(struct ra_eth_s *priv)
{
  uint16_t status;
  uint16_t ability;
  int ret;

  ret = ra_mdio_read(priv, MII_MSR, &status);
  if (ret == 0)
    {
      ret = ra_mdio_read(priv, MII_MSR, &status);
    }

  if (ret < 0)
    {
      return ret;
    }

  if ((status & MII_MSR_LINKSTATUS) == 0)
    {
      return -ENETDOWN;
    }

  if ((status & MII_MSR_ANEGCOMPLETE) == 0)
    {
      return -EAGAIN;
    }

  ret = ra_mdio_read(priv, GMII_1000BTSR, &ability);
  if (ret < 0)
    {
      return ret;
    }

  if ((ability & RA_PHY_1000_STATUS_ALL) != 0)
    {
      return 2;
    }

  ret = ra_mdio_read(priv, MII_LPA, &ability);
  if (ret < 0)
    {
      return ret;
    }

  if ((ability & RA_PHY_LPA_100) != 0)
    {
      return 1;
    }

  return (ability & RA_PHY_LPA_10) != 0 ? 0 : -ENOLINK;
}

static void ra_set_mac(struct ra_eth_s *priv, unsigned int port)
{
  const uint8_t *mac = priv->dev.d_mac.ether.ether_addr_octet;

  putreg32(((uint32_t)mac[0] << 8) | mac[1],
           RA8P1_RMAC_MRMAC0(port));
  putreg32(((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
           ((uint32_t)mac[4] << 8) | mac[5],
           RA8P1_RMAC_MRMAC1(port));
}

static void ra_forward(struct ra_eth_s *priv, bool enable)
{
  uintptr_t addr = RA8P1_MFWD_FWPBFC(priv->cfg->port);
  uint32_t value = getreg32(addr) & ~MFWD_FWPBFC_DEST_MASK;

  if (enable)
    {
      value |= MFWD_FWPBFC_CPU;
    }

  putreg32(value, addr);
}

static void ra_initialize_descriptors(struct ra_eth_s *priv)
{
  unsigned int i;
  unsigned int q;

  for (i = 0; i < RA_TX_DESCRIPTORS; i++)
    {
      ra_desc_reset(&priv->txdesc[i], GWCA_DESC_TYPE_LEMPTY, 0, 0,
                    false);
    }

  for (q = 0; q < RA_RX_QUEUES; q++)
    {
      for (i = 0; i < RA_RX_DATA_DESCRIPTORS; i++)
        {
          ra_desc_reset(&priv->rxdesc[q][i], GWCA_DESC_TYPE_FEMPTY,
                        (uintptr_t)priv->rxbuf[q][i], RA_MAX_FRAME, true);
        }

      ra_desc_reset(&priv->rxdesc[q][RA_RX_DATA_DESCRIPTORS],
                    GWCA_DESC_TYPE_LEMPTY, 0, 0, false);
      priv->rxpos[q] = 0;
      priv->rxready[q] = true;
    }

  priv->rxcurrent = -1;
  priv->rxscan = 0;
  UP_DSB();
}

static void ra_initialize_linkfix(struct ra_eth_s *priv)
{
  unsigned int i;

  for (i = 0; i < RA_LINK_COUNT; i++)
    {
      priv->links[i].control =
        GWCA_DESC_TYPE_LEMPTY << GWCA_DESC_TYPE_SHIFT;
      priv->links[i].pointer = 0;
    }

  priv->links[RA_TX_QUEUE].pointer = (uintptr_t)priv->txdesc;
  priv->links[RA_TX_QUEUE].control =
    GWCA_DESC_TYPE_LINKFIX << GWCA_DESC_TYPE_SHIFT;
  priv->links[RA_RX_QUEUE_BASE].pointer =
    (uintptr_t)priv->rxdesc[0];
  priv->links[RA_RX_QUEUE_BASE].control =
    GWCA_DESC_TYPE_LINKFIX << GWCA_DESC_TYPE_SHIFT;
  priv->links[RA_RX_QUEUE_BASE + 1].pointer =
    (uintptr_t)priv->rxdesc[1];
  priv->links[RA_RX_QUEUE_BASE + 1].control =
    GWCA_DESC_TYPE_LINKFIX << GWCA_DESC_TYPE_SHIFT;
  UP_DSB();

  putreg32(0, RA8P1_GWCA_GWDCBAC0);
  putreg32((uintptr_t)priv->links, RA8P1_GWCA_GWDCBAC1);
}

static int ra_reload_queue(unsigned int queue)
{
  modifyreg32(RA8P1_GWCA_GWDCC(queue), 0, GWCA_GWDCC_RELOAD);
  return ra_waitreg(RA8P1_GWCA_GWDCC(queue), GWCA_GWDCC_RELOAD, 0);
}

static int ra_start_rx(struct ra_eth_s *priv, unsigned int q)
{
  unsigned int queue = RA_RX_QUEUE_BASE + q;
  int ret;

  priv->rxready[q] = false;
  ret = ra_reload_queue(queue);
  if (ret == 0)
    {
      putreg32(queue & MFWD_FWPBFCSDC_MASK,
               RA8P1_MFWD_FWPBFCSDC(priv->cfg->port));
      priv->rxcurrent = q;
    }
  else
    {
      priv->rxready[q] = true;
    }

  return ret;
}

static void ra_switch_rx(struct ra_eth_s *priv)
{
  int current = priv->rxcurrent;
  int next;

  if (current < 0 ||
      ra_desc_type(&priv->rxdesc[current]
                   [RA_RX_DATA_DESCRIPTORS - 1]) ==
      GWCA_DESC_TYPE_FEMPTY)
    {
      return;
    }

  next = current ^ 1;
  priv->rxcurrent = -1;
  if (priv->rxready[next] && ra_start_rx(priv, next) == 0)
    {
      return;
    }

  priv->errstatus |= RA_RX_IRQ(current);
}

static int ra_interrupt(int irq, void *context, void *arg)
{
  struct ra_eth_s *priv = arg;
  uint32_t status;
  uint32_t errors;

  status = getreg32(RA8P1_GWCA_GWDIS0) & RA_QUEUE_IRQ_MASK;
  errors = getreg32(RA8P1_GWCA_GWEIS20) & RA_QUEUE_IRQ_MASK;
  if (status != 0)
    {
      putreg32(status, RA8P1_GWCA_GWDIS0);
      priv->irqstatus |= status;
    }

  if (errors != 0)
    {
      putreg32(errors, RA8P1_GWCA_GWEIS20);
      priv->errstatus |= errors;
    }

  if ((status | errors) & (RA_RX_IRQ(0) | RA_RX_IRQ(1)))
    {
      ra_switch_rx(priv);
    }

  ra_clear_ir(irq);
  if (priv->ifup)
    {
      ra_schedule(priv);
    }

  return 0;
}

static int ra_create_queues(struct ra_eth_s *priv)
{
  uint32_t status;
  int ret;

  putreg32(GWCA_GWDCC_EXTENDED | GWCA_GWDCC_TX_QUEUE,
           RA8P1_GWCA_GWDCC(RA_TX_QUEUE));
  putreg32(GWCA_GWDCC_EXTENDED,
           RA8P1_GWCA_GWDCC(RA_RX_QUEUE_BASE));
  putreg32(GWCA_GWDCC_EXTENDED,
           RA8P1_GWCA_GWDCC(RA_RX_QUEUE_BASE + 1));

  status = getreg32(RA8P1_GWCA_GWDIS0) & RA_QUEUE_IRQ_MASK;
  putreg32(status, RA8P1_GWCA_GWDIS0);
  status = getreg32(RA8P1_GWCA_GWEIS20) & RA_QUEUE_IRQ_MASK;
  putreg32(status, RA8P1_GWCA_GWEIS20);
  modifyreg32(RA8P1_GWCA_GWDIE0, 0, RA_QUEUE_IRQ_MASK);
  modifyreg32(RA8P1_GWCA_GWEIE20, 0,
              RA_RX_IRQ(0) | RA_RX_IRQ(1));

  ra_initialize_descriptors(priv);
  if (!priv->irqattached)
    {
      ret = irq_attach(RA_IRQ_GWDI0, ra_interrupt, priv);
      if (ret < 0)
        {
          return ret;
        }

      priv->irqattached = true;
    }

#ifdef CONFIG_ARCH_IRQPRIO
  up_prioritize_irq(RA_IRQ_GWDI0, 12 << 4);
#endif

  ra_clear_ir(RA_IRQ_GWDI0);
  up_enable_irq(RA_IRQ_GWDI0);
  return 0;
}

static int ra_switch_initialize(struct ra_eth_s *priv)
{
  uint32_t media_reset = 0;
  unsigned int port;
  unsigned int q;
  int ret;

  putreg32(COMA_RRC_RESET, RA8P1_COMA_RRC);
  putreg32(0, RA8P1_COMA_RRC);
  up_udelay(1000);
  putreg32(COMA_RCEC_ENABLE, RA8P1_COMA_RCEC);
  up_udelay(1000);
  putreg32(COMA_CABPIRM_INITIALIZE, RA8P1_COMA_CABPIRM);
  ret = ra_waitreg(RA8P1_COMA_CABPIRM, COMA_CABPIRM_READY,
                   COMA_CABPIRM_READY);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(COMA_RCEC_ENABLE | COMA_RCEC_AGENT_ALL, RA8P1_COMA_RCEC);

  for (port = 0; port < 3; port++)
    {
      putreg32(0, RA8P1_MFWD_FWPBFC(port));
      modifyreg32(RA8P1_MFWD_FWPC1(port), 0, MFWD_FWPC1_DIRECT);
    }

  ret = ra_gwca_mode(GWCA_MODE_DISABLE);
  if (ret == 0)
    {
      ret = ra_gwca_mode(GWCA_MODE_CONFIG);
    }

  if (ret < 0)
    {
      return ret;
    }

  putreg32(GWCA_GWARIRM_INITIALIZE, RA8P1_GWCA_GWARIRM);
  ret = ra_waitreg(RA8P1_GWCA_GWARIRM, GWCA_GWARIRM_READY,
                   GWCA_GWARIRM_READY);
  if (ret < 0)
    {
      return ret;
    }

  ra_initialize_linkfix(priv);
  ret = ra_gwca_mode(GWCA_MODE_DISABLE);
  if (ret == 0)
    {
      ret = ra_gwca_mode(GWCA_MODE_OPERATION);
    }

  if (ret < 0)
    {
      return ret;
    }

  for (port = 0; port < 2; port++)
    {
      if ((priv->cfg->etha_mask & (1u << port)) == 0)
        {
          continue;
        }

      ret = ra_etha_mode(port, ETHA_MODE_DISABLE);
      if (ret == 0)
        {
          ret = ra_etha_mode(port, ETHA_MODE_CONFIG);
        }

      if (ret < 0)
        {
          return ret;
        }

      ra_set_mac(priv, port);
      for (q = 0; q < 8; q++)
        {
          putreg32(64 & ETHA_QUEUE_DEPTH_MASK,
                   RA8P1_ETHA_EATDQDC(port, q));
        }

      putreg32(0, RA8P1_ETHA_EATPEC(port));
      modifyreg32(RA8P1_RMAC_MRGC(port), 0, RMAC_MRGC_MAGIC_PACKET);
      putreg32(ra_mpic(2), RA8P1_RMAC_MPIC(port));
    }

  modifyreg8(RA8P1_PMISC_PFENET,
             PMISC_PFENET_PHYMODE(0) | PMISC_PFENET_PHYMODE(1), 0);
  for (port = 0; port < 2; port++)
    {
      if ((priv->cfg->etha_mask & (1u << port)) == 0)
        {
          continue;
        }

      ret = ra_etha_mode(port, ETHA_MODE_DISABLE);
      if (ret == 0)
        {
          ret = ra_etha_mode(port, ETHA_MODE_OPERATION);
        }

      if (ret < 0)
        {
          return ret;
        }

      putreg32(ESWM_MIICR_RGMII | ESWM_MIICR_TXC_DELAY,
               RA8P1_ESWM_MIICR(port));
      media_reset |= ESWM_MIIRR_RGMII_RESET(port);
    }

  modifyreg32(RA8P1_ESWM_MIIRR, 0, media_reset);
  ret = ra_phy_initialize(priv);
  if (ret < 0)
    {
      return ret;
    }

  return ra_create_queues(priv);
}

static int ra_hw_open(struct ra_eth_s *priv)
{
  int ret;

  ret = ra_powerup();
  if (ret == 0)
    {
      ret = ra_switch_initialize(priv);
    }

  if (ret < 0)
    {
      nerr("RMAC initialization failed: %d\n", ret);
      ra_module_control(RA8P1_MSTPCRC,
                        MSTPCRC_ETHPHY | MSTPCRC_ESWM, false);
      return ret;
    }

  priv->irqstatus = 0;
  priv->errstatus = 0;
  priv->hwopen = true;
  return 0;
}

static void ra_hw_close(struct ra_eth_s *priv)
{
  unsigned int port;

  up_disable_irq(RA_IRQ_GWDI0);
  putreg32(RA_QUEUE_IRQ_MASK, RA8P1_GWCA_GWDID0);
  putreg32(RA_QUEUE_IRQ_MASK, RA8P1_GWCA_GWEID20);
  ra_forward(priv, false);

  for (port = 0; port < 2; port++)
    {
      if ((priv->cfg->etha_mask & (1u << port)) != 0)
        {
          putreg32(RMAC_MIOC_RECOVERY_CLOCK, RA8P1_RMAC_MIOC(port));
          (void)ra_etha_mode(port, ETHA_MODE_DISABLE);
        }
    }

  (void)ra_gwca_mode(GWCA_MODE_DISABLE);
  for (port = 0; port < 2; port++)
    {
      putreg32(0, RA8P1_RMAC_MIOC(port));
    }

  priv->hwopen = false;
  ra_module_control(RA8P1_MSTPCRC,
                    MSTPCRC_ETHPHY | MSTPCRC_ESWM, false);
}

static int ra_link_up(struct ra_eth_s *priv, unsigned int speed)
{
  unsigned int port = priv->cfg->port;
  int ret;

  ret = ra_etha_mode(port, ETHA_MODE_DISABLE);
  if (ret == 0)
    {
      ret = ra_etha_mode(port, ETHA_MODE_CONFIG);
    }

  if (ret < 0)
    {
      return ret;
    }

  putreg32(ra_mpic(speed), RA8P1_RMAC_MPIC(port));
  ra_set_mac(priv, port);
  putreg32(RMAC_MRAFC_MULTICAST_E | RMAC_MRAFC_BROADCAST_E |
           RMAC_MRAFC_MULTICAST_P | RMAC_MRAFC_BROADCAST_P,
           RA8P1_RMAC_MRAFC(port));

  ret = ra_etha_mode(port, ETHA_MODE_DISABLE);
  if (ret == 0)
    {
      ret = ra_etha_mode(port, ETHA_MODE_OPERATION);
    }

  if (ret < 0)
    {
      return ret;
    }

  ra_initialize_descriptors(priv);
  ra_forward(priv, true);
  ret = ra_start_rx(priv, 0);
  if (ret < 0)
    {
      ra_forward(priv, false);
      return ret;
    }

  priv->linkspeed = speed;
  priv->txbusy = false;
  priv->link = true;
  netdev_carrier_on(&priv->dev);
  ninfo("RMAC link up, %u Mbps\n",
        speed == 2 ? 1000 : speed == 1 ? 100 : 10);
  return 0;
}

static void ra_link_down(struct ra_eth_s *priv)
{
  if (priv->link)
    {
      ninfo("RMAC link down\n");
    }

  ra_forward(priv, false);
  priv->link = false;
  priv->txbusy = false;
  priv->rxcurrent = -1;
  netdev_carrier_off(&priv->dev);
}

static int ra_transmit(struct ra_eth_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  struct ra_desc_s *desc = &priv->txdesc[0];
  unsigned int len = dev->d_len;
  int ret;

  DEBUGASSERT(!priv->txbusy);
  if (len > RA_MAX_FRAME)
    {
      NETDEV_TXERRORS(dev);
      dev->d_len = 0;
      return -EMSGSIZE;
    }

  memcpy(priv->txbuf, dev->d_buf, len);
  if (len < RA_MIN_FRAME)
    {
      memset(priv->txbuf + len, 0, RA_MIN_FRAME - len);
      len = RA_MIN_FRAME;
    }

  ra_desc_reset(&priv->txdesc[1], GWCA_DESC_TYPE_LEMPTY, 0, 0,
                false);
  desc->pointer = (uintptr_t)priv->txbuf;
  desc->info1 = GWCA_DESC_INFO1_DIRECT;
  desc->info2 = GWCA_DESC_INFO2_DV(priv->cfg->port);
  UP_DSB();
  desc->control = (len & GWCA_DESC_SIZE_MASK) | GWCA_DESC_INTERRUPT |
                  (GWCA_DESC_TYPE_FSINGLE << GWCA_DESC_TYPE_SHIFT);
  UP_DSB();

  priv->txbusy = true;
  priv->txtime = clock_systime_ticks();
  ret = ra_reload_queue(RA_TX_QUEUE);
  if (ret == 0)
    {
      modifyreg32(RA8P1_GWCA_GWTRC0, 0, RA_TX_IRQ);
      NETDEV_TXPACKETS(dev);
    }
  else
    {
      priv->txbusy = false;
      NETDEV_TXERRORS(dev);
      ra_desc_reset(desc, GWCA_DESC_TYPE_LEMPTY, 0, 0, false);
    }

  dev->d_len = 0;
  return ret;
}

static int ra_txpoll(struct net_driver_s *dev)
{
  struct ra_eth_s *priv = dev->d_private;

  if (dev->d_len > 0)
    {
      ra_transmit(priv);
    }

  return priv->txbusy ? -EBUSY : 0;
}

static void ra_finish_rx_queue(struct ra_eth_s *priv, unsigned int q)
{
  irqstate_t flags;

  if (priv->rxpos[q] != RA_RX_DATA_DESCRIPTORS)
    {
      return;
    }

  flags = enter_critical_section();
  ra_switch_rx(priv);
  if (priv->rxcurrent != (int)q)
    {
      priv->rxpos[q] = 0;
      priv->rxready[q] = true;
      UP_DSB();
      if (priv->rxcurrent < 0 && priv->link)
        {
          (void)ra_start_rx(priv, q);
        }
    }

  leave_critical_section(flags);
}

static void ra_dispatch(struct ra_eth_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  struct eth_hdr_s *hdr = (struct eth_hdr_s *)dev->d_buf;

  NETDEV_RXPACKETS(dev);
#ifdef CONFIG_NET_PKT
  pkt_input(dev);
#endif
#ifdef CONFIG_NET_IPv4
  if (hdr->type == HTONS(ETHTYPE_IP))
    {
      NETDEV_RXIPV4(dev);
      ipv4_input(dev);
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (hdr->type == HTONS(ETHTYPE_IP6))
    {
      NETDEV_RXIPV6(dev);
      ipv6_input(dev);
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  if (hdr->type == HTONS(ETHTYPE_ARP))
    {
      NETDEV_RXARP(dev);
      arp_input(dev);
    }
  else
#endif
    {
      NETDEV_RXDROPPED(dev);
      dev->d_len = 0;
    }

  if (dev->d_len > 0)
    {
      ra_transmit(priv);
    }
}

static void ra_receive(struct ra_eth_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  struct ra_desc_s *desc;
  uint32_t control;
  unsigned int len;
  unsigned int q;
  unsigned int pos;
  unsigned int count;
  bool found;

  for (count = 0; count < 8 && !priv->txbusy; count++)
    {
      found = false;
      for (unsigned int scan = 0; scan < RA_RX_QUEUES; scan++)
        {
          q = (priv->rxscan + scan) % RA_RX_QUEUES;
          pos = priv->rxpos[q];
          if (pos >= RA_RX_DATA_DESCRIPTORS)
            {
              ra_finish_rx_queue(priv, q);
              continue;
            }

          desc = &priv->rxdesc[q][pos];
          if (ra_desc_type(desc) == GWCA_DESC_TYPE_FEMPTY ||
              ra_desc_type(desc) == GWCA_DESC_TYPE_LEMPTY)
            {
              continue;
            }

          found = true;
          control = desc->control;
          len = control & GWCA_DESC_SIZE_MASK;
          if ((control & GWCA_DESC_ERROR_MASK) == 0 &&
              len >= sizeof(struct eth_hdr_s) && len <= RA_MAX_FRAME)
            {
              memcpy(dev->d_buf, priv->rxbuf[q][pos], len);
              dev->d_len = len;
            }
          else
            {
              NETDEV_RXERRORS(dev);
              dev->d_len = 0;
            }

          if (pos == RA_RX_DATA_DESCRIPTORS - 1 &&
              priv->rxcurrent == (int)q)
            {
              irqstate_t flags = enter_critical_section();

              ra_switch_rx(priv);
              leave_critical_section(flags);
            }

          ra_desc_reset(desc, GWCA_DESC_TYPE_FEMPTY,
                        (uintptr_t)priv->rxbuf[q][pos], RA_MAX_FRAME, true);
          priv->rxpos[q]++;
          priv->rxscan = q ^ 1;
          ra_finish_rx_queue(priv, q);

          if (dev->d_len > 0)
            {
              ra_dispatch(priv);
            }

          break;
        }

      if (!found)
        {
          break;
        }
    }
}

static void ra_network_work(void *arg)
{
  struct ra_eth_s *priv = arg;
  irqstate_t flags;
  uint32_t status;
  uint32_t errors;

  net_lock();
  flags = enter_critical_section();
  status = priv->irqstatus;
  errors = priv->errstatus;
  priv->irqstatus = 0;
  priv->errstatus = 0;
  leave_critical_section(flags);

  if (priv->ifup && priv->link)
    {
      if ((errors & (RA_RX_IRQ(0) | RA_RX_IRQ(1))) != 0)
        {
          NETDEV_RXERRORS(&priv->dev);
        }

      if ((status & RA_TX_IRQ) != 0 && priv->txbusy)
        {
          priv->txbusy = false;
          ra_desc_reset(&priv->txdesc[0], GWCA_DESC_TYPE_LEMPTY,
                        0, 0, false);
          NETDEV_TXDONE(&priv->dev);
        }

      ra_receive(priv);
      if (!priv->txbusy)
        {
          devif_poll(&priv->dev, ra_txpoll);
        }
    }

  net_unlock();
  if (priv->irqstatus != 0 || priv->errstatus != 0)
    {
      ra_schedule(priv);
    }
}

static void ra_link_work(void *arg)
{
  struct ra_eth_s *priv = arg;
  int speed;

  net_lock();
  if (priv->ifup)
    {
      speed = ra_phy_link(priv);
      if (speed >= 0)
        {
          if (!priv->link)
            {
              if (ra_link_up(priv, speed) < 0)
                {
                  ra_link_down(priv);
                }
            }
          else if (priv->linkspeed != speed)
            {
              putreg32(ra_mpic(speed),
                       RA8P1_RMAC_MPIC(priv->cfg->port));
              priv->linkspeed = speed;
            }
        }
      else if (speed != -EAGAIN && priv->link)
        {
          ra_link_down(priv);
        }

      if (priv->txbusy &&
          clock_systime_ticks() - priv->txtime > RA_TX_TIMEOUT)
        {
          NETDEV_TXTIMEOUTS(&priv->dev);
          nerr("RMAC TX timeout\n");
          priv->txbusy = false;
          ra_desc_reset(&priv->txdesc[0], GWCA_DESC_TYPE_LEMPTY,
                        0, 0, false);
          ra_link_down(priv);
        }

      ra_schedule(priv);
      work_queue(LPWORK, &priv->linkwork, ra_link_work, priv,
                 RA_LINK_TICKS);
    }

  net_unlock();
}

static int ra_ifup(struct net_driver_s *dev)
{
  struct ra_eth_s *priv = dev->d_private;
  unsigned int i;
  int speed;
  int ret;

  if (priv->ifup)
    {
      return 0;
    }

  ret = ra_hw_open(priv);
  if (ret < 0)
    {
      return ret;
    }

  priv->ifup = true;
  priv->link = false;
  priv->txbusy = false;
  netdev_carrier_off(dev);

  /* Permit a bounded initial negotiation so DHCP can send immediately. */

  for (i = 0; i < 50; i++)
    {
      speed = ra_phy_link(priv);
      if (speed >= 0)
        {
          ret = ra_link_up(priv, speed);
          break;
        }

      nxsig_usleep(100000);
    }

  if (ret < 0)
    {
      priv->ifup = false;
      ra_hw_close(priv);
      return ret;
    }

  work_queue(LPWORK, &priv->linkwork, ra_link_work, priv, 0);
  return 0;
}

static int ra_ifdown(struct net_driver_s *dev)
{
  struct ra_eth_s *priv = dev->d_private;

  priv->ifup = false;
  work_cancel(LPWORK, &priv->irqwork);
  work_cancel(LPWORK, &priv->linkwork);
  ra_link_down(priv);
  if (priv->hwopen)
    {
      ra_hw_close(priv);
    }

  return 0;
}

static int ra_txavail(struct net_driver_s *dev)
{
  struct ra_eth_s *priv = dev->d_private;

  ra_schedule(priv);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_netinitialize
 ****************************************************************************/

void arm_netinitialize(void)
{
  struct ra_eth_s *priv = &g_eth;
  int ret;

  priv->cfg = ra_board_ethernet_config();
  if (priv->cfg == NULL)
    {
      nerr("ERROR: Missing RA8P1 Ethernet board configuration\n");
      return;
    }

  priv->dev.d_buf = priv->pktbuf;
  priv->dev.d_ifup = ra_ifup;
  priv->dev.d_ifdown = ra_ifdown;
  priv->dev.d_txavail = ra_txavail;
  priv->dev.d_private = priv;
  memcpy(priv->dev.d_mac.ether.ether_addr_octet, priv->cfg->mac,
         IFHWADDRLEN);
  ret = netdev_register(&priv->dev, NET_LL_ETHERNET);
  if (ret < 0)
    {
      nerr("ERROR: Failed to register RA8P1 Ethernet: %d\n", ret);
    }
}
