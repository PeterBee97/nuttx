/****************************************************************************
 * arch/arm/src/ra8p1/ra_canfd.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/can/can.h>
#include <nuttx/can/can_common.h>
#include <nuttx/irq.h>

#include <debug.h>

#include <arch/board/board.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "hardware/ra8p1_canfd.h"
#include "hardware/ra8p1_system.h"
#include "ra_canfd.h"
#include "ra_clockconfig.h"
#include "ra_icu.h"

#ifdef CONFIG_RA8P1_CANFD0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_CANFD_NOMINAL_TSEG1_MAX 256
#define RA_CANFD_NOMINAL_TSEG2_MAX 128
#define RA_CANFD_NOMINAL_BRP_MAX   1024
#define RA_CANFD_DATA_TSEG1_MAX    32
#define RA_CANFD_DATA_TSEG2_MAX    16
#define RA_CANFD_DATA_BRP_MAX      256
#define RA_CANFD_TIMEOUT_US        10000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_canfd_timing_s
{
  uint16_t prescaler;
  uint16_t tseg1;
  uint8_t tseg2;
  uint8_t sjw;
};

struct ra_canfd_dev_s
{
  struct can_dev_s dev;
  struct ra_canfd_timing_s nominal;
  struct ra_canfd_timing_s data;
  volatile bool txpending;
  bool opened;
  bool loopback;
  bool silent;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ra_canfd_reset(struct can_dev_s *dev);
static int ra_canfd_setup(struct can_dev_s *dev);
static void ra_canfd_shutdown(struct can_dev_s *dev);
static void ra_canfd_rxint(struct can_dev_s *dev, bool enable);
static void ra_canfd_txint(struct can_dev_s *dev, bool enable);
static int ra_canfd_ioctl(struct can_dev_s *dev, int cmd,
                          unsigned long arg);
static int ra_canfd_send(struct can_dev_s *dev, struct can_msg_s *msg);
static bool ra_canfd_txready(struct can_dev_s *dev);
static bool ra_canfd_txempty(struct can_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_canfd_ops =
{
  .co_reset         = ra_canfd_reset,
  .co_setup         = ra_canfd_setup,
  .co_shutdown      = ra_canfd_shutdown,
  .co_rxint         = ra_canfd_rxint,
  .co_txint         = ra_canfd_txint,
  .co_ioctl         = ra_canfd_ioctl,
  .co_remoterequest = NULL,
  .co_send          = ra_canfd_send,
  .co_txready       = ra_canfd_txready,
  .co_txempty       = ra_canfd_txempty,
};

static struct ra_canfd_dev_s g_canfd0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ra_canfd_wait(uintptr_t reg, uint32_t mask, uint32_t expected)
{
  unsigned int timeout;

  for (timeout = 0; timeout < RA_CANFD_TIMEOUT_US; timeout++)
    {
      if ((getreg32(reg) & mask) == expected)
        {
          return OK;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

static int ra_canfd_global_mode(uint32_t mode)
{
  modifyreg32(RA8P1_CANFD_GCTR, CANFD_MODE_MASK, mode);
  return ra_canfd_wait(RA8P1_CANFD_GSTS, CANFD_MODE_MASK, mode);
}

static int ra_canfd_channel_mode(uint32_t mode)
{
  modifyreg32(RA8P1_CANFD_CTR, CANFD_MODE_MASK, mode);
  return ra_canfd_wait(RA8P1_CANFD_STS, CANFD_MODE_MASK, mode);
}

static int ra_canfd_make_timing(struct ra_canfd_timing_s *timing,
                                uint32_t bitrate, bool data)
{
  uint32_t tseg1_max;
  uint32_t tseg2_max;
  uint32_t prescaler_max;
  uint32_t prescaler;

  tseg1_max = data ? RA_CANFD_DATA_TSEG1_MAX :
                     RA_CANFD_NOMINAL_TSEG1_MAX;
  tseg2_max = data ? RA_CANFD_DATA_TSEG2_MAX :
                     RA_CANFD_NOMINAL_TSEG2_MAX;
  prescaler_max = data ? RA_CANFD_DATA_BRP_MAX :
                         RA_CANFD_NOMINAL_BRP_MAX;

  for (prescaler = 1; prescaler <= prescaler_max; prescaler++)
    {
      uint64_t denominator = (uint64_t)bitrate * prescaler;
      uint32_t tseg1;
      uint32_t tseg2;
      uint32_t total;

      if (denominator == 0 ||
          BOARD_CANFD_FREQUENCY % denominator != 0)
        {
          continue;
        }

      total = BOARD_CANFD_FREQUENCY / denominator;
      if (total < 3 || total > tseg1_max + tseg2_max + 1)
        {
          continue;
        }

      /* Prefer the 75 percent sample point used by the board SDK while
       * keeping both segments inside the controller limits.
       */

      tseg1 = (total * 3) / 4 - 1;
      tseg2 = total - tseg1 - 1;
      if (tseg1 < 1 || tseg1 > tseg1_max ||
          tseg2 < 1 || tseg2 > tseg2_max ||
          (data ? tseg1 < tseg2 : tseg1 <= tseg2))
        {
          continue;
        }

      timing->prescaler = prescaler;
      timing->tseg1 = tseg1;
      timing->tseg2 = tseg2;
      timing->sjw = tseg2 < 4 ? tseg2 : 4;
      return OK;
    }

  return -EINVAL;
}

static uint32_t ra_canfd_test_mode(struct ra_canfd_dev_s *priv)
{
  if (priv->loopback)
    {
      return CANFD_TEST_LOOPBACK_INTERNAL;
    }

  if (priv->silent)
    {
      return CANFD_TEST_LISTEN;
    }

  return CANFD_TEST_DISABLED;
}

static void ra_canfd_filter(bool loopback)
{
  putreg32(CANFD_GAFLECTR_ACCESS, RA8P1_CANFD_GAFLECTR);
  putreg32(loopback ? CANFD_GAFL_ID_LOOPBACK : 0,
           RA8P1_CANFD_GAFL_ID(0));
  putreg32(0, RA8P1_CANFD_GAFL_MASK(0));
  putreg32(0, RA8P1_CANFD_GAFL_P0(0));
  putreg32(CANFD_GAFL_P1_FIFO0, RA8P1_CANFD_GAFL_P1(0));
  putreg32(0, RA8P1_CANFD_GAFLECTR);
}

static int ra_canfd_apply_mode(struct ra_canfd_dev_s *priv)
{
  uint32_t mode = ra_canfd_test_mode(priv);
  int ret;

  ret = ra_canfd_channel_mode(CANFD_MODE_HALT);
  if (ret < 0)
    {
      return ret;
    }

  ra_canfd_filter(priv->loopback);
  modifyreg32(RA8P1_CANFD_CTR, 7u << CANFD_CTR_TEST_SHIFT,
              mode << CANFD_CTR_TEST_SHIFT);
  return ra_canfd_channel_mode(CANFD_MODE_OPERATION);
}

static void ra_canfd_module(bool enable)
{
  ra_module_control(RA8P1_MSTPCRC, MSTPCRC_CANFD0, enable);
}

static int ra_canfd_configure(struct ra_canfd_dev_s *priv)
{
  uint32_t value;
  int ret;

  ret = ra_canfd_wait(RA8P1_CANFD_GSTS, CANFD_GSTS_RAMINIT, 0);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(0, RA8P1_CANFD_RFCC(0));
  putreg32(0, RA8P1_CANFD_RFCC(1));
  ret = ra_canfd_global_mode(CANFD_MODE_RESET);
  if (ret < 0)
    {
      return ret;
    }

  ret = ra_canfd_channel_mode(CANFD_MODE_RESET);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(1, RA8P1_CANFD_GCFG);
  putreg32(CANFD_GAFLCFG0_RULES(1), RA8P1_CANFD_GAFLCFG0);
  putreg32(0, RA8P1_CANFD_GFDCFG);
  putreg32(0, RA8P1_CANFD_RMNB);
  putreg32(CANFD_RFCC_FIFO0_CONFIG, RA8P1_CANFD_RFCC(0));

  value = CANFD_NCFG_BRP(priv->nominal.prescaler) |
          CANFD_NCFG_SJW(priv->nominal.sjw) |
          CANFD_NCFG_TSEG1(priv->nominal.tseg1) |
          CANFD_NCFG_TSEG2(priv->nominal.tseg2);
  putreg32(value, RA8P1_CANFD_NCFG);

  value = CANFD_DCFG_BRP(priv->data.prescaler) |
          CANFD_DCFG_SJW(priv->data.sjw) |
          CANFD_DCFG_TSEG1(priv->data.tseg1) |
          CANFD_DCFG_TSEG2(priv->data.tseg2);
  putreg32(value, RA8P1_CANFD_DCFG);
  putreg32(CANFD_FDCFG_TDCO(priv->data.tseg1) |
           CANFD_FDCFG_TDCE | CANFD_FDCFG_ESIC | 1,
           RA8P1_CANFD_FDCFG);

  putreg32(1, RA8P1_CANFD_TMIEC);
#ifdef CONFIG_CAN_ERRORS
  modifyreg32(RA8P1_CANFD_CTR, 0, CANFD_CTR_ERROR_IRQS);
  modifyreg32(RA8P1_CANFD_GCTR, 0, (1u << 8) | (1u << 9) |
              (1u << 11));
#endif

  ret = ra_canfd_global_mode(CANFD_MODE_OPERATION);
  if (ret < 0)
    {
      return ret;
    }

  modifyreg32(RA8P1_CANFD_RFCC(0), 0, CANFD_RFCC_ENABLE);
  ret = ra_canfd_apply_mode(priv);
  if (ret == OK)
    {
      caninfo("GCTR=%08" PRIx32 " GSTS=%08" PRIx32
              " CTR=%08" PRIx32 " STS=%08" PRIx32
              " RFCC=%08" PRIx32 " GAFLID=%08" PRIx32 "\n",
              getreg32(RA8P1_CANFD_GCTR),
              getreg32(RA8P1_CANFD_GSTS),
              getreg32(RA8P1_CANFD_CTR),
              getreg32(RA8P1_CANFD_STS),
              getreg32(RA8P1_CANFD_RFCC(0)),
              getreg32(RA8P1_CANFD_GAFL_ID(0)));
    }

  return ret;
}

#ifdef CONFIG_CAN_ERRORS
static void ra_canfd_error(struct ra_canfd_dev_s *priv, uint32_t error,
                           bool global)
{
  struct can_hdr_s hdr;
  uint8_t data[CAN_ERROR_DLC];
  uint32_t status;
  uint8_t rec;
  uint8_t tec;

  memset(&hdr, 0, sizeof(hdr));
  memset(data, 0, sizeof(data));
  status = getreg32(RA8P1_CANFD_STS);
  rec = status >> CANFD_STS_REC_SHIFT;
  tec = status >> CANFD_STS_TEC_SHIFT;

  if (global)
    {
      if ((error & (CANFD_GERFL_MESSAGE_LOST |
                    CANFD_GERFL_PAYLOAD_OVERFLOW)) != 0)
        {
          hdr.ch_id |= CAN_ERROR_INTERNAL;
          data[5] |= CAN_ERROR5_RXOVERFLOW;
        }

      if ((error & (CANFD_GERFL_DLC | CANFD_GERFL_ECC)) != 0)
        {
          hdr.ch_id |= CAN_ERROR_INTERNAL;
        }
    }
  else
    {
      if ((error & CANFD_ERFL_WARNING) != 0)
        {
          hdr.ch_id |= CAN_ERROR_CONTROLLER;
          data[1] |= rec >= 96 ? CAN_ERROR1_RXWARNING : 0;
          data[1] |= tec >= 96 ? CAN_ERROR1_TXWARNING : 0;
        }

      if ((error & CANFD_ERFL_PASSIVE) != 0)
        {
          hdr.ch_id |= CAN_ERROR_CONTROLLER;
          data[1] |= rec >= 128 ? CAN_ERROR1_RXPASSIVE : 0;
          data[1] |= tec >= 128 ? CAN_ERROR1_TXPASSIVE : 0;
        }

      if ((error & CANFD_ERFL_BUSOFF) != 0)
        {
          hdr.ch_id |= CAN_ERROR_BUSOFF;
        }

      if ((error & CANFD_ERFL_RECOVERY) != 0)
        {
          hdr.ch_id |= CAN_ERROR_RESTARTED;
        }

      if ((error & CANFD_ERFL_ARBLOST) != 0)
        {
          hdr.ch_id |= CAN_ERROR_LOSTARB;
        }

      if ((error & CANFD_ERFL_ACK) != 0)
        {
          hdr.ch_id |= CAN_ERROR_NOACK | CAN_ERROR_PROTOCOL;
          data[2] |= CAN_ERROR2_TX;
          data[3] = CAN_ERROR3_ACK;
        }

      if ((error & (CANFD_ERFL_BUS | CANFD_ERFL_BUSLOCK)) != 0)
        {
          hdr.ch_id |= CAN_ERROR_BUSERROR;
        }

      if ((error & (CANFD_ERFL_STUFF | CANFD_ERFL_FORM |
                    CANFD_ERFL_CRC | CANFD_ERFL_BIT0 |
                    CANFD_ERFL_BIT1 | CANFD_ERFL_ACKDEL |
                    CANFD_ERFL_OVERLOAD)) != 0)
        {
          hdr.ch_id |= CAN_ERROR_PROTOCOL;
          data[2] |= (error & CANFD_ERFL_STUFF) ?
                     CAN_ERROR2_STUFF : 0;
          data[2] |= (error & CANFD_ERFL_FORM) ?
                     CAN_ERROR2_FORM : 0;
          data[2] |= (error & CANFD_ERFL_BIT0) ?
                     CAN_ERROR2_BIT0 : 0;
          data[2] |= (error & CANFD_ERFL_BIT1) ?
                     CAN_ERROR2_BIT1 : 0;
          data[2] |= (error & CANFD_ERFL_OVERLOAD) ?
                     CAN_ERROR2_OVERLOAD : 0;
        }
    }

  if (hdr.ch_id != 0)
    {
      hdr.ch_dlc = can_bytes2dlc(CAN_ERROR_DLC);
      hdr.ch_error = 1;
      (void)can_receive(&priv->dev, &hdr, data);
    }
}
#endif

static int ra_canfd_rxinterrupt(int irq, void *context, void *arg)
{
  struct ra_canfd_dev_s *priv = arg;

  caninfo("RX irq RFSTS=%08" PRIx32 "\n",
          getreg32(RA8P1_CANFD_RFSTS(0)));
  while ((getreg32(RA8P1_CANFD_RFSTS(0)) & CANFD_RFSTS_EMPTY) == 0)
    {
      struct can_hdr_s hdr;
      uint8_t data[CAN_MAXDATALEN];
      uint32_t id;
      uint32_t fdsts;
      uint8_t dlc;
      uint8_t bytes;
      uint8_t i;

      memset(&hdr, 0, sizeof(hdr));
      id = getreg32(RA8P1_CANFD_RF_ID(0));
      dlc = getreg32(RA8P1_CANFD_RF_PTR(0)) >> CANFD_DLC_SHIFT;
      fdsts = getreg32(RA8P1_CANFD_RF_FDSTS(0));
      bytes = can_dlc2bytes(dlc & 15);

      hdr.ch_id = id & CANFD_ID_MASK;
      hdr.ch_rtr = (id & CANFD_ID_RTR) != 0;
#ifdef CONFIG_CAN_EXTID
      hdr.ch_extid = (id & CANFD_ID_EXTENDED) != 0;
#endif
#ifdef CONFIG_CAN_FD
      hdr.ch_edl = (fdsts & CANFD_FD_FRAME) != 0;
      hdr.ch_brs = (fdsts & CANFD_FD_BRS) != 0;
      hdr.ch_esi = (fdsts & CANFD_FD_ESI) != 0;
#endif
      hdr.ch_dlc = dlc;
      for (i = 0; i < bytes; i++)
        {
          data[i] = getreg8(RA8P1_CANFD_RF_DATA(0) + i);
        }

      putreg32(0xff, RA8P1_CANFD_RFPCTR(0));
      (void)can_receive(&priv->dev, &hdr, data);
    }

#ifdef CONFIG_CAN_ERRORS
  if ((getreg32(RA8P1_CANFD_RFSTS(0)) &
       CANFD_RFSTS_MESSAGE_LOST) != 0)
    {
      ra_canfd_error(priv, CANFD_GERFL_MESSAGE_LOST, true);
      modifyreg32(RA8P1_CANFD_RFSTS(0), CANFD_RFSTS_MESSAGE_LOST, 0);
    }
#endif

  modifyreg32(RA8P1_CANFD_RFSTS(0), CANFD_RFSTS_IRQ, 0);
  ra_clear_ir(irq);
  return OK;
}

static int ra_canfd_txinterrupt(int irq, void *context, void *arg)
{
  struct ra_canfd_dev_s *priv = arg;
  uint32_t status;

  status = getreg32(RA8P1_CANFD_GTINTSTS0);
  caninfo("TX irq status=%08" PRIx32 " TMSTS=%02x\n",
          status, getreg8(RA8P1_CANFD_TMSTS(0)));
  if ((status & (CANFD_GTINT_TX_COMPLETE | CANFD_GTINT_TX_ABORT)) != 0)
    {
      modifyreg8(RA8P1_CANFD_TMSTS(0), CANFD_TMSTS_RESULT_MASK, 0);
      priv->txpending = false;
#ifdef CONFIG_CAN_ERRORS
      if ((status & CANFD_GTINT_TX_ABORT) != 0)
        {
          struct can_hdr_s hdr;
          uint8_t data[CAN_ERROR_DLC];

          memset(&hdr, 0, sizeof(hdr));
          memset(data, 0, sizeof(data));
          hdr.ch_id = CAN_ERROR_TXTIMEOUT;
          hdr.ch_dlc = can_bytes2dlc(CAN_ERROR_DLC);
          hdr.ch_error = 1;
          (void)can_receive(&priv->dev, &hdr, data);
        }
#endif

      can_txdone(&priv->dev);
    }

  ra_clear_ir(irq);
  return OK;
}

#ifdef CONFIG_CAN_ERRORS
static int ra_canfd_cherrinterrupt(int irq, void *context, void *arg)
{
  struct ra_canfd_dev_s *priv = arg;
  uint32_t error = getreg32(RA8P1_CANFD_ERFL) & CANFD_ERFL_ALL;

  putreg32(0, RA8P1_CANFD_ERFL);
  ra_canfd_error(priv, error, false);
  ra_clear_ir(irq);
  return OK;
}

static int ra_canfd_glerrinterrupt(int irq, void *context, void *arg)
{
  struct ra_canfd_dev_s *priv = arg;
  uint32_t error = getreg32(RA8P1_CANFD_GERFL);

  putreg32(0, RA8P1_CANFD_GERFL);
  ra_canfd_error(priv, error, true);
  ra_clear_ir(irq);
  return OK;
}
#endif

static void ra_canfd_reset(struct can_dev_s *dev)
{
  struct ra_canfd_dev_s *priv = dev->cd_priv;

  if (priv->opened)
    {
      (void)ra_canfd_channel_mode(CANFD_MODE_RESET);
      priv->txpending = false;
    }
}

static int ra_canfd_setup(struct can_dev_s *dev)
{
  struct ra_canfd_dev_s *priv = dev->cd_priv;
  int ret;

  if (priv->opened)
    {
      return -EBUSY;
    }

  ra_canfd_module(true);
  caninfo("module started MSTPCRC=%08" PRIx32 "\n",
          getreg32(RA8P1_MSTPCRC));
  ret = irq_attach(RA_IRQ_CAN_RXF, ra_canfd_rxinterrupt, priv);
  if (ret < 0)
    {
      goto errout_module;
    }

  ret = irq_attach(RA_IRQ_CAN0_TX, ra_canfd_txinterrupt, priv);
  if (ret < 0)
    {
      goto errout_rx;
    }

#ifdef CONFIG_CAN_ERRORS
  ret = irq_attach(RA_IRQ_CAN0_CHERR, ra_canfd_cherrinterrupt, priv);
  if (ret < 0)
    {
      goto errout_tx;
    }

  ret = irq_attach(RA_IRQ_CAN_GLERR, ra_canfd_glerrinterrupt, priv);
  if (ret < 0)
    {
      goto errout_cherr;
    }
#endif

  ret = ra_canfd_configure(priv);
  if (ret < 0)
    {
      canerr("configuration failed: %d\n", ret);
      goto errout_irqs;
    }

  priv->opened = true;
  priv->txpending = false;
  ra_canfd_rxint(dev, false);
  ra_canfd_txint(dev, false);
#ifdef CONFIG_CAN_ERRORS
  up_enable_irq(RA_IRQ_CAN0_CHERR);
  up_enable_irq(RA_IRQ_CAN_GLERR);
#endif
  return OK;

errout_irqs:
#ifdef CONFIG_CAN_ERRORS
  irq_detach(RA_IRQ_CAN_GLERR);
errout_cherr:
  irq_detach(RA_IRQ_CAN0_CHERR);
errout_tx:
#endif
  irq_detach(RA_IRQ_CAN0_TX);
errout_rx:
  irq_detach(RA_IRQ_CAN_RXF);
errout_module:
  ra_canfd_module(false);
  return ret;
}

static void ra_canfd_shutdown(struct can_dev_s *dev)
{
  struct ra_canfd_dev_s *priv = dev->cd_priv;

  if (!priv->opened)
    {
      return;
    }

  up_disable_irq(RA_IRQ_CAN_RXF);
  up_disable_irq(RA_IRQ_CAN0_TX);
#ifdef CONFIG_CAN_ERRORS
  up_disable_irq(RA_IRQ_CAN0_CHERR);
  up_disable_irq(RA_IRQ_CAN_GLERR);
#endif
  (void)ra_canfd_channel_mode(CANFD_MODE_RESET);
  putreg32(0, RA8P1_CANFD_RFCC(0));
  (void)ra_canfd_global_mode(CANFD_MODE_RESET);
  irq_detach(RA_IRQ_CAN_RXF);
  irq_detach(RA_IRQ_CAN0_TX);
#ifdef CONFIG_CAN_ERRORS
  irq_detach(RA_IRQ_CAN0_CHERR);
  irq_detach(RA_IRQ_CAN_GLERR);
#endif
  ra_canfd_module(false);
  priv->opened = false;
  priv->txpending = false;
}

static void ra_canfd_rxint(struct can_dev_s *dev, bool enable)
{
  (void)dev;
  if (enable)
    {
      up_enable_irq(RA_IRQ_CAN_RXF);
    }
  else
    {
      up_disable_irq(RA_IRQ_CAN_RXF);
    }
}

static void ra_canfd_txint(struct can_dev_s *dev, bool enable)
{
  (void)dev;
  if (enable)
    {
      up_enable_irq(RA_IRQ_CAN0_TX);
    }
  else
    {
      up_disable_irq(RA_IRQ_CAN0_TX);
    }
}

static int ra_canfd_set_timing(struct ra_canfd_dev_s *priv,
                               const struct canioc_bittiming_s *bt)
{
  struct ra_canfd_timing_s *timing;
  struct ra_canfd_timing_s previous;
  uint32_t prescaler_max;
  uint32_t tseg1_max;
  uint32_t tseg2_max;
  uint64_t denominator;
  uint32_t prescaler;
  bool data = false;
  int ret;

#ifdef CONFIG_CAN_FD
  data = bt->type == CAN_BITTIMING_DATA;
#endif
  if (priv->txpending)
    {
      return -EBUSY;
    }

  timing = data ? &priv->data : &priv->nominal;
  prescaler_max = data ? RA_CANFD_DATA_BRP_MAX :
                         RA_CANFD_NOMINAL_BRP_MAX;
  tseg1_max = data ? RA_CANFD_DATA_TSEG1_MAX :
                     RA_CANFD_NOMINAL_TSEG1_MAX;
  tseg2_max = data ? RA_CANFD_DATA_TSEG2_MAX :
                     RA_CANFD_NOMINAL_TSEG2_MAX;
  denominator = (uint64_t)bt->bt_baud *
                (bt->bt_tseg1 + bt->bt_tseg2 + 1);

  if (denominator == 0 ||
      BOARD_CANFD_FREQUENCY % denominator != 0)
    {
      return -EINVAL;
    }

  prescaler = BOARD_CANFD_FREQUENCY / denominator;
  if (prescaler < 1 || prescaler > prescaler_max ||
      bt->bt_tseg1 < 1 || bt->bt_tseg1 > tseg1_max ||
      bt->bt_tseg2 < 1 || bt->bt_tseg2 > tseg2_max ||
      bt->bt_sjw < 1 || bt->bt_sjw > bt->bt_tseg2 ||
      (data ? bt->bt_tseg1 < bt->bt_tseg2 :
              bt->bt_tseg1 <= bt->bt_tseg2))
    {
      return -EINVAL;
    }

  previous = *timing;
  timing->prescaler = prescaler;
  timing->tseg1 = bt->bt_tseg1;
  timing->tseg2 = bt->bt_tseg2;
  timing->sjw = bt->bt_sjw;

  ret = ra_canfd_configure(priv);
  if (ret < 0)
    {
      *timing = previous;
      (void)ra_canfd_configure(priv);
    }

  return ret;
}

static int ra_canfd_ioctl(struct can_dev_s *dev, int cmd,
                          unsigned long arg)
{
  struct ra_canfd_dev_s *priv = dev->cd_priv;

  switch (cmd)
    {
      case CANIOC_GET_BITTIMING:
        {
          struct canioc_bittiming_s *bt =
            (struct canioc_bittiming_s *)(uintptr_t)arg;
          struct ra_canfd_timing_s *timing;
          bool data = false;

          if (bt == NULL)
            {
              return -EINVAL;
            }

#ifdef CONFIG_CAN_FD
          data = bt->type == CAN_BITTIMING_DATA;
#endif
          timing = data ? &priv->data : &priv->nominal;
          bt->bt_baud = BOARD_CANFD_FREQUENCY / timing->prescaler /
                        (timing->tseg1 + timing->tseg2 + 1);
          bt->bt_tseg1 = timing->tseg1;
          bt->bt_tseg2 = timing->tseg2;
          bt->bt_sjw = timing->sjw;
          return OK;
        }

      case CANIOC_SET_BITTIMING:
        {
          const struct canioc_bittiming_s *bt =
            (const struct canioc_bittiming_s *)(uintptr_t)arg;

          return bt == NULL ? -EINVAL : ra_canfd_set_timing(priv, bt);
        }

      case CANIOC_GET_CONNMODES:
        {
          struct canioc_connmodes_s *modes =
            (struct canioc_connmodes_s *)(uintptr_t)arg;

          if (modes == NULL)
            {
              return -EINVAL;
            }

          modes->bm_loopback = priv->loopback;
          modes->bm_silent = priv->silent;
          return OK;
        }

      case CANIOC_SET_CONNMODES:
        {
          const struct canioc_connmodes_s *modes =
            (const struct canioc_connmodes_s *)(uintptr_t)arg;
          bool old_loopback;
          bool old_silent;
          int ret;

          if (modes == NULL ||
              (modes->bm_loopback && modes->bm_silent))
            {
              return -EINVAL;
            }

          if (priv->txpending)
            {
              return -EBUSY;
            }

          old_loopback = priv->loopback;
          old_silent = priv->silent;
          priv->loopback = modes->bm_loopback;
          priv->silent = modes->bm_silent;
          ret = ra_canfd_apply_mode(priv);
          if (ret < 0)
            {
              priv->loopback = old_loopback;
              priv->silent = old_silent;
              (void)ra_canfd_apply_mode(priv);
            }

          return ret;
        }

      case CANIOC_BUSOFF_RECOVERY:
        if ((getreg32(RA8P1_CANFD_STS) & CANFD_STS_BUSOFF) == 0)
          {
            return -EINVAL;
          }

        modifyreg32(RA8P1_CANFD_CTR, 0, CANFD_CTR_RETURN_BUSOFF);
        return OK;

      case CANIOC_GET_STATE:
        {
          int *state = (int *)(uintptr_t)arg;

          if (state == NULL)
            {
              return -EINVAL;
            }

          *state = priv->opened ? CAN_STATE_START : CAN_STATE_STOP;
          return OK;
        }

      default:
        return -ENOTTY;
    }
}

static int ra_canfd_send(struct can_dev_s *dev, struct can_msg_s *msg)
{
  struct ra_canfd_dev_s *priv = dev->cd_priv;
  uint32_t id;
  uint32_t fdctr = 0;
  uint8_t nbytes;
  uint8_t dlc;
  uint8_t i;

  if (!priv->opened || priv->silent)
    {
      return -EPERM;
    }

  if (priv->txpending ||
      (getreg8(RA8P1_CANFD_TMSTS(0)) & CANFD_TMSTS_REQUEST) != 0)
    {
      return -EBUSY;
    }

  nbytes = can_dlc2bytes(msg->cm_hdr.ch_dlc);
  dlc = can_bytes2dlc(nbytes);

#ifdef CONFIG_CAN_FD
  if (msg->cm_hdr.ch_edl)
    {
      fdctr |= CANFD_FD_FRAME;
      fdctr |= msg->cm_hdr.ch_brs ? CANFD_FD_BRS : 0;
      fdctr |= msg->cm_hdr.ch_esi ? CANFD_FD_ESI : 0;
    }
  else
#endif
    {
      if (nbytes > 8)
        {
          return -EINVAL;
        }
    }

  if (msg->cm_hdr.ch_rtr && fdctr != 0)
    {
      return -EINVAL;
    }

  id = msg->cm_hdr.ch_id & CANFD_ID_MASK;
  id |= msg->cm_hdr.ch_rtr ? CANFD_ID_RTR : 0;
#ifdef CONFIG_CAN_EXTID
  id |= msg->cm_hdr.ch_extid ? CANFD_ID_EXTENDED : 0;
#endif

  putreg32(id, RA8P1_CANFD_TM_ID(0));
  putreg32((uint32_t)dlc << CANFD_DLC_SHIFT, RA8P1_CANFD_TM_PTR(0));
  putreg32(fdctr, RA8P1_CANFD_TM_FDCTR(0));
  for (i = 0; i < nbytes; i++)
    {
      putreg8(msg->cm_data[i], RA8P1_CANFD_TM_DATA(0) + i);
    }

  priv->txpending = true;
  putreg8(1, RA8P1_CANFD_TMC(0));
  caninfo("send id=%08" PRIx32 " dlc=%u fd=%08" PRIx32
          " TMC=%02x TMSTS=%02x\n",
          id, dlc, fdctr, getreg8(RA8P1_CANFD_TMC(0)),
          getreg8(RA8P1_CANFD_TMSTS(0)));
  return OK;
}

static bool ra_canfd_txready(struct can_dev_s *dev)
{
  struct ra_canfd_dev_s *priv = dev->cd_priv;

  return priv->opened && !priv->silent && !priv->txpending &&
         (getreg8(RA8P1_CANFD_TMSTS(0)) & CANFD_TMSTS_REQUEST) == 0;
}

static bool ra_canfd_txempty(struct can_dev_s *dev)
{
  struct ra_canfd_dev_s *priv = dev->cd_priv;

  return !priv->txpending &&
         (getreg8(RA8P1_CANFD_TMSTS(0)) & CANFD_TMSTS_REQUEST) == 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_canfd_initialize
 ****************************************************************************/

struct can_dev_s *ra_canfd_initialize(int port)
{
  int ret;

  if (port != 0)
    {
      return NULL;
    }

  ret = ra_canfd_make_timing(&g_canfd0.nominal,
                             CONFIG_RA8P1_CANFD0_BITRATE, false);
  if (ret < 0)
    {
      return NULL;
    }

  ret = ra_canfd_make_timing(&g_canfd0.data,
                             CONFIG_RA8P1_CANFD0_DATA_BITRATE, true);
  if (ret < 0)
    {
      return NULL;
    }

  g_canfd0.dev.cd_ops = &g_canfd_ops;
  g_canfd0.dev.cd_priv = &g_canfd0;
#ifdef CONFIG_CAN_LOOPBACK
  g_canfd0.loopback = true;
#endif
  return &g_canfd0.dev;
}

#endif /* CONFIG_RA8P1_CANFD0 */
