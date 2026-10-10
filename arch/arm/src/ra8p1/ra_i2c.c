/****************************************************************************
 * arch/arm/src/ra8p1/ra_i2c.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/clock.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "hardware/ra8p1_i2c.h"
#include "hardware/ra8p1_system.h"
#include "ra_clockconfig.h"
#include "ra_i2c.h"

#ifdef CONFIG_RA8P1_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_I2C_TIMEOUT_TICKS      MSEC2TICK(100)
#define RA_I2C_DEFAULT_FREQUENCY  I2C_SPEED_FAST
#define RA_I2C_SUPPORTED_FLAGS    (I2C_M_READ | I2C_M_NOSTOP | \
                                   I2C_M_NOSTART)
#define RA_I2C_REG(priv, reg)     ((priv)->base + \
                                   RA8P1_IIC_ ## reg ## _OFFSET)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum ra_i2c_end_e
{
  RA_I2C_END_STOP,
  RA_I2C_END_RESTART,
  RA_I2C_END_HOLD
};

struct ra_i2c_priv_s
{
  struct i2c_master_s dev;
  mutex_t lock;
  uintptr_t base;
  uint32_t mstp;
  uint32_t frequency;
  int refs;
  bool held;
  bool restart_pending;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra_i2c_transfer(FAR struct i2c_master_s *dev,
                           FAR struct i2c_msg_s *msgs, int count);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_ra_i2c_ops =
{
  .transfer = ra_i2c_transfer,
};

#ifdef CONFIG_RA8P1_I2C0
static struct ra_i2c_priv_s g_ra_i2c0 =
{
  .dev =
    {
      .ops = &g_ra_i2c_ops,
    },
  .lock = NXMUTEX_INITIALIZER,
  .base = RA8P1_IIC0_BASE,
  .mstp = MSTPCRB_IIC0,
};
#endif

#ifdef CONFIG_RA8P1_I2C1
static struct ra_i2c_priv_s g_ra_i2c1 =
{
  .dev =
    {
      .ops = &g_ra_i2c_ops,
    },
  .lock = NXMUTEX_INITIALIZER,
  .base = RA8P1_IIC1_BASE,
  .mstp = MSTPCRB_IIC1,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ra_i2c_clear_status(FAR struct ra_i2c_priv_s *priv,
                                uint8_t mask)
{
  uint8_t status = getreg8(RA_I2C_REG(priv, ICSR2));

  putreg8(status & ~mask, RA_I2C_REG(priv, ICSR2));
}

static int ra_i2c_status_error(uint8_t status)
{
  if ((status & IIC_ICSR2_TMOF) != 0)
    {
      return -ETIMEDOUT;
    }

  if ((status & IIC_ICSR2_AL) != 0)
    {
      return -EAGAIN;
    }

  if ((status & IIC_ICSR2_NACKF) != 0)
    {
      return -ENXIO;
    }

  return OK;
}

static int ra_i2c_wait_status(FAR struct ra_i2c_priv_s *priv,
                              uint8_t mask, bool check_errors)
{
  clock_t start = clock_systime_ticks();

  for (; ; )
    {
      uint8_t status = getreg8(RA_I2C_REG(priv, ICSR2));
      int ret;

      if (check_errors)
        {
          ret = ra_i2c_status_error(status);
          if (ret < 0)
            {
              return ret;
            }
        }

      if ((status & mask) != 0)
        {
          return OK;
        }

      if (clock_systime_ticks() - start >= RA_I2C_TIMEOUT_TICKS)
        {
          return -ETIMEDOUT;
        }
    }
}

static int ra_i2c_wait_bus_free(FAR struct ra_i2c_priv_s *priv)
{
  clock_t start = clock_systime_ticks();

  while ((getreg8(RA_I2C_REG(priv, ICCR2)) & IIC_ICCR2_BBSY) != 0)
    {
      if (clock_systime_ticks() - start >= RA_I2C_TIMEOUT_TICKS)
        {
          return -EBUSY;
        }
    }

  return OK;
}

static int ra_i2c_setfrequency(FAR struct ra_i2c_priv_s *priv,
                               uint32_t frequency)
{
  uint8_t cks;

  if (frequency == I2C_SPEED_STANDARD)
    {
      cks = 4;
    }
  else if (frequency == I2C_SPEED_FAST)
    {
      cks = 2;
    }
  else
    {
      return -EINVAL;
    }

  putreg8(0, RA_I2C_REG(priv, ICCR1));
  putreg8(IIC_ICCR1_IICRST, RA_I2C_REG(priv, ICCR1));
  putreg8(IIC_ICCR1_ICE | IIC_ICCR1_IICRST,
          RA_I2C_REG(priv, ICCR1));

  putreg8(IIC_ICBR_RESERVED | 15, RA_I2C_REG(priv, ICBRL));
  putreg8(IIC_ICBR_RESERVED | 15, RA_I2C_REG(priv, ICBRH));
  putreg8(IIC_ICMR1_BCWP | (cks << IIC_ICMR1_CKS_SHIFT),
          RA_I2C_REG(priv, ICMR1));
  putreg8(IIC_ICMR2_TMOH | IIC_ICMR2_TMOL | IIC_ICMR2_TMOS,
          RA_I2C_REG(priv, ICMR2));
  putreg8(0, RA_I2C_REG(priv, ICMR3));
  putreg8(IIC_ICFER_MASTER_INIT, RA_I2C_REG(priv, ICFER));
  putreg8(0, RA_I2C_REG(priv, ICSER));
  putreg8(0, RA_I2C_REG(priv, ICIER));

  putreg8(IIC_ICCR1_ICE | IIC_ICCR1_OUTPUT_HIGH,
          RA_I2C_REG(priv, ICCR1));

  priv->frequency = frequency;
  priv->held = false;
  priv->restart_pending = false;
  return OK;
}

static void ra_i2c_set_nack(FAR struct ra_i2c_priv_s *priv, bool nack)
{
  modifyreg8(RA_I2C_REG(priv, ICMR3), 0, IIC_ICMR3_ACKWP);
  modifyreg8(RA_I2C_REG(priv, ICMR3), IIC_ICMR3_ACKBT,
             nack ? IIC_ICMR3_ACKBT : 0);
  modifyreg8(RA_I2C_REG(priv, ICMR3), IIC_ICMR3_ACKWP, 0);
}

static int ra_i2c_start(FAR struct ra_i2c_priv_s *priv, bool restart)
{
  int ret;

  if (!restart)
    {
      ret = ra_i2c_wait_bus_free(priv);
      if (ret < 0)
        {
          return ret;
        }
    }

  ra_i2c_clear_status(priv, IIC_ICSR2_EVENTS);
  putreg8(restart ? IIC_ICCR2_RS : IIC_ICCR2_ST,
          RA_I2C_REG(priv, ICCR2));

  ret = ra_i2c_wait_status(priv, IIC_ICSR2_START, true);
  if (ret >= 0)
    {
      ra_i2c_clear_status(priv, IIC_ICSR2_START);
      priv->held = true;
      priv->restart_pending = false;
    }

  return ret;
}

static int ra_i2c_send_address(FAR struct ra_i2c_priv_s *priv,
                               FAR const struct i2c_msg_s *msg)
{
  int ret;

  ret = ra_i2c_wait_status(priv, IIC_ICSR2_TDRE, true);
  if (ret < 0)
    {
      return ret;
    }

  putreg8(I2C_ADDR8(msg->addr) |
          ((msg->flags & I2C_M_READ) != 0 ? I2C_READBIT : 0),
          RA_I2C_REG(priv, ICDRT));
  return OK;
}

static int ra_i2c_stop(FAR struct ra_i2c_priv_s *priv)
{
  int ret;

  ra_i2c_clear_status(priv, IIC_ICSR2_EVENTS);
  putreg8(IIC_ICCR2_SP, RA_I2C_REG(priv, ICCR2));

  ret = ra_i2c_wait_status(priv, IIC_ICSR2_STOP, false);
  if (ret >= 0)
    {
      ret = ra_i2c_wait_bus_free(priv);
    }

  ra_i2c_clear_status(priv, IIC_ICSR2_EVENTS);
  priv->held = false;
  priv->restart_pending = false;
  return ret;
}

static int ra_i2c_restart(FAR struct ra_i2c_priv_s *priv)
{
  int ret;

  ret = ra_i2c_start(priv, true);
  if (ret >= 0)
    {
      priv->restart_pending = true;
    }

  return ret;
}

static void ra_i2c_abort(FAR struct ra_i2c_priv_s *priv)
{
  if ((getreg8(RA_I2C_REG(priv, ICCR2)) & IIC_ICCR2_MST) != 0)
    {
      ra_i2c_stop(priv);
    }

  ra_i2c_setfrequency(priv, priv->frequency);
}

static int ra_i2c_finish(FAR struct ra_i2c_priv_s *priv,
                         enum ra_i2c_end_e end)
{
  if (end == RA_I2C_END_STOP)
    {
      return ra_i2c_stop(priv);
    }

  if (end == RA_I2C_END_RESTART)
    {
      return ra_i2c_restart(priv);
    }

  priv->held = true;
  priv->restart_pending = false;
  return OK;
}

static int ra_i2c_write_msg(FAR struct ra_i2c_priv_s *priv,
                            FAR const struct i2c_msg_s *msg,
                            bool address, enum ra_i2c_end_e end)
{
  ssize_t i;
  int ret;

  if (address)
    {
      ret = ra_i2c_send_address(priv, msg);
      if (ret < 0)
        {
          return ret;
        }
    }

  for (i = 0; i < msg->length; i++)
    {
      ret = ra_i2c_wait_status(priv, IIC_ICSR2_TDRE, true);
      if (ret < 0)
        {
          return ret;
        }

      putreg8(msg->buffer[i], RA_I2C_REG(priv, ICDRT));
    }

  ret = ra_i2c_wait_status(priv, IIC_ICSR2_TEND, true);
  if (ret < 0)
    {
      return ret;
    }

  return ra_i2c_finish(priv, end);
}

static int ra_i2c_read_msg(FAR struct ra_i2c_priv_s *priv,
                           FAR const struct i2c_msg_s *msg,
                           enum ra_i2c_end_e end)
{
  ssize_t remain = msg->length;
  ssize_t offset = 0;
  int ret;

  DEBUGASSERT(end != RA_I2C_END_HOLD);

  ret = ra_i2c_send_address(priv, msg);
  if (ret < 0)
    {
      return ret;
    }

  if (remain <= 2)
    {
      modifyreg8(RA_I2C_REG(priv, ICMR3), 0, IIC_ICMR3_WAIT);
    }

  if (remain == 1)
    {
      ra_i2c_set_nack(priv, true);
    }

  ret = ra_i2c_wait_status(priv, IIC_ICSR2_RDRF, true);
  if (ret < 0)
    {
      return ret;
    }

  getreg8(RA_I2C_REG(priv, ICDRR));

  while (remain > 0)
    {
      ret = ra_i2c_wait_status(priv, IIC_ICSR2_RDRF, true);
      if (ret < 0)
        {
          return ret;
        }

      if (remain == 3)
        {
          modifyreg8(RA_I2C_REG(priv, ICMR3), 0, IIC_ICMR3_WAIT);
        }
      else if (remain == 2)
        {
          ra_i2c_set_nack(priv, true);
        }
      else if (remain == 1)
        {
          if (end == RA_I2C_END_STOP)
            {
              ra_i2c_clear_status(priv, IIC_ICSR2_STOP);
              putreg8(IIC_ICCR2_SP, RA_I2C_REG(priv, ICCR2));
            }
          else
            {
              ra_i2c_set_nack(priv, false);
              ra_i2c_clear_status(priv, IIC_ICSR2_START);
              putreg8(IIC_ICCR2_RS, RA_I2C_REG(priv, ICCR2));
            }
        }

      msg->buffer[offset++] = getreg8(RA_I2C_REG(priv, ICDRR));
      remain--;
    }

  modifyreg8(RA_I2C_REG(priv, ICMR3), IIC_ICMR3_WAIT, 0);

  if (end == RA_I2C_END_STOP)
    {
      ret = ra_i2c_wait_status(priv, IIC_ICSR2_STOP, false);
      if (ret >= 0)
        {
          ret = ra_i2c_wait_bus_free(priv);
        }

      ra_i2c_clear_status(priv, IIC_ICSR2_EVENTS);
      priv->held = false;
      priv->restart_pending = false;
    }
  else
    {
      ret = ra_i2c_wait_status(priv, IIC_ICSR2_START, true);
      if (ret >= 0)
        {
          ra_i2c_clear_status(priv, IIC_ICSR2_START);
          priv->held = true;
          priv->restart_pending = true;
        }
    }

  ra_i2c_set_nack(priv, false);
  return ret;
}

static int ra_i2c_validate(FAR struct ra_i2c_priv_s *priv,
                           FAR const struct i2c_msg_s *msgs, int count)
{
  int i;

  if (msgs == NULL || count <= 0)
    {
      return -EINVAL;
    }

  for (i = 0; i < count; i++)
    {
      FAR const struct i2c_msg_s *msg = &msgs[i];

      if ((msg->flags & ~RA_I2C_SUPPORTED_FLAGS) != 0 ||
          msg->addr > 0x7f ||
          msg->buffer == NULL || msg->length <= 0)
        {
          return -EINVAL;
        }

      if (msg->frequency != I2C_SPEED_STANDARD &&
          msg->frequency != I2C_SPEED_FAST)
        {
          return -EINVAL;
        }

      if ((msg->flags & I2C_M_NOSTART) != 0)
        {
          if (i == 0 || (msg->flags & I2C_M_READ) != 0 ||
              (msgs[i - 1].flags & I2C_M_READ) != 0 ||
              msg->addr != msgs[i - 1].addr ||
              msg->frequency != msgs[i - 1].frequency)
            {
              return -EINVAL;
            }
        }
    }

  if (priv->restart_pending &&
      (msgs[0].flags & I2C_M_NOSTART) != 0)
    {
      return -EINVAL;
    }

  return OK;
}

static enum ra_i2c_end_e
ra_i2c_end_action(FAR const struct i2c_msg_s *msgs, int count, int index)
{
  if (index + 1 < count &&
      (msgs[index + 1].flags & I2C_M_NOSTART) != 0)
    {
      return RA_I2C_END_HOLD;
    }

  if ((msgs[index].flags & I2C_M_NOSTOP) != 0)
    {
      return RA_I2C_END_RESTART;
    }

  return RA_I2C_END_STOP;
}

static int ra_i2c_transfer(FAR struct i2c_master_s *dev,
                           FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct ra_i2c_priv_s *priv = (FAR struct ra_i2c_priv_s *)dev;
  bool address;
  int ret;
  int i;

  DEBUGASSERT(priv != NULL && priv->refs > 0);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = ra_i2c_validate(priv, msgs, count);
  if (ret < 0)
    {
      goto out;
    }

  for (i = 0; i < count; i++)
    {
      FAR struct i2c_msg_s *msg = &msgs[i];
      enum ra_i2c_end_e end = ra_i2c_end_action(msgs, count, i);

      address = (msg->flags & I2C_M_NOSTART) == 0;
      if (address)
        {
          if (!priv->held)
            {
              if (priv->frequency != msg->frequency)
                {
                  ret = ra_i2c_setfrequency(priv, msg->frequency);
                  if (ret < 0)
                    {
                      break;
                    }
                }

              ret = ra_i2c_start(priv, false);
            }
          else if (priv->restart_pending)
            {
              priv->restart_pending = false;
              ret = OK;
            }
          else
            {
              ret = ra_i2c_start(priv, true);
            }

          if (ret < 0)
            {
              break;
            }
        }

      if ((msg->flags & I2C_M_READ) != 0)
        {
          ret = ra_i2c_read_msg(priv, msg, end);
        }
      else
        {
          ret = ra_i2c_write_msg(priv, msg, address, end);
        }

      if (ret < 0)
        {
          break;
        }
    }

  if (ret < 0)
    {
      ra_i2c_abort(priv);
    }

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_i2cbus_initialize
 ****************************************************************************/

FAR struct i2c_master_s *ra_i2cbus_initialize(int port)
{
  FAR struct ra_i2c_priv_s *priv;
  int ret;

  switch (port)
    {
#ifdef CONFIG_RA8P1_I2C0
      case 0:
        priv = &g_ra_i2c0;
        break;
#endif

#ifdef CONFIG_RA8P1_I2C1
      case 1:
        priv = &g_ra_i2c1;
        break;
#endif

      default:
        return NULL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return NULL;
    }

  if (priv->refs++ == 0)
    {
      ra_module_control(RA8P1_MSTPCRB, priv->mstp, true);
      priv->frequency = RA_I2C_DEFAULT_FREQUENCY;
      ret = ra_i2c_setfrequency(priv, priv->frequency);
      if (ret < 0)
        {
          ra_module_control(RA8P1_MSTPCRB, priv->mstp, false);
          priv->refs = 0;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret < 0 ? NULL : &priv->dev;
}

/****************************************************************************
 * Name: ra_i2cbus_uninitialize
 ****************************************************************************/

int ra_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  FAR struct ra_i2c_priv_s *priv = (FAR struct ra_i2c_priv_s *)dev;
  int ret;

  if (priv == NULL || priv->dev.ops != &g_ra_i2c_ops)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->refs == 0)
    {
      ret = -EINVAL;
    }
  else if (--priv->refs == 0)
    {
      if (priv->held)
        {
          ra_i2c_stop(priv);
        }

      putreg8(0, RA_I2C_REG(priv, ICCR1));
      ra_module_control(RA8P1_MSTPCRB, priv->mstp, false);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

#endif /* CONFIG_RA8P1_I2C */
