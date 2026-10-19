/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_pdm.c
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

#include <sys/param.h>
#include <sys/types.h>
#include <poll.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>
#include <nuttx/audio/pdm.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/ra8p1_icu.h"
#include "hardware/ra8p1_pdm.h"
#include "hardware/ra8p1_system.h"
#include "ra_clockconfig.h"
#include "ra_icu.h"
#include "ra_pinmux.h"
#ifdef CONFIG_TITAN_MINI_PDM_REFERENCE
#  include "ra_i2s.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TITAN_PDM_DEVICE              "/dev/pdm0"
#define TITAN_PDM_LEFT_CHANNEL        0
#define TITAN_PDM_RIGHT_CHANNEL       2
#define TITAN_PDM_CHANNEL_MASK        (PDM_CHANNEL(TITAN_PDM_LEFT_CHANNEL) | \
                                       PDM_CHANNEL(TITAN_PDM_RIGHT_CHANNEL))
#define TITAN_PDM_FIFO_DEPTH          32
#define TITAN_PDM_INITIAL_READS       (TITAN_PDM_FIFO_DEPTH - 2)
#define TITAN_PDM_PENDING_SAMPLES     64
#define TITAN_PDM_FRAME_BYTES         (2 * sizeof(int16_t))
#define TITAN_PDM_FILTER_SETTLE_US    37000
#define TITAN_PDM_STOP_TIMEOUT_US     1000

#ifdef CONFIG_TITAN_MINI_PDM_REFERENCE
#  define TITAN_PDM_CHANNELS          3
#else
#  define TITAN_PDM_CHANNELS          2
#endif

#define TITAN_PDM_HPF_SHIFT           0x3f61
#define TITAN_PDM_HPF_GAIN            0x3ec1
#define TITAN_PDM_LPF_GAIN            0x0400
#define TITAN_PDM_SC_THRESHOLD        0x1fff1fff
#define TITAN_PDM_OVL_THRESHOLD       0x00085ee0
#define TITAN_PDM_OVU_THRESHOLD       0x0007a120

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra8p1_pdm_fifo_s
{
  int16_t sample[TITAN_PDM_PENDING_SAMPLES];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
};

struct ra8p1_pdm_dev_s
{
  mutex_t lock;
  mutex_t readlock;
  sem_t waitsem;
  FAR struct pollfd *fds;
  struct ra8p1_pdm_fifo_s left;
  struct ra8p1_pdm_fifo_s right;
  int16_t ring[CONFIG_TITAN_MINI_PDM_BUFFER_FRAMES][TITAN_PDM_CHANNELS];
#ifdef CONFIG_TITAN_MINI_PDM_REFERENCE
  uint32_t sampletime;
  uint32_t fraction;
#endif
  unsigned int head;
  unsigned int tail;
  unsigned int count;
  uint32_t frames;
  uint32_t highwater;
  uint32_t overruns;
  uint32_t desyncs;
  uint32_t hwerrors;
  uint32_t fifoerrors;
  uint32_t levelerrors;
  uint32_t shorterrors;
  bool opened;
  bool initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra8p1_pdm_open(FAR struct file *filep);
static int ra8p1_pdm_close(FAR struct file *filep);
static ssize_t ra8p1_pdm_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static int ra8p1_pdm_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
static int ra8p1_pdm_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup);
#ifdef CONFIG_TITAN_MINI_PDM_REFERENCE
static ssize_t ra8p1_pdm_refread(FAR struct file *filep, FAR char *buffer,
                                size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Filter coefficients and thresholds match the board SDK's 16 kHz,
 * fourth-order SINC profile for the IM72D128 microphones.
 */

static const uint16_t g_pdm_hpf_h[] =
{
  0x4000, 0xc000
};

static const uint16_t g_pdm_compensation_h[] =
{
  0x1fe8, 0x0039, 0x003c, 0x1e56, 0x01dc, 0x06e1,
  0x01dc, 0x1e56, 0x003c, 0x0039, 0x1fe8
};

static const uint16_t g_pdm_lpf_h1[] =
{
  0x1ff8, 0x000a, 0x1ff0, 0x0018, 0x1fdc,
  0x0034, 0x1fb3, 0x0076, 0x1f2e, 0x0289,
  0x0289, 0x1f2e, 0x0076, 0x1fb3, 0x0034,
  0x1fdc, 0x0018, 0x1ff0, 0x000a, 0x1ff8
};

static const struct file_operations g_pdm_fops =
{
  .open  = ra8p1_pdm_open,
  .close = ra8p1_pdm_close,
  .read  = ra8p1_pdm_read,
  .ioctl = ra8p1_pdm_ioctl,
  .poll  = ra8p1_pdm_poll,
};

static struct ra8p1_pdm_dev_s g_pdm;

#ifdef CONFIG_TITAN_MINI_PDM_REFERENCE
static const struct file_operations g_pdm_ref_fops =
{
  .open  = ra8p1_pdm_open,
  .close = ra8p1_pdm_close,
  .read  = ra8p1_pdm_refread,
  .ioctl = ra8p1_pdm_ioctl,
  .poll  = ra8p1_pdm_poll,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int16_t ra8p1_pdm_convert(uint32_t sample)
{
  return (int16_t)((uint16_t)sample << 1);
}

static void ra8p1_pdm_fifo_reset(FAR struct ra8p1_pdm_fifo_s *fifo)
{
  fifo->head = 0;
  fifo->tail = 0;
  fifo->count = 0;
}

static void ra8p1_pdm_fifo_put(FAR struct ra8p1_pdm_dev_s *priv,
                               FAR struct ra8p1_pdm_fifo_s *fifo,
                               int16_t sample)
{
  if (fifo->count >= TITAN_PDM_PENDING_SAMPLES)
    {
      fifo->tail++;
      if (fifo->tail >= TITAN_PDM_PENDING_SAMPLES)
        {
          fifo->tail = 0;
        }

      fifo->count--;
      priv->desyncs++;
    }

  fifo->sample[fifo->head] = sample;
  fifo->head++;
  if (fifo->head >= TITAN_PDM_PENDING_SAMPLES)
    {
      fifo->head = 0;
    }

  fifo->count++;
}

static int16_t ra8p1_pdm_fifo_get(FAR struct ra8p1_pdm_fifo_s *fifo)
{
  int16_t sample = fifo->sample[fifo->tail];

  fifo->tail++;
  if (fifo->tail >= TITAN_PDM_PENDING_SAMPLES)
    {
      fifo->tail = 0;
    }

  fifo->count--;
  return sample;
}

static void ra8p1_pdm_pair_samples(FAR struct ra8p1_pdm_dev_s *priv)
{
  bool notify = false;

#ifdef CONFIG_TITAN_MINI_PDM_REFERENCE
  unsigned int pending = MIN(priv->left.count, priv->right.count);

  if (pending > 0)
    {
      uint32_t first = (uint32_t)perf_gettime() -
        (uint64_t)(pending - 1) * perf_getfreq() /
        CONFIG_TITAN_MINI_PDM_REFERENCE_RATE;

      if (priv->frames == 0)
        {
          priv->sampletime = first;
          priv->fraction = 0;
        }
      else
        {
          /* Re-anchor the pending batch to the current capture timestamp.
           * Fractional sample spacing is retained within the batch.
           */

          priv->sampletime = first;
        }
    }
#endif

  while (priv->left.count > 0 && priv->right.count > 0)
    {
      if (priv->count >= CONFIG_TITAN_MINI_PDM_BUFFER_FRAMES)
        {
          priv->tail++;
          if (priv->tail >= CONFIG_TITAN_MINI_PDM_BUFFER_FRAMES)
            {
              priv->tail = 0;
            }

          priv->count--;
          priv->overruns++;
        }

      if (priv->count == 0)
        {
          notify = true;
        }

      priv->ring[priv->head][0] = ra8p1_pdm_fifo_get(&priv->left);
      priv->ring[priv->head][1] = ra8p1_pdm_fifo_get(&priv->right);
#ifdef CONFIG_TITAN_MINI_PDM_REFERENCE
      priv->ring[priv->head][2] =
        ra_i2s_reference(priv->sampletime - perf_getfreq() / 125);
      priv->fraction += perf_getfreq();
      priv->sampletime += priv->fraction /
                          CONFIG_TITAN_MINI_PDM_REFERENCE_RATE;
      priv->fraction %= CONFIG_TITAN_MINI_PDM_REFERENCE_RATE;
#endif
      priv->head++;
      if (priv->head >= CONFIG_TITAN_MINI_PDM_BUFFER_FRAMES)
        {
          priv->head = 0;
        }

      priv->count++;
      priv->frames++;
      if (priv->count > priv->highwater)
        {
          priv->highwater = priv->count;
        }
    }

  if (notify)
    {
      nxsem_post(&priv->waitsem);
      poll_notify(&priv->fds, 1, POLLIN);
    }
}

static void ra8p1_pdm_check_errors(FAR struct ra8p1_pdm_dev_s *priv,
                                   unsigned int channel)
{
  uint32_t status = getreg32(RA8P1_PDM_PDSR(channel));
  uint32_t errors = status & PDM_PDSR_ERROR_MASK;

  if (errors != 0)
    {
      priv->hwerrors++;
      priv->fifoerrors += (errors & PDM_PDSR_BFOWDF) != 0;
      priv->levelerrors += (errors &
                           (PDM_PDSR_OVLDF | PDM_PDSR_OVUDF)) != 0;
      priv->shorterrors += (errors & PDM_PDSR_SCDF) != 0;
      putreg32(errors, RA8P1_PDM_PDSCR(channel));
    }
}

static int ra8p1_pdm_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ra8p1_pdm_dev_s *priv = arg;
  FAR struct ra8p1_pdm_fifo_s *fifo;
  unsigned int channel;
  unsigned int count;

  ra_clear_ir(irq);

  if (irq == RA_IRQ_PDM_DAT0)
    {
      channel = TITAN_PDM_LEFT_CHANNEL;
      fifo = &priv->left;
    }
  else
    {
      channel = TITAN_PDM_RIGHT_CHANNEL;
      fifo = &priv->right;
    }

  ra8p1_pdm_check_errors(priv, channel);
  count = getreg32(RA8P1_PDM_PDDSR(channel)) & PDM_PDDSR_DATNUM_MASK;
  if (count > TITAN_PDM_FIFO_DEPTH)
    {
      count = TITAN_PDM_FIFO_DEPTH;
    }

  while (count-- > 0)
    {
      ra8p1_pdm_fifo_put(priv, fifo,
                         ra8p1_pdm_convert(
                           getreg32(RA8P1_PDM_PDDRR(channel))));
    }

  ra8p1_pdm_pair_samples(priv);
  return OK;
}

static void ra8p1_pdm_channel_configure(unsigned int channel, bool falling)
{
  uint32_t mode = PDM_PDMDSR_PCM_16_0_14 |
                  PDM_PDMDSR_SINC_ORDER_4;
  unsigned int i;

  if (falling)
    {
      mode |= PDM_PDMDSR_INPUT_FALL;
    }

  putreg32(mode, RA8P1_PDM_PDMDSR(channel));
  putreg32(PDM_PDSFCR_SINCRNG(5) | PDM_PDSFCR_SINCDEC(124),
           RA8P1_PDM_PDSFCR(channel));

  putreg32(TITAN_PDM_HPF_SHIFT, RA8P1_PDM_PDHFCS0R(channel));
  putreg32(TITAN_PDM_HPF_GAIN, RA8P1_PDM_PDHFCK1R(channel));
  for (i = 0; i < nitems(g_pdm_hpf_h); i++)
    {
      putreg32(g_pdm_hpf_h[i], RA8P1_PDM_PDHFCHR(channel, i));
    }

  for (i = 0; i < nitems(g_pdm_compensation_h); i++)
    {
      putreg32(g_pdm_compensation_h[i],
               RA8P1_PDM_PDCFCHR(channel, i));
    }

  putreg32(TITAN_PDM_LPF_GAIN, RA8P1_PDM_PDLFCH010R(channel));
  for (i = 0; i < nitems(g_pdm_lpf_h1); i++)
    {
      putreg32(g_pdm_lpf_h1[i], RA8P1_PDM_PDLFCH1R(channel, i));
    }

  putreg32(PDM_PDDBCR_THRESHOLD_16, RA8P1_PDM_PDDBCR(channel));
  putreg32(TITAN_PDM_SC_THRESHOLD, RA8P1_PDM_PDSCTSR(channel));
  putreg32(TITAN_PDM_OVL_THRESHOLD, RA8P1_PDM_PDOVLTR(channel));
  putreg32(TITAN_PDM_OVU_THRESHOLD, RA8P1_PDM_PDOVUTR(channel));
  putreg32(PDM_PDSCR_CLEAR_MASK, RA8P1_PDM_PDSCR(channel));
  putreg32(PDM_PDSDCR_SCDE | PDM_PDSDCR_OVLDE |
           PDM_PDSDCR_OVUDE | PDM_PDSDCR_BFOWDE,
           RA8P1_PDM_PDSDCR(channel));
  putreg32(PDM_PDDCR_DATC, RA8P1_PDM_PDDCR(channel));
}

static void ra8p1_pdm_module_enable(bool enable)
{
  ra_module_control(RA8P1_MSTPCRC, MSTPCRC_PDM, enable);
}

static int ra8p1_pdm_start(FAR struct ra8p1_pdm_dev_s *priv)
{
  int ret;
  int i;

  priv->head = 0;
  priv->tail = 0;
  priv->count = 0;
  priv->frames = 0;
  priv->highwater = 0;
  priv->overruns = 0;
  priv->desyncs = 0;
  priv->hwerrors = 0;
  priv->fifoerrors = 0;
  priv->levelerrors = 0;
  priv->shorterrors = 0;
  ra8p1_pdm_fifo_reset(&priv->left);
  ra8p1_pdm_fifo_reset(&priv->right);
  nxsem_reset(&priv->waitsem, 0);

  ra8p1_pdm_module_enable(true);
  ra8p1_pdm_channel_configure(TITAN_PDM_LEFT_CHANNEL, true);
  ra8p1_pdm_channel_configure(TITAN_PDM_RIGHT_CHANNEL, false);

  ra_icu_set_event(RA_IRQ_PDM_DAT0, ICU_EVENT_PDM_DAT0);
  ra_icu_set_event(RA_IRQ_PDM_DAT2, ICU_EVENT_PDM_DAT2);
  ret = irq_attach(RA_IRQ_PDM_DAT0, ra8p1_pdm_interrupt, priv);
  if (ret < 0)
    {
      goto errout_with_module;
    }

  ret = irq_attach(RA_IRQ_PDM_DAT2, ra8p1_pdm_interrupt, priv);
  if (ret < 0)
    {
      irq_detach(RA_IRQ_PDM_DAT0);
      goto errout_with_module;
    }

  putreg32(TITAN_PDM_CHANNEL_MASK, RA8P1_PDM_PDCSTRTR);
  nxsig_usleep(TITAN_PDM_FILTER_SETTLE_US);

  putreg32(PDM_PDDRCR_DATRE,
           RA8P1_PDM_PDDRCR(TITAN_PDM_LEFT_CHANNEL));
  putreg32(PDM_PDDRCR_DATRE,
           RA8P1_PDM_PDDRCR(TITAN_PDM_RIGHT_CHANNEL));
  for (i = 0; i < TITAN_PDM_INITIAL_READS; i++)
    {
      getreg32(RA8P1_PDM_PDDRR(TITAN_PDM_LEFT_CHANNEL));
      getreg32(RA8P1_PDM_PDDRR(TITAN_PDM_RIGHT_CHANNEL));
    }

  /* The settling interval deliberately fills the FIFO before DATRE.
   * Discard its latched overflow flags together with the settling samples.
   */

  putreg32(PDM_PDSCR_CLEAR_MASK,
           RA8P1_PDM_PDSCR(TITAN_PDM_LEFT_CHANNEL));
  putreg32(PDM_PDSCR_CLEAR_MASK,
           RA8P1_PDM_PDSCR(TITAN_PDM_RIGHT_CHANNEL));
  ra_clear_ir(RA_IRQ_PDM_DAT0);
  ra_clear_ir(RA_IRQ_PDM_DAT2);
  putreg32(PDM_PDICR_IDRE,
           RA8P1_PDM_PDICR(TITAN_PDM_LEFT_CHANNEL));
  putreg32(PDM_PDICR_IDRE,
           RA8P1_PDM_PDICR(TITAN_PDM_RIGHT_CHANNEL));

#ifdef CONFIG_ARCH_IRQPRIO
  up_prioritize_irq(RA_IRQ_PDM_DAT0, NVIC_SYSH_PRIORITY_DEFAULT);
  up_prioritize_irq(RA_IRQ_PDM_DAT2, NVIC_SYSH_PRIORITY_DEFAULT);
#endif

  up_enable_irq(RA_IRQ_PDM_DAT0);
  up_enable_irq(RA_IRQ_PDM_DAT2);
  return OK;

errout_with_module:
  putreg32(TITAN_PDM_CHANNEL_MASK, RA8P1_PDM_PDCSTPTR);
  ra8p1_pdm_module_enable(false);
  return ret;
}

static int ra8p1_pdm_stop(FAR struct ra8p1_pdm_dev_s *priv)
{
  irqstate_t flags;
  int timeout;
  int ret;

  up_disable_irq(RA_IRQ_PDM_DAT0);
  up_disable_irq(RA_IRQ_PDM_DAT2);
  putreg32(0, RA8P1_PDM_PDICR(TITAN_PDM_LEFT_CHANNEL));
  putreg32(0, RA8P1_PDM_PDICR(TITAN_PDM_RIGHT_CHANNEL));
  putreg32(0, RA8P1_PDM_PDDRCR(TITAN_PDM_LEFT_CHANNEL));
  putreg32(0, RA8P1_PDM_PDDRCR(TITAN_PDM_RIGHT_CHANNEL));
  putreg32(TITAN_PDM_CHANNEL_MASK, RA8P1_PDM_PDCSTPTR);

  for (timeout = 0; timeout < TITAN_PDM_STOP_TIMEOUT_US; timeout++)
    {
      if ((getreg32(RA8P1_PDM_PDCSR) & TITAN_PDM_CHANNEL_MASK) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  ret = timeout >= TITAN_PDM_STOP_TIMEOUT_US ? -ETIMEDOUT : OK;
  putreg32(PDM_PDDCR_DATC,
           RA8P1_PDM_PDDCR(TITAN_PDM_LEFT_CHANNEL));
  putreg32(PDM_PDDCR_DATC,
           RA8P1_PDM_PDDCR(TITAN_PDM_RIGHT_CHANNEL));
  irq_detach(RA_IRQ_PDM_DAT0);
  irq_detach(RA_IRQ_PDM_DAT2);
  ra8p1_pdm_module_enable(false);
  nxsem_post(&priv->waitsem);

  flags = enter_critical_section();
  if (priv->fds != NULL)
    {
      poll_notify(&priv->fds, 1, POLLHUP);
    }

  leave_critical_section(flags);

  syslog(LOG_INFO,
         "PDM: stopped, overruns=%" PRIu32 " desyncs=%" PRIu32
         " errors=%" PRIu32 "\n",
         priv->overruns, priv->desyncs, priv->hwerrors);
  return ret;
}

static int ra8p1_pdm_open(FAR struct file *filep)
{
  FAR struct ra8p1_pdm_dev_s *priv = filep->f_inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->opened)
    {
      ret = -EBUSY;
    }
  else
    {
      priv->opened = true;
      ret = ra8p1_pdm_start(priv);
      if (ret < 0)
        {
          priv->opened = false;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int ra8p1_pdm_close(FAR struct file *filep)
{
  FAR struct ra8p1_pdm_dev_s *priv = filep->f_inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->opened)
    {
      priv->opened = false;
      ret = ra8p1_pdm_stop(priv);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static ssize_t ra8p1_pdm_readframes(FAR struct file *filep, FAR char *buffer,
                                   size_t buflen, unsigned int channels)
{
  FAR struct ra8p1_pdm_dev_s *priv = filep->f_inode->i_private;
  irqstate_t flags;
  size_t requested;
  size_t frames;
  size_t first;
  size_t second;
  int ret;
  size_t framebytes = channels * sizeof(int16_t);

  if (buffer == NULL || buflen < framebytes)
    {
      return -EINVAL;
    }

  requested = buflen / framebytes;
  ret = nxmutex_lock(&priv->readlock);
  if (ret < 0)
    {
      return ret;
    }

  for (; ; )
    {
      flags = enter_critical_section();
      if (!priv->opened)
        {
          leave_critical_section(flags);
          ret = -EPIPE;
          break;
        }

      frames = requested;
      if (frames > priv->count)
        {
          frames = priv->count;
        }

      if (frames > 0)
        {
          first = CONFIG_TITAN_MINI_PDM_BUFFER_FRAMES - priv->tail;
          if (first > frames)
            {
              first = frames;
            }

          second = frames - first;
          if (channels == TITAN_PDM_CHANNELS)
            {
              memcpy(buffer, &priv->ring[priv->tail][0],
                     first * framebytes);
              if (second > 0)
                {
                  memcpy(buffer + first * framebytes,
                         &priv->ring[0][0], second * framebytes);
                }
            }
          else
            {
              size_t i;

              for (i = 0; i < frames; i++)
                {
                  memcpy(buffer + i * framebytes,
                         priv->ring[(priv->tail + i) %
                           CONFIG_TITAN_MINI_PDM_BUFFER_FRAMES],
                         framebytes);
                }
            }

          priv->tail += frames;
          if (priv->tail >= CONFIG_TITAN_MINI_PDM_BUFFER_FRAMES)
            {
              priv->tail -= CONFIG_TITAN_MINI_PDM_BUFFER_FRAMES;
            }

          priv->count -= frames;
          leave_critical_section(flags);
          ret = frames * framebytes;
          break;
        }

      leave_critical_section(flags);
      if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          ret = -EAGAIN;
          break;
        }

      ret = nxsem_wait(&priv->waitsem);
      if (ret < 0 && ret != -EINTR)
        {
          break;
        }
    }

  nxmutex_unlock(&priv->readlock);
  return ret;
}

static ssize_t ra8p1_pdm_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  return ra8p1_pdm_readframes(filep, buffer, buflen, 2);
}

#ifdef CONFIG_TITAN_MINI_PDM_REFERENCE
static ssize_t ra8p1_pdm_refread(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  return ra8p1_pdm_readframes(filep, buffer, buflen, 3);
}
#endif

static int ra8p1_pdm_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct ra8p1_pdm_dev_s *priv = filep->f_inode->i_private;
  irqstate_t flags;
  int ret;

  if (cmd != FIONREAD && cmd != PDMIOC_GETSTATS)
    {
      return -ENOTTY;
    }

  if (arg == 0)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  if (cmd == PDMIOC_GETSTATS)
    {
      FAR struct pdm_stats_s *stats = (FAR void *)(uintptr_t)arg;

      stats->frames = priv->frames;
      stats->overruns = priv->overruns;
      stats->desyncs = priv->desyncs;
      stats->hwerrors = priv->hwerrors;
      stats->fifoerrors = priv->fifoerrors;
      stats->levelerrors = priv->levelerrors;
      stats->shorterrors = priv->shorterrors;
      stats->buffered = priv->count;
      stats->highwater = priv->highwater;
      leave_critical_section(flags);
      return OK;
    }

  ret = priv->count * TITAN_PDM_FRAME_BYTES;
#ifdef CONFIG_TITAN_MINI_PDM_REFERENCE
  if (filep->f_inode->u.i_ops == &g_pdm_ref_fops)
    {
      ret = priv->count * 3 * sizeof(int16_t);
    }
#endif

  leave_critical_section(flags);
  *(FAR int *)(uintptr_t)arg = ret;
  return OK;
}

static int ra8p1_pdm_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup)
{
  FAR struct ra8p1_pdm_dev_s *priv = filep->f_inode->i_private;
  irqstate_t flags;
  int ret = OK;

  flags = enter_critical_section();
  if (setup)
    {
      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
        }
      else if (priv->fds != NULL)
        {
          ret = -EBUSY;
        }
      else
        {
          priv->fds = fds;
          fds->priv = &priv->fds;
          if (priv->count > 0)
            {
              poll_notify(&fds, 1, POLLIN);
            }
        }
    }
  else if (fds->priv != NULL)
    {
      FAR struct pollfd **slot = fds->priv;

      *slot = NULL;
      fds->priv = NULL;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8p1_pdm_initialize
 ****************************************************************************/

int ra8p1_pdm_initialize(void)
{
  FAR struct ra8p1_pdm_dev_s *priv = &g_pdm;
  int ret;

  if (priv->initialized)
    {
      return OK;
    }

  ra_pinmux_configure(BOARD_PDM_DATA_PIN);
  ra_pinmux_configure(BOARD_PDM_CLOCK_PIN);
  nxmutex_init(&priv->lock);
  nxmutex_init(&priv->readlock);
  nxsem_init(&priv->waitsem, 0, 0);

  ret = register_driver(TITAN_PDM_DEVICE, &g_pdm_fops, 0444, priv);
  if (ret < 0)
    {
      nxsem_destroy(&priv->waitsem);
      nxmutex_destroy(&priv->readlock);
      nxmutex_destroy(&priv->lock);
      syslog(LOG_ERR, "ERROR: Failed to register %s: %d\n",
             TITAN_PDM_DEVICE, ret);
      return ret;
    }

#ifdef CONFIG_TITAN_MINI_PDM_REFERENCE
  ret = register_driver("/dev/pdmref", &g_pdm_ref_fops, 0444, priv);
  if (ret < 0)
    {
      unregister_driver(TITAN_PDM_DEVICE);
      nxsem_destroy(&priv->waitsem);
      nxmutex_destroy(&priv->readlock);
      nxmutex_destroy(&priv->lock);
      return ret;
    }
#endif

  priv->initialized = true;
  syslog(LOG_INFO, "PDM: %s is 16 kHz 16-bit stereo (L=fall, R=rise)\n",
         TITAN_PDM_DEVICE);
  return OK;
}
