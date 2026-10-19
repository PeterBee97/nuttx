/****************************************************************************
 * arch/arm/src/ra8p1/ra_i2s.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/queue.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

#include <debug.h>

#include "arm_internal.h"
#include "hardware/ra8p1_ssi.h"
#include "hardware/ra8p1_system.h"
#include "ra_clockconfig.h"
#include "ra_i2s.h"
#include "ra_icu.h"

#ifdef CONFIG_RA8P1_SSI0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_I2S_SAMPLE_RATE        16000
#define RA_I2S_CHANNELS           2
#define RA_I2S_DATA_WIDTH         16
#define RA_I2S_RESET_TIMEOUT_US   1000
#define RA_I2S_FIFO_WATERMARK     15

#define RA_I2S_SSICR_CONFIG       (SSI_SSICR_CKS_INTERNAL | \
                                   SSI_SSICR_DWL_16 | \
                                   SSI_SSICR_SWL_16 | \
                                   SSI_SSICR_MST | \
                                   SSI_SSICR_CKDV_DIV12)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_i2s_buffer_s
{
  sq_entry_t entry;
  FAR struct ap_buffer_s *apb;
  i2s_callback_t callback;
  FAR void *arg;
  uint32_t timeout;
  int result;
};

struct ra_i2s_s
{
  /* dev must remain first for i2s_dev_s cast compatibility. */

  struct i2s_dev_s dev;
  mutex_t lock;
  sq_queue_t pending;
  sq_queue_t done;
  FAR struct ra_i2s_buffer_s *active;
  FAR struct ra_i2s_buffer_s *free;
  FAR const uint16_t *tx;
  uint32_t tx_samples;
  struct wdog_s watchdog;
  struct work_s work;
  bool initialized;
  struct ra_i2s_buffer_s buffers[CONFIG_RA8P1_SSI0_MAXINFLIGHT];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra_i2s_txchannels(FAR struct i2s_dev_s *dev, uint8_t channels);
static int32_t ra_i2s_txsamplerate(FAR struct i2s_dev_s *dev,
                                   uint32_t rate);
static int32_t ra_i2s_txdatawidth(FAR struct i2s_dev_s *dev, int bits);
static int ra_i2s_send(FAR struct i2s_dev_s *dev,
                       FAR struct ap_buffer_s *apb,
                       i2s_callback_t callback, FAR void *arg,
                       uint32_t timeout);
static int ra_i2s_ioctl(FAR struct i2s_dev_s *dev, int cmd,
                        unsigned long arg);
static void ra_i2s_timeout(wdparm_t arg);
static void ra_i2s_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2s_ops_s g_ra_i2s_ops =
{
  .i2s_txchannels = ra_i2s_txchannels,
  .i2s_txsamplerate = ra_i2s_txsamplerate,
  .i2s_txdatawidth = ra_i2s_txdatawidth,
  .i2s_send = ra_i2s_send,
  .i2s_ioctl = ra_i2s_ioctl,
};

static struct ra_i2s_s g_ra_i2s =
{
  .dev =
  {
    .ops = &g_ra_i2s_ops,
  },
  .lock = NXMUTEX_INITIALIZER,
};

#ifdef CONFIG_RA8P1_SSI0_REFERENCE
#define RA_REFERENCE_SIZE 2048

static struct
{
  uint32_t time[RA_REFERENCE_SIZE];
  int16_t sample[RA_REFERENCE_SIZE];
  uint32_t written;
  uint32_t clock;
  uint32_t fraction;
  int16_t left;
  bool right;
} g_reference;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ra_i2s_module(bool enable)
{
  ra_module_control(RA8P1_MSTPCRC, MSTPCRC_SSI0, enable);
}

static FAR struct ra_i2s_buffer_s *
ra_i2s_buffer_alloc(FAR struct ra_i2s_s *priv)
{
  FAR struct ra_i2s_buffer_s *buffer;
  irqstate_t flags;

  flags = enter_critical_section();
  buffer = priv->free;
  if (buffer != NULL)
    {
      priv->free = (FAR struct ra_i2s_buffer_s *)buffer->entry.flink;
    }

  leave_critical_section(flags);
  return buffer;
}

static void ra_i2s_buffer_free(FAR struct ra_i2s_s *priv,
                               FAR struct ra_i2s_buffer_s *buffer)
{
  irqstate_t flags;

  flags = enter_critical_section();
  buffer->entry.flink = (FAR sq_entry_t *)priv->free;
  priv->free = buffer;
  leave_critical_section(flags);
}

static void ra_i2s_schedule_worker(FAR struct ra_i2s_s *priv)
{
  int ret;

  if (work_available(&priv->work))
    {
      ret = work_queue(HPWORK, &priv->work, ra_i2s_worker, priv, 0);
      if (ret < 0)
        {
          i2serr("ERROR: Failed to queue SSI completion: %d\n", ret);
        }
    }
}

static void ra_i2s_start_next(FAR struct ra_i2s_s *priv)
{
  FAR struct ra_i2s_buffer_s *buffer;
  FAR struct ap_buffer_s *apb;
  int ret;

  if (priv->active != NULL || sq_empty(&priv->pending))
    {
      return;
    }

  buffer = (FAR struct ra_i2s_buffer_s *)sq_remfirst(&priv->pending);
  priv->active = buffer;
  apb = buffer->apb;
  priv->tx = (FAR const uint16_t *)&apb->samp[apb->curbyte];
  priv->tx_samples = (apb->nbytes - apb->curbyte) >> 1;

  if (buffer->timeout > 0)
    {
      ret = wd_start(&priv->watchdog, buffer->timeout,
                     ra_i2s_timeout, (wdparm_t)priv);
      if (ret < 0)
        {
          i2serr("ERROR: Failed to start SSI timeout: %d\n", ret);
        }
    }
}

static bool ra_i2s_complete_active(FAR struct ra_i2s_s *priv, int result)
{
  FAR struct ra_i2s_buffer_s *buffer = priv->active;

  if (buffer == NULL)
    {
      return false;
    }

  if (buffer->timeout > 0)
    {
      wd_cancel(&priv->watchdog);
    }

  buffer->result = result;
  priv->active = NULL;
  priv->tx = NULL;
  priv->tx_samples = 0;
  sq_addlast(&buffer->entry, &priv->done);
  return true;
}

static bool ra_i2s_cancel(FAR struct ra_i2s_s *priv, int result)
{
  FAR struct ra_i2s_buffer_s *buffer;
  bool completed;

  completed = ra_i2s_complete_active(priv, result);
  for (; ; )
    {
      buffer = (FAR struct ra_i2s_buffer_s *)
               sq_remfirst(&priv->pending);
      if (buffer == NULL)
        {
          break;
        }

      buffer->result = result;
      sq_addlast(&buffer->entry, &priv->done);
      completed = true;
    }

  return completed;
}

static bool ra_i2s_feed_fifo(FAR struct ra_i2s_s *priv)
{
  uint32_t free_stages;
  uint32_t fsr;
  uint32_t samples;
  bool completed = false;
  bool wrote = false;

  fsr = getreg32(RA8P1_SSI_SSIFSR);
  if ((fsr & SSI_SSIFSR_TDE) == 0)
    {
      return false;
    }

  free_stages = SSI_FIFO_STAGES -
                ((fsr & SSI_SSIFSR_TDC_MASK) >>
                 SSI_SSIFSR_TDC_SHIFT);

#ifdef CONFIG_RA8P1_SSI0_REFERENCE
  g_reference.clock = (uint32_t)perf_gettime() +
    (uint64_t)(SSI_FIFO_STAGES - free_stages) * perf_getfreq() /
    (2 * RA_I2S_SAMPLE_RATE);
  g_reference.fraction = 0;
#endif

  while (free_stages > 0)
    {
      ra_i2s_start_next(priv);
      if (priv->active == NULL)
        {
          break;
        }

      samples = priv->tx_samples;
      if (samples > free_stages)
        {
          samples = free_stages;
        }

      priv->tx_samples -= samples;
      free_stages -= samples;
      while (samples-- > 0)
        {
#ifdef CONFIG_RA8P1_SSI0_REFERENCE
          if (!g_reference.right)
            {
              g_reference.left = (int16_t)*priv->tx;
            }
          else
            {
              unsigned int slot =
                g_reference.written % RA_REFERENCE_SIZE;

              g_reference.sample[slot] =
                ((int32_t)g_reference.left + (int16_t)*priv->tx) / 2;
              g_reference.time[slot] = g_reference.clock;
              g_reference.written++;
              g_reference.fraction += perf_getfreq();
              g_reference.clock +=
                g_reference.fraction / RA_I2S_SAMPLE_RATE;
              g_reference.fraction %= RA_I2S_SAMPLE_RATE;
            }

          g_reference.right = !g_reference.right;
#endif
          putreg16(*priv->tx++, RA8P1_SSI_SSIFTDR);
          wrote = true;
        }

      if (priv->tx_samples == 0)
        {
          completed |= ra_i2s_complete_active(priv, OK);
        }
    }

  if (wrote)
    {
      (void)getreg32(RA8P1_SSI_SSIFSR);
      putreg32(SSI_SSIFSR_TDE_CLEAR, RA8P1_SSI_SSIFSR);
    }

  return completed;
}

static int ra_i2s_hardware_start(FAR struct ra_i2s_s *priv,
                                 FAR bool *completed)
{
  uint32_t cr;
  uint32_t fcr;
  int timeout;

  cr = getreg32(RA8P1_SSI_SSICR);
  if ((cr & SSI_SSICR_TEN) != 0)
    {
      modifyreg32(RA8P1_SSI_SSIFCR, 0, SSI_SSIFCR_TIE);
      *completed |= ra_i2s_feed_fifo(priv);
      return OK;
    }

  if ((getreg32(RA8P1_SSI_SSISR) & SSI_SSISR_IIRQ) == 0)
    {
      return -EBUSY;
    }

  fcr = SSI_SSIFCR_AUCKE;
  putreg32(fcr | SSI_SSIFCR_FIFO_RESET, RA8P1_SSI_SSIFCR);
  putreg32(fcr, RA8P1_SSI_SSIFCR);
  for (timeout = RA_I2S_RESET_TIMEOUT_US; timeout > 0; timeout--)
    {
      if ((getreg32(RA8P1_SSI_SSIFCR) &
           SSI_SSIFCR_FIFO_RESET) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }

  for (timeout = RA_I2S_RESET_TIMEOUT_US; timeout > 0; timeout--)
    {
      if ((getreg32(RA8P1_SSI_SSIFSR) & SSI_SSIFSR_TDE) != 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }

#ifdef CONFIG_RA8P1_SSI0_REFERENCE
  g_reference.clock = perf_gettime();
  g_reference.fraction = 0;
  g_reference.right = false;
#endif
  *completed |= ra_i2s_feed_fifo(priv);
  putreg32(fcr | SSI_SSIFCR_TIE, RA8P1_SSI_SSIFCR);
  putreg32(RA_I2S_SSICR_CONFIG | SSI_SSICR_TUIEN |
           SSI_SSICR_TEN, RA8P1_SSI_SSICR);
  return OK;
}

static void ra_i2s_hardware_stop(void)
{
  uint32_t cr;
  bool running;

  cr = getreg32(RA8P1_SSI_SSICR);
  running = (cr & (SSI_SSICR_TEN | SSI_SSICR_REN)) != 0;
  cr &= ~(SSI_SSICR_TEN | SSI_SSICR_REN | SSI_SSICR_TXRX_IRQS);
  if (running)
    {
      cr |= SSI_SSICR_IIEN;
    }

  putreg32(cr, RA8P1_SSI_SSICR);
  putreg32(SSI_SSIFCR_AUCKE, RA8P1_SSI_SSIFCR);
}

static void ra_i2s_timeout(wdparm_t arg)
{
  FAR struct ra_i2s_s *priv = (FAR struct ra_i2s_s *)arg;
  irqstate_t flags;
  bool completed;

  flags = enter_critical_section();
  completed = priv->active != NULL;
  if (completed)
    {
      priv->active->timeout = 0;
      ra_i2s_hardware_stop();
      (void)ra_i2s_complete_active(priv, -ETIMEDOUT);
    }

  leave_critical_section(flags);

  if (completed)
    {
      ra_i2s_schedule_worker(priv);
    }
}

static bool ra_i2s_kick(FAR struct ra_i2s_s *priv)
{
  bool completed = false;
  int ret;

  ra_i2s_start_next(priv);
  if (priv->active == NULL)
    {
      return false;
    }

  ret = ra_i2s_hardware_start(priv, &completed);
  if (ret == -EBUSY)
    {
      return false;
    }

  if (ret < 0)
    {
      return ra_i2s_cancel(priv, ret);
    }

  return completed;
}

static int ra_i2s_txinterrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ra_i2s_s *priv = arg;
  bool completed;

  ra_clear_ir(irq);
  completed = ra_i2s_feed_fifo(priv);
  if (completed)
    {
      ra_i2s_schedule_worker(priv);
    }

  return OK;
}

static int ra_i2s_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ra_i2s_s *priv = arg;
  uint32_t status;
  bool completed = false;

  status = getreg32(RA8P1_SSI_SSISR);
  putreg32(0, RA8P1_SSI_SSISR);

  if ((status & SSI_SSISR_IIRQ) != 0)
    {
      modifyreg32(RA8P1_SSI_SSICR, SSI_SSICR_IIEN, 0);
      completed = ra_i2s_kick(priv);
    }
  else if ((status & SSI_SSISR_ERRORS) != 0)
    {
      ra_i2s_hardware_stop();
      completed = ra_i2s_cancel(priv, -EIO);
    }

  ra_clear_ir(irq);
  if (completed)
    {
      ra_i2s_schedule_worker(priv);
    }

  return OK;
}

static int ra_i2s_hardware_initialize(FAR struct ra_i2s_s *priv)
{
  uint32_t fcr;
  int timeout;
  int ret;

  ra_i2s_module(true);
  putreg32(RA_I2S_SSICR_CONFIG, RA8P1_SSI_SSICR);

  fcr = SSI_SSIFCR_AUCKE;
  putreg32(fcr | SSI_SSIFCR_SSIRST, RA8P1_SSI_SSIFCR);
  putreg32(fcr, RA8P1_SSI_SSIFCR);
  for (timeout = RA_I2S_RESET_TIMEOUT_US; timeout > 0; timeout--)
    {
      if ((getreg32(RA8P1_SSI_SSIFCR) & SSI_SSIFCR_SSIRST) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (timeout == 0)
    {
      ret = -ETIMEDOUT;
      goto errout_module;
    }

  putreg32(SSI_SSISCR_TDES(RA_I2S_FIFO_WATERMARK) |
           SSI_SSISCR_RDFS(1), RA8P1_SSI_SSISCR);
  putreg32(SSI_SSIOFR_LRCONT, RA8P1_SSI_SSIOFR);

  ret = irq_attach(RA_IRQ_SSI0_TXI, ra_i2s_txinterrupt, priv);
  if (ret < 0)
    {
      goto errout_module;
    }

  ret = irq_attach(RA_IRQ_SSI0_INT, ra_i2s_interrupt, priv);
  if (ret < 0)
    {
      irq_detach(RA_IRQ_SSI0_TXI);
      goto errout_module;
    }

#ifdef CONFIG_ARCH_IRQPRIO
  /* SSI handlers use work queues and must obey OS critical sections. */

  up_prioritize_irq(RA_IRQ_SSI0_TXI, NVIC_SYSH_PRIORITY_DEFAULT);
  up_prioritize_irq(RA_IRQ_SSI0_INT, NVIC_SYSH_PRIORITY_DEFAULT);
#endif

  ra_clear_ir(RA_IRQ_SSI0_TXI);
  ra_clear_ir(RA_IRQ_SSI0_INT);
  up_enable_irq(RA_IRQ_SSI0_TXI);
  up_enable_irq(RA_IRQ_SSI0_INT);
  return OK;

errout_module:
  ra_i2s_module(false);
  return ret;
}

static void ra_i2s_worker(FAR void *arg)
{
  FAR struct ra_i2s_s *priv = arg;
  FAR struct ra_i2s_buffer_s *buffer;
  irqstate_t flags;

  for (; ; )
    {
      flags = enter_critical_section();
      buffer = (FAR struct ra_i2s_buffer_s *)sq_remfirst(&priv->done);
      leave_critical_section(flags);
      if (buffer == NULL)
        {
          break;
        }

      buffer->callback(&priv->dev, buffer->apb, buffer->arg,
                       buffer->result);
      apb_free(buffer->apb);
      ra_i2s_buffer_free(priv, buffer);
    }
}

static int ra_i2s_txchannels(FAR struct i2s_dev_s *dev, uint8_t channels)
{
  (void)dev;
  return channels == RA_I2S_CHANNELS ? RA_I2S_CHANNELS : -EINVAL;
}

static int32_t ra_i2s_txsamplerate(FAR struct i2s_dev_s *dev,
                                   uint32_t rate)
{
  (void)dev;
  return rate == RA_I2S_SAMPLE_RATE ? RA_I2S_SAMPLE_RATE : -EINVAL;
}

static int32_t ra_i2s_txdatawidth(FAR struct i2s_dev_s *dev, int bits)
{
  (void)dev;
  return bits == RA_I2S_DATA_WIDTH ? RA_I2S_DATA_WIDTH : -EINVAL;
}

static int ra_i2s_send(FAR struct i2s_dev_s *dev,
                       FAR struct ap_buffer_s *apb,
                       i2s_callback_t callback, FAR void *arg,
                       uint32_t timeout)
{
  FAR struct ra_i2s_s *priv = (FAR struct ra_i2s_s *)dev;
  FAR struct ra_i2s_buffer_s *buffer;
  irqstate_t flags;
  bool completed;
  int ret;

  if (apb == NULL || callback == NULL ||
      apb->nbytes <= apb->curbyte ||
      ((apb->nbytes - apb->curbyte) & 3) != 0)
    {
      return -EINVAL;
    }

  buffer = ra_i2s_buffer_alloc(priv);
  if (buffer == NULL)
    {
      return -ENOMEM;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      ra_i2s_buffer_free(priv, buffer);
      return ret;
    }

  apb_reference(apb);
  buffer->apb = apb;
  buffer->callback = callback;
  buffer->arg = arg;
  buffer->timeout = timeout;
  buffer->result = -EBUSY;

  flags = enter_critical_section();
  sq_addlast(&buffer->entry, &priv->pending);
  completed = ra_i2s_kick(priv);
  leave_critical_section(flags);
  nxmutex_unlock(&priv->lock);

  if (completed)
    {
      ra_i2s_schedule_worker(priv);
    }

  return OK;
}

static int ra_i2s_ioctl(FAR struct i2s_dev_s *dev, int cmd,
                        unsigned long arg)
{
  FAR struct ra_i2s_s *priv = (FAR struct ra_i2s_s *)dev;
  FAR struct audio_buf_desc_s *desc;
  irqstate_t flags;
  bool completed;

  switch (cmd)
    {
      case AUDIOIOC_START:
      case AUDIOIOC_RESUME:
        return OK;

      case AUDIOIOC_STOP:
      case AUDIOIOC_PAUSE:
      case AUDIOIOC_SHUTDOWN:
        flags = enter_critical_section();
        ra_i2s_hardware_stop();
        completed = ra_i2s_cancel(priv, -ECANCELED);
        leave_critical_section(flags);
        if (completed)
          {
            ra_i2s_schedule_worker(priv);
          }

        return OK;

      case AUDIOIOC_ALLOCBUFFER:
        desc = (FAR struct audio_buf_desc_s *)(uintptr_t)arg;
        if (desc == NULL)
          {
            return -EINVAL;
          }

        return apb_alloc(desc);

      case AUDIOIOC_FREEBUFFER:
        desc = (FAR struct audio_buf_desc_s *)(uintptr_t)arg;
        if (desc == NULL || desc->u.buffer == NULL)
          {
            return -EINVAL;
          }

        apb_free(desc->u.buffer);
        return sizeof(*desc);

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_RA8P1_SSI0_REFERENCE
int16_t ra_i2s_reference(uint32_t time)
{
  uint32_t end = g_reference.written;
  uint32_t begin = end > RA_REFERENCE_SIZE ?
                   end - RA_REFERENCE_SIZE : 0;
  uint32_t middle;
  uint32_t first;
  uint32_t second;
  int32_t span;
  int32_t offset;

  /* Binary search also handles the performance counter wrapping. The
   * retained history is much shorter than half its wrap interval.
   */

  if (end == begin ||
      (int32_t)(time - g_reference.time[begin % RA_REFERENCE_SIZE]) < 0 ||
      (int32_t)(time -
        g_reference.time[(end - 1) % RA_REFERENCE_SIZE]) > 0)
    {
      return 0;
    }

  while (end - begin > 1)
    {
      middle = begin + (end - begin) / 2;
      if ((int32_t)(time -
          g_reference.time[middle % RA_REFERENCE_SIZE]) >= 0)
        {
          begin = middle;
        }
      else
        {
          end = middle;
        }
    }

  first = begin % RA_REFERENCE_SIZE;
  if (begin + 1 == g_reference.written)
    {
      return g_reference.sample[first];
    }

  second = (begin + 1) % RA_REFERENCE_SIZE;
  span = g_reference.time[second] - g_reference.time[first];
  offset = time - g_reference.time[first];

  /* A stopped/restarted transmitter leaves a gap, not a ramp. */

  if (span <= 0 || span > (int32_t)(perf_getfreq() / 8000))
    {
      return 0;
    }

  return g_reference.sample[first] +
         (int64_t)(g_reference.sample[second] -
                   g_reference.sample[first]) * offset / span;
}
#endif

/****************************************************************************
 * Name: ra_i2s_initialize
 ****************************************************************************/

FAR struct i2s_dev_s *ra_i2s_initialize(int port)
{
  FAR struct ra_i2s_s *priv = &g_ra_i2s;
  int ret;
  int i;

  if (port != 0)
    {
      return NULL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return NULL;
    }

  if (priv->initialized)
    {
      nxmutex_unlock(&priv->lock);
      return &priv->dev;
    }

  sq_init(&priv->pending);
  sq_init(&priv->done);
  priv->free = NULL;
  for (i = 0; i < CONFIG_RA8P1_SSI0_MAXINFLIGHT; i++)
    {
      priv->buffers[i].entry.flink = (FAR sq_entry_t *)priv->free;
      priv->free = &priv->buffers[i];
    }

  ret = ra_i2s_hardware_initialize(priv);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  priv->initialized = true;
  nxmutex_unlock(&priv->lock);
  return &priv->dev;
}

#endif /* CONFIG_RA8P1_SSI0 */
