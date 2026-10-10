/****************************************************************************
 * arch/arm/src/ra8p1/ra_ethosu.c
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

#include <nuttx/aie/ai_engine.h>
#include <nuttx/aie/ethosu.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <arch/irq.h>

#include "arm_internal.h"
#include "ethosu_driver.h"
#include "pmu_ethosu.h"
#include "hardware/ra8p1_ethosu.h"
#include "hardware/ra8p1_memorymap.h"
#include "hardware/ra8p1_system.h"
#include "nvic.h"
#include "ra_clockconfig.h"
#include "ra_ethosu.h"
#include "ra_icu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_ETHOSU_COP1_MAGIC          0x31504f43u
#define RA_ETHOSU_ACTION_OPTIMIZER    1
#define RA_ETHOSU_ACTION_STREAM       2
#define RA_ETHOSU_ACTION_NOP          5

#define RA_ETHOSU_NPU_SRAM_BASE       0x02000000u
#define RA_ETHOSU_NPU_SRAM_SIZE       0x00100000u
#define RA_ETHOSU_NPU_MRAM_BASE       0x22000000u
#define RA_ETHOSU_NPU_MRAM_SIZE       0x001d4000u

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra8p1_ethosu_sync_s
{
  mutex_t lock;
};

struct ra8p1_ethosu_sem_s
{
  sem_t sem;
};

struct ra8p1_ethosu_dev_s
{
  /* lower must remain first for aie_lowerhalf_s cast compatibility. */

  struct aie_lowerhalf_s lower;
  struct ethosu_driver driver;
  mutex_t initlock;
  uint64_t last_pmu_cycles;
  int last_result;
  bool initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra8p1_ethosu_model_init(FAR struct aie_lowerhalf_s *lower,
                                    uintptr_t model);
static int ra8p1_ethosu_model_deinit(FAR struct aie_lowerhalf_s *lower,
                                      int id);
static int ra8p1_ethosu_feed_input(FAR struct aie_lowerhalf_s *lower,
                                    int id, uintptr_t input);
static int ra8p1_ethosu_get_output(FAR struct aie_lowerhalf_s *lower,
                                   int id, uintptr_t output);
static int ra8p1_ethosu_control(FAR struct aie_lowerhalf_s *lower, int id,
                                int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct aie_ops_s g_ethosu_ops =
{
  .init = ra8p1_ethosu_model_init,
  .deinit = ra8p1_ethosu_model_deinit,
  .feed_input = ra8p1_ethosu_feed_input,
  .get_output = ra8p1_ethosu_get_output,
  .control = ra8p1_ethosu_control,
};

static struct ra8p1_ethosu_dev_s g_ethosu =
{
  .lower =
  {
    .ops = &g_ethosu_ops,
  },
  .initlock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Only bus-visible secure memory can be used by the NPU.  TCM and
 * peripheral addresses are deliberately excluded.
 */

static bool ra8p1_ethosu_range(uintptr_t base, size_t length,
                               uintptr_t address, size_t size)
{
  return address >= base && size <= length &&
         address - base <= length - size;
}

static bool ra8p1_ethosu_memory(uint64_t address, size_t size)
{
  if (address > UINTPTR_MAX)
    {
      return false;
    }

  if (ra8p1_ethosu_range(RA_ETHOSU_NPU_SRAM_BASE,
                         RA_ETHOSU_NPU_SRAM_SIZE, address, size) ||
      ra8p1_ethosu_range(RA_ETHOSU_NPU_MRAM_BASE,
                         RA_ETHOSU_NPU_MRAM_SIZE, address, size))
    {
      return true;
    }

#ifdef CONFIG_RA8P1_SDRAM
  return ra8p1_ethosu_range(RA8P1_SDRAM_BASE, RA8P1_SDRAM_SIZE,
                            address, size);
#else
  return false;
#endif
}

/* Validate the entire COP1 envelope before starting DMA.  The vendor parser
 * assumes a well-formed payload and starts the stream while parsing it.
 */

static int ra8p1_ethosu_validate(const struct ethosu_invoke_s *invoke)
{
  const uint32_t *words = invoke->custom_data;
  size_t count;
  size_t pos;
  size_t length;
  bool stream = false;
  unsigned int i;

  if (invoke->custom_data_size < 8 ||
      (invoke->custom_data_size & 3) != 0 ||
      ((uintptr_t)words & 15) != 0 ||
      invoke->num_base_addr < 1 ||
      invoke->num_base_addr > ETHOSU_MAX_BASE_ADDR ||
      !ra8p1_ethosu_memory((uintptr_t)words, invoke->custom_data_size))
    {
      return -EINVAL;
    }

  if (words[0] != RA_ETHOSU_COP1_MAGIC)
    {
      return -EINVAL;
    }

  for (i = 0; i < invoke->num_base_addr; i++)
    {
      if ((invoke->base_addr[i] & 15) != 0 ||
          invoke->base_addr_size[i] == 0 ||
          !ra8p1_ethosu_memory(invoke->base_addr[i],
                               invoke->base_addr_size[i]))
        {
          return -EINVAL;
        }
    }

  count = invoke->custom_data_size / sizeof(*words);
  for (pos = 1; pos < count; pos += length)
    {
      switch (words[pos] & 0xff)
        {
          case RA_ETHOSU_ACTION_OPTIMIZER:
            if (stream)
              {
                return -EINVAL;
              }

            length = 3;
            break;

          case RA_ETHOSU_ACTION_STREAM:
            length = 1 + (words[pos] >> 16) +
                     (((words[pos] >> 8) & 0xff) << 16);
            if (stream || length == 1 ||
                ((uintptr_t)&words[pos + 1] & 15) != 0)
              {
                return -EINVAL;
              }

            stream = true;
            break;

          case RA_ETHOSU_ACTION_NOP:
            length = 1;
            break;

          default:
            return -EINVAL;
        }

      if (length > count - pos)
        {
          return -EINVAL;
        }
    }

  return stream ? OK : -EINVAL;
}

static int ra8p1_ethosu_interrupt(int irq, void *context, void *arg)
{
  struct ra8p1_ethosu_dev_s *priv = arg;

  ethosu_irq_handler(&priv->driver);
  ra_clear_ir(irq);
  return OK;
}

/* The generic AIE load/feed/get interface has no tensor sizes or Vela base
 * address metadata, so it cannot safely describe an Ethos-U invocation.
 * Applications use the public ETHOSUIOC_INVOKE extension through the same
 * AIE upper-half device.
 */

static int ra8p1_ethosu_model_init(FAR struct aie_lowerhalf_s *lower,
                                    uintptr_t model)
{
  (void)lower;
  (void)model;
  return -ENOSYS;
}

static int ra8p1_ethosu_model_deinit(FAR struct aie_lowerhalf_s *lower,
                                      int id)
{
  (void)lower;
  (void)id;
  return -ENOSYS;
}

static int ra8p1_ethosu_feed_input(FAR struct aie_lowerhalf_s *lower,
                                    int id, uintptr_t input)
{
  (void)lower;
  (void)id;
  (void)input;
  return -ENOSYS;
}

static int ra8p1_ethosu_get_output(FAR struct aie_lowerhalf_s *lower,
                                   int id, uintptr_t output)
{
  (void)lower;
  (void)id;
  (void)output;
  return -ENOSYS;
}

static int ra8p1_ethosu_control(FAR struct aie_lowerhalf_s *lower, int id,
                                int cmd, unsigned long arg)
{
  struct ra8p1_ethosu_dev_s *priv =
    (FAR struct ra8p1_ethosu_dev_s *)lower;

  (void)id;

  if (cmd == ETHOSUIOC_GET_INFO)
    {
      struct ethosu_info_s *info = (struct ethosu_info_s *)(uintptr_t)arg;
      struct ethosu_driver_version version;
      struct ethosu_hw_info hw;

      if (info == NULL)
        {
          return -EINVAL;
        }

      ethosu_get_driver_version(&version);
      ethosu_get_hw_info(&priv->driver, &hw);
      info->driver_major = version.major;
      info->driver_minor = version.minor;
      info->driver_patch = version.patch;
      info->reserved = 0;
      info->version_status = hw.version.version_status;
      info->version_minor = hw.version.version_minor;
      info->version_major = hw.version.version_major;
      info->product_major = hw.version.product_major;
      info->arch_patch_rev = hw.version.arch_patch_rev;
      info->arch_minor_rev = hw.version.arch_minor_rev;
      info->arch_major_rev = hw.version.arch_major_rev;
      info->macs_per_cc = hw.cfg.macs_per_cc;
      info->cmd_stream_version = hw.cfg.cmd_stream_version;
      info->custom_dma = hw.cfg.custom_dma;
      return OK;
    }
  else if (cmd == ETHOSUIOC_GET_STATS)
    {
      struct ethosu_stats_s *stats =
        (struct ethosu_stats_s *)(uintptr_t)arg;

      if (stats == NULL)
        {
          return -EINVAL;
        }

      stats->pmu_cycles = priv->last_pmu_cycles;
      stats->result = priv->last_result;
      return OK;
    }
  else if (cmd == ETHOSUIOC_INVOKE)
    {
      struct ethosu_invoke_s *invoke =
        (struct ethosu_invoke_s *)(uintptr_t)arg;
      struct ethosu_invoke_s request;
      int ret;

      if (invoke == NULL || invoke->custom_data == NULL ||
          invoke->custom_data_size == 0 || invoke->num_base_addr == 0 ||
          invoke->num_base_addr > ETHOSU_MAX_BASE_ADDR)
        {
          return -EINVAL;
        }

      request = *invoke;
      ret = ra8p1_ethosu_validate(&request);
      if (ret < 0)
        {
          return ret;
        }

      priv->last_pmu_cycles = 0;
      priv->last_result = ETHOSU_JOB_RESULT_ERROR;
      UP_DSB();
      ret = ethosu_invoke_v3(&priv->driver, request.custom_data,
                             request.custom_data_size, request.base_addr,
                             request.base_addr_size, request.num_base_addr,
                             NULL);
      if (ret < 0)
        {
          struct ra8p1_ethosu_sem_s *sync = priv->driver.semaphore;

          /* Do not return ownership of DMA buffers before a confirmed
           * reset.  A failed reset requires restarting the processor.
           */

          up_disable_irq(RA_IRQ_NPU);
          if (ethosu_soft_reset(&priv->driver) != 0)
            {
              up_systemreset();
            }

          ra_clear_ir(RA_IRQ_NPU);
          putreg32(1u << ((RA_IRQ_NPU - RA_IRQ_FIRST) & 31),
                   NVIC_IRQ_CLRPEND(RA_IRQ_NPU - RA_IRQ_FIRST));
          while (nxsem_trywait(&sync->sem) == OK)
            {
            }

          up_enable_irq(RA_IRQ_NPU);
        }

      UP_DSB();
      invoke->pmu_cycles = priv->last_pmu_cycles;
      ret = priv->last_result == ETHOSU_JOB_RESULT_TIMEOUT ?
            -ETIMEDOUT : (ret < 0 ? -EIO : OK);
      return ret;
    }

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void *ethosu_mutex_create(void)
{
  struct ra8p1_ethosu_sync_s *sync = kmm_malloc(sizeof(*sync));

  if (sync != NULL)
    {
      nxmutex_init(&sync->lock);
    }

  return sync;
}

void ethosu_mutex_destroy(void *mutex)
{
  struct ra8p1_ethosu_sync_s *sync = mutex;

  if (sync != NULL)
    {
      nxmutex_destroy(&sync->lock);
      kmm_free(sync);
    }
}

int ethosu_mutex_lock(void *mutex)
{
  struct ra8p1_ethosu_sync_s *sync = mutex;

  return nxmutex_lock(&sync->lock);
}

int ethosu_mutex_unlock(void *mutex)
{
  struct ra8p1_ethosu_sync_s *sync = mutex;

  return nxmutex_unlock(&sync->lock);
}

void *ethosu_semaphore_create(void)
{
  struct ra8p1_ethosu_sem_s *sync = kmm_malloc(sizeof(*sync));

  if (sync != NULL)
    {
      nxsem_init(&sync->sem, 0, 0);
    }

  return sync;
}

void ethosu_semaphore_destroy(void *sem)
{
  struct ra8p1_ethosu_sem_s *sync = sem;

  if (sync != NULL)
    {
      nxsem_destroy(&sync->sem);
      kmm_free(sync);
    }
}

int ethosu_semaphore_take(void *sem, uint64_t timeout)
{
  struct ra8p1_ethosu_sem_s *sync = sem;

  if (timeout == ETHOSU_SEMAPHORE_WAIT_FOREVER)
    {
      return nxsem_wait_uninterruptible(&sync->sem);
    }

  if (timeout > 60000)
    {
      timeout = 60000;
    }

  return nxsem_tickwait_uninterruptible(&sync->sem,
                                        MSEC2TICK((uint32_t)timeout));
}

int ethosu_semaphore_give(void *sem)
{
  struct ra8p1_ethosu_sem_s *sync = sem;

  return nxsem_post(&sync->sem);
}

void ethosu_inference_begin(struct ethosu_driver *driver, void *user_arg)
{
  (void)user_arg;
  ethosu_pmu_enable(driver);
  ethosu_pmu_cycle_reset(driver);
  ethosu_pmu_counter_enable(driver, ETHOSU_PMU_CYCLE_MASK);
}

void ethosu_inference_end(struct ethosu_driver *driver, void *user_arg)
{
  (void)user_arg;
  g_ethosu.last_pmu_cycles = ethosu_pmu_cycle_get(driver);
  g_ethosu.last_result = driver->job.result;
  ethosu_pmu_disable(driver);
}

/****************************************************************************
 * Name: ra8p1_ethosu_initialize
 ****************************************************************************/

int ra8p1_ethosu_initialize(FAR const char *devpath)
{
  irqstate_t flags;
  unsigned int timeout;
  int ret;

  if (devpath == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_ethosu.initlock);
  if (ret < 0)
    {
      return ret;
    }

  if (g_ethosu.initialized)
    {
      nxmutex_unlock(&g_ethosu.initlock);
      return -EALREADY;
    }

  flags = enter_critical_section();
  putreg16(SYSTEM_PRCR_KEY | SYSTEM_PRCR_PRC0 | SYSTEM_PRCR_PRC1,
           RA8P1_SYSTEM_PRCR);

  if (getreg8(RA8P1_SYSTEM_PDCTRNPU) != 0)
    {
      timeout = 10000;
      while ((getreg8(RA8P1_SYSTEM_PDCTRNPU) &
              SYSTEM_PDCTRNPU_STATUS) != SYSTEM_PDCTRNPU_DOWN)
        {
          if (--timeout == 0)
            {
              goto power_timeout;
            }

          up_udelay(1);
        }

      putreg8(0, RA8P1_SYSTEM_PDCTRNPU);
      timeout = 10000;
      while ((getreg8(RA8P1_SYSTEM_PDCTRNPU) &
              SYSTEM_PDCTRNPU_STATUS) != 0)
        {
          if (--timeout == 0)
            {
              goto power_timeout;
            }

          up_udelay(1);
        }
    }

  putreg16(SYSTEM_PRCR_KEY, RA8P1_SYSTEM_PRCR);
  leave_critical_section(flags);

  ra_module_control(RA8P1_MSTPCRA, MSTPCRA_NPU, true);
  ret = ethosu_init(&g_ethosu.driver, (void *)RA8P1_ETHOSU_BASE,
                    NULL, 0, 1, 1);
  if (ret < 0)
    {
      ra_module_control(RA8P1_MSTPCRA, MSTPCRA_NPU, false);
      nxmutex_unlock(&g_ethosu.initlock);
      return -ENODEV;
    }

  ret = irq_attach(RA_IRQ_NPU, ra8p1_ethosu_interrupt, &g_ethosu);
  if (ret < 0)
    {
      ethosu_deinit(&g_ethosu.driver);
      ra_module_control(RA8P1_MSTPCRA, MSTPCRA_NPU, false);
      nxmutex_unlock(&g_ethosu.initlock);
      return ret;
    }

#ifdef CONFIG_ARCH_IRQPRIO
  up_prioritize_irq(RA_IRQ_NPU, 12 << 4);
#endif

  ra_clear_ir(RA_IRQ_NPU);
  up_enable_irq(RA_IRQ_NPU);

  ret = aie_register(devpath, &g_ethosu.lower);
  if (ret < 0)
    {
      up_disable_irq(RA_IRQ_NPU);
      irq_detach(RA_IRQ_NPU);
      ethosu_deinit(&g_ethosu.driver);
      ra_module_control(RA8P1_MSTPCRA, MSTPCRA_NPU, false);
      nxmutex_unlock(&g_ethosu.initlock);
      return ret;
    }

  g_ethosu.initialized = true;
  nxmutex_unlock(&g_ethosu.initlock);
  return OK;

power_timeout:
  putreg16(SYSTEM_PRCR_KEY, RA8P1_SYSTEM_PRCR);
  leave_critical_section(flags);
  nxmutex_unlock(&g_ethosu.initlock);
  return -ETIMEDOUT;
}
