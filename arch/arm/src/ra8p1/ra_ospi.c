/****************************************************************************
 * arch/arm/src/ra8p1/ra_ospi.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/qspi.h>

#include <debug.h>

#include "arm_internal.h"
#include "hardware/ra8p1_ospi.h"
#include "hardware/ra8p1_system.h"
#include "ra_clockconfig.h"
#include "ra_ospi.h"

#ifdef CONFIG_RA8P1_OSPI_B

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_OSPI_FREQUENCY              111111111u
#define RA_OSPI_WRITE_BUFFER_SIZE      64
#define RA_OSPI_TRANSFER_TIMEOUT_US    10000

#define RA_OSPI_DIRECT_READ_CMD        0x03

#define RA_OSPI_COMMAND_UNSUPPORTED \
  (QSPICMD_IDUAL | QSPICMD_IQUAD | QSPICMD_IOCTAL | \
   QSPICMD_OCTALIO | QSPICMD_DTR)

#define RA_OSPI_MEMORY_UNSUPPORTED \
  (QSPIMEM_SCRAMBLE | QSPIMEM_RANDOM | QSPIMEM_IDUAL | \
   QSPIMEM_IQUAD | QSPIMEM_QUADDATA | QSPIMEM_IOCTAL | \
   QSPIMEM_OCTALIO | QSPIMEM_DTR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_ospi_s
{
  struct qspi_dev_s qspi;
  mutex_t lock;
  bool initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra_ospi_lock(FAR struct qspi_dev_s *dev, bool lock);
static uint32_t ra_ospi_setfrequency(FAR struct qspi_dev_s *dev,
                                     uint32_t frequency);
static void ra_ospi_setmode(FAR struct qspi_dev_s *dev,
                            enum qspi_mode_e mode);
static void ra_ospi_setbits(FAR struct qspi_dev_s *dev, int nbits);
static int ra_ospi_command(FAR struct qspi_dev_s *dev,
                           FAR struct qspi_cmdinfo_s *cmdinfo);
static int ra_ospi_memory(FAR struct qspi_dev_s *dev,
                          FAR struct qspi_meminfo_s *meminfo);
static FAR void *ra_ospi_alloc(FAR struct qspi_dev_s *dev, size_t buflen);
static void ra_ospi_free(FAR struct qspi_dev_s *dev, FAR void *buffer);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qspi_ops_s g_ra_ospi_ops =
{
  .lock = ra_ospi_lock,
  .setfrequency = ra_ospi_setfrequency,
  .setmode = ra_ospi_setmode,
  .setbits = ra_ospi_setbits,
  .command = ra_ospi_command,
  .memory = ra_ospi_memory,
  .alloc = ra_ospi_alloc,
  .free = ra_ospi_free,
};

static struct ra_ospi_s g_ra_ospi0 =
{
  .qspi.ops = &g_ra_ospi_ops,
  .lock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ra_ospi_wait(uintptr_t reg, uint32_t mask, uint32_t expected,
                        unsigned int timeout)
{
  unsigned int elapsed;

  for (elapsed = 0; elapsed < timeout; elapsed++)
    {
      if ((getreg32(reg) & mask) == expected)
        {
          return OK;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

static void ra_ospi_module(bool enable)
{
  ra_module_control(RA8P1_MSTPCRB, MSTPCRB_OSPI0, enable);
}

static int ra_ospi_transfer(uint8_t command, uint32_t address,
                            uint8_t address_length, FAR void *buffer,
                            uint8_t data_length, uint8_t dummy_cycles,
                            bool write)
{
  uint64_t data = 0;
  uint32_t descriptor;
  uint32_t status;
  int ret;

  if (address_length > 4 || data_length > sizeof(data) ||
      dummy_cycles > 31 || (data_length > 0 && buffer == NULL))
    {
      return -EINVAL;
    }

  ret = ra_ospi_wait(RA8P1_OSPI_CDCTL0, OSPI_CDCTL0_TRREQ, 0,
                     RA_OSPI_TRANSFER_TIMEOUT_US);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(OSPI_CDCTL0_CS1, RA8P1_OSPI_CDCTL0);
  descriptor = OSPI_CDT_COMMAND_SIZE(1) |
               OSPI_CDT_ADDRESS_SIZE(address_length) |
               OSPI_CDT_DATA_SIZE(data_length) |
               OSPI_CDT_DUMMY_CYCLES(dummy_cycles) |
               OSPI_CDT_COMMAND_1_BYTE(command);
  if (write)
    {
      descriptor |= OSPI_CDT_WRITE;
    }

  putreg32(descriptor, RA8P1_OSPI_CDTBUF(0));
  putreg32(address, RA8P1_OSPI_CDABUF(0));
  if (write && data_length > 0)
    {
      memcpy(&data, buffer, data_length);
      putreg32((uint32_t)data, RA8P1_OSPI_CDD0BUF(0));
      putreg32((uint32_t)(data >> 32), RA8P1_OSPI_CDD1BUF(0));
    }

  putreg32(OSPI_CDCTL0_CS1 | OSPI_CDCTL0_TRREQ,
           RA8P1_OSPI_CDCTL0);
  ret = ra_ospi_wait(RA8P1_OSPI_CDCTL0, OSPI_CDCTL0_TRREQ, 0,
                     RA_OSPI_TRANSFER_TIMEOUT_US);
  status = getreg32(RA8P1_OSPI_INTS);
  putreg32(status, RA8P1_OSPI_INTC);
  if (ret < 0)
    {
      return ret;
    }

  if ((status & OSPI_INTS_ERRORS) != 0)
    {
      spierr("manual transfer error: %08lx\n", (unsigned long)status);
      return -EIO;
    }

  if (!write && data_length > 0)
    {
      data = getreg32(RA8P1_OSPI_CDD0BUF(0));
      data |= (uint64_t)getreg32(RA8P1_OSPI_CDD1BUF(0)) << 32;
      memcpy(buffer, &data, data_length);
    }

  return OK;
}

static int ra_ospi_configure(void)
{
  ra_ospi_module(true);

  modifyreg32(RA8P1_OSPI_BMCTL0, OSPI_BMCTL0_CS1_ACCESS, 0);
  modifyreg32(RA8P1_OSPI_WRAPCFG, 31u << 24,
              OSPI_WRAPCFG_DSSFTCS1(6));
  putreg32(OSPI_LIOCFGCS_CSMIN_2_CLOCKS,
           RA8P1_OSPI_LIOCFGCS(1));
  putreg32(OSPI_CMCFG0_ADDRESS_3_BYTES, RA8P1_OSPI_CMCFG0(1));
  putreg32(OSPI_CMCFG_COMMAND_1_BYTE(RA_OSPI_DIRECT_READ_CMD),
           RA8P1_OSPI_CMCFG1(1));
  putreg32(0, RA8P1_OSPI_CMCFG2(1));
  putreg32(OSPI_BMCFGCH_COMBINE_64, RA8P1_OSPI_BMCFGCH(0));
  putreg32(OSPI_BMCFGCH_COMBINE_64, RA8P1_OSPI_BMCFGCH(1));
  modifyreg32(RA8P1_OSPI_BMCTL0, 0, OSPI_BMCTL0_CS1_ACCESS);
  putreg32(getreg32(RA8P1_OSPI_INTS), RA8P1_OSPI_INTC);

  spiinfo("BMCTL0=%08lx LIOCFG=%08lx CMCFG=%08lx/%08lx/%08lx\n",
          (unsigned long)getreg32(RA8P1_OSPI_BMCTL0),
          (unsigned long)getreg32(RA8P1_OSPI_LIOCFGCS(1)),
          (unsigned long)getreg32(RA8P1_OSPI_CMCFG0(1)),
          (unsigned long)getreg32(RA8P1_OSPI_CMCFG1(1)),
          (unsigned long)getreg32(RA8P1_OSPI_CMCFG2(1)));
  return OK;
}

static int ra_ospi_lock(FAR struct qspi_dev_s *dev, bool lock)
{
  FAR struct ra_ospi_s *priv = (FAR struct ra_ospi_s *)dev;

  return lock ? nxmutex_lock(&priv->lock) : nxmutex_unlock(&priv->lock);
}

static uint32_t ra_ospi_setfrequency(FAR struct qspi_dev_s *dev,
                                     uint32_t frequency)
{
  (void)dev;
  (void)frequency;
  return RA_OSPI_FREQUENCY;
}

static void ra_ospi_setmode(FAR struct qspi_dev_s *dev,
                            enum qspi_mode_e mode)
{
  (void)dev;
  DEBUGASSERT(mode == QSPIDEV_MODE0);
}

static void ra_ospi_setbits(FAR struct qspi_dev_s *dev, int nbits)
{
  (void)dev;
  DEBUGASSERT(nbits == 8);
}

static int ra_ospi_command(FAR struct qspi_dev_s *dev,
                           FAR struct qspi_cmdinfo_s *cmdinfo)
{
  bool read;
  bool write;

  if (cmdinfo == NULL || cmdinfo->cmd > UINT8_MAX ||
      cmdinfo->addrlen > 4 ||
      cmdinfo->buflen > sizeof(uint64_t) ||
      (cmdinfo->flags & RA_OSPI_COMMAND_UNSUPPORTED) != 0)
    {
      return -EINVAL;
    }

  read = QSPICMD_ISREAD(cmdinfo->flags);
  write = QSPICMD_ISWRITE(cmdinfo->flags);
  if ((read && write) ||
      (QSPICMD_ISDATA(cmdinfo->flags) && cmdinfo->buffer == NULL))
    {
      return -EINVAL;
    }

  return ra_ospi_transfer(cmdinfo->cmd, cmdinfo->addr,
                          QSPICMD_ISADDRESS(cmdinfo->flags) ?
                          cmdinfo->addrlen : 0,
                          cmdinfo->buffer,
                          QSPICMD_ISDATA(cmdinfo->flags) ?
                          cmdinfo->buflen : 0,
                          0, !read);
}

static int ra_ospi_memory(FAR struct qspi_dev_s *dev,
                          FAR struct qspi_meminfo_s *meminfo)
{
  uint64_t staging[RA_OSPI_WRITE_BUFFER_SIZE / sizeof(uint64_t)];
  FAR const uint8_t *src;
  uint32_t remaining;
  uint32_t address;
  uint32_t chunk;
  uint32_t offset;
  int ret;

  if (meminfo == NULL || meminfo->buffer == NULL ||
      meminfo->addrlen != 3 ||
      meminfo->cmd > UINT8_MAX ||
      meminfo->addr >= RA8P1_OSPI_CS1_SIZE ||
      meminfo->buflen > RA8P1_OSPI_CS1_SIZE - meminfo->addr ||
      (meminfo->flags & RA_OSPI_MEMORY_UNSUPPORTED) != 0)
    {
      return -EINVAL;
    }

  address = meminfo->addr;
  if (QSPIMEM_ISREAD(meminfo->flags))
    {
      /* The direct-map window is configured once for a standard read.
       * Protocol flags supplied by an MTD driver do not alter the
       * CPU-visible contents of that window.
       */

      memcpy(meminfo->buffer,
             (FAR const void *)(RA8P1_OSPI_CS1_BASE + address),
             meminfo->buflen);
      return OK;
    }

  if ((address & (sizeof(uint64_t) - 1)) != 0 ||
      (meminfo->buflen & (sizeof(uint64_t) - 1)) != 0)
    {
      return -EINVAL;
    }

  modifyreg32(RA8P1_OSPI_BMCTL0, OSPI_BMCTL0_CS1_ACCESS, 0);
  putreg32(OSPI_CMCFG_COMMAND_1_BYTE(meminfo->cmd),
           RA8P1_OSPI_CMCFG2(1));
  modifyreg32(RA8P1_OSPI_BMCTL0, 0, OSPI_BMCTL0_CS1_ACCESS);

  src = (FAR const uint8_t *)meminfo->buffer;
  remaining = meminfo->buflen;
  while (remaining > 0)
    {
      chunk = remaining > RA_OSPI_WRITE_BUFFER_SIZE ?
              RA_OSPI_WRITE_BUFFER_SIZE : remaining;

      memcpy(staging, src, chunk);
      for (offset = 0; offset < chunk; offset += sizeof(uint64_t))
        {
          *(FAR volatile uint64_t *)(RA8P1_OSPI_CS1_BASE + address +
                                    offset) =
            staging[offset / sizeof(uint64_t)];
        }

      UP_DMB();
      putreg32(OSPI_BMCTL1_PUSH_ALL, RA8P1_OSPI_BMCTL1);
      ret = ra_ospi_wait(RA8P1_OSPI_COMSTT, OSPI_COMSTT_BUSY, 0,
                         RA_OSPI_TRANSFER_TIMEOUT_US);
      if (ret < 0)
        {
          return ret;
        }

      src += chunk;
      address += chunk;
      remaining -= chunk;
    }

  return OK;
}

static FAR void *ra_ospi_alloc(FAR struct qspi_dev_s *dev, size_t buflen)
{
  (void)dev;
  return kmm_memalign(sizeof(uint64_t), buflen);
}

static void ra_ospi_free(FAR struct qspi_dev_s *dev, FAR void *buffer)
{
  (void)dev;
  kmm_free(buffer);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_ospi_initialize
 ****************************************************************************/

FAR struct qspi_dev_s *ra_ospi_initialize(int unit, int channel)
{
  int ret;

  if (unit != 0 || channel != 1)
    {
      return NULL;
    }

  ret = nxmutex_lock(&g_ra_ospi0.lock);
  if (ret < 0)
    {
      return NULL;
    }

  if (!g_ra_ospi0.initialized)
    {
      ret = ra_ospi_configure();
      if (ret < 0)
        {
          ra_ospi_module(false);
          nxmutex_unlock(&g_ra_ospi0.lock);
          return NULL;
        }

      g_ra_ospi0.initialized = true;
    }

  nxmutex_unlock(&g_ra_ospi0.lock);
  return &g_ra_ospi0.qspi;
}

#endif /* CONFIG_RA8P1_OSPI_B */
