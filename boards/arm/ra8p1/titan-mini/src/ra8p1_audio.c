/****************************************************************************
 * boards/arm/ra8p1/titan-mini/src/ra8p1_audio.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>

#include <errno.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/audio_i2s.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/ra8p1_gpt.h"
#include "hardware/ra8p1_system.h"
#include "ra_clockconfig.h"
#include "ra_i2c.h"
#include "ra_i2s.h"
#include "ra_pinmux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA8P1_ES8156_BUS       1
#define RA8P1_ES8156_ADDRESS   0x08
#define RA8P1_ES8156_FREQUENCY 400000

#define RA8P1_AUDIO_MCLK_PERIOD 49
#define RA8P1_AUDIO_MCLK_DUTY   24

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra8p1_es8156_reg_s
{
  uint8_t reg;
  uint8_t value;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ra8p1_es8156_reg_s g_es8156_init[] =
{
  { 0x02, 0x04 },
  { 0x13, 0x00 },
  { 0x20, 0x2a },
  { 0x21, 0x3c },
  { 0x22, 0x02 },
  { 0x24, 0x07 },
  { 0x23, 0x40 },
  { 0x0a, 0x01 },
  { 0x0b, 0x01 },
  { 0x06, 0x11 },
  { 0x18, 0x10 },
  { 0x11, 0x30 },
  { 0x0d, 0x14 },
  { 0x18, 0x08 },
  { 0x19, 0x00 },
  { 0x08, 0x3f },
  { 0x00, 0x02 },
  { 0x00, 0x03 },
  { 0x25, 0x20 },
  { 0x14, CONFIG_TITAN_MINI_AUDIO_VOLUME },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ra8p1_audio_mclk_initialize(void)
{
  uintptr_t base = RA8P1_GPT2_BASE;

  ra_module_control(RA8P1_MSTPCRE, MSTPCRE_GPT2, true);

  putreg32(GPT_GTWP_DISABLE,
           RA8P1_GPT_REG(base, GTWP));
  putreg32(GPT_CHANNEL(2),
           RA8P1_GPT_REG(base, GTSTP));
  putreg32(0, RA8P1_GPT_REG(base, GTCR));
  putreg32(GPT_GROUP_SOFTWARE_UPDATE,
           RA8P1_GPT_REG(base, GTSSR));
  putreg32(GPT_GROUP_SOFTWARE_UPDATE,
           RA8P1_GPT_REG(base, GTPSR));
  putreg32(GPT_GROUP_SOFTWARE_UPDATE,
           RA8P1_GPT_REG(base, GTCSR));
  putreg32(0, RA8P1_GPT_REG(base, GTCNT));
  putreg32(RA8P1_AUDIO_MCLK_PERIOD - 1,
           RA8P1_GPT_REG(base, GTPR));
  putreg32(RA8P1_AUDIO_MCLK_DUTY - 1,
           RA8P1_GPT_REG(base, GTCCRA));
  putreg32(GPT_GTIOR_GTIOA_PWM | GPT_GTIOR_OAE,
           RA8P1_GPT_REG(base, GTIOR));
  putreg32(3, RA8P1_GPT_REG(base, GTUDDTYC));
  putreg32(1, RA8P1_GPT_REG(base, GTUDDTYC));
  putreg32(GPT_CHANNEL(2),
           RA8P1_GPT_REG(base, GTCLR));
  putreg32(GPT_GTWP_ENABLE,
           RA8P1_GPT_REG(base, GTWP));
  putreg32(GPT_CHANNEL(2),
           RA8P1_GPT_REG(base, GTSTR));
}

static int ra8p1_es8156_write(FAR struct i2c_master_s *i2c,
                              uint8_t reg, uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t data[2] =
  {
    reg, value
  };

  msg.frequency = RA8P1_ES8156_FREQUENCY;
  msg.addr = RA8P1_ES8156_ADDRESS;
  msg.flags = 0;
  msg.buffer = data;
  msg.length = sizeof(data);
  return I2C_TRANSFER(i2c, &msg, 1);
}

static int ra8p1_es8156_read(FAR struct i2c_master_s *i2c,
                             uint8_t reg, FAR uint8_t *value)
{
  struct i2c_config_s config =
  {
    .frequency = RA8P1_ES8156_FREQUENCY,
    .address   = RA8P1_ES8156_ADDRESS,
    .addrlen   = 7,
  };

  return i2c_writeread(i2c, &config, &reg, 1, value, 1);
}

static int ra8p1_es8156_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  uint8_t chip_id;
  uint8_t state;
  unsigned int i;
  int ret;

  i2c = ra_i2cbus_initialize(RA8P1_ES8156_BUS);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  ret = ra8p1_es8156_read(i2c, 0xfd, &chip_id);
  if (ret < 0)
    {
      goto out;
    }

  if (chip_id != 0x81)
    {
      syslog(LOG_ERR, "ERROR: ES8156 unexpected chip ID: 0x%02x\n",
             chip_id);
      ret = -ENODEV;
      goto out;
    }

  for (i = 0; i < nitems(g_es8156_init); i++)
    {
      ret = ra8p1_es8156_write(i2c, g_es8156_init[i].reg,
                               g_es8156_init[i].value);
      if (ret < 0)
        {
          goto out;
        }
    }

  ret = ra8p1_es8156_read(i2c, 0x0c, &state);
  if (ret == OK && state != 0x60)
    {
      syslog(LOG_ERR, "ERROR: ES8156 unexpected state: 0x%02x\n",
             state);
      ret = -EIO;
    }

out:
  ra_i2cbus_uninitialize(i2c);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ra8p1_audio_initialize(void)
{
  FAR struct audio_lowerhalf_s *audio;
  FAR struct audio_lowerhalf_s *pcm;
  FAR struct i2s_dev_s *i2s;
  int ret;

  ra_pinmux_configure(BOARD_AUDIO_BCLK_PIN);
  ra_pinmux_configure(BOARD_AUDIO_WCLK_PIN);
  ra_pinmux_configure(BOARD_AUDIO_DATA_PIN);
  ra_pinmux_configure(BOARD_AUDIO_MCLK_PIN);

  ret = ra8p1_es8156_initialize();
  if (ret < 0)
    {
      return ret;
    }

  ra8p1_audio_mclk_initialize();

  i2s = ra_i2s_initialize(0);
  if (i2s == NULL)
    {
      return -ENODEV;
    }

  audio = audio_i2s_initialize(i2s, true);
  if (audio == NULL)
    {
      return -ENOMEM;
    }

  pcm = pcm_decode_initialize(audio);
  if (pcm == NULL)
    {
      return -ENOMEM;
    }

  ret = audio_register("pcm0", pcm);
  if (ret == OK)
    {
      syslog(LOG_INFO, "ES8156 audio registered at /dev/audio/pcm0\n");
    }

  return ret;
}
