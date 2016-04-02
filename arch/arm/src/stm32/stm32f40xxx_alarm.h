/****************************************************************************
 * arch/arm/src/include/stm32/stm32f0xxx_alarm.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Neil hancock - delegated to Gregory Nutt Mar 30, 2016
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

#ifndef __ARCH_ARM_SRC_STM32_STM32F40XXX_ALARM_H
#define __ARCH_ARM_SRC_STM32_STM32F40XXX_ALARM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_RTC_ALARM

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef CODE void (*alm_callback_t)(FAR void *arg, unsigned int alarmid);

/* These features are known to map to STM32 RTC from stm32F4xx and appear to
 * map to beyond stm32F4xx and stm32L0xx there appears to be a small variant
 * with stm32F3 but do not map to stm32F0, F1, F2
 */

enum alm_id_e
{
  RTC_ALARMA = 0,              /* RTC ALARM A */
  RTC_ALARMB,                  /* RTC ALARM B */
  RTC_ALARM_LAST
};

/* Structure used to pass parmaters to set a absolute alarm */

struct tm;                     /* Forward reference */
struct alm_setalarm_s
{
  int as_id;                   /* enum alm_id_e */
  struct tm as_time;           /* Alarm expiration time */
  alm_callback_t as_cb;        /* Callback (if non-NULL) */
  FAR void *as_arg;            /* Argument for callback */
};

/* Structure used to pass parmaters to set an alaram relative to the
 * current time.
 */

struct alm_setrelative_s
{
  int asr_id;                  /* enum alm_id_e */
  int asr_minutes;             /* Relative time in minutes */
  alm_callback_t asr_cb;       /* Callback (if non-NULL) */
  FAR void *asr_arg;           /* Argument for callback */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rtc_setalarm
 *
 * Description:
 *   Set an alarm to an asbolute time using associated hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32_rtc_setalarm(struct alm_setalarm_s *alminfo);

/****************************************************************************
 * Name: stm32_rtc_setalarm_rel
 *
 * Description:
 *   Set a relative alarm in minutes using associated hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the relative alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32_rtc_setalarm_rel(struct alm_setrelative_s *alminfo);

#endif /* CONFIG_RTC_ALARM */
#endif /* __ARCH_ARM_SRC_STM32_STM32F40XXX_ALARM_H */
