/****************************************************************************
 * drivers/audio/audio_pdm.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/pdm.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct audio_pdm_s
{
  struct audio_lowerhalf_s dev;
  FAR struct pdm_dev_s *pdm;
  bool playback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int audio_pdm_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                             FAR struct audio_caps_s *caps);
static int audio_pdm_shutdown(FAR struct audio_lowerhalf_s *dev);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_pdm_configure(FAR struct audio_lowerhalf_s *dev,
                               FAR void *session,
                               FAR const struct audio_caps_s *caps);
static int audio_pdm_start(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int audio_pdm_stop(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int audio_pdm_pause(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session);
static int audio_pdm_resume(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session);
#endif
static int audio_pdm_reserve(FAR struct audio_lowerhalf_s *dev,
                             FAR void **session);
static int audio_pdm_release(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session);
#else
static int audio_pdm_configure(FAR struct audio_lowerhalf_s *dev,
                               FAR const struct audio_caps_s *caps);
static int audio_pdm_start(FAR struct audio_lowerhalf_s *dev);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int audio_pdm_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int audio_pdm_pause(FAR struct audio_lowerhalf_s *dev);
static int audio_pdm_resume(FAR struct audio_lowerhalf_s *dev);
#endif
static int audio_pdm_reserve(FAR struct audio_lowerhalf_s *dev);
static int audio_pdm_release(FAR struct audio_lowerhalf_s *dev);
#endif
static int audio_pdm_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct audio_buf_desc_s *bufdesc);
static int audio_pdm_freebuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct audio_buf_desc_s *bufdesc);
static int audio_pdm_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                   FAR struct ap_buffer_s *apb);
static int audio_pdm_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                           unsigned long arg);
static void audio_pdm_callback(FAR struct pdm_dev_s *dev,
                               FAR struct ap_buffer_s *apb, FAR void *arg,
                               int result);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audio_pdm_ops =
{
  audio_pdm_getcaps,       /* getcaps        */
  audio_pdm_configure,     /* configure      */
  audio_pdm_shutdown,      /* shutdown       */
  audio_pdm_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  audio_pdm_stop,          /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  audio_pdm_pause,         /* pause          */
  audio_pdm_resume,        /* resume         */
#endif
  audio_pdm_allocbuffer,   /* allocbuffer    */
  audio_pdm_freebuffer,    /* freebuffer     */
  audio_pdm_enqueuebuffer, /* enqueue_buffer */
  NULL,                    /* cancel_buffer  */
  audio_pdm_ioctl,         /* ioctl          */
  NULL,                    /* read           */
  NULL,                    /* write          */
  audio_pdm_reserve,       /* reserve        */
  audio_pdm_release        /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int audio_pdm_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                             FAR struct audio_caps_s *caps)
{
  FAR struct audio_pdm_s *audio_pdm = (FAR struct audio_pdm_s *)dev;
  FAR struct pdm_dev_s *pdm = audio_pdm->pdm;

  /* Validate the structure */

  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));
  audinfo("type=%d ac_type=%d\n", type, caps->ac_type);

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      /* Caller is querying for the types of units we support */

      case AUDIO_TYPE_QUERY:

        /* Provide our overall capabilities.  The interfacing software
         * must then call us back for specific info for each capability.
         */

        if (caps->ac_subtype == AUDIO_TYPE_QUERY)
          {
              /* We don't decode any formats!  Only something above us in
               * the audio stream can perform decoding on our behalf.
               */

              /* The types of audio units we implement */

              if (audio_pdm->playback)
                {
                  caps->ac_controls.b[0] = AUDIO_TYPE_OUTPUT;
                }
              else
                {
                  caps->ac_controls.b[0] = AUDIO_TYPE_INPUT;
                }

              caps->ac_format.hw = 1 << (AUDIO_FMT_PCM - 1);
              break;
          }

         caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
         break;

        /* Provide capabilities of our OUTPUT unit */

      case AUDIO_TYPE_OUTPUT:
      case AUDIO_TYPE_INPUT:

        if (caps->ac_subtype == AUDIO_TYPE_QUERY)
          {
            /* Report the Sample rates we support */

              caps->ac_controls.hw[0] = AUDIO_SAMP_RATE_DEF_ALL;

              caps->ac_channels = 2;

              break;
          }

      default:
        PDM_IOCTL(pdm, AUDIOIOC_GETCAPS, (unsigned long)caps);
        break;
    }

  return caps->ac_len;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_pdm_configure(FAR struct audio_lowerhalf_s *dev,
                               FAR void *session,
                               FAR const struct audio_caps_s *caps)
#else
static int audio_pdm_configure(FAR struct audio_lowerhalf_s *dev,
                               FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct audio_pdm_s *audio_pdm = (FAR struct audio_pdm_s *)dev;
  FAR struct pdm_dev_s *pdm;
  int samprate;
  int nchannels;
  int bpsamp;
  int ret = OK;

  DEBUGASSERT(audio_pdm != NULL && caps != NULL);
  pdm = audio_pdm->pdm;
  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_OUTPUT:
      case AUDIO_TYPE_INPUT:

        /* Save the current stream configuration */

        samprate  = caps->ac_controls.hw[0] |
                    (caps->ac_controls.b[3] << 16);
        nchannels = caps->ac_channels;
        bpsamp    = caps->ac_controls.b[2];

        if (audio_pdm->playback)
          {
            PDM_TXCHANNELS(pdm, nchannels);
            PDM_TXDATAWIDTH(pdm, bpsamp);
            PDM_TXSAMPLERATE(pdm, samprate);
          }
        else
          {
            PDM_RXCHANNELS(pdm, nchannels);
            PDM_RXDATAWIDTH(pdm, bpsamp);
            PDM_RXSAMPLERATE(pdm, samprate);
          }
        break;

      default:
        ret = PDM_IOCTL(pdm, AUDIOIOC_CONFIGURE, (unsigned long)caps);
        break;
    }

  return ret;
}

static int audio_pdm_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct audio_pdm_s *audio_pdm = (FAR struct audio_pdm_s *)dev;
  FAR struct pdm_dev_s *pdm = audio_pdm->pdm;

  return PDM_IOCTL(pdm, AUDIOIOC_SHUTDOWN, audio_pdm->playback);
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_pdm_start(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session)
#else
static int audio_pdm_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_pdm_s *audio_pdm = (FAR struct audio_pdm_s *)dev;
  FAR struct pdm_dev_s *pdm = audio_pdm->pdm;

  return PDM_IOCTL(pdm, AUDIOIOC_START, audio_pdm->playback);
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_pdm_stop(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session)
#else
static int audio_pdm_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_pdm_s *audio_pdm = (FAR struct audio_pdm_s *)dev;
  FAR struct pdm_dev_s *pdm = audio_pdm->pdm;

  return PDM_IOCTL(pdm, AUDIOIOC_STOP, audio_pdm->playback);
}
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_pdm_pause(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session)
#else
static int audio_pdm_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_pdm_s *audio_pdm = (FAR struct audio_pdm_s *)dev;
  FAR struct pdm_dev_s *pdm = audio_pdm->pdm;

  return PDM_IOCTL(pdm, AUDIOIOC_PAUSE, audio_pdm->playback);
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_pdm_resume(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session)
#else
static int audio_pdm_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_pdm_s *audio_pdm = (FAR struct audio_pdm_s *)dev;
  FAR struct pdm_dev_s *pdm = audio_pdm->pdm;

  return PDM_IOCTL(pdm, AUDIOIOC_RESUME, audio_pdm->playback);
}
#endif

static int audio_pdm_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct audio_buf_desc_s *bufdesc)
{
  FAR struct audio_pdm_s *audio_pdm = (FAR struct audio_pdm_s *)dev;
  FAR struct pdm_dev_s *pdm = audio_pdm->pdm;

  return PDM_IOCTL(pdm, AUDIOIOC_ALLOCBUFFER, (unsigned long)bufdesc);
}

static int audio_pdm_freebuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct audio_buf_desc_s *bufdesc)
{
  FAR struct audio_pdm_s *audio_pdm = (FAR struct audio_pdm_s *)dev;
  FAR struct pdm_dev_s *pdm = audio_pdm->pdm;

  return PDM_IOCTL(pdm, AUDIOIOC_FREEBUFFER, (unsigned long)bufdesc);
}

static int audio_pdm_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                   FAR struct ap_buffer_s *apb)
{
  FAR struct audio_pdm_s *audio_pdm = (FAR struct audio_pdm_s *)dev;
  FAR struct pdm_dev_s *pdm = audio_pdm->pdm;

  if (audio_pdm->playback)
    {
      return PDM_SEND(pdm, apb, audio_pdm_callback, audio_pdm, 0);
    }
  else
    {
      return PDM_RECEIVE(pdm, apb, audio_pdm_callback, audio_pdm, 0);
    }
}

static int audio_pdm_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                           unsigned long arg)
{
  FAR struct audio_pdm_s *audio_pdm = (FAR struct audio_pdm_s *)dev;
  FAR struct pdm_dev_s *pdm = audio_pdm->pdm;

  return PDM_IOCTL(pdm, cmd, arg);
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_pdm_reserve(FAR struct audio_lowerhalf_s *dev,
                             FAR void **session)
#else
static int audio_pdm_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
#ifdef CONFIG_AUDIO_MULTI_SESSION
  *session = (FAR void *)audio_pdm->playback;
#endif
  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_pdm_release(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session)
#else
static int audio_pdm_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  return OK;
}

static void audio_pdm_callback(FAR struct pdm_dev_s *dev,
                               FAR struct ap_buffer_s *apb,
                               FAR void *arg, int result)
{
  FAR struct audio_pdm_s *audio_pdm = arg;
  bool final = false;

  if ((apb->flags & AUDIO_APB_FINAL) != 0)
    {
      final = true;
    }

#ifdef CONFIG_AUDIO_MULTI_SESSION
  audio_pdm->dev.upper(audio_pdm->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb,
                       OK, NULL);
#else
  audio_pdm->dev.upper(audio_pdm->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb,
                       OK);
#endif
  if (final)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      audio_pdm->dev.upper(audio_pdm->dev.priv, AUDIO_CALLBACK_COMPLETE,
                           NULL, OK, NULL);
#else
      audio_pdm->dev.upper(audio_pdm->dev.priv, AUDIO_CALLBACK_COMPLETE,
                           NULL, OK);
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct audio_lowerhalf_s *audio_pdm_initialize(FAR struct pdm_dev_s *pdm,
                                                   bool playback)
{
  FAR struct audio_pdm_s *audio_pdm;

  if (pdm == NULL)
    {
      return NULL;
    }

  audio_pdm = kmm_zalloc(sizeof(struct audio_pdm_s));
  if (audio_pdm == NULL)
    {
      return NULL;
    }

  audio_pdm->playback = playback;
  audio_pdm->pdm = pdm;
  audio_pdm->dev.ops = &g_audio_pdm_ops;

  return &audio_pdm->dev;
}
