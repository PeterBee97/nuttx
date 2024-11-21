/****************************************************************************
 * include/nuttx/audio/pdm.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_PDM_H
#define __INCLUDE_NUTTX_AUDIO_PDM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/audio/audio.h>

#ifdef CONFIG_AUDIO_PDM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Access macros ************************************************************/

/****************************************************************************
 * Name: PDM_RXCHANNELS
 *
 * Description:
 *   Set the PDM RX channel num. NOTE: This may also have unexpected side-
 *   effects of the RX channel num is coupled with the TX channel num.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   channel - The PDM channel num
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#define PDM_RXCHANNELS(d,c) \
  ((d)->ops->pdm_rxchannels ? (d)->ops->pdm_rxchannels(d,c) : -ENOTTY)

/****************************************************************************
 * Name: PDM_RXSAMPLERATE
 *
 * Description:
 *   Set the PDM RX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an PDM receiver or if (2) the sample rate is
 *   driven by the PDM frame clock.  This may also have unexpected side-
 *   effects of the RX sample is coupled with the TX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The PDM sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define PDM_RXSAMPLERATE(d,r) \
  ((d)->ops->pdm_rxsamplerate ? (d)->ops->pdm_rxsamplerate(d,r) : -ENOTTY)

/****************************************************************************
 * Name: PDM_RXDATAWIDTH
 *
 * Description:
 *   Set the PDM RX data width.  The RX bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The PDM data with in bits.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define PDM_RXDATAWIDTH(d,b) \
  ((d)->ops->pdm_rxdatawidth ? (d)->ops->pdm_rxdatawidth(d,b) : -ENOTTY)

/****************************************************************************
 * Name: PDM_RECEIVE
 *
 * Description:
 *   Receive a block of data from PDM.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer in which to receive data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete.
 *   timeout  - The timeout value to use.  The transfer will be canceled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

#define PDM_RECEIVE(d,b,c,a,t) \
  ((d)->ops->pdm_receive ? (d)->ops->pdm_receive(d,b,c,a,t) : -ENOTTY)

/****************************************************************************
 * Name: PDM_TXCHANNELS
 *
 * Description:
 *   Set the PDM TX channel num. NOTE: This may also have unexpected side-
 *   effects of the TX channel num is coupled with the RX channel num.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   channel - The PDM channel num
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#define PDM_TXCHANNELS(d,c) \
  ((d)->ops->pdm_txchannels ? (d)->ops->pdm_txchannels(d,c) : -ENOTTY)

/****************************************************************************
 * Name: PDM_TXSAMPLERATE
 *
 * Description:
 *   Set the PDM TX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an PDM transmitter or if (2) the sample rate is
 *   driven by the PDM frame clock.  This may also have unexpected side-
 *   effects of the TX sample is coupled with the RX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The PDM sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define PDM_TXSAMPLERATE(d,r) \
  ((d)->ops->pdm_txsamplerate ? (d)->ops->pdm_txsamplerate(d,r) : -ENOTTY)

/****************************************************************************
 * Name: PDM_TXDATAWIDTH
 *
 * Description:
 *   Set the PDM TX data width.  The TX bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The PDM data with in bits.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define PDM_TXDATAWIDTH(d,b) \
  ((d)->ops->pdm_txdatawidth ? (d)->ops->pdm_txdatawidth(d,b) : -ENOTTY)

/****************************************************************************
 * Name: PDM_SEND
 *
 * Description:
 *   Send a block of data on PDM.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer from which to send data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer completes.
 *   timeout  - The timeout value to use.  The transfer will be cancelled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

#define PDM_SEND(d,b,c,a,t) \
  ((d)->ops->pdm_send ? (d)->ops->pdm_send(d,b,c,a,t) : -ENOTTY)

/****************************************************************************
 * Name: PDM_GETMCLKFREQUENCY
 *
 * Description:
 *   Get the current master clock frequency. NOTE: this parameter may not
 *   be implemented on PDM driver. If not implemented, the PDM may set
 *   internally any value to the master clock (or even does not support it).
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *
 * Returned Value:
 *   Returns the current master clock.
 *
 ****************************************************************************/

#define PDM_GETMCLKFREQUENCY(d) \
  ((d)->ops->pdm_getmclkfrequency ? \
   (d)->ops->pdm_getmclkfrequency(d) : -ENOTTY)

/****************************************************************************
 * Name: PDM_SETMCLKFREQUENCY
 *
 * Description:
 *   Set the master clock frequency. Usually, the MCLK is a multiple of the
 *   sample rate. Most of the audio codecs require setting specific MCLK
 *   frequency according to the sample rate. NOTE: this parameter may not
 *   be implemented on PDM driver. If not implemented, the PDM may set
 *   internally any value to the master clock (or even does not support it).
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *   frequency  - The PDM master clock's frequency
 *
 * Returned Value:
 *   Returns the resulting master clock or a negated errno value on failure.
 *
 ****************************************************************************/

#define PDM_SETMCLKFREQUENCY(d,f) \
  ((d)->ops->pdm_setmclkfrequency ? \
   (d)->ops->pdm_setmclkfrequency(d,f) : -ENOTTY)

/****************************************************************************
 * Name: PDM_IOCTL
 *
 * Description:
 *   IOCTL of PDM.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   cmd      - A pointer to the audio buffer from which to send data
 *   arg      - An opaque argument that will be provided to the callback
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#define PDM_IOCTL(d,c,a) \
  ((d)->ops->pdm_ioctl ? (d)->ops->pdm_ioctl(d,c,a) : -ENOTTY)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Transfer complete callbacks */

struct pdm_dev_s;
typedef CODE void (*pdm_callback_t)(FAR struct pdm_dev_s *dev,
                   FAR struct ap_buffer_s *apb, FAR void *arg, int result);

/* The PDM vtable */

struct pdm_ops_s
{
  /* Receiver methods */

  CODE int      (*pdm_rxchannels)(FAR struct pdm_dev_s *dev,
                                  uint8_t channels);
  CODE uint32_t (*pdm_rxsamplerate)(FAR struct pdm_dev_s *dev,
                                    uint32_t rate);
  CODE uint32_t (*pdm_rxdatawidth)(FAR struct pdm_dev_s *dev,
                                   int bits);
  CODE int      (*pdm_receive)(FAR struct pdm_dev_s *dev,
                               FAR struct ap_buffer_s *apb,
                               pdm_callback_t callback,
                               FAR void *arg,
                               uint32_t timeout);

  /* Transmitter methods */

  CODE int      (*pdm_txchannels)(FAR struct pdm_dev_s *dev,
                                  uint8_t channels);
  CODE uint32_t (*pdm_txsamplerate)(FAR struct pdm_dev_s *dev,
                                    uint32_t rate);
  CODE uint32_t (*pdm_txdatawidth)(FAR struct pdm_dev_s *dev,
                                   int bits);
  CODE int      (*pdm_send)(FAR struct pdm_dev_s *dev,
                            FAR struct ap_buffer_s *apb,
                            pdm_callback_t callback,
                            FAR void *arg,
                            uint32_t timeout);

  /* Master Clock methods */

  CODE uint32_t (*pdm_getmclkfrequency)(FAR struct pdm_dev_s *dev);
  CODE uint32_t (*pdm_setmclkfrequency)(FAR struct pdm_dev_s *dev,
                                        uint32_t frequency);

  /* Ioctl */

  CODE int      (*pdm_ioctl)(FAR struct pdm_dev_s *dev,
                             int cmd, unsigned long arg);
};

/* PDM private data.  This structure only defines the initial fields of the
 * structure visible to the PDM client.  The specific implementation may
 * add additional, device specific fields
 */

struct pdm_dev_s
{
  FAR const struct pdm_ops_s *ops;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pdmchar_register
 *
 * Description:
 *   Create and register the PDM character driver.
 *
 *   The PDM character driver is a simple character driver that supports PDM
 *   transfers via a read() and write().  The intent of this driver is to
 *   support PDM testing.  It is not an audio driver but does conform to some
 *   of the buffer management heuristics of an audio driver.  It is not
 *   suitable for use in any real driver application in its current form.
 *
 * Input Parameters:
 *   pdm - An instance of the lower half PDM driver
 *   minor - The device minor number.  The PDM character device will be
 *     registers as /dev/pdmcharN where N is the minor number
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int pdmchar_register(FAR struct pdm_dev_s *pdm, int minor);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_PDM */
#endif /* __INCLUDE_NUTTX_AUDIO_PDM_H */
