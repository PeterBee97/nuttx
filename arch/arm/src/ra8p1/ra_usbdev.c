/****************************************************************************
 * arch/arm/src/ra8p1/ra_usbdev.c
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

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "arm_internal.h"
#include "hardware/ra8p1_system.h"
#include "hardware/ra8p1_usbfs.h"
#include "ra_clockconfig.h"
#include "ra_icu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_USB_NPIPES                10
#define RA_USB_EP0_MAXPACKET         64
#define RA_USB_SETUP_MAXDATA         256
#define RA_USB_FIFO_WAIT             100000

#define RA_USB_PIPE_MIN_BULK         1
#define RA_USB_PIPE_MAX_BULK         5
#define RA_USB_PIPE_MIN_INT          6
#define RA_USB_PIPE_MAX_INT          9

#define RA_USB_INTENB0_DEFAULT \
  (USBFS_INT_BEMP | USBFS_INT_BRDY | USBFS_INT_NRDY | USBFS_INT_CTRT | \
   USBFS_INT_DVST | USBFS_INT_RESM | USBFS_INT_VBINT)

#define RA_TRACEERR_ALLOCFAIL        1
#define RA_TRACEERR_BINDFAILED       2
#define RA_TRACEERR_DRIVER           3
#define RA_TRACEERR_FIFO             4
#define RA_TRACEERR_INVALIDPARMS     5
#define RA_TRACEERR_IRQ              6
#define RA_TRACEERR_NOEP             7
#define RA_TRACEERR_SETUP            8

#define RA_TRACEINTID_SETUP          1
#define RA_TRACEINTID_DISPATCH       2
#define RA_TRACEINTID_RESET          3
#define RA_TRACEINTID_SUSPEND        4
#define RA_TRACEINTID_RESUME         5

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_usbdev_s;

struct ra_usb_req_s
{
  struct usbdev_req_s req;
  FAR struct ra_usb_req_s *flink;
};

struct ra_usb_ep_s
{
  struct usbdev_ep_s ep;
  FAR struct ra_usbdev_s *dev;
  FAR struct ra_usb_req_s *head;
  FAR struct ra_usb_req_s *tail;
  uint8_t pipe;
  uint8_t eptype;
  bool in;
  bool allocated;
  bool configured;
  bool stalled;
  bool active;
  bool zlp;
};

struct ra_usbdev_s
{
  struct usbdev_s usbdev;
  FAR struct usbdevclass_driver_s *driver;
  struct ra_usb_ep_s eplist[RA_USB_NPIPES];
  struct usb_ctrlreq_s ctrl;
  uint8_t ep0data[RA_USB_SETUP_MAXDATA];
  uint16_t ep0expected;
  uint16_t ep0received;
  uint16_t ep0internal_len;
  uint16_t ep0internal_xfrd;
  bool ep0out;
  bool ep0internal;
  bool ep0status;
  bool ep0status_req;
  bool selfpowered;
  bool remotewakeup;
  bool pullup;
  bool suspended;
  bool initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra_usb_epconfigure(FAR struct usbdev_ep_s *ep,
                              FAR const struct usb_epdesc_s *desc,
                              bool last);
static int ra_usb_epdisable(FAR struct usbdev_ep_s *ep);
static FAR struct usbdev_req_s *
ra_usb_epallocreq(FAR struct usbdev_ep_s *ep);
static void ra_usb_epfreereq(FAR struct usbdev_ep_s *ep,
                             FAR struct usbdev_req_s *req);
static int ra_usb_epsubmit(FAR struct usbdev_ep_s *ep,
                           FAR struct usbdev_req_s *req);
static int ra_usb_epcancel(FAR struct usbdev_ep_s *ep,
                           FAR struct usbdev_req_s *req);
static int ra_usb_epstall(FAR struct usbdev_ep_s *ep, bool resume);

static FAR struct usbdev_ep_s *
ra_usb_allocep(FAR struct usbdev_s *dev, uint8_t eplog, bool in,
               uint8_t eptype);
static void ra_usb_freeep(FAR struct usbdev_s *dev,
                          FAR struct usbdev_ep_s *ep);
static int ra_usb_getframe(FAR struct usbdev_s *dev);
static int ra_usb_wakeup(FAR struct usbdev_s *dev);
static int ra_usb_selfpowered(FAR struct usbdev_s *dev, bool selfpowered);
static int ra_usb_pullup(FAR struct usbdev_s *dev, bool enable);
static int ra_usb_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_deverror[] =
{
  TRACE_STR(RA_TRACEERR_ALLOCFAIL),
  TRACE_STR(RA_TRACEERR_BINDFAILED),
  TRACE_STR(RA_TRACEERR_DRIVER),
  TRACE_STR(RA_TRACEERR_FIFO),
  TRACE_STR(RA_TRACEERR_INVALIDPARMS),
  TRACE_STR(RA_TRACEERR_IRQ),
  TRACE_STR(RA_TRACEERR_NOEP),
  TRACE_STR(RA_TRACEERR_SETUP),
  TRACE_STR_END
};

const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  TRACE_STR(RA_TRACEINTID_SETUP),
  TRACE_STR(RA_TRACEINTID_DISPATCH),
  TRACE_STR(RA_TRACEINTID_RESET),
  TRACE_STR(RA_TRACEINTID_SUSPEND),
  TRACE_STR(RA_TRACEINTID_RESUME),
  TRACE_STR_END
};
#endif

static const struct usbdev_epops_s g_ra_usb_epops =
{
  .configure = ra_usb_epconfigure,
  .disable   = ra_usb_epdisable,
  .allocreq  = ra_usb_epallocreq,
  .freereq   = ra_usb_epfreereq,
  .submit    = ra_usb_epsubmit,
  .cancel    = ra_usb_epcancel,
  .stall     = ra_usb_epstall,
  .poll      = NULL
};

static const struct usbdev_ops_s g_ra_usb_devops =
{
  .allocep     = ra_usb_allocep,
  .freeep      = ra_usb_freeep,
  .getframe    = ra_usb_getframe,
  .wakeup      = ra_usb_wakeup,
  .selfpowered = ra_usb_selfpowered,
  .pullup      = ra_usb_pullup,
  .ioctl       = NULL
};

static struct ra_usbdev_s g_ra_usbdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR struct ra_usb_req_s *
ra_usb_req_dequeue(FAR struct ra_usb_ep_s *privep)
{
  FAR struct ra_usb_req_s *req = privep->head;

  if (req != NULL)
    {
      privep->head = req->flink;
      if (privep->head == NULL)
        {
          privep->tail = NULL;
        }

      req->flink = NULL;
    }

  return req;
}

static void ra_usb_req_enqueue(FAR struct ra_usb_ep_s *privep,
                               FAR struct ra_usb_req_s *req)
{
  req->flink = NULL;

  if (privep->head == NULL)
    {
      privep->head = req;
      privep->tail = req;
    }
  else
    {
      privep->tail->flink = req;
      privep->tail = req;
    }
}

static void ra_usb_req_complete(FAR struct ra_usb_ep_s *privep,
                                int16_t result)
{
  FAR struct ra_usb_req_s *privreq;
  irqstate_t flags;

  flags = enter_critical_section();
  privreq = ra_usb_req_dequeue(privep);
  leave_critical_section(flags);

  if (privreq != NULL)
    {
      privreq->req.result = result;
      privreq->req.callback(&privep->ep, &privreq->req);
    }
}

static void ra_usb_cancel_all(FAR struct ra_usb_ep_s *privep,
                              int16_t result)
{
  while (privep->head != NULL)
    {
      ra_usb_req_complete(privep, result);
    }

  privep->active = false;
  privep->zlp = false;
}

static uintptr_t ra_usb_pipectr(uint8_t pipe)
{
  return pipe == 0 ? RA_USBFS_DCPCTR : RA_USBFS_PIPECTR(pipe);
}

static void ra_usb_pipe_set_pid(uint8_t pipe, uint16_t pid)
{
  uintptr_t regaddr = ra_usb_pipectr(pipe);
  uint16_t regval = getreg16(regaddr);
  uint16_t oldpid = regval & USBFS_PIPECTR_PID_MASK;

  if (pid == USBFS_PIPECTR_PID_STALL &&
      oldpid == USBFS_PIPECTR_PID_BUF)
    {
      putreg16(regval | USBFS_PIPECTR_PID_MASK, regaddr);
      regval = getreg16(regaddr);
    }
  else if (oldpid == USBFS_PIPECTR_PID_MASK &&
           pid != USBFS_PIPECTR_PID_STALL)
    {
      putreg16((regval & ~USBFS_PIPECTR_PID_MASK) |
               USBFS_PIPECTR_PID_STALL, regaddr);
      regval = getreg16(regaddr);
    }

  putreg16((regval & ~USBFS_PIPECTR_PID_MASK) | pid, regaddr);
}

static bool ra_usb_pipe_idle(uint8_t pipe)
{
  unsigned int timeout;

  for (timeout = 0; timeout < RA_USB_FIFO_WAIT; timeout++)
    {
      if ((getreg16(ra_usb_pipectr(pipe)) &
           USBFS_PIPECTR_PBUSY) == 0)
        {
          return true;
        }
    }

  return false;
}

static void ra_usb_pipe_counter_stop(uint8_t pipe)
{
  if (pipe >= 1 && pipe <= 5)
    {
      putreg16(USBFS_PIPETRE_TRCLR, RA_USBFS_PIPETRE(pipe));
      putreg16(0, RA_USBFS_PIPETRN(pipe));
    }
}

static void ra_usb_pipe_flush(uint8_t pipe)
{
  uintptr_t regaddr;

  ra_usb_pipe_set_pid(pipe, USBFS_PIPECTR_PID_NAK);
  (void)ra_usb_pipe_idle(pipe);

  if (pipe == 0)
    {
      putreg16(USBFS_FIFOCTR_BCLR, RA_USBFS_CFIFOCTR);
      putreg16(USBFS_PIPECTR_SQCLR, RA_USBFS_DCPCTR);
    }
  else
    {
      regaddr = RA_USBFS_PIPECTR(pipe);
      putreg16(USBFS_PIPECTR_ACLRM | USBFS_PIPECTR_SQCLR,
               regaddr);
      putreg16(0, regaddr);
      ra_usb_pipe_counter_stop(pipe);
    }
}

static bool ra_usb_fifo_select(uint8_t pipe, bool in,
                               FAR uintptr_t *fifo,
                               FAR uintptr_t *fifoctr)
{
  uintptr_t fifosel;
  uint16_t regval;
  uint16_t expected;
  unsigned int timeout;

  if (pipe == 0)
    {
      fifosel = RA_USBFS_CFIFOSEL;
      *fifo = RA_USBFS_CFIFO;
      *fifoctr = RA_USBFS_CFIFOCTR;
      regval = USBFS_FIFOSEL_MBW_16;
      if (in)
        {
          regval |= USBFS_CFIFOSEL_ISEL;
        }

      expected = regval & (USBFS_FIFOSEL_CURPIPE_MASK |
                           USBFS_CFIFOSEL_ISEL);
    }
  else
    {
      fifosel = RA_USBFS_D0FIFOSEL;
      *fifo = RA_USBFS_D0FIFO;
      *fifoctr = RA_USBFS_D0FIFOCTR;
      regval = USBFS_FIFOSEL_MBW_16 | pipe;
      expected = pipe;
    }

  putreg16(regval, fifosel);

  for (timeout = 0; timeout < RA_USB_FIFO_WAIT; timeout++)
    {
      uint16_t current = getreg16(fifosel);

      if ((current & (USBFS_FIFOSEL_CURPIPE_MASK |
                      USBFS_CFIFOSEL_ISEL)) == expected &&
          (getreg16(*fifoctr) & USBFS_FIFOCTR_FRDY) != 0)
        {
          return true;
        }
    }

  usbtrace(TRACE_DEVERROR(RA_TRACEERR_FIFO), pipe);
  return false;
}

static void ra_usb_fifo_deselect(uint8_t pipe)
{
  unsigned int timeout;

  if (pipe == 0)
    {
      return;
    }

  putreg16(USBFS_FIFOSEL_MBW_16, RA_USBFS_D0FIFOSEL);
  for (timeout = 0; timeout < RA_USB_FIFO_WAIT; timeout++)
    {
      if ((getreg16(RA_USBFS_D0FIFOSEL) &
           USBFS_FIFOSEL_CURPIPE_MASK) == 0)
        {
          return;
        }
    }
}

static int ra_usb_fifo_write(uint8_t pipe, FAR const uint8_t *buffer,
                             uint16_t length, uint16_t maxpacket)
{
  uintptr_t fifo;
  uintptr_t fifoctr;
  uint16_t i;

  if (!ra_usb_fifo_select(pipe, true, &fifo, &fifoctr))
    {
      return -EBUSY;
    }

  for (i = 0; i + 1 < length; i += 2)
    {
      putreg16((uint16_t)buffer[i] |
               ((uint16_t)buffer[i + 1] << 8), fifo);
    }

  if (i < length)
    {
      putreg8(buffer[i], fifo);
    }

  if (length < maxpacket)
    {
      putreg16(USBFS_FIFOCTR_BVAL, fifoctr);
    }

  ra_usb_fifo_deselect(pipe);
  return OK;
}

static int ra_usb_fifo_read(uint8_t pipe, FAR uint8_t *buffer,
                            uint16_t buflen, FAR uint16_t *packetlen)
{
  uintptr_t fifo;
  uintptr_t fifoctr;
  uint16_t available;
  uint16_t copied;
  uint16_t i;

  if (!ra_usb_fifo_select(pipe, false, &fifo, &fifoctr))
    {
      return -EBUSY;
    }

  available = getreg16(fifoctr) & USBFS_FIFOCTR_DTLN_MASK;
  copied = available < buflen ? available : buflen;

  for (i = 0; i + 1 < copied; i += 2)
    {
      uint16_t data = getreg16(fifo);

      buffer[i] = data & 0xff;
      buffer[i + 1] = data >> 8;
    }

  if (i < copied)
    {
      buffer[i] = getreg8(fifo);
    }

  if (available != copied || available < 64)
    {
      putreg16(USBFS_FIFOCTR_BCLR, fifoctr);
    }

  ra_usb_fifo_deselect(pipe);
  *packetlen = available;
  return copied;
}

static FAR struct ra_usb_ep_s *
ra_usb_ep_find(FAR struct ra_usbdev_s *priv, uint8_t eplog)
{
  unsigned int pipe;

  if (USB_EPNO(eplog) == 0)
    {
      return &priv->eplist[0];
    }

  for (pipe = 1; pipe < RA_USB_NPIPES; pipe++)
    {
      FAR struct ra_usb_ep_s *privep = &priv->eplist[pipe];

      if (privep->allocated && privep->ep.eplog == eplog)
        {
          return privep;
        }
    }

  return NULL;
}

static int ra_usb_load_in(FAR struct ra_usb_ep_s *privep)
{
  FAR struct ra_usb_req_s *privreq = privep->head;
  size_t remaining;
  uint16_t nbytes;
  int ret;

  if (privreq == NULL)
    {
      return -ENOENT;
    }

  remaining = privreq->req.len - privreq->req.xfrd;
  if (remaining == 0)
    {
      nbytes = 0;
      privep->zlp = false;
    }
  else
    {
      nbytes = remaining > privep->ep.maxpacket ?
               privep->ep.maxpacket : remaining;
    }

  ret = ra_usb_fifo_write(privep->pipe,
                          privreq->req.buf + privreq->req.xfrd,
                          nbytes, privep->ep.maxpacket);
  if (ret >= 0)
    {
      privreq->req.xfrd += nbytes;
    }

  return ret;
}

static int ra_usb_start_in(FAR struct ra_usb_ep_s *privep)
{
  FAR struct ra_usb_req_s *privreq = privep->head;
  int ret;

  if (privreq == NULL || privep->stalled)
    {
      return OK;
    }

  privep->active = true;
  privep->zlp = privreq->req.len > 0 &&
                (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0 &&
                (privreq->req.len % privep->ep.maxpacket) == 0;

  ret = ra_usb_load_in(privep);
  if (ret < 0)
    {
      privep->active = false;
      privep->zlp = false;
    }
  else
    {
      /* EP0 has no configure callback to arm its PID.  Make the loaded
       * packet available to the host, including control IN responses.
       */

      ra_usb_pipe_set_pid(privep->pipe, USBFS_PIPECTR_PID_BUF);
    }

  return ret;
}

static void ra_usb_advance_in(FAR struct ra_usb_ep_s *privep)
{
  FAR struct ra_usb_req_s *privreq = privep->head;
  int ret;

  if (!privep->active || privreq == NULL)
    {
      return;
    }

  if (privreq->req.xfrd < privreq->req.len || privep->zlp)
    {
      ret = ra_usb_load_in(privep);
      if (ret < 0)
        {
          privep->active = false;
          ra_usb_req_complete(privep, ret);
        }

      return;
    }

  privep->active = false;
  if (privep->pipe == 0)
    {
      privep->dev->ep0status = true;
      privep->dev->ep0status_req = false;
      modifyreg16(RA_USBFS_DCPCTR, 0,
                  USBFS_DCPCTR_CCPL | USBFS_PIPECTR_PID_BUF);
    }

  ra_usb_req_complete(privep, OK);
  if (privep->head != NULL && !privep->active)
    {
      (void)ra_usb_start_in(privep);
    }
}

static int ra_usb_start_out(FAR struct ra_usb_ep_s *privep)
{
  FAR struct ra_usb_req_s *privreq = privep->head;
  size_t remaining;
  uint32_t packets;

  if (privreq == NULL || privep->stalled)
    {
      return OK;
    }

  remaining = privreq->req.len - privreq->req.xfrd;
  packets = (remaining + privep->ep.maxpacket - 1) /
            privep->ep.maxpacket;
  if (packets == 0)
    {
      packets = 1;
    }

  ra_usb_pipe_set_pid(privep->pipe, USBFS_PIPECTR_PID_NAK);
  if (!ra_usb_pipe_idle(privep->pipe))
    {
      return -EBUSY;
    }

  if (privep->pipe <= 5)
    {
      putreg16(USBFS_PIPETRE_TRCLR,
               RA_USBFS_PIPETRE(privep->pipe));
      putreg16((uint16_t)packets, RA_USBFS_PIPETRN(privep->pipe));
      putreg16(USBFS_PIPETRE_TRENB,
               RA_USBFS_PIPETRE(privep->pipe));
    }

  privep->active = true;
  ra_usb_pipe_set_pid(privep->pipe, USBFS_PIPECTR_PID_BUF);
  return OK;
}

static void ra_usb_advance_out(FAR struct ra_usb_ep_s *privep)
{
  FAR struct ra_usb_req_s *privreq = privep->head;
  uint16_t packetlen;
  uint16_t remaining;
  int copied;
  int16_t result = OK;
  bool complete;

  if (!privep->active || privreq == NULL)
    {
      ra_usb_pipe_set_pid(privep->pipe, USBFS_PIPECTR_PID_NAK);
      return;
    }

  remaining = privreq->req.len - privreq->req.xfrd;
  copied = ra_usb_fifo_read(privep->pipe,
                            privreq->req.buf + privreq->req.xfrd,
                            remaining, &packetlen);
  if (copied < 0)
    {
      result = copied;
      packetlen = 0;
      copied = 0;
    }
  else if (packetlen > remaining)
    {
      result = -EOVERFLOW;
    }

  privreq->req.xfrd += copied;
  complete = result < 0 ||
             packetlen < privep->ep.maxpacket ||
             privreq->req.xfrd >= privreq->req.len;

  if (!complete)
    {
      return;
    }

  ra_usb_pipe_set_pid(privep->pipe, USBFS_PIPECTR_PID_NAK);
  ra_usb_pipe_counter_stop(privep->pipe);
  privep->active = false;
  ra_usb_req_complete(privep, result);

  if (privep->head != NULL && !privep->active)
    {
      (void)ra_usb_start_out(privep);
    }
}

static int ra_usb_ep0_respond(FAR struct ra_usbdev_s *priv,
                              FAR const uint8_t *data, uint16_t length)
{
  uint16_t nbytes;
  int ret;

  if (length > sizeof(priv->ep0data))
    {
      return -E2BIG;
    }

  if (data != priv->ep0data && length > 0)
    {
      memcpy(priv->ep0data, data, length);
    }

  priv->ep0internal = true;
  priv->ep0internal_len = length;
  priv->ep0internal_xfrd = 0;
  priv->ep0status = false;
  priv->ep0status_req = false;

  putreg16(USBFS_FIFOCTR_BCLR, RA_USBFS_CFIFOCTR);
  nbytes = length > RA_USB_EP0_MAXPACKET ?
           RA_USB_EP0_MAXPACKET : length;
  ret = ra_usb_fifo_write(0, priv->ep0data, nbytes,
                          RA_USB_EP0_MAXPACKET);
  if (ret < 0)
    {
      priv->ep0internal = false;
      return ret;
    }

  priv->ep0internal_xfrd = nbytes;
  modifyreg16(RA_USBFS_BEMPENB, 0, USBFS_PIPE_BIT(0));
  ra_usb_pipe_set_pid(0, USBFS_PIPECTR_PID_BUF);
  return OK;
}

static void ra_usb_ep0_internal_advance(FAR struct ra_usbdev_s *priv)
{
  uint16_t remaining;
  uint16_t nbytes;
  int ret;

  if (!priv->ep0internal)
    {
      return;
    }

  remaining = priv->ep0internal_len - priv->ep0internal_xfrd;
  if (remaining > 0)
    {
      nbytes = remaining > RA_USB_EP0_MAXPACKET ?
               RA_USB_EP0_MAXPACKET : remaining;
      ret = ra_usb_fifo_write(0,
                              &priv->ep0data[priv->ep0internal_xfrd],
                              nbytes, RA_USB_EP0_MAXPACKET);
      if (ret < 0)
        {
          priv->ep0internal = false;
          ra_usb_pipe_set_pid(0, USBFS_PIPECTR_PID_STALL);
          return;
        }

      priv->ep0internal_xfrd += nbytes;
      return;
    }

  priv->ep0internal = false;
  priv->ep0status = true;
  modifyreg16(RA_USBFS_DCPCTR, 0,
              USBFS_DCPCTR_CCPL | USBFS_PIPECTR_PID_BUF);
}

static void ra_usb_ep0_status(FAR struct ra_usbdev_s *priv,
                              bool request)
{
  unsigned int timeout;

  putreg16(USBFS_FIFOSEL_MBW_16 | USBFS_CFIFOSEL_ISEL,
           RA_USBFS_CFIFOSEL);
  for (timeout = 0; timeout < RA_USB_FIFO_WAIT; timeout++)
    {
      if ((getreg16(RA_USBFS_CFIFOSEL) & USBFS_CFIFOSEL_ISEL) != 0)
        {
          break;
        }
    }

  priv->ep0status = true;
  priv->ep0status_req = request;
  modifyreg16(RA_USBFS_DCPCTR, 0,
              USBFS_DCPCTR_CCPL | USBFS_PIPECTR_PID_BUF);
}

static int ra_usb_dispatch(FAR struct ra_usbdev_s *priv,
                           FAR uint8_t *dataout, size_t outlen)
{
  int ret = -ENODEV;

  usbtrace(TRACE_INTDECODE(RA_TRACEINTID_DISPATCH), priv->ctrl.req);

  if (priv->driver != NULL)
    {
      ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl,
                        dataout, outlen);
    }

  if (ret < 0)
    {
      priv->eplist[0].stalled = true;
      ra_usb_pipe_set_pid(0, USBFS_PIPECTR_PID_STALL);
    }

  return ret;
}

static int ra_usb_standard_request(FAR struct ra_usbdev_s *priv,
                                   FAR uint8_t *dataout, size_t outlen)
{
  FAR struct ra_usb_ep_s *privep;
  uint16_t value = GETUINT16(priv->ctrl.value);
  uint16_t index = GETUINT16(priv->ctrl.index);
  uint16_t length = GETUINT16(priv->ctrl.len);
  uint8_t response[2] =
  {
    0, 0
  };

  uint8_t recipient;

  recipient = priv->ctrl.type & USB_REQ_RECIPIENT_MASK;

  switch (priv->ctrl.req)
    {
      case USB_REQ_GETSTATUS:
        if ((priv->ctrl.type & USB_REQ_DIR_IN) == 0 ||
            value != 0 || length != 2)
          {
            return -EINVAL;
          }

        if (recipient == USB_REQ_RECIPIENT_DEVICE)
          {
            if (index != 0)
              {
                return -EINVAL;
              }

            if (priv->selfpowered)
              {
                response[0] |= 1 << USB_FEATURE_SELFPOWERED;
              }

            if (priv->remotewakeup)
              {
                response[0] |= 1 << USB_FEATURE_REMOTEWAKEUP;
              }
          }
        else if (recipient == USB_REQ_RECIPIENT_INTERFACE)
          {
            if ((index & 0xff00) != 0)
              {
                return -EINVAL;
              }
          }
        else if (recipient == USB_REQ_RECIPIENT_ENDPOINT)
          {
            if ((index & 0xff00) != 0)
              {
                return -EINVAL;
              }

            privep = ra_usb_ep_find(priv, index & 0xff);
            if (privep == NULL)
              {
                return -ENOENT;
              }

            if (privep->stalled)
              {
                response[0] = 1;
              }
          }
        else
          {
            return -EINVAL;
          }

        return ra_usb_ep0_respond(priv, response, sizeof(response));

      case USB_REQ_CLEARFEATURE:
      case USB_REQ_SETFEATURE:
        if ((priv->ctrl.type & USB_REQ_DIR_IN) != 0 || length != 0)
          {
            return -EINVAL;
          }

        if (recipient == USB_REQ_RECIPIENT_ENDPOINT &&
            value == USB_FEATURE_ENDPOINTHALT &&
            (index & 0xff00) == 0)
          {
            privep = ra_usb_ep_find(priv, index & 0xff);
            if (privep == NULL)
              {
                return -ENOENT;
              }

            ra_usb_epstall(&privep->ep,
                           priv->ctrl.req == USB_REQ_CLEARFEATURE);
            ra_usb_ep0_status(priv, false);
            return OK;
          }

#ifdef CONFIG_USBDEV_REMOTEWAKEUP
        if (recipient == USB_REQ_RECIPIENT_DEVICE &&
            value == USB_FEATURE_REMOTEWAKEUP && index == 0)
          {
            priv->remotewakeup =
              priv->ctrl.req == USB_REQ_SETFEATURE;
            ra_usb_ep0_status(priv, false);
            return OK;
          }
#endif

        return ra_usb_dispatch(priv, dataout, outlen);

      case USB_REQ_SETADDRESS:
        if (priv->ctrl.type != USB_REQ_RECIPIENT_DEVICE ||
            value > 127 || index != 0 || length != 0)
          {
            return -EINVAL;
          }

        /* USBFS handles SET_ADDRESS, including its status stage, in
         * hardware.  USBADDR changes after the host acknowledges it.
         */

        return OK;

      case USB_REQ_GETDESCRIPTOR:
      case USB_REQ_SETDESCRIPTOR:
      case USB_REQ_GETCONFIGURATION:
      case USB_REQ_SETCONFIGURATION:
      case USB_REQ_GETINTERFACE:
      case USB_REQ_SETINTERFACE:
        return ra_usb_dispatch(priv, dataout, outlen);

      default:
        return -EOPNOTSUPP;
    }
}

static void ra_usb_process_setup(FAR struct ra_usbdev_s *priv,
                                 FAR uint8_t *dataout, size_t outlen)
{
  int ret;

  if ((priv->ctrl.type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      ret = ra_usb_standard_request(priv, dataout, outlen);
    }
  else
    {
      ret = ra_usb_dispatch(priv, dataout, outlen);
    }

  if (ret < 0)
    {
      usbtrace(TRACE_DEVERROR(RA_TRACEERR_SETUP),
               (uint16_t)-ret);
      priv->eplist[0].stalled = true;
      ra_usb_pipe_set_pid(0, USBFS_PIPECTR_PID_STALL);
    }
}

static void ra_usb_setup(FAR struct ra_usbdev_s *priv)
{
  uint16_t request;
  uint16_t value;
  uint16_t index;
  uint16_t length;

  usbtrace(TRACE_INTDECODE(RA_TRACEINTID_SETUP), 0);

  ra_usb_cancel_all(&priv->eplist[0], -EPROTO);
  priv->ep0out = false;
  priv->ep0internal = false;
  priv->ep0status = false;
  priv->ep0status_req = false;
  priv->eplist[0].stalled = false;

  request = getreg16(RA_USBFS_USBREQ);
  value = getreg16(RA_USBFS_USBVAL);
  index = getreg16(RA_USBFS_USBINDX);
  length = getreg16(RA_USBFS_USBLENG);

  priv->ctrl.type = request & 0xff;
  priv->ctrl.req = request >> 8;
  priv->ctrl.value[0] = value & 0xff;
  priv->ctrl.value[1] = value >> 8;
  priv->ctrl.index[0] = index & 0xff;
  priv->ctrl.index[1] = index >> 8;
  priv->ctrl.len[0] = length & 0xff;
  priv->ctrl.len[1] = length >> 8;

  putreg16((uint16_t)~USBFS_INTSTS0_VALID, RA_USBFS_INTSTS0);

  uinfo("SETUP type=%02x req=%02x value=%04x index=%04x len=%u\n",
        priv->ctrl.type, priv->ctrl.req, value, index, length);

  if (USB_REQ_ISOUT(priv->ctrl.type) && length > 0)
    {
      unsigned int timeout;

      if (length > sizeof(priv->ep0data))
        {
          ra_usb_pipe_set_pid(0, USBFS_PIPECTR_PID_STALL);
          return;
        }

      priv->ep0expected = length;
      priv->ep0received = 0;
      priv->ep0out = true;

      putreg16(USBFS_FIFOSEL_MBW_16, RA_USBFS_CFIFOSEL);
      for (timeout = 0; timeout < RA_USB_FIFO_WAIT; timeout++)
        {
          if ((getreg16(RA_USBFS_CFIFOSEL) &
               USBFS_CFIFOSEL_ISEL) == 0)
            {
              break;
            }
        }

      if (timeout == RA_USB_FIFO_WAIT)
        {
          priv->ep0out = false;
          ra_usb_pipe_set_pid(0, USBFS_PIPECTR_PID_STALL);
          return;
        }

      putreg16(USBFS_FIFOCTR_BCLR, RA_USBFS_CFIFOCTR);
      modifyreg16(RA_USBFS_BRDYENB, 0, USBFS_PIPE_BIT(0));
      ra_usb_pipe_set_pid(0, USBFS_PIPECTR_PID_BUF);
      return;
    }

  ra_usb_process_setup(priv, NULL, 0);
}

static void ra_usb_ep0_out(FAR struct ra_usbdev_s *priv)
{
  uint16_t packetlen;
  uint16_t remaining;
  int copied;

  if (!priv->ep0out)
    {
      ra_usb_pipe_set_pid(0, USBFS_PIPECTR_PID_NAK);
      return;
    }

  remaining = priv->ep0expected - priv->ep0received;
  copied = ra_usb_fifo_read(0, &priv->ep0data[priv->ep0received],
                            remaining, &packetlen);
  if (copied < 0 || packetlen > remaining)
    {
      priv->ep0out = false;
      ra_usb_pipe_set_pid(0, USBFS_PIPECTR_PID_STALL);
      return;
    }

  priv->ep0received += copied;
  if (packetlen == RA_USB_EP0_MAXPACKET &&
      priv->ep0received < priv->ep0expected)
    {
      ra_usb_pipe_set_pid(0, USBFS_PIPECTR_PID_BUF);
      return;
    }

  priv->ep0out = false;
  ra_usb_pipe_set_pid(0, USBFS_PIPECTR_PID_NAK);
  ra_usb_process_setup(priv, priv->ep0data, priv->ep0received);
}

static void ra_usb_ep0_status_complete(FAR struct ra_usbdev_s *priv)
{
  FAR struct ra_usb_ep_s *ep0 = &priv->eplist[0];

  if (!priv->ep0status)
    {
      return;
    }

  priv->ep0status = false;
  if (priv->ep0status_req && ep0->head != NULL)
    {
      priv->ep0status_req = false;
      ep0->active = false;
      ra_usb_req_complete(ep0, OK);
    }
  else
    {
      priv->ep0status_req = false;
    }
}

static void ra_usb_pipe_reset(FAR struct ra_usbdev_s *priv)
{
  unsigned int pipe;

  putreg16(USBFS_PIPE_BIT(0), RA_USBFS_BEMPENB);
  putreg16(USBFS_PIPE_BIT(0), RA_USBFS_BRDYENB);
  putreg16(0, RA_USBFS_NRDYENB);
  putreg16(0, RA_USBFS_D0FIFOSEL);
  putreg16(0, RA_USBFS_D1FIFOSEL);
  putreg16(USBFS_FIFOCTR_BCLR, RA_USBFS_CFIFOCTR);
  putreg16(0, RA_USBFS_BEMPSTS);
  putreg16(0, RA_USBFS_BRDYSTS);
  putreg16(0, RA_USBFS_NRDYSTS);

  for (pipe = 1; pipe < RA_USB_NPIPES; pipe++)
    {
      FAR struct ra_usb_ep_s *privep = &priv->eplist[pipe];

      putreg16(pipe, RA_USBFS_PIPESEL);
      putreg16(0, RA_USBFS_PIPECFG);
      putreg16(USBFS_PIPECTR_ACLRM | USBFS_PIPECTR_SQCLR,
               RA_USBFS_PIPECTR(pipe));
      putreg16(0, RA_USBFS_PIPECTR(pipe));
      ra_usb_pipe_counter_stop(pipe);

      privep->configured = false;
      privep->stalled = false;
      privep->active = false;
      privep->zlp = false;
      ra_usb_cancel_all(privep, -ESHUTDOWN);
    }

  putreg16(0, RA_USBFS_PIPESEL);
  putreg16(0, RA_USBFS_DCPCFG);
  putreg16(RA_USB_EP0_MAXPACKET, RA_USBFS_DCPMAXP);
  putreg16(USBFS_PIPECTR_SQCLR, RA_USBFS_DCPCTR);
  ra_usb_cancel_all(&priv->eplist[0], -ESHUTDOWN);

  priv->ep0out = false;
  priv->ep0internal = false;
  priv->ep0status = false;
  priv->ep0status_req = false;
  priv->remotewakeup = false;
  priv->eplist[0].stalled = false;
}

static void ra_usb_bus_reset(FAR struct ra_usbdev_s *priv)
{
  usbtrace(TRACE_INTDECODE(RA_TRACEINTID_RESET), 0);

  priv->usbdev.speed = USB_SPEED_FULL;
  priv->suspended = false;

  if (priv->driver != NULL)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }

  ra_usb_pipe_reset(priv);
}

static void ra_usb_bemp(FAR struct ra_usbdev_s *priv, uint16_t status)
{
  if ((status & USBFS_PIPE_BIT(0)) == 0)
    {
      return;
    }

  if (priv->ep0status)
    {
      return;
    }

  if (priv->ep0internal)
    {
      ra_usb_ep0_internal_advance(priv);
    }
  else
    {
      ra_usb_advance_in(&priv->eplist[0]);
    }
}

static void ra_usb_brdy(FAR struct ra_usbdev_s *priv, uint16_t status)
{
  unsigned int pipe;

  if ((status & USBFS_PIPE_BIT(0)) != 0)
    {
      ra_usb_ep0_out(priv);
      status &= ~USBFS_PIPE_BIT(0);
    }

  for (pipe = 1; pipe < RA_USB_NPIPES && status != 0; pipe++)
    {
      FAR struct ra_usb_ep_s *privep;

      if ((status & USBFS_PIPE_BIT(pipe)) == 0)
        {
          continue;
        }

      privep = &priv->eplist[pipe];
      if (privep->allocated && privep->in)
        {
          ra_usb_advance_in(privep);
        }
      else if (privep->allocated)
        {
          ra_usb_advance_out(privep);
        }
      else
        {
          ra_usb_pipe_set_pid(pipe, USBFS_PIPECTR_PID_NAK);
        }

      status &= ~USBFS_PIPE_BIT(pipe);
    }
}

static int ra_usb_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ra_usbdev_s *priv = arg;
  uint16_t intsts;
  uint16_t active;
  uint16_t status;
  uint16_t clear;
  uint16_t state;

  intsts = getreg16(RA_USBFS_INTSTS0);
  active = intsts & getreg16(RA_USBFS_INTENB0);
  clear = active & (USBFS_INT_CTRT | USBFS_INT_DVST |
                    USBFS_INT_SOFR | USBFS_INT_RESM |
                    USBFS_INT_VBINT);
  putreg16((uint16_t)~clear | USBFS_INTSTS0_VALID,
           RA_USBFS_INTSTS0);

  if ((active & USBFS_INT_VBINT) != 0)
    {
      if ((getreg16(RA_USBFS_INTSTS0) & USBFS_INTSTS0_VBSTS) == 0)
        {
          modifyreg16(RA_USBFS_SYSCFG, USBFS_SYSCFG_DPRPU, 0);
          if (priv->driver != NULL)
            {
              CLASS_DISCONNECT(priv->driver, &priv->usbdev);
            }

          ra_usb_pipe_reset(priv);
          priv->usbdev.speed = USB_SPEED_UNKNOWN;
        }
      else if (priv->pullup)
        {
          modifyreg16(RA_USBFS_SYSCFG, 0, USBFS_SYSCFG_DPRPU);
        }
    }

  if ((active & USBFS_INT_RESM) != 0 && priv->suspended)
    {
      usbtrace(TRACE_INTDECODE(RA_TRACEINTID_RESUME), 0);
      priv->suspended = false;
      if (priv->driver != NULL)
        {
          CLASS_RESUME(priv->driver, &priv->usbdev);
        }
    }

  if ((active & USBFS_INT_DVST) != 0)
    {
      state = intsts & USBFS_INTSTS0_DVSQ_MASK;
      if (state == USBFS_INTSTS0_DVSQ_DEFAULT)
        {
          ra_usb_bus_reset(priv);
        }
      else if (state >= USBFS_INTSTS0_DVSQ_SUSPENDED &&
               !priv->suspended)
        {
          usbtrace(TRACE_INTDECODE(RA_TRACEINTID_SUSPEND), state);
          priv->suspended = true;
          if (priv->driver != NULL)
            {
              CLASS_SUSPEND(priv->driver, &priv->usbdev);
            }
        }
    }

  if ((active & USBFS_INT_CTRT) != 0)
    {
      state = intsts & USBFS_INTSTS0_CTSQ_MASK;
      if ((intsts & USBFS_INTSTS0_VALID) != 0)
        {
          ra_usb_setup(priv);
        }
      else if (state == USBFS_INTSTS0_CTSQ_IDLE)
        {
          ra_usb_ep0_status_complete(priv);
        }
      else if (state == USBFS_INTSTS0_CTSQ_SQER)
        {
          ra_usb_pipe_set_pid(0, USBFS_PIPECTR_PID_STALL);
        }
    }

#ifdef CONFIG_USBDEV_SOFINTERRUPT
  if ((active & USBFS_INT_SOFR) != 0)
    {
      up_usbdev_sof_irq(&priv->usbdev,
                        getreg16(RA_USBFS_FRMNUM) & USBFS_FRMNUM_MASK);
    }
#endif

  if ((active & USBFS_INT_BEMP) != 0)
    {
      status = getreg16(RA_USBFS_BEMPSTS) &
               getreg16(RA_USBFS_BEMPENB);
      putreg16((uint16_t)~status, RA_USBFS_BEMPSTS);
      ra_usb_bemp(priv, status);
    }

  if ((active & USBFS_INT_BRDY) != 0)
    {
      status = getreg16(RA_USBFS_BRDYSTS) &
               getreg16(RA_USBFS_BRDYENB);
      putreg16((uint16_t)~status, RA_USBFS_BRDYSTS);
      ra_usb_brdy(priv, status);
    }

  if ((active & USBFS_INT_NRDY) != 0)
    {
      status = getreg16(RA_USBFS_NRDYSTS) &
               getreg16(RA_USBFS_NRDYENB);
      putreg16((uint16_t)~status, RA_USBFS_NRDYSTS);
    }

  ra_clear_ir(irq);
  return OK;
}

static uint8_t ra_usb_interval(uint8_t interval)
{
  uint8_t value = 0;

  while (interval > 1 && value < 7)
    {
      interval >>= 1;
      value++;
    }

  return value;
}

static int ra_usb_epconfigure(FAR struct usbdev_ep_s *ep,
                              FAR const struct usb_epdesc_s *desc,
                              bool last)
{
  FAR struct ra_usb_ep_s *privep = (FAR struct ra_usb_ep_s *)ep;
  uint16_t maxpacket;
  uint16_t pipecfg;
  irqstate_t flags;

  (void)last;

  if (ep == NULL || desc == NULL || privep->pipe == 0 ||
      desc->addr != ep->eplog)
    {
      usbtrace(TRACE_DEVERROR(RA_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  maxpacket = GETUINT16(desc->mxpacketsize);
  if (maxpacket == 0 || maxpacket > 64)
    {
      return -ERANGE;
    }

  if ((desc->attr & USB_EP_ATTR_XFERTYPE_MASK) != privep->eptype)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  ra_usb_pipe_flush(privep->pipe);

  putreg16(privep->pipe, RA_USBFS_PIPESEL);
  putreg16(0, RA_USBFS_PIPECFG);
  putreg16(maxpacket & USBFS_PIPEMAXP_MASK, RA_USBFS_PIPEMAXP);
  putreg16(ra_usb_interval(desc->interval) &
           USBFS_PIPEPERI_IITV_MASK, RA_USBFS_PIPEPERI);

  pipecfg = USB_EPNO(desc->addr);
  if (USB_ISEPIN(desc->addr))
    {
      pipecfg |= USBFS_PIPECFG_DIR_IN;
    }

  if (privep->eptype == USB_EP_ATTR_XFER_BULK)
    {
      pipecfg |= USBFS_PIPECFG_TYPE_BULK |
                 USBFS_PIPECFG_SHTNAK |
                 USBFS_PIPECFG_DBLB;
    }
  else if (privep->eptype == USB_EP_ATTR_XFER_INT)
    {
      pipecfg |= USBFS_PIPECFG_TYPE_INT;
    }
  else
    {
      leave_critical_section(flags);
      return -ENOTSUP;
    }

  putreg16(pipecfg, RA_USBFS_PIPECFG);
  putreg16((uint16_t)~USBFS_PIPE_BIT(privep->pipe),
           RA_USBFS_BRDYSTS);
  modifyreg16(RA_USBFS_BRDYENB, 0,
              USBFS_PIPE_BIT(privep->pipe));

  privep->ep.maxpacket = maxpacket;
  privep->configured = true;
  privep->stalled = false;
  if (privep->in)
    {
      ra_usb_pipe_set_pid(privep->pipe, USBFS_PIPECTR_PID_BUF);
    }

  putreg16(0, RA_USBFS_PIPESEL);
  leave_critical_section(flags);
  return OK;
}

static int ra_usb_epdisable(FAR struct usbdev_ep_s *ep)
{
  FAR struct ra_usb_ep_s *privep = (FAR struct ra_usb_ep_s *)ep;
  irqstate_t flags;

  if (ep == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  if (privep->pipe != 0)
    {
      modifyreg16(RA_USBFS_BRDYENB,
                  USBFS_PIPE_BIT(privep->pipe), 0);
      ra_usb_pipe_flush(privep->pipe);
      putreg16(privep->pipe, RA_USBFS_PIPESEL);
      putreg16(0, RA_USBFS_PIPECFG);
      putreg16(0, RA_USBFS_PIPESEL);
    }

  privep->configured = false;
  privep->stalled = false;
  ra_usb_cancel_all(privep, -ESHUTDOWN);
  leave_critical_section(flags);
  return OK;
}

static FAR struct usbdev_req_s *
ra_usb_epallocreq(FAR struct usbdev_ep_s *ep)
{
  FAR struct ra_usb_req_s *privreq;

  if (ep == NULL)
    {
      return NULL;
    }

  privreq = kmm_zalloc(sizeof(*privreq));
  if (privreq == NULL)
    {
      usbtrace(TRACE_DEVERROR(RA_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  return &privreq->req;
}

static void ra_usb_epfreereq(FAR struct usbdev_ep_s *ep,
                             FAR struct usbdev_req_s *req)
{
  if (ep != NULL && req != NULL)
    {
      kmm_free((FAR struct ra_usb_req_s *)req);
    }
}

static int ra_usb_epsubmit(FAR struct usbdev_ep_s *ep,
                           FAR struct usbdev_req_s *req)
{
  FAR struct ra_usb_ep_s *privep = (FAR struct ra_usb_ep_s *)ep;
  FAR struct ra_usb_req_s *privreq = (FAR struct ra_usb_req_s *)req;
  bool empty;
  irqstate_t flags;
  int ret = OK;

  if (ep == NULL || req == NULL || req->callback == NULL ||
      (req->len > 0 && req->buf == NULL))
    {
      usbtrace(TRACE_DEVERROR(RA_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (privep->dev->driver == NULL)
    {
      return -ESHUTDOWN;
    }

  req->result = -EINPROGRESS;
  req->xfrd = 0;

  flags = enter_critical_section();
  empty = privep->head == NULL;
  ra_usb_req_enqueue(privep, privreq);

  if (privep->pipe == 0)
    {
      if (USB_REQ_ISOUT(privep->dev->ctrl.type))
        {
          if (!empty || req->len != 0)
            {
              ret = -EINVAL;
            }
          else
            {
              privep->active = true;
              ra_usb_ep0_status(privep->dev, true);
            }
        }
      else if (empty)
        {
          ret = ra_usb_start_in(privep);
        }
    }
  else if (!privep->configured)
    {
      ret = -ESHUTDOWN;
    }
  else if (empty && !privep->stalled)
    {
      ret = privep->in ? ra_usb_start_in(privep) :
                         ra_usb_start_out(privep);
    }

  if (ret < 0)
    {
      FAR struct ra_usb_req_s *removed = ra_usb_req_dequeue(privep);

      DEBUGASSERT(removed == privreq);
      privep->active = false;
      req->result = ret;
    }

  leave_critical_section(flags);
  return ret;
}

static int ra_usb_epcancel(FAR struct usbdev_ep_s *ep,
                           FAR struct usbdev_req_s *req)
{
  FAR struct ra_usb_ep_s *privep = (FAR struct ra_usb_ep_s *)ep;
  FAR struct ra_usb_req_s *target = (FAR struct ra_usb_req_s *)req;
  FAR struct ra_usb_req_s *prev = NULL;
  FAR struct ra_usb_req_s *curr;
  irqstate_t flags;

  if (ep == NULL || req == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  for (curr = privep->head; curr != NULL; curr = curr->flink)
    {
      if (curr == target)
        {
          break;
        }

      prev = curr;
    }

  if (curr == NULL)
    {
      leave_critical_section(flags);
      return -ENOENT;
    }

  if (prev == NULL)
    {
      if (privep->pipe != 0)
        {
          ra_usb_pipe_flush(privep->pipe);
        }

      privep->active = false;
      (void)ra_usb_req_dequeue(privep);
    }
  else
    {
      prev->flink = curr->flink;
      if (privep->tail == curr)
        {
          privep->tail = prev;
        }
    }

  curr->flink = NULL;
  curr->req.result = -ECANCELED;
  curr->req.callback(ep, &curr->req);

  if (prev == NULL && privep->head != NULL && !privep->stalled)
    {
      if (privep->pipe == 0 || privep->in)
        {
          (void)ra_usb_start_in(privep);
        }
      else
        {
          (void)ra_usb_start_out(privep);
        }
    }

  leave_critical_section(flags);
  return OK;
}

static int ra_usb_epstall(FAR struct usbdev_ep_s *ep, bool resume)
{
  FAR struct ra_usb_ep_s *privep = (FAR struct ra_usb_ep_s *)ep;
  irqstate_t flags;

  if (ep == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  if (resume)
    {
      ra_usb_pipe_flush(privep->pipe);
      privep->stalled = false;

      if (privep->head != NULL)
        {
          if (privep->pipe == 0 || privep->in)
            {
              (void)ra_usb_start_in(privep);
            }
          else
            {
              (void)ra_usb_start_out(privep);
            }
        }
      else if (privep->pipe != 0 && privep->in)
        {
          ra_usb_pipe_set_pid(privep->pipe, USBFS_PIPECTR_PID_BUF);
        }
    }
  else
    {
      if (privep->pipe != 0)
        {
          ra_usb_pipe_set_pid(privep->pipe, USBFS_PIPECTR_PID_NAK);
          (void)ra_usb_pipe_idle(privep->pipe);
        }

      ra_usb_pipe_set_pid(privep->pipe, USBFS_PIPECTR_PID_STALL);
      privep->stalled = true;
    }

  leave_critical_section(flags);
  return OK;
}

static FAR struct usbdev_ep_s *
ra_usb_allocep(FAR struct usbdev_s *dev, uint8_t eplog, bool in,
               uint8_t eptype)
{
  FAR struct ra_usbdev_s *priv = (FAR struct ra_usbdev_s *)dev;
  FAR struct ra_usb_ep_s *privep;
  uint8_t first;
  uint8_t last;
  uint8_t epno;
  uint8_t pipe;
  irqstate_t flags;

  epno = USB_EPNO(eplog);
  eplog = in ? USB_EPIN(epno) : epno;

  if (eptype == USB_EP_ATTR_XFER_BULK)
    {
      first = RA_USB_PIPE_MIN_BULK;
      last = RA_USB_PIPE_MAX_BULK;
    }
  else if (eptype == USB_EP_ATTR_XFER_INT)
    {
      first = RA_USB_PIPE_MIN_INT;
      last = RA_USB_PIPE_MAX_INT;
    }
  else
    {
      return NULL;
    }

  flags = enter_critical_section();
  if (epno != 0 && ra_usb_ep_find(priv, eplog) != NULL)
    {
      leave_critical_section(flags);
      return NULL;
    }

  for (pipe = first; pipe <= last; pipe++)
    {
      privep = &priv->eplist[pipe];
      if (!privep->allocated)
        {
          if (epno == 0)
            {
              epno = pipe;
              eplog = in ? USB_EPIN(epno) : epno;
            }

          privep->allocated = true;
          privep->configured = false;
          privep->stalled = false;
          privep->active = false;
          privep->in = in;
          privep->eptype = eptype;
          privep->ep.eplog = eplog;
          leave_critical_section(flags);
          return &privep->ep;
        }
    }

  leave_critical_section(flags);
  usbtrace(TRACE_DEVERROR(RA_TRACEERR_NOEP), eplog);
  return NULL;
}

static void ra_usb_freeep(FAR struct usbdev_s *dev,
                          FAR struct usbdev_ep_s *ep)
{
  FAR struct ra_usb_ep_s *privep = (FAR struct ra_usb_ep_s *)ep;
  irqstate_t flags;

  (void)dev;

  if (ep == NULL || privep->pipe == 0)
    {
      return;
    }

  (void)ra_usb_epdisable(ep);
  flags = enter_critical_section();
  privep->allocated = false;
  privep->ep.eplog = 0;
  privep->eptype = 0;
  privep->in = false;
  leave_critical_section(flags);
}

static int ra_usb_getframe(FAR struct usbdev_s *dev)
{
  (void)dev;
  return getreg16(RA_USBFS_FRMNUM) & USBFS_FRMNUM_MASK;
}

static int ra_usb_wakeup(FAR struct usbdev_s *dev)
{
  FAR struct ra_usbdev_s *priv = (FAR struct ra_usbdev_s *)dev;

  if (!priv->remotewakeup)
    {
      return -EACCES;
    }

  modifyreg16(RA_USBFS_DVSTCTR0, 0, USBFS_DVSTCTR0_WKUP);
  return OK;
}

static int ra_usb_selfpowered(FAR struct usbdev_s *dev, bool selfpowered)
{
  FAR struct ra_usbdev_s *priv = (FAR struct ra_usbdev_s *)dev;

  priv->selfpowered = selfpowered;
  return OK;
}

static int ra_usb_pullup(FAR struct usbdev_s *dev, bool enable)
{
  FAR struct ra_usbdev_s *priv = (FAR struct ra_usbdev_s *)dev;

  priv->pullup = enable;
  if (enable)
    {
      modifyreg16(RA_USBFS_SYSCFG, 0, USBFS_SYSCFG_DPRPU);
    }
  else
    {
      modifyreg16(RA_USBFS_SYSCFG, USBFS_SYSCFG_DPRPU, 0);
    }

  return OK;
}

static int ra_usb_hw_initialize(FAR struct ra_usbdev_s *priv)
{
  unsigned int timeout;

  ra_module_control(RA8P1_MSTPCRB, MSTPCRB_USBFS0, true);

  modifyreg16(RA_USBFS_SYSCFG, 0, USBFS_SYSCFG_SCKE);
  for (timeout = 0; timeout < RA_USB_FIFO_WAIT; timeout++)
    {
      if ((getreg16(RA_USBFS_SYSCFG) & USBFS_SYSCFG_SCKE) != 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (timeout >= RA_USB_FIFO_WAIT)
    {
      ra_module_control(RA8P1_MSTPCRB, MSTPCRB_USBFS0, false);
      return -ETIMEDOUT;
    }

  modifyreg16(RA_USBFS_SYSCFG,
              USBFS_SYSCFG_DCFM | USBFS_SYSCFG_DRPD |
              USBFS_SYSCFG_DPRPU,
              USBFS_SYSCFG_USBE);
  putreg16(USBFS_FIFOSEL_MBW_16, RA_USBFS_CFIFOSEL);
  putreg16(USBFS_FIFOSEL_MBW_16, RA_USBFS_D0FIFOSEL);
  putreg16(USBFS_FIFOSEL_MBW_16, RA_USBFS_D1FIFOSEL);
  putreg16(RA_USB_EP0_MAXPACKET, RA_USBFS_DCPMAXP);
  putreg16(0, RA_USBFS_INTSTS0);
  putreg16(0, RA_USBFS_INTSTS1);
  putreg16(RA_USB_INTENB0_DEFAULT, RA_USBFS_INTENB0);
  putreg16(0, RA_USBFS_INTENB1);
  ra_usb_pipe_reset(priv);
  return OK;
}

static void ra_usb_hw_uninitialize(void)
{
  modifyreg16(RA_USBFS_SYSCFG,
              USBFS_SYSCFG_DPRPU | USBFS_SYSCFG_USBE |
              USBFS_SYSCFG_SCKE, 0);
  ra_module_control(RA8P1_MSTPCRB, MSTPCRB_USBFS0, false);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm_usbinitialize(void)
{
  FAR struct ra_usbdev_s *priv = &g_ra_usbdev;
  unsigned int pipe;
  int ret;

  usbtrace(TRACE_DEVINIT, 0);
  if (priv->initialized)
    {
      return;
    }

  memset(priv, 0, sizeof(*priv));
  priv->usbdev.ops = &g_ra_usb_devops;
  priv->usbdev.ep0 = &priv->eplist[0].ep;
  priv->usbdev.speed = USB_SPEED_UNKNOWN;
  priv->usbdev.dualspeed = 0;
#ifdef CONFIG_USBDEV_SELFPOWERED
  priv->selfpowered = true;
#endif

  for (pipe = 0; pipe < RA_USB_NPIPES; pipe++)
    {
      FAR struct ra_usb_ep_s *privep = &priv->eplist[pipe];

      privep->ep.ops = &g_ra_usb_epops;
      privep->ep.maxpacket = RA_USB_EP0_MAXPACKET;
      privep->dev = priv;
      privep->pipe = pipe;
    }

  priv->eplist[0].allocated = true;
  priv->eplist[0].configured = true;
  priv->eplist[0].ep.eplog = 0;

  ret = ra_usb_hw_initialize(priv);
  if (ret < 0)
    {
      usbtrace(TRACE_DEVERROR(RA_TRACEERR_FIFO), (uint16_t)-ret);
      return;
    }

  up_disable_irq(RA_IRQ_USBFS_INT);
  ret = irq_attach(RA_IRQ_USBFS_INT, ra_usb_interrupt, priv);
  if (ret < 0)
    {
      ra_usb_hw_uninitialize();
      usbtrace(TRACE_DEVERROR(RA_TRACEERR_IRQ), (uint16_t)-ret);
      return;
    }

  priv->initialized = true;
}

void arm_usbuninitialize(void)
{
  FAR struct ra_usbdev_s *priv = &g_ra_usbdev;

  usbtrace(TRACE_DEVUNINIT, 0);
  if (!priv->initialized)
    {
      return;
    }

  if (priv->driver != NULL)
    {
      (void)usbdev_unregister(priv->driver);
    }

  up_disable_irq(RA_IRQ_USBFS_INT);
  irq_detach(RA_IRQ_USBFS_INT);
  ra_usb_hw_uninitialize();
  priv->initialized = false;
}

int usbdev_register(FAR struct usbdevclass_driver_s *driver)
{
  FAR struct ra_usbdev_s *priv = &g_ra_usbdev;
  int ret;

  if (!priv->initialized)
    {
      return -ENODEV;
    }

  if (driver == NULL || driver->ops == NULL ||
      driver->ops->bind == NULL || driver->ops->unbind == NULL ||
      driver->ops->setup == NULL)
    {
      usbtrace(TRACE_DEVERROR(RA_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (priv->driver != NULL)
    {
      usbtrace(TRACE_DEVERROR(RA_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }

  priv->driver = driver;
  priv->usbdev.speed = USB_SPEED_FULL;
  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret < 0)
    {
      priv->driver = NULL;
      usbtrace(TRACE_DEVERROR(RA_TRACEERR_BINDFAILED),
               (uint16_t)-ret);
      return ret;
    }

  ra_clear_ir(RA_IRQ_USBFS_INT);
  up_enable_irq(RA_IRQ_USBFS_INT);
  return OK;
}

int usbdev_unregister(FAR struct usbdevclass_driver_s *driver)
{
  FAR struct ra_usbdev_s *priv = &g_ra_usbdev;

  if (driver == NULL || driver != priv->driver)
    {
      return -EINVAL;
    }

  up_disable_irq(RA_IRQ_USBFS_INT);
  (void)ra_usb_pullup(&priv->usbdev, false);
  CLASS_DISCONNECT(driver, &priv->usbdev);
  ra_usb_pipe_reset(priv);
  CLASS_UNBIND(driver, &priv->usbdev);
  priv->driver = NULL;
  priv->usbdev.speed = USB_SPEED_UNKNOWN;
  return OK;
}
