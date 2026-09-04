/****************************************************************************
 * drivers/usbhost/usbhost_audio.c
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

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#include <debug.h>
#include <nuttx/audio/audio.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/usb/audio.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/wqueue.h>

#include "usbhost_registry.h"

#ifdef CONFIG_USBHOST_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define USBHOST_AUDIO_NFORMATS 16
#define USBHOST_AUDIO_NRATES   8
#define USBHOST_AUDIO_NSTREAMS 2
#define USBHOST_AUDIO_PLAYBACK 0
#define USBHOST_AUDIO_CAPTURE  1
#define USBHOST_AUDIO_PROTOCOL_2 0x20
#define USBHOST_AUDIO_FU_MUTE    1
#define USBHOST_AUDIO_FU_VOLUME  2
#define USBHOST_AUDIO_GET_MIN    0x82
#define USBHOST_AUDIO_GET_MAX    0x83
#define USBHOST_AUDIO_UAC2_MUTE_CONTROL_MASK   0x03
#define USBHOST_AUDIO_UAC2_VOLUME_CONTROL_MASK 0x0c

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct usbhost_audio_format_s
{
  struct usbhost_epdesc_s epdesc;
  struct usbhost_epdesc_s feedbackdesc;
  uint32_t rate[USBHOST_AUDIO_NRATES];
  uint8_t nrates;
  uint8_t ifno;
  uint8_t alt;
  uint8_t channels;
  uint8_t subslot;
  uint8_t resolution;
  uint8_t syncaddr;
  uint8_t clockid;
  uint8_t terminal;
  uint8_t feature;
  bool continuous;
  bool feedback;
};

struct usbhost_audio_s;

struct usbhost_audio_stream_s
{
  struct audio_lowerhalf_s dev;
  FAR struct usbhost_audio_s *audio;
  struct usbhost_audio_format_s format[USBHOST_AUDIO_NFORMATS];
  dq_queue_t pendq;
  mutex_t lock;
  sem_t sem;
  pthread_t thread;
  pthread_t feedbackthread;
  usbhost_ep_t ep;
  usbhost_ep_t feedbackep;
  FAR struct ap_buffer_s *active;
  uint32_t rate;
  uint32_t accumulator;
  volatile uint32_t feedbackrate;
  uint8_t feedbackbuf[4];
  uint8_t nformats;
  uint8_t current;
  uint8_t direction;
  uint8_t devno;
  bool registered;
  bool reserved;
  bool running;
  bool feedbackrunning;
  bool terminate;
  bool feedbackstop;
};

struct usbhost_audio_s
{
  struct usbhost_class_s usbclass;
  struct usbhost_audio_stream_s stream[USBHOST_AUDIO_NSTREAMS];
  struct work_s destroywork;
  FAR struct usb_ctrlreq_s *ctrlreq;
  FAR uint8_t *ctrlbuf;
  size_t ctrlbuflen;
  mutex_t ctrllock;
  uint8_t protocol;
  uint8_t controlif;
  uint8_t clockmap[UINT8_MAX + 1];
  uint8_t source[UINT8_MAX + 1];
  bool feature[UINT8_MAX + 1];
  uint8_t featurecontrols[UINT8_MAX + 1];
  volatile bool disconnected;
  bool controlpresent;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR struct usbhost_class_s *
  usbhost_audio_create(FAR struct usbhost_hubport_s *hport,
                       FAR const struct usbhost_id_s *id);
static int usbhost_audio_connect(FAR struct usbhost_class_s *usbclass,
                                 FAR const uint8_t *configdesc,
                                 int desclen);
static int usbhost_audio_disconnected(FAR struct usbhost_class_s *usbclass);

static int usbhost_audio_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                                 FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int usbhost_audio_configure(FAR struct audio_lowerhalf_s *dev,
                                   FAR void *session,
                                   FAR const struct audio_caps_s *caps);
static int usbhost_audio_start(FAR struct audio_lowerhalf_s *dev,
                               FAR void *session);
#  ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int usbhost_audio_stop(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session);
#  endif
static int usbhost_audio_reserve(FAR struct audio_lowerhalf_s *dev,
                                 FAR void **session);
static int usbhost_audio_release(FAR struct audio_lowerhalf_s *dev,
                                 FAR void *session);
#else
static int usbhost_audio_configure(FAR struct audio_lowerhalf_s *dev,
                                   FAR const struct audio_caps_s *caps);
static int usbhost_audio_start(FAR struct audio_lowerhalf_s *dev);
#  ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int usbhost_audio_stop(FAR struct audio_lowerhalf_s *dev);
#  endif
static int usbhost_audio_reserve(FAR struct audio_lowerhalf_s *dev);
static int usbhost_audio_release(FAR struct audio_lowerhalf_s *dev);
#endif
static int usbhost_audio_shutdown(FAR struct audio_lowerhalf_s *dev);
static int usbhost_audio_enqueue(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb);
static int usbhost_audio_cancel(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb);
static int usbhost_audio_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                               unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct usbhost_id_s g_audio_id[] =
{
  {USB_CLASS_AUDIO, ADC_SUBCLASS_AUDIOCONTROL, ADC_PROTOCOL_UNDEF, 0, 0},
  {USB_CLASS_AUDIO, ADC_SUBCLASS_AUDIOCONTROL,
    USBHOST_AUDIO_PROTOCOL_2, 0, 0},
  {USB_CLASS_AUDIO, ADC_SUBCLASS_AUDIOSTREAMING, ADC_PROTOCOL_UNDEF, 0, 0},
  {USB_CLASS_AUDIO, ADC_SUBCLASS_AUDIOSTREAMING,
    USBHOST_AUDIO_PROTOCOL_2, 0, 0},
  {USB_CLASS_AUDIO, 0, ADC_PROTOCOL_UNDEF, 0, 0},
  {USB_CLASS_AUDIO, 0, USBHOST_AUDIO_PROTOCOL_2, 0, 0}
};

static struct usbhost_registry_s g_audio_registry =
{
  NULL,
  usbhost_audio_create,
  sizeof(g_audio_id) / sizeof(g_audio_id[0]),
  g_audio_id
};

static const struct audio_ops_s g_audio_ops =
{
  usbhost_audio_getcaps,
  usbhost_audio_configure,
  usbhost_audio_shutdown,
  usbhost_audio_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  usbhost_audio_stop,
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  NULL,
  NULL,
#endif
  NULL,
  NULL,
  usbhost_audio_enqueue,
  usbhost_audio_cancel,
  usbhost_audio_ioctl,
  NULL,
  NULL,
  usbhost_audio_reserve,
  usbhost_audio_release
};

static uint32_t g_audio_devinuse;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t usbhost_audio_getle16(FAR const uint8_t *value)
{
  return (uint16_t)value[0] | ((uint16_t)value[1] << 8);
}

static uint32_t usbhost_audio_getle24(FAR const uint8_t *value)
{
  return (uint32_t)value[0] | ((uint32_t)value[1] << 8) |
         ((uint32_t)value[2] << 16);
}

static uint32_t usbhost_audio_getle32(FAR const uint8_t *value)
{
  return (uint32_t)value[0] | ((uint32_t)value[1] << 8) |
         ((uint32_t)value[2] << 16) | ((uint32_t)value[3] << 24);
}

static void usbhost_audio_putle16(FAR uint8_t *value, uint16_t data)
{
  value[0] = data & 0xff;
  value[1] = data >> 8;
}

static void usbhost_audio_putle32(FAR uint8_t *value, uint32_t data)
{
  value[0] = data & 0xff;
  value[1] = (data >> 8) & 0xff;
  value[2] = (data >> 16) & 0xff;
  value[3] = data >> 24;
}

static int usbhost_audio_setinterface(FAR struct usbhost_audio_s *audio,
                                      uint8_t ifno, uint8_t alt)
{
  FAR struct usbhost_hubport_s *hport = audio->usbclass.hport;
  int ret;

  nxmutex_lock(&audio->ctrllock);
  memset(audio->ctrlreq, 0, sizeof(*audio->ctrlreq));
  audio->ctrlreq->type = USB_REQ_DIR_OUT | USB_REQ_RECIPIENT_INTERFACE;
  audio->ctrlreq->req  = USB_REQ_SETINTERFACE;
  usbhost_audio_putle16(audio->ctrlreq->value, alt);
  usbhost_audio_putle16(audio->ctrlreq->index, ifno);
  ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, audio->ctrlreq, NULL);
  nxmutex_unlock(&audio->ctrllock);
  return ret;
}

static uint16_t usbhost_audio_ratemask(uint32_t rate)
{
  switch (rate)
    {
      case 8000:   return AUDIO_SAMP_RATE_8K;
      case 11025:  return AUDIO_SAMP_RATE_11K;
      case 12000:  return AUDIO_SAMP_RATE_12K;
      case 16000:  return AUDIO_SAMP_RATE_16K;
      case 22050:  return AUDIO_SAMP_RATE_22K;
      case 24000:  return AUDIO_SAMP_RATE_24K;
      case 32000:  return AUDIO_SAMP_RATE_32K;
      case 44100:  return AUDIO_SAMP_RATE_44K;
      case 48000:  return AUDIO_SAMP_RATE_48K;
      case 88200:  return AUDIO_SAMP_RATE_88K;
      case 96000:  return AUDIO_SAMP_RATE_96K;
      case 128000: return AUDIO_SAMP_RATE_128K;
      case 160000: return AUDIO_SAMP_RATE_160K;
      case 176400: return AUDIO_SAMP_RATE_172K;
      case 192000: return AUDIO_SAMP_RATE_192K;
      default:     return 0;
    }
}

static int usbhost_audio_addformat(FAR struct usbhost_audio_s *audio,
                                   FAR struct usbhost_audio_format_s *format,
                                   FAR const struct usb_epdesc_s *epdesc,
                                   FAR struct usbhost_audio_format_s **added)
{
  FAR struct usbhost_audio_stream_s *stream;
  uint8_t direction;

  if ((epdesc->attr & USB_EP_ATTR_USAGE_MASK) != USB_EP_ATTR_USAGE_DATA)
    {
      return -ENOTSUP;
    }

  direction = USB_ISEPIN(epdesc->addr) ? USBHOST_AUDIO_CAPTURE :
                                         USBHOST_AUDIO_PLAYBACK;
  stream = &audio->stream[direction];
  if (stream->nformats >= USBHOST_AUDIO_NFORMATS)
    {
      return -E2BIG;
    }

  format->epdesc.hport        = audio->usbclass.hport;
  format->epdesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
  format->epdesc.in           = direction == USBHOST_AUDIO_CAPTURE;
  format->epdesc.xfrtype      = USB_EP_ATTR_XFER_ISOC;
  format->epdesc.interval     = epdesc->interval;
  format->epdesc.mxpacketsize = usbhost_audio_getle16(epdesc->mxpacketsize);
  if (epdesc->len >= USB_SIZEOF_AUDIOEPDESC)
    {
      FAR const struct usb_audioepdesc_s *audioep =
        (FAR const struct usb_audioepdesc_s *)epdesc;

      format->syncaddr = audioep->synchaddr;
    }

  memcpy(&stream->format[stream->nformats], format, sizeof(*format));
  *added = &stream->format[stream->nformats++];
  return OK;
}

static uint8_t usbhost_audio_findfeature(FAR struct usbhost_audio_s *audio,
                                         uint8_t terminal)
{
  bool visited[UINT8_MAX + 1];
  uint8_t queue[UINT8_MAX + 1];
  unsigned int head = 0;
  unsigned int tail = 0;
  unsigned int entity;
  unsigned int i;

  if (terminal == 0)
    {
      return 0;
    }

  memset(visited, 0, sizeof(visited));
  queue[tail++] = terminal;
  visited[terminal] = true;

  while (head < tail)
    {
      entity = queue[head++];
      if (audio->feature[entity])
        {
          return entity;
        }

      if (audio->source[entity] != 0 &&
          !visited[audio->source[entity]])
        {
          visited[audio->source[entity]] = true;
          queue[tail++] = audio->source[entity];
        }

      for (i = 1; i <= UINT8_MAX; i++)
        {
          if (audio->source[i] == entity && !visited[i])
            {
              visited[i] = true;
              queue[tail++] = i;
            }
        }
    }

  return 0;
}

static int usbhost_audio_parse(FAR struct usbhost_audio_s *audio,
                               FAR const uint8_t *configdesc, int desclen)
{
  struct usbhost_audio_format_s format;
  FAR const struct usb_desc_s *desc;
  FAR const struct usb_ifdesc_s *ifdesc;
  FAR const struct usb_epdesc_s *epdesc;
  FAR struct usbhost_audio_format_s *added = NULL;
  FAR struct usbhost_epdesc_s *lastep = NULL;
  uint8_t protocol = ADC_PROTOCOL_UNDEF;
  bool streaming = false;
  bool control = false;
  bool pcm = false;
  bool haveformat = false;
  int offset = 0;
  int ret;
  int i;

  memset(&format, 0, sizeof(format));

  while (offset <= desclen - sizeof(struct usb_desc_s))
    {
      desc = (FAR const struct usb_desc_s *)&configdesc[offset];
      if (desc->len < sizeof(struct usb_desc_s) ||
          offset + desc->len > desclen)
        {
          return -EINVAL;
        }

      if (desc->type == USB_DESC_TYPE_INTERFACE)
        {
          if (desc->len < USB_SIZEOF_IFDESC)
            {
              return -EINVAL;
            }

          ifdesc = (FAR const struct usb_ifdesc_s *)desc;
          if (ifdesc->classid == USB_CLASS_AUDIO &&
              ifdesc->subclass == ADC_SUBCLASS_AUDIOCONTROL)
            {
              audio->controlif = ifdesc->ifno;
              audio->protocol = ifdesc->protocol;
              audio->controlpresent = true;
            }

          control = ifdesc->classid == USB_CLASS_AUDIO &&
                    ifdesc->subclass == ADC_SUBCLASS_AUDIOCONTROL;

          streaming = ifdesc->classid == USB_CLASS_AUDIO &&
                      ifdesc->subclass == ADC_SUBCLASS_AUDIOSTREAMING &&
                      ifdesc->alt != 0;
          protocol = ifdesc->protocol;
          pcm = false;
          haveformat = false;
          added = NULL;
          lastep = NULL;
          memset(&format, 0, sizeof(format));
          format.ifno = ifdesc->ifno;
          format.alt = ifdesc->alt;
        }
      else if (control && desc->type == ADC_CS_INTERFACE && desc->len >= 4)
        {
          uint8_t subtype = configdesc[offset + 2];
          uint8_t terminal = configdesc[offset + 3];

          if (subtype == ADC_AC_INPUT_TERMINAL &&
              audio->protocol == USBHOST_AUDIO_PROTOCOL_2 && desc->len >= 8)
            {
              audio->clockmap[terminal] = configdesc[offset + 7];
            }
          else if (subtype == ADC_AC_OUTPUT_TERMINAL && desc->len >= 8)
            {
              audio->source[terminal] = configdesc[offset + 7];
              if (audio->protocol == USBHOST_AUDIO_PROTOCOL_2 &&
                  desc->len >= 9)
                {
                  audio->clockmap[terminal] = configdesc[offset + 8];
                }
            }
          else if (subtype == ADC_AC_FEATURE_UNIT && desc->len >= 5)
            {
              audio->feature[terminal] = true;
              audio->source[terminal] = configdesc[offset + 4];
              if (audio->protocol == USBHOST_AUDIO_PROTOCOL_2 &&
                  desc->len >= 9)
                {
                  uint32_t controls =
                    usbhost_audio_getle32(&configdesc[offset + 5]);

                  if ((controls & USBHOST_AUDIO_UAC2_MUTE_CONTROL_MASK) ==
                      USBHOST_AUDIO_UAC2_MUTE_CONTROL_MASK)
                    {
                      audio->featurecontrols[terminal] |= AUDIO_FU_MUTE;
                    }

                  if ((controls & USBHOST_AUDIO_UAC2_VOLUME_CONTROL_MASK) ==
                      USBHOST_AUDIO_UAC2_VOLUME_CONTROL_MASK)
                    {
                      audio->featurecontrols[terminal] |= AUDIO_FU_VOLUME;
                    }
                }
              else if (desc->len >= 7 && configdesc[offset + 5] > 0)
                {
                  audio->featurecontrols[terminal] =
                    configdesc[offset + 6] &
                    (AUDIO_FU_MUTE | AUDIO_FU_VOLUME);
                }
            }
        }
      else if (streaming && desc->type == ADC_CS_INTERFACE &&
               desc->len >= 4)
        {
          if (configdesc[offset + 2] == ADC_AS_GENERAL)
            {
              if (protocol == USBHOST_AUDIO_PROTOCOL_2)
                {
                  if (desc->len < USB_SIZEOF_ADC_AS_IFDESC)
                    {
                      return -EINVAL;
                    }

                  pcm = (configdesc[offset + 6] & ADC_FORMAT_TYPEI_PCM) != 0;
                  format.channels = configdesc[offset + 10];
                  format.terminal = configdesc[offset + 3];
                  format.clockid =
                    audio->clockmap[configdesc[offset + 3]];
                }
              else
                {
                  if (desc->len < 7)
                    {
                      return -EINVAL;
                    }

                  pcm = usbhost_audio_getle16(&configdesc[offset + 5]) == 1;
                  format.terminal = configdesc[offset + 3];
                }
            }
          else if (configdesc[offset + 2] == ADC_AS_FORMAT_TYPE &&
                   configdesc[offset + 3] == ADC_FORMAT_TYPEI)
            {
              if (protocol == USBHOST_AUDIO_PROTOCOL_2)
                {
                  if (desc->len < USB_SIZEOF_ADC_T1_FORMAT_DESC)
                    {
                      return -EINVAL;
                    }

                  format.subslot = configdesc[offset + 4];
                  format.resolution = configdesc[offset + 5];
                  haveformat = true;
                }
              else
                {
                  uint8_t nrates;

                  if (desc->len < 8)
                    {
                      return -EINVAL;
                    }

                  format.channels = configdesc[offset + 4];
                  format.subslot = configdesc[offset + 5];
                  format.resolution = configdesc[offset + 6];
                  nrates = configdesc[offset + 7];
                  if (nrates == 0)
                    {
                      if (desc->len < 14)
                        {
                          return -EINVAL;
                        }

                      format.rate[0] =
                        usbhost_audio_getle24(&configdesc[offset + 8]);
                      format.rate[1] =
                        usbhost_audio_getle24(&configdesc[offset + 11]);
                      format.nrates = 2;
                      format.continuous = true;
                    }
                  else
                    {
                      if (desc->len < 8 + 3 * nrates)
                        {
                          return -EINVAL;
                        }

                      if (nrates > USBHOST_AUDIO_NRATES)
                        {
                          nrates = USBHOST_AUDIO_NRATES;
                        }

                      for (i = 0; i < nrates; i++)
                        {
                          format.rate[i] = usbhost_audio_getle24(
                            &configdesc[offset + 8 + 3 * i]);
                        }

                      format.nrates = nrates;
                    }

                  haveformat = true;
                }
            }
        }
      else if (streaming && desc->type == USB_DESC_TYPE_ENDPOINT &&
               pcm && haveformat)
        {
          if (desc->len < USB_SIZEOF_EPDESC)
            {
              return -EINVAL;
            }

          epdesc = (FAR const struct usb_epdesc_s *)desc;
          if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
              USB_EP_ATTR_XFER_ISOC)
            {
              if ((epdesc->attr & USB_EP_ATTR_USAGE_MASK) ==
                  USB_EP_ATTR_USAGE_FEEDBACK)
                {
                  FAR struct usbhost_audio_stream_s *playback =
                    &audio->stream[USBHOST_AUDIO_PLAYBACK];

                  if (playback->nformats > 0)
                    {
                      added = &playback->format[playback->nformats - 1];
                      if (added->ifno == format.ifno &&
                          added->alt == format.alt)
                        {
                          added->feedbackdesc.hport = audio->usbclass.hport;
                          added->feedbackdesc.addr = epdesc->addr &
                            USB_EP_ADDR_NUMBER_MASK;
                          added->feedbackdesc.in = USB_ISEPIN(epdesc->addr);
                          added->feedbackdesc.xfrtype =
                            USB_EP_ATTR_XFER_ISOC;
                          added->feedbackdesc.interval = epdesc->interval;
                          added->feedbackdesc.mxpacketsize =
                            usbhost_audio_getle16(epdesc->mxpacketsize);
                          added->feedback = true;
                          lastep = &added->feedbackdesc;
                        }
                    }
                }
              else
                {
                  ret = usbhost_audio_addformat(audio, &format, epdesc,
                                                &added);
                  if (ret < 0 && ret != -ENOTSUP)
                    {
                      return ret;
                    }

                  lastep = ret == OK ? &added->epdesc : NULL;
                }
            }
        }
      else if (streaming &&
               desc->type == USB_DESC_TYPE_ENDPOINT_COMPANION &&
               desc->len >= USB_SIZEOF_SS_EPCOMPDESC && lastep != NULL)
        {
          FAR const struct usb_ss_epcompdesc_s *comp =
            (FAR const struct usb_ss_epcompdesc_s *)desc;

          lastep->maxburst = comp->mxburst;
        }

      offset += desc->len;
    }

  for (i = 0; i < USBHOST_AUDIO_NSTREAMS; i++)
    {
      int j;

      for (j = 0; j < audio->stream[i].nformats; j++)
        {
          format = audio->stream[i].format[j];
          audio->stream[i].format[j].feature =
            usbhost_audio_findfeature(audio, format.terminal);
        }
    }

  return audio->controlpresent || audio->stream[0].nformats != 0 ||
         audio->stream[1].nformats != 0 ? OK : -ENODEV;
}

static bool usbhost_audio_rate_supported(
  FAR const struct usbhost_audio_format_s *format, uint32_t rate)
{
  int i;

  if (format->continuous)
    {
      return rate >= format->rate[0] && rate <= format->rate[1];
    }

  for (i = 0; i < format->nrates; i++)
    {
      if (format->rate[i] == rate)
        {
          return true;
        }
    }

  return format->nrates == 0;
}

static int usbhost_audio_setrate(FAR struct usbhost_audio_stream_s *stream,
                                 uint32_t rate)
{
  FAR struct usbhost_audio_s *audio = stream->audio;
  FAR struct usbhost_hubport_s *hport = audio->usbclass.hport;
  FAR struct usbhost_audio_format_s *format =
    &stream->format[stream->current];
  int ret;

  if (audio->ctrlbuflen < sizeof(uint32_t))
    {
      return -ENOSPC;
    }

  if (audio->protocol == USBHOST_AUDIO_PROTOCOL_2)
    {
      if (format->clockid == 0)
        {
          return -ENOTSUP;
        }

      nxmutex_lock(&audio->ctrllock);
      memset(audio->ctrlreq, 0, sizeof(*audio->ctrlreq));
      audio->ctrlreq->type = USB_REQ_DIR_OUT | USB_REQ_TYPE_CLASS |
                             USB_REQ_RECIPIENT_INTERFACE;
      audio->ctrlreq->req = ADC_REQUEST_CUR;
      usbhost_audio_putle16(audio->ctrlreq->value, 1 << 8);
      usbhost_audio_putle16(audio->ctrlreq->index,
                           ((uint16_t)format->clockid << 8) |
                           audio->controlif);
      usbhost_audio_putle16(audio->ctrlreq->len, sizeof(uint32_t));
      usbhost_audio_putle32(audio->ctrlbuf, rate);
      ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, audio->ctrlreq,
                         audio->ctrlbuf);
      nxmutex_unlock(&audio->ctrllock);
      return ret;
    }

  nxmutex_lock(&audio->ctrllock);
  memset(audio->ctrlreq, 0, sizeof(*audio->ctrlreq));
  audio->ctrlreq->type = USB_REQ_DIR_OUT | USB_REQ_TYPE_CLASS |
                         USB_REQ_RECIPIENT_ENDPOINT;
  audio->ctrlreq->req = ADC_REQUEST_CUR;
  usbhost_audio_putle16(audio->ctrlreq->value, 1 << 8);
  usbhost_audio_putle16(audio->ctrlreq->index,
                       format->epdesc.addr |
                       (format->epdesc.in ? USB_EP_ADDR_DIR_MASK : 0));
  usbhost_audio_putle16(audio->ctrlreq->len, 3);
  audio->ctrlbuf[0] = rate & 0xff;
  audio->ctrlbuf[1] = (rate >> 8) & 0xff;
  audio->ctrlbuf[2] = (rate >> 16) & 0xff;
  ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, audio->ctrlreq,
                     audio->ctrlbuf);
  nxmutex_unlock(&audio->ctrllock);
  return ret;
}

static int usbhost_audio_getrates(
  FAR struct usbhost_audio_s *audio,
  FAR struct usbhost_audio_format_s *format)
{
  FAR struct usbhost_hubport_s *hport = audio->usbclass.hport;
  size_t length;
  uint16_t nranges;
  uint32_t minimum;
  uint32_t maximum;
  uint32_t resolution;
  int ret;
  int i;

  if (audio->protocol != USBHOST_AUDIO_PROTOCOL_2 || format->clockid == 0)
    {
      return OK;
    }

  length = 2 + USBHOST_AUDIO_NRATES * 12;
  if (length > audio->ctrlbuflen)
    {
      length = audio->ctrlbuflen;
    }

  if (length < 14)
    {
      return -ENOSPC;
    }

  nxmutex_lock(&audio->ctrllock);
  memset(audio->ctrlreq, 0, sizeof(*audio->ctrlreq));
  memset(audio->ctrlbuf, 0, length);
  audio->ctrlreq->type = USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
                         USB_REQ_RECIPIENT_INTERFACE;
  audio->ctrlreq->req = ADC_REQUEST_RANGE;
  usbhost_audio_putle16(audio->ctrlreq->value, 1 << 8);
  usbhost_audio_putle16(audio->ctrlreq->index,
                       ((uint16_t)format->clockid << 8) |
                       audio->controlif);
  usbhost_audio_putle16(audio->ctrlreq->len, length);
  ret = DRVR_CTRLIN(hport->drvr, hport->ep0, audio->ctrlreq,
                    audio->ctrlbuf);
  if (ret < 0)
    {
      nxmutex_unlock(&audio->ctrllock);
      return ret;
    }

  nranges = usbhost_audio_getle16(audio->ctrlbuf);
  if (nranges > (length - 2) / 12)
    {
      nranges = (length - 2) / 12;
    }

  for (i = 0; i < nranges; i++)
    {
      minimum = usbhost_audio_getle32(&audio->ctrlbuf[2 + i * 12]);
      maximum = usbhost_audio_getle32(&audio->ctrlbuf[6 + i * 12]);
      resolution = usbhost_audio_getle32(&audio->ctrlbuf[10 + i * 12]);
      if (minimum == maximum &&
          format->nrates < USBHOST_AUDIO_NRATES)
        {
          format->rate[format->nrates++] = minimum;
        }
      else if (minimum <= maximum)
        {
          if (!format->continuous)
            {
              format->rate[0] = minimum;
              format->rate[1] = maximum;
            }
          else
            {
              if (minimum < format->rate[0])
                {
                  format->rate[0] = minimum;
                }

              if (maximum > format->rate[1])
                {
                  format->rate[1] = maximum;
                }
            }

          format->nrates = 2;
          format->continuous = resolution != 0 || minimum != maximum;
        }
    }

  nxmutex_unlock(&audio->ctrllock);
  return OK;
}

static int usbhost_audio_feature_request(
  FAR struct usbhost_audio_stream_s *stream, bool in, uint8_t request,
  uint8_t selector, FAR uint8_t *data, size_t length)
{
  FAR struct usbhost_audio_s *audio = stream->audio;
  FAR struct usbhost_audio_format_s *format =
    &stream->format[stream->current];
  FAR struct usbhost_hubport_s *hport = audio->usbclass.hport;
  int ret;

  if (format->feature == 0 || length > audio->ctrlbuflen)
    {
      return -ENOTSUP;
    }

  if ((audio->featurecontrols[format->feature] &
       (selector == USBHOST_AUDIO_FU_MUTE ? AUDIO_FU_MUTE :
        AUDIO_FU_VOLUME)) == 0)
    {
      return -ENOTSUP;
    }

  nxmutex_lock(&audio->ctrllock);
  if (!in)
    {
      memcpy(audio->ctrlbuf, data, length);
    }

  memset(audio->ctrlreq, 0, sizeof(*audio->ctrlreq));
  audio->ctrlreq->type = (in ? USB_REQ_DIR_IN : USB_REQ_DIR_OUT) |
                         USB_REQ_TYPE_CLASS |
                         USB_REQ_RECIPIENT_INTERFACE;
  audio->ctrlreq->req = request;
  usbhost_audio_putle16(audio->ctrlreq->value, (uint16_t)selector << 8);
  usbhost_audio_putle16(audio->ctrlreq->index,
                       ((uint16_t)format->feature << 8) |
                       audio->controlif);
  usbhost_audio_putle16(audio->ctrlreq->len, length);
  if (in)
    {
      ret = DRVR_CTRLIN(hport->drvr, hport->ep0, audio->ctrlreq,
                        audio->ctrlbuf);
      if (ret >= 0)
        {
          memcpy(data, audio->ctrlbuf, length);
        }
    }
  else
    {
      ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, audio->ctrlreq,
                         audio->ctrlbuf);
    }

  nxmutex_unlock(&audio->ctrllock);
  return ret;
}

static int usbhost_audio_setvolume(
  FAR struct usbhost_audio_stream_s *stream, uint16_t volume)
{
  FAR struct usbhost_audio_s *audio = stream->audio;
  int16_t minimum;
  int16_t maximum;
  int16_t value;
  uint8_t data[8];
  int ret;

  if (volume > 1000)
    {
      return -ERANGE;
    }

  if (audio->protocol == USBHOST_AUDIO_PROTOCOL_2)
    {
      ret = usbhost_audio_feature_request(stream, true, ADC_REQUEST_RANGE,
                                          USBHOST_AUDIO_FU_VOLUME, data, 8);
      if (ret < 0 || usbhost_audio_getle16(data) == 0)
        {
          return ret < 0 ? ret : -ERANGE;
        }

      minimum = (int16_t)usbhost_audio_getle16(&data[2]);
      maximum = (int16_t)usbhost_audio_getle16(&data[4]);
    }
  else
    {
      ret = usbhost_audio_feature_request(stream, true,
                                          USBHOST_AUDIO_GET_MIN,
                                          USBHOST_AUDIO_FU_VOLUME, data, 2);
      if (ret < 0)
        {
          return ret;
        }

      minimum = (int16_t)usbhost_audio_getle16(data);
      ret = usbhost_audio_feature_request(stream, true,
                                          USBHOST_AUDIO_GET_MAX,
                                          USBHOST_AUDIO_FU_VOLUME, data, 2);
      if (ret < 0)
        {
          return ret;
        }

      maximum = (int16_t)usbhost_audio_getle16(data);
    }

  value = minimum + ((int32_t)(maximum - minimum) * volume) / 1000;
  usbhost_audio_putle16(data, value);
  return usbhost_audio_feature_request(stream, false, ADC_REQUEST_CUR,
                                       USBHOST_AUDIO_FU_VOLUME, data, 2);
}

static int usbhost_audio_setmute(FAR struct usbhost_audio_stream_s *stream,
                                 bool mute)
{
  uint8_t data = mute;

  return usbhost_audio_feature_request(stream, false, ADC_REQUEST_CUR,
                                       USBHOST_AUDIO_FU_MUTE, &data, 1);
}

static void usbhost_audio_callback(FAR struct usbhost_audio_stream_s *stream,
                                   uint16_t reason,
                                   FAR struct ap_buffer_s *apb,
                                   int status)
{
  if (stream->dev.upper == NULL)
    {
      return;
    }

#ifdef CONFIG_AUDIO_MULTI_SESSION
  stream->dev.upper(stream->dev.priv, reason, apb,
                    status < 0 ? -status : status, NULL);
#else
  stream->dev.upper(stream->dev.priv, reason, apb,
                    status < 0 ? -status : status);
#endif
}

static FAR struct ap_buffer_s *
usbhost_audio_dequeue(FAR struct usbhost_audio_stream_s *stream)
{
  FAR dq_entry_t *entry;

  nxmutex_lock(&stream->lock);
  entry = dq_remfirst(&stream->pendq);
  nxmutex_unlock(&stream->lock);
  return entry == NULL ? NULL :
         container_of(entry, struct ap_buffer_s, dq_entry);
}

static int usbhost_audio_transfer(FAR struct usbhost_audio_stream_s *stream,
                                  FAR struct ap_buffer_s *apb)
{
  FAR struct usbhost_audio_format_s *format =
    &stream->format[stream->current];
  FAR struct usbhost_hubport_s *hport = stream->audio->usbclass.hport;
  size_t remaining;
  size_t packet;
  size_t maxpayload;
  size_t offset;
  uint32_t service;
  uint32_t frames;
  uint32_t framebytes;
  ssize_t nbytes;

  maxpayload = (format->epdesc.mxpacketsize & USB_EP_MAX_PACKET_MASK) *
               (USB_EP_MAX_PACKET_MULT(format->epdesc.mxpacketsize) + 1);
  framebytes = format->channels * format->subslot;
  if (maxpayload == 0 || framebytes == 0)
    {
      return -EINVAL;
    }

  if (stream->direction == USBHOST_AUDIO_PLAYBACK &&
      (apb->curbyte > apb->nbytes ||
       (apb->nbytes - apb->curbyte) % framebytes != 0))
    {
      return -EINVAL;
    }

  if (hport->speed >= USB_SPEED_HIGH)
    {
      service = 8000 / (1 << (format->epdesc.interval == 0 ? 0 :
                              format->epdesc.interval > 14 ? 13 :
                              format->epdesc.interval - 1));
    }
  else
    {
      service = 1000 / (1 << (format->epdesc.interval == 0 ? 0 :
                              format->epdesc.interval > 11 ? 10 :
                              format->epdesc.interval - 1));
    }

  if (service == 0)
    {
      service = 1;
    }

  offset = stream->direction == USBHOST_AUDIO_PLAYBACK ? apb->curbyte : 0;
  remaining = stream->direction == USBHOST_AUDIO_PLAYBACK ?
              apb->nbytes - apb->curbyte : apb->nmaxbytes;
  if (stream->direction == USBHOST_AUDIO_CAPTURE)
    {
      remaining -= remaining % framebytes;
      if (remaining == 0)
        {
          return -ENOSPC;
        }
    }

  while (remaining > 0 && !stream->terminate &&
         !stream->audio->disconnected)
    {
      stream->accumulator += stream->feedbackrate != 0 ?
                             stream->feedbackrate : stream->rate;
      frames = stream->accumulator / service;
      stream->accumulator %= service;
      packet = frames * framebytes;
      if (packet == 0)
        {
          packet = framebytes;
        }

      if (packet > maxpayload)
        {
          return -ERANGE;
        }

      if (packet > remaining)
        {
          packet = remaining;
        }

      nbytes = DRVR_TRANSFER(hport->drvr, stream->ep,
                            &apb->samp[offset], packet);
      if (nbytes < 0)
        {
          return nbytes;
        }

      if (nbytes == 0)
        {
          return -EIO;
        }

      if ((size_t)nbytes > remaining)
        {
          return -EOVERFLOW;
        }

      offset += nbytes;
      remaining -= nbytes;
    }

  if (stream->direction == USBHOST_AUDIO_PLAYBACK)
    {
      apb->curbyte = offset;
    }
  else
    {
      apb->nbytes = offset;
      apb->curbyte = 0;
    }

  return remaining == 0 ? OK : -ESHUTDOWN;
}

static FAR void *usbhost_audio_worker(FAR void *arg)
{
  FAR struct usbhost_audio_stream_s *stream = arg;
  FAR struct ap_buffer_s *apb;
  int ret;

  while (!stream->terminate)
    {
      ret = nxsem_wait_uninterruptible(&stream->sem);
      if (ret < 0 || stream->terminate)
        {
          break;
        }

      while (!stream->terminate &&
             (apb = usbhost_audio_dequeue(stream)) != NULL)
        {
          nxmutex_lock(&stream->lock);
          stream->active = apb;
          nxmutex_unlock(&stream->lock);
          ret = usbhost_audio_transfer(stream, apb);
          nxmutex_lock(&stream->lock);
          stream->active = NULL;
          nxmutex_unlock(&stream->lock);
          usbhost_audio_callback(stream, AUDIO_CALLBACK_DEQUEUE, apb, ret);
          if (ret < 0)
            {
              usbhost_audio_callback(stream, AUDIO_CALLBACK_IOERR, NULL,
                                     ret);
            }

          if ((apb->flags & AUDIO_APB_FINAL) != 0)
            {
              usbhost_audio_callback(stream, AUDIO_CALLBACK_COMPLETE, NULL,
                                     ret);
            }
        }
    }

  return NULL;
}

static FAR void *usbhost_audio_feedback_worker(FAR void *arg)
{
  FAR struct usbhost_audio_stream_s *stream = arg;
  FAR struct usbhost_hubport_s *hport = stream->audio->usbclass.hport;
  uint32_t raw;
  uint32_t scale;
  size_t length;
  ssize_t nbytes;

  length = hport->speed >= USB_SPEED_HIGH ||
           stream->audio->protocol == USBHOST_AUDIO_PROTOCOL_2 ? 4 : 3;
  while (!stream->feedbackstop && !stream->audio->disconnected)
    {
      nbytes = DRVR_TRANSFER(hport->drvr, stream->feedbackep,
                            stream->feedbackbuf, length);
      if (nbytes < 0)
        {
          if (!stream->feedbackstop && !stream->audio->disconnected)
            {
              uerr("feedback transfer failed: %zd\n", nbytes);
            }

          break;
        }

      if (nbytes == 3)
        {
          raw = usbhost_audio_getle24(stream->feedbackbuf);
          scale = hport->speed >= USB_SPEED_HIGH ? 8000 : 1000;
          stream->feedbackrate =
            ((uint64_t)raw * scale + (1 << 13)) >> 14;
        }
      else if (nbytes == 4)
        {
          raw = usbhost_audio_getle32(stream->feedbackbuf);
          scale = hport->speed >= USB_SPEED_HIGH ? 8000 : 1000;
          stream->feedbackrate =
            ((uint64_t)raw * scale + (1 << 15)) >> 16;
        }
    }

  return NULL;
}

static int usbhost_audio_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                                 FAR struct audio_caps_s *caps)
{
  FAR struct usbhost_audio_stream_s *stream =
    (FAR struct usbhost_audio_stream_s *)dev;
  uint16_t rates = 0;
  uint8_t channels = 0;
  int i;
  int j;

  UNUSED(type);

  caps->ac_format.hw = 0;
  caps->ac_controls.w = 0;

  if (caps->ac_type == AUDIO_TYPE_QUERY)
    {
      caps->ac_channels = 0;
      if (caps->ac_subtype == AUDIO_TYPE_QUERY)
        {
          caps->ac_controls.b[0] =
            stream->direction == USBHOST_AUDIO_PLAYBACK ? AUDIO_TYPE_OUTPUT :
                                                          AUDIO_TYPE_INPUT;
          caps->ac_format.hw = 1 << (AUDIO_FMT_PCM - 1);
        }

      return caps->ac_len;
    }

  if (caps->ac_type == AUDIO_TYPE_FEATURE)
    {
      if (caps->ac_subtype == AUDIO_FU_UNDEF)
        {
          for (i = 0; i < stream->nformats; i++)
            {
              if (stream->format[i].feature != 0)
                {
                  caps->ac_controls.b[0] |=
                    stream->audio->featurecontrols[
                      stream->format[i].feature];
                }
            }
        }

      return caps->ac_len;
    }

  if ((caps->ac_type == AUDIO_TYPE_OUTPUT &&
       stream->direction != USBHOST_AUDIO_PLAYBACK) ||
      (caps->ac_type == AUDIO_TYPE_INPUT &&
       stream->direction != USBHOST_AUDIO_CAPTURE))
    {
      return -ENOTTY;
    }

  for (i = 0; i < stream->nformats; i++)
    {
      if (stream->format[i].channels > channels)
        {
          channels = stream->format[i].channels;
        }

      if (stream->format[i].continuous)
        {
          rates |= AUDIO_SAMP_RATE_DEF_ALL;
        }
      else
        {
          if (stream->format[i].nrates == 0)
            {
              rates |= AUDIO_SAMP_RATE_DEF_ALL;
            }

          for (j = 0; j < stream->format[i].nrates; j++)
            {
              rates |= usbhost_audio_ratemask(stream->format[i].rate[j]);
            }
        }
    }

  caps->ac_channels = channels;

  if (caps->ac_subtype == AUDIO_TYPE_QUERY)
    {
      caps->ac_controls.hw[0] = rates;
    }

  return caps->ac_len;
}

static int usbhost_audio_configure_common(
  FAR struct usbhost_audio_stream_s *stream,
  FAR const struct audio_caps_s *caps)
{
  FAR struct usbhost_audio_format_s *format;
  FAR struct usbhost_hubport_s *hport = stream->audio->usbclass.hport;
  uint32_t rate = caps->ac_controls.hw[0] |
                  ((uint32_t)caps->ac_controls.b[3] << 16);
  uint8_t width = caps->ac_controls.b[2];
  int selected = -1;
  int ret;
  int i;

  if (stream->audio->disconnected || caps->ac_subtype != AUDIO_FMT_PCM)
    {
      return -ENODEV;
    }

  for (i = 0; i < stream->nformats; i++)
    {
      format = &stream->format[i];
      if (format->channels == caps->ac_channels &&
          format->resolution == width &&
          usbhost_audio_rate_supported(format, rate))
        {
          selected = i;
          break;
        }
    }

  if (selected < 0)
    {
      return -ERANGE;
    }

  nxmutex_lock(&stream->lock);
  if (stream->running)
    {
      nxmutex_unlock(&stream->lock);
      return -EBUSY;
    }

  if (stream->ep != NULL)
    {
      DRVR_EPFREE(hport->drvr, stream->ep);
      stream->ep = NULL;
    }

  stream->current = selected;
  stream->rate = rate;
  format = &stream->format[selected];
  ret = DRVR_EPALLOC(hport->drvr, &format->epdesc, &stream->ep);
  nxmutex_unlock(&stream->lock);
  return ret;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int usbhost_audio_configure(FAR struct audio_lowerhalf_s *dev,
                                   FAR void *session,
                                   FAR const struct audio_caps_s *caps)
#else
static int usbhost_audio_configure(FAR struct audio_lowerhalf_s *dev,
                                   FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct usbhost_audio_stream_s *stream =
    (FAR struct usbhost_audio_stream_s *)dev;

  if (caps->ac_type == AUDIO_TYPE_FEATURE)
    {
      switch (caps->ac_format.hw)
        {
          case AUDIO_FU_VOLUME:
            return usbhost_audio_setvolume(stream,
                                           caps->ac_controls.hw[0]);

          case AUDIO_FU_MUTE:
            return usbhost_audio_setmute(stream,
                                         caps->ac_controls.b[0] != 0);

          default:
            return -ENOTTY;
        }
    }

  if ((caps->ac_type == AUDIO_TYPE_OUTPUT &&
       stream->direction != USBHOST_AUDIO_PLAYBACK) ||
      (caps->ac_type == AUDIO_TYPE_INPUT &&
       stream->direction != USBHOST_AUDIO_CAPTURE))
    {
      return -ENOTTY;
    }

  return usbhost_audio_configure_common(stream, caps);
}

static int usbhost_audio_start_common(
  FAR struct usbhost_audio_stream_s *stream)
{
  FAR struct usbhost_audio_format_s *format;
  struct sched_param sparam;
  pthread_attr_t attr;
  int ret;

  nxmutex_lock(&stream->lock);
  if (stream->audio->disconnected)
    {
      nxmutex_unlock(&stream->lock);
      return -ENODEV;
    }

  if (stream->running)
    {
      nxmutex_unlock(&stream->lock);
      return OK;
    }

  format = &stream->format[stream->current];
  if (stream->ep == NULL)
    {
      ret = DRVR_EPALLOC(stream->audio->usbclass.hport->drvr,
                         &format->epdesc, &stream->ep);
      if (ret < 0)
        {
          nxmutex_unlock(&stream->lock);
          return ret;
        }
    }

  ret = usbhost_audio_setrate(stream, stream->rate);
  if (ret < 0 &&
      !(format->nrates == 1 && format->rate[0] == stream->rate))
    {
      nxmutex_unlock(&stream->lock);
      return ret;
    }

  ret = usbhost_audio_setinterface(stream->audio, format->ifno, format->alt);
  if (ret < 0)
    {
      nxmutex_unlock(&stream->lock);
      return ret;
    }

  stream->terminate = false;
  stream->feedbackstop = false;
  stream->feedbackrate = 0;
  stream->accumulator = 0;
  pthread_attr_init(&attr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  pthread_attr_setschedparam(&attr, &sparam);
  pthread_attr_setstacksize(&attr, CONFIG_USBHOST_AUDIO_STACKSIZE);
  ret = pthread_create(&stream->thread, &attr, usbhost_audio_worker, stream);
  if (ret == OK)
    {
      stream->running = true;
      pthread_setname_np(stream->thread,
                         stream->direction == USBHOST_AUDIO_PLAYBACK ?
                         "usb-audio-out" : "usb-audio-in");

      if (format->feedback)
        {
          ret = DRVR_EPALLOC(stream->audio->usbclass.hport->drvr,
                             &format->feedbackdesc, &stream->feedbackep);
          if (ret == OK)
            {
              ret = pthread_create(&stream->feedbackthread, &attr,
                                   usbhost_audio_feedback_worker, stream);
            }

          if (ret != OK)
            {
              stream->terminate = true;
              nxsem_post(&stream->sem);
            }
          else
            {
              stream->feedbackrunning = true;
              pthread_setname_np(stream->feedbackthread,
                                 "usb-audio-fb");
            }
        }
    }

  pthread_attr_destroy(&attr);
  if (ret != OK)
    {
      stream->terminate = true;
      stream->feedbackstop = true;
      nxsem_post(&stream->sem);
      nxmutex_unlock(&stream->lock);
      if (stream->running)
        {
          pthread_join(stream->thread, NULL);
        }

      nxmutex_lock(&stream->lock);
      stream->running = false;
      if (stream->feedbackep != NULL)
        {
          DRVR_EPFREE(stream->audio->usbclass.hport->drvr,
                      stream->feedbackep);
          stream->feedbackep = NULL;
        }

      usbhost_audio_setinterface(stream->audio, format->ifno, 0);
    }

  nxmutex_unlock(&stream->lock);
  return ret;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int usbhost_audio_start(FAR struct audio_lowerhalf_s *dev,
                               FAR void *session)
#else
static int usbhost_audio_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  return usbhost_audio_start_common(
    (FAR struct usbhost_audio_stream_s *)dev);
}

static int usbhost_audio_stop_common(
  FAR struct usbhost_audio_stream_s *stream)
{
  FAR struct usbhost_audio_format_s *format;
  FAR struct ap_buffer_s *apb;
  FAR void *value;
  bool wasrunning;

  nxmutex_lock(&stream->lock);
  wasrunning = stream->running;
  if (!wasrunning)
    {
      nxmutex_unlock(&stream->lock);
      goto drain;
    }

  stream->terminate = true;
  stream->feedbackstop = true;
  nxsem_post(&stream->sem);
  if (stream->ep != NULL)
    {
      DRVR_CANCEL(stream->audio->usbclass.hport->drvr, stream->ep);
    }

  if (stream->feedbackep != NULL)
    {
      DRVR_CANCEL(stream->audio->usbclass.hport->drvr,
                  stream->feedbackep);
    }

  nxmutex_unlock(&stream->lock);
  pthread_join(stream->thread, &value);
  if (stream->feedbackrunning)
    {
      pthread_join(stream->feedbackthread, &value);
      stream->feedbackrunning = false;
    }

  if (stream->feedbackep != NULL)
    {
      DRVR_EPFREE(stream->audio->usbclass.hport->drvr,
                  stream->feedbackep);
      stream->feedbackep = NULL;
    }

  nxmutex_lock(&stream->lock);
  stream->running = false;
  format = &stream->format[stream->current];
  if (!stream->audio->disconnected)
    {
      usbhost_audio_setinterface(stream->audio, format->ifno, 0);
    }

  if (stream->ep != NULL)
    {
      DRVR_EPFREE(stream->audio->usbclass.hport->drvr, stream->ep);
      stream->ep = NULL;
    }

  nxmutex_unlock(&stream->lock);

drain:
  while ((apb = usbhost_audio_dequeue(stream)) != NULL)
    {
      usbhost_audio_callback(stream, AUDIO_CALLBACK_DEQUEUE, apb,
                             -ESHUTDOWN);
    }

  if (wasrunning)
    {
      usbhost_audio_callback(stream, AUDIO_CALLBACK_COMPLETE, NULL, OK);
    }

  return OK;
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int usbhost_audio_stop(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session)
#  else
static int usbhost_audio_stop(FAR struct audio_lowerhalf_s *dev)
#  endif
{
  return usbhost_audio_stop_common(
    (FAR struct usbhost_audio_stream_s *)dev);
}
#endif

static int usbhost_audio_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  return usbhost_audio_stop_common(
    (FAR struct usbhost_audio_stream_s *)dev);
}

static int usbhost_audio_enqueue(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb)
{
  FAR struct usbhost_audio_stream_s *stream =
    (FAR struct usbhost_audio_stream_s *)dev;

  if (apb == NULL || stream->audio->disconnected)
    {
      return -ENODEV;
    }

  nxmutex_lock(&stream->lock);
  dq_addlast(&apb->dq_entry, &stream->pendq);
  nxmutex_unlock(&stream->lock);
  nxsem_post(&stream->sem);
  return OK;
}

static int usbhost_audio_cancel(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  FAR struct usbhost_audio_stream_s *stream =
    (FAR struct usbhost_audio_stream_s *)dev;
  FAR dq_entry_t *entry;
  bool found = false;
  bool active;

  nxmutex_lock(&stream->lock);
  active = stream->active == apb;
  for (entry = dq_peek(&stream->pendq); entry != NULL;
       entry = dq_next(entry))
    {
      if (entry == &apb->dq_entry)
        {
          dq_rem(entry, &stream->pendq);
          found = true;
          break;
        }
    }

  nxmutex_unlock(&stream->lock);

  if (active && stream->ep != NULL)
    {
      return DRVR_CANCEL(stream->audio->usbclass.hport->drvr, stream->ep);
    }

  if (found)
    {
      usbhost_audio_callback(stream, AUDIO_CALLBACK_DEQUEUE, apb,
                             -ECANCELED);
      return OK;
    }

  return -ENOENT;
}

static int usbhost_audio_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                               unsigned long arg)
{
  FAR struct usbhost_audio_stream_s *stream =
    (FAR struct usbhost_audio_stream_s *)dev;

  if (cmd == AUDIOIOC_GETBUFFERINFO)
    {
      FAR struct ap_buffer_info_s *info =
        (FAR struct ap_buffer_info_s *)(uintptr_t)arg;
      FAR struct usbhost_audio_format_s *format =
        &stream->format[stream->current];

      info->nbuffers = CONFIG_USBHOST_AUDIO_NBUFFERS;
      info->buffer_size =
        (format->epdesc.mxpacketsize & USB_EP_MAX_PACKET_MASK) *
        CONFIG_USBHOST_AUDIO_BUFFER_INTERVALS;
      return OK;
    }

  return -ENOTTY;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int usbhost_audio_reserve(FAR struct audio_lowerhalf_s *dev,
                                 FAR void **session)
#else
static int usbhost_audio_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct usbhost_audio_stream_s *stream =
    (FAR struct usbhost_audio_stream_s *)dev;
  int ret = OK;

  nxmutex_lock(&stream->lock);
  if (stream->reserved)
    {
      ret = -EBUSY;
    }
  else
    {
      stream->reserved = true;
#ifdef CONFIG_AUDIO_MULTI_SESSION
      *session = stream;
#endif
    }

  nxmutex_unlock(&stream->lock);
  return ret;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int usbhost_audio_release(FAR struct audio_lowerhalf_s *dev,
                                 FAR void *session)
#else
static int usbhost_audio_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct usbhost_audio_stream_s *stream =
    (FAR struct usbhost_audio_stream_s *)dev;

  usbhost_audio_stop_common(stream);
  nxmutex_lock(&stream->lock);
  stream->reserved = false;
  nxmutex_unlock(&stream->lock);
  return OK;
}

static int usbhost_audio_allocdevno(void)
{
  irqstate_t flags;
  int devno;

  flags = enter_critical_section();
  for (devno = 0; devno < 32; devno++)
    {
      if ((g_audio_devinuse & (1 << devno)) == 0)
        {
          g_audio_devinuse |= 1 << devno;
          leave_critical_section(flags);
          return devno;
        }
    }

  leave_critical_section(flags);
  return -EMFILE;
}

static void usbhost_audio_freedevno(uint8_t devno)
{
  irqstate_t flags = enter_critical_section();

  g_audio_devinuse &= ~(1 << devno);
  leave_critical_section(flags);
}

static void usbhost_audio_destroy(FAR void *arg)
{
  FAR struct usbhost_audio_s *audio = arg;
  FAR struct usbhost_audio_stream_s *stream;
  char devname[8];
  int i;

  for (i = 0; i < USBHOST_AUDIO_NSTREAMS; i++)
    {
      stream = &audio->stream[i];
      usbhost_audio_stop_common(stream);
      if (stream->registered)
        {
          snprintf(devname, sizeof(devname), "usb%c%u",
                   stream->direction == USBHOST_AUDIO_PLAYBACK ? 'p' : 'c',
                   stream->devno);
          audio_unregister(devname, &stream->dev);
          usbhost_audio_freedevno(stream->devno);
        }

      if (stream->ep != NULL)
        {
          DRVR_EPFREE(audio->usbclass.hport->drvr, stream->ep);
        }

      nxsem_destroy(&stream->sem);
      nxmutex_destroy(&stream->lock);
    }

  if (audio->ctrlbuf != NULL)
    {
      DRVR_FREE(audio->usbclass.hport->drvr, audio->ctrlbuf);
    }

  if (audio->ctrlreq != NULL)
    {
      DRVR_FREE(audio->usbclass.hport->drvr,
                (FAR uint8_t *)audio->ctrlreq);
    }

  nxmutex_destroy(&audio->ctrllock);

  kmm_free(audio);
}

static int usbhost_audio_register_stream(
  FAR struct usbhost_audio_stream_s *stream)
{
  char devname[8];
  int ret;

  if (stream->nformats == 0)
    {
      return OK;
    }

  ret = usbhost_audio_allocdevno();
  if (ret < 0)
    {
      return ret;
    }

  stream->devno = ret;
  snprintf(devname, sizeof(devname), "usb%c%u",
           stream->direction == USBHOST_AUDIO_PLAYBACK ? 'p' : 'c',
           stream->devno);
  ret = audio_register(devname, &stream->dev);
  if (ret < 0)
    {
      usbhost_audio_freedevno(stream->devno);
      return ret;
    }

  stream->registered = true;
  uinfo("registered /dev/audio/%s with %u formats\n",
        devname, stream->nformats);
  return OK;
}

static int usbhost_audio_connect(FAR struct usbhost_class_s *usbclass,
                                 FAR const uint8_t *configdesc,
                                 int desclen)
{
  FAR struct usbhost_audio_s *audio =
    (FAR struct usbhost_audio_s *)usbclass;
  int ret;
  int i;

  ret = DRVR_ALLOC(usbclass->hport->drvr,
                   (FAR uint8_t **)&audio->ctrlreq,
                   &audio->ctrlbuflen);
  if (ret < 0)
    {
      return ret;
    }

  ret = DRVR_ALLOC(usbclass->hport->drvr, &audio->ctrlbuf,
                   &audio->ctrlbuflen);
  if (ret < 0)
    {
      return ret;
    }

  ret = usbhost_audio_parse(audio, configdesc, desclen);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < USBHOST_AUDIO_NSTREAMS; i++)
    {
      int j;

      for (j = 0; j < audio->stream[i].nformats; j++)
        {
          ret = usbhost_audio_getrates(audio,
                                       &audio->stream[i].format[j]);
          if (ret < 0)
            {
              uinfo("clock range unavailable for stream %d format %d: %d\n",
                    i, j, ret);
            }
        }
    }

  for (i = 0; i < USBHOST_AUDIO_NSTREAMS; i++)
    {
      ret = usbhost_audio_register_stream(&audio->stream[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

static int usbhost_audio_disconnected(FAR struct usbhost_class_s *usbclass)
{
  FAR struct usbhost_audio_s *audio =
    (FAR struct usbhost_audio_s *)usbclass;
  int ret;

  audio->disconnected = true;
  ret = work_queue(LPWORK, &audio->destroywork,
                   usbhost_audio_destroy, audio, 0);
  if (ret < 0)
    {
      uerr("failed to schedule USB audio teardown: %d\n", ret);
    }

  return ret;
}

static FAR struct usbhost_class_s *
usbhost_audio_create(FAR struct usbhost_hubport_s *hport,
                     FAR const struct usbhost_id_s *id)
{
  FAR struct usbhost_audio_s *audio;
  FAR struct usbhost_audio_stream_s *stream;
  int i;

  audio = kmm_zalloc(sizeof(*audio));
  if (audio == NULL)
    {
      return NULL;
    }

  audio->usbclass.hport = hport;
  audio->usbclass.connect = usbhost_audio_connect;
  audio->usbclass.disconnected = usbhost_audio_disconnected;
  audio->protocol = id->proto;
  nxmutex_init(&audio->ctrllock);

  for (i = 0; i < USBHOST_AUDIO_NSTREAMS; i++)
    {
      stream = &audio->stream[i];
      stream->dev.ops = &g_audio_ops;
      stream->audio = audio;
      stream->direction = i;
      dq_init(&stream->pendq);
      nxmutex_init(&stream->lock);
      nxsem_init(&stream->sem, 0, 0);
    }

  return &audio->usbclass;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int usbhost_audio_initialize(void)
{
  return usbhost_registerclass(&g_audio_registry);
}

#endif /* CONFIG_USBHOST_AUDIO */
