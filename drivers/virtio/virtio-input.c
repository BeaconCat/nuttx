/****************************************************************************
 * drivers/virtio/virtio-input.c
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

#include <stdio.h>
#include <errno.h>
#include <debug.h>
#include <stdint.h>
#include <string.h>

#include <sys/param.h>

#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/input/kbd_codec.h>

#include "virtio-input.h"
#include "virtio-input-event-codes.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_INPUT_EVENT           0
#define VIRTIO_INPUT_NUM             1

#define VIRTIO_INPUT_CFG_UNSET       0x00
#define VIRTIO_INPUT_CFG_ID_NAME     0x01
#define VIRTIO_INPUT_CFG_ID_SERIAL   0x02
#define VIRTIO_INPUT_CFG_ID_DEVIDS   0x03
#define VIRTIO_INPUT_CFG_PROP_BITS   0x10
#define VIRTIO_INPUT_CFG_EV_BITS     0x11
#define VIRTIO_INPUT_CFG_ABS_INFO    0x12

#define VIRTIO_INPUT_EVT_SIZE        8
#define VIRTIO_INPUT_DOWN            1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct virtio_input_priv;
typedef void (*virtio_send_event_handler)(struct virtio_input_priv *,
                                          struct virtio_input_event *);

struct virtio_input_priv
{
  struct virtqueue              *vq;
  FAR struct virtio_device      *vdev;
  char                          name[NAME_MAX]; /* Device name */
  struct virtio_input_event     evt[VIRTIO_INPUT_EVT_SIZE];
  int                           evtnum;         /* Input event number */
  struct work_s                 work;           /* Supports the interrupt handling "bottom half" */
  virtio_send_event_handler     eventhandler;
  uint8_t                       touchid;             /* Touch ID */

  union
  {
    struct mouse_lowerhalf_s    mouselower;     /* Mouse device lowerhalf instance */
    struct keyboard_lowerhalf_s keyboardlower;  /* Keyboard device lowerhalf instance */
    struct touch_lowerhalf_s    touchlower;     /* Touchpad device lowerhalf instance */
  };

  union
  {
    struct mouse_report_s       mousesample;    /* Mouse event */
    struct keyboard_event_s     keyboardsample; /* Keyboard event */
    struct touch_sample_s       touchsample;    /* Touchpad event */
  };
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int virtio_input_probe(FAR struct virtio_device *vdev);
static void virtio_input_remove(FAR struct virtio_device *vdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_driver g_virtio_input_driver =
{
  .node   = LIST_INITIAL_VALUE(g_virtio_input_driver.node), /* node */
  .device = VIRTIO_ID_INPUT,                                /* device id */
  .probe  = virtio_input_probe,                             /* probe */
  .remove = virtio_input_remove,                            /* remove */
};

static int g_virtio_mouse_idx = 0;
static int g_virtio_touch_idx = 0;
static int g_virtio_kbd_idx   = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_keycode_translate
 ****************************************************************************/

static uint32_t virtio_keycode_translate(uint16_t keycode)
{
  switch (keycode)
    {
      case KEY_DELETE:
        return KEYCODE_FWDDEL;
      case KEY_BACKSPACE:
        return KEYCODE_BACKDEL;
      case KEY_HOME:
        return KEYCODE_HOME;
      case KEY_END:
        return KEYCODE_END;
      case KEY_LEFT:
        return KEYCODE_LEFT;
      case KEY_RIGHT:
        return KEYCODE_RIGHT;
      case KEY_UP:
        return KEYCODE_UP;
      case KEY_DOWN:
        return KEYCODE_DOWN;
      case KEY_PAGEUP:
        return KEYCODE_PAGEUP;
      case KEY_PAGEDOWN:
        return KEYCODE_PAGEDOWN;
      case KEY_ENTER:
        return KEYCODE_ENTER;
      case KEY_CAPSLOCK:
        return KEYCODE_CAPSLOCK;
      case KEY_SCROLLLOCK:
        return KEYCODE_SCROLLLOCK;
      case KEY_NUMLOCK:
        return KEYCODE_NUMLOCK;
      case KEY_SYSRQ:
        return KEYCODE_PRTSCRN;
      case KEY_F1:
        return KEYCODE_F1;
      case KEY_F2:
        return KEYCODE_F2;
      case KEY_F3:
        return KEYCODE_F3;
      case KEY_F4:
        return KEYCODE_F4;
      case KEY_F5:
        return KEYCODE_F5;
      case KEY_F6:
        return KEYCODE_F6;
      case KEY_F7:
        return KEYCODE_F7;
      case KEY_F8:
        return KEYCODE_F8;
      case KEY_F9:
        return KEYCODE_F9;
      case KEY_F10:
        return KEYCODE_F10;
      case KEY_F11:
        return KEYCODE_F11;
      case KEY_F12:
        return KEYCODE_F12;
      case KEY_F13:
        return KEYCODE_F13;
      case KEY_F14:
        return KEYCODE_F14;
      case KEY_F15:
        return KEYCODE_F15;
      case KEY_F16:
        return KEYCODE_F16;
      case KEY_F17:
        return KEYCODE_F17;
      case KEY_F18:
        return KEYCODE_F18;
      case KEY_F19:
        return KEYCODE_F19;
      case KEY_F20:
        return KEYCODE_F20;
      case KEY_F21:
        return KEYCODE_F21;
      case KEY_F22:
        return KEYCODE_F22;
      case KEY_F23:
        return KEYCODE_F23;
      case KEY_F24:
        return KEYCODE_F24;
      default:
        return keycode;
    }
}

/****************************************************************************
 * Name: virtio_send_keyboard_event
 ****************************************************************************/

static void virtio_send_keyboard_event(struct virtio_input_priv *priv,
                                       struct virtio_input_event *event)
{
  if (event->type == EV_KEY)
    {
      priv->keyboardsample.code = virtio_keycode_translate(event->code);
      priv->keyboardsample.type = event->value;
    }
  else if (event->type == EV_SYN && event->code == SYN_REPORT)
    {
      keyboard_event(&(priv->keyboardlower),
                     priv->keyboardsample.code,
                     priv->keyboardsample.type);
      memset(&priv->keyboardsample, 0, sizeof(priv->keyboardsample));
    }
}

/****************************************************************************
 * Name: virtio_send_mouse_event
 ****************************************************************************/

static void virtio_send_mouse_event(struct virtio_input_priv *priv,
                                    struct virtio_input_event *event)
{
  if (event->type == EV_REL)
    {
      switch (event->code)
        {
          case REL_X:
            priv->mousesample.x = event->value;
            break;

          case REL_Y:
            priv->mousesample.y = event->value;
            break;

        #ifdef CONFIG_INPUT_MOUSE_WHEEL
          case REL_WHEEL:
            priv->mousesample.wheel = event->value;
            break;
        #endif

          default:
            break;
        }
    }
  else if (event->type == EV_KEY)
    {
      switch (event->code)
        {
          case BTN_LEFT:
            priv->mousesample.buttons = (event->value == VIRTIO_INPUT_DOWN ?
                                         MOUSE_BUTTON_1 : MOUSE_BUTTON_4);
            break;

          case BTN_RIGHT:
            priv->mousesample.buttons = (event->value == VIRTIO_INPUT_DOWN ?
                                         MOUSE_BUTTON_2 : MOUSE_BUTTON_5);
            break;

          case BTN_MIDDLE:
            priv->mousesample.buttons = (event->value == VIRTIO_INPUT_DOWN ?
                                         MOUSE_BUTTON_3 : MOUSE_BUTTON_6);
            break;

          default:
            priv->mousesample.buttons = event->code;
            break;
        }
    }
  else if (event->type == EV_SYN && event->code == SYN_REPORT)
    {
      mouse_event(priv->mouselower.priv, &priv->mousesample);
      memset(&priv->mousesample, 0, sizeof(priv->mousesample));
    }
}

/****************************************************************************
 * Name: virtio_send_touch_event
 ****************************************************************************/

static void virtio_send_touch_event(struct virtio_input_priv *priv,
                                    struct virtio_input_event *event)
{
  if (event->type == EV_ABS)
    {
      switch (event->code)
        {
          case ABS_PRESSURE:
            priv->touchsample.point[0].pressure = event->value;
            break;

          case ABS_X:
            priv->touchsample.point[0].x = event->value;
            break;

          case ABS_Y:
            priv->touchsample.point[0].y = event->value;
            break;

          default:
            break;
        }
    }
  else if (event->type == EV_SYN && event->code == SYN_REPORT)
    {
      priv->touchsample.npoints = 1;
      priv->touchsample.point[0].gesture = 0xff;
      priv->touchsample.point[0].w = 1;
      priv->touchsample.point[0].timestamp = touch_get_time();
      priv->touchsample.point[0].id = priv->touchid++;

      touch_event(priv->touchlower.priv, &priv->touchsample);
      memset(&priv->touchsample, 0, sizeof(priv->touchsample));
    }
}

/****************************************************************************
 * Name: input_worker
 ****************************************************************************/

static void input_worker(FAR void *arg)
{
  FAR struct virtio_input_priv *priv = (FAR struct virtio_input_priv *)arg;
  FAR struct virtio_input_event *evt;
  FAR struct virtqueue_buf vb;
  uint32_t len;

  while ((evt = (FAR struct virtio_input_event *)
         virtqueue_get_buffer(priv->vq, &len, NULL)) != NULL)
    {
      vrtinfo("input_worker (type,code,value) - (%d,%d,%d).\n",
              evt->type, evt->code, evt->value);

      priv->eventhandler(priv, evt);

      vb.buf = evt;
      vb.len = len;
      virtqueue_add_buffer(priv->vq, &vb, 0, 1, vb.buf);
    }

  virtqueue_kick(priv->vq);
  return;
}

/****************************************************************************
 * Name: virtio_input_recv_events
 ****************************************************************************/

static void virtio_input_recv_events(FAR struct virtqueue *vq)
{
  FAR struct virtio_input_priv *priv = vq->vq_dev->priv;
  int ret;

  priv->vq = vq;

  ret = work_queue(HPWORK, &priv->work, input_worker, priv, 0);
  if (ret != 0)
    {
      vrterr("ERROR: Failed to queue work: %d\n", ret);
    }

  return;
}

/****************************************************************************
 * Name: virtio_input_evtfill
 ****************************************************************************/

static void virtio_input_evtfill(FAR struct virtio_input_priv *priv)
{
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_INPUT_EVENT].vq;
  FAR struct virtqueue_buf vb;
  int i;

  for (i = 0; i < priv->evtnum; i++)
    {
      vb.buf = &priv->evt[i];
      vb.len = sizeof(struct virtio_input_event);
      virtqueue_add_buffer(vq, &vb, 0, 1, vb.buf);
    }

  if (i > 0)
    {
      virtqueue_kick(vq);
    }
}

/****************************************************************************
 * Name: virtio_input_select_cfg
 ****************************************************************************/

static uint8_t virtio_input_select_cfg(FAR struct virtio_input_priv *priv,
                                       uint8_t select, uint8_t subsel)
{
  uint8_t cfg_size = 0;
  virtio_write_config(priv->vdev, offsetof(struct virtio_input_config,
                      select), &select, sizeof(uint8_t));
  virtio_write_config(priv->vdev, offsetof(struct virtio_input_config,
                      subsel), &subsel, sizeof(uint8_t));
  virtio_read_config(priv->vdev, offsetof(struct virtio_input_config,
                     size), &cfg_size, sizeof(uint8_t));
  return cfg_size;
}

/****************************************************************************
 * Name: virtio_input_register
 ****************************************************************************/

static void virtio_input_register(struct virtio_input_priv *priv,
                                  int select)
{
  if (virtio_input_select_cfg(priv, select, EV_ABS) != 0)
    {
      priv->touchid = 0;
      priv->touchlower.maxpoint = 1;
      snprintf(priv->name, NAME_MAX, "/dev/virtinput%d",
               g_virtio_touch_idx);
      touch_register(&(priv->touchlower),
                     priv->name,
                     CONFIG_DRIVERS_VIRTIO_INPUT_BUFNUM);
      priv->eventhandler = virtio_send_touch_event;
      g_virtio_touch_idx++;
    }
  else if (virtio_input_select_cfg(priv, select, EV_REL) != 0)
    {
      snprintf(priv->name, NAME_MAX, "/dev/virtmouse%d",
               g_virtio_mouse_idx);
      mouse_register(&(priv->mouselower),
                     priv->name,
                     CONFIG_DRIVERS_VIRTIO_INPUT_BUFNUM);
      priv->eventhandler = virtio_send_mouse_event;
      g_virtio_mouse_idx++;
    }
  else if (virtio_input_select_cfg(priv, select, EV_KEY) != 0)
    {
      snprintf(priv->name, NAME_MAX, "/dev/virtkbd%d",
               g_virtio_kbd_idx);
      keyboard_register(&(priv->keyboardlower),
                        priv->name,
                        CONFIG_DRIVERS_VIRTIO_INPUT_BUFNUM);
      priv->eventhandler = virtio_send_keyboard_event;
      g_virtio_kbd_idx++;
    }
}

/****************************************************************************
 * Name: virtio_input_probe
 ****************************************************************************/

static int virtio_input_probe(FAR struct virtio_device *vdev)
{
  struct virtio_input_priv *priv;
  FAR const char *vqnames[VIRTIO_INPUT_NUM];
  vq_callback callbacks[VIRTIO_INPUT_NUM];
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  priv->vdev = vdev;
  vdev->priv = priv;

  /* Initialize the virtio device */

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
  virtio_set_features(vdev, 0);
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  vqnames[VIRTIO_INPUT_EVENT] = "virtio_input_event";
  callbacks[VIRTIO_INPUT_EVENT] = virtio_input_recv_events;
  ret = virtio_create_virtqueues(vdev, 0, VIRTIO_INPUT_NUM, vqnames,
                                 callbacks);
  if (ret < 0)
    {
      vrterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
      virtio_reset_device(vdev);
      kmm_free(priv);
      return ret;
    }

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);

  /* register lower half drivers */

  virtio_input_register(priv, VIRTIO_INPUT_CFG_EV_BITS);

  virtqueue_enable_cb(vdev->vrings_info[VIRTIO_INPUT_EVENT].vq);
  priv->evtnum = MIN(vdev->vrings_info[VIRTIO_INPUT_EVENT].info.num_descs,
                     VIRTIO_INPUT_EVT_SIZE);

  virtio_input_evtfill(priv);

  return ret;
}

/****************************************************************************
 * Name: virtio_input_remove
 ****************************************************************************/

static void virtio_input_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_input_priv *priv = vdev->priv;

  if (priv->eventhandler == virtio_send_keyboard_event)
    {
      keyboard_unregister(&(priv->keyboardlower), priv->name);
    }
  else if (priv->eventhandler == virtio_send_mouse_event)
    {
      mouse_unregister(&(priv->mouselower), priv->name);
    }
  else if (priv->eventhandler == virtio_send_touch_event)
    {
      touch_unregister(&(priv->touchlower), priv->name);
    }

  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_register_input_driver
 ****************************************************************************/

int virtio_register_input_driver(void)
{
  return virtio_register_driver(&g_virtio_input_driver);
}
