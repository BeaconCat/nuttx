/****************************************************************************
 * include/nuttx/audio/es8388.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_ES8388_H
#define __INCLUDE_NUTTX_AUDIO_ES8388_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

#ifdef CONFIG_AUDIO_ES8388

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 *
 * CONFIG_AUDIO_ES8388 - Enables ES8388 support
 * CONFIG_ES8388_INITVOLUME - The initial volume level
 *                             in the range {0..1000}
 * CONFIG_ES8388_INFLIGHT - Maximum number of buffers that the ES8388
 *   driver will send to the I2S driver before any have completed.
 * CONFIG_ES8388_MSG_PRIO - Priority of messages sent to the ES8388
 *   worker thread.
 * CONFIG_ES8388_BUFFER_SIZE - Preferred buffer size
 * CONFIG_ES8388_NUM_BUFFERS - Preferred number of buffers
 * CONFIG_ES8388_WORKER_STACKSIZE - Stack size to use when creating the the
 *   ES8388 worker thread.
 * CONFIG_ES8388_REGDUMP - Enable logic to dump all ES8388 registers to
 *   the SYSLOG device.
 */

/* Pre-requisites */

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO is required for audio subsystem support
#endif

#ifndef CONFIG_I2S
#  error CONFIG_I2S is required by the ES8388 driver
#endif

#ifndef CONFIG_I2C
#  error CONFIG_I2C is required by the ES8388 driver
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error CONFIG_SCHED_WORKQUEUE is required by the ES8388 driver
#endif

/* Default configuration values */

#ifndef CONFIG_ES8388_INPUT_INITVOLUME
#  define CONFIG_ES8388_INPUT_INITVOLUME       1000
#endif

#ifndef CONFIG_ES8388_OUTPUT_INITVOLUME
#  define CONFIG_ES8388_OUTPUT_INITVOLUME      400
#endif

#ifndef CONFIG_ES8388_INFLIGHT
#  define CONFIG_ES8388_INFLIGHT               2
#endif

#if CONFIG_ES8388_INFLIGHT > 255
#  error CONFIG_ES8388_INFLIGHT must fit in a uint8_t
#endif

#ifndef CONFIG_ES8388_MSG_PRIO
#  define CONFIG_ES8388_MSG_PRIO               1
#endif

#ifndef CONFIG_ES8388_BUFFER_SIZE
#  define CONFIG_ES8388_BUFFER_SIZE            8192
#endif

#ifndef CONFIG_ES8388_NUM_BUFFERS
#  define CONFIG_ES8388_NUM_BUFFERS            4
#endif

#ifndef CONFIG_ES8388_WORKER_STACKSIZE
#  define CONFIG_ES8388_WORKER_STACKSIZE       2048
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum es8388_output_route_e {
  ES8388_OUTPUT_ROUTE_NONE = 0,
  ES8388_OUTPUT_ROUTE_LINE1,
  ES8388_OUTPUT_ROUTE_LINE2,
  ES8388_OUTPUT_ROUTE_BOTH,
  ES8388_OUTPUT_ROUTE_AUTO,
};

enum es8388_input_route_e
{
  ES8388_INPUT_ROUTE_NONE = 0,
  ES8388_INPUT_ROUTE_LINE1,
  ES8388_INPUT_ROUTE_LINE2,
  ES8388_INPUT_ROUTE_BOTH,
};

#define ES8388IOC_SET_CONTROL _AUDIOIOC(0x80)
#define ES8388IOC_GET_CONTROL _AUDIOIOC(0x81)

#define ES8388_CONTROL_OUTPUT (1 << 0)
#define ES8388_CONTROL_INPUT  (1 << 1)
#define ES8388_CONTROL_ALL    (ES8388_CONTROL_OUTPUT | ES8388_CONTROL_INPUT)

struct es8388_control_s {
  uint32_t mask;
  enum es8388_output_route_e route;
  enum es8388_output_route_e active_route;
  enum es8388_input_route_e input_route;
  bool mono;
  bool swap;
  bool invert_left;
  bool invert_right;
  bool microphone_muted;
  bool headphones_connected;
};

struct es8388_lower_s;

struct es8388_lower_s
{
  /* I2C characterization */

  uint32_t frequency;  /* Initial I2C frequency */
  uint8_t  address;    /* 7-bit I2C address (only bits 0-6 used) */
  uint8_t stream_type; /* 0: legacy; AUDIO_TYPE_INPUT or AUDIO_TYPE_OUTPUT */

  /* Fixed-direction instances sharing a physical codec require compatible
   * formats and synchronized I2S clocks. Stream queues remain independent.
   */

  /* Board-level DAC channel wiring and analog output control. */

  bool swap_dac_lr;
  CODE void (*set_output)(enum es8388_output_route_e route, bool enable);
  CODE enum es8388_output_route_e (*resolve_output)(void);
  CODE bool (*headphones_connected)(void);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: es8388_initialize
 *
 * Description:
 *   Initialize the ES8388 device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   i2s     - An I2S driver instance
 *   lower   - Persistent board configuration data
 *
 * Returned Value:
 *   A new lower half audio interface for the ES8388 device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

struct i2c_master_s;
struct i2s_dev_s;
struct audio_lowerhalf_s;

FAR struct audio_lowerhalf_s *
es8388_initialize(FAR struct i2c_master_s *i2c, FAR struct i2s_dev_s *i2s,
                  FAR const struct es8388_lower_s *lower);

/****************************************************************************
 * Name: es8388_dump_registers
 *
 * Description:
 *   Dump the contents of all ES8388 registers to the syslog device
 *
 * Input Parameters:
 *   dev - The device instance returned by es8388_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ES8388_REGDUMP
void es8388_dump_registers(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg);
#else
#  define es8388_dump_registers(d,m)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AUDIO_ES8388 */
#endif /* __INCLUDE_NUTTX_AUDIO_ES8388_H */
