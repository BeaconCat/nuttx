/****************************************************************************
 * drivers/usbdev/usbdev_fs.h
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

#ifndef __DRIVERS_USBDEV_FS_H
#define __DRIVERS_USBDEV_FS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/usb/usbdev.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Container to support a list of requests */

struct usbdev_fs_req_s
{
  sq_entry_t               node;    /* Implements a singly linked list */
  FAR struct usbdev_req_s *req;     /* The contained request */
  uint16_t                 offset;  /* Offset to valid data in the RX request */
};

/* Manage char device non blocking io */

typedef struct usbdev_fs_waiter_sem_s
{
  sem_t                              sem;
  FAR struct usbdev_fs_waiter_sem_s *next;
} usbdev_fs_waiter_sem_t;

/* This structure describes the char device */

struct usbdev_fs_ep_s
{
  uint16_t                    reqnum;
  uint16_t                    reqsize;
  FAR struct usbdev_ep_s     *ep;         /* EP entry */
  FAR usbdev_fs_waiter_sem_t *sems;       /* List of blocking request */
  struct sq_queue_s           reqq;       /* Available request containers */
  FAR struct usbdev_fs_req_s *reqbuffer;  /* Request buffer */
};

struct usbdev_fs_s
{
  FAR const char             *name;
  uint8_t                     crefs;      /* Count of opened instances */
  mutex_t                     lock;       /* Enforces device exclusive access */
  struct usbdev_fs_ep_s       fs_epin;
  struct usbdev_fs_ep_s       fs_epout;
  FAR struct pollfd          *fds[CONFIG_USBDEV_NPOLLWAITERS];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_fs_bind
 *
 * Description:
 *   Bind usbdev fs device to ep.
 *
 ****************************************************************************/

int usbdev_fs_bind(FAR const char *path, FAR struct usbdev_fs_s *fs);

/****************************************************************************
 * Name: usbdev_fs_unregister
 *
 * Description:
 *   Unbind usbdev fs device to ep.
 *
 ****************************************************************************/

int usbdev_fs_unbind(FAR struct usbdev_fs_s *fs);

/****************************************************************************
 * Name: usbdev_fs_connect
 *
 * Description:
 *   Notify usbdev fs device connect state.
 *
 ****************************************************************************/

void usbdev_fs_connect(FAR struct usbdev_fs_s *fs, int connect);

/****************************************************************************
 * Name: usbdev_fs_submit_rdreqs
 *
 * Description:
 *   Submit rdreq nodes to usb controller.
 *
 ****************************************************************************/

void usbdev_fs_submit_rdreqs(FAR struct usbdev_fs_ep_s *fs_ep);

#endif /* __DRIVERS_USBDEV_FS_H */
