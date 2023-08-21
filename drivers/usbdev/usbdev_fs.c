/****************************************************************************
 * drivers/usbdev/usbdev_fs.c
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

#include <debug.h>
#include <fcntl.h>
#include <poll.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/mutex.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/fs/fs.h>

#include "usbdev_fs.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int usbdev_fs_open(FAR struct file *filep);
static int usbdev_fs_close(FAR struct file *filep);
static ssize_t usbdev_fs_read(FAR struct file *filep, FAR char *buffer,
                              size_t len);
static ssize_t usbdev_fs_write(FAR struct file *filep,
                               FAR const char *buffer, size_t len);
static int usbdev_fs_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_usbdev_fops =
{
  usbdev_fs_open,  /* open */
  usbdev_fs_close, /* close */
  usbdev_fs_read,  /* read */
  usbdev_fs_write, /* write */
  NULL,            /* seek */
  NULL,            /* ioctl */
  NULL,            /* mmap */
  NULL,            /* truncate */
  usbdev_fs_poll   /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_fs_notify
 *
 * Description:
 *   Notify threads waiting to read device. This function must be called
 *   with interrupt disabled.
 *
 ****************************************************************************/

static void usbdev_fs_notify(FAR struct usbdev_fs_s *fs,
                             FAR struct usbdev_fs_ep_s *fs_ep)
{
  /* Notify all of the waiting readers */

  FAR usbdev_fs_waiter_sem_t *cur_sem = fs_ep->sems;
  while (cur_sem != NULL)
    {
      nxsem_post(&cur_sem->sem);
      cur_sem = cur_sem->next;
    }

  fs_ep->sems = NULL;

  /* Notify all poll/select waiters */

  poll_notify(fs->fds, CONFIG_USBDEV_NPOLLWAITERS, POLLIN);
}

/****************************************************************************
 * Name: usbdev_fs_submit_wrreq
 *
 * Description:
 *   Handle completion of write request on the bulk IN endpoint.
 *
 ****************************************************************************/

static int usbdev_fs_submit_wrreq(FAR struct usbdev_ep_s *ep,
                                  FAR struct usbdev_fs_req_s *container,
                                  uint16_t len)
{
  FAR struct usbdev_req_s *req = container->req;

  req->len   = len;
  req->flags = 0;
  req->priv  = container;
  return EP_SUBMIT(ep, req);
}

/****************************************************************************
 * Name: usbdev_fs_submit_rdreq
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.
 *
 ****************************************************************************/

static int usbdev_fs_submit_rdreq(FAR struct usbdev_ep_s *ep,
                                  FAR struct usbdev_fs_req_s *container)
{
  FAR struct usbdev_req_s *req = container->req;

  req->len = ep->maxpacket;
  return EP_SUBMIT(ep, req);
}

/****************************************************************************
 * Name: usbdev_fs_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.
 *
 ****************************************************************************/

static void usbdev_fs_rdcomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
  FAR struct usbdev_fs_req_s *container;
  FAR struct usbdev_fs_s *priv;
  FAR struct usbdev_fs_ep_s *fs_ep;
  irqstate_t flags;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !ep->priv || !req)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract references to private data */

  priv      = (FAR struct usbdev_fs_s *)ep->fs;
  fs_ep     = &priv->fs_epout;
  container = (FAR struct usbdev_fs_req_s *)req->priv;

  /* Process the received data unless this is some unusual condition */

  switch (req->result)
    {
      case 0: /* Normal completion */

      usbtrace(TRACE_CLASSRDCOMPLETE, sq_count(&fs_ep->reqq));

      /* Restart request due to either no reader or
       * empty frame received.
       */

      if (priv->crefs == 0)
        {
          uwarn("drop frame\n");
          goto restart_req;
        }

      if (req->xfrd <= 0)
        {
          goto restart_req;
        }

      /* Queue request and notify readers */

      flags = enter_critical_section();

      /* Put request on RX pending queue */

      container->offset = 0;
      sq_addlast(&container->node, &fs_ep->reqq);

      usbdev_fs_notify(priv, fs_ep);

      leave_critical_section(flags);
      return;

      case -ESHUTDOWN: /* Disconnection */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSHUTDOWN), 0);
      return;

      default: /* Some other error occurred */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDUNEXPECTED),
               (uint16_t)-req->result);
      break;
    };

restart_req:

  /* Restart request */

  usbdev_fs_submit_rdreq(fs_ep->ep, container);
}

/****************************************************************************
 * Name: usbdev_fs_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static void usbdev_fs_wrcomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
  FAR struct usbdev_fs_req_s *container;
  FAR struct usbdev_fs_s *priv;
  FAR struct usbdev_fs_ep_s *fs_ep;
  irqstate_t flags;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !ep->priv || !req || !req->priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract references to private data */

  priv      = (FAR struct usbdev_fs_s *)ep->fs;
  fs_ep     = &priv->fs_epin;
  container = (FAR struct usbdev_fs_req_s *)req->priv;

  /* Return the write request to the free list */

  flags = enter_critical_section();
  sq_addlast(&container->node, &fs_ep->reqq);

  /* Check for termination condition */

  switch (req->result)
    {
      case OK: /* Normal completion */
        {
          usbtrace(TRACE_CLASSWRCOMPLETE, sq_count(&fs_ep->reqq));

          /* Notify all waiting writers that write req is available */

          usbdev_fs_waiter_sem_t *cur_sem = fs_ep->sems;
          while (cur_sem != NULL)
            {
              nxsem_post(&cur_sem->sem);
              cur_sem = cur_sem->next;
            }

          fs_ep->sems = NULL;

          /* Notify all poll/select waiters */

          poll_notify(priv->fds, CONFIG_USBDEV_NPOLLWAITERS, POLLOUT);
        }
        break;

      case -ESHUTDOWN: /* Disconnection */
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRSHUTDOWN),
                   sq_count(&fs_ep->reqq));
        }
        break;

      default: /* Some other error occurred */
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRUNEXPECTED),
                   (uint16_t)-req->result);
        }
        break;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: usbdev_fs_blocking_io
 *
 * Description:
 *   Handle read/write blocking io.
 *
 ****************************************************************************/

static int usbdev_fs_blocking_io(FAR struct usbdev_fs_s *priv,
                                 FAR usbdev_fs_waiter_sem_t *sem,
                                 FAR usbdev_fs_waiter_sem_t **slist,
                                 FAR struct sq_queue_s *queue)
{
  int ret;
  irqstate_t flags;

  flags = enter_critical_section();

  if (!sq_empty(queue))
    {
      /* Queue not empty after all */

      leave_critical_section(flags);
      return 0;
    }

  /* Register waiter semaphore */

  sem->next = *slist;
  *slist = sem;

  leave_critical_section(flags);

  nxmutex_unlock(&priv->lock);

  /* Wait for USB device to notify */

  ret = nxsem_wait(&sem->sem);

  if (ret < 0)
    {
      /* Interrupted wait, unregister semaphore
       * TODO ensure that lock wait does not fail (ECANCELED)
       */

      nxmutex_lock(&priv->lock);

      flags = enter_critical_section();

      FAR usbdev_fs_waiter_sem_t *cur_sem = *slist;

      if (cur_sem == sem)
        {
          *slist = sem->next;
        }
      else
        {
          while (cur_sem)
            {
              if (cur_sem->next == sem)
                {
                  cur_sem->next = sem->next;
                  break;
                }
            }
        }

      leave_critical_section(flags);
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  return nxmutex_lock(&priv->lock);
}

/****************************************************************************
 * Name: usbdev_fs_open
 *
 * Description:
 *   Open usbdev fs device. Only one open() instance is supported.
 *
 ****************************************************************************/

static int usbdev_fs_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_fs_s *priv = inode->i_private;
  int ret;

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  finfo("entry: <%s> %d\n", inode->i_name, priv->crefs);

  priv->crefs += 1;

  assert(priv->crefs != 0);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: usbdev_fs_close
 *
 * Description:
 *   Close usbdev fs device.
 *
 ****************************************************************************/

static int usbdev_fs_close(FAR struct file *filep)
{
  int ret;
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_fs_s *priv = inode->i_private;

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  finfo("entry: <%s> %d\n", inode->i_name, priv->crefs);

  priv->crefs -= 1;

  assert(priv->crefs >= 0);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: usbdev_fs_read
 *
 * Description:
 *   Read usbdev fs device.
 *
 ****************************************************************************/

static ssize_t usbdev_fs_read(FAR struct file *filep, FAR char *buffer,
                              size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_fs_s *priv = inode->i_private;
  FAR struct usbdev_fs_ep_s *fs_ep = &priv->fs_epout;
  irqstate_t flags;
  ssize_t ret;
  size_t retlen;

  assert(len > 0 && buffer != NULL);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check for available data */

  if (sq_empty(&fs_ep->reqq))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          nxmutex_unlock(&priv->lock);
          return -EAGAIN;
        }

      usbdev_fs_waiter_sem_t sem;
      nxsem_init(&sem.sem, 0, 0);

      do
        {
          /* RX queue seems empty. Check again with interrupts disabled */

          ret = usbdev_fs_blocking_io(
            priv, &sem, &fs_ep->sems, &fs_ep->reqq);
          if (ret < 0)
            {
              nxsem_destroy(&sem.sem);
              return ret;
            }
        }
      while (sq_empty(&fs_ep->reqq));

      /* RX queue not empty and lock locked so we are the only reader */

      nxsem_destroy(&sem.sem);
    }

  /* Device ready for read */

  retlen = 0;

  while (!sq_empty(&fs_ep->reqq) && len > 0)
    {
      FAR struct usbdev_fs_req_s *container;
      uint16_t reqlen;

      /* Process each packet in the priv->reqq list */

      container = container_of(sq_peek(&fs_ep->reqq),
                               struct usbdev_fs_req_s, node);

      reqlen = container->req->xfrd - container->offset;

      if (reqlen > len)
        {
          /* Output buffer full */

          memcpy(&buffer[retlen],
                 &container->req->buf[container->offset],
                 len);
          container->offset += len;
          retlen += len;
          break;
        }

      memcpy(&buffer[retlen],
             &container->req->buf[container->offset], reqlen);
      retlen += reqlen;
      len -= reqlen;

      /* The entire packet was processed and may be removed from the
       * pending RX list.
       */

      /* FIXME use atomic queue primitives ? */

      flags = enter_critical_section();
      sq_remfirst(&fs_ep->reqq);
      leave_critical_section(flags);

      ret = usbdev_fs_submit_rdreq(fs_ep->ep, container);
      if (ret != OK)
        {
          /* TODO handle error */

          PANIC();
        }
    }

  nxmutex_unlock(&priv->lock);
  return retlen;
}

/****************************************************************************
 * Name: usbdev_fs_write
 *
 * Description:
 *   Write adb device.
 *
 ****************************************************************************/

static ssize_t usbdev_fs_write(FAR struct file *filep,
                               FAR const char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_fs_s *priv = inode->i_private;
  FAR struct usbdev_fs_ep_s *fs_ep = &priv->fs_epin;
  FAR struct usbdev_fs_req_s *container;
  FAR struct usbdev_req_s *req;
  irqstate_t flags;
  int wlen;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check for available write request */

  if (sq_empty(&fs_ep->reqq))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
        }

      usbdev_fs_waiter_sem_t sem;
      nxsem_init(&sem.sem, 0, 0);

      do
        {
          /* TX queue seems empty. Check again with interrupts disabled */

          ret = usbdev_fs_blocking_io(
            priv, &sem, &fs_ep->sems, &fs_ep->reqq);
          if (ret < 0)
            {
              nxsem_destroy(&sem.sem);
              return ret;
            }
        }
      while (sq_empty(&fs_ep->reqq));

      nxsem_destroy(&sem.sem);
    }

  /* Device ready for write */

  wlen = 0;

  while (len > 0 && !sq_empty(&fs_ep->reqq))
    {
      uint16_t cur_len;

      /* Get available TX request slot */

      flags = enter_critical_section();

      container = container_of(sq_remfirst(&fs_ep->reqq),
                               struct usbdev_fs_req_s, node);

      leave_critical_section(flags);

      req = container->req;

      /* Fill the request with data */

      if (len > fs_ep->ep->maxpacket)
        {
          cur_len = fs_ep->ep->maxpacket;
        }
      else
        {
          cur_len = len;
        }

      memcpy(req->buf, &buffer[wlen], cur_len);

      /* Then submit the request to the endpoint */

      ret = usbdev_fs_submit_wrreq(fs_ep->ep, container, cur_len);
      if (ret != OK)
        {
          /* TODO add tx request back in txfree queue */

          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL),
                   (uint16_t)-ret);
          PANIC();
          break;
        }

      wlen += cur_len;
      len -= cur_len;
    }

  assert(wlen > 0);
  ret = wlen;

errout:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: adb_char_poll
 *
 * Description:
 *   Poll adb device.
 *
 ****************************************************************************/

static int usbdev_fs_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_fs_s *priv = inode->i_private;
  FAR struct usbdev_fs_ep_s *fs_epin = &priv->fs_epin;
  FAR struct usbdev_fs_ep_s *fs_epout = &priv->fs_epout;
  pollevent_t eventset;
  irqstate_t flags;
  int ret;
  int i;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!setup)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
      goto errout;
    }

  /* FIXME only parts of this function required interrupt disabled */

  flags = enter_critical_section();

  /* This is a request to set up the poll. Find an available
   * slot for the poll structure reference
   */

  for (i = 0; i < CONFIG_USBDEV_NPOLLWAITERS; i++)
    {
      /* Find an available slot */

      if (!priv->fds[i])
        {
          /* Bind the poll structure and this slot */

          priv->fds[i] = fds;
          fds->priv    = &priv->fds[i];
          break;
        }
    }

  if (i >= CONFIG_USBDEV_NPOLLWAITERS)
    {
      fds->priv = NULL;
      ret       = -EBUSY;
      goto exit_leave_critical;
    }

  eventset = 0;

  /* Notify the POLLIN/POLLOUT event if at least one request is available */

  if (fs_epin && !sq_empty(&fs_epin->reqq))
    {
      eventset |= POLLOUT;
    }

  if (fs_epout && !sq_empty(&fs_epout->reqq))
    {
      eventset |= POLLIN;
    }

  poll_notify(priv->fds, CONFIG_USBDEV_NPOLLWAITERS, eventset);

exit_leave_critical:
  leave_critical_section(flags);
errout:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: usbdev_fs_ep_bind
 *
 * Description:
 *   Bind usbdev fs device.
 *
 ****************************************************************************/

static int usbdev_fs_ep_bind(FAR struct usbdev_fs_s *fs,
                             FAR struct usbdev_fs_ep_s *fs_ep,
                             uint8_t dir)
{
  FAR struct usbdev_ep_s *ep = fs_ep->ep;
  uint16_t i;

  /* Initialize request queue */

  sq_init(&fs_ep->reqq);

  /* Initialize request buffer */

  fs_ep->reqbuffer =
    kmm_zalloc(sizeof(struct usbdev_fs_req_s) * fs_ep->reqnum);
  if (!fs_ep->reqbuffer)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), 0);
      return -ENOMEM;
    }

  for (i = 0; i < fs_ep->reqnum; i++)
    {
      FAR struct usbdev_fs_req_s *container;

      container = &fs_ep->reqbuffer[i];
      container->req = usbdev_allocreq(ep, fs_ep->reqsize);
      if (container->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), -ENOMEM);
          return -ENOMEM;
        }

      container->offset    = 0;
      container->req->priv = container;

      if (dir == USB_DIR_IN)
        {
          container->req->callback = usbdev_fs_wrcomplete;
          sq_addlast(&container->node, &fs_ep->reqq);
        }
    }

  ep->fs = fs;

  return OK;
}

/****************************************************************************
 * Name: usbdev_fs_ep_unbind
 *
 * Description:
 *   Register usbdev fs endpoint.
 *
 ****************************************************************************/

static void usbdev_fs_ep_unbind(FAR struct usbdev_fs_ep_s *fs_ep)
{
  if (fs_ep->reqbuffer)
    {
      uint16_t i;

      for (i = 0; i < fs_ep->reqnum; i++)
        {
          FAR struct usbdev_fs_req_s *container =
            &fs_ep->reqbuffer[i];
          if (container->req)
            {
              usbdev_freereq(fs_ep->ep, container->req);
            }
        }

      kmm_free(fs_ep->reqbuffer);
      fs_ep->reqbuffer = NULL;
    }

  sq_init(&fs_ep->reqq);
  fs_ep->ep = NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_fs_bind
 *
 * Description:
 *   Bind usbdev fs device.
 *
 ****************************************************************************/

int usbdev_fs_bind(FAR const char *path, FAR struct usbdev_fs_s *fs)
{
  int ret;

  /* Initialize the char device structure */

  nxmutex_init(&fs->lock);
  fs->crefs = 0;

  if (fs->fs_epin.ep)
    {
      ret = usbdev_fs_ep_bind(fs, &fs->fs_epin, USB_DIR_IN);
      if (ret < 0)
        {
          uerr("Failed to bind fs ep in");
          goto errout;
        }
    }

  if (fs->fs_epout.ep)
    {
      ret = usbdev_fs_ep_bind(fs, &fs->fs_epout, USB_DIR_OUT);
      if (ret < 0)
        {
          uerr("Failed to bind fs ep out");
          goto errout;
        }
    }

  /* Register char device driver */

  fs->name = path;
  ret = register_driver(fs->name, &g_usbdev_fops, 0666, fs);
  if (ret < 0)
    {
      uerr("Failed to register char device");
      goto errout;
    }

  return OK;

errout:
  usbdev_fs_unbind(fs);
  return ret;
}

/****************************************************************************
 * Name: usbdev_fs_unbind
 *
 * Description:
 *   Unregister usbdev fs device.
 *
 ****************************************************************************/

int usbdev_fs_unbind(FAR struct usbdev_fs_s *fs)
{
  fs->crefs = 0;
  unregister_driver(fs->name);

  if (fs->fs_epin.ep)
    {
      usbdev_fs_ep_unbind(&fs->fs_epin);
    }

  if (fs->fs_epout.ep)
    {
      usbdev_fs_ep_unbind(&fs->fs_epout);
    }

  nxmutex_destroy(&fs->lock);
  return OK;
}

/****************************************************************************
 * Name: usbdev_fs_connect
 *
 * Description:
 *   Notify usbdev fs device connect state.
 *
 ****************************************************************************/

void usbdev_fs_connect(FAR struct usbdev_fs_s *fs, int connect)
{
  FAR struct usbdev_fs_ep_s *fs_epin = &fs->fs_epin;
  FAR struct usbdev_fs_ep_s *fs_epout = &fs->fs_epout;
  FAR usbdev_fs_waiter_sem_t *cur_sem;
  irqstate_t flags = enter_critical_section();

  if (connect)
    {
      /* Notify poll/select with POLLIN */

      poll_notify(fs->fds, CONFIG_USBDEV_NPOLLWAITERS, POLLIN);
    }
  else
    {
      /* Notify all of the char device */

      if (fs_epin)
        {
          cur_sem = fs_epin->sems;
          while (cur_sem != NULL)
            {
              nxsem_post(&cur_sem->sem);
              cur_sem = cur_sem->next;
            }

          fs_epin->sems = NULL;
        }

      if (fs_epout)
        {
          cur_sem = fs_epout->sems;
          while (cur_sem != NULL)
            {
              nxsem_post(&cur_sem->sem);
              cur_sem = cur_sem->next;
            }

          fs_epout->sems = NULL;
        }

      /* Notify all poll/select waiters that a hangup occurred */

      poll_notify(fs->fds, CONFIG_USBDEV_NPOLLWAITERS, POLLERR | POLLHUP);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: usbdev_fs_submit_rdreqs
 *
 * Description:
 *   Submit rdreq nodes to usb controller.
 *
 ****************************************************************************/

void usbdev_fs_submit_rdreqs(FAR struct usbdev_fs_ep_s *fs_ep)
{
  FAR struct usbdev_fs_req_s *container;
  uint16_t i;

  for (i = 0; i < fs_ep->reqnum; i++)
    {
      container = &fs_ep->reqbuffer[i];
      container->req->callback = usbdev_fs_rdcomplete;
      usbdev_fs_submit_rdreq(fs_ep->ep, container);
    }
}
