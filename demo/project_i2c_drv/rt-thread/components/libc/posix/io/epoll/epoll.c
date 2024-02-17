/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-29   zmq810150896   first version
 */

#include <rtthread.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>
#include <dfs_file.h>
#include "sys/epoll.h"
#include "poll.h"
#include <lwp_signal.h>

#define EPOLL_MUTEX_NAME "EVENTEPOLL"

#define EFD_SHARED_EPOLL_TYPE (EPOLL_CTL_ADD | EPOLL_CTL_DEL | EPOLL_CTL_MOD)
#define EPOLLINOUT_BITS (EPOLLIN | EPOLLOUT | EPOLLRDNORM | EPOLLWRNORM)
#define EPOLLEXCLUSIVE_BITS (EPOLLINOUT_BITS | EPOLLERR | EPOLLHUP | \
                EPOLLET | EPOLLEXCLUSIVE)

struct rt_eventpoll;

/* Monitor queue */
struct rt_fd_list
{
    rt_uint32_t revents; /* Monitored events */
    struct epoll_event epev;
    rt_pollreq_t req;
    struct rt_eventpoll *ep;
    struct rt_wqueue_node wqn;
    int exclusive;/* If triggered horizontally, a check is made to see if the data has been read, and if there is any data left to read, the readability event is returned in the next epoll_wait */
    rt_bool_t is_rdl_node;
    int fd;
    struct rt_fd_list *next;
    rt_slist_t rdl_node;
};

struct rt_eventpoll
{
    rt_uint32_t tirggered;      /* the waited thread whether triggered */
    rt_wqueue_t epoll_read;
    rt_thread_t polling_thread;
    struct rt_mutex lock;
    struct rt_fd_list *fdlist;  /* Monitor list */
    int eventpoll_num;          /* Number of ready lists */
    rt_pollreq_t req;
    struct rt_spinlock spinlock;
    rt_slist_t rdl_head;
};

static int epoll_close(struct dfs_file *file);
static int epoll_poll(struct dfs_file *file, struct rt_pollreq *req);
static int epoll_get_event(struct rt_fd_list *fl, rt_pollreq_t *req);
static int epoll_do_ctl(int epfd, int op, int fd, struct epoll_event *event);

static const struct dfs_file_ops epoll_fops =
{
    .close      = epoll_close,
    .poll       = epoll_poll,
};

static int epoll_close_fdlist(struct rt_fd_list *fdlist)
{
    struct rt_fd_list *fre_node, *list;

    if (fdlist != RT_NULL)
    {
        list = fdlist;
        while (list->next != RT_NULL)
        {
            fre_node = list->next;
            if (fre_node->wqn.wqueue)
                rt_wqueue_remove(&fre_node->wqn);

            list->next = fre_node->next;
            rt_free(fre_node);
        }

        rt_free(fdlist);
    }

    return 0;
}

static int epoll_close(struct dfs_file *file)
{
    struct rt_eventpoll *ep;

    if (file->vnode->ref_count != 1)
        return 0;

    if (file->vnode)
    {
        if (file->vnode->data)
        {
            ep = file->vnode->data;
            if (ep)
            {
                rt_mutex_take(&ep->lock, RT_WAITING_FOREVER);
                if (ep->fdlist)
                {
                    epoll_close_fdlist(ep->fdlist);
                }

                rt_mutex_release(&ep->lock);
                rt_mutex_detach(&ep->lock);
                rt_free(ep);
            }
        }
    }

    return 0;
}

static int epoll_poll(struct dfs_file *file, struct rt_pollreq *req)
{
    struct rt_eventpoll *ep;
    int events = 0;
    rt_base_t level;

    if (file->vnode->data)
    {
        ep = file->vnode->data;
        ep->req._key = req->_key;

        rt_mutex_take(&ep->lock, RT_WAITING_FOREVER);
        rt_poll_add(&ep->epoll_read, req);

        level = rt_spin_lock_irqsave(&ep->spinlock);

        if (!rt_slist_isempty(&ep->rdl_head))
            events |= POLLIN | EPOLLRDNORM | POLLOUT;

        rt_spin_unlock_irqrestore(&ep->spinlock, level);
        rt_mutex_release(&ep->lock);
    }

    return events;
}

static int epoll_wqueue_callback(struct rt_wqueue_node *wait, void *key)
{
    struct rt_fd_list *fdlist;
    struct rt_eventpoll *ep;
    rt_base_t level;

    if (key && !((rt_ubase_t)key & wait->key))
        return -1;

    fdlist = rt_container_of(wait, struct rt_fd_list, wqn);

    ep = fdlist->ep;
    if (ep)
    {
        level = rt_spin_lock_irqsave(&ep->spinlock);
        if (fdlist->is_rdl_node == RT_FALSE)
        {
            rt_slist_append(&ep->rdl_head, &fdlist->rdl_node);
            fdlist->exclusive = 0;
            fdlist->is_rdl_node = RT_TRUE;
            ep->tirggered = 1;
            ep->eventpoll_num ++;
            rt_wqueue_wakeup(&ep->epoll_read, (void *)POLLIN);
        }
        rt_spin_unlock_irqrestore(&ep->spinlock, level);
    }

    return __wqueue_default_wake(wait, key);
}

static void epoll_wqueue_add_callback(rt_wqueue_t *wq, rt_pollreq_t *req)
{
    struct rt_fd_list *fdlist;
    struct rt_eventpoll *ep;

    fdlist = rt_container_of(req, struct rt_fd_list, req);

    ep = fdlist->ep;
    fdlist->wqn.key = req->_key;

    rt_list_init(&(fdlist->wqn.list));

    fdlist->wqn.polling_thread = ep->polling_thread;
    fdlist->wqn.wakeup = epoll_wqueue_callback;
    rt_wqueue_add(wq, &fdlist->wqn);
}

static void epoll_ctl_install(struct rt_fd_list *fdlist, struct rt_eventpoll *ep)
{
    rt_uint32_t mask = 0;
    rt_base_t level;

    fdlist->req._key = fdlist->revents;

    mask = epoll_get_event(fdlist, &fdlist->req);

    if (mask & fdlist->revents)
    {
        if (ep)
        {
            rt_mutex_take(&ep->lock, RT_WAITING_FOREVER);
            level = rt_spin_lock_irqsave(&ep->spinlock);
            rt_slist_append(&ep->rdl_head, &fdlist->rdl_node);
            fdlist->exclusive = 0;
            fdlist->is_rdl_node = RT_TRUE;
            ep->tirggered = 1;
            ep->eventpoll_num ++;
            rt_spin_unlock_irqrestore(&ep->spinlock, level);
            rt_mutex_release(&ep->lock);
        }
    }
}

static void epoll_member_init(struct rt_eventpoll *ep)
{
    ep->tirggered = 0;
    ep->eventpoll_num = 0;
    ep->polling_thread = rt_thread_self();
    ep->fdlist = RT_NULL;
    ep->req._key = 0;
    rt_slist_init(&(ep->rdl_head));
    rt_wqueue_init(&ep->epoll_read);
    rt_mutex_init(&ep->lock, EPOLL_MUTEX_NAME, RT_IPC_FLAG_FIFO);
    rt_spin_lock_init(&ep->spinlock);
}

static int epoll_epf_init(int fd)
{
    struct dfs_file *df;
    struct rt_eventpoll *ep;
    rt_err_t ret = 0;

    df = fd_get(fd);

    if (df)
    {
        ep = (struct rt_eventpoll *)rt_malloc(sizeof(struct rt_eventpoll));
        if (ep)
        {
            epoll_member_init(ep);

            #ifdef RT_USING_DFS_V2
            df->fops = &epoll_fops;
            #endif

            df->vnode = (struct dfs_vnode *)rt_malloc(sizeof(struct dfs_vnode));
            if (df->vnode)
            {
                ep->fdlist = (struct rt_fd_list *)rt_malloc(sizeof(struct rt_fd_list));
                if (ep->fdlist)
                {
                    ep->fdlist->next = RT_NULL;
                    ep->fdlist->fd = fd;
                    ep->fdlist->ep = ep;
                    ep->fdlist->exclusive = 0;
                    ep->fdlist->is_rdl_node = RT_FALSE;
                    dfs_vnode_init(df->vnode, FT_REGULAR, &epoll_fops);
                    df->vnode->data = ep;
                    rt_slist_init(&ep->fdlist->rdl_node);
                }
                else
                {
                    ret = -ENOMEM;
                    rt_free(df->vnode);
                    rt_free(ep);
                }
            }
            else
            {
                ret = -ENOMEM;
                rt_free(ep);
            }
        }
        else
        {
            ret = -ENOMEM;
        }
    }

    return ret;
}

static int epoll_do_create(int size)
{
    rt_err_t ret = -1;
    int status;
    int fd;

    if (size < 0)
    {
        rt_set_errno(EINVAL);
    }
    else
    {
        fd = fd_new();
        if (fd >= 0)
        {
            ret = fd;
            status = epoll_epf_init(fd);
            if (status < 0)
            {
                fd_release(fd);
                rt_set_errno(-status);
            }
        }
        else
        {
            rt_set_errno(-fd);
        }
    }

    return ret;
}

static int epoll_ctl_add(struct dfs_file *df, int fd, struct epoll_event *event)
{
    struct rt_fd_list *fdlist;
    struct rt_eventpoll *ep;
    rt_err_t ret = -EINVAL;

    if (df->vnode->data)
    {
        ep = df->vnode->data;
        fdlist = ep->fdlist;
        ret = 0;

        rt_mutex_take(&ep->lock, RT_WAITING_FOREVER);
        while (fdlist->next != RT_NULL)
        {
            if (fdlist->next->fd == fd)
            {
                rt_mutex_release(&ep->lock);
                return 0;
            }
            fdlist = fdlist->next;
        }
        rt_mutex_release(&ep->lock);

        fdlist = (struct rt_fd_list *)rt_malloc(sizeof(struct rt_fd_list));
        if (fdlist)
        {
            fdlist->fd = fd;
            memcpy(&fdlist->epev.data, &event->data, sizeof(event->data));
            fdlist->epev.events = 0;
            fdlist->ep = ep;
            fdlist->exclusive = 0;
            fdlist->is_rdl_node = RT_FALSE;
            fdlist->req._proc = epoll_wqueue_add_callback;
            fdlist->revents = event->events;
            rt_mutex_take(&ep->lock, RT_WAITING_FOREVER);
            fdlist->next = ep->fdlist->next;
            ep->fdlist->next = fdlist;
            rt_mutex_release(&ep->lock);
            rt_slist_init(&fdlist->rdl_node);

            epoll_ctl_install(fdlist, ep);
        }
        else
        {
            ret = -ENOMEM;
        }
    }

    return ret;
}

static int epoll_ctl_del(struct dfs_file *df, int fd)
{
    struct rt_fd_list *fdlist, *fre_fd, *rdlist;
    struct rt_eventpoll *ep = RT_NULL;
    rt_slist_t *node = RT_NULL;
    rt_err_t ret = -EINVAL;
    rt_base_t level;

    if (df->vnode->data)
    {
        ep = df->vnode->data;

        if (ep)
        {
            rt_mutex_take(&ep->lock, RT_WAITING_FOREVER);
            level = rt_spin_lock_irqsave(&ep->spinlock);
            rt_slist_for_each(node, &ep->rdl_head)
            {
                rdlist = rt_slist_entry(node, struct rt_fd_list, rdl_node);
                if (rdlist->fd == fd)
                    rt_slist_remove(&ep->rdl_head, node);
            }
            rt_spin_unlock_irqrestore(&ep->spinlock, level);

            fdlist = ep->fdlist;
            while (fdlist->next != RT_NULL)
            {
                if (fdlist->next->fd == fd)
                {
                    fre_fd = fdlist->next;
                    fdlist->next = fdlist->next->next;

                    if (fre_fd->wqn.wqueue)
                        rt_wqueue_remove(&fre_fd->wqn);

                    rt_free(fre_fd);
                    break;
                }
                else
                {
                    fdlist = fdlist->next;
                }
            }

            rt_mutex_release(&ep->lock);
        }

        ret = 0;
    }

    return ret;
}

static int epoll_ctl_mod(struct dfs_file *df, int fd, struct epoll_event *event)
{
    struct rt_fd_list *fdlist;
    struct rt_eventpoll *ep = RT_NULL;
    rt_err_t ret = -EINVAL;

    if (df->vnode->data)
    {
        ep = df->vnode->data;

        fdlist = ep->fdlist;
        while (fdlist->next != RT_NULL)
        {
            if (fdlist->next->fd == fd)
            {
                rt_mutex_take(&ep->lock, RT_WAITING_FOREVER);
                memcpy(&fdlist->next->epev.data, &event->data, sizeof(event->data));
                fdlist->next->revents = event->events;
                if (fdlist->next->wqn.wqueue)
                    rt_wqueue_remove(&fdlist->next->wqn);

                rt_mutex_release(&ep->lock);
                epoll_ctl_install(fdlist->next, ep);
                break;
            }

            fdlist = fdlist->next;
        }

        ret = 0;
    }

    return ret;
}

static int epoll_do_ctl(int epfd, int op, int fd, struct epoll_event *event)
{
    struct dfs_file *epdf;
    struct rt_eventpoll *ep;
    rt_err_t ret = 0;

    if (op & ~EFD_SHARED_EPOLL_TYPE)
    {
        rt_set_errno(EINVAL);
        return -1;
    }

    if ((epfd == fd) || (epfd < 0))
    {
        rt_set_errno(EINVAL);
        return -1;
    }

    if (!(op & EPOLL_CTL_DEL))
    {
        if (!(event->events & EPOLLEXCLUSIVE_BITS))
        {
            rt_set_errno(EINVAL);
            return -1;
        }
        event->events  |= EPOLLERR | EPOLLHUP;
    }

    if (!fd_get(fd))
    {
        rt_set_errno(EBADF);
        return -1;
    }

    epdf = fd_get(epfd);

    if (epdf->vnode->data)
    {
        ep = epdf->vnode->data;

        switch (op)
        {
        case EPOLL_CTL_ADD:
            ret = epoll_ctl_add(epdf, fd, event);
            break;
        case EPOLL_CTL_DEL:
            ret = epoll_ctl_del(epdf, fd);
            break;
        case EPOLL_CTL_MOD:
            ret = epoll_ctl_mod(epdf, fd, event);
            break;
        default:
            rt_set_errno(EINVAL);
            break;
        }

        if (ret < 0)
        {
            rt_set_errno(-ret);
            ret = -1;
        }
        else
        {
            ep->polling_thread = rt_thread_self();
        }
    }

    return ret;
}

static int epoll_wait_timeout(struct rt_eventpoll *ep, int msec)
{
    rt_int32_t timeout;
    struct rt_thread *thread;
    rt_base_t level;
    int ret = 0;

    thread = ep->polling_thread;

    timeout = rt_tick_from_millisecond(msec);

    level = rt_spin_lock_irqsave(&ep->spinlock);

    if (timeout != 0 && !ep->tirggered)
    {
        if (rt_thread_suspend_with_flag(thread, RT_KILLABLE) == RT_EOK)
        {
            if (timeout > 0)
            {
                rt_timer_control(&(thread->thread_timer),
                        RT_TIMER_CTRL_SET_TIME,
                        &timeout);
                rt_timer_start(&(thread->thread_timer));
            }

            rt_spin_unlock_irqrestore(&ep->spinlock, level);

            rt_schedule();

            level = rt_spin_lock_irqsave(&ep->spinlock);
        }
    }

    ret = !ep->tirggered;
    rt_spin_unlock_irqrestore(&ep->spinlock, level);

    return ret;
}

static int epoll_get_event(struct rt_fd_list *fl, rt_pollreq_t *req)
{
    struct dfs_file *df;
    int mask = 0;
    int fd = 0;

    fd = fl->fd;
    if (fd >= 0)
    {
        df = fd_get(fd);
        if (df)
        {
            if (df->vnode->fops->poll)
            {
                req->_key = fl->revents | POLLERR | POLLHUP;
                mask = df->vnode->fops->poll(df, req);
                if (mask < 0)
                    return mask;
            }

            mask &= fl->revents | EPOLLOUT | POLLERR;
        }
    }

    return mask;
}

static int epoll_do(struct rt_eventpoll *ep, struct epoll_event *events, int maxevents, int timeout)
{
    struct rt_fd_list *rdlist;
    rt_slist_t *node = RT_NULL;
    int event_num = 0;
    int istimeout = 0;
    int isn_add = 0;
    int isfree = 0;
    int mask = 0;
    rt_base_t level;

    while (1)
    {
        rt_mutex_take(&ep->lock, RT_WAITING_FOREVER);
        level = rt_spin_lock_irqsave(&ep->spinlock);
        if (ep->eventpoll_num > 0)
        {
            rt_slist_for_each(node,&ep->rdl_head)
            {
                rdlist = rt_slist_entry(node, struct rt_fd_list, rdl_node);
                ep->eventpoll_num --;
                rt_slist_remove(&ep->rdl_head, &rdlist->rdl_node);
                rdlist->is_rdl_node = RT_FALSE;

                rt_spin_unlock_irqrestore(&ep->spinlock, level);

                isfree = 0;
                isn_add = 0;
                if (event_num < maxevents)
                {
                    if (rdlist->wqn.wqueue)
                    {
                        rt_wqueue_remove(&rdlist->wqn);
                    }

                    mask = epoll_get_event(rdlist, &rdlist->req);

                    if (mask & rdlist->revents)
                    {
                        rdlist->epev.events = mask & rdlist->revents;
                    }
                    else
                    {
                        isfree = 1;
                        isn_add = 1;
                    }

                    if (rdlist->revents & EPOLLONESHOT)
                    {
                        rdlist->revents = 0;
                        isfree = 1;
                        if (rdlist->wqn.wqueue)
                            rt_wqueue_remove(&rdlist->wqn);
                    }
                    else
                    {
                        if (rdlist->revents & EPOLLET)
                        {
                            isfree = 1;
                        }
                        else
                        {
                            level = rt_spin_lock_irqsave(&ep->spinlock);
                            if (rdlist->exclusive != 1)
                            {
                                rdlist->exclusive = 1;
                            }
                            rt_spin_unlock_irqrestore(&ep->spinlock, level);
                        }
                    }

                    if (!isn_add)
                    {
                        memcpy(&events[event_num], &rdlist->epev, sizeof(rdlist->epev));
                        event_num ++;
                    }

                    if (!isfree)
                    {
                        if (rdlist->is_rdl_node == RT_FALSE)
                        {
                            level = rt_spin_lock_irqsave(&ep->spinlock);
                            ep->eventpoll_num ++;
                            rt_slist_append(&ep->rdl_head, &rdlist->rdl_node);
                            rdlist->is_rdl_node = RT_TRUE;
                            rt_spin_unlock_irqrestore(&ep->spinlock, level);
                        }
                        else
                        {
                            level = rt_spin_lock_irqsave(&ep->spinlock);
                            if (!rdlist->wqn.wqueue)
                            {
                                epoll_get_event(rdlist, &rdlist->req);
                            }
                            rt_spin_unlock_irqrestore(&ep->spinlock, level);
                        }
                    }
                }
                else
                {
                    level = rt_spin_lock_irqsave(&ep->spinlock);
                    break;
                }

                level = rt_spin_lock_irqsave(&ep->spinlock);
            }
        }

        rt_spin_unlock_irqrestore(&ep->spinlock, level);
        rt_mutex_release(&ep->lock);

        if (event_num || istimeout)
        {
            level = rt_spin_lock_irqsave(&ep->spinlock);
            ep->tirggered = 0;
            rt_spin_unlock_irqrestore(&ep->spinlock, level);
            if ((timeout >= 0) || (event_num > 0))
                break;
        }

        if (epoll_wait_timeout(ep, timeout))
        {
            istimeout = 1;
        }
    }

    return event_num;
}

static int epoll_do_wait(int epfd, struct epoll_event *events, int maxevents, int timeout, const sigset_t *ss)
{
    struct rt_eventpoll *ep;
    struct dfs_file *df;
    lwp_sigset_t old_sig, new_sig;
    rt_err_t ret = 0;

    if (ss)
    {
        memcpy(&new_sig, ss, sizeof(lwp_sigset_t));
        lwp_thread_signal_mask(rt_thread_self(), LWP_SIG_MASK_CMD_BLOCK, &new_sig, &old_sig);
    }

    if ((maxevents > 0) && (epfd >=0))
    {
        df = fd_get(epfd);
        if (df && df->vnode)
        {
            ep = (struct rt_eventpoll *)df->vnode->data;
            if (ep)
            {
                ret = epoll_do(ep, events, maxevents, timeout);
            }
        }
    }

    if (ss)
    {
        lwp_thread_signal_mask(rt_thread_self(), LWP_SIG_MASK_CMD_SET_MASK, &old_sig, RT_NULL);
    }

    if (ret < 0)
    {
        rt_set_errno(-ret);
        ret = -1;
    }

    return ret;
}

int epoll_create(int size)
{
    return epoll_do_create(size);
}

int epoll_ctl(int epfd, int op, int fd, struct epoll_event *event)
{
    return epoll_do_ctl(epfd, op, fd, event);
}

int epoll_wait(int epfd, struct epoll_event *events, int maxevents, int timeout)
{
    return epoll_do_wait(epfd, events, maxevents, timeout, RT_NULL);
}

int epoll_pwait(int epfd, struct epoll_event *events, int maxevents, int timeout, const sigset_t *ss)
{
    return epoll_do_wait(epfd, events, maxevents, timeout, ss);
}

int epoll_pwait2(int epfd, struct epoll_event *events, int maxevents, int timeout, const sigset_t *ss)
{
    return epoll_do_wait(epfd, events, maxevents, timeout, ss);
}
