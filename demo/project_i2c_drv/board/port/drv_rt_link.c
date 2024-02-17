/*
 *
 * RT-Thread Link Board Support Package
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-02-08     Dyyt587   first version
 */
#include<rtthread.h>
#include<rtdevice.h>
#include "board.h"
#include "drv_config.h"

#include "drv_rt_link.h"

/* 需要在传输端口中实现的功能 */
rt_err_t rt_link_port_init(void)
{
	return 0;
}
rt_err_t rt_link_port_deinit(void)
{
		return 0;
    
}
rt_err_t rt_link_port_reconnect(void)
{
		return 0;

}
rt_size_t rt_link_port_send(void *data, rt_size_t length)
{
		return 0;

}

