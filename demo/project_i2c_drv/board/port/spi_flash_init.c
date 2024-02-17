/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-07     wanghaijing  the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <spi_flash.h>
#include <drv_spi.h>
#include <fal.h>

static int rt_flash_init(void)
{
    extern rt_spi_flash_device_t rt_sfud_flash_probe(const char *spi_flash_dev_name, const char *spi_dev_name);
    extern int fal_init(void);

    rt_hw_spi_device_attach("spi2", "spi_w25q", GET_PIN(F, 10));

    /* initialize SPI Flash device */
    rt_sfud_flash_probe("norflash0", "spi_w25q");

    fal_init();
    /*register device*/
		rt_device_t fal_blk_device = fal_blk_device_create("filesys");
    if (fal_blk_device == RT_NULL)
    {
        rt_kprintf("fal_blk_device_create failed please check fal part name!\n");
    }
    else
    {
        rt_kprintf("fal_blk_device_create success\n");
    }
				rt_device_t fal_mtd_device1 = fal_mtd_nor_device_create("download");
    if (fal_mtd_device1 == RT_NULL)
    {
        rt_kprintf("fal_blk_device_create failed please check fal part name!\n");
    }
    else
    {
        rt_kprintf("fal_blk_device_create success\n");
    }
    return 0;
}
INIT_ENV_EXPORT(rt_flash_init);
