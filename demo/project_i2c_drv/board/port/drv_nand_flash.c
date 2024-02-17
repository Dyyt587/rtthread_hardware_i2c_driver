#include "drv_nand_flash.h"
#include <rtdevice.h>
/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-06-30     thread-liu   first version
 * 2021-06-21     THEWON       add MT29F4G08
 */

#include <rtthread.h>
#include <rtdevice.h>

#ifdef RT_USING_MTD_NAND

#define DRV_DEBUG
#define LOG_TAG     "drv.nand"
#include <drv_log.h>
#include "drv_nand_flash.h"
//#include <dfs_fs.h>
//#include <dfs_posix.h>

struct rthw_fmc
{
    rt_uint32_t id;
};
static struct rthw_fmc _device = {0};
static struct rt_mtd_nand_device nand_drv;
static NAND_HandleTypeDef NAND_Handler;

/* nand wait RB */
static rt_err_t rt_hw_nand_wait_rb()
{
    rt_uint32_t time = 0;
    int pin = 0;

    while (1)
    {
        pin = rt_pin_read(NAND_WAITRB_PIN);
        if (pin == 1)
        {
            break;
        }
        time++;
        if (time >= 0X1FFFFFFF)
        {
            return -RT_ETIMEOUT;
        }
    }
    return RT_EOK;
}

/* nand delay */
static void rt_hw_nand_delay(volatile rt_uint32_t i)
{
    while (i > 0)
    {
        i--;
    }
}

/* read nand flash status */
static rt_uint8_t rt_hw_nand_read_status(void)
{
    rt_uint8_t result = RT_EOK;

    NAND_CMD = NAND_STATUS;

    rt_hw_nand_delay(NAND_TWHR_DELAY);

    result = NAND_DATA;

    return result;
}

/* wait nand flash read */
static rt_err_t rt_hw_nand_wait_ready(void)
{
    rt_err_t result = RT_EOK;
    rt_uint32_t time = 0;

    while (1)
    {
        result = rt_hw_nand_read_status();

        if (result & NAND_READY)
        {
            break;
        }
        time++;
        if (time >= 0X1FFFFFFF)
        {
            return -RT_ETIMEOUT;
        }
    }

    return RT_EOK;
}

/* set nand mode */
static rt_err_t rt_hw_nand_set_mode(rt_uint8_t mode)
{
    NAND_CMD = NAND_FEATURE;
    NAND_ADDR = 0x01;
    NAND_DATA = mode;
    NAND_DATA = 0;
    NAND_DATA = 0;
    NAND_DATA = 0;

    if (rt_hw_nand_wait_ready() == RT_EOK)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

/* reset nand flash */
static rt_err_t rt_hw_nand_reset(void)
{
    NAND_CMD = NAND_RESET;

    if (rt_hw_nand_wait_ready() == RT_EOK)
    {
        return RT_EOK; /* success */
    }
    else
    {
        return -RT_ERROR;
    }
}

/* read nand flash id */
static rt_err_t _read_id(struct rt_mtd_nand_device *device)
{
    rt_uint32_t id;
    rt_uint8_t deviceid[5];

    NAND_CMD = NAND_READID; /* read id command */
    NAND_ADDR = 0x00;

    deviceid[0] = NAND_DATA; /* Byte 0 */
    deviceid[1] = NAND_DATA; /* Byte 1 */
    deviceid[2] = NAND_DATA; /* Byte 2 */
    deviceid[3] = NAND_DATA; /* Byte 3 */
    deviceid[4] = NAND_DATA; /* Byte 4 */

    id = ((uint32_t)deviceid[4]) << 24 | ((uint32_t)deviceid[3]) << 16 | ((uint32_t)deviceid[2]) << 8 | deviceid[1];

    LOG_D("nand id: 0x%08x", id);

    return id;
}

static rt_err_t mt29f4g08_erase_block(struct rt_mtd_nand_device *device,rt_uint32_t block)
{
    block <<= NAND_PPB_BW;

    NAND_CMD = NAND_ERASE_MODE;
    NAND_ADDR = (rt_uint8_t)block;
    NAND_ADDR = (rt_uint8_t)(block >> 8);
    NAND_ADDR = (rt_uint8_t)(block >> 16);
    NAND_CMD = NAND_ERASE_BLOCK;

    rt_hw_nand_wait_rb();

    return rt_hw_nand_read_status();
}

static rt_err_t mt29f4g08_erase_chip(struct rt_mtd_nand_device *device,rt_mtd_nand_t nand)
{
    int i;
    for (i = 0; i < nand->block_total; i++)
    {
        mt29f4g08_erase_block(device,i);
    }
    return RT_EOK;
}

static rt_err_t mt29f4g08_cmdfunc(rt_mtd_nand_t nand, int cmd, int page, int offset)
{
    int ret = 0;

    RT_ASSERT(nand != RT_NULL);

    switch (cmd)
    {
    case RT_NAND_CMD_PAGE_RD:
        NAND_CMD = NAND_READ_MODE;// ncmd(K9F1G_PGAEREAD1);

        NAND_ADDR = offset;
        NAND_ADDR = (offset >> 8);
        NAND_ADDR = (page);
        NAND_ADDR = (page >> 8);
        NAND_ADDR = (page >> 16);

        NAND_CMD = NAND_READ_PAGE;// (K9F1G_PAGEREAD2);
        rt_hw_nand_wait_rb();// nwait();
        NAND_CMD = NAND_READ_MODE;// ncmd(K9F1G_PGAEREAD1);
    break;
    case RT_NAND_CMD_PAGE_WR_BGN:
        NAND_CMD = NAND_PROG_MODE;// ncmd(K9F1G_PAGEPROGRAM1);

        NAND_ADDR = offset;// naddr(offset);
        NAND_ADDR = (offset >> 8);// naddr(offset >> 8);
        NAND_ADDR = (page);// naddr(page);
        NAND_ADDR = (page >> 8);// naddr(page >> 8);
        NAND_ADDR = (page >> 16);
    break;
    case RT_NAND_CMD_PAGE_WR_END:
        NAND_CMD = NAND_PROG_PAGE;// ncmd(K9F1G_PAGEPROGRAM2);
        rt_hw_nand_wait_rb();// nwait();
        ret = rt_hw_nand_read_status() & NAND_ERROR;
    break;
    case RT_NAND_CMD_BLK_ERASE:
        ret = mt29f4g08_erase_block(nand,page);
    break;
    case RT_NAND_CMD_ECC_EN:
        HAL_NAND_ECC_Enable(&NAND_Handler);// FSMC_NANDECCCmd(FSMC_Bank2_NAND, ENABLE);
    break;
    case RT_NAND_CMD_ECC_DIS:
        HAL_NAND_ECC_Disable(&NAND_Handler);// FSMC_NANDECCCmd(FSMC_Bank2_NAND,DISABLE);
    break;
    default:
    break;
    }

    return ret;
}

static rt_err_t mt29f4g08_read_buf(rt_mtd_nand_t nand, rt_uint8_t *buf, int len)
{
    RT_ASSERT(nand != RT_NULL);
    RT_ASSERT(buf != RT_NULL);

    while (len)
    {
        *buf = NAND_DATA;
        buf++;
        len--;
    }

    return 0;
}

static rt_err_t mt29f4g08_write_buf(rt_mtd_nand_t nand, const rt_uint8_t *buf, int len)
{
    RT_ASSERT(nand != RT_NULL);
    RT_ASSERT(buf != RT_NULL);

    while (len)
    {
        NAND_DATA = *buf;
        buf++;
        len--;
    }
    return 0;
}

static void stm32f4_calc_ecc(rt_mtd_nand_t nand, const rt_uint8_t *data, rt_uint8_t *ecc_code)
{
    rt_uint32_t e;
    RT_ASSERT(nand != RT_NULL);
    RT_ASSERT(data != RT_NULL);
    RT_ASSERT(ecc_code != RT_NULL);

    HAL_NAND_GetECC(&NAND_Handler, &e, 5);

    ecc_code[0] = e;
    ecc_code[1] = e >> 8;
    ecc_code[2] = e >> 16;
}

static int stm32f4_correct(rt_mtd_nand_t nand, rt_uint8_t *data, rt_uint8_t *ecc_read, rt_uint8_t *ecc_calc)
{
    rt_uint32_t diff0, diff1, diff2;
    rt_uint32_t bit, byte;

    RT_ASSERT(nand != RT_NULL);
    RT_ASSERT(data != RT_NULL);
    RT_ASSERT(ecc_read != RT_NULL);
    RT_ASSERT(ecc_calc != RT_NULL);

    diff0 = ecc_read[0] ^ ecc_calc[0];
    diff1 = ecc_read[1] ^ ecc_calc[1];
    diff2 = ecc_read[2] ^ ecc_calc[2];

    if ((diff0 | diff1 | diff2) == 0)
    {
        return 0;       /* ECC is ok */
    }

    /* sometimes people do not think about using the ECC, so check
     * to see if we have an 0xff,0xff,0xff read ECC and then ignore
     * the error, on the assumption that this is an un-eccd page.
     */
    if ((ecc_read[0] & ecc_read[1] & ecc_read[2]) == 0xff)
    {
        return 0;
    }

    /* Can we correct this ECC (ie, one row and column change).
     * Note, this is similar to the 256 error code on smartmedia */

    if (((diff0 ^ (diff0 >> 1)) & 0x55) == 0x55 &&
        ((diff1 ^ (diff1 >> 1)) & 0x55) == 0x55 &&
        ((diff2 ^ (diff2 >> 1)) & 0x55) == 0x55) {
        /* calculate the bit position of the error */

        bit  = ((diff2 >> 3) & 1) |
               ((diff2 >> 4) & 2) |
               ((diff2 >> 5) & 4);

        /* calculate the byte position of the error */

        byte = ((diff2 << 7) & 0x100) |
               ((diff1 << 0) & 0x80)  |
               ((diff1 << 1) & 0x40)  |
               ((diff1 << 2) & 0x20)  |
               ((diff1 << 3) & 0x10)  |
               ((diff0 >> 4) & 0x08)  |
               ((diff0 >> 3) & 0x04)  |
               ((diff0 >> 2) & 0x02)  |
               ((diff0 >> 1) & 0x01);

        data[byte] ^= (1 << bit);
        return 0;
    }

    /* if there is only one bit difference in the ECC, then
     * one of only a row or column parity has changed, which
     * means the error is most probably in the ECC itself */

    diff0 |= (diff1 << 8);
    diff0 |= (diff2 << 16);

    /* equal to "(diff0 & ~(1 << __ffs(diff0)))" */
    if ((diff0 & (diff0 - 1)) == 0)
    {
        return 1;
    }

    return -1;
}

//static const struct rt_mtd_oob_region _layout[2] =
//{
//    {2, 38}, //free
//    {40, 24} //ecc
//};

static const struct rt_mtd_nand_driver_ops _chip_ops =
{
    _read_id,
    mt29f4g08_read_buf,
    mt29f4g08_write_buf,
	    0,
	    mt29f4g08_erase_block,
    0,
    0
};

#ifdef MT29F4G08ABADA
static rt_err_t stm32_nand_init()
{
    FMC_NAND_PCC_TimingTypeDef ComSpaceTiming, AttSpaceTiming;

    NAND_Handler.Instance = FMC_NAND_DEVICE;
    NAND_Handler.Init.NandBank = FMC_NAND_BANK3;
    NAND_Handler.Init.Waitfeature = FMC_NAND_PCC_WAIT_FEATURE_DISABLE;
    NAND_Handler.Init.MemoryDataWidth = FMC_NAND_PCC_MEM_BUS_WIDTH_8;
    NAND_Handler.Init.EccComputation = FMC_NAND_ECC_DISABLE;
    NAND_Handler.Init.ECCPageSize = FMC_NAND_ECC_PAGE_SIZE_2048BYTE;
    NAND_Handler.Init.TCLRSetupTime = 0;
    NAND_Handler.Init.TARSetupTime = 1;

    ComSpaceTiming.SetupTime = 2;
    ComSpaceTiming.WaitSetupTime = 3;
    ComSpaceTiming.HoldSetupTime = 2;
    ComSpaceTiming.HiZSetupTime = 1;

    AttSpaceTiming.SetupTime = 2;
    AttSpaceTiming.WaitSetupTime = 3;
    AttSpaceTiming.HoldSetupTime = 2;
    AttSpaceTiming.HiZSetupTime = 1;

    HAL_NAND_Init(&NAND_Handler, &ComSpaceTiming, &AttSpaceTiming);

    rt_pin_mode(NAND_WAITRB_PIN, PIN_MODE_INPUT_PULLUP);

    rt_hw_nand_reset(); /* reset nand flash*/
    rt_thread_delay(100);

    /* read id */
    _device.id = _read_id();

    if (_device.id != NAND_DEV_ID)
    {
        LOG_E("nand id 0x%08x not support", _device.id);
        return -RT_ERROR; /* can't find nand flash */
    }

    rt_hw_nand_set_mode(4); /* set mode 4, high speed mode*/

    return RT_EOK;
}
#elif defined(MT29F8G08ABACA)
static void rt_hw_nand_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    if (IS_ENGINEERING_BOOT_MODE())
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FMC;
        PeriphClkInit.AdcClockSelection = RCC_FMCCLKSOURCE_ACLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Error_Handler();
        }
    }
    __HAL_RCC_FMC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* PD6 R/B */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* PG9 NCE */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* PD0,1,4,5,11,12,14,15 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 |
                          GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* PE7,8,9,10 */
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}
static rt_err_t stm32_nand_init()
{
    uint32_t tempreg = 0;

    rt_hw_nand_gpio_init();

    tempreg |= 0 << 1;          /* disable Wait feature enable bit */
    tempreg |= 0 << 4;          /* Data bus width 8*/
    tempreg |= 0 << 6;          /* disable ECC */
    tempreg |= 1 << 17;         /* ECC page 512 BYTE */
    tempreg |= 5 << 9;          /* set TCLR */
    tempreg |= 5 << 13;         /* set TAR */
    FMC_Bank3_R->PCR = tempreg; /* set nand control register */

    tempreg &= 0;
    tempreg |= 3 << 0;  /* set MEMSET  */
    tempreg |= 5 << 8;  /* set MEMWAIT */
    tempreg |= 2 << 16; /* set MEMHOLD */
    tempreg |= 3 << 24; /* set MEMHIZ  */
    FMC_Bank3_R->PMEM = tempreg;
    FMC_Bank3_R->PATT = 0;                     /* Attribute memory space timing registers */
    FMC_Bank3_R->PCR |= 1 << 2;                /* NAND Flash memory bank enable bit */
    FMC_Bank1_R->BTCR[0] |= (uint32_t)1 << 31; /* enable fmc */

    rt_hw_nand_reset(); /* reset nand flash*/
    rt_thread_delay(100);

    /* read id */
    _device.id = _read_id();

    if (_device.id != NAND_DEV_ID)
    {
        LOG_E("nand id 0x%08x not support", _device.id);
        return RT_ERROR; /* can't find nand flash */
    }

    rt_hw_nand_set_mode(4); /* set mode 4, high speed mode*/

    return RT_EOK;
}
#endif
int rt_hw_nand_init(void)
{
    rt_err_t result = RT_EOK;
     rt_mtd_nand_t nand_dev;
    nand_dev = RT_MTD_NAND_DEVICE(rt_malloc(sizeof(struct rt_mtd_nand_device)));
		if(nand_dev==0){
		    LOG_D("low memery!");
		}
    result = stm32_nand_init();
    if (result != RT_EOK)
    {
        LOG_D("nand flash init error!");
        return -RT_ERROR;
    }

#ifdef MT29F4G08ABADA
    nand_dev->page_size       = NAND_MAX_PAGE_SIZE;
    nand_dev->pages_per_block = 64;
    nand_dev->plane_num       = 2;
    nand_dev->oob_size        = 64;
    nand_dev->oob_free        = 64 - ((2048) * 3 / 256);
    nand_dev->block_start     = 0;
    nand_dev->block_end       = 4095;
#elif defined(MT29F8G08ABACA)
    nand_dev.total_size      = 1024 * 1024 * 1024;
    nand_dev.page_size       = NAND_MAX_PAGE_SIZE;
    nand_dev.pages_per_block = 64;
    nand_dev.plane_num       = 4;
    nand_dev.oob_size        = 64;
    nand_dev.oob_free        = 64 - ((2048) * 3 / 256);
    nand_dev.block_start     = 0;
    nand_dev.block_end       = 8191;
#endif
    nand_dev->block_total = nand_dev->block_end - nand_dev->block_start + 1;
//    nand_dev.total_size = nand_dev.block_total
//                        * nand_dev.pages_per_block
//                        * nand_dev.page_size;

    
    nand_dev->ops = &_chip_ops;

    //rt_mtd_nand_driver_init(nand_drv);

    result = rt_mtd_nand_register_device("mt29f", nand_dev);
    if (result != RT_EOK)
    {
        return -RT_ERROR;
    }

    LOG_D("nand flash init success, id: 0x%08x\n", _device.id);

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_nand_init);

#endif

