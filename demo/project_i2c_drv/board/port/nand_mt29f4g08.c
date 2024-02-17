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
#include "nand_mt29f4g08.h"
//#include <dfs_fs.h>
//#include <dfs_posix.h>
//#include "yaffsfs.h"

// struct rthw_fmc
// {
//     rt_uint32_t id;
// };
// static struct rthw_fmc _device = {0};
static rt_mtd_nand_t nand_drv;
static NAND_HandleTypeDef NAND_Handler;

// /* nand wait RB */
// static rt_err_t rt_hw_nand_wait_rb()
// {
//     rt_uint32_t time = 0;
//     int pin = 0;
//     while (1)
//     {
//         pin = rt_pin_read(NAND_WAITRB_PIN);
//         if (pin == 1)
//         {
//             break;
//         }
//         time++;
//         if (time >= 0X1FFFFFFF)
//         {
//             return -RT_ETIMEOUT;
//         }
//     }
//     return RT_EOK;
// }

/* nand delay */
static void rt_hw_nand_delay(volatile rt_uint32_t i)
{
    while (i > 0)
    {
        i--;
    }
}

/* read nand flash status */
static rt_uint32_t rt_hw_nand_read_status(void)
{
    return HAL_NAND_Read_Status(&NAND_Handler);
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
    return HAL_NAND_Reset(&NAND_Handler);
}

/* read nand flash id */
static rt_uint32_t _read_id()
{
	NAND_IDTypeDef id;
    if(HAL_NAND_Read_ID(&NAND_Handler,&id)!=HAL_OK){
        LOG_D("nand read id error");
        return -RT_EBUSY;
    }
    LOG_D("nand id: 0x%08x", *((unsigned int *)&id));
    return *((unsigned int *)&id);
}

static rt_err_t mt29f4g08_erase_block(rt_uint32_t block)
{
    NAND_AddressTypeDef paddress;
    paddress.Block=block;
    paddress.Page=0;
    paddress.Plane=0;
    if(HAL_NAND_Erase_Block(&NAND_Handler,&paddress)!=HAL_OK)
    {
        return -RT_EBUSY;
    }

    return RT_EOK;
}

// static rt_err_t mt29f4g08_erase_chip(rt_nand_driver_t nand)
// {
//     int i;
//     for (i = 0; i < nand->parent.block_total; i++)
//     {
//         mt29f4g08_erase_block(i);
//     }
//     return RT_EOK;
// }
// static rt_err_t mt29f4g08_cmdfunc(rt_nand_driver_t nand, int cmd, int page, int offset)
// {
//     int ret = 0;
//
//     RT_ASSERT(nand != RT_NULL);
//
//     switch (cmd)
//     {
//     case RT_NAND_CMD_PAGE_RD:
//         NAND_CMD = NAND_READ_MODE;// ncmd(K9F1G_PGAEREAD1);
//
//         NAND_ADDR = offset;
//         NAND_ADDR = (offset >> 8);
//         NAND_ADDR = (page);
//         NAND_ADDR = (page >> 8);
//         NAND_ADDR = (page >> 16);
//
//         NAND_CMD = NAND_READ_PAGE;// (K9F1G_PAGEREAD2);
//         rt_hw_nand_wait_rb();// nwait();
//         NAND_CMD = NAND_READ_MODE;// ncmd(K9F1G_PGAEREAD1);
//     break;
//     case RT_NAND_CMD_PAGE_WR_BGN:
//         NAND_CMD = NAND_PROG_MODE;// ncmd(K9F1G_PAGEPROGRAM1);
//
//         NAND_ADDR = offset;// naddr(offset);
//         NAND_ADDR = (offset >> 8);// naddr(offset >> 8);
//         NAND_ADDR = (page);// naddr(page);
//         NAND_ADDR = (page >> 8);// naddr(page >> 8);
//         NAND_ADDR = (page >> 16);
//     break;
//     case RT_NAND_CMD_PAGE_WR_END:
//         NAND_CMD = NAND_PROG_PAGE;// ncmd(K9F1G_PAGEPROGRAM2);
//         rt_hw_nand_wait_rb();// nwait();
//         ret = rt_hw_nand_read_status() & NAND_ERROR;
//     break;
//     case RT_NAND_CMD_BLK_ERASE:
//         ret = mt29f4g08_erase_block(page);
//     break;
//     case RT_NAND_CMD_ECC_EN:
//         HAL_NAND_ECC_Enable(&NAND_Handler);// FSMC_NANDECCCmd(FSMC_Bank2_NAND, ENABLE);
//     break;
//     case RT_NAND_CMD_ECC_DIS:
//         HAL_NAND_ECC_Disable(&NAND_Handler);// FSMC_NANDECCCmd(FSMC_Bank2_NAND,DISABLE);
//     break;
//     default:
//     break;
//     }
//
//     return ret;
// }

static rt_err_t mt29f4g08_read_page(struct rt_mtd_nand_device *device,
                                    rt_off_t page,
                                    rt_uint8_t *data, rt_uint32_t data_len,
                                    rt_uint8_t *spare, rt_uint32_t spare_len)

{
    RT_ASSERT(device != RT_NULL);
    NAND_AddressTypeDef address;
    address.Page=page% (device->pages_per_block);
    address.Block = page/ (device->pages_per_block);
    address.Plane=0;
    HAL_NAND_Read_Page_8b(&NAND_Handler,&address,data,data_len);

    return 0;
}

// static rt_err_t mt29f4g08_write_buf(rt_nand_driver_t nand, const rt_uint8_t *buf, int len)
// {
//     RT_ASSERT(nand != RT_NULL);
//     RT_ASSERT(buf != RT_NULL);
//
//     while (len)
//     {
//         NAND_DATA = *buf;
//         buf++;
//         len--;
//     }
//     return 0;
// }

// static void stm32f4_calc_ecc(rt_nand_driver_t nand, const rt_uint8_t *data, rt_uint8_t *ecc_code)
// {
//     rt_uint32_t e;
//     RT_ASSERT(nand != RT_NULL);
//     RT_ASSERT(data != RT_NULL);
//     RT_ASSERT(ecc_code != RT_NULL);
//
//     HAL_NAND_GetECC(&NAND_Handler, &e, 5);
//
//     ecc_code[0] = e;
//     ecc_code[1] = e >> 8;
//     ecc_code[2] = e >> 16;
// }

// static int stm32f4_correct(rt_nand_driver_t nand, rt_uint8_t *data, rt_uint8_t *ecc_read, rt_uint8_t *ecc_calc)
// {
//     rt_uint32_t diff0, diff1, diff2;
//     rt_uint32_t bit, byte;

//     RT_ASSERT(nand != RT_NULL);
//     RT_ASSERT(data != RT_NULL);
//     RT_ASSERT(ecc_read != RT_NULL);
//     RT_ASSERT(ecc_calc != RT_NULL);

//     diff0 = ecc_read[0] ^ ecc_calc[0];
//     diff1 = ecc_read[1] ^ ecc_calc[1];
//     diff2 = ecc_read[2] ^ ecc_calc[2];

//     if ((diff0 | diff1 | diff2) == 0)
//     {
//         return 0;       /* ECC is ok */
//     }

//     /* sometimes people do not think about using the ECC, so check
//      * to see if we have an 0xff,0xff,0xff read ECC and then ignore
//      * the error, on the assumption that this is an un-eccd page.
//      */
//     if ((ecc_read[0] & ecc_read[1] & ecc_read[2]) == 0xff)
//     {
//         return 0;
//     }

//     /* Can we correct this ECC (ie, one row and column change).
//      * Note, this is similar to the 256 error code on smartmedia */

//     if (((diff0 ^ (diff0 >> 1)) & 0x55) == 0x55 &&
//         ((diff1 ^ (diff1 >> 1)) & 0x55) == 0x55 &&
//         ((diff2 ^ (diff2 >> 1)) & 0x55) == 0x55) {
//         /* calculate the bit position of the error */

//         bit  = ((diff2 >> 3) & 1) |
//                ((diff2 >> 4) & 2) |
//                ((diff2 >> 5) & 4);

//         /* calculate the byte position of the error */

//         byte = ((diff2 << 7) & 0x100) |
//                ((diff1 << 0) & 0x80)  |
//                ((diff1 << 1) & 0x40)  |
//                ((diff1 << 2) & 0x20)  |
//                ((diff1 << 3) & 0x10)  |
//                ((diff0 >> 4) & 0x08)  |
//                ((diff0 >> 3) & 0x04)  |
//                ((diff0 >> 2) & 0x02)  |
//                ((diff0 >> 1) & 0x01);

//         data[byte] ^= (1 << bit);
//         return 0;
//     }

//     /* if there is only one bit difference in the ECC, then
//      * one of only a row or column parity has changed, which
//      * means the error is most probably in the ECC itself */

//     diff0 |= (diff1 << 8);
//     diff0 |= (diff2 << 16);

//     /* equal to "(diff0 & ~(1 << __ffs(diff0)))" */
//     if ((diff0 & (diff0 - 1)) == 0)
//     {
//         return 1;
//     }

//     return -1;
// }


//#include "lx_stm32_nand_custom_driver.h"
//LX_NAND_FLASH nand_flash;
//int rt_hw_nand_init(void)
//{
//   
//// //    rt_mtd_nand_driver_init(nand_drv);
//// 		rt_size_t result;
////     //result = rt_mtd_nand_register_device("mt29f", &nand_drv->parent);
////     if (result != RT_EOK)
////     {
////         return -RT_ERROR;
////     }

////    // LOG_D("nand flash init success, id: 0x%08x\n", _device.id);
//       NAND_IDTypeDef pNAND_ID;
//    pNAND_ID.Maker_Id = 0x00;
//    pNAND_ID.Device_Id = 0x00;
//    pNAND_ID.Third_Id = 0x00;
//    pNAND_ID.Fourth_Id = 0x00;

//// if(_lx_nand_flash_initialize_driver( &nand_flash)!=LX_SUCCESS)
//// {
////	   LOG_D("nand flash init error");
//////    HAL_NAND_Read_ID(NAND_INSTANCE, &pNAND_ID);
//////    //日志输出id号
//////    LOG_D("maker id: 0x%02x",pNAND_ID.Maker_Id);
//////    LOG_D("device id: 0x%02x",pNAND_ID.Device_Id);
//////    LOG_D("third id: 0x%02x",pNAND_ID.Third_Id);
//////    LOG_D("fourth id: 0x%02x",pNAND_ID.Fourth_Id);
//////    
//////    
////    
////    return RT_EFULL;
//// }

//    /* Read the NAND memory ID */
////    HAL_NAND_Read_ID(NAND_INSTANCE, &pNAND_ID);

////    //日志输出id号
//    LOG_D("nand flash init success");
////    LOG_D("maker id: 0x%02x",pNAND_ID.Maker_Id);
////    LOG_D("device id: 0x%02x",pNAND_ID.Device_Id);
////    LOG_D("third id: 0x%02x",pNAND_ID.Third_Id);
////    LOG_D("fourth id: 0x%02x",pNAND_ID.Fourth_Id);
//// 

// //LOG_D("nand flash init success, maker id: 0x%02x device id: 0x%02x ",pNAND_ID.Maker_Id);
//    
//    return RT_EOK;
//}
//INIT_APP_EXPORT(rt_hw_nand_init);

#endif
