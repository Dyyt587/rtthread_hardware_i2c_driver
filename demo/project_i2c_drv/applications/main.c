/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-17     supperthomas first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <ulog.h>
#include <cmsis_os2.h>
// #include "dfs_fs.h"
#include <sfud.h>
#include "spi_flash_sfud.h"
#include "at24c_hal.h"
#include "drv_i2c.h"

/* defined the LED0 pin: PB1 */
#define LED0_PIN GET_PIN(B, 1)

#ifdef RT_USING_WIFI
extern void wlan_autoconnect_init(void);
#endif
extern NAND_HandleTypeDef hnand1;
/* FMC initialization function */
static void MX_FMC_Init(void)
{

    /* USER CODE BEGIN FMC_Init 0 */

    /* USER CODE END FMC_Init 0 */

    FMC_NAND_PCC_TimingTypeDef ComSpaceTiming = {0};
    FMC_NAND_PCC_TimingTypeDef AttSpaceTiming = {0};
    FMC_SDRAM_TimingTypeDef SdramTiming = {0};

    /* USER CODE BEGIN FMC_Init 1 */

    /* USER CODE END FMC_Init 1 */

    /** Perform the NAND1 memory initialization sequence
     */
    hnand1.Instance = FMC_NAND_DEVICE;
    /* hnand1.Init */
    hnand1.Init.NandBank = FMC_NAND_BANK3;
    hnand1.Init.Waitfeature = FMC_NAND_WAIT_FEATURE_ENABLE;
    hnand1.Init.MemoryDataWidth = FMC_NAND_MEM_BUS_WIDTH_8;
    hnand1.Init.EccComputation = FMC_NAND_ECC_ENABLE;
    hnand1.Init.ECCPageSize = FMC_NAND_ECC_PAGE_SIZE_512BYTE;
    hnand1.Init.TCLRSetupTime = 0;
    hnand1.Init.TARSetupTime = 0;
    /* hnand1.Config */
    hnand1.Config.PageSize = 2048;
    hnand1.Config.SpareAreaSize = 64;
    hnand1.Config.BlockSize = 64;
    hnand1.Config.BlockNbr = 2048;
    hnand1.Config.PlaneNbr = 2;
    hnand1.Config.PlaneSize = 4096;
    hnand1.Config.ExtraCommandEnable = DISABLE;
    /* ComSpaceTiming */
    ComSpaceTiming.SetupTime = 9;
    ComSpaceTiming.WaitSetupTime = 9;
    ComSpaceTiming.HoldSetupTime = 10;
    ComSpaceTiming.HiZSetupTime = 9;
    /* AttSpaceTiming */
    AttSpaceTiming.SetupTime = 9;
    AttSpaceTiming.WaitSetupTime = 9;
    AttSpaceTiming.HoldSetupTime = 10;
    AttSpaceTiming.HiZSetupTime = 9;

    if (HAL_NAND_Init(&hnand1, &ComSpaceTiming, &AttSpaceTiming) != HAL_OK)
    {
        Error_Handler();
    }
}

#define NAND_PAGE_SIZE 2048
uint8_t buf[NAND_PAGE_SIZE];

I2C_HandleTypeDef *hi2c2;

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2->Instance = I2C2;
    hi2c2->Init.Timing = 0x10707DBC;
    hi2c2->Init.OwnAddress1 = 0;
    hi2c2->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2->Init.OwnAddress2 = 0;
    hi2c2->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(hi2c2) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(hi2c2, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */
}

int main(void)
{
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    extern int lx_nor_simulator_test(void);

    // lx_nor_simulator_test();

    extern void lx_test(void);
    // lx_test();

    rt_device_t device = rt_device_find("i2c2");
    struct stm32_i2c *i2c_drv = rt_container_of(device, struct stm32_i2c, i2c_bus);
    struct rt_completion *completion = &i2c_drv->completion;
    hi2c2 = &(i2c_drv->handle);
    // MX_I2C2_Init();
//    LOG_D("测试程序开始\r\n");

			int res = AT24CXX_Check();
//    static uint8_t buff[16];
//    if (!res)
//    {
//        LOG_D("AT24CXX OK!\r\n");
//    }
//    else
//    {
//        LOG_D("AT24CXX ERROR!\r\n");
//    }
//    AT24CXX_Read(0, buff, 5);
//    AT24CXX_Write(0, (uint8_t *)"tttt", 5);
//    LOG_D("buff:%s\r\n", buff);

//    osDelay(1);

//    AT24CXX_Read(0, buff, 5);
//    AT24CXX_Write(0, (uint8_t *)"buff", 5);
//    LOG_D("buff:%s\r\n", buff);

//    LOG_D("测试程序结束\r\n");

    static uint8_t data[256] ={ 32};
//    HAL_I2C_Master_Seq_Transmit_IT(hi2c2, 0xa0, &(data[0]), 1, I2C_FIRST_FRAME);
//    if(rt_completion_wait(completion, 100)!=RT_EOK){
//    		LOG_E("time out1");
//    }
//    //osDelay(1);
//    // HAL_I2C_Master_Seq_Transmit_IT(hi2c2,0xa1,0xff,1,I2C_LAST_FRAME_NO_STOP);


//    HAL_I2C_Master_Seq_Receive_IT(hi2c2, 0xa0, &(data[0]), 128, I2C_FIRST_AND_NEXT_FRAME);
//    if (rt_completion_wait(completion, 100) != RT_EOK)
//    {
//        LOG_E("time out2");
//    }

//		HAL_I2C_Master_Seq_Receive_IT(hi2c2, 0xa0, &(data[128]), 128, I2C_LAST_FRAME);
//    if (rt_completion_wait(completion, 100) != RT_EOK)
//    {
//        LOG_E("time out4");
//    }
//		
//		rt_memset(data,0,256);
		//rt_i2c_master_recv(&i2c_drv->i2c_bus,0xa0,0,&(data[0]),256);

//    rt_ssize_t ret;
//    struct rt_i2c_msg msg[2];
//    uint8_t addr=0;
//    msg[0].addr   = 0xa0;
//    msg[0].flags  = RT_I2C_WR;
//    msg[0].len    = 1;
//    msg[0].buf    = &addr;

//    ret = rt_i2c_transfer(&i2c_drv->i2c_bus, msg, 1);
//		
//    msg[0].addr   = 0xa0;
//    msg[0].flags  = RT_I2C_RD;
//    msg[0].len    = 128;
//    msg[0].buf    = &data[0];
//		
//    msg[1].addr   = 0xa0;
//    msg[1].flags  = RT_I2C_NO_START|RT_I2C_RD;
//    msg[1].len    = 128;
//    msg[1].buf    = &data[128];

//    ret = rt_i2c_transfer(&i2c_drv->i2c_bus, msg, sizeof(msg)/sizeof(struct rt_i2c_msg));

    rt_ssize_t ret;
    struct rt_i2c_msg msg[3];
    uint8_t addr=0;
    msg[0].addr   = 0xa0;
    msg[0].flags  = RT_I2C_WR;
    msg[0].len    = 1;
    msg[0].buf    = &addr;
		
    msg[1].addr   = 0xa0;
    msg[1].flags  = RT_I2C_RD;
    msg[1].len    = 128;
    msg[1].buf    = &data[0];

    msg[2].addr   = 0xa0;
    msg[2].flags  = RT_I2C_NO_START|RT_I2C_RD;
    msg[2].len    = 128;
    msg[2].buf    = &data[128];

    ret = rt_i2c_transfer(&i2c_drv->i2c_bus, msg, 3);
	LOG_D("ret=%d",ret);
    LOG_HEX("data",16,data,sizeof(data));
    while (1)
    {
        // rt_device_write(dev, 0, buf, rt_strlen(buf));

        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
        // LOG_E("test 00");
    }
}

#include "stm32h7xx.h"
static int vtor_config(void)
{
    /* Vector Table Relocation in Internal QSPI_FLASH */
    SCB->VTOR = QSPI_BASE;
    return 0;
}
INIT_BOARD_EXPORT(vtor_config);
