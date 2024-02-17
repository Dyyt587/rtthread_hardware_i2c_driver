/*
 * Copyright (c) Dyyt587
 * SPDX-License-Identifier: MIT-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-02-17     Dyyt587   first version
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <rtthread.h>
#include <rthw.h>
#include <rtconfig.h>
#include "drv_i2c.h"
#include "drv_config.h"
#include "i2c_config.h"
#include <string.h>
#ifdef RT_USING_I2C
#if defined(BSP_USING_I2C1) || defined(BSP_USING_I2C2) || defined(BSP_USING_I2C3) || \
    defined(BSP_USING_I2C4) || defined(BSP_USING_I2C5) || defined(BSP_USING_I2C6)

#define DRV_DEBUG
#define LOG_TAG "drv.i2c"
#include <drv_log.h>
/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

#define I2C_TIMEOUT ((uint32_t)0x10000)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
// extern rt_err_t rt_hw_board_i2c_init(CM_I2C_TypeDef *I2Cx);

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
enum
{
#ifdef BSP_USING_I2C1
    I2C1_INDEX,
#endif
#ifdef BSP_USING_I2C2
    I2C2_INDEX,
#endif
#ifdef BSP_USING_I2C3
    I2C3_INDEX,
#endif

};

static struct stm32_i2c_config i2c_config[] =
    {
#ifdef BSP_USING_I2C1
        I2C1_BUS_CONFIG,
#endif
#ifdef BSP_USING_I2C2
        I2C2_BUS_CONFIG,
#endif
#ifdef BSP_USING_I2C3
        I2C3_BUS_CONFIG,
#endif

};

static void stm32_i2c_dma_configure(struct rt_i2c_bus_device *bus);
static struct stm32_i2c i2c_objs[sizeof(i2c_config) / sizeof(i2c_config[0])] = {0};

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

static rt_err_t stm32_i2c_init(struct stm32_i2c *i2c_drv)
{
    RT_ASSERT(i2c_drv != RT_NULL);
    // RT_ASSERT(cfg != RT_NULL);

    I2C_HandleTypeDef *i2c_handle = &i2c_drv->handle;
    rt_memset(i2c_handle, 0, sizeof(I2C_HandleTypeDef));
    struct stm32_i2c_config *cfg = i2c_drv->config;
    i2c_handle->Instance = cfg->Instance;
    i2c_handle->Init.Timing = cfg->timing;
    i2c_handle->Init.OwnAddress1 = 0;
    i2c_handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c_handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c_handle->Init.OwnAddress2 = 0;
    i2c_handle->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    i2c_handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c_handle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    //    if (HAL_I2C_DeInit(i2c_handle) != HAL_OK)
    //    {
    //        return RT_EFAULT;
    //    }
    // TODO:先确保总线被释放
    if (HAL_I2C_Init(i2c_handle) != HAL_OK)
    {
        return RT_EFAULT;
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(i2c_handle, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        return RT_EFAULT;
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(i2c_handle, 0) != HAL_OK)
    {
        return RT_EFAULT;
    }

    /* I2C2 DMA Init */
    if (i2c_drv->i2c_dma_flag & I2C_USING_RX_DMA_FLAG)
    {
        HAL_DMA_Init(&i2c_drv->dma.handle_rx);

        __HAL_LINKDMA(&i2c_drv->handle, hdmarx, i2c_drv->dma.handle_rx);

        /* NVIC configuration for DMA transfer complete interrupt */
        HAL_NVIC_SetPriority(i2c_drv->config->dma_rx->dma_irq, 0, 0);
        HAL_NVIC_EnableIRQ(i2c_drv->config->dma_rx->dma_irq);
    }

    if (i2c_drv->i2c_dma_flag & I2C_USING_TX_DMA_FLAG)
    {
        HAL_DMA_Init(&i2c_drv->dma.handle_tx);

        __HAL_LINKDMA(&i2c_drv->handle, hdmatx, i2c_drv->dma.handle_tx);

        /* NVIC configuration for DMA transfer complete interrupt */
        //        HAL_NVIC_SetPriority(i2c_drv->config->dma_tx->dma_irq, 1, 0);
        //        HAL_NVIC_EnableIRQ(i2c_drv->config->dma_tx->dma_irq);
    }

    if (i2c_drv->i2c_dma_flag & I2C_USING_TX_DMA_FLAG || i2c_drv->i2c_dma_flag & I2C_USING_RX_DMA_FLAG)
    {
        HAL_NVIC_SetPriority(i2c_drv->config->evirq_type, 2, 0);
        HAL_NVIC_EnableIRQ(i2c_drv->config->evirq_type);
    }

    LOG_D("%s init success", i2c_drv->config->name);
    return RT_EOK;
}

static rt_err_t stm32_i2c_configure(struct rt_i2c_bus_device *bus)
{
    int ret = -RT_ERROR;
    // stm_i2c_init_t i2c_init;
    float f32Error = 0.0F;
    rt_uint32_t I2cSrcClk;
    rt_uint32_t I2cClkDiv;
    rt_uint32_t I2cClkDivReg;

    RT_ASSERT(RT_NULL != bus);
    struct stm32_i2c *i2c_drv = rt_container_of(bus, struct stm32_i2c, i2c_bus);

    return stm32_i2c_init(i2c_drv);
}

static rt_ssize_t stm32_i2c_master_xfer(struct rt_i2c_bus_device *bus,
                                        struct rt_i2c_msg msgs[],
                                        rt_uint32_t num)
{
    rt_int32_t i, ret;
    struct rt_i2c_msg *msg = msgs;
    struct rt_i2c_msg *next_msg = 0;
    struct stm32_i2c *i2c_obj;
    uint32_t mode = 0;
	  uint8_t next_flag = 0;
    struct rt_completion *completion;
    if (num == 0)
    {
        return 0;
    }
    RT_ASSERT((msgs != RT_NULL) && (bus != RT_NULL));
    i2c_obj = rt_container_of(bus, struct stm32_i2c, i2c_bus);
    completion = &i2c_obj->completion;
    I2C_HandleTypeDef *handle = &i2c_obj->handle;
    rt_uint32_t timeout = i2c_obj->config->timeout;

    LOG_D("xfer start %d mags", num);
    for (i = 0; i < (num - 1); i++)
    {
        mode=0;
        msg = &msgs[i];
        LOG_D("xfer       msgs[%d] buf=0x%x len= 0x%x flags= 0x%x", i, msg->buf, msg->len, msg->flags);
        next_msg = &msgs[i + 1];
        next_flag = next_msg->flags;
        if (next_flag & RT_I2C_NO_START)
        {
            if ((next_flag & RT_I2C_RD) == (msg->flags & RT_I2C_RD))
            { /*相同的模式，可以使用no start*/
                mode = I2C_FIRST_AND_NEXT_FRAME;
            }
            else
            {
                // 不允许使用no start 换方向必须发送地址
                // 用户设置错误
                LOG_W("user set flags error msg[%d] flags RT_I2C_NO_START has canceled", i + 1);
                mode = I2C_LAST_FRAME_NO_STOP;
            }
        }
        else
        {
            mode = I2C_LAST_FRAME_NO_STOP;
        }

        if (msg->flags & RT_I2C_RD)
        {
            LOG_D("xfer  rec  msgs[%d] hal mode = %s", i, mode == I2C_FIRST_AND_NEXT_FRAME ? "I2C_FIRST_AND_NEXT_FRAME" : mode == I2C_LAST_FRAME_NO_STOP ? "I2C_FIRST_FRAME/I2C_LAST_FRAME_NO_STOP"
                                                                                                                      : mode == I2C_LAST_FRAME   ? "I2C_LAST_FRAME"
                                                                                                                                                         : "nuknown mode");
            ret = HAL_I2C_Master_Seq_Receive_IT(handle, msg->addr | RT_I2C_RD, msg->buf, msg->len, mode);
            if (ret != RT_EOK)
            {
                LOG_E("[%s:%d]I2C Read error(%d)!\n", __func__, __LINE__, ret);
                goto out;
            }
            if (rt_completion_wait(completion, timeout) != RT_EOK)
            {
                LOG_E("receive time out");
            }
        }
        else
        {
            LOG_D("xfer trans msgs[%d] hal mode = %s", i, mode == I2C_FIRST_AND_NEXT_FRAME ? "I2C_FIRST_AND_NEXT_FRAME" : mode == I2C_LAST_FRAME_NO_STOP ? "I2C_FIRST_FRAME/I2C_LAST_FRAME_NO_STOP"
                                                                                                                       : mode == I2C_LAST_FRAME   ? "I2C_LAST_FRAME"
                                                                                                                                                          : "nuknown mode");
            ret = HAL_I2C_Master_Seq_Transmit_IT(handle, msg->addr | RT_I2C_WR, msg->buf, msg->len, mode);
            if (ret != RT_EOK)
            {
                LOG_E("[%s:%d]I2C Write error(%d)!\n", __func__, __LINE__, ret);
                goto out;
            }
            if (rt_completion_wait(completion, timeout) != RT_EOK)
            {
                LOG_E("transmit time out");
            }
        }
        LOG_D("xfer  next msgs[%d] buf= 0x%x len= 0x%x flags = 0x%x\r\n", i+1, next_msg->buf, next_msg->len, next_msg->flags);

    }
    //最后的一个包
    msg = &msgs[i];
    if (msg->flags & RT_I2C_NO_STOP)
        mode = I2C_LAST_FRAME_NO_STOP;
    else
        mode = I2C_LAST_FRAME;
    LOG_D("xfer  last msgs[%d] buf= 0x%x len= 0x%x flags = 0x%x", i, msg->buf, msg->len, msg->flags);
    if (msg->flags & RT_I2C_RD)
    {
        LOG_D("xfer  rec  msgs[%d] hal mode=%s", i, mode == I2C_FIRST_AND_NEXT_FRAME ? "I2C_FIRST_AND_NEXT_FRAME" : mode == I2C_LAST_FRAME_NO_STOP ? "I2C_FIRST_FRAME/I2C_LAST_FRAME_NO_STOP"
                                                                                                                       : mode == I2C_LAST_FRAME   ? "I2C_LAST_FRAME"
                                                                                                                                                          : "nuknown mode");
        ret = HAL_I2C_Master_Seq_Receive_IT(handle, msg->addr | RT_I2C_RD, msg->buf, msg->len, mode);
        if (ret != RT_EOK)
        {
            LOG_E("[%s:%d]I2C Read error(%d)!\n", __func__, __LINE__, ret);
            goto out;
        }
        if (rt_completion_wait(completion, timeout) != RT_EOK)
        {
            LOG_E("receive time out");
        }
    }
    else
    {
        LOG_D("xfer trans msgs[%d] hal mode = %s", i, mode == I2C_FIRST_AND_NEXT_FRAME ? "I2C_FIRST_AND_NEXT_FRAME" : mode == I2C_LAST_FRAME ? "I2C_LAST_FRAME"
                                                                                                                        : mode == I2C_LAST_FRAME_NO_STOP   ? "I2C_FIRST_FRAME/I2C_LAST_FRAME_NO_STOP"
                                                                                                                                                           : "nuknown mode");
        ret = HAL_I2C_Master_Seq_Transmit_IT(handle, msg->addr | RT_I2C_WR, msg->buf, msg->len, mode);
        if (ret != RT_EOK)
        {
            LOG_E("[%s:%d]I2C Write error(%d)!\n", __func__, __LINE__, ret);
            goto out;
        }
        if (rt_completion_wait(completion, timeout) != RT_EOK)
        {
            LOG_E("transmit time out");
        }
    }
    ret = num;
    LOG_D("xfer  end  %d mags\r\n", num);
out:
    return ret;
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    struct stm32_i2c *i2c_drv = rt_container_of(hi2c, struct stm32_i2c, handle);
    rt_completion_done(&i2c_drv->completion);
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    struct stm32_i2c *i2c_drv = rt_container_of(hi2c, struct stm32_i2c, handle);
    rt_completion_done(&i2c_drv->completion);
}
static const struct rt_i2c_bus_device_ops stm32_i2c_ops =
    {
        .master_xfer = stm32_i2c_master_xfer,
        RT_NULL,
        RT_NULL};

int RT_hw_i2c_init(void)
{
    int ret = -RT_ERROR;
    rt_size_t obj_num = sizeof(i2c_objs) / sizeof(i2c_objs[0]);
    LOG_D("%s start\n", __func__);

    for (int i = 0; i < obj_num; i++)
    {
        i2c_objs[i].i2c_bus.ops = &stm32_i2c_ops;
        i2c_objs[i].config = &i2c_config[i];
        i2c_objs[i].i2c_bus.timeout = i2c_config[i].timeout;

        if (i2c_objs[i].i2c_dma_flag & I2C_USING_TX_DMA_FLAG)
        {
            i2c_objs[i].dma.handle_tx.Instance = DMA1_Stream3;
            i2c_objs[i].dma.handle_tx.Init.Request = DMA_REQUEST_I2C2_TX;
            i2c_objs[i].dma.handle_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
            i2c_objs[i].dma.handle_tx.Init.PeriphInc = DMA_PINC_DISABLE;
            i2c_objs[i].dma.handle_tx.Init.MemInc = DMA_MINC_ENABLE;
            i2c_objs[i].dma.handle_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            i2c_objs[i].dma.handle_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            i2c_objs[i].dma.handle_tx.Init.Mode = DMA_NORMAL;
            i2c_objs[i].dma.handle_tx.Init.Priority = DMA_PRIORITY_LOW;
            i2c_objs[i].dma.handle_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        }
        if ((i2c_objs[i].i2c_dma_flag & I2C_USING_RX_DMA_FLAG))
        {
            i2c_objs[i].dma.handle_rx.Instance = DMA1_Stream2;
            i2c_objs[i].dma.handle_rx.Init.Request = DMA_REQUEST_I2C2_RX;
            i2c_objs[i].dma.handle_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
            i2c_objs[i].dma.handle_rx.Init.PeriphInc = DMA_PINC_DISABLE;
            i2c_objs[i].dma.handle_rx.Init.MemInc = DMA_MINC_ENABLE;
            i2c_objs[i].dma.handle_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            i2c_objs[i].dma.handle_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            i2c_objs[i].dma.handle_rx.Init.Mode = DMA_NORMAL;
            i2c_objs[i].dma.handle_rx.Init.Priority = DMA_PRIORITY_LOW;
            i2c_objs[i].dma.handle_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        }
        stm32_i2c_configure(&i2c_objs[i].i2c_bus);
        rt_completion_init(&i2c_objs[i].completion);
        ret = rt_i2c_bus_device_register(&i2c_objs[i].i2c_bus, i2c_objs[i].config->name);
        RT_ASSERT(ret == RT_EOK);
        LOG_D("%s bus init done", i2c_config[i].name);
    }
    return ret;
}

INIT_CORE_EXPORT(RT_hw_i2c_init);

/**
 * @brief This function handles I2C2 event interrupt.
 */
void I2C2_EV_IRQHandler(void)
{
    /* USER CODE BEGIN I2C2_EV_IRQn 0 */
    /* enter interrupt */
    rt_interrupt_enter();
    /* USER CODE END I2C2_EV_IRQn 0 */
    HAL_I2C_EV_IRQHandler(&i2c_objs[I2C2_INDEX].handle);
    /* USER CODE BEGIN I2C2_EV_IRQn 1 */
    /* leave interrupt */
    rt_interrupt_leave();
    /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
 * @brief This function handles I2C2 error interrupt.
 */
void I2C2_ER_IRQHandler(void)
{
    /* USER CODE BEGIN I2C2_ER_IRQn 0 */
    /* enter interrupt */
    rt_interrupt_enter();
    /* USER CODE END I2C2_ER_IRQn 0 */
    HAL_I2C_ER_IRQHandler(&i2c_objs[I2C2_INDEX].handle);
    /* USER CODE BEGIN I2C2_ER_IRQn 1 */
    /* leave interrupt */
    rt_interrupt_leave();
    /* USER CODE END I2C2_ER_IRQn 1 */
}

#endif
#endif /* RT_USING_I2C */
