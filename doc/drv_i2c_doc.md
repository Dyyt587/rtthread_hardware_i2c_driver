# 设计考虑——RTThread I2C设备

参考RTThread版本 当前最新版本master分支v5.0.2

i2c驱动框架包含三个文件分别是

![1708165617189](images/drv_i2c_doc/1708165617189.png)

用pin设备模拟i2c时序并且对接i2c设备框架的 ` i2c_bit-ops.c`

```
不是本次讨论重点
```

负责提供i2c框架通用接口函数和注册总线设备到rtt设备框架的i2c核心/框架 ` i2c_core.c`

![1708165918125](https://file+.vscode-resource.vscode-cdn.net/d%3A/project_i2c_drv/applications/images/drv_i2c_doc/1708165918125.png)

负责提供的i2c设备驱动 ` i2c_dev.c`

![1708166119750](images/drv_i2c_doc/1708166119750.png)

其中dev依赖于bus的传输函数，相当于套了一个壳，装饰一下，同时RTT没有明确的总线->设备概念，可能是为了简化或者其他原因，设备和总线使用同一个结构体，个人偏向是一个是对接device 向上层提供read write 一个面向驱动提供 `rt_i2c_transfer`之类的代码

## RTThread I2C驱动编写关注点

### control

```
static rt_err_t i2c_bus_device_control(rt_device_t dev,
                                      int         cmd,
                                      void       *args)
{
   rt_err_t ret;
   struct rt_i2c_priv_data *priv_data;
   struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)dev->user_data;

   RT_ASSERT(bus != RT_NULL);

   switch (cmd)
   {
   /* set 10-bit addr mode */
   case RT_I2C_DEV_CTRL_10BIT:
       bus->flags |= RT_I2C_ADDR_10BIT;
       break;
   case RT_I2C_DEV_CTRL_TIMEOUT:
       bus->timeout = *(rt_uint32_t *)args;
       break;
   case RT_I2C_DEV_CTRL_RW:
       priv_data = (struct rt_i2c_priv_data *)args;
       ret = rt_i2c_transfer(bus, priv_data->msgs, priv_data->number);
       if (ret < 0)
       {
           return -RT_EIO;
       }
       break;
   default:
       return rt_i2c_control(bus, cmd, args);
   }

   return RT_EOK;
}
```

注意到提供10bit 地址控制模式，读写和控制参数向下级传递尝试获取支持（说明驱动程序可以自己编写控制函数提供独特的控制）

### xfer

`ret = bus->ops->master_xfer(bus, msgs, num);`
这句code表明驱动需要实现同时与
`ret = bus->ops->i2c_bus_control(bus, cmd, args);`
分析表明我们要编写的驱动大概率就是ops里面定义的函数

### `struct rt_i2c_bus_device_ops`

```
struct rt_i2c_bus_device_ops
{
    rt_ssize_t (*master_xfer)(struct rt_i2c_bus_device *bus,
                             struct rt_i2c_msg msgs[],
                             rt_uint32_t num);
    rt_ssize_t (*slave_xfer)(struct rt_i2c_bus_device *bus,
                            struct rt_i2c_msg msgs[],
                            rt_uint32_t num);
    rt_err_t (*i2c_bus_control)(struct rt_i2c_bus_device *bus,
                                int cmd,
                                void *args);
};
```

我们看到ops要我们实现三个函数，但是slave_xfsr在框架里面并没有实现，推测对于i2c slave的开发进度缓慢
现在我们已经清楚要如何编写一个rtt i2c 设备驱动，我们再来看看stm32

## STM32 HAL库下的I2C

这里使用 STM32Cube_FW_H7_V1.11.1 包进行参考
board使用正点原子北极星stm32h750xhb6核心板验证(artpi同款芯片)
bsp工程继承自artpi
参考文档 STM32H7xx参考手册（V3中文版 和 STM32H750勘误手册

由于对于f1等芯片新版本的i2c有一个比较大的变化就是timing寄存器
对于timing的描述是这样的

![1708168556430](images/drv_i2c_doc/1708168556430.png)

![1708168614559](images/drv_i2c_doc/1708168614559.png)

![1708168636804](images/drv_i2c_doc/1708168636804.png)

可以看到目前感觉自动计算timing值还比较麻烦，那不如直接使用cubemx计算，官方也是如此推荐的

初始化就注意这个

### HAL库函数

```
HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                 uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                                uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                               uint32_t XferOptions);
```

我们关注seq函数，值得注意的是对于阻塞模式并没有提供这些函数，虽然可以通过mem系列函数模拟或者直接寄存器实现但是考虑复杂性和通用性故放弃阻塞驱动编写，直接使用中断模式，后续也能十分方便的扩展到dma模式

```
#define I2C_FIRST_FRAME                 ((uint32_t)I2C_SOFTEND_MODE)
#define I2C_FIRST_AND_NEXT_FRAME        ((uint32_t)(I2C_RELOAD_MODE | I2C_SOFTEND_MODE))
#define I2C_NEXT_FRAME                  ((uint32_t)(I2C_RELOAD_MODE | I2C_SOFTEND_MODE))
#define I2C_FIRST_AND_LAST_FRAME        ((uint32_t)I2C_AUTOEND_MODE)
#define I2C_LAST_FRAME                  ((uint32_t)I2C_AUTOEND_MODE)
#define I2C_LAST_FRAME_NO_STOP          ((uint32_t)I2C_SOFTEND_MODE)

```

```
#define  I2C_RELOAD_MODE                I2C_CR2_RELOAD
#define  I2C_AUTOEND_MODE               I2C_CR2_AUTOEND
#define  I2C_SOFTEND_MODE               (0x00000000U)
```

可以看到xferOption提供的宏和对应扩展，softend 表示不产生stop信号 reload 不产生addr传输

## 代码编写

先验证hal库驱动板载at24c02

```
    static uint8_t data[256] ={ 32};
    HAL_I2C_Master_Seq_Transmit_IT(hi2c2, 0xa0, &(data[0]), 1, I2C_FIRST_FRAME);
    if(rt_completion_wait(completion, 100)!=RT_EOK){
    		LOG_E("time out1");
    }
    //osDelay(1);
    // HAL_I2C_Master_Seq_Transmit_IT(hi2c2,0xa1,0xff,1,I2C_LAST_FRAME_NO_STOP);


    HAL_I2C_Master_Seq_Receive_IT(hi2c2, 0xa0, &(data[0]), 128, I2C_FIRST_AND_NEXT_FRAME);
    if (rt_completion_wait(completion, 100) != RT_EOK)
    {
        LOG_E("time out2");
    }

    HAL_I2C_Master_Seq_Receive_IT(hi2c2, 0xa0, &(data[128]), 128, I2C_LAST_FRAME);
    if (rt_completion_wait(completion, 100) != RT_EOK)
    {
        LOG_E("time out4");
    }
```

将数据打印出来显然是对的，没有截图

然后编写rtt驱动

参考模拟驱动的编写和硬件spi编写方式

代码过长，直接提供链接

完成基本驱动后进行测试

```
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

    ret = rt_i2c_transfer(&i2c_drv->i2c_bus, msg, sizeof(msg)/sizeof(struct rt_i2c_msg));

    LOG_HEX("data",16,data,sizeof(data));
```

打印数据进行查看

![1708170354979](images/drv_i2c_doc/1708170354979.png)

数据符合，实现初代版本，简单添加dma相关配置，但未验证
