/*
 * @Author: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @Date: 2024-02-16 16:35:08
 * @LastEditors: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @LastEditTime: 2024-02-17 00:42:39
 * @FilePath: \project_i2c_drv\applications\at24c_hal.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <ulog.h>
//#include "dfs_fs.h"
#include <sfud.h>
#include "spi_flash_sfud.h"
#include "at24c_hal.h"
//初始化IIC接口
extern I2C_HandleTypeDef* hi2c2;

#define AT24CXX_HANDLE	(hi2c2)	//IIC接口
#define AT24C_DEV_ADDR  (0XA0) //设备地址

void AT24CXX_Init(void)
{
	//IIC_Init();//IIC初始化
	AT24CXX_Check();
}

/*****************************************
函数名：void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
参数：WriteAddr :要写入数据的地址  pBuffer：要写入的数据的首地址 NumToWrite：要写入数据的长度
功能描述：从指定地址开始写入多个字节数据
返回值：无
*****************************************/
void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
{
	if(EE_TYPE < AT24C16){
		uint32_t state;
		state = HAL_I2C_Mem_Write_IT(AT24CXX_HANDLE,AT24C_DEV_ADDR,WriteAddr,I2C_MEMADD_SIZE_8BIT,pBuffer,NumToWrite);
 
		if(state!=HAL_OK)
			{
				LOG_D("at24 write error(%d)",state);
		}
	}else
		HAL_I2C_Mem_Write(AT24CXX_HANDLE,AT24C_DEV_ADDR,WriteAddr,I2C_MEMADD_SIZE_16BIT,pBuffer,NumToWrite,HAL_MAX_DELAY);

}
/*****************************************
函数名：AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
参数： ReadAddr：要读取数据的地址 pBuffer：回填数据首地址 NumToRead:数据长度
功能描述：从指定地址开始读取多个个字节数据
返回值：无
*****************************************/
void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
{
	if(EE_TYPE < AT24C16){
		uint32_t state;
		state  = HAL_I2C_Mem_Read_IT(AT24CXX_HANDLE,AT24C_DEV_ADDR,ReadAddr,I2C_MEMADD_SIZE_8BIT,pBuffer,NumToRead);
		if(state!=HAL_OK)
		{
				LOG_D("at24 read error(%d)",state);
		}
	}else
		HAL_I2C_Mem_Read(AT24CXX_HANDLE,AT24C_DEV_ADDR,ReadAddr,I2C_MEMADD_SIZE_16BIT,pBuffer,NumToRead,HAL_MAX_DELAY);
	        rt_thread_mdelay(100);
} 
/*****************************************
函数名：uint8_t AT24CXX_Check(void)
参数：无
功能描述：检查AT24CXX是否正常，这里用了24XX的最后一个地址(255)来存储标志字.如果用其他24C系列,这个地址要修改
返回值：检测成功返回0 失败返回1
*****************************************/
uint8_t AT24CXX_Check(void)
{
	uint8_t temp=0;
	uint8_t data = 0x3a;
	AT24CXX_Read(EE_TYPE,&temp,1);//避免每次开机都写AT24CXX			   
	if(temp != 0x3a)//排除第一次初始化的情况
	{
		LOG_D("iehi");
		AT24CXX_Write(EE_TYPE,&data,1);
	    AT24CXX_Read(EE_TYPE,&temp,1);;	  
		if(temp != 0x3a)
			return 1;
	}
	return 0;											  
}

