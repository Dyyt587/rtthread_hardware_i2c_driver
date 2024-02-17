/*
 * @Author: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @Date: 2024-02-16 16:35:08
 * @LastEditors: Dyyt587 67887002+Dyyt587@users.noreply.github.com
 * @LastEditTime: 2024-02-17 00:42:39
 * @FilePath: \project_i2c_drv\applications\at24c_hal.c
 * @Description: ����Ĭ������,������`customMade`, ��koroFileHeader�鿴���� ��������: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <ulog.h>
//#include "dfs_fs.h"
#include <sfud.h>
#include "spi_flash_sfud.h"
#include "at24c_hal.h"
//��ʼ��IIC�ӿ�
extern I2C_HandleTypeDef* hi2c2;

#define AT24CXX_HANDLE	(hi2c2)	//IIC�ӿ�
#define AT24C_DEV_ADDR  (0XA0) //�豸��ַ

void AT24CXX_Init(void)
{
	//IIC_Init();//IIC��ʼ��
	AT24CXX_Check();
}

/*****************************************
��������void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
������WriteAddr :Ҫд�����ݵĵ�ַ  pBuffer��Ҫд������ݵ��׵�ַ NumToWrite��Ҫд�����ݵĳ���
������������ָ����ַ��ʼд�����ֽ�����
����ֵ����
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
��������AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
������ ReadAddr��Ҫ��ȡ���ݵĵ�ַ pBuffer�����������׵�ַ NumToRead:���ݳ���
������������ָ����ַ��ʼ��ȡ������ֽ�����
����ֵ����
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
��������uint8_t AT24CXX_Check(void)
��������
�������������AT24CXX�Ƿ���������������24XX�����һ����ַ(255)���洢��־��.���������24Cϵ��,�����ַҪ�޸�
����ֵ�����ɹ�����0 ʧ�ܷ���1
*****************************************/
uint8_t AT24CXX_Check(void)
{
	uint8_t temp=0;
	uint8_t data = 0x3a;
	AT24CXX_Read(EE_TYPE,&temp,1);//����ÿ�ο�����дAT24CXX			   
	if(temp != 0x3a)//�ų���һ�γ�ʼ�������
	{
		LOG_D("iehi");
		AT24CXX_Write(EE_TYPE,&data,1);
	    AT24CXX_Read(EE_TYPE,&temp,1);;	  
		if(temp != 0x3a)
			return 1;
	}
	return 0;											  
}

