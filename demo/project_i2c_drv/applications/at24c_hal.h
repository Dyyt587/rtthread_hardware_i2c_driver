#ifndef AT24C_HAL_H__
#define AT24C_HAL_H__
/*****************************************
						本驱动文件仅适配HAL库版本
******************************************/
#include "main.h"	//链接HAL库

#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64	  8191
#define AT24C128	16383
#define AT24C256	32767  
//我使用的是AT24C02
#define EE_TYPE AT24C02

void AT24CXX_Init(void);

void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);

void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);

uint8_t AT24CXX_Check(void);

#endif

