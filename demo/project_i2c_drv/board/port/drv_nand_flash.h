#ifndef DRV_NAND_FLASH_H
#define DRV_NAND_FLASH_H

#include "board.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef RT_DEBUG_NAND
#define RT_DEBUG_NAND                   0
#endif


/* MTD control cmd types */
typedef enum
{
    RT_MTD_CTRL_GET_ID = 0,
    RT_MTD_CTRL_ERASE,
    RT_MTD_CTRL_CHECK,
    RT_MTD_CTRL_MARK,
    RT_MTD_CTRL_READ,
    RT_MTD_CTRL_WRITE,
} rt_mtd_cmd_t;

/* NAND Flash control cmd types */
typedef enum
{
    RT_NAND_CMD_PAGE_RD,           /* read data to chip's page buffer, do WaitBusy after this cmd in low driver */
    RT_NAND_CMD_PAGE_WR_BGN,       /* write data to chip's page buffer */
    RT_NAND_CMD_PAGE_WR_END,       /* do flash programe */
    RT_NAND_CMD_BLK_ERASE,         /* erase block */
    RT_NAND_CMD_ECC_EN,            /* enable gen HWECC */
    RT_NAND_CMD_ECC_DIS            /* disable gen HWECC */
} rt_nand_cmd_t;


/* define your nand flash here */
#define MT29F4G08ABADA
//#define MT29F8G08ABACA
//#define MT29F16G08ABABA

#ifdef MT29F4G08ABADA
#define NAND_DEV_ID                     0x569590DC      /* Byte 4 3 2 1 */
#define NAND_MAX_PAGE_SIZE              2048
#define NAND_PPB_BW                     6               // pow(2, NAND_PPB_BW) == NAND_MAX_PAGE_SIZE
#elif defined(MT29F8G08ABACA)
#define NAND_DEV_ID                     0x64A690D3
#define NAND_MAX_PAGE_SIZE              2048
#define NAND_PPB_BW                     6
#elif defined(MT29F16G08ABABA)
#define NAND_DEV_ID                     0x89260048
#define NAND_MAX_PAGE_SIZE              2048
#define NAND_PPB_BW                     6
#else
#error "Must define one NAND Flash device"
#endif

#define NAND_WAITRB_PIN                 GET_PIN(D, 6)

#define NAND_ECC_SECTOR_SIZE            512

#define NAND_TWHR_DELAY                 25
#define NAND_TBERS_DELAY                4

#ifdef STM32F429xx
#define NAND_REG_PCR                    FMC_Bank2_3->PCR3
#define NAND_REG_SR                     FMC_Bank2_3->SR3
#define NAND_REG_ECCR                   FMC_Bank2_3->ECCR3
#define NAND_REG_PMEM                   FMC_Bank2_3->PMEM3
#define NAND_REG_PATT                   FMC_Bank2_3->PATT3
#define NAND_REG_BTCR                   FMC_Bank1->BTCR
#elif defined(STMA32MP1xx)
#define NAND_REG_PCR                    FMC_Bank3_R->PCR
#define NAND_REG_SR                     FMC_Bank3_R->SR
#define NAND_REG_ECCR                   FMC_Bank3_R->HECCR
#define NAND_REG_PMEM                   FMC_Bank3_R->PMEM
#define NAND_REG_PATT                   FMC_Bank3_R->PATT
#define NAND_REG_BTCR                   FMC_Bank1_R->BTCR
#endif

#define NAND_BASE_ADDR                  ((rt_uint32_t)0x80000000)                       /* nand base address */
#define NAND_DATA                       (*(__IO rt_uint8_t *)NAND_BASE_ADDR)
#define NAND_CMD                        (*(__IO rt_uint8_t *)(NAND_BASE_ADDR | 1 << 16))     /*  command */
#define NAND_ADDR                       (*(__IO rt_uint8_t *)(NAND_BASE_ADDR | 1 << 17))     /*  data */

/* nand flash command */
#define NAND_READID                     0x90
#define NAND_FEATURE                    0xEF
#define NAND_RESET                      0xFF
#define NAND_STATUS                     0x70

#define NAND_READ_MODE                  0x00
#define NAND_READ_PAGE                  0x30
#define NAND_PROG_MODE                  0x80
#define NAND_PROG_PAGE                  0x10
#define NAND_ERASE_MODE                 0x60
#define NAND_ERASE_BLOCK                0xD0

#define NAND_MOVEOUT_MODE               0x00
#define NAND_MOVEOUT_DATA               0x35
#define NAND_MOVEIN_MODE                0x85
#define NAND_MOVEIN_DATA                0x10

/* nand flash status */
#define NAND_ECC1BITERR                 0x03 /* ECC 1bit err */
#define NAND_ECC2BITERR                 0x04 /* ECC 2bit or more err */

#ifdef __cplusplus
}
#endif
#endif
