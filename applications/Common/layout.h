/*
 * Copyrite (C) 2016, BeeLinker
 * 
 * 文件名称: layout.h
 * 文件说明：定义flash 分区
 *
 *
 * 版本信息：
 *
 */

#ifndef __LAYOUT_H__
#define __LAYOUT_H__

/********************************************
* Defines *
********************************************/
#define FLASH_PAGE_SIZE             ((uint32_t)0x800)

#define CFG_FLASH_ADDR              0x08000000
#define CFG_FLASH_SIZE              0x40000

#define CFG_BOOTLOADER_ADDR         CFG_FLASH_ADDR
#define CFG_BOOTLOADER_SIZE         0x2000

#define CFG_NVRAM_ADDR              (CFG_BOOTLOADER_ADDR + CFG_BOOTLOADER_SIZE)
#define CFG_NVRAM_SIZE              0x2000

#define CFG_IMG_ADDR                (CFG_NVRAM_ADDR + CFG_NVRAM_SIZE)
#define CFG_IMG_SIZE                0x1E000

#define CFG_BKP_ADDR                (CFG_IMG_ADDR + CFG_IMG_SIZE)
#define CFG_BKP_SIZE                0x1E000


/********************************************
* Typedefs *
********************************************/


/********************************************
* Globals * 
********************************************/


/********************************************
* Function defines *
********************************************/

#endif
