/*
 * 文件名: image.c
 *
 * 文件说明：
 *
 */


#include <image.h>
#include <string.h>
#include "endian.h"
#include "crc.h"
#include "layout.h"
#include "stm32f10x.h"

/********************************************
* Defines *
********************************************/

/********************************************
* Typedefs *
********************************************/


/********************************************
* Globals * 
********************************************/


/********************************************
* Function defines *
********************************************/
int imageCheck(const uint32_t img_addr)
{
    image_header_t hdr;    
    uint32_t checksum;
    
    memcpy(&hdr, (void *)img_addr, sizeof(image_header_t));
    
    if (ntohl(hdr.ih_magic) != IH_MAGIC)
        return -1;
    
    checksum = ntohl(hdr.ih_hcrc);
    hdr.ih_hcrc = 0;
    
    if (checksum != crc32(0, (unsigned char *)&hdr, sizeof(image_header_t)))
        return -1;
    
    checksum = crc32(0, (unsigned char *)(img_addr + sizeof(image_header_t)), ntohl(hdr.ih_size));
        
    if (checksum != ntohl(hdr.ih_dcrc))
        return -1;
    
    if (hdr.ih_type.imgid != GTWAY_IMGID)
        return -1;
    
    if (hdr.ih_type.len != STM32_VERLEN)
        return -1;
    
    return 0;
}

int imageUpgrade(uint32_t img_addr)
{
    image_header_t hdr;
    uint32_t src, dst;
    uint32_t NbPage = 0;
    uint32_t PageAddr = 0;
    int i;
    
    memcpy(&hdr, (void *)img_addr, sizeof(image_header_t));
    
    src = img_addr + sizeof(image_header_t);
    dst = CFG_IMG_ADDR;
    
    /* erase image area */
    FLASH_Unlock();
    
    /* Clear all FLASH flags */
    PageAddr = CFG_IMG_ADDR;
    NbPage = CFG_IMG_SIZE / FLASH_PAGE_SIZE;
    
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    for(i=0; i<NbPage; i++)
    {
        if (FLASH_COMPLETE != FLASH_ErasePage(PageAddr+i*FLASH_PAGE_SIZE))
            goto FAILED;
    }
    
    for (i = 0; i < ntohl(hdr.ih_size); i += 4)
    {
        uint32_t u32tmp;
        
        memcpy(&u32tmp, (void *)src, 4);

        if(FLASH_COMPLETE != FLASH_ProgramWord(dst, u32tmp))
            goto FAILED;
        
        src += 4;
        dst += 4;
    }
    
    if (crc32(0, (unsigned char *)CFG_IMG_ADDR, ntohl(hdr.ih_size)) != ntohl(hdr.ih_dcrc))
        goto FAILED;
    
    /* upgrade successfully, erase first page of bkp area */
    if (FLASH_COMPLETE != FLASH_ErasePage(CFG_BKP_ADDR))
        goto FAILED;
    
    FLASH_Lock();
    
    return 0;
    
FAILED:
    FLASH_Lock();
    
    return -1;
}
