/*
 * nvram.c
 * Description: nvram managerment for system
 */

#include <string.h>
#include "stm32f10x.h"
#include "crc.h"
#include "nvram.h"

static const block_info_t fb[] =
{
    /* block name (must < 8)       offset      size */
    {"swap",                        0x0,        0x800},
    {"user",                        0x800,      0x800},
    {"res0",                        0x1000,      0x800},
    {"sys",                         0x1800,      0x800},
};

#define FB_SIZE         (sizeof(fb)/sizeof(block_info_t))

/* Init nvram */
int Nvram_Init(void)
{    
    /* check swap area */
    if (Nvram_Block_Check("swap") == 0)
    {
        int i;
        char *name;
        uint8_t *ptr;
        uint16_t dstblkOffset;
        uint32_t src, dst;
        
        ptr = (uint8_t *)(CFG_NVRAM_ADDR + fb[0].offset);
        
        name = (char *)(ptr + sizeof(uint32_t));
        
        Nvram_Get_BlkOffset_By_Name(name, &dstblkOffset);
        
        for (i = 1; i < FB_SIZE; i ++)
        {
            if (dstblkOffset == fb[i].offset)
                break;
        }
        
        if (i != FB_SIZE)
        {
            src = CFG_NVRAM_ADDR;
            dst = CFG_NVRAM_ADDR + dstblkOffset;
            
            //erase dst block
            if(Flash_Area_Erase(dst,BLK_MAX_SIZE) == -1)
                return -1;
            
            //copy word from swap to dst block
            if(Flash_Area_Prog(dst,(uint8_t*)src,BLK_MAX_SIZE) == -1)
                return -1;
            
            //erase src block
            if(Flash_Area_Erase(src,BLK_MAX_SIZE) == -1)
                return -1;
        }
    }
    
    if (Nvram_Block_Check("user") != 0)
    {
        Nvram_Block_Init("user");
    }

    if (Nvram_Block_Check("sys") != 0)
    {
        Nvram_Block_Init("sys");
    }
    
    return 0;
}

/*
 * write a block to nvram
 */ 
int Nvram_Block_Write(char *name, void *buf, uint16_t size)
{
    int i;
    uint32_t  src;
    uint32_t  dst;

    for (i = 0; i < FB_SIZE; i ++)
    {
        if (strcmp(name, fb[i].name) == 0)
            break;
    }

    if (i == FB_SIZE) 
        return -1;

    if (fb[i].size != size)
        return -1;
    
    //step 1: erase swap partition
    if(Flash_Area_Erase(CFG_NVRAM_ADDR + fb[0].offset,BLK_MAX_SIZE) == -1)
        return -1;

    //step 2: write data to swap partition
    src = (uint32_t)buf;
    dst = CFG_NVRAM_ADDR + fb[0].offset;
    if(Flash_Area_Prog(dst,(uint8_t*)src,size) == -1)
        return -1;

    //step 3: erase dst partition 
    if(Flash_Area_Erase(CFG_NVRAM_ADDR + fb[i].offset,BLK_MAX_SIZE) == -1)
        return -1;

    //step 4: copy data to dst partiton
    src = (uint32_t)buf;
    dst = CFG_NVRAM_ADDR + fb[i].offset;
    if(Flash_Area_Prog(dst,(uint8_t*)src,size) == -1)
        return -1;
    
    //step 5: erase swap partition
    if(Flash_Area_Erase(CFG_NVRAM_ADDR + fb[0].offset,BLK_MAX_SIZE) == -1)
        return -1;

    return 0;
}

int Nvram_Block_Read(char *name, void *buf, uint16_t bufSize)
{
    int i;
    uint8_t *dst;

    for (i = 0; i < FB_SIZE; i ++)
    {
        if (strcmp(name, fb[i].name) == 0)
            break;
    }

    if (i == FB_SIZE) 
        return -1;

    if (fb[i].size > bufSize)
        return -1;
    
    dst = (uint8_t *)(CFG_NVRAM_ADDR + fb[i].offset);

    memcpy(buf, dst, fb[i].size);

    return fb[i].size;
}

int Nvram_Block_Check(char *name)
{
    int         i;
    uint32_t    crc;
    uint8_t     *ptr;

    for (i = 0; i < FB_SIZE; i ++)
    {
        if (strcmp(name, fb[i].name) == 0)
            break;
    }

    if (i == FB_SIZE)
        return -1;

    ptr = (uint8_t *)(CFG_NVRAM_ADDR + fb[i].offset);
    
    crc = *(uint32_t *)ptr;
    ptr += sizeof(crc);

    if (crc != crc32(0, ptr, BLK_MAX_SIZE - sizeof(crc)))
        return -1;
    
    return 0;
}

int Nvram_Block_Init(char *name)
{
    int         i;
    uint8_t      *ptr;
    uint8_t     tmpblk[BLK_MAX_SIZE];

    for (i = 0; i < FB_SIZE; i ++)
    {
        if (strcmp(name, fb[i].name) == 0)
            break;
    }

    if (i == FB_SIZE)
        return -1;

    memset(tmpblk, 0, BLK_MAX_SIZE);

    ptr = tmpblk + sizeof(uint32_t);
    strcpy((char *)ptr, name);
    
    if (strcmp(name, "user") == 0)
    {
        nv_user_param_t *pblk = (nv_user_param_t *)tmpblk;
        
        //strcpy(pblk->ssid, "MediaSoc-BLE");
        //strcpy(pblk->key, "mediasoc");
        //strcpy(pblk->sip, "58.246.5.170");
        strcpy(pblk->ssid, "MS_BLE");
        strcpy(pblk->key, "ms123456");
        strcpy(pblk->sip, "139.196.197.176");
        pblk->port = 10680;
        pblk->net_mode = 0;
        pblk->lora_1para.bit_t.channel = 0;
        pblk->lora_1para.bit_t.power   = 1;
        pblk->lora_1para.bit_t.rate    = 1;
    }
    
    *(uint32_t *)tmpblk = crc32(0, ptr, BLK_MAX_SIZE-sizeof(uint32_t));
                                
    Nvram_Block_Write(name, tmpblk, BLK_MAX_SIZE);

    return 0;
}

int Nvram_Get_BlkSize_By_Name(char *name, uint16_t *blksize)
{
    int i;
    
    for (i = 0; i < FB_SIZE; i ++)
    {
        if (strcmp(name, fb[i].name) == 0)
        {
            *blksize = fb[i].size;
            return 0;
        }
    }
    
    return -1;
}

int Nvram_Get_BlkOffset_By_Name(char *name, uint16_t *offset)
{
    int i;
    
    for (i = 0; i < FB_SIZE; i ++)
    {
        if (strcmp(name, fb[i].name) == 0)
        {
            *offset = fb[i].offset;
            return 0;
        }
    }
    
    return -1;
}

