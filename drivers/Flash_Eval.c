/***********************************************************
*文件名:   Flash_Eval.c                                    * 
*                             							   *
*作者:	   wanchenchen						               *
*														   *
*文件说明：							     		           *
************************************************************/
 
#include "Flash_Eval.h"

/*******************************************************************************
* Function Name  : Flash_Area_Erase
* Description    : 擦除区域
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int Flash_Area_Erase(uint32_t Area_Addr, uint32_t Area_Size)
{
    uint32_t NbPage = 0;
    int i;
    
    /* erase image area */
    FLASH_Unlock();
    
    /* Clear all FLASH flags */
    NbPage = (Area_Size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
    
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    for(i=0; i<NbPage; i++)
    {
        IWDG_ReloadCounter();
        if (FLASH_COMPLETE != FLASH_ErasePage(Area_Addr+i*FLASH_PAGE_SIZE))
            goto FAILED;
    }
        
    FLASH_Lock();
    return 0;
    
FAILED:
    FLASH_Lock();
    return -1; 
}

/*******************************************************************************
* Function Name  : Flash_Area_Prog
* Description    : 对区域编程
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int Flash_Area_Prog(uint32_t Area_Addr, uint8_t*Data_Buf, uint32_t Data_Size)
{
    int i;
 
    /* program image area */
    FLASH_Unlock();
    
    /* Clear all FLASH flags */ 
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR); 
    for (i = 0; i < Data_Size; i += 4)
    {
        uint32_t u32tmp;
        
        memcpy(&u32tmp, (void *)Data_Buf, 4);
        
        IWDG_ReloadCounter();
        if(FLASH_COMPLETE != FLASH_ProgramWord(Area_Addr+i, u32tmp))
            goto FAILED;
        
        Data_Buf += 4;
    }
    
    FLASH_Lock();
    return 0;
    
FAILED:
    FLASH_Lock();
    return -1; 
}

