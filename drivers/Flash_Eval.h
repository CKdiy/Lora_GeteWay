
#ifndef  __Flash_Eval_h__
#define  __Flash_Eval_h__

#include "main.h"

/***************Function**************************/
int Flash_Area_Erase(uint32_t Area_Addr, uint32_t Area_Size);
int Flash_Area_Prog(uint32_t Area_Addr, uint8_t*Data_Buf, uint32_t Data_Size);

#endif

