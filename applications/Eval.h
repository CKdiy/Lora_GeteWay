
#ifndef  __Eval_h__
#define  __Eval_h__

#include "main.h"

/***************调试器模式设置********************/
typedef enum 
{
	JTAG    = 0x00,
	SWD     = 0x01,
	ALL_OFF = 0x10
} Debug_Status;

extern uint32_t CpuID[4];

/***************Function**************************/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void IWDG_Configuration(void);
void Sys_Soft_Reset(void);
void JTAG_Set(Debug_Status mode);

#ifdef MD5_ENABLE
void GetDeviceId(uint8_t *buf, uint8_t size);
#endif

#endif

