/*
 * Copyrite (C) 2016, BeeLinker
 * 
 * 文件名称: dbg.h
 * 文件说明：debug定义头文件
 *
 *
 * 版本信息：
 *
 */


#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "main.h"

/********************************************
* Defines *
********************************************/
//#define DEBUG

#ifdef DEBUG
    #define dbg(fmt, arg...)    printf(fmt, ##arg)
#else
    #define dbg(fmt, arg...)    do{}while(0)
#endif


/********************************************
* Typedefs *
********************************************/


/********************************************
* Globals * 
********************************************/


/********************************************
* Function defines *
********************************************/
void Uart1_PutChar(uint8_t ch);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);

#endif
