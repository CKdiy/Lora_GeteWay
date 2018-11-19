/*
 * Copyrite (C) 2016, BeeLinker
 * 
 * �ļ�����: dbg.h
 * �ļ�˵����debug����ͷ�ļ�
 *
 *
 * �汾��Ϣ��
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
