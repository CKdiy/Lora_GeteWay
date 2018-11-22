
#ifndef  __Spi_Eval_h__
#define  __Spi_Eval_h__

#include "main.h"

void Spi2_Configuration(void);
void Spi2_AssertCSN(void);
void Spi2_DeAssertCSN(void);
uint8_t Spi2_ReadWriteByte(uint8_t TxData);

void Spi1_Configuration(void);
void Spi1_AssertCSN(void);
void Spi1_DeAssertCSN(void);
uint8_t Spi1_ReadWriteByte(uint8_t TxData);
#endif

