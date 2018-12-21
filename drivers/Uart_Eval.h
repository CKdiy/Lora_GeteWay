
#ifndef __Uart_Eval_h__
#define __Uart_Eval_h__

#include "main.h"

typedef struct
{
    uint8_t sys1;
    uint8_t sys2;
    uint8_t sys3;
    uint8_t sys4;
    uint16_t ID;
    uint8_t Data[8];
    uint8_t crc1;
    uint8_t crc2;  
}Uart_AllData; 

#define USART_NONE  0x00
#define USART_TX    0x01

/****************USART1接收缓存*******************/
#define UART1_RXBUF_SIZE        1024
#define UART1_TXBUF_SIZE        1024
extern uint8_t  Uart1_RxBuf[UART1_RXBUF_SIZE];
extern volatile uint16_t Uart1_RxBuf_Read;
extern volatile uint16_t Uart1_RxBuf_Write;
extern volatile uint32_t Uart1_RxCnt;
/*************************************************/

/****************USART2接收缓存*******************/
#define UART2_RXBUF_SIZE        1024
#define UART2_TXBUF_SIZE        1024
extern uint8_t  Uart2_RxBuf[UART2_RXBUF_SIZE];
extern volatile uint16_t Uart2_RxBuf_Read;
extern volatile uint16_t Uart2_RxBuf_Write;
extern volatile uint32_t Uart2_RxCnt;
/*************************************************/

/****************USART3接收缓存*******************/
#define UART3_RXBUF_SIZE        1024
#define UART3_TXBUF_SIZE        1024
extern uint8_t  Uart3_RxBuf[UART3_RXBUF_SIZE];
extern volatile uint16_t Uart3_RxBuf_Read;
extern volatile uint16_t Uart3_RxBuf_Write;
extern volatile uint32_t Uart3_RxCnt;
/*************************************************/

void UART_Work_Init(void);

void USART1_Configuration(void);
void USART1_SendData( uint8_t *pData , uint16_t lens );

void USART2_Configuration(void);
void USART2_SendData( uint8_t *pData , uint16_t lens );

void USART3_Configuration(void);
void USART3_SendData( uint8_t *pData , uint16_t lens );

#endif
