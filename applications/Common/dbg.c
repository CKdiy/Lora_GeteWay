
#include "dbg.h"

/*******************************************************************************
* Function Name  : Uart1_PutChar
* Description    : 串口发送单个字节
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Uart1_PutChar(uint8_t ch)
{
    USART_ClearFlag(USART1,USART_FLAG_TC); 
	USART_SendData(USART1, (uint8_t) ch);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

/*******************************************************************************
* Function Name  : int fputc(int ch, FILE *f)
* Description    : Retargets the C library printf function to the USART.printf???¨?ò
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int fputc(int ch, FILE *f)
{
 	Uart1_PutChar((uint8_t) ch);	
	return ch;
}

/*******************************************************************************
* Function Name  : int fgetc(FILE *f)
* Description    : Retargets the C library printf function to the USART.fgetc???¨?ò
* Input          : None
* Output         : None
* Return         : ?áè?μ?μ?×?·?
*******************************************************************************/
int fgetc(FILE *f)
{
	/* Loop until received a char */
	while(!(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET))
	{
	}
	
	/* Read a character from the USART and RETURN */
	return (USART_ReceiveData(USART1));
}


