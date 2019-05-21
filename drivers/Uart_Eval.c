/***********************************************************
*�ļ���: USART_Eval.c                                      *
*                                                          *
*����:   wanchenchen                                       *
*                                                          *
*�ļ�˵����                                                *
************************************************************/

#include "Uart_Eval.h"

/*******************************************************************************
* Function Name  : UART_Work_Init
* Description    : ��ʼ��UART
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void UART_Work_Init(void)
{
    /*********���ڴ�ӡ��Ԫ**************/
	//USART1_Configuration();	
    /*********�������ͨѶ**************/
	USART2_Configuration();	
    /*********ATָ������****************/
	UART4_Configuration();	
}

//---------------------------����1������ʼ-------------------------------------//
/****************USART1���ջ���*******************/
uint8_t  Uart1_RxBuf[UART1_RXBUF_SIZE];
volatile uint16_t Uart1_RxBuf_Read  = 0;
volatile uint16_t Uart1_RxBuf_Write = 0;
volatile uint32_t Uart1_RxCnt = 0;
/*************************************************/

/****************USART1���ͻ���*******************/
uint8_t Uart1_TxBuf[UART1_TXBUF_SIZE];
volatile uint16_t Uart1_TxBuf_Read  = 0;
volatile uint16_t Uart1_TxBuf_Write = 0;
volatile uint8_t Uart1_Tx_Is_Trigered = USART_NONE;
/*************************************************/
 /*******************************************************************************
��������USART1_Configuration
��  ��:
��  ��:
����˵������ʼ������Ӳ���豸�������ж�
���ò��裺
		(1)��GPIO��USART1��ʱ��
		(2)����USART1�����ܽ�GPIOģʽ
		(3)����USART1���ݸ�ʽ�������ʵȲ���
		(4)ʹ��USART1�����жϹ���
		(5)���ʹ��USART1����
********************************************************************************/
void USART1_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;		
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART1, &USART_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       //ͨ������Ϊ����1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	   //�ж���Ӧ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		   //���ж�
	NVIC_Init(&NVIC_InitStructure); 						   //��ʼ��
	
    /* ���������ݼĴ�������������ж� */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);
	
	/* �����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(USART1, USART_FLAG_TC);     // ���־
}

/*******************************************************************************
* Function Name  : USART1_SendDataString
* Description    : �ж���ʽ��������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_SendData( uint8_t *pData , uint16_t lens )
{	
    uint16_t remain;
    uint16_t   tmpWrite, tmpRead;
    
    ATOMIC
    (
        tmpWrite = Uart1_RxBuf_Write;
        tmpRead = Uart1_RxBuf_Read;
    )
    
    
    if (tmpWrite < tmpRead)
        remain = tmpRead - tmpWrite - 1;
    else
        remain = UART1_TXBUF_SIZE + tmpRead - tmpWrite - 1;
     
    while(lens && remain)
    {
        Uart1_TxBuf[Uart1_TxBuf_Write++] = *pData ++;
        if (Uart1_TxBuf_Write >= UART1_TXBUF_SIZE)
        {
            Uart1_TxBuf_Write = 0;  
        }

        remain --;
        lens --;
    }

    if (Uart1_Tx_Is_Trigered != USART_TX)
    {  
        Uart1_Tx_Is_Trigered = USART_TX;
        USART_ClearFlag(USART1,USART_FLAG_TC); 

        USART_SendData(USART1,Uart1_TxBuf[Uart1_TxBuf_Read ++]);
        if(Uart1_TxBuf_Read >=UART1_TXBUF_SIZE) 
        {              
            Uart1_TxBuf_Read = 0;  
        }
        USART_ITConfig(USART1, USART_IT_TC, ENABLE);  
    } 
}

/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : �����ж�
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void USART1_IRQHandler(void)           
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    //���������ݼĴ�����
	{
        uint16_t tmpWrite;
        uint8_t  u8tmp;
        
		u8tmp = USART_ReceiveData(USART1); 
        USART_ClearITPendingBit(USART1,USART_IT_RXNE); 
        
        tmpWrite = Uart1_RxBuf_Write + 1;
        if(tmpWrite >= UART1_RXBUF_SIZE)         
        {
            tmpWrite = 0;
        }
        
        if (tmpWrite != Uart1_RxBuf_Read)
        {
            Uart1_RxBuf[Uart1_RxBuf_Write] = u8tmp;
            Uart1_RxBuf_Write = tmpWrite;
            Uart1_RxCnt++;
        }
        
	}
	else if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)    //���������ݼĴ�����
	{
        USART_ClearITPendingBit(USART1,USART_IT_TC);
        if(Uart1_TxBuf_Write!=Uart1_TxBuf_Read)
        {
            USART_SendData(USART1,Uart1_TxBuf[Uart1_TxBuf_Read ++]);
            if(Uart1_TxBuf_Read >=UART1_TXBUF_SIZE) 
            {              
                Uart1_TxBuf_Read = 0;  
            }
        }
        else
        {
            USART_ITConfig(USART1, USART_IT_TC, DISABLE);
            Uart1_Tx_Is_Trigered = USART_NONE;
        }
	}	
} 
//---------------------------����1��������-------------------------------------//

//---------------------------����2������ʼ-------------------------------------//
/****************USART2���ջ���*******************/
uint8_t  Uart2_RxBuf[UART2_RXBUF_SIZE];
volatile uint16_t Uart2_RxBuf_Read  = 0;
volatile uint16_t Uart2_RxBuf_Write = 0;
volatile uint32_t Uart2_RxCnt = 0;
/*************************************************/

/****************USART2���ͻ���*******************/
uint8_t Uart2_TxBuf[UART2_TXBUF_SIZE];
volatile uint16_t Uart2_TxBuf_Read  = 0;
volatile uint16_t Uart2_TxBuf_Write = 0;
volatile uint8_t Uart2_Tx_Is_Trigered = USART_NONE;
/*************************************************/
 /*******************************************************************************
��������USART2_Configuration
��  ��:
��  ��:
����˵������ʼ������Ӳ���豸�������ж�
���ò��裺
		(1)��GPIO��USART1��ʱ��
		(2)����USART1�����ܽ�GPIOģʽ
		(3)����USART1���ݸ�ʽ�������ʵȲ���
		(4)ʹ��USART1�����жϹ���
		(5)���ʹ��USART1����
********************************************************************************/
void USART2_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;		
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART2, &USART_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;       //ͨ������Ϊ����1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	   //�ж���Ӧ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		   //���ж�
	NVIC_Init(&NVIC_InitStructure); 						   //��ʼ��
	
    /* ���������ݼĴ�������������ж� */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	/* Enable USART */
	USART_Cmd(USART2, ENABLE);
	
	/* �����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(USART2, USART_FLAG_TC);     // ���־
}

/*******************************************************************************
* Function Name  : USART1_SendDataString
* Description    : �ж���ʽ��������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_SendData( uint8_t *pData , uint16_t lens )
{	
    uint16_t remain;
    uint16_t   tmpWrite, tmpRead;
    
    ATOMIC
    (
        tmpWrite = Uart2_RxBuf_Write;
        tmpRead = Uart2_RxBuf_Read;
    )
    
    
    if (tmpWrite < tmpRead)
        remain = tmpRead - tmpWrite - 1;
    else
        remain = UART2_TXBUF_SIZE + tmpRead - tmpWrite - 1;
     
    while(lens && remain)
    {
        Uart2_TxBuf[Uart2_TxBuf_Write++] = *pData ++;
        if (Uart2_TxBuf_Write >= UART2_TXBUF_SIZE)
        {
            Uart2_TxBuf_Write = 0;  
        }

        remain --;
        lens --;
    }

    if (Uart2_Tx_Is_Trigered != USART_TX)
    {  
        Uart2_Tx_Is_Trigered = USART_TX;
        USART_ClearFlag(USART2,USART_FLAG_TC); 

        USART_SendData(USART2,Uart2_TxBuf[Uart2_TxBuf_Read ++]);
        if(Uart2_TxBuf_Read >=UART2_TXBUF_SIZE) 
        {              
            Uart2_TxBuf_Read = 0;  
        }
        USART_ITConfig(USART2, USART_IT_TC, ENABLE);  
    } 
}

/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : �����ж�
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void USART2_IRQHandler(void)           
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)    //���������ݼĴ�����
	{
        uint16_t tmpWrite;
        uint8_t  u8tmp;
        
		u8tmp = USART_ReceiveData(USART2); 
        USART_ClearITPendingBit(USART2,USART_IT_RXNE); 

        tmpWrite = Uart2_RxBuf_Write + 1;
        if(tmpWrite >= UART2_RXBUF_SIZE)         
            tmpWrite = 0;
            
            Uart2_RxBuf[Uart2_RxBuf_Write] = u8tmp;
            Uart2_RxBuf_Write = tmpWrite;
            Uart2_RxCnt++;        
	}
	else if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)    //���������ݼĴ�����
	{
        USART_ClearITPendingBit(USART2,USART_IT_TC);
        if(Uart2_TxBuf_Write!=Uart2_TxBuf_Read)
        {
            USART_SendData(USART2,Uart2_TxBuf[Uart2_TxBuf_Read ++]);
            if(Uart2_TxBuf_Read >=UART2_TXBUF_SIZE) 
            {              
                Uart2_TxBuf_Read = 0;  
            }
        }
        else
        {
            USART_ITConfig(USART2, USART_IT_TC, DISABLE);
            Uart2_Tx_Is_Trigered = USART_NONE;
        }
	}	
} 
//---------------------------����2��������-------------------------------------//

//---------------------------����4������ʼ-------------------------------------//
/****************UART4���ջ���*******************/
uint8_t  Uart4_RxBuf[UART4_RXBUF_SIZE];
volatile uint16_t Uart4_RxBuf_Read  = 0;
volatile uint16_t Uart4_RxBuf_Write = 0;
volatile uint32_t Uart4_RxCnt = 0;
/*************************************************/

/****************UART4���ͻ���*******************/
uint8_t Uart4_TxBuf[UART4_TXBUF_SIZE];
volatile uint16_t Uart4_TxBuf_Read  = 0;
volatile uint16_t Uart4_TxBuf_Write = 0;
volatile uint8_t Uart4_Tx_Is_Trigered = USART_NONE;
/*************************************************/
 /*******************************************************************************
��������USART4_Configuration
��  ��:
��  ��:
����˵������ʼ������Ӳ���豸�������ж�
���ò��裺
		(1)��GPIO��USART1��ʱ��
		(2)����USART4�����ܽ�GPIOģʽ
		(3)����USART4���ݸ�ʽ�������ʵȲ���
		(4)ʹ��USART4�����жϹ���
		(5)���ʹ��USART4����
********************************************************************************/
void UART4_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;		
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(UART4, &USART_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;       //ͨ������Ϊ����1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	   //�ж���Ӧ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		   //���ж�
	NVIC_Init(&NVIC_InitStructure); 						   //��ʼ��
	
    /* ���������ݼĴ�������������ж� */
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

	/* Enable USART */
	USART_Cmd(UART4, ENABLE);
	
	/* �����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(UART4, USART_FLAG_TC);     // ���־
}

/*******************************************************************************
* Function Name  : USART4_SendDataString
* Description    : �ж���ʽ��������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART4_SendData( uint8_t *pData , uint16_t lens )
{	
    uint16_t remain;
    uint16_t   tmpWrite, tmpRead;
    
    ATOMIC
    (
        tmpWrite = Uart4_RxBuf_Write;
        tmpRead = Uart4_RxBuf_Read;
    )
    
    
    if (tmpWrite < tmpRead)
        remain = tmpRead - tmpWrite - 1;
    else
        remain = UART4_TXBUF_SIZE + tmpRead - tmpWrite - 1;
     
    while(lens && remain)
    {
        Uart4_TxBuf[Uart4_TxBuf_Write++] = *pData ++;
        if (Uart4_TxBuf_Write >= UART4_TXBUF_SIZE)
        {
            Uart4_TxBuf_Write = 0;  
        }

        remain --;
        lens --;
    }

    if (Uart4_Tx_Is_Trigered != USART_TX)
    {  
        Uart4_Tx_Is_Trigered = USART_TX;
        USART_ClearFlag(UART4,USART_FLAG_TC); 

        USART_SendData(UART4,Uart4_TxBuf[Uart4_TxBuf_Read ++]);
        if(Uart4_TxBuf_Read >=UART4_TXBUF_SIZE) 
        {              
            Uart4_TxBuf_Read = 0;  
        }
        USART_ITConfig(UART4, USART_IT_TC, ENABLE);  
    } 
}

/*******************************************************************************
* Function Name  : UART4_IRQHandler
* Description    : �����ж�
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void UART4_IRQHandler(void)           
{
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)    //���������ݼĴ�����
	{
        uint16_t tmpWrite;
        uint8_t  u8tmp;
        
		u8tmp = USART_ReceiveData(UART4); 
        USART_ClearITPendingBit(UART4,USART_IT_RXNE); 
        
        tmpWrite = Uart4_RxBuf_Write + 1;
        if(tmpWrite >= UART4_RXBUF_SIZE)         
        {
            tmpWrite = 0;
        }
        
        if (tmpWrite != Uart4_RxBuf_Read)
        {
            Uart4_RxBuf[Uart4_RxBuf_Write] = u8tmp;
            Uart4_RxBuf_Write = tmpWrite;
            Uart4_RxCnt++;
        }
        
	}
	else if(USART_GetITStatus(UART4, USART_IT_TC) != RESET)    //���������ݼĴ�����
	{
        USART_ClearITPendingBit(UART4,USART_IT_TC);
        if(Uart4_TxBuf_Write!=Uart4_TxBuf_Read)
        {
            USART_SendData(UART4,Uart4_TxBuf[Uart4_TxBuf_Read ++]);
            if(Uart4_TxBuf_Read >=UART4_TXBUF_SIZE) 
            {              
                Uart4_TxBuf_Read = 0;  
            }
        }
        else
        {
            USART_ITConfig(UART4, USART_IT_TC, DISABLE);
            Uart4_Tx_Is_Trigered = USART_NONE;
        }
	}	
} 
//---------------------------����4��������-------------------------------------//


