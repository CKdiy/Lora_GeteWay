/***********************************************************
*�ļ���:   Led_Eval.c                                      * 
*                             							   *
*����:	   wanchenchen						               *
*														   *
*�ļ�˵����							     		           *
************************************************************/
 
#include "Led_Eval.h"

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED0_GPIO_PORT,LED1_GPIO_PORT,LED2_GPIO_PORT,LED3_GPIO_PORT,LED4_GPIO_PORT,STN_RST_GPIO_PORT,CHIPEN_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED0_GPIO_PIN,LED1_GPIO_PIN,LED2_GPIO_PIN,LED3_GPIO_PIN,LED4_GPIO_PIN,STN_RST_GPIO_PIN,CHIPEN_GPIO_PIN};

/*******************************************************************************
* Function Name  : LED_Init
* Description    : ��ʼ��LED�ƶ˿�
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;        //����GPIO��ʼ���ṹ��
	char i;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC,ENABLE);

	for(i=0;i<LEDn;i++)
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_PIN[i];
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIO_PORT[i], &GPIO_InitStructure); 
	}  
	
	LED_Control(LED0,Off);
	LED_Control(LED1,Off);
    LED_Control(LED2,Off);	 
    LED_Control(LED3,Off);	 
    LED_Control(LED4,Off);
    LED_Control(STN_RST,On);
    LED_Control(CHIPEN,On);
}

/*******************************************************************************
* Function Name  : LED_Control
* Description    : ����LED�Ƶ�״̬
* Input          : 1��LEDn(n=1,2����)��2��LED״̬�������𡢷�ת��
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LED_Control(Led_TypeDef Led,Led_Status Status)
{
	if(Status == On)
	{
		GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];
	}
	else if(Status == Off)
	{
		GPIO_PORT[Led]->BRR = GPIO_PIN[Led];
	}
	else 
	{
		GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
	}
}
