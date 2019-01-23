/***********************************************************
*�ļ���: Timer_Eval.c                                      *
*                                                          *
*����:   wanchenchen                                       *
*                                                          *
*�ļ�˵����                                                *
************************************************************/
  
#include "Timer_Eval.h"

uint16_t ext_tick = 0;
volatile uint32_t system_synchro_time = 0;
volatile uint16_t update_one_senond = 0;
volatile bool sendLoraTag_Flg = false;
	
/*************************************************
����: void Timer4_Configuration(void)
����: TIM2 ����
����: ��
����: ��
��ʱ���㣺(1 /(72 / (36 - 1 + 1))) * 2000 us = 1000us 
**************************************************/
void Timer4_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	    //��TIM2��ʱ����ʱ��
	
	TIM_DeInit(TIM4);		                                    //TIMx�Ĵ�������Ϊȱʡֵ
	
	TIM_TimeBaseStructure.TIM_Period = 1000;		       //�Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler=720 - 1;               //TIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;     //������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);                       //�����Զ���װ�ؼĴ�����ARR��
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	              //����жϱ�־
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);	                //����TIM2����ж�
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;			 //ͨ������ΪTimer2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 //�ж���Ӧ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 //���ж�
	NVIC_Init(&NVIC_InitStructure);  						//��ʼ�� 
	
	TIM_Cmd(TIM4, ENABLE);	                                //TIM2����ʱ��
}

/*************************************************
����: void TIM4_IRQHandler(void)
����: TIM4�жϴ�����
����: ��
����: ��
˵����1ms�ж�1��
**************************************************/
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4,TIM_IT_Update)!=RESET)
	{      
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	 //����жϱ�־
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
        
        ext_tick++;
        
        update_one_senond++;
        if(update_one_senond >= 100)
        {
            update_one_senond = 0;
            system_synchro_time++;
            sendLoraTag_Flg = true;
        }
	}
}

/*************************************************
����: uint32_t get_current_tick(void)
����: ��ȡϵͳ��ʱֵ
����: ��
����: ����32λ�ļ���ֵ
˵������
**************************************************/
uint32_t get_current_tick(void)
{
    uint32_t tick;
    uint16_t    tickH1, tickH2;
    uint16_t    tickL;
    
    do
    {
        tickH1 = ext_tick;
        tickL = TIM_GetCounter(TIM4);
        tickH2 = ext_tick;
    }while(tickH1 != tickH2);
    
    tick = tickH1;
    tick *= 1000;
    tick += tickL;

    return tick;
}

/*************************************************
����: uint32_t get_synchro_time(void)
����: ��ȡͬ��ʱ��ֵ
����: ��
����: ����32λ��ͬ��ʱ��
˵������
**************************************************/
uint32_t get_synchro_time(void)
{
    uint32_t temp_syn_time;
    
    ATOMIC
    (
        temp_syn_time = system_synchro_time;
    )
    
    return temp_syn_time;
}

/*************************************************
����: void set_synchro_time(uint32_t syn_time_value)
����: ����ͬ��ʱ��ֵ
����: ͬ��ʱ������ֵ
����: ��
˵������
**************************************************/
void set_synchro_time(uint32_t syn_time_value)
{
    ATOMIC
    (
        system_synchro_time = syn_time_value;
		update_one_senond = 0;
    )
}

/*************************************************
����: void Timer4_Rstart(void)
����: ����Timer4���¼�ʱ
����: ��
����: ��
˵������
**************************************************/
void Timer4_Rstart(void)
{
	TIM_SetCounter(TIM4, 1);
	system_synchro_time = 0;
}
