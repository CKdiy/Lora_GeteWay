/***********************************************************
*文件名: Timer_Eval.c                                      *
*                                                          *
*作者:   wanchenchen                                       *
*                                                          *
*文件说明：                                                *
************************************************************/
  
#include "Timer_Eval.h"

uint16_t ext_tick = 0;
volatile uint32_t system_synchro_time = 0;
volatile uint16_t update_one_senond = 0;
volatile bool sendLoraTag_Flg = false;
	
/*************************************************
函数: void Timer4_Configuration(void)
功能: TIM2 配置
参数: 无
返回: 无
定时计算：(1 /(72 / (36 - 1 + 1))) * 2000 us = 1000us 
**************************************************/
void Timer4_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	    //打开TIM2定时器的时钟
	
	TIM_DeInit(TIM4);		                                    //TIMx寄存器重设为缺省值
	
	TIM_TimeBaseStructure.TIM_Period = 1000;		       //自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler=720 - 1;               //TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;     //采样分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);                       //允许自动重装载寄存器（ARR）
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	              //清除中断标志
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);	                //允许TIM2溢出中断
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;			 //通道设置为Timer2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 //中断响应优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 //打开中断
	NVIC_Init(&NVIC_InitStructure);  						//初始化 
	
	TIM_Cmd(TIM4, ENABLE);	                                //TIM2开启时钟
}

/*************************************************
函数: void TIM4_IRQHandler(void)
功能: TIM4中断处理函数
参数: 无
返回: 无
说明：1ms中断1次
**************************************************/
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4,TIM_IT_Update)!=RESET)
	{      
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	 //清除中断标志
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
函数: uint32_t get_current_tick(void)
功能: 获取系统计时值
参数: 无
返回: 返回32位的计数值
说明：无
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
函数: uint32_t get_synchro_time(void)
功能: 获取同步时钟值
参数: 无
返回: 返回32位的同步时钟
说明：无
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
函数: void set_synchro_time(uint32_t syn_time_value)
功能: 设置同步时钟值
参数: 同步时钟设置值
返回: 无
说明：无
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
函数: void Timer4_Rstart(void)
功能: 设置Timer4重新计时
参数: 无
返回: 无
说明：无
**************************************************/
void Timer4_Rstart(void)
{
	TIM_SetCounter(TIM4, 1);
	system_synchro_time = 0;
}
