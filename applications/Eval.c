/***********************************************************
*文件名: Eval.c                                            *
*                                                          *
*作者:   wanchenchen                                       *
*                                                          *
*文件说明：                                                *
************************************************************/
  
#include "Eval.h"

/*************************************************
函数: void RCC_Configuration(void)
功能: 复位和时钟控制 配置
参数: 无
返回: 无
**************************************************/
void RCC_Configuration(void)
{
  ErrorStatus HSEStartUpStatus;                    //定义外部高速晶体启动状态枚举变量
  RCC_DeInit();                                    //复位RCC外部设备寄存器到默认值
  RCC_HSEConfig(RCC_HSE_ON);                       //打开外部高速晶振
  HSEStartUpStatus = RCC_WaitForHSEStartUp();      //等待外部高速时钟准备好
  if(HSEStartUpStatus == SUCCESS)                  //外部高速时钟已经准别好
  {
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); //开启FLASH预读缓冲功能，加速FLASH的读取。所有程序中必须的用法.位置：RCC初始化子函数里面，时钟起振之后
    FLASH_SetLatency(FLASH_Latency_2);                    //flash操作的延时
      	
    RCC_HCLKConfig(RCC_SYSCLK_Div1);               //配置AHB(HCLK)时钟等于==SYSCLK
    RCC_PCLK2Config(RCC_HCLK_Div1);                //配置APB2(PCLK2)钟==AHB时钟
    RCC_PCLK1Config(RCC_HCLK_Div2);                //配置APB1(PCLK1)钟==AHB1/2时钟
         
#ifdef STM32F10X_CL
    /* Configure PLLs *********************************************************/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    RCC_PREDIV2Config(RCC_PREDIV2_Div5);
    RCC_PLL2Config(RCC_PLL2Mul_8);

    /* Enable PLL2 */
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {}

    /* PLL configuration: PLLCLK = (PLL2 / 5) * 9 = 72 MHz */ 
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);
#else
    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);
#endif

    RCC_PLLCmd(ENABLE);                                   //使能PLL时钟
   
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)    //等待PLL时钟就绪
    {
    }
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);            //配置系统时钟 = PLL时钟
    while(RCC_GetSYSCLKSource() != 0x08)                  //检查PLL时钟是否作为系统时钟
    {
    }
  }
}

/*******************************************************************************
* Function Name   : NVIC_Configuration
* Description        : Configures NVIC and Vector Table base location.
* Input                    : None
* Output                 : None
* Return                 : None
*******************************************************************************/
void NVIC_Configuration(void)
{
	//设置向量表的位置和偏移
	#ifdef  VECT_TAB_RAM  
		/* Set the Vector Table base location at 0x20000000 */ 
		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0000); 		//向量表位于RAM
	#else  /* VECT_TAB_FLASH  */
		/* Set the Vector Table base location at 0x08000000 */ 
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);   //向量表位于FLASH
	#endif
	
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
}

/**********************************************************************
* 名    称：IWDG_Configuration()
* 功    能：看门狗配置
* 入口参数： 
* 出口参数：
***********************************************************************/
void IWDG_Configuration(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); 	//访问之前要首先使能寄存器写
	IWDG_SetPrescaler(IWDG_Prescaler_64);           //内部低速时钟16分频,即频率为：40K / 64 =  0.625K，所以一个周期为：1.6ms
	IWDG_SetReload(800);							//800*1.6ms = 1.28S
	IWDG_ReloadCounter();						    //喂狗程序。软件必须以一定的间隔写入0xAAAA，否则，当计数器为0时，看门狗会产生复位
	IWDG_Enable(); 									//使能
}

/*******************************************************************************
* Function Name   : Sys_Soft_Reset
* Description     : 系统软件复位
* Input           : None
* Output          : None
* Return          : None
*******************************************************************************/
void Sys_Soft_Reset(void)
{   
    __set_FAULTMASK(1); 
	SCB->AIRCR =0X05FA0000|(uint32_t)0x04;	  
} 

/*******************************************************************************
* Function Name   : JTAG_Set
* Description     : JTAG模式设置
* Input           : mode:jtag,swd模式设置;00,全使能;01,使能SWD;10,全关闭;
* Output          : None
* Return          : None
*******************************************************************************/	  
void JTAG_Set(Debug_Status mode)
{
	uint32_t temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //开启辅助时钟	   
	AFIO->MAPR&=0XF8FFFFFF; //清除MAPR的[26:24]
	AFIO->MAPR|=temp;       //设置jtag模式
} 

/*******************************************************************************
* Function Name   : GetDeviceId
* Description     : 获取设备号
* Input           : ID存储地址,长度
* Output          : None
* Return          : None
*******************************************************************************/
#ifdef MD5_ENABLE
void GetDeviceId(uint8_t *buf, uint8_t size)
{
    MD5_CTX mdContext;
    uint32_t uid[3];
    char str[64];
    
    if (buf == NULL)
        return;
    
    memset(str, 0, sizeof(str));
    uid[0] = *(volatile uint32_t*)(0x1FFFF7E8);
    uid[1] = *(volatile uint32_t*)(0x1FFFF7EC);
    uid[2] = *(volatile uint32_t*)(0x1FFFF7F0);
    sprintf(str, "%08X%08X%08X4655434B594F5521", uid[0],uid[1],uid[2]);
    
    MD5Init(&mdContext);
    MD5Update(&mdContext, (uint8_t *)str, strlen(str));
    MD5Final(&mdContext);
    
    memcpy(&buf[1], mdContext.digest, size-1);
    
    buf[0] = crc8(0, &buf[1], size-1);
}
#endif
