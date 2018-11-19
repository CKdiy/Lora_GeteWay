/***********************************************************
*�ļ���: Eval.c                                            *
*                                                          *
*����:   wanchenchen                                       *
*                                                          *
*�ļ�˵����                                                *
************************************************************/
  
#include "Eval.h"

/*************************************************
����: void RCC_Configuration(void)
����: ��λ��ʱ�ӿ��� ����
����: ��
����: ��
**************************************************/
void RCC_Configuration(void)
{
  ErrorStatus HSEStartUpStatus;                    //�����ⲿ���پ�������״̬ö�ٱ���
  RCC_DeInit();                                    //��λRCC�ⲿ�豸�Ĵ�����Ĭ��ֵ
  RCC_HSEConfig(RCC_HSE_ON);                       //���ⲿ���پ���
  HSEStartUpStatus = RCC_WaitForHSEStartUp();      //�ȴ��ⲿ����ʱ��׼����
  if(HSEStartUpStatus == SUCCESS)                  //�ⲿ����ʱ���Ѿ�׼���
  {
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); //����FLASHԤ�����幦�ܣ�����FLASH�Ķ�ȡ�����г����б�����÷�.λ�ã�RCC��ʼ���Ӻ������棬ʱ������֮��
    FLASH_SetLatency(FLASH_Latency_2);                    //flash��������ʱ
      	
    RCC_HCLKConfig(RCC_SYSCLK_Div1);               //����AHB(HCLK)ʱ�ӵ���==SYSCLK
    RCC_PCLK2Config(RCC_HCLK_Div1);                //����APB2(PCLK2)��==AHBʱ��
    RCC_PCLK1Config(RCC_HCLK_Div2);                //����APB1(PCLK1)��==AHB1/2ʱ��
         
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

    RCC_PLLCmd(ENABLE);                                   //ʹ��PLLʱ��
   
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)    //�ȴ�PLLʱ�Ӿ���
    {
    }
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);            //����ϵͳʱ�� = PLLʱ��
    while(RCC_GetSYSCLKSource() != 0x08)                  //���PLLʱ���Ƿ���Ϊϵͳʱ��
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
	//�����������λ�ú�ƫ��
	#ifdef  VECT_TAB_RAM  
		/* Set the Vector Table base location at 0x20000000 */ 
		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0000); 		//������λ��RAM
	#else  /* VECT_TAB_FLASH  */
		/* Set the Vector Table base location at 0x08000000 */ 
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);   //������λ��FLASH
	#endif
	
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
}

/**********************************************************************
* ��    �ƣ�IWDG_Configuration()
* ��    �ܣ����Ź�����
* ��ڲ����� 
* ���ڲ�����
***********************************************************************/
void IWDG_Configuration(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); 	//����֮ǰҪ����ʹ�ܼĴ���д
	IWDG_SetPrescaler(IWDG_Prescaler_64);           //�ڲ�����ʱ��16��Ƶ,��Ƶ��Ϊ��40K / 64 =  0.625K������һ������Ϊ��1.6ms
	IWDG_SetReload(800);							//800*1.6ms = 1.28S
	IWDG_ReloadCounter();						    //ι���������������һ���ļ��д��0xAAAA�����򣬵�������Ϊ0ʱ�����Ź��������λ
	IWDG_Enable(); 									//ʹ��
}

/*******************************************************************************
* Function Name   : Sys_Soft_Reset
* Description     : ϵͳ�����λ
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
* Description     : JTAGģʽ����
* Input           : mode:jtag,swdģʽ����;00,ȫʹ��;01,ʹ��SWD;10,ȫ�ر�;
* Output          : None
* Return          : None
*******************************************************************************/	  
void JTAG_Set(Debug_Status mode)
{
	uint32_t temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //��������ʱ��	   
	AFIO->MAPR&=0XF8FFFFFF; //���MAPR��[26:24]
	AFIO->MAPR|=temp;       //����jtagģʽ
} 

/*******************************************************************************
* Function Name   : GetDeviceId
* Description     : ��ȡ�豸��
* Input           : ID�洢��ַ,����
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
