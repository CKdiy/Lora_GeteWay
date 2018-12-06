/***********************************************************
*�ļ���: Spi_Eval.c                                        *
*                                                          *
*����:   ck                                                *
*                                                          *
*�ļ�˵����                                                *
************************************************************/
  
#include "Spi_Eval.h"

#define	SPI2CS_PORT  	GPIOB
#define	SPI1CS_PORT   	GPIOA
#define	SPI2_CSPIN  	GPIO_Pin_12
#define	SPI1_CSPIN  	GPIO_Pin_4

/*************************************************
����: void Spi2_Configuration(void)
����: SPI ����
����: ��
����: ��
ע�⣺��
**************************************************/
void Spi2_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	//PORTBʱ��ʹ�� 
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );	//SPI2ʱ��ʹ�� 	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  		//PB13/14/15����������� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//��ʼ��GPIOB
	
	GPIO_InitStructure.GPIO_Pin = SPI2_CSPIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  		//PB12������� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI2CS_PORT, &GPIO_InitStructure);			//��ʼ��Spi2 cs

	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);  //PB13/14/15����
	GPIO_SetBits(SPI2CS_PORT,SPI2_CSPIN);  

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;			//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;				//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;			//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;				//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ32
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;		//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;				//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);  					//����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI2, ENABLE); 									//ʹ��SPI����
}

/*************************************************
����: void Spi2_Configuration(void)
����: SPI ����
����: TxData Ҫ���͵�����
����: �������ֽ�
ע�⣺��
**************************************************/
uint8_t Spi2_ReadWriteByte(uint8_t TxData)
{		
	uint8_t retry=0;	
	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		retry++;
		if(retry>200)
			return 0;
	}			  
	SPI_I2S_SendData(SPI2, TxData); //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI2); //����ͨ��SPIx������յ�����					    
}

void Spi2_AssertCSN(void)
{
	GPIO_ResetBits(SPI2CS_PORT, SPI2_CSPIN);
}

void Spi2_DeAssertCSN(void)
{
	GPIO_SetBits(SPI2CS_PORT, SPI2_CSPIN);
}


/*************************************************
����: void Spi3_Configuration(void)
����: SPI ����
����: ��
����: ��
ע�⣺��
**************************************************/
void Spi1_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );	//PORTAʱ��ʹ�� 
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_SPI1,  ENABLE );	//SPI3ʱ��ʹ�� 	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  		//PA5/6/7����������� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//��ʼ��GPIOA
	
	GPIO_InitStructure.GPIO_Pin = SPI1_CSPIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI1CS_PORT, &GPIO_InitStructure);			//��ʼ��Spi1 cs   
	
	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);  //PA5/6/7����
	GPIO_SetBits(SPI1CS_PORT,SPI1_CSPIN); 
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;			//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;				//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;			//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;				//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;		//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;				//CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure);  					//����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI1, ENABLE); 									//ʹ��SPI����
}

/*************************************************
����: void Spi3_Configuration(void)
����: SPI ����
����: TxData Ҫ���͵�����
����: �������ֽ�
ע�⣺��
**************************************************/
uint8_t Spi1_ReadWriteByte(uint8_t TxData)
{		
	uint8_t retry=0;	
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)  //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		retry++;
		if(retry>200)
			return 0;
	}			  
	SPI_I2S_SendData(SPI1, TxData);  //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
		retry++;
		if(retry>200)
			return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����					    
}

void Spi1_AssertCSN(void)
{
	GPIO_ResetBits(SPI1CS_PORT, SPI1_CSPIN);
}

void Spi1_DeAssertCSN(void)
{
	GPIO_SetBits(SPI1CS_PORT, SPI1_CSPIN);
}
