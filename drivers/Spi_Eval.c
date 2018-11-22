/***********************************************************
*文件名: Spi_Eval.c                                        *
*                                                          *
*作者:   ck                                                *
*                                                          *
*文件说明：                                                *
************************************************************/
  
#include "Spi_Eval.h"

#define	SPI2CS_PORT  	GPIOB
#define	SPI1CS_PORT   	GPIOA
#define	SPI2_CSPIN  	GPIO_Pin_12
#define	SPI1_CSPIN  	GPIO_Pin_4

/*************************************************
函数: void Spi2_Configuration(void)
功能: SPI 配置
参数: 无
返回: 无
注意：无
**************************************************/
void Spi2_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	//PORTB时钟使能 
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );	//SPI2时钟使能 	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  		//PB13/14/15复用推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//初始化GPIOB
	
	GPIO_InitStructure.GPIO_Pin = SPI2_CSPIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  		//PB12推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI2CS_PORT, &GPIO_InitStructure);			//初始化Spi2 cs

	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);  //PB13/14/15上拉
	GPIO_SetBits(SPI2CS_PORT,SPI2_CSPIN);  

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;			//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;				//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;			//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;				//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		//定义波特率预分频的值:波特率预分频值为32
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;		//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;				//CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);  					//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI2, ENABLE); 									//使能SPI外设
}

/*************************************************
函数: void Spi2_Configuration(void)
功能: SPI 配置
参数: TxData 要发送的数据
返回: 读到的字节
注意：无
**************************************************/
uint8_t Spi2_ReadWriteByte(uint8_t TxData)
{		
	uint8_t retry=0;	
	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
		retry++;
		if(retry>200)
			return 0;
	}			  
	SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据					    
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
函数: void Spi3_Configuration(void)
功能: SPI 配置
参数: 无
返回: 无
注意：无
**************************************************/
void Spi1_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );	//PORTA时钟使能 
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_SPI1,  ENABLE );	//SPI3时钟使能 	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  		//PA5/6/7复用推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//初始化GPIOA
	
	GPIO_InitStructure.GPIO_Pin = SPI1_CSPIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI1CS_PORT, &GPIO_InitStructure);			//初始化Spi1 cs   
	
	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);  //PA5/6/7上拉
	GPIO_SetBits(SPI1CS_PORT,SPI1_CSPIN); 
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;			//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;				//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;			//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;				//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;		//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;				//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);  					//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI1, ENABLE); 									//使能SPI外设
}

/*************************************************
函数: void Spi3_Configuration(void)
功能: SPI 配置
参数: TxData 要发送的数据
返回: 读到的字节
注意：无
**************************************************/
uint8_t Spi1_ReadWriteByte(uint8_t TxData)
{		
	uint8_t retry=0;	
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)  //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
		retry++;
		if(retry>200)
			return 0;
	}			  
	SPI_I2S_SendData(SPI1, TxData);  //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if(retry>200)
			return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据					    
}

void Spi1_AssertCSN(void)
{
	GPIO_ResetBits(SPI1CS_PORT, SPI1_CSPIN);
}

void Spi1_DeAssertCSN(void)
{
	GPIO_SetBits(SPI1CS_PORT, SPI1_CSPIN);
}
