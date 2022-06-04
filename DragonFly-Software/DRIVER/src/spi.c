/***************************************************************************************
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：V1.01
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "spi.h"

/*****************************************************************************
*函  数：void SPI2_Init(void)
*功  能：SPI2初始化
*参  数：无
*返回值：无
*备  注：调试SPI通信时一定要分清主机从机模式；
*        主机从机模式的空闲状态电平（）
*****************************************************************************/
void SPI2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;
 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_APB1Periph_SPI2 , ENABLE);
 
  //将PB13 14 15复用为SPI2的对应引脚 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);    
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	//GPIO模式配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; //SPI2 SCK引脚
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; //SPI2 MISO引脚
  GPIO_Init(GPIOB, &GPIO_InitStructure);  

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //SPI2 MOSI引脚
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 , ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2 ,DISABLE);
	
	//SPI2模式配置
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //配置为主机模式
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //双线全双工模式
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //8位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //空闲状态为低电平
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //第一个时钟沿捕获
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSS信号软件管理
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //SPI的波特率 PCLK2(50M)/8=6.25M
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //先发送高位
  SPI_InitStructure.SPI_CRCPolynomial = 7; //CRC值计算的多项式
	
  SPI_Init(SPI2, &SPI_InitStructure);
	
  SPI_Cmd(SPI2, ENABLE); //SPI2使能
}

/*****************************************************************************
*函  数：void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)
*功  能：SPI2通信速度设置
*参  数：无
*返回值：无
*备  注：无
*****************************************************************************/
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI_Cmd(SPI2, DISABLE);   //失能SPI2
	SPI2->CR1 &= ~(0x07 << 3);//清除原来的设置
	SPI2->CR1 |= SPI_BaudRatePrescaler;
	SPI_Cmd(SPI2, ENABLE);   //使能SPI2
}

/*****************************************************************************
*函  数：uint8_t SPI2_WriteReadByte(uint8_t data)
*功  能：SPI2读写一个字节
*参  数：无
*返回值：无
*备  注：无
*****************************************************************************/
uint8_t SPI2_WriteReadByte(uint8_t data)
{
	 while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	 SPI_I2S_SendData(SPI2, data);
	 while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	 return SPI_I2S_ReceiveData(SPI2);
}
