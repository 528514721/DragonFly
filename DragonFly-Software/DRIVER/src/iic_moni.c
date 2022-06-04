/***************************************************************************************
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：V1.01
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "iic_moni.h"
#include "delay.h"
#include "usart.h"
#include "stdio.h"
#include "math.h"

/************************代码移植修改区************************************/

#define RCC_IIC_SCL    RCC_AHB1Periph_GPIOB  //端口时钟
#define IIC_SCL_PORT   GPIOB                 //端口
#define IIC_SCL        GPIO_Pin_6            //引脚

#define RCC_IIC_SDA    RCC_AHB1Periph_GPIOB  //端口时钟
#define IIC_SDA_PORT   GPIOB                 //端口
#define IIC_SDA        GPIO_Pin_7            //引脚

/**************************************************************************/

//求偏移量
#define IIC_SCL_OFFSET (uint8_t)(log(IIC_SCL)/log(2))
#define IIC_SDA_OFFSET (uint8_t)(log(IIC_SDA)/log(2))

//SCL-->PB6
//SDA-->PB7
#define SDA_IN()  {IIC_SDA_PORT->MODER&=~(3<<(IIC_SDA_OFFSET*2));IIC_SDA_PORT->MODER|=0<<IIC_SDA_OFFSET*2;}	
#define SDA_OUT() {IIC_SDA_PORT->MODER&=~(3<<(IIC_SDA_OFFSET*2));IIC_SDA_PORT->MODER|=1<<IIC_SDA_OFFSET*2;} 

//IO操作函数	 
#define IIC_SCL_H    IIC_SCL_PORT->BSRRL |= IIC_SCL //SCL
#define IIC_SCL_L    IIC_SCL_PORT->BSRRH |= IIC_SCL //SCL
#define IIC_SDA_H    IIC_SDA_PORT->BSRRL |= IIC_SDA //SDA	 
#define IIC_SDA_L    IIC_SDA_PORT->BSRRH |= IIC_SDA //SDA	 
#define READ_SDA    (IIC_SDA_PORT->IDR&IIC_SDA) ? 1 : 0 //输入SDA 

/******************************************************************************
*函  数：void IIC_Delay(void)
*功　能：IIC延时
*参  数：无
*返回值：无
*备  注: 移植时只需要将delay_us()换成自己的延时即可
*******************************************************************************/	
void IIC_Delay(void)
{
	delay_us(4);
}

/******************************************************************************
*函  数：void IIC_Init(void)
*功　能：IIC初始化
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/	
void IIC_Init(void)
{			
  GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_IIC_SDA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_IIC_SCL, ENABLE);

  GPIO_InitStructure.GPIO_Pin = IIC_SCL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //选择模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //开漏输出类型  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(IIC_SCL_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = IIC_SDA;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //选择模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //开漏输出类型  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //仅仅对配置成输入时有效
  GPIO_Init(IIC_SDA_PORT, &GPIO_InitStructure);
	
	IIC_SCL_H;
	IIC_SDA_H;
}
/******************************************************************************
*函  数：void IIC_Start(void)
*功　能：产生IIC起始信号
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/	
void IIC_Start(void)
{
	SDA_OUT(); //sda线输出 
	IIC_SDA_H;	
	IIC_SCL_H;
	delay_us(4);
 	IIC_SDA_L; //START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_L; //钳住I2C总线，准备发送或接收数据 
}

/******************************************************************************
*函  数：void IIC_Stop(void)
*功　能：产生IIC停止信号
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT(); //sda线输出
	IIC_SCL_L;
	IIC_SDA_L; //STOP:when CLK is high DATA change form low to high
  delay_us(4);
	IIC_SCL_H; 
	IIC_SDA_H; //发送I2C总线结束信号
  delay_us(4);							   	
}

/******************************************************************************
*函  数: uint8_t IIC_WaitAck(void)
*功　能: 等待应答信号到来 （有效应答：从机第9个 SCL=0 时 SDA 被从机拉低,
                            并且 SCL = 1时 SDA依然为低）
*参  数：无
*返回值：1，接收应答失败
         0，接收应答成功
*备  注：从机给主机的应答
*******************************************************************************/
uint8_t IIC_WaitAck(void)
{
	uint8_t ucErrTime=0;
	SDA_IN(); //SDA设置为输入  （从机给一个低电平做为应答） 
	IIC_SDA_H;delay_us(1);	   
	IIC_SCL_H;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L; //时钟输出0 	   
	return 0;  
} 

/******************************************************************************
*函  数: void IIC_Ack(void)
*功　能: 产生ACK应答 （主机接收完一个字节数据后，主机产生的ACK通知从机一个
                       字节数据已正确接收）
*参  数：无
*返回值：无
*备  注：主机给从机的应答
*******************************************************************************/

void IIC_Ack(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_L;
	delay_us(1);
	IIC_SCL_H;
	delay_us(2);
	IIC_SCL_L;
}

/******************************************************************************
*函  数: void IIC_NAck(void)
*功　能: 产生NACK应答 （主机接收完最后一个字节数据后，主机产生的NACK通知从机
                        发送结束，释放SDA,以便主机产生停止信号）
*参  数：无
*返回值：无
*备  注：主机给从机的应答
*******************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_H;
	delay_us(1);
	IIC_SCL_H;
	delay_us(1);
	IIC_SCL_L;
}					 				     

/******************************************************************************
*函  数：void IIC_SendByte(uint8_t txd)
*功  能：IIC发送一个字节
*参  数：data 要写的数据
*返回值：无
*备  注：主机往从机发
*******************************************************************************/		  
void IIC_SendByte(uint8_t data)
{                        
    uint8_t t;   
	  SDA_OUT(); 	    
    IIC_SCL_L; //拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
      if((data&0x80)>>7)
				IIC_SDA_H;
			else
				IIC_SDA_L;
      data<<=1;
			delay_us(1);			
		  IIC_SCL_H;
		  delay_us(1);
			IIC_SCL_L;	
		  delay_us(1);
    }	 
} 	 
   
/******************************************************************************
*函  数：uint8_t IIC_ReadByte(uint8_t ack)
*功  能：IIC读取一个字节
*参  数：ack=1 时，主机数据还没接收完 ack=0 时主机数据已全部接收完成
*返回值：无
*备  注：从机往主机发
*******************************************************************************/	
uint8_t IIC_ReadByte(uint8_t ack)
{
	uint8_t i,receive=0;
	SDA_IN(); //SDA设置为输入模式 等待接收从机返回数据
  for(i=0;i<8;i++ )
	{
     IIC_SCL_L; 
     delay_us(1);
		 IIC_SCL_H;
     receive<<=1;
     if(READ_SDA)receive++; //从机发送的电平
	   delay_us(1); 
   }					 
    if(ack)
        IIC_Ack(); //发送ACK 
    else
        IIC_NAck(); //发送nACK  
    return receive;
}

/******************************************************************************
*函  数：uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t addr)
*功　能：读取指定设备 指定寄存器的一个值
*参  数：I2C_Addr  目标设备地址
		     reg	     寄存器地址
         *buf      读取数据要存储的地址    
*返回值：返回 1失败 0成功
*备  注：无
*******************************************************************************/ 
uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
	IIC_Start();	
	IIC_SendByte(I2C_Addr<<1|0);	 //发送从机地址
	if(IIC_WaitAck()) //如果从机未应答则数据发送失败
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg); //发送寄存器地址
	IIC_WaitAck();	  
	
	IIC_Start();
	IIC_SendByte(I2C_Addr<<1|1); //进入接收模式			   
	IIC_WaitAck();
	*buf=IIC_ReadByte(0);	   
  IIC_Stop(); //产生一个停止条件
	return 0;
}

/******************************************************************************
*函  数：uint8_t IIC_WriteByteFromSlave(uint8_t I2C_Addr,uint8_t addr，uint8_t buf))
*功　能：写入指定设备 指定寄存器的一个值
*参  数：I2C_Addr  目标设备地址
		     reg	     寄存器地址
         buf       要写入的数据
*返回值：1 失败 0成功
*备  注：无
*******************************************************************************/ 
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
	IIC_Start();
	IIC_SendByte(I2C_Addr<<1|0); //发送从机地址
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1; //从机地址写入失败
	}
	IIC_SendByte(reg); //发送寄存器地址
  IIC_WaitAck();	  
	IIC_SendByte(data); 
	if(IIC_WaitAck())
	{
		IIC_Stop(); 
		return 1; //数据写入失败
	}
	IIC_Stop(); //产生一个停止条件

  //return 1; //status == 0;
	return 0;
}

/******************************************************************************
*函  数：uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*功　能：读取指定设备 指定寄存器的 length个值
*参  数：dev     目标设备地址
		     reg	   寄存器地址
         length  要读的字节数
		     *data   读出的数据将要存放的指针
*返回值：1成功 0失败
*备  注：无
*******************************************************************************/ 
uint8_t IIC_ReadMultByteFromSlave(uint8_t I2C_Addr, uint8_t reg, uint8_t length, uint8_t *data)
{
  uint8_t count = 0;
	uint8_t temp;
	IIC_Start();
	IIC_SendByte(I2C_Addr<<1|0); //发送从机地址
	if(IIC_WaitAck())
	{
		IIC_Stop(); 
		return 1; //从机地址写入失败
	}
	IIC_SendByte(reg); //发送寄存器地址
  IIC_WaitAck();	  
	IIC_Start();
	IIC_SendByte(I2C_Addr<<1|1); //进入接收模式	
	IIC_WaitAck();
  for(count=0;count<length;count++)
	{
		if(count!=(length-1))
		temp = IIC_ReadByte(1); //带ACK的读数据
		else  
		temp = IIC_ReadByte(0); //最后一个字节NACK

		data[count] = temp;
	}
    IIC_Stop(); //产生一个停止条件
    //return count;
	 return 0;
}

/******************************************************************************
*函  数：uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*功　能：将多个字节写入指定设备 指定寄存器
*参  数：dev     目标设备地址
		     reg	   寄存器地址
         length  要写的字节数
		     *data   要写入的数据将要存放的指针
*返回值：1成功 0失败
*备  注：无
*******************************************************************************/ 
uint8_t IIC_WriteMultByteToSlave(uint8_t I2C_Addr, uint8_t reg, uint8_t length, uint8_t* data)
{
  
 	uint8_t count = 0;
	IIC_Start();
	IIC_SendByte(I2C_Addr<<1|0); //发送从机地址
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1; //从机地址写入失败
	}
	IIC_SendByte(reg); //发送寄存器地址
  IIC_WaitAck();	  
	for(count=0;count<length;count++)
	{
		IIC_SendByte(data[count]); 
		if(IIC_WaitAck()) //每一个字节都要等从机应答
		{
			IIC_Stop();
			return 1; //数据写入失败
		}
	}
	IIC_Stop(); //产生一个停止条件

	return 0;
}
