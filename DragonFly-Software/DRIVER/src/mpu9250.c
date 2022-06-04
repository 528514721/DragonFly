/***************************************************************************************
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：V1.01
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "MPU9250.h"
#include "iic_moni.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "stdio.h"
#include "structconfig.h"
#include "paramsave.h"
#include "ICM20948.h"


static uint8_t    MPU9250_buff[14];                  //加速度 陀螺仪 温度 原始数据
INT16_XYZ	 GYRO_OFFSET_RAW,ACC_OFFSET_RAW;			     //零漂数据
INT16_XYZ	 MPU9250_ACC_RAW,MPU9250_GYRO_RAW;	     	 //读取值原始数据
uint8_t    SENSER_OFFSET_FLAG;                       //传感器校准标志位

/*****************************************************************************
*函  数：uint8_t MPU9250_WriteByte(uint8_t addr,uint8_t reg,uint8_t data)
*功  能：写一个字节数据到 MPU9250 寄存器
*参  数：reg： 寄存器地址
*        data: 要写入的数据
*返回值：0成功 1失败
*备  注：MPU9250代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU9250_WriteByte(uint8_t reg,uint8_t data)
{
	if(IIC_WriteByteToSlave(MPU9250Addr,reg,data))
	   return 1;
	else
	   return 0;
}
/*****************************************************************************
*函  数：uint8_t MPU9250_ReadByte(uint8_t reg,uint8_t *buf)
*功  能：从指定MPU9250寄存器读取一个字节数据
*参  数：reg： 寄存器地址
*        buf:  读取数据存放的地址
*返回值：1失败 0成功
*备  注：MPU9250代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU9250_ReadByte(uint8_t reg,uint8_t *buf)
{
	if(IIC_ReadByteFromSlave(MPU9250Addr,reg,buf))
	   return 1;
	else
	   return 0;
}

/*****************************************************************************
*函  数：uint8_t MPU9250_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
*功  能：从指定寄存器写入指定长度数据
*参  数：reg：寄存器地址
*        len：写入数据长度 
*        buf: 写入数据存放的地址
*返回值：0成功 1失败
*备  注：MPU9250代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU9250_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_WriteMultByteToSlave(MPU9250Addr,reg,len,buf))
	   return 1;
	else
	   return 0;
}

/*****************************************************************************
*函  数：uint8_t MPU9250_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
*功  能：从指定寄存器读取指定长度数据
*参  数：reg：寄存器地址
*        len：读取数据长度 
*        buf: 读取数据存放的地址
*返回值：0成功 0失败
*备  注：MPU9250代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU9250_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_ReadMultByteFromSlave(MPU9250Addr,reg,len,buf))
	   return 1;
	else
	   return 0;
}

/*============================以上代码移植时需要修改=========================*/

/******************************************************************************
*函  数：uint8_tMPU9250_getDeviceID(void)
*功  能：读取  MPU9250 WHO_AM_I 标识将返回 0x68
*参  数：无
*返回值：返回读取数据
*备  注：无
*******************************************************************************/
uint8_t MPU9250_getDeviceID(void)
{
    uint8_t buf;
	  MPU9250_ReadByte(MPU9250_RA_WHO_AM_I, &buf);
    return buf;
}

/******************************************************************************
*函  数：uint8_tMPU9250_testConnection(void)
*功  能：检测MPU9250 是否已经连接
*参  数：无
*返回值：1已连接 0未链接
*备  注：无
*******************************************************************************/
uint8_t MPU9250_testConnection(void) 
{
   if(MPU9250_getDeviceID() == 0x71)  
   return 1;
   else 
	 return 1;
}

/******************************************************************************
*函  数：void MPU9250_Check()
*功  能：检测IIC总线上的MPU9250是否存在
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
 void MPU9250_Check(void) 
{ 
	while(!MPU9250_testConnection())
	{
		printf("\rMPU9250 no connect...\r\n");
		RGB_LED_green(); //绿灯常亮
	}
} 

/******************************************************************************
*函  数：void MPU9250_AccRead(int16_t *accData)
*功  能：读取加速度的原始数据
*参  数：*accData 原始数据的指针
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_AccRead(int16_t *accData)
{
    uint8_t buf[6];
   	MPU9250_ReadMultBytes(MPU9250_RA_ACCEL_XOUT_H,6,buf);
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

/******************************************************************************
*函  数：void MPU9250_GyroRead(int16_t *gyroData)
*功  能：读取陀螺仪的原始数据
*参  数：*gyroData 原始数据的指针
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
	
	  MPU9250_ReadMultBytes(MPU9250_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]) ;
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]) ;
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]) ;
}

/******************************************************************************
*函  数：void MPU9250_TempRead(float *tempdata)
*功  能：温度值读取
*参  数：*tempdata 温度数据的指针
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_TempRead(float *tempdata)
{
	uint8_t buf[2];
	short data;
	MPU9250_ReadMultBytes(MPU9250_RA_TEMP_OUT_H, 2, buf);
	data = (int16_t)((buf[0] << 8) | buf[1]) ;
	*tempdata = 36.53f + ((float)data/340.0f);
}

/******************************************************************************
*函  数：void MPU9250_Init(void)
*功  能：初始化MPU9250进入工作状态
*参  数：无
*返回值：无
*备  注：DLPF 最好设为采样频率的一半！！！
*******************************************************************************/
void MPU9250_Init(void)
{
    MPU9250_Check(); //检查MPU9250是否连接
	
	  MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x80); //复位MPU9250
    delay_ms(100);
    MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x01); //唤醒MPU9250，并选择陀螺仪x轴PLL为时钟源
	  MPU9250_WriteByte(MPU9250_RA_INT_ENABLE, 0x00); //禁止中断
	  MPU9250_WriteByte(MPU9250_RA_ACCEL_CONFIG_2,0x03); //
	  MPU9250_WriteByte(MPU9250_RA_SMPLRT_DIV, 0x00); //采样分频  （采样频率 = 陀螺仪输出频率 / (1+DIV)，采样频率1000hz）
    MPU9250_WriteByte(MPU9250_RA_INT_PIN_CFG, 0x02);//MPU 可直接访问MPU9250辅助I2C
    MPU9250_WriteByte(MPU9250_RA_CONFIG, MPU9250_DLPF_BW_20);//设置陀螺的输出为1kHZ,DLPF=20Hz  
    MPU9250_WriteByte(MPU9250_RA_GYRO_CONFIG, 0x18); //陀螺仪满量程+-2000度/秒     (最低分辨率 = 2^15/2000 = 16.4LSB/度/秒 
	  MPU9250_WriteByte(MPU9250_RA_ACCEL_CONFIG, 0x08); //加速度满量程+-4g            (最低分辨率 = 2^15/4g = 8196LSB/g )
}

/******************************************************************************
*函  数：void MPU9250_CalOff(void)
*功  能：陀螺仪加速度校准
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_CalOff(void)
{

	 SENSER_FLAG_SET(ACC_OFFSET);//加速度校准
	 SENSER_FLAG_SET(GYRO_OFFSET);//陀螺仪校准
}

/******************************************************************************
*函  数：void MPU9250_CalOff_Acc(void)
*功  能：加速度计校准
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_CalOff_Acc(void)
{
	 SENSER_FLAG_SET(ACC_OFFSET);//加速度校准
}

/******************************************************************************
*函  数：void MPU9250_CalOff_Gyr(void)
*功  能：陀螺仪校准
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_CalOff_Gyr(void)
{
	 SENSER_FLAG_SET(GYRO_OFFSET);//陀螺仪校准
}

/******************************************************************************
*函  数：void MPU9250_Read(void)
*功  能：读取陀螺仪加速度计的原始数据
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_Read(void)
{
	MPU9250_ReadMultBytes(MPU9250_RA_ACCEL_XOUT_H, 14, MPU9250_buff);// 查询法读取MPU9250的原始数据
}

/******************************************************************************
*函  数：uint8_t MPU9250_OffSet(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
*功  能：MPU9250零偏校准
*参  数：value： MPU9250原始数据
*        offset：校准后的零偏值
*        sensivity：加速度计的灵敏度
*返回值：1校准完成 0校准未完成
*备  注：无
*******************************************************************************/
uint8_t MPU9250_OffSet(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
{
	static int32_t tempgx=0,tempgy=0,tempgz=0; 
	static uint16_t cnt_a=0;//使用static修饰的局部变量，表明次变量具有静态存储周期，也就是说该函数执行完后不释放内存
		if(cnt_a==0)
		{
			value.X=0;
			value.Y=0;
			value.Z=0;
			tempgx = 0;
			tempgy = 0;
			tempgz = 0;
			cnt_a = 1;
			sensivity = 0;
			offset->X = 0;
			offset->Y = 0;
			offset->Z = 0;
		}
		tempgx+= 	value.X;
		tempgy+= 	value.Y; 
		tempgz+= 	value.Z-sensivity ;//加速度计校准 sensivity 等于 MPU9250初始化时设置的灵敏度值（8196LSB/g）;陀螺仪校准 sensivity = 0；
		if(cnt_a==200)               //200个数值求平均
		{
			offset->X=tempgx/cnt_a;
			offset->Y=tempgy/cnt_a;
			offset->Z=tempgz/cnt_a;
			cnt_a = 0;
			return 1;
		}
		cnt_a++;
	return 0;
}	

/******************************************************************************
*函  数：void MPU9250_DataProcess(void)
*功  能：对MPU9250进行去零偏处理
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_Offset(void)
{
	
	//加速度去零偏AD值 
	MPU9250_ACC_RAW.X =((((int16_t)MPU9250_buff[0]) << 8) | MPU9250_buff[1]) - ACC_OFFSET_RAW.X;
	MPU9250_ACC_RAW.Y =((((int16_t)MPU9250_buff[2]) << 8) | MPU9250_buff[3]) - ACC_OFFSET_RAW.Y;
	MPU9250_ACC_RAW.Z =((((int16_t)MPU9250_buff[4]) << 8) | MPU9250_buff[5]) - ACC_OFFSET_RAW.Z;
	//陀螺仪去零偏AD值 
	MPU9250_GYRO_RAW.X =((((int16_t)MPU9250_buff[8]) << 8) | MPU9250_buff[9]) - GYRO_OFFSET_RAW.X;
	MPU9250_GYRO_RAW.Y =((((int16_t)MPU9250_buff[10]) << 8) | MPU9250_buff[11]) - GYRO_OFFSET_RAW.Y;
	MPU9250_GYRO_RAW.Z =((((int16_t)MPU9250_buff[12]) << 8) | MPU9250_buff[13]) - GYRO_OFFSET_RAW.Z;
	
	if(GET_FLAG(GYRO_OFFSET)) //陀螺仪进行零偏校准
	{
		if(MPU9250_OffSet(MPU9250_GYRO_RAW,&GYRO_OFFSET_RAW,0))
		{
			
			 SENSER_FLAG_RESET(GYRO_OFFSET);
			 PID_WriteFlash(); //保存陀螺仪的零偏数据
       GYRO_Offset_LED();
		   SENSER_FLAG_SET(ACC_OFFSET);//校准加速度
			
//			 printf("GYRO_OFFSET_RAW Value :X=%d  Y=%d  Z=%d\n",GYRO_OFFSET_RAW.X,GYRO_OFFSET_RAW.Y,GYRO_OFFSET_RAW.Z);
//			 printf("\n");
		}
	}
	if(GET_FLAG(ACC_OFFSET)) //加速度计进行零偏校准 
	{
		if(MPU9250_OffSet(MPU9250_ACC_RAW,&ACC_OFFSET_RAW,8196))
		{
			 SENSER_FLAG_RESET(ACC_OFFSET);
			 PID_WriteFlash(); //保存加速度计的零偏数据
       ACC_Offset_LED();
//			 printf("ACC_OFFSET_RAW Value X=%d  Y=%d  Z=%d\n",ACC_OFFSET_RAW.X,ACC_OFFSET_RAW.Y,ACC_OFFSET_RAW.Z); 
//			 printf("\n");
		}
	}
}
