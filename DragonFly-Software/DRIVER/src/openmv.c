/***************************************************************************************
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：V1.01
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "openmv.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "structconfig.h"
#include "imu.h"
#include "math.h"
#include "pid.h"

#define YAnglehalf 20.97f*DegtoRad       //摄像头上下角度的一半       
#define XAnglehalf 27.62f*DegtoRad       //摄像头左右角度的一半     
#define Xmidpoint  79             //摄像头X轴坐标的中点坐标(158,0)
#define Ymidpoint  59             //摄像头Y轴坐标的中点坐标(0,118)

static uint8_t MVx,MVy,Pre_MVy,Pre_MVx;
uint8_t MV_flag = 0;
float Position_x=0,Position_y=0;
/************************************请注意！！！******************************************
1.此文件简单实现了黄色小球色块识别及跟踪。
2.Openmv 将视野内小球的位置坐标通过串口发送给MCU,然后通过Openmv_Data_ReceiveAnalysis();接收
坐标数据，然后再通过void Position_Calculate(void)；计算飞机与小球的相对位置（不是距离）
******************************************************************************************/


/******************************************************************************************
*函  数：void Openmv_Data_ReceiveAnalysis(uint8_t*buff,uint8_t cnt)
*功  能：接收Opemv串口发过来的数据
*参  数：*buff 要接收数据的地址
*         cnt  要接收的数据长度
*返回值：无
*备  注：测试函数
*******************************************************************************************/
void Openmv_Data_ReceiveAnalysis(uint8_t*buff,uint8_t cnt)
{
  if(buff[0]==0xaa)
	{
		MVx = buff[1];
		MVy = buff[2];
		MV_flag = 1;	
	}
}

/******************************************************************************************
*函  数：void Position_Calculate(void)
*功  能：计算出小球相对于飞机的位置坐标
*参  数：无
*返回值：无
*备  注：测试函数
*******************************************************************************************/
void Position_Calculate(void)
{
	double Ax,Bx,Ay,By;
	
	Ax = -1*Att_Angle.rol*DegtoRad;
	Bx = (double)(XAnglehalf-Ax);
	Position_x = MVx - (Xmidpoint * tan(Bx))/(tan(Bx)+tan(Ax));
  //printf("Position_x:%0.2f,  MVx=%d\r\n",Position_x,MVx);
	
	Ay =  Att_Angle.pit*DegtoRad;
	By = (double)(YAnglehalf-Ay);
	Position_y = (Ymidpoint * tan(By))/(tan(By)+tan(Ay))-MVy;
	//printf("Position_y:%0.2f,  MVy=%d\r\n",Position_y,MVy);
	if(Pre_MVy == MVy || Pre_MVx == MVx) //如果小球不在OpenMV的视野内，就把数据置零
	{
		MVx = 0;
		MVy = 0;
		Position_x = 0;
		Position_y = 0;
	}
	
	Pre_MVx = MVx;
	Pre_MVy = MVy;
}

///******************************************************************************************
//*函  数：void Follow_Ball(FLOAT_ANGLE Measure_Angle)
//*功  能：实现小球跟随
//*参  数：无
//*返回值：无
//*备  注：测试函数
//*******************************************************************************************/
//void Follow_Ball(FLOAT_ANGLE Measure_Angle)
//{
//	FLOAT_ANGLE Target_Angle;
//	
//	Target_Angle.rol = (float)Position_x/2;
//	Target_Angle.pit = (float)Position_y/2;
////	printf("roll:%0.2f pit:%0.2f\r\n",Target_Angle.rol,Target_Angle.pit);
//	if(!Target_Angle.rol && !Target_Angle.pit && MV_flag)
//	{
//	 MV_flag = 0;
//   PID_Postion_Cal(&PID_ROL_Angle,Target_Angle.rol,Measure_Angle.rol);//ROLL角度环PID （输入角度 输出角速度）
//	 PID_Postion_Cal(&PID_PIT_Angle,Target_Angle.pit,Measure_Angle.pit);//PITH角度环PID （输入角度 输出角速度）
//	}
//	else
//	{
//	 PID_Postion_Cal(&PID_ROL_Angle,Target_Angle.rol,Measure_Angle.rol);//ROLL角度环PID （输入角度 输出角速度）
//	 PID_Postion_Cal(&PID_PIT_Angle,Target_Angle.pit,Measure_Angle.pit);//PITH角度环PID （输入角度 输出角速度）
//	}
//	
//}
