/***************************************************************************************
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：V1.01
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "stm32f4xx.h"
#include "pid.h"
#include "led.h"
#include "ParamSave.h"
#include "structconfig.h"

/*****************************************************************************
*函  数：void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
*功  能：位置式PID算法
*参  数：PID: 算法P I D参数的结构体
*        target: 目标值
*        measure: 测量值 
*返回值：无 
*备  注: 角度环和角速度环共用此函数
*****************************************************************************/
void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
{
//	int dt = 0;//采样时间(也就是扫描时间10ms)
	PID->Error  = target - measure;              //误差
	PID->Differ = PID->Error - PID->PreError;    //微分量
	
	PID->Pout = PID->P * PID->Error;                        //比例控制
	PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral;  //积分控制
	PID->Dout = PID->D * PID->Differ;                       //微分控制
	
	PID->OutPut =  PID->Pout + PID->Iout + PID->Dout;       //比例 + 积分 + 微分总控制
	
	if(Airplane_Enable == 1&&RC_Control.THROTTLE >= 180)    //飞机解锁之后再加入积分,防止积分过调
	{
		if(measure > (PID->Ilimit)||measure < -PID->Ilimit)   //积分分离
		{PID->Ilimit_flag = 0;}
		else
		{
			PID->Ilimit_flag = 1;                               //加入积分(只有测量值在-PID->Ilimit~PID->Ilimit 范围内时才加入积分)
			PID->Integral += PID->Error;                        //对误差进行积分
			if(PID->Integral > PID->Irang)                      //积分限幅
          PID->Integral = PID->Irang;
		  if(PID->Integral < -PID->Irang)                     //积分限幅
			    PID->Integral = -PID->Irang;                    
		}
	}else
	{PID->Integral = 0;}
	PID->PreError = PID->Error ;                            //前一个误差值
}

/*****************************************************************************
*函  数：void PidParameter_init(void)
*功  能：初始化PID结构体里的一些成员值
*参  数：无
*返回值：无 
*备  注: 由于PID参数我们都保存在FLASH中，所以此函数初始化时不用初始化这些参数，
*        但是Flash中的参数有可能因为误操作被擦除，如果Flash读取参数失败，则就
*        初始化为默认参数。
*****************************************************************************/
void PidParameter_init(void)
{
	
	//ROLL轴
  PID_ROL_Rate.Ilimit_flag = 0; //Roll轴角速度积分的分离标志
	PID_ROL_Rate.Ilimit = 250;    //Roll轴角速度积分范围
	PID_ROL_Rate.Irang = 2000;    //Roll轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
  PID_ROL_Angle.Ilimit_flag = 0;//Roll轴角度积分的分离标志
	PID_ROL_Angle.Ilimit = 35;    //Roll轴角度积分范围
	PID_ROL_Angle.Irang = 1000;    //Roll轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）

	//PITCH轴
	PID_PIT_Rate.Ilimit_flag = 0; //Pitch轴角速度积分的分离标志
	PID_PIT_Rate.Ilimit = 250;    //Pitch轴角速度积分范围
	PID_PIT_Rate.Irang = 2000;    //Pitch轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
  PID_PIT_Angle.Ilimit_flag = 0;//Roll轴角度积分的分离标志
	PID_PIT_Angle.Ilimit = 35;    //Roll轴角度积分范围
	PID_PIT_Angle.Irang = 1000;    //Roll轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
	
	//YAW轴

	PID_YAW_Rate.Ilimit_flag = 0; //Yaw轴角速度积分的分离标志
	PID_YAW_Rate.Ilimit = 150;    //Yaw轴角速度积分范围
	PID_YAW_Rate.Irang = 1200;    //Yaw轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
	PID_YAW_Angle.Ilimit_flag = 0;//Yaw轴角度积分的分离标志
	PID_YAW_Angle.Ilimit = 35;    //Yaw轴角度积分范围
	PID_YAW_Angle.Irang = 200;    //Yaw轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
  

	//高度环
	PID_ALT_Rate.Ilimit_flag = 0;
	PID_ALT_Rate.Ilimit = 0;
	PID_ALT_Rate.Irang = 0;
	PID_ALT.Ilimit_flag = 0;
	PID_ALT.Ilimit = 100;
	PID_ALT.Irang = 200;
}
