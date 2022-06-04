/***************************************************************************************
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：V1.01
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "control.h"
#include "pid.h"
#include "led.h"
#include "motor.h"
#include "math.h"
#include "structconfig.h"
#include "stdio.h"

//角度环PID 
PID_TYPE PID_ROL_Angle;
PID_TYPE PID_PIT_Angle;
PID_TYPE PID_YAW_Angle;
//角速度环PID 
PID_TYPE PID_ROL_Rate;
PID_TYPE PID_PIT_Rate;
PID_TYPE PID_YAW_Rate;
//高度环PID 
PID_TYPE PID_ALT_Rate;
PID_TYPE PID_ALT;

float Pre_THROTTLE,THROTTLE;
float Moto_PWM_1=0.0f,Moto_PWM_2=0.0f,Moto_PWM_3=0.0f,Moto_PWM_4=0.0f;
uint8_t WiFi_Controlflag = 0,Airplane_Enable;

/******************************************************************************************
*函  数：void Control(FLOAT_ANGLE *att_in,FLOAT_XYZ *gyr_in, RC_TYPE *rc_in, uint8_t armed)
*功  能：姿态控制,角度环控制和角速度环控制
*参  数：att_in 测量值
*        gry_in:MPU6050读取的角速度值
*        rc_in:遥控器设定值
*        armed记录命令
*返回值：无
*备  注：DragonFly 小四轴机头与电机示意图	
               机头
          M1    ↑    M2
            \   |   /
						 \  |  /
							\ | /
			Y<————-———+————————		
							/ | \
					   /  |  \
					  /   |   \
					M4    |    M3
						    X
	
	1. M1 M3电机逆时针旋转，M2 M4电机顺时针旋转
	2. X:是MPU9250的 X 轴 ，Y:是MPU9250的 Y 轴
	3. 绕 X 轴旋转为ROLL 角 
	   绕 Y 轴旋转为PITCH角 
		 绕 Z 轴旋转为YAW  角
	4. 自己DIY时进行动力分配可以一个轴一个轴的分配，切勿三个轴同时分配。
*******************************************************************************************/
FLOAT_ANGLE Measure_Angle,Target_Angle;
void Control(FLOAT_ANGLE *att_in,FLOAT_XYZ *gyr_in, RC_TYPE *rc_in, uint8_t armed)
{

	Measure_Angle.rol = att_in->rol; 
	Measure_Angle.pit = att_in->pit; 
	Measure_Angle.yaw = att_in->yaw; 
	Target_Angle.rol = (float)((rc_in->ROLL-1500)/12.0f);
	Target_Angle.pit = (float)((rc_in->PITCH-1500)/12.0f);
	Target_Angle.yaw = (float)((1500-rc_in->YAW)/12.0f); 
  
  if(WiFi_Controlflag)
	{
		Target_Angle.yaw = 0;
	}else
	{
		if(fabs(Target_Angle.yaw )<4)
		{
			Target_Angle.yaw = 0;
		}
	}
	if(1 == (GET_FLAG(FLY_MODE))) //无头模式
	{
	  Yaw_Carefree(&Target_Angle,&Measure_Angle);
	}

	//角度环
	PID_Postion_Cal(&PID_ROL_Angle,Target_Angle.rol,Measure_Angle.rol);//ROLL角度环PID （输入角度 输出角速度）
	PID_Postion_Cal(&PID_PIT_Angle,Target_Angle.pit,Measure_Angle.pit);//PITH角度环PID （输入角度 输出角速度）
//	PID_Postion_Cal(&PID_YAW_Angle,Target_Angle.yaw,Measure_Angle.yaw);//YAW角度环PID （输入角度 输出角速度）
	
  //角速度环
	PID_Postion_Cal(&PID_ROL_Rate,PID_ROL_Angle.OutPut,gyr_in->X*RadtoDeg); //ROLL角速度环PID （输入角度环的输出，输出电机控制量）
	PID_Postion_Cal(&PID_PIT_Rate,PID_PIT_Angle.OutPut,gyr_in->Y*RadtoDeg); //PITH角速度环PID （输入角度环的输出，输出电机控制量）
	PID_Postion_Cal(&PID_YAW_Rate,Target_Angle.yaw*PID_YAW_Angle.P,gyr_in->Z*RadtoDeg); //YAW角速度环PID （输入角度，输出电机控制量）
	
	//动力分配（自己DIY是动力分配一定要好好研究，动力分配搞错飞机肯定飞不起来!!!）
	if(rc_in->THROTTLE>180&&armed)//当油门大于150时和飞机解锁时动力分配才生效
	{                                                                                 
		Moto_PWM_1 = rc_in->THROTTLE + PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;   
		Moto_PWM_2 = rc_in->THROTTLE - PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;   
		Moto_PWM_3 = rc_in->THROTTLE - PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;   
		Moto_PWM_4 = rc_in->THROTTLE + PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;   
	} 
	else
	{
		Moto_PWM_1 = 0;
		Moto_PWM_2 = 0;
		Moto_PWM_3 = 0;
		Moto_PWM_4 = 0;
	}
   Moto_Pwm(Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4); //将此数值分配到定时器，输出对应占空比的PWM波
}

/******************************************************************************************
*函  数：int16_t Yaw_Control(float TARGET_YAW)
*功  能：航向角不回中控制
*参  数：TARGET_YAW 目标航向角
*返回值：针对目标航向角计算出的航向角
*备  注：由于遥控器航向舵会自动回中，所以需要对目标航向角进行不回中处理;
*        由于APP遥控航向舵和油门在一起容易误操作，所以在手机操作时不允许改变航向角;
*******************************************************************************************/
int16_t Yaw_Control(float TARGET_YAW)
{
	static int16_t YAW=0; //根据目标航向角计算出的不回中角度
	if(Airplane_Enable) 
	{
		if(!WiFi_Controlflag) //遥控器控制航向角
		{
			if(TARGET_YAW>2) //目标航向角为正时YAW增大
				YAW +=2;
			if(TARGET_YAW<-2) //目标航向角为负时YAW减小
				YAW -=2;
			
	  }else //APP遥控时，不允许改变航向角
		{
			YAW = YAW;
		}
  }
	return YAW;
}

/******************************************************************************************
*函  数：void Yaw_Carefree(FLOAT_ANGLE *Target_Angle, const FLOAT_ANGLE *Measure_Angle)
*功  能：无头角度控制
*参  数：*Target_Angle 指向目标姿态角的指针
*        *Measure_Angle 测量姿态角的指针
*返回值：针对目标航向角计算出的航向角
*备  注：无头模式需调用此函数
*******************************************************************************************/
void Yaw_Carefree(FLOAT_ANGLE *Target_Angle, const FLOAT_ANGLE *Measure_Angle)
{
	float yawRad = fabs(Measure_Angle->yaw) * DegtoRad;
	float cosy = cosf(yawRad);
	float siny = sinf(yawRad);
	float originalRoll = Target_Angle->rol;
	float originalPitch = Target_Angle->pit;

	Target_Angle->rol = originalRoll * cosy + originalPitch * siny;
	Target_Angle->pit = originalPitch * cosy - originalRoll * siny;
}
/******************************************************************************************
*函  数：void Safety_Check(void)
*功  能：飞机姿态安全监测
*参  数：无
*返回值：无
*备  注：如果飞机角度和加速度异常就将飞机上锁并停止电机，防止电机狂转打坏桨叶
*******************************************************************************************/
void Safety_Check(void)
{
  if((fabs(Att_Angle.pit)>45.0f||fabs(Att_Angle.rol)>45.0f) && (fabs(Acc_filt.X)>9.0f||fabs(Acc_filt.Y)>9.0f))
	{
		 Airplane_Enable = 0;
		 Moto_PWM_1 = 0;
		 Moto_PWM_2 = 0;
		 Moto_PWM_3 = 0;
		 Moto_PWM_4 = 0;
	}
}


//ALT Test
//	if(!Altitude_mode)
//	{
//    THROTTLE = rc_in->THROTTLE; 
//  }
//	else
//	{
//		Altitude = rc_in->THROTTLE/50;
//		if(Altitude < 2) Altitude = 2;
//	}
