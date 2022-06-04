/***************************************************************************************
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：V1.01
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "altitude.h"
#include "fbm320.h"
#include "structconfig.h"
#include "imu.h"
#include "pid.h"
#include "control.h"
#include "spl06.h"
#include <math.h>

extern uint8_t AccbUpdate;
uint8_t Altitude_mode = 0;
float Altitude = 2;
float height;
nav_t nav;		    //NED frame in earth
float z_est[3];	  // estimate z Vz  Az
static float w_z_baro=10.0f;
static float w_z_acc=30.0f;
static float w_acc_bias=0.05f;

/* acceleration in NED frame */
float accel_NED[3] = { 0.0f, 0.0f, -G };
/* store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
float corr_acc[] = { 0.0f, 0.0f, 0.0f };	// N E D ,  m/s2
float acc_bias[] = { 0.0f, 0.0f, 0.0f };	// body frame ,
float corr_baro = 0.0f;					    //m（气压计矫正系数）



static void inertial_filter_predict(float dt, float x[2],float acc);
static void inertial_filter_correct(float e, float dt, float x[3], int i, float w);
	
void Altitude_Combine(void)
{
	uint8_t i,j;
	float dt = 0.005; //次程序没次被调用的时间间隔
	float accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f }; /* 加速度计的偏移矫正 */
	
	if(AccbUpdate)
	{
		accb[0] -= acc_bias[0];
		accb[1] -= acc_bias[1];
		accb[2] -= acc_bias[2];
		
		 for(i=0; i<3; i++)
		 {
				accel_NED[i]=0.0f;
				for(j=0; j<3; j++)
				{
					accel_NED[i] += DCMgb[j][i]* accb[j];
				}
		 }
			accel_NED[2]=-accel_NED[2];
			corr_acc[2] = accel_NED[2] + G - z_est[2]; //corr_acc[] 机身坐标系 转换到 地理坐标系的加速度
		  AccbUpdate = 0;
  }
	 
	 if(ALT_Updated)
	 {
		 corr_baro = 0 - FBM.AltitudeFilter - z_est[0];    //初始化值为零 测量值与估计值求差
		 ALT_Updated = 0;
	 }
	 
	  accel_bias_corr[2] -= corr_baro * w_z_baro * w_z_baro;
	 
	 if(accel_bias_corr[2])
	 {
	  for (i = 0; i < 3; i++)
    {
       float c = 0.0f;
       for (j = 0; j < 3; j++) 
			 {
            c += DCMgb[i][j] * accel_bias_corr[j];
       }
        acc_bias[i] += c * w_acc_bias * dt;		//accumulate bias偏差积分（acc_bias[i] 较正后的机体系下的加速度）
    }
    acc_bias[2]=-acc_bias[2];
		
		/* 惯性滤波器的位移与速度预测 */
    inertial_filter_predict(dt, z_est,z_est[2]);
    /* 高度惯性滤波器的较正 */
    inertial_filter_correct(corr_baro, dt, z_est, 0, w_z_baro);   	//0.5f (corr_baro气压计较正系数 ;  z_est预测的Z轴数据 z , vz , az)
    inertial_filter_correct(corr_acc[2], dt, z_est, 2, w_z_acc);		//20.0f(corr_acc[] 机身坐标系 转换到 地理坐标系的加速度值 ;  z_est预测的Z轴数据 z , vz , az)
	}
		nav.z=z_est[0];
    nav.vz=z_est[1];
    nav.az=z_est[2];
		

	//	printf("nav.z:%0.2f  nav.vz:%0.2f  nav.az:%0.2f  Altiude:%0.2f\r\n",nav.z,nav.vz,nav.az,FBM.Altitude);
}

/* 惯性滤波器的位移与速度预测 */
static void inertial_filter_predict(float dt, float x[2],float acc)
{
    x[0] += x[1] * dt + acc * dt * dt / 2.0f; // 位移 S = Vt + At^2/2 
    x[1] += acc * dt;                         // 速度 V = At
}

static void inertial_filter_correct(float e, float dt, float x[3], int i, float w)
{
    float ewdt = e * w * dt;
    x[i] += ewdt;

    if (i == 0) 
		{
        x[1] += w * ewdt;
        x[2] += w * w * ewdt / 3.0f;

     }else if (i == 1) {
        x[2] += w * ewdt;
    }
}

void Altitude_Control(void)
{
	PID_Postion_Cal(&PID_ALT,Altitude,FBM.AltitudeFilter);
	
	if(Altitude_mode && RC_Control.THROTTLE>200)
	THROTTLE = 480 + PID_ALT.OutPut;
}





void altitude_get(void)
{
	float __height;
//		altitude = ( 101400 - press ) / 1000.0f;
//    height = 0.82f * altitude * altitude * altitude + 0.09f * ( 101400 - press ) * 100.0f ;
	
	__height = 44330.f * (powf((1015.7f / (press/100.0f)), 0.190295f) - 1.0f);
	height = (int32_t)(__height*100);

//	printf("height = %d\r\n", (int32_t)(height*100));

//    alt_high = ( height - baro_Offset ) ; //cm +
}
