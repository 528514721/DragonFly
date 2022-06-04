/***************************************************************************************
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：V1.01
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "main.h"


int main(void)
{
	NVIV_Config();
	LED_Init();
	Delay_Init();
	Usart1_Init(460800);
	Usart2_Init(921600);
	IIC_Init();
	TIM4_Init();
	Exti_Init(); //外部中断初始化
	NRF24l01_Init(); //NRF初始化（红）
	MPU9250_Init(); //MPU9250初始化（绿） 
//	spl06_init(); //SPL06初始化(气压计蓝) 
//	ICM_PowerOn();
//	FBM320_Init(); //FBM320初始化(气压计蓝) 
	MOTOR_Init(); //电机输出初始 	化
	BATT_Init(); //电池电压检测初始化
	WiFi_Switch(DISABLE); //WiFi模块开关
	OpenMV_Switch(DISABLE); //OpenMV模块开关
	PID_ReadFlash(); //Flash中的数据读取
	PidParameter_init(); //PID初始化
	RGB_LED_Off();
	while(1)
	{
		if(ANO_Scan) //500Hz
		{
			ANO_Scan = 0;
			ANO_DT_Data_Exchange(); //更新数据到上位机
		}
		if(IMU_Scan) //100Hz
		{
			IMU_Scan  = 0;
			Prepare_Data(); //获取姿态解算所需数据
			IMUupdate(&Gyr_rad,&Acc_filt,&Att_Angle); //四元数姿态解算
			Control(&Att_Angle,&Gyr_rad,&RC_Control,Airplane_Enable); //姿态控制
			RunTimer_Test();
			
			spl06_update();
			altitude_get();
			
			
		}
		if(LED_Scan) //10Hz
		{
			LED_Scan = 0;
			LED_Run();
			if(!Airplane_Enable&&Run_flag&&!WiFi_LEDflag)
			{
				RGB_LED_Runing(); //飞机上锁状态灯
			}
			WiFi_OFFON_LED(); //WiFi开关状态灯 
			BATT_Alarm_LED(); //电池低电压报警
			  
		}
		if(IRQ_Scan) //5Hz
		{
			IRQ_Scan = 0;
			NRF_SingalCheck(); //NRF通信检测
			SendToRemote(); //发送数据给遥控器
		}
		if(Batt_Scan) //2.5Hz
		{
			Batt_Scan = 0;
			NRF_GetAddr(); //分配NRF地址
			LowVoltage_Alarm();
		}
	}
}
