/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
 * 作者   ：匿名科创
 * 文件名  ：data_transfer.c
 * 描述    ：数据传输
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "ANO_DT.h"
#include "usart.h"
#include "imu.h"
#include "mpu9250.h"
#include "timer.h"
#include "structconfig.h"
#include "paramsave.h"
#include "filter.h"
#include "fbm320.h"
#include "altitude.h"

//ANO的发送标志的数据结构
typedef struct FLAG_TYPE
{
	uint8_t send_version;
	uint8_t send_status;
	uint8_t send_senser;
	uint8_t send_rcdata;
	uint8_t send_motopwm;
	uint8_t send_power;
	uint8_t send_pid1;
	uint8_t send_pid2;
	uint8_t send_pid3;
	uint8_t send_pid4;
}FLAG_TYPE;
extern FLOAT_ANGLE Measure_Angle,Target_Angle;
/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
u8 flag = 0;
FLAG_TYPE f;					    //需要发送数据的标志
uint8_t data_to_send[50];	//发送数据缓存
/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每5ms发送一次传感器数据至上位机，即在此函数内实现
//此函数应由用户每1ms调用一次
void ANO_DT_Data_Exchange(void)
{
	static uint8_t status_cnt 	= 0;
	static uint8_t senser_cnt 	= 0;
	static uint8_t rcdata_cnt 	= 0;
	static uint8_t motopwm_cnt	= 0;
	static uint8_t power_cnt		=	0;
	status_cnt++;
	senser_cnt++;
  rcdata_cnt++;
	motopwm_cnt++;
	power_cnt++;
	if(status_cnt>=55)
	{
		status_cnt = 0;
		f.send_status = 1;
	}
	if(senser_cnt>=5)
	{
		senser_cnt = 0;
		f.send_senser = 1;
	}
	if(rcdata_cnt>=20)
	{
		rcdata_cnt = 0;
		f.send_rcdata = 1;
	}
	if(motopwm_cnt>=35)
	{
		motopwm_cnt = 0;
		f.send_motopwm = 1;
	}
	if(power_cnt>=60)
	{
		power_cnt = 0;
		f.send_power = 1;
	}

/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_status)
	{
		f.send_status = 0;
	  ANO_DT_Send_Status(Att_Angle.rol,Att_Angle.pit,Att_Angle.yaw,height,0,Airplane_Enable);//fly_ready
	}	
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_senser)
	{
		f.send_senser = 0;
		ANO_DT_Send_Senser((s16)Acc_filt.X,(s16)Acc_filt.Y,(s16)Acc_filt.Z,
											 (s16)Gyr_rad.X*RadtoDeg,(s16)Gyr_rad.Y*RadtoDeg,(s16)Gyr_rad.Z*RadtoDeg,0,0,0,0);
		#if defined (ROL_PID_DEBUG)   //ROLL角调试
			Data_Send_AngleRate(Gyr_rad.X*RadtoDeg,PID_ROL_Rate.Pout,PID_ROL_Rate.Iout,PID_ROL_Rate.Dout,
		                       PID_ROL_Angle.Error,PID_ROL_Angle.Pout,PID_ROL_Angle.Iout,PID_ROL_Rate.OutPut);
		
		#elif defined (PIT_PID_DEBUG) //PITCH角调试	
			Data_Send_AngleRate(Gyr_rad.Y*RadtoDeg,PID_PIT_Rate.Pout,PID_PIT_Rate.Iout,PID_PIT_Rate.Dout,
		                       Target_Angle.pit,Measure_Angle.pit,PID_PIT_Angle.OutPut,PID_PIT_Rate.OutPut);
		
		#elif defined (YAW_PID_DEBUG) //YAW角调试
			Data_Send_AngleRate(Gyr_rad.Z*RadtoDeg,PID_YAW_Rate.Pout,PID_YAW_Rate.Iout,PID_YAW_Rate.Dout,
													PID_YAW_Angle.Error,PID_YAW_Angle.Pout,PID_YAW_Angle.Iout,PID_YAW_Angle.Dout);
		
		#elif defined (ALT_PID_DEBUGE)//ALT环调试
			Data_Send_AngleRate(FBM.AltitudeFilter,PID_ALT.Error,PID_ALT.Pout,PID_ALT.Iout,PID_ALT.Dout,0,0,0);
		#else
			 Data_Send_Filter();
		#endif
	}	 
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_rcdata)
	{
		f.send_rcdata = 0;
	//	ANO_DT_Send_RCData(RC_Control.THROTTLE,RC_Control.YAW,RC_Control.ROLL,RC_Control.PITCH,0,0,0,0,0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////	
	if(f.send_motopwm)
	{
		f.send_motopwm = 0;
		//ANO_DT_Send_MotoPWM(Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4,5,6,7,8);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_power)
	{
		f.send_power = 0;
		ANO_DT_Send_Power((u16)(BAT.BattMeasureV*100),456);
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_pid1)
	{
		f.send_pid1 = 0;
		ANO_DT_Send_PID(1,PID_ROL_Rate.P,PID_ROL_Rate.I,PID_ROL_Rate.D,
		                  PID_PIT_Rate.P,PID_PIT_Rate.I,PID_PIT_Rate.D,
		                  PID_YAW_Rate.P,PID_YAW_Rate.I,PID_YAW_Rate.D);

	}	
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_pid2)
	{
		f.send_pid2 = 0;
		ANO_DT_Send_PID(2,PID_ROL_Angle.P,PID_ROL_Angle.I,PID_ROL_Angle.D,
	                   	PID_PIT_Angle.P,PID_PIT_Angle.I,PID_PIT_Angle.D,
		                  PID_YAW_Angle.P,PID_YAW_Angle.I,PID_YAW_Angle.D);
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_pid3)
	{
		f.send_pid3 = 0;
	  ANO_DT_Send_PID(3,PID_ALT_Rate.P,PID_ALT_Rate.I,PID_ALT_Rate.D,
		                  PID_ALT.P,PID_ALT.I,PID_ALT.D,0,0,0);
		
		
	}
	
	if(f.send_pid4)//清除上位机PID数据显示
	{
		f.send_pid4 = 0;
		ANO_DT_Send_PID(1,0.93,0.005,0.86,0.93,0.005,0.86,2,0.05,1.0);
		ANO_DT_Send_PID(2,2.3,0.01,0.05,2.3,0.01,0.05,3.5,0,1.0);
		ANO_DT_Send_PID(3,0,0,0,0,0,0,0,0,0);
	}

}

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length)
{
	Usart_Send(dataToSend, length);
}

static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum)
{
	uint8_t sum = 0,i;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	for(i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(uint8_t data)
{
	static uint8_t RxBuffer[50];
	static uint8_t _data_len = 0,_data_cnt = 0;
	static uint8_t state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num)
{
	uint8_t sum = 0,i;
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
		{
			SENSER_FLAG_SET(ACC_OFFSET);//加速度校准
			ACC_OFFSET_RAW.X = 0;ACC_OFFSET_RAW.Y = 0;ACC_OFFSET_RAW.Z = 0;
		}
		if(*(data_buf+4)==0X02)
		{
			SENSER_FLAG_SET(GYRO_OFFSET);//陀螺仪校准
			GYRO_OFFSET_RAW.X = 0;GYRO_OFFSET_RAW.Y = 0;GYRO_OFFSET_RAW.Z = 0;
		}
		if(*(data_buf+4)==0X03)
		{
			SENSER_FLAG_SET(ACC_OFFSET);//加速度校准
			SENSER_FLAG_SET(GYRO_OFFSET);//陀螺仪校准
		}
		if(*(data_buf+4)==0X04)
		{
			flag ^= 0x01;
			if(flag)
				Airplane_Enable = 1;
			else
				Airplane_Enable = 0;
		}
		PID_WriteFlash(); //清除原来零偏
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;//读取PID1
			f.send_pid2 = 1;//读取PID2
			f.send_pid3 = 1;//读取PID3
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
			//Para_ResetToFactorySetup();
			f.send_pid4 = 1;        //参数清零
		}
	}

	if(*(data_buf+2)==0X10)								//PID1
    {
        PID_ROL_Rate.P = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        PID_ROL_Rate.I = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        PID_ROL_Rate.D = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        PID_PIT_Rate.P = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        PID_PIT_Rate.I = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        PID_PIT_Rate.D = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        PID_YAW_Rate.P = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        PID_YAW_Rate.I = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        PID_YAW_Rate.D = 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
		    PID_WriteFlash();
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
        PID_ROL_Angle.P 	= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        PID_ROL_Angle.I 	= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        PID_ROL_Angle.D 	= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        PID_PIT_Angle.P 	= 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        PID_PIT_Angle.I 	= 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        PID_PIT_Angle.D 	= 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        PID_YAW_Angle.P	  = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        PID_YAW_Angle.I 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        PID_YAW_Angle.D 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
			 
        ANO_DT_Send_Check(*(data_buf+2),sum);
				PID_WriteFlash();
    }
  if(*(data_buf+2)==0X12)								//PID3
  {	
		PID_ALT_Rate.P 	= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		PID_ALT_Rate.I 	= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		PID_ALT_Rate.D 	= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		PID_ALT.P 	= 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		PID_ALT.I 	= 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		PID_ALT.D 	= 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//		PID_ALT.P	  = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//		PID_ALT.I 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//		PID_ALT.D 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
    ANO_DT_Send_Check(*(data_buf+2),sum);
		PID_WriteFlash();
  }
	if(*(data_buf+2)==0X13)								//PID4
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
}

//void ANO_DT_Send_Version(uint8_t hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
//{
//	uint8_t _cnt=0;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x00;
//	data_to_send[_cnt++]=0;
//	
//	data_to_send[_cnt++]=hardware_type;
//	data_to_send[_cnt++]=BYTE1(hardware_ver);
//	data_to_send[_cnt++]=BYTE0(hardware_ver);
//	data_to_send[_cnt++]=BYTE1(software_ver);
//	data_to_send[_cnt++]=BYTE0(software_ver);
//	data_to_send[_cnt++]=BYTE1(protocol_ver);
//	data_to_send[_cnt++]=BYTE0(protocol_ver);
//	data_to_send[_cnt++]=BYTE1(bootloader_ver);
//	data_to_send[_cnt++]=BYTE0(bootloader_ver);
//	
//	data_to_send[3] = _cnt-4;
//	
//	uint8_t sum = 0;
//	for(uint8_t i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	data_to_send[_cnt++]=sum;
//	
//	ANO_DT_Send_Data(data_to_send, _cnt);
//}
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, uint8_t FLY_ENABLEl, uint8_t armed)
{
	uint8_t _cnt=0,sum = 0,i;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;    
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;   
	data_to_send[_cnt++]=0;      
	
	_temp = (int)(angle_rol*100); 
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = FLY_ENABLEl;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,s32 bar)
{
	uint8_t _cnt=0,sum = 0,i;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	uint8_t _cnt=0,sum = 0,i;

	data_to_send[_cnt++]=0xAA;         //帧头
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;         //功能字
	data_to_send[_cnt++]=0;            //数据长度
	data_to_send[_cnt++]=BYTE1(thr);   //油门
	data_to_send[_cnt++]=BYTE0(thr);   
	data_to_send[_cnt++]=BYTE1(yaw);   //航向角
	data_to_send[_cnt++]=BYTE0(yaw);   
	data_to_send[_cnt++]=BYTE1(rol);   //横滚
	data_to_send[_cnt++]=BYTE0(rol); 
	data_to_send[_cnt++]=BYTE1(pit);   //俯仰
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;         //数据帧长度赋值
	
	for(i=0;i<_cnt;i++)             
		sum += data_to_send[i];         //数据校验求解
	
	data_to_send[_cnt++]=sum;         //数据校验赋值
	
	ANO_DT_Send_Data(data_to_send, _cnt); //帧发送
}
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	uint8_t _cnt=0,sum = 0,i;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
  
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	uint8_t _cnt=0,	sum = 0,i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	uint8_t _cnt=0,sum = 0,i;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

/***************************************自定义帧**********************************************/
//角速度环调试,波形显示
void Data_Send_AngleRate(float data1,float data2,float data3,float data4,float data5,float data6,float data7,float data8)
{
	u8 _cnt=0,sum = 0,i;
	float _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; 
	data_to_send[_cnt++]=0;
	
	_temp = data1;//RadtoDeg
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data2;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data3;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data4;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data5;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data6;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data7;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data8;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	

	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}


////滤波效果观察函数调试
void Data_Send_Filter(void)
{
	u8 _cnt=0,sum = 0,i;
	float _temp;
	vs16 temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF2;
	data_to_send[_cnt++]=0;

	_temp = FBM.Altitude;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = nav.z;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = nav.vz;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
   _temp = nav.az;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp = Acc_filt.Z;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	//_temp = Gyr_rad.X*RadtoDeg;
	_temp =FBM.AltitudeFilter;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	//_temp = Gyr_rad.Y*RadtoDeg;
	_temp = (float)MPU9250_ACC_RAW.Y*0.011964f;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	//_temp = Gyr_rad.Z*RadtoDeg;
	_temp = (float)MPU9250_ACC_RAW.Z*0.011964f;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

