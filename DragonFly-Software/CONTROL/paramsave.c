/***************************************************************************************
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：V1.01
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "paramsave.h"
#include "flash.h"
#include "pid.h"
#include "usart.h"

#define INT160(dwTemp)       (*(int16_t*)(&dwTemp))
#define INT161(dwTemp)       (*((int16_t *)(&dwTemp) + 1))
#define INT162(dwTemp)       (*((int16_t *)(&dwTemp) + 2))
#define INT163(dwTemp)       (*((int16_t *)(&dwTemp) + 3))

PID_SAVE PIDflash;
/******************************************************************************************
*函  数：void ParamsToTable(void)
*功  能：重要参数赋值给Flash的存储结构体
*参  数：无
*返回值：无
*备  注：由于Flash的存储的结构体为u16，但是PID参数为float保留三位精度
         因此PID参数都乘以1000然后强制转换为u16类型
*******************************************************************************************/
void ParamsToTable(void)
{
	//零偏数据
	PIDflash.ACC_OFFSET_X = ACC_OFFSET_RAW.X;
	PIDflash.ACC_OFFSET_Y = ACC_OFFSET_RAW.Y;
	PIDflash.ACC_OFFSET_Z = ACC_OFFSET_RAW.Z; 
	PIDflash.GYRO_OFFSET_X = GYRO_OFFSET_RAW.X;
	PIDflash.GYRO_OFFSET_Y = GYRO_OFFSET_RAW.Y;
	PIDflash.GYRO_OFFSET_Z = GYRO_OFFSET_RAW.Z;
	
	//角度环数据PID参数
	PIDflash.ROL_Angle_P = (u16)(PID_ROL_Angle.P*1000);
	PIDflash.ROL_Angle_I = (u16)(PID_ROL_Angle.I*1000);
	PIDflash.ROL_Angle_D = (u16)(PID_ROL_Angle.D*1000);
	PIDflash.PIT_Angle_P = (u16)(PID_PIT_Angle.P*1000);
	PIDflash.PIT_Angle_I = (u16)(PID_PIT_Angle.I*1000);
	PIDflash.PIT_Angle_D = (u16)(PID_PIT_Angle.D*1000);
	PIDflash.YAW_Angle_P = (u16)(PID_YAW_Angle.P*1000);
	PIDflash.YAW_Angle_I = (u16)(PID_YAW_Angle.I*1000);
	PIDflash.YAW_Angle_D = (u16)(PID_YAW_Angle.D*1000);
	
	//角速度环PID参数
	PIDflash.ROL_Rate_P  = (u16)(PID_ROL_Rate.P*1000);
	PIDflash.ROL_Rate_I  = (u16)(PID_ROL_Rate.I*1000);
	PIDflash.ROL_Rate_D  = (u16)(PID_ROL_Rate.D*1000);
	PIDflash.PIT_Rate_P  = (u16)(PID_PIT_Rate.P*1000);
	PIDflash.PIT_Rate_I  = (u16)(PID_PIT_Rate.I*1000);
	PIDflash.PIT_Rate_D  = (u16)(PID_PIT_Rate.D*1000);
	PIDflash.YAW_Rate_P  = (u16)(PID_YAW_Rate.P*1000);
	PIDflash.YAW_Rate_I  = (u16)(PID_YAW_Rate.I*1000);
	PIDflash.YAW_Rate_D  = (u16)(PID_YAW_Rate.D*1000);
	
	//高度环PID
	PIDflash.ALT_Rate_P = (u16)(PID_ALT_Rate.P*1000);
	PIDflash.ALT_Rate_I = (u16)(PID_ALT_Rate.I*1000);
	PIDflash.ALT_Rate_D = (u16)(PID_ALT_Rate.D*1000);
	PIDflash.ALT_P = (u16)(PID_ALT.P*1000);
	PIDflash.ALT_I = (u16)(PID_ALT.I*1000);
	PIDflash.ALT_D = (u16)(PID_ALT.D*1000);
	
	//随机获取的NRF地址最低字节
  PIDflash.NRFaddr = NRFaddr;
	
}

/******************************************************************************************
*函  数：void TableToParams(void)
*功  能：从Flash中读出的参数赋值给个全局变量
*参  数：无
*返回值：无
*备  注：由于PID参数保存的时候都乘以1000，所以读取的时候就需要除以1000
*******************************************************************************************/
void TableToParams(void)
{
	//零偏数据
	ACC_OFFSET_RAW.X = PIDflash.ACC_OFFSET_X;
	ACC_OFFSET_RAW.Y = PIDflash.ACC_OFFSET_Y;
	ACC_OFFSET_RAW.Z = PIDflash.ACC_OFFSET_Z;
	GYRO_OFFSET_RAW.X = PIDflash.GYRO_OFFSET_X;
	GYRO_OFFSET_RAW.Y = PIDflash.GYRO_OFFSET_Y;
	GYRO_OFFSET_RAW.Z = PIDflash.GYRO_OFFSET_Z;
	
	//角度环数据PID参数
	PID_ROL_Angle.P = PIDflash.ROL_Angle_P/1000.0f;
	PID_ROL_Angle.I = PIDflash.ROL_Angle_I/1000.0f;
	PID_ROL_Angle.D = PIDflash.ROL_Angle_D/1000.0f;
	PID_PIT_Angle.P = PIDflash.PIT_Angle_P/1000.0f;
	PID_PIT_Angle.I = PIDflash.PIT_Angle_I/1000.0f;
	PID_PIT_Angle.D = PIDflash.PIT_Angle_D/1000.0f;
	PID_YAW_Angle.P = PIDflash.YAW_Angle_P/1000.0f;
	PID_YAW_Angle.I = PIDflash.YAW_Angle_I/1000.0f;
	PID_YAW_Angle.D = PIDflash.YAW_Angle_D/1000.0f;
		
	//角速度环PID参数	
	PID_ROL_Rate.P = PIDflash.ROL_Rate_P/1000.0f;
	PID_ROL_Rate.I = PIDflash.ROL_Rate_I/1000.0f;
	PID_ROL_Rate.D = PIDflash.ROL_Rate_D/1000.0f;
	PID_PIT_Rate.P = PIDflash.PIT_Rate_P/1000.0f;
	PID_PIT_Rate.I = PIDflash.PIT_Rate_I/1000.0f;
	PID_PIT_Rate.D = PIDflash.PIT_Rate_D/1000.0f;
	PID_YAW_Rate.P = PIDflash.YAW_Rate_P/1000.0f;
	PID_YAW_Rate.I = PIDflash.YAW_Rate_I/1000.0f;
	PID_YAW_Rate.D = PIDflash.YAW_Rate_D/1000.0f;
	
	//高度环PID
	PID_ALT_Rate.P = PIDflash.ALT_Rate_P/1000.0f;
	PID_ALT_Rate.I = PIDflash.ALT_Rate_I/1000.0f;
	PID_ALT_Rate.D = PIDflash.ALT_Rate_D/1000.0f;
	PID_ALT.P = PIDflash.ALT_P/1000.0f;
	PID_ALT.I = PIDflash.ALT_I/1000.0f;
	PID_ALT.D = PIDflash.ALT_D/1000.0f;
	
	//随机获取的NRF地址最低字节
  NRFaddr = PIDflash.NRFaddr;
}

/******************************************************************************************
*函  数：void DefaultParams(void)
*功  能：默认参数
*参  数：无
*返回值：无
*备  注：如果由于误操作将Flash中的数据都擦除了可调用此函数重新初始化或者写入Flash
*******************************************************************************************/
void DefaultParams(void)
{          
	//零偏数据
	PIDflash.ACC_OFFSET_X  = 0;
	PIDflash.ACC_OFFSET_Y  = 0;
	PIDflash.ACC_OFFSET_Z  = 0;
	PIDflash.GYRO_OFFSET_X = 0;
	PIDflash.GYRO_OFFSET_Y = 0;
	PIDflash.GYRO_OFFSET_Z = 0;
	
	//角度环数据PID参数
	PIDflash.ROL_Angle_P = 1800;
	PIDflash.ROL_Angle_I = 8;
	PIDflash.ROL_Angle_D = 50;
	PIDflash.PIT_Angle_P = 1800;
	PIDflash.PIT_Angle_I = 8;
	PIDflash.PIT_Angle_D = 50;
	PIDflash.YAW_Angle_P = 3500;
	PIDflash.YAW_Angle_I = 0;
	PIDflash.YAW_Angle_D = 1000;
	
	//角速度环PID参数
	PIDflash.ROL_Rate_P  = 930;
	PIDflash.ROL_Rate_I  = 5;
	PIDflash.ROL_Rate_D  = 860;
	PIDflash.PIT_Rate_P  = 930;
	PIDflash.PIT_Rate_I  = 5;
	PIDflash.PIT_Rate_D  = 860;
	PIDflash.YAW_Rate_P  = 2000;
	PIDflash.YAW_Rate_I  = 50;
	PIDflash.YAW_Rate_D  = 1000;
	
	//高度环PID
	PIDflash.ALT_Rate_P = 0;
	PIDflash.ALT_Rate_I = 0;
	PIDflash.ALT_Rate_D = 0;
	PIDflash.ALT_P = 0;
	PIDflash.ALT_I = 0;
	PIDflash.ALT_D = 0;            	
}

/************************************************************************
*函  数：void ParamsClearAll(void)
*功  能：从Flash存储结构的数据都清零
*参  数：无
*返回值：无
*备  注：无
*************************************************************************/
void ParamsClearAll(void)
{
	//零偏数据
	PIDflash.ACC_OFFSET_X = 0;
	PIDflash.ACC_OFFSET_Y = 0;
	PIDflash.ACC_OFFSET_Z = 0;
	PIDflash.GYRO_OFFSET_X = 0;
	PIDflash.GYRO_OFFSET_Y = 0;
	PIDflash.GYRO_OFFSET_Z = 0;
	
	//角度环数据PID参数
  PIDflash.ROL_Angle_P = 0;
  PIDflash.ROL_Angle_I = 0;
  PIDflash.ROL_Angle_D = 0;
  PIDflash.PIT_Angle_P = 0;
  PIDflash.PIT_Angle_I = 0;
  PIDflash.PIT_Angle_D = 0;
  PIDflash.YAW_Angle_P = 0;
  PIDflash.YAW_Angle_I = 0;
  PIDflash.YAW_Angle_D = 0;

	//角速度环PID参数	
  PIDflash.ROL_Rate_P = 0;
  PIDflash.ROL_Rate_I = 0; 
  PIDflash.ROL_Rate_D = 0;
  PIDflash.PIT_Rate_P = 0; 
  PIDflash.PIT_Rate_I = 0;
  PIDflash.PIT_Rate_D = 0; 
	PIDflash.YAW_Rate_P = 0;
	PIDflash.YAW_Rate_I = 0; 
	PIDflash.YAW_Rate_D = 0; 

  //高度环PID
  PIDflash.ALT_Rate_P = 0;
  PIDflash.ALT_Rate_I = 0;
	PIDflash.ALT_Rate_D = 0;
  PIDflash.ALT_P = 0;
  PIDflash.ALT_I = 0;
  PIDflash.ALT_D = 0;
	
	//随机获取的NRF地址最低字节
  PIDflash.NRFaddr = 0;
}

/******************************************************************************************
*函  数：void PID_ClearFlash(void)
*功  能：将Flash中的存储参数单元数据都清零
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************************/
void PID_ClearFlash(void)
{
	u8 size,len;
	ParamsClearAll();//数据清除
	len =  sizeof(PIDflash);
	size = len/(4+(len%4)?1:0); //保存的数据长度
	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)(&PIDflash),size);
}

/******************************************************************************************
*函  数：void PID_WriteFlash(void)
*功  能：将重要参数写入Flash
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************************/
void PID_WriteFlash(void)
{
	u8 size,len;
	ParamsToTable();//浮点数转换成整数
	len =  sizeof(PIDflash);
	size = len/(4+(len%4)?1:0); //保存的数据长度
	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)(&PIDflash),size);
}

/******************************************************************************************
*函  数：void ParamsClearAll(void)
*功  能：从Flash读取参数
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************************/
 void PID_ReadFlash(void)
{
	u8 size,len;
	len =  sizeof(PIDflash);
	size = len/(4+(len%4)?1:0);//保存的数据长度
	STMFLASH_Read(FLASH_SAVE_ADDR,(uint32_t*)(&PIDflash),size);
  TableToParams();//将整数转换成浮点数
	//读取参数失败
	if(PIDflash.PIT_Rate_P ==0xFFFF && PIDflash.ROL_Rate_P==0xFFFF && PIDflash.YAW_Rate_P==0xFFFF)
	{
		
		DefaultParams_WriteFlash();
		printf("Flash Read Error!!!\r\n");
	}
	else
	{
		printf("Flash Read OK!!!\r\n");
	}
	/*
	printf("\r***************PID Angle loop parameter****************\r\n");
	printf("\rPID_ROL_Angle.P = %0.2f\r\n",PID_ROL_Angle.P);
	printf("\rPID_ROL_Angle.I = %0.2f\r\n",PID_ROL_Angle.I);
	printf("\rPID_ROL_Angle.D = %0.2f\r\n",PID_ROL_Angle.D);
	printf("\rPID_PIT_Angle.P = %0.2f\r\n",PID_PIT_Angle.P);
	printf("\rPID_PIT_Angle.I = %0.2f\r\n",PID_PIT_Angle.I);
	printf("\rPID_PIT_Angle.D = %0.2f\r\n",PID_PIT_Angle.D);
	printf("\rPID_YAW_Angle.P = %0.2f\r\n",PID_YAW_Angle.P);
	printf("\rPID_YAW_Angle.I = %0.2f\r\n",PID_YAW_Angle.I);
	printf("\rPID_YAW_Angle.D = %0.2f\r\n",PID_YAW_Angle.D);
	printf("\r***************PID AngleRate loop parameter**************\r\n");
	printf("\rPID_ROL_Rate.P = %0.2f\r\n",PID_ROL_Rate.P );
	printf("\rPID_ROL_Rate.I = %0.2f\r\n",PID_ROL_Rate.I );
	printf("\rPID_ROL_Rate.D = %0.2f\r\n",PID_ROL_Rate.D );
	printf("\rPID_PIT_Rate.P = %0.2f\r\n",PID_PIT_Rate.P );
	printf("\rPID_PIT_Rate.I = %0.2f\r\n",PID_PIT_Rate.I );
	printf("\rPID_PIT_Rate.D = %0.2f\r\n",PID_PIT_Rate.D );
	printf("\rPID_YAW_Rate.P = %0.2f\r\n",PID_YAW_Rate.P );
	printf("\rPID_YAW_Rate.I = %0.2f\r\n",PID_YAW_Rate.I );
	printf("\rPID_YAW_Rate.D = %0.2f\r\n",PID_YAW_Rate.D );
	printf("\r*****************PID ALTRate loop parameter**************\r\n");
	printf("\rPID_ALT_Rate.P = %0.2f\r\n",PID_ALT_Rate.P );
	printf("\rPID_ALT_Rate.I = %0.2f\r\n",PID_ALT_Rate.I );
	printf("\rPID_ALT_Rate.D = %0.2f\r\n",PID_ALT_Rate.D );
	printf("\r********************PID ALT loop parameter****************\r\n");
	printf("\rPID_ALT.P = %0.2f\r\n",PID_ALT.P );
	printf("\rPID_ALT.I = %0.2f\r\n",PID_ALT.I );
	printf("\rPID_ALT.D = %0.2f\r\n",PID_ALT.D );
	printf("\rNRFaddr = 0x%x\r\n",NRFaddr);
	*/
}

/******************************************************************************************
*函  数：void DefaultParams_WriteFlash(void)
*功  能：将默认PID参数写入Flash
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************************/
void DefaultParams_WriteFlash(void)
{
	u8 size,len;
	DefaultParams();//初始化默认参数
	len =  sizeof(PIDflash);
	size = len/(4+(len%4)?1:0); //保存的数据长度
	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)(&PIDflash),size);
}


//void ParamsClear_MPUOffset(void)
//{
//	PIDflash.ACC_OFFSET_X = 0xFFFF;
//	PIDflash.ACC_OFFSET_Y = 0xFFFF;
//	PIDflash.ACC_OFFSET_Z = 0xFFFF;
//	PIDflash.GYRO_OFFSET_X = 0xFFFF;
//	PIDflash.GYRO_OFFSET_Y = 0xFFFF;
//	PIDflash.GYRO_OFFSET_Z = 0xFFFF;
//}

//void ParamsClear_AnglePID(void)
//{
//	PIDflash.ROL_Angle_P = 0xFFFF;
//  PIDflash.ROL_Angle_I = 0xFFFF;
//  PIDflash.ROL_Angle_D = 0xFFFF;
//  PIDflash.PIT_Angle_P = 0xFFFF;
//  PIDflash.PIT_Angle_I = 0xFFFF;
//  PIDflash.PIT_Angle_D = 0xFFFF;
//  PIDflash.YAW_Angle_P = 0xFFFF;
//  PIDflash.YAW_Angle_I = 0xFFFF;
//  PIDflash.YAW_Angle_D = 0xFFFF;
//}

//void ParamsClear_RatePID(void)
//{
//	PIDflash.ROL_Rate_P = 0xFFFF;
//  PIDflash.ROL_Rate_I = 0xFFFF; 
//  PIDflash.ROL_Rate_D = 0xFFFF;
//  PIDflash.ROL_Rate_C = 0xFFFF; 
//  PIDflash.PIT_Rate_P = 0xFFFF; 
//  PIDflash.PIT_Rate_I = 0xFFFF;
//  PIDflash.PIT_Rate_D = 0xFFFF; 
//  PIDflash.PIT_Rate_C = 0xFFFF; 
//	PIDflash.YAW_Rate_P = 0xFFFF;
//	PIDflash.YAW_Rate_I = 0xFFFF; 
//	PIDflash.YAW_Rate_D = 0xFFFF; 
//	PIDflash.YAW_Rate_C = 0xFFFF; 
//}


