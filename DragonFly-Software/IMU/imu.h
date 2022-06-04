#ifndef _IMU_H_
#define _IMU_H_
#include "stm32f4xx.h"
#include "structconfig.h"

#define G					  9.80665f		      // m/s^2	
#define RadtoDeg    57.324841f				//弧度到角度 (弧度 * 180/3.1415)
#define DegtoRad    0.0174533f				//角度到弧度 (角度 * 3.1415/180)

void Prepare_Data(void);
void IMUupdate(FLOAT_XYZ *Gyr_rad,FLOAT_XYZ *Acc_filt,FLOAT_ANGLE *Att_Angle);
#endif
