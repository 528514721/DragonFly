#ifndef ANO_DT_H
#define ANO_DT_H
#include "stm32f4xx.h"

void ANO_DT_Data_Exchange(void);
void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length);
void ANO_DT_Data_Receive_Prepare(uint8_t data);
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, uint8_t fly_model, uint8_t armed);
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar);
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_DT_Send_Power(u16 votage, u16 current);
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);

//自定义帧
void Data_Send_AngleRate(float data1,float data2,float data3,float data4,float data5,float data6,float data7,float data8); //角速度调试环
void Data_Send_Filter(void);    //滤波调试
#endif

