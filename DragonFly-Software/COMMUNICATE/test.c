#include "stm32F4xx.h"
#include "test.h"
#include "usart.h"
#include "structconfig.h"

uint8_t TxBUFF[60];
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(data)       ( *( (char *)(&data)		) )
#define BYTE1(data)       ( *( (char *)(&data) + 1) )
#define BYTE2(data)       ( *( (char *)(&data) + 2) )
#define BYTE3(data)       ( *( (char *)(&data) + 3) )
	


static void CAT_Senddata(uint8_t *data,uint8_t lenth)
{
	Usart_Send(data, lenth);
}
static void Status_Send(float ROLL,float PITCH,float YAW,uint8_t Mode,uint8_t lock)
{
	uint8_t cnt=0,i;
	int16_t temp,sum=0;
	
	TxBUFF[cnt++] = '&'; //0x26 //帧头
	TxBUFF[cnt++] = '&'; //0x26
	TxBUFF[cnt++] = 0x01; //功能字
	
	TxBUFF[cnt++] = 0;    //数据长度，先设为零
	
	temp = (int16_t)(ROLL*100);
	TxBUFF[cnt++] = BYTE1(temp);
	TxBUFF[cnt++] = BYTE0(temp);
	temp = (int16_t)(PITCH*100);
	TxBUFF[cnt++] = BYTE1(temp);
	TxBUFF[cnt++] = BYTE0(temp);
	temp = (int16_t)(YAW*100);
	TxBUFF[cnt++] = BYTE1(temp);
	TxBUFF[cnt++] = BYTE0(temp);
	
	TxBUFF[cnt++] = Mode;
	TxBUFF[cnt++] = lock;
	
	TxBUFF[3] = cnt-4;   //数据长度
	
	for(i=2;i<cnt;i++)   //求校验位
		sum += TxBUFF[i]; 
		
	TxBUFF[cnt++]=(0xFF&sum); //校验
	TxBUFF[cnt++] = '^';      //帧尾0x5E
	
	CAT_Senddata(TxBUFF,cnt);
}

void Senser_send(int16_t ACC_X,int16_t ACC_Y,int16_t ACC_Z,int16_t GYRO_X,int16_t GYRO_Y,int16_t GYRO_Z,
	               int16_t MAG_X,int16_t MAG_Y,int16_t MAG_Z)
{
	uint8_t cnt=0,i;
	int16_t temp,sum=0;
	
	TxBUFF[cnt++] = '&'; //0x26 //帧头
	TxBUFF[cnt++] = '&'; //0x26
	TxBUFF[cnt++] = 0x01; //功能字
	
	TxBUFF[cnt++] = 0;    //数据长度，先设为零
	
	temp = ACC_X;
	TxBUFF[cnt++] = BYTE1(temp);
	TxBUFF[cnt++] = BYTE0(temp);
	temp = ACC_Y;
	TxBUFF[cnt++] = BYTE1(temp);
	TxBUFF[cnt++] = BYTE0(temp);
	temp = ACC_Z;
	TxBUFF[cnt++] = BYTE1(temp);
	TxBUFF[cnt++] = BYTE0(temp);
	temp = GYRO_X;
	TxBUFF[cnt++] = BYTE1(temp);
	TxBUFF[cnt++] = BYTE0(temp);
	temp = GYRO_Y;
	TxBUFF[cnt++] = BYTE1(temp);
	TxBUFF[cnt++] = BYTE0(temp);
	temp = GYRO_Z;
	TxBUFF[cnt++] = BYTE1(temp);
	TxBUFF[cnt++] = BYTE0(temp);
	temp = MAG_X;
	TxBUFF[cnt++] = BYTE1(temp);
	TxBUFF[cnt++] = BYTE0(temp);
	temp = MAG_Y;
	TxBUFF[cnt++] = BYTE1(temp);
	TxBUFF[cnt++] = BYTE0(temp);
	temp = MAG_Z;
	TxBUFF[cnt++] = BYTE1(temp);
	TxBUFF[cnt++] = BYTE0(temp);
	
	TxBUFF[3] = cnt-4;   //数据长度
	
	for(i=2;i<cnt;i++)   //求校验位
		sum += TxBUFF[i]; 
		
	TxBUFF[cnt++]=(0xFF&sum); //校验
	TxBUFF[cnt++] = '^';      //帧尾0x5E
	
	CAT_Senddata(TxBUFF,cnt);
}

void Fly_Txdata(void)
{
	Status_Send(Att_Angle.rol,Att_Angle.pit,Att_Angle.yaw,0x01,Airplane_Enable);
	//Senser_send();
}
