/***************************************************************************************
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：V1.01
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "power.h"
#include "nrf2401.h"
#include "stdio.h"
#include "structconfig.h"
#include "filter.h"

BATT_TYPE BAT=
{
	.BattAdc = 0,        //电池电压采集ADC值
	.BattRealV = 3.31f,  //实际测量的飞机供电电压 (注意此电压必须亲测否则测量的电压不准)
	.BattMeasureV = 0,   //程序测量的实际电池电压
	.BattAlarmV = 3.2f,  //电池低电压报警瞬时值 (这个值需要根据机身不同重量实测，实测380mh是2.8v)
	.BattFullV = 4.2f,   //电池充满电值 4.2V
};
uint8_t BATT_LEDflag = 0;
/******************************************************************************************
*函  数：void BATT_Init(void)
*功  能：电源管理初始化
*参  数：无
*返回值：无
*备  注：WiFi和OpenMV默认是关闭状态 
*******************************************************************************************/
void BATT_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_InitTypeDef   ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
	
	//模拟输入模式选择 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;      
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  //开漏输出模式选择 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	
	//ESP8266(wifi) EN,BOOT控制引脚 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA,GPIO_Pin_4); //ESP_EN
	GPIO_ResetBits(GPIOA,GPIO_Pin_5); //ESP_BOOT
	
	//OpenMV OM_PWR电源控制引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	
	
	//ADC通用配置(ADC时钟频率最好不要超过36MHz)
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; //独立模式
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; //4分频 fplck2/4 = 25MHz 
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; // 两个采样之间间隔5个时钟 
  ADC_CommonInit(&ADC_CommonInitStructure);

  //ADC1参数初始化
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; //12位采样精度
  ADC_InitStructure.ADC_ScanConvMode = DISABLE; //失能扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //失能连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; //不开启触发，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //数据右对齐
  ADC_InitStructure.ADC_NbrOfConversion = 1; //一个转化在规则序列中
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_144Cycles); //ADC1的ADC_Channel_0进行规则转换配置

  ADC_Cmd(ADC1, ENABLE); //使能ADC1
}

/******************************************************************************************
*函  数：uint16_t Get_BatteryAdc(uint8_t ch)
*功  能：获取电池采样点电压的ADC值
*参  数：ch ADC采样通道
*返回值：返回通道AD值
*备  注：电池电压采样点的ADC值，电池电压采样电路见原理图
*******************************************************************************************/
uint16_t Get_BatteryAdc(uint8_t ch)
{
  ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_144Cycles);
	ADC_SoftwareStartConv(ADC1);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	return ADC_GetConversionValue(ADC1);
}

/******************************************************************************************
*函  数：void BATT_GetVoltage(void)
*功  能：获取电池电压
*参  数：无
*返回值：无
*备  注：电池电压 = ADC检测电压*2 具体原理可看原理图
*******************************************************************************************/
void BATT_GetVoltage(void)
{
	float V;
	Aver_Filter((float)Get_BatteryAdc(ADC_Channel_0),&BAT.BattAdc,6); //滑动滤波一下电压值，提高精度
	if(BAT.BattAdc)
	V = BAT.BattAdc * BAT.BattRealV / 4095.0f;
	BAT.BattMeasureV = 2*V; //根据原理电阻分压，可知 电池实际电压 = ADC侧量电压 * 2
	//printf("Test Voltage :%0.2f   temp:%0.0f \r\n ",BAT.BattMeasureV,BAT.BattAdc);
}

/******************************************************************************************
*函  数：void LowVoltage_Alarm(void)
*功  能：低电量报警
*参  数：无
*返回值：无
*备  注：电池电压 = ADC检测电压*2 具体原理可看原理图
*******************************************************************************************/
void LowVoltage_Alarm(void)
{
	static uint8_t cnt=0,cnt1=0;
	BATT_GetVoltage();
	if(Airplane_Enable)
	{
		if(BAT.BattMeasureV < BAT.BattAlarmV)//飞行时测量
		{
			if(cnt1++>10)
			{
				cnt1 = 0;
			  BATT_LEDflag = 1;
			}
		}
		else
		{
			cnt1 = 0;
			BATT_LEDflag = 0;
		}
	}else
	{
		if(BAT.BattMeasureV < 3.7f)//落地时测量（380mh时是3.5V）
		{
			if(cnt++>10)
			{
				Run_flag = 0;
				cnt = 0;
			  BATT_LEDflag = 1;
			}
		}
		else
		{
			Run_flag = 1;
			cnt = 0;
			BATT_LEDflag = 0;
		}
	}
	
}
/******************************************************************************************
*函  数：void WiFi_Switch(uint8_t flag)
*功  能：开关Wifi模块
*参  数：flag=1开启WiFi;flag=0关闭WiFi
*返回值：无
*备  注：ESP8266(wifi) EN，BOOT 控制引脚，当EN和BOOT都置1时wifi进入工作模式，反之进入低功耗
*******************************************************************************************/
void WiFi_Switch(uint8_t flag)
{
	if(flag)/* 开启WiFi */
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_4); //ESP_EN
    GPIO_SetBits(GPIOA,GPIO_Pin_5); //ESP_BOOT
	}
	else    /* 关闭WiFi */
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_4); //ESP_EN
    GPIO_ResetBits(GPIOA,GPIO_Pin_5); //ESP_BOOT
	}
}
/******************************************************************************************
*函  数：void OpenMV_Switch(uint8_t flag)
*功  能：开关OpenMV模块
*参  数：flag=1开启OpenMV;flag=0关闭OpenMV
*返回值：无
*备  注：OpenMV OM_PWR电源控制引脚，当OM_PWR置1 OpenMV进入工作模式,反之进入低功耗
******************************************************************************************/
void OpenMV_Switch(uint8_t flag)
{
	if(flag)/* 开启OpenMV */
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_13);//OM_PWR
	}
	else    /* 关闭OpenMV */
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);//OM_PWR
	}
}
/******************************************************************************************
*函  数：void NRF2401_Switch(uint8_t flag)
*功  能：开关NRF2401模块
*参  数：flag=1开启NRF2401;flag=0关闭NRF2401
*返回值：无
*备  注：NRF2401 CE电源控制引脚，当CE置1 NRF2401进入工作模式,反之进入低功耗
******************************************************************************************/
void NRF2401_Switch(uint8_t flag)
{
	if(flag)/* 开启NRF2401 */
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_8);  //NRF_CE
	  NRF24l01_write_reg(W_REGISTER+CONFIG,0x0A);	 
	}
	else    /* 关闭NRF2401 */
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_8);//NRF_CE
		NRF24l01_write_reg(W_REGISTER+CONFIG,0x0F);	 
	}
}
