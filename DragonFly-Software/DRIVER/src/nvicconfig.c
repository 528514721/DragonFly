/***************************************************************************************
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：V1.01
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "nvicconfig.h"
#include "stm32f4xx.h"

/*****************************************************************************
*函  数：void NVIV_Config(void)
*功  能：配置工程中所有中断的优先级
*参  数：无
*返回值：无
*备  注：此优先级中断不要随便更改哦
*****************************************************************************/
void NVIV_Config(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断分组2
	
	//程序时基TIM4定时器中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢断优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //子优选级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	//USART2中断
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢断优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//子优选级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//USART1中断
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢断优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //子优选级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	//NRF2401的IRQ中断
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢断优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //子优选级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
