#ifndef _NRF24L01_H
#define _NRF24L01_H

#include "stm32f4xx.h"

/******* NRF24L01命令 *********/
#define R_REGISTER   0X00//读配置寄存器
#define W_REGISTER   0x20//写配置寄存器
#define R_RX_PAYLOAD 0X61//读RX有效数据
#define W_RX_PAYLOAD 0XA0//写TX有效数据
#define FLUSH_TX     0XE1//清除TX FIFO寄存器 应用于发射模式
#define FLUSH_RX     0XE2//清除RX FIFO寄存器 应用于接收模式
#define REUSE_TX_PL  0XE3//从新使用上一数据包
#define NOP          0XFF
/*******************************************************NRF24L01寄存器地址*******************************************************/
#define CONFIG       0X00 //bit0(PRIM_RX ):1接收模式 0发射模式 bit1(pwr_up):1上电0掉电bit3：1 16位CRC校验 0 八位CRC校验
//bit3:CRC使能 bit4:可屏蔽中断MAX_RT bit5:可屏蔽中断TX_DS bit6可屏蔽中断RX_RD
#define EN_AA        0X01//使能0——5通道的自动应答功能
#define EN_RXADDR    0X02//接收地址允许 0-5 通道 ，默认通道0通道1启动
#define SETUP_AW     0X03//设置地址宽度 00：无效 01：3字节 10：字节 11：5字节
#define SETUP_RETR   0X04//建立自动重发 3:0 自动重发计数 7:4 自动重发延时
#define RF_CH        0X05//射频通道 6:0 设置nRF24l01工作频率
#define RF_SETUP     0X06//射频寄存器 0:低噪声放大器增益 2:1 发射功率 3:传输效率
#define STATUS       0X07//状态寄存器 0:TX FIFO寄存器满标志 3:1 接收数据通道号 4:达到最大重发中断
//5：数据发送完成中断 6：数据接收中断
#define MAX_TX  		         0x10  //最大重发次数
#define TX_OK   		         0x20  //发送完成
#define RX_OK   		         0x40  //接收完成
#define RX_P_NO              0x0E  //
#define OBSERVE_TX   0X08//3:0 重发计数器(发送新数据包时复位) 7:4 数据包丢失计数器(写RF_CH时复位)
#define CD           0X09//载波检测
#define RX_ADDR_P0   0X0A//数据通道0接收地址 ，最大长度:5字节(先写低字节，所写字节数量覵ETUP_AW设定)
#define RX_ADDR_P1   0X0B//数据通道1接收地址 ，最大长度:5字节(先写低字节，所写字节数量覵ETUP_AW设定)
#define RX_ADDR_P2   0X0C//数据通道2接收地址 ,最低字节可设置。高字节部分必须与RX_ADDR_P1[39:8]相等
#define RX_ADDR_P3   0X0D//数据通道3接收地址 ,最低字节可设置。高字节部分必须与RX_ADDR_P1[39:8]相等
#define RX_ADDR_P4   0X0E//数据通道4接收地址 ,最低字节可设置。高字节部分必须与RX_ADDR_P1[39:8]相等
#define RX_ADDR_P5   0X0F//数据通道5接收地址 ,最低字节可设置。高字节部分必须与RX_ADDR_P1[39:8]相等
#define TX_ADDR      0X10//发送地址 39:0
#define RX_PW_P0     0X11//接收数据通道0有效数据宽度(从1到32字节)
#define RX_PW_P1     0X12//接收数据通道1有效数据宽度(从1到32字节)
#define RX_PW_P2     0X13//接收数据通道2有效数据宽度(从1到32字节)
#define RX_PW_P3     0X14//接收数据通道3有效数据宽度(从1到32字节)
#define RX_PW_P4     0X15//接收数据通道4有效数据宽度(从1到32字节)
#define RX_PW_P5     0X16//接收数据通道5有效数据宽度(从1到32字节)
#define FIFO_STATUS  0X17//FIFO状态寄存器 0:RX FIFO寄存器空标志 1:RX FIFO寄存器满标志 4:TX FIFO寄存器空标志

#define  RX_DR 6 //数据接收完成中断标志位
#define  TX_DR 5 //数据发送完成中断标志位 (状态寄存器位置)
#define IT_TX 0x0E //发送模式
#define IT_RX 0x0F //接收模式

#define TX_ADR_WIDTH 5
#define RX_ADR_WIDTH 5
#define TX_PAYLO_WIDTH 32
#define RX_PAYLO_WIDTH 32

extern uint8_t NRF_TX_DATA[TX_PAYLO_WIDTH];
extern uint8_t NRF_RX_DATA[TX_PAYLO_WIDTH];

void NRF24l01_Init(void);		
void NRF24L01_Check(void);
void NRFset_Mode(uint8_t mode);
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);			  
uint8_t NRF24l01_read_reg(uint8_t reg);					
uint8_t NRF24l01_write_reg(uint8_t reg, uint8_t value);		

void NRF24L01_TxPacket(uint8_t *txbuf);				
void NRF24L01_RxPacket(uint8_t *rxbuf);	
void Remote_Connectiong(void);
void NRF_GetAddr(void);
void NRF_Test(void);

#endif
