/***************************************************************************************
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：V1.01
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "flash.h"
#include "delay.h"
#include "usart.h" 
 
/******************************************************************************
*函  数：u32 STMFLASH_ReadWord(u32 faddr)
*功　能：读取指定地址的字(32位数据) 
*参  数：faddr:读地址 
*返回值：对应数据
*备  注：无
*******************************************************************************/
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  

/******************************************************************************
*函  数：uint16_t STMFLASH_GetFlashSector(u32 addr)
*功　能：获取某个地址所在的flash扇区
*参  数：addr:flash地址
*返回值：0~7,即addr所在的扇区
*备  注：无
*******************************************************************************/
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	return FLASH_Sector_7;	
}

/******************************************************************************
*函  数：void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
*功　能：从指定地址开始写入指定长度的数据 
*参  数：WriteAddr:起始地址(此地址必须为4的倍数!!)
*        pBuffer:数据指针
*        NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
*返回值：无
*备  注：STM32F4的Flash未写扇区默认是0xFFF...F,调用此函数时需要
*******************************************************************************/
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									 //解锁 
  FLASH_DataCacheCmd(DISABLE);     //FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				         //写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	 //写入的结束地址
	if(addrx<0X1FFF0000)			       //只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		       //扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	    //发生错误了
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr) //写数据
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE) //写入数据
			{ 
				break; //写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock(); //上锁
} 

/******************************************************************************
*函  数：void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)  
*功　能：从指定地址开始读出指定长度的数据
*参  数：ReadAddr:起始地址
*        pBuffer:数据指针
*        NumToRead:字(4位)数
*返回值：无
*备  注：无
*******************************************************************************/
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}








