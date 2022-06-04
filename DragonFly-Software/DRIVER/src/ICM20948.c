/*
 * ICM20948.c
 *
 *  Created on: Oct 26, 2018
 *      Author: cory
 */
 
#include "ICM20948.h"
#include "main.h"
#include "iic_moni.h"
#include "usart.h"
#include "delay.h"
#include <string.h>


uint16_t accel_data[3];
uint16_t gyro_data[3];
int16_t mag_data[3];




/*
 *
 * I2C abstraction
 * Creates a layer between STM32 HAL and the ICM library that will allow for easy platform swap
 *
 */
void ICM_ReadBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	IIC_ReadMultByteFromSlave(ICM20948Addr, reg, Size, pData);
}


void ICM_WriteBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	IIC_WriteMultByteToSlave(ICM20948Addr, reg, Size, pData);
}


void ICM_ReadOneByte(uint8_t reg, uint8_t* pData) // ***
{
	IIC_ReadByteFromSlave(ICM20948Addr, reg, pData);
}


void ICM_WriteOneByte(uint8_t reg, uint8_t Data) // ***
{
	IIC_WriteByteToSlave(ICM20948Addr, reg, Data);
}


/*
 *
 * AUX I2C abstraction for magnetometer
 * Creates a layer between STM32 HAL and the ICM library that will allow for easy platform swap
 *
 */
void ICM_MagWrite(uint8_t reg, uint8_t value) 
{
	ICM_WriteOneByte(0x7F, 0x30);
	delay_ms(1);
	ICM_WriteOneByte(0x03, 0x0C);
	delay_ms(1);
	ICM_WriteOneByte(0x04, reg);
	delay_ms(1);
	ICM_WriteOneByte(0x06, value);
	delay_ms(1);
}


static uint8_t ICM_MagRead(uint8_t reg) 
{
	uint8_t Data;
	ICM_WriteOneByte(0x7F, 0x30);
	delay_ms(1);
	ICM_WriteOneByte(0x03, 0x0C | 0x80);
	delay_ms(1);
	ICM_WriteOneByte(0x04, reg);
	delay_ms(1);
	ICM_WriteOneByte(0x06, 0xff);
	delay_ms(1);
	ICM_WriteOneByte(0x7F, 0x00);
	ICM_ReadOneByte(0x3B, &Data);
	delay_ms(1);
	return Data;
}


void ICM_ReadMagData(int16_t heading[3]) 
{
	uint8_t mag_buffer[10];
	mag_buffer[0] = ICM_MagRead(0x01);
	mag_buffer[1] = ICM_MagRead(0x11);
	mag_buffer[2] = ICM_MagRead(0x12);
	heading[0] = mag_buffer[1] | mag_buffer[2] << 8;
	mag_buffer[3] = ICM_MagRead(0x13);
	mag_buffer[4] = ICM_MagRead(0x14);
	heading[1] = mag_buffer[3] | mag_buffer[4] << 8;
	mag_buffer[5] = ICM_MagRead(0x15);
	mag_buffer[6] = ICM_MagRead(0x16);
	heading[2] = mag_buffer[5] | mag_buffer[6] << 8;
	ICM_MagWrite(0x31, 0x01);
}


/*
 *
 * Sequence to setup ICM290948 as early as possible after power on
 *
 */
void ICM_PowerOn(void) 
{
//	char uart_buffer[200];
	uint8_t whoami = 0xEA;
	uint8_t test = ICM_WHOAMI();
	
	printf("test = %x\r\n", test);
	
	

	delay_ms(10);
	ICM_SelectBank(USER_BANK_0);
	delay_ms(10);
//	ICM_Disable_I2C();
	delay_ms(10);
	ICM_SetClock((uint8_t) CLK_BEST_AVAIL);
	delay_ms(10);
	ICM_AccelGyroOff();
	delay_ms(20);
	ICM_AccelGyroOn();
	delay_ms(10);
	ICM_Initialize();
}


uint16_t ICM_Initialize(void) 
{
	ICM_SelectBank(USER_BANK_2);
	delay_ms(20);
	ICM_SetGyroRateLPF(GYRO_RATE_250, GYRO_LPF_17HZ);
	delay_ms(10);

	// Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
	ICM_WriteOneByte(0x00, 0x0A);
	delay_ms(10);

	// Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
	ICM_WriteOneByte(0x14, (0x04 | 0x11));

	// Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
	ICM_WriteOneByte(0x10, 0x00);
	delay_ms(10);

	// Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
	ICM_WriteOneByte(0x11, 0x0A);
	delay_ms(10);

	ICM_SelectBank(USER_BANK_2);
	delay_ms(20);

	// Configure AUX_I2C Magnetometer (onboard ICM-20948)
	ICM_WriteOneByte(0x7F, 0x00);
	ICM_WriteOneByte(0x0F, 0x30);
	ICM_WriteOneByte(0x03, 0x20);
	ICM_WriteOneByte(0x7F, 0x30);
	ICM_WriteOneByte(0x01, 0x4D);
	ICM_WriteOneByte(0x02, 0x01);
	ICM_WriteOneByte(0x05, 0x81);
	ICM_MagWrite(0x32, 0x01);
	delay_ms(1000);
	ICM_MagWrite(0x31, 0x02);
	return 1337;
}


void ICM_ReadAccelGyroData(void) 
{
	uint8_t raw_data[12];
	ICM_ReadBytes(0x2D, raw_data, 12);

	accel_data[0] = (raw_data[0] << 8) | raw_data[1];
	accel_data[1] = (raw_data[2] << 8) | raw_data[3];
	accel_data[2] = (raw_data[4] << 8) | raw_data[5];

	
	gyro_data[0] = (raw_data[6] << 8) | raw_data[7];
	gyro_data[1] = (raw_data[8] << 8) | raw_data[9];
	gyro_data[2] = (raw_data[10] << 8) | raw_data[11];

//	accel_data[0] = accel_data[0] / 8;
//	accel_data[1] = accel_data[1] / 8;
//	accel_data[2] = accel_data[2] / 8;

//	gyro_data[0] = gyro_data[0] / 250;
//	gyro_data[1] = gyro_data[1] / 250;
//	gyro_data[2] = gyro_data[2] / 250;
}


void ICM_SelectBank(uint8_t bank) 
{
	ICM_WriteOneByte(USER_BANK_SEL, bank);
}


void ICM_Disable_I2C(void) 
{
	ICM_WriteOneByte(0x03, 0x78);
}


void ICM_SetClock(uint8_t clk) 
{
	ICM_WriteOneByte(PWR_MGMT_1, clk);
}


void ICM_AccelGyroOff(void) 
{
	ICM_WriteOneByte(PWR_MGMT_2, (0x38 | 0x07));
}


void ICM_AccelGyroOn(void) 
{
	ICM_WriteOneByte(0x07, (0x00 | 0x00));
}


uint8_t ICM_WHOAMI(void) 
{
	uint8_t spiData = 0x01;
	ICM_ReadOneByte(0x00, &spiData);
	return spiData;
}


void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf) 
{
	ICM_WriteOneByte(GYRO_CONFIG_1, (rate | lpf));
}
