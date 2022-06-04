#include "spl06.h"
#include "led.h"

int16_t C0;
int16_t C1;
int32_t C00;
int32_t C10;
int16_t C01;
int16_t C11;
int16_t C20;
int16_t C21;
int16_t C30;


float raw_temp, raw_press;


float _kT, _kP;
float temp, press;

uint8_t spl06_init(void)
{
    uint8_t coef[18];
    uint8_t id;

    if(spl06_write_reg(SP06_RESET, 0x89))
    {
        printf("spl06 reset  fail\r\n");
        return 1;
    }
		
    spl06_read_reg(SP06_ID, &id);
		
//		printf("sol06 id  = %x\r\n", id);
		
    if(id != 0x1D)
    {
			while(1)
			{
				RGB_LED_Blue();//蓝灯常亮
			}
    }

    delay_ms(100);        //复位后系数准备好需要至少40ms

    spl06_read_buffer(SP06_COEF,coef,18);
    C0 = ((int16_t)coef[0]<<4 ) + ((coef[1]&0xF0)>>4);
    C0 = (C0&0x0800)?(0xF000|C0):C0;
    C1 = ((int16_t)(coef[1]&0x0F)<<8 ) + coef[2];
    C1 = (C1&0x0800)?(0xF000|C1):C1;
    C00 = ((int32_t)coef[3]<<12 ) + ((uint32_t)coef[4]<<4 ) + (coef[5]>>4);
    C10 = ((int32_t)(coef[5]&0x0F)<<16 ) + ((uint32_t)coef[6]<<8 ) + coef[7];
    C00 = (C00&0x080000)?(0xFFF00000|C00):C00;
    C10 = (C10&0x080000)?(0xFFF00000|C10):C10;
    C01 = ((int16_t)coef[8]<<8 ) + coef[9];
    C11 = ((int16_t)coef[10]<<8 ) + coef[11];
    C11 = (C11&0x0800)?(0xF000|C11):C11;
    C20 = ((int16_t)coef[12]<<8 ) + coef[13];
    C20 = (C20&0x0800)?(0xF000|C20):C20;
    C21 = ((int16_t)coef[14]<<8 ) + coef[15];
    C21 = (C21&0x0800)?(0xF000|C21):C21;
    C30 = ((int16_t)coef[16]<<8 ) + coef[17];
    C30 = (C30&0x0800)?(0xF000|C30):C30;

    spl06_config_pressure(PM_RATE_128,PM_PRC_64);
    spl06_config_temperature(PM_RATE_8,TMP_PRC_8);

    spl06_start(MEAS_CTRL_ContinuousPressTemp); //启动连续的气压温度测量
    delay_ms(20);

    return 0;
}

void spl06_start(uint8_t mode)
{
    spl06_write_reg(SP06_MEAS_CFG, mode);
}

void spl06_config_temperature(uint8_t rate,uint8_t oversampling)
{
    switch(oversampling)
    {
        case TMP_PRC_1:
            _kT = 524288;
            break;
        case TMP_PRC_2:
            _kT = 1572864;
            break;
        case TMP_PRC_4:
            _kT = 3670016;
            break;
        case TMP_PRC_8:
            _kT = 7864320;
            break;
        case TMP_PRC_16:
            _kT = 253952;
            break;
        case TMP_PRC_32:
            _kT = 516096;
            break;
        case TMP_PRC_64:
            _kT = 1040384;
            break;
        case TMP_PRC_128:
            _kT = 2088960;
            break;
    }

    spl06_write_reg(SP06_TMP_CFG,rate|oversampling|0x80);   //温度每秒128次测量一次
    if(oversampling > TMP_PRC_8)
    {
        uint8_t temp;
			  spl06_read_reg(SP06_CFG_REG, &temp);
        spl06_write_reg(SP06_CFG_REG,temp|SPL06_CFG_T_SHIFT);
    }
}

void spl06_config_pressure(uint8_t rate,uint8_t oversampling)
{
    switch(oversampling)
    {
        case PM_PRC_1:
            _kP = 524288;
            break;
        case PM_PRC_2:
            _kP = 1572864;
            break;
        case PM_PRC_4:
            _kP = 3670016;
            break;
        case PM_PRC_8:
            _kP = 7864320;
            break;
        case PM_PRC_16:
            _kP = 253952;
            break;
        case PM_PRC_32:
            _kP = 516096;
            break;
        case PM_PRC_64:
            _kP = 1040384;
            break;
        case PM_PRC_128:
            _kP = 2088960;
            break;
    }

    spl06_write_reg(SP06_PSR_CFG,rate|oversampling);
    if(oversampling > PM_PRC_8)
    {
        uint8_t temp;
			  spl06_read_reg(SP06_CFG_REG, &temp);
        spl06_write_reg(SP06_CFG_REG,temp|SPL06_CFG_P_SHIFT);
    }
}

int32_t spl06_get_pressure_adc()
{
    uint8_t buf[3];
    int32_t adc;

    spl06_read_buffer(SP06_PSR_B2,buf,3);
    adc = (int32_t)(buf[0]<<16) + (buf[1]<<8) + buf[2];
    adc = (adc&0x800000)?(0xFF000000|adc):adc;

    return adc;
}

int32_t spl06_get_temperature_adc()
{
    uint8_t buf[3];
    int32_t adc;

    spl06_read_buffer(SP06_TMP_B2,buf,3);
    adc = (int32_t)(buf[0]<<16) + (buf[1]<<8) + buf[2];

    return adc;
}

void spl06_update_pressure()
{
    float Traw_src, Praw_src;
    float qua2, qua3;
	
		Traw_src =  raw_temp/_kT;
    Praw_src =  raw_press/_kP;
	
//	  printf("raw_temp = %.2f\traw_press = %.2f\r\n", raw_temp, raw_press);

    //计算温度
    temp = 0.5f*C0 + Traw_src * C1;

    //计算气压
    qua2 = C10 + Praw_src * (C20 + Praw_src* C30);
    qua3 = Traw_src * Praw_src * (C11 + Praw_src * C21);
    press = C00 + Praw_src * qua2 + Traw_src * C01 + qua3;
//		printf("_Press = %.4f\t _Temp = %.4f\r\n", press, temp);
}

void spl06_update()
{
    raw_temp = spl06_get_temperature_adc();
    raw_press = spl06_get_pressure_adc();
    spl06_update_pressure();
}

float spl06_get_temperature()
{
    return temp;
}

float spl06_get_pressure()
{
    return press;
}

uint8_t spl06_write_reg(uint8_t reg_addr,uint8_t reg_val)
{
	if(IIC_WriteByteToSlave(SPL06_IIC_ADDR, reg_addr, reg_val))
	   return 1;
	else
	   return 0;
}

uint8_t spl06_read_reg(uint8_t reg_addr,uint8_t *buf)
{
	if(IIC_ReadByteFromSlave(SPL06_IIC_ADDR, reg_addr, buf))
	   return 1;
	else
	   return 0;
}

uint8_t spl06_read_buffer(uint8_t reg_addr,void *buffer,uint16_t len)
{
	if(IIC_ReadMultByteFromSlave(SPL06_IIC_ADDR, reg_addr, len, buffer))
	   return 1;
	else
	   return 0;
}
