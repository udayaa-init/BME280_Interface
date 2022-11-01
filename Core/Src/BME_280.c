/*
 * BME_280.c
 *
 *  Created on: Nov 1, 2022
 *      Author: Dell
 */


#include "BME_280.h"

//notes:
// when entered in forced mode we have to call the wake up function before measuring everytime.

//when ctrl_hum is written with new value them ctrl_meas must be written for ctrl_hum to come into effect
#define BME_addr 0xEC

#define BME280_S32_t  int32_t
#define BME280_U32_t uint32_t

extern I2C_HandleTypeDef hi2c1;
extern float Temperature, Pressure, Humidity;

// these functions are given in Manual of the BME sensor
BME280_U32_t bme280_compensate_H_int32(BME280_S32_t adc_H);
BME280_U32_t BME280_compensate_P_int32(BME280_S32_t adc_P);
BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T);
BME280_S32_t t_fine;





// extract the calibration parameter stored in NVM;
unsigned short int dig_T1, dig_P1;

signed short int dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5;

unsigned char dig_H1 , dig_H3;

signed char dig_H6;

void trim_read()
{

	uint8_t trim_paras[32];

    //HAL_MEM_READ
    HAL_I2C_Mem_Read(&hi2c1, BME_addr, 0X88, 1, trim_paras, 25, 1000);
    HAL_I2C_Mem_Read(&hi2c1, BME_addr, 0XE1, 1, (uint8_t*)trim_paras+25, 7, 1000);
    //extract from mem address and store in paras variable defined above

    dig_T1 = trim_paras[1]<<8 | trim_paras[0];
    dig_T2 = trim_paras[3]<<8 | trim_paras[2];
    dig_T3 = trim_paras[5]<<8 | trim_paras[4];
    dig_P1 = trim_paras[7]<<8 | trim_paras[6];
    dig_P2 = trim_paras[9]<<8 | trim_paras[8];
    dig_P3 = trim_paras[11]<<8 | trim_paras[10];
    dig_P4 = trim_paras[13]<<8 | trim_paras[12];
    dig_P5 = trim_paras[15]<<8 | trim_paras[14];
    dig_P6 = trim_paras[17]<<8 | trim_paras[16];
    dig_P7 = trim_paras[19]<<8 | trim_paras[18];
    dig_P8 = trim_paras[21]<<8 | trim_paras[20];
    dig_P9 = trim_paras[23]<<8 | trim_paras[22];
    dig_H1 = trim_paras[24];
    dig_H2 = trim_paras[26]<<8 | trim_paras[25];
    dig_H3 = trim_paras[27];
    dig_H4 = trim_paras[28]<<4 | (trim_paras[29]&0x0F);
    dig_H5 = (trim_paras[29]& 0xF0)>>4 | trim_paras[30]<<4 ;
    dig_H6 = trim_paras[31];


}

// configure the the sensor with the oversampling rate for pressure, temperature and humidity, filter coefficient and standby time
//standby time is the interval between the measurement reading in normal mode
void BME_Config(uint8_t osrs_p, uint8_t osrs_t,uint8_t osrs_h,uint8_t mode, uint8_t filter , uint8_t standby)
{
	//reset
    uint8_t writ = 0xB6;
    if(HAL_I2C_Mem_Write(&hi2c1, BME_addr, RESET, 1, &writ, 1, 1000)== HAL_OK)
    {
    	writ = 0x0c;
    }



    uint8_t temp_conf= 0 ;
    temp_conf |= standby<<5|filter<<2;
    uint8_t check;
    //halwrite to conf register
    HAL_I2C_Mem_Write(&hi2c1, BME_addr, CONFIG, 1, &temp_conf, 1, 1000);
    HAL_Delay(100);
    uint8_t hum =0;
    hum |= osrs_h ;

    uint8_t meas=0;
    meas |= osrs_t << 5 |osrs_p<<2| mode;

    //hal write to ctrl_hum
    HAL_I2C_Mem_Write(&hi2c1, BME_addr, CTRL_HUM, 1, &hum, 1, 1000);
    HAL_Delay(100);
    //hal write to ctrl_meas
    HAL_I2C_Mem_Write(&hi2c1, BME_addr, CTRL_MEAS, 1, &meas, 1, 1000);
    HAL_Delay(100);
    HAL_I2C_Mem_Read(&hi2c1, BME_addr, CTRL_MEAS, 1, &check, 1, 1000);
    HAL_Delay(100);
    trim_read();
}



void Measure()
{
    int32_t pres_raw, humidity_raw, temp_raw;

    uint8_t raw_read[3];

    //hal read mem from addres F7 to 3 byte for pressure
    HAL_I2C_Mem_Read(&hi2c1, BME_addr, 0xF7, 1, raw_read, 3, 1000);
    pres_raw = raw_read[0]<<12 | raw_read[1]<<4 | (raw_read[2]&0xf0 )>>4;


    // hal_Read from mem add FA to 3 byte for temp
    HAL_I2C_Mem_Read(&hi2c1, BME_addr, 0xFA, 1, raw_read, 3, 1000);
    temp_raw = raw_read[0]<<12 | raw_read[1]<<4 | (raw_read[2]&0xf0 )>>4;

    //hal read from mem add FD to 2 byte
    HAL_I2C_Mem_Read(&hi2c1, BME_addr, 0xFD, 1, raw_read, 2, 1000);
    humidity_raw = raw_read[0]<<8 | (raw_read[1] );

    Temperature = BME280_compensate_T_int32(temp_raw)/100;
    Pressure =  BME280_compensate_P_int32(pres_raw);
    Humidity = bme280_compensate_H_int32(humidity_raw)/1024;

}


//calibration code

BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T)
{
    BME280_S32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((BME280_S32_t)dig_T1<<1))) * ((BME280_S32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((BME280_S32_t)dig_T1)) * ((adc_T>>4) - ((BME280_S32_t)dig_T1))) >> 12) * ((BME280_S32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}


BME280_U32_t bme280_compensate_H_int32(BME280_S32_t adc_H)
{
    BME280_S32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)dig_H4) << 20) - (((BME280_S32_t)dig_H5) *\
                    v_x1_u32r)) + ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r *\
                        ((BME280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((BME280_S32_t)dig_H3)) >> 11) +\
                        ((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) * ((BME280_S32_t)dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (BME280_U32_t)(v_x1_u32r>>12);
}

BME280_U32_t BME280_compensate_P_int32(BME280_S32_t adc_P)
{
    BME280_S32_t var1, var2;
    BME280_U32_t p;
    var1 = (((BME280_S32_t)t_fine)>>1) - (BME280_S32_t)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((BME280_S32_t)dig_P6);
    var2 = var2 + ((var1*((BME280_S32_t)dig_P5))<<1);
    var2 = (var2>>2)+(((BME280_S32_t)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((BME280_S32_t)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((BME280_S32_t)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = (((BME280_U32_t)(((BME280_S32_t)1048576)-adc_P)-(var2>>12)))*3125;
    if (p < 0x80000000)
    {
        p = (p << 1) / ((BME280_U32_t)var1);
    }
    else
    {
        p = (p / (BME280_U32_t)var1) * 2;
    }
    var1 = (((BME280_S32_t)dig_P9) * ((BME280_S32_t)(((p>>3) * (p>>3))>>13)))>>12;
    var2 = (((BME280_S32_t)(p>>2)) * ((BME280_S32_t)dig_P8))>>13;
    p = (BME280_U32_t)((BME280_S32_t)p + ((var1 + var2 + dig_P7) >> 4));
    return p;
}


















