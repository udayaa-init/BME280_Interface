/*
 * BME_280.h
 *
 *  Created on: Nov 1, 2022
 *      Author: Udaya
 */

#ifndef INC_BME_280_H_
#define INC_BME_280_H_
#include "stm32f0xx_hal.h"
// functions needed

//measurement
void Measure();
//configuration
void BME_Config(uint8_t osrs_p, uint8_t osrs_t,uint8_t osrs_h,uint8_t mode, uint8_t filter , uint8_t standby);
//compensation based on 32 or 64 bit: taken from user manual
// parameter reading/ calibration values stored in NVM






// filter coefficient
#define FILTER_OFF  0x00
#define FILTER_2    0x01
#define FILTER_4    0x02
#define FILTER_8    0x03
#define FILTER_16   0x04

// Mode selection
#define SLEEP   0x00
#define NORMAL  0x03
#define Forced  0x02 // or 0x01

// t_sb settings standby mode
#define MS0_5   0x00 //0.5ms
#define MS62_5  0x01 //62.5ms
#define MS125   0x02 //125ms
#define MS250   0x03 //250ms
#define MS500   0x04 //500ms
#define MS1000  0x05 //1000ms
#define MS10    0x06 //10ms
#define MS20    0x07 //20ms

// Oversampling
#define OSRS_OFF 0x00
#define OSRS_1 0x01
#define OSRS_2 0x02
#define OSRS_4 0x03
#define OSRS_8 0x04
#define OSRS_16 0x05


//memoeryyyyyyyyy

#define ID      0xD0
#define RESET   0xE0
#define STATUS  0xF3
#define CONFIG  0xF5
#define CTRL_HUM   0xF2
#define CTRL_MEAS   0xF4
#define PRES_MSB   0xF7
#define PRES_LSB   0xF8



#endif /* INC_BME_280_H_ */
