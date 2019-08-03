/*
 * main.h
 *
 *  Created on: 15 бер. 2019 р.
 *      Author: Alexander
 */

#ifndef MAIN_H_
#define MAIN_H_

#define PWM_PERIOD 2880 //72Mhz / 2880 = 25Khz PWM

#define LIGHT_SENSOR_I2C_ADDR  0x23 //Default L address for BH1750FVI sensor
#define LIGHT_SERSOR_MEASUREMENT_TIME 120
#define I2C_TIMEOUT 100

#define MOVEMENT_PIN 1U<<8
#define TOUCH_PIN 1U<<1
#define TICKS_BETWEEN_DIM 1
#define PWM_INCREMENTSIZE 10

#define DEBOUNCE_PERIOD 20
#define LONG_PRESS_PERIOD 700

#define START_WORKHOUR 7
#define END_WORKHOUR 22

#define WORKHOURS f_States & (1U<<5)
#define DIMNEEDED f_States & (1U<<1)
#define SHUTDOWN f_States & (1U)

#define TUNE_TIMEOUT 60000 //Max tuning time is 1 minute

#endif /* MAIN_H_ */
