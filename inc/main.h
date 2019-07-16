/*
 * main.h
 *
 *  Created on: 15 бер. 2019 р.
 *      Author: Alexander
 */

#ifndef MAIN_H_
#define MAIN_H_

#define PWM_PERIOD 2880 //72Mhz / 2880 = 25Khz PWM

#define I2C_OWN_ADDR 0x38 //Can be changed
#define LIGHT_SENSOR_I2C_ADDR  0x23 //Default L address for BH1750FVI sensor
#define LIGHT_SERSOR_MEASUREMENT_TIME 120
#define I2C_TIMEOUT 100

#define MOVEMENT_PIN 1U<<8
#define TOUCH_PIN 1U<<1
#define TICKS_BETWEEN_DIM 1
#define PWM_INCREMENTSIZE 10

#define DEBOUNCE_PERIOD 20
#define LONG_PRESS_PERIOD 700

#define START_WORKHOUR 5
#define END_WORKHOUR 22

#define WORKHOURS f_States & (1U<<5)
#define DIMNEEDED f_States & (1U<<1)
#define SHUTDOWN f_States & (1U)

#define TUNE_TIMEOUT 60000 //Max tuning time is 1 minute

#define 	I2C_DUTYCYCLE_16_9   ((uint16_t)0x4000)
#define 	I2C_DUTYCYCLE_2   ((uint16_t)0xBFFF)

/* Thank you, HAL devs!!! */
#define I2C_CCR_CALCULATION(__PCLK__, __SPEED__, __COEFF__)     (((((__PCLK__) - 1U)/((__SPEED__) * (__COEFF__))) + 1U) & I2C_CCR_CCR)
#define I2C_FREQRANGE(__PCLK__)                            ((__PCLK__)/1000000U)
#define I2C_RISE_TIME(__FREQRANGE__, __SPEED__)            (((__SPEED__) <= 100000U) ? ((__FREQRANGE__) + 1U) : ((((__FREQRANGE__) * 300U) / 1000U) + 1U))
#define I2C_SPEED_STANDARD(__PCLK__, __SPEED__)            ((I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 2U) < 4U)? 4U:I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 2U))
#define I2C_SPEED_FAST(__PCLK__, __SPEED__, __DUTYCYCLE__) (((__DUTYCYCLE__) == I2C_DUTYCYCLE_2)? I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 3U) : (I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 25U) | I2C_DUTYCYCLE_16_9))
#define I2C_SPEED(__PCLK__, __SPEED__, __DUTYCYCLE__)      (((__SPEED__) <= 100000U)? (I2C_SPEED_STANDARD((__PCLK__), (__SPEED__))) : \
                                                                  ((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__)) & I2C_CCR_CCR) == 0U)? 1U : \
                                                                  ((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__))) | I2C_CCR_FS))

#endif /* MAIN_H_ */
