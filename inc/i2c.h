/*
 * i2c.h
 *
 *  Created on: 5 черв. 2019 р.
 *      Author: Alexander
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#define RES_OK 0
#define RES_TIMEOUT 1

#define SENSOR_TIMEOUT 100

void InitializeI2C(void);

void AddressAndSizeI2C(uint16_t DevAddress, uint8_t IsReading, uint8_t Size);
uint8_t StartI2C(uint32_t Timeout);
uint8_t StopI2C(uint32_t Timeout);
/* *
 * Will transmit data to DevAddress.
 * */
uint8_t TransmitI2C(uint16_t DevAddress, uint8_t *pData, uint8_t Size,
		uint32_t Timeout);
/* *
 * Will receive data from  DevAddress.
 * */
uint8_t ReceiveI2C(uint16_t DevAddress, uint8_t *pData, uint8_t Size,
		uint32_t Timeout) ;

#endif /* INC_I2C_H_ */
