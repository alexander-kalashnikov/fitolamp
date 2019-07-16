/*
 * i2c.c
 *
 *  Created on: 5 черв. 2019 р.
 *      Author: Alexander
 */

#include "stm32f3xx.h"
#include "main.h"
#include "i2c.h"

extern uint32_t Ticks();

volatile uint32_t w_TicksStart;

void InitializeI2C(void) {

	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	I2C1->TIMINGR = (uint32_t) 0x00E0257A; /*Magic number generated via I2C Timing Config Tool by ST*/
	I2C1->CR2 = 0x00;

	I2C1->CR1 |= I2C_CR1_PE;

}

void AddressAndSizeI2C(uint16_t DevAddress, uint8_t IsReading, uint8_t Size) {

	I2C1->CR2 = 0x0;

	I2C1->CR2 |= (Size << 16) | (DevAddress << 1)
			| (IsReading ? I2C_CR2_RD_WRN : 0);

}

uint8_t StartI2C(uint32_t Timeout) {

	if (!(I2C1->CR1 & I2C_CR1_PE))
		I2C1->CR1 |= I2C_CR1_PE;

	I2C1->CR2 |= I2C_CR2_START;
	while (I2C1->CR2 & I2C_CR2_START && (Ticks() - w_TicksStart < Timeout))
		;

	return (Ticks() - w_TicksStart < Timeout) ? RES_OK : RES_TIMEOUT;
}

uint8_t StopI2C(uint32_t Timeout) {

	I2C1->CR2 |= I2C_CR2_STOP;
	while (I2C1->CR2 & I2C_CR2_STOP && (Ticks() - w_TicksStart < Timeout))
		;

	if (I2C1->CR1 & I2C_CR1_PE)
		I2C1->CR1 &= ~I2C_CR1_PE;

	return (Ticks() - w_TicksStart < Timeout) ? RES_OK : RES_TIMEOUT;
}

/* *
 * Will transmit data to DevAddress.
 * */
uint8_t TransmitI2C(uint16_t DevAddress, uint8_t *pData, uint8_t Size,
		uint32_t Timeout) {

	uint8_t * pBuffer = pData;
	uint8_t bToTransfer = Size;
	w_TicksStart = Ticks();

	AddressAndSizeI2C(DevAddress, 0, Size);

	if (StartI2C(Timeout) == RES_OK) {

		while (bToTransfer-- > 0) {

			I2C1->TXDR = (*pBuffer++);

			while (!(I2C1->ISR & I2C_ISR_TXE)
					&& (Ticks() - w_TicksStart < Timeout))
				;

		}

		StopI2C(Timeout);

	}

	return (Ticks() - w_TicksStart < Timeout) ? RES_OK : RES_TIMEOUT;
}

/* *
 * Will receive data from  DevAddress.
 * */
uint8_t ReceiveI2C(uint16_t DevAddress, uint8_t *pData, uint8_t Size,
		uint32_t Timeout) {

	uint8_t * pBuffer = pData;
	uint8_t bToReceive = Size;
	w_TicksStart = Ticks();

	AddressAndSizeI2C(DevAddress, 1, Size);

	if (StartI2C(Timeout) == RES_OK) {
		pBuffer += Size - 1;
		while (bToReceive-- > 0) {

			while (!(I2C1->ISR & I2C_ISR_RXNE)
					&& (Ticks() - w_TicksStart < Timeout))
				;

			(*pBuffer--) = I2C1->RXDR;

		}

		StopI2C(Timeout);
	}

	return (Ticks() - w_TicksStart < Timeout) ? RES_OK : RES_TIMEOUT;

}
