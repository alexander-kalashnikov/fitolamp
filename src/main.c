#include "stm32f3xx.h"
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/**
 * f_States: state flags.
 * bit 0: 1 - Lamp was shut down permanently
 * bit 1: 1 - Slow dimming required
 * bit 3: 1 - Command received via USART
 * bit 5: 1 - Now are the working hours
 **/
volatile uint8_t f_States = 0;

volatile uint16_t uw_CurrentLuxes = 0;
volatile uint16_t uw_TargetLuxes = 0;
volatile uint16_t uw_PWMTargetWidth = 0;
volatile uint16_t uw_MaxLuxes = 0;
volatile uint8_t ub_PWMIncrementSize = PWM_INCREMENTSIZE;
volatile uint8_t ub_TicksBetweenDim = TICKS_BETWEEN_DIM;
volatile char cUSARTBuffer[32] = { 0 };
volatile extern uint32_t uwTicksUSARTLastReceive;
const char * cCommands[] = { "getdate", "setdate=", "getalarms", "setalarms=",
		"poweron", "poweroff", "getlux", "getmaxlux", "setmaxlux=" };

extern uint32_t Ticks();

uint8_t b_QuickIncrementAmmount = 34;
uint8_t b_SlowIncrementAmmount = 8;

void Delay(uint32_t Delay) {
	uint32_t tickstart = Ticks();
	uint32_t wait = Delay;
	while ((Ticks() - tickstart) < wait)
		;
}

void StartDimTo(uint16_t width) {

	if (TIM1->CCR1 != width) {

		f_States |= 1UL << 1;
		uw_PWMTargetWidth = width;

	}

}

uint8_t IsWorkHoursNow(void) {

	return IsHoursBetweenNow(START_WORKHOUR, END_WORKHOUR);

}

void InitializeInterrupts(void) {

	/*
	 * To enable the RTC Alarm interrupt, the following sequence is required:
	 * 1.Configure and enable the NVIC line corresponding to the RTC Alarm event in interrupt mode and select the rising edge sensitivity.
	 * 2. Configure and enable the RTC_ALARM IRQ channel in the NVIC.
	 * 3. Configure the RTC to generate RTC alarms.
	 * */
	/* Enabling Line 17 (RTC Alarm) EXTI rising edge */
	EXTI->RTSR |= EXTI_RTSR_TR17;

	/* Unmasking Line 17 (RTC Alarm) EXTI interrupt */
	EXTI->IMR |= EXTI_IMR_MR17;

	/* Enabling external Line 17 interrupt that uses RTC_Alarm_IRQn NVIC vector */
	NVIC_EnableIRQ(RTC_Alarm_IRQn);

}

void InitializeLightSersor(void) {

	uint8_t command = 0x01; //Power On command

	TransmitI2C((LIGHT_SENSOR_I2C_ADDR), &command, 1, I2C_TIMEOUT);

	command = 0x07; //Reset command

	TransmitI2C((LIGHT_SENSOR_I2C_ADDR), &command, 1, I2C_TIMEOUT);

	command = 0x11; //H-Resolution Continuous

	TransmitI2C((LIGHT_SENSOR_I2C_ADDR), &command, 1, I2C_TIMEOUT);
}

uint16_t ReadLightSensorLuxes(void) {

	uint16_t readings = 0xFFFF;

	ReceiveI2C((LIGHT_SENSOR_I2C_ADDR), (uint8_t *) &readings, sizeof(readings),
	I2C_TIMEOUT);

	return readings;

}
void StartContinuousHResolution(void) {
	uint8_t command = 0x11;
	TransmitI2C((LIGHT_SENSOR_I2C_ADDR), &command, 1, I2C_TIMEOUT);
}

void StartContinuousLResolution(void) {
	uint8_t command = 0x13;
	TransmitI2C((LIGHT_SENSOR_I2C_ADDR), &command, 1, I2C_TIMEOUT);
}

uint16_t GetLux() {

	uint16_t luxes = ReadLightSensorLuxes();
	if (luxes < 0xFFFF)
		luxes /= 1.2;
	return luxes;

}

void InitializeGPIO() {

	/* A and B ports clock */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOAEN;

	/* Configure GPIOA pins*/

	/* AF mode for PA6, PA8, PA9, PA10, PA11 */

	GPIOA->MODER = 0xA8000000 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER8_1
			| GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;

	/* High speed for PA6, PA8, PA9, PA10, PA11 and PA1 for touch sensor*/
	GPIOA->OSPEEDR = 0xC0000000 | GPIO_OSPEEDER_OSPEEDR1_1
			| GPIO_OSPEEDER_OSPEEDR1_0 | GPIO_OSPEEDER_OSPEEDR6_1
			| GPIO_OSPEEDER_OSPEEDR6_0 | GPIO_OSPEEDER_OSPEEDR8_1
			| GPIO_OSPEEDER_OSPEEDR8_0 | GPIO_OSPEEDER_OSPEEDR9_1
			| GPIO_OSPEEDER_OSPEEDR9_0 | GPIO_OSPEEDER_OSPEEDR10_1
			| GPIO_OSPEEDER_OSPEEDR10_0 | GPIO_OSPEEDER_OSPEEDR11_1
			| GPIO_OSPEEDER_OSPEEDR11_0;

	/* Output push-pull type for PA6, PA8, PA9, PA10, PA11 configured via reset value*/
	GPIOA->OTYPER = 0x00000000;

	/* Pull down for PA6, PA8, PA9, PA10, PA11 and PA1 for touch sensor */
	GPIOA->PUPDR = 0x64000000 | GPIO_PUPDR_PUPDR1_1 | GPIO_PUPDR_PUPDR6_1
			| GPIO_PUPDR_PUPDR8_1 | GPIO_PUPDR_PUPDR9_1 | GPIO_PUPDR_PUPDR10_1
			| GPIO_PUPDR_PUPDR11_1;

	GPIOA->AFR[0] = 1U << 24U; // PA6: AF1
	GPIOA->AFR[1] = 6U | (6U << 4U) | (6U << 8U) | (11U << 12U); // PA8, PA9, PA10 : AF6, PA11: AF11

	/* Configure GPIOB pins */

	/* AF mode for SCL and SDA and input mode for others */
	GPIOB->MODER = 0x00000280 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;

	GPIOB->OSPEEDR = 0x000000C0 | GPIO_OSPEEDER_OSPEEDR6_1
			| GPIO_OSPEEDER_OSPEEDR6_0 | GPIO_OSPEEDER_OSPEEDR7_1
			| GPIO_OSPEEDER_OSPEEDR7_0 | GPIO_OSPEEDER_OSPEEDR8_1
			| GPIO_OSPEEDER_OSPEEDR8_0;
	GPIOB->OTYPER = 0x00000000;
	GPIOB->AFR[0] = (4U << 24U) | (4U << 28U); // PB6, PB7: AF4 for SLC and SDA

	/* Configure movement sensor input pin (pull-up) all others are no pull up/pull down*/
	GPIOB->PUPDR = 0x00000100 | GPIO_PUPDR_PUPDR8_0;

	/* USART as AF mode for TX and RX */
	GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;

	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_1 | GPIO_OSPEEDER_OSPEEDR10_0
			| GPIO_OSPEEDER_OSPEEDR11_1 | GPIO_OSPEEDER_OSPEEDR11_0;

	GPIOB->AFR[1] |= (7U << GPIO_AFRH_AFRH2_Pos) | (7U << GPIO_AFRH_AFRH3_Pos); // PB10, PB11: AF7 for RX and TX

	/* Configure all pins with no pull up/pull down, but pull up PB10 (TX)*/
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0;

}

void InitializePWMs() {

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM16EN; //TIM1 and TIM16

	/*TIM1 initialization and start*/

	TIM1->PSC = 0; //Prescaler
	TIM1->ARR = PWM_PERIOD; //PWM period

	/*Initial PWM duty cycle*/
	TIM1->CCR1 = uw_PWMTargetWidth;
	TIM1->CCR2 = uw_PWMTargetWidth;
	TIM1->CCR3 = uw_PWMTargetWidth;
	TIM1->CCR4 = uw_PWMTargetWidth;

	/* Enable all channels, positive polarity */
	TIM1->CCER = TIM_CCER_CC4E | TIM_CCER_CC3E | TIM_CCER_CC2E | TIM_CCER_CC1E;

	/* Enabling main output */
	TIM1->BDTR |= TIM_BDTR_MOE;

	/* PWM mode 1, four channels, preload enable */
	TIM1->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC3M_2
			| TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
	TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2
			| TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

	/*Count Forward, Front, Fast PWM, Start timer*/
	TIM1->CR1 = TIM_CR1_CEN;

	/*TIM16 initialization and start*/

	TIM16->PSC = 0; //Prescaler
	TIM16->ARR = PWM_PERIOD; //PWM period

	/*Initial PWM duty cycle*/
	TIM16->CCR1 = uw_PWMTargetWidth;

	/* Enable first channel output, positive polarity */
	TIM16->CCER = TIM_CCER_CC1E;

	/* Enabling main output */
	TIM16->BDTR = TIM_BDTR_MOE;

	/*PWM mode 1, first channel, preload enable*/
	TIM16->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;

	/*Count Forward, Front, Fast PWM, Start timer*/
	TIM16->CR1 = TIM_CR1_CEN;

}

void SetPWMPulseWidth(uint32_t width) {

	/*
	 *  CCR preload enabled,
	 * so all changes will be
	 * copied into corresponding
	 * registers at the following
	 * update events:
	 * – Counter overflow/underflow
	 * – Setting the UG bit
	 * – Update generation through the slave mode controller
	 *
	 * */
	TIM1->CCR1 = width;
	TIM1->CCR2 = width;
	TIM1->CCR3 = width;
	TIM1->CCR4 = width;
	TIM16->CCR1 = width;

}

void Callibrate(uint8_t QuickCalibration) {

	uint16_t * Luxes = 0;
	uint32_t b_DataCollectedQuick = 0;
	uint32_t b_DataCollectedItemsQuick = 0;
	uint32_t b_DataCollectedSlow = 0;
	uint32_t b_DataCollectedItemsSlow = 0;

	f_States |= 1;

	/* Get max luxes with 100% PWM width */
	SetPWMPulseWidth(PWM_PERIOD);

	/* Get measurement three times*/
	Delay(LIGHT_SERSOR_MEASUREMENT_TIME);
	uw_MaxLuxes = GetLux();
	Delay(LIGHT_SERSOR_MEASUREMENT_TIME);
	uw_MaxLuxes += GetLux();
	Delay(LIGHT_SERSOR_MEASUREMENT_TIME);
	uw_MaxLuxes += GetLux();

	/* Calculate arithmetic mean of all theese measurements */
	uw_MaxLuxes /= 3;

	if (!QuickCalibration) {

		Luxes = (uint16_t *) malloc(sizeof(uint16_t) * (PWM_PERIOD / 20 + 1));

		for (uint16_t i = PWM_PERIOD / 20; i != 0xFFFF; i--) {

			SetPWMPulseWidth(i * 20);
			Delay(LIGHT_SERSOR_MEASUREMENT_TIME);
			Luxes[i] = GetLux();

		};

		for (uint16_t i = 0; i <= PWM_PERIOD / 20; i++) {

			for (uint16_t j = i + 1; j <= PWM_PERIOD / 20; j++) {

				if (Luxes[j] - Luxes[i] >= 3 && Luxes[j] - Luxes[i] <= 5) {

					b_DataCollectedItemsSlow++;
					b_DataCollectedSlow += j - i;

				}

				if (Luxes[j] - Luxes[i] >= 199) {

					b_DataCollectedItemsQuick++;
					b_DataCollectedQuick += j - i;

				}

			}

		}

		b_QuickIncrementAmmount =
				b_DataCollectedItemsQuick ?
						b_DataCollectedQuick / b_DataCollectedItemsQuick : 100;
		b_SlowIncrementAmmount =
				b_DataCollectedItemsSlow ?
						b_DataCollectedSlow / b_DataCollectedItemsSlow : 1;
		free(Luxes);
	}

	f_States &= ~1;

}

void InitializeUSART() {
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	USART3->BRR = 3750; /* 9600 */
	USART3->CR3 |= USART_CR3_OVRDIS;
	USART3->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	USART3->CR1 |= USART_CR1_RXNEIE;
	NVIC_EnableIRQ(USART3_IRQn);
}

int8_t USART_Wait_For(char * cString) {

	uint32_t ticks = Ticks();
	int8_t i = -1;

	while (!(USART_RECEIVED))
		if (Ticks() - ticks > USART_TIMEOUT * 50) {
			return i;
		}

	while ((++i < sizeof(cUSARTBuffer)) && cString[i]
			&& (cUSARTBuffer[i] == cString[i]))
		; /* Awful but fast */

	return i;
}

int8_t USART_Send(char * chr) {

	uint32_t ticks = 0;
	uint8_t len = strlen(chr);

	do {

		ticks = Ticks();

		while (!(USART3->ISR & USART_ISR_TXE))
			if (Ticks() - ticks > USART_TIMEOUT)
				return len;

		USART3->TDR = *chr++;

	} while (--len > 0);

	ticks = Ticks();

	while (!(USART3->ISR & USART_ISR_TC) && (Ticks() - ticks < USART_TIMEOUT))
		;

	return 0;

}

void USART_Clear_All(void) {
	f_States &= ~(1U << 3); /* Clear USART received flag */
	cUSARTBuffer[0] = 0; /* "Clear" USART receive buffer */
}

uint8_t USART_BT_Connected() {
	Delay(1000);
	USART_Send("AT");
	USART_Clear_All();
	if (USART_Wait_For("OK") == 2) {
		USART_Send("AT+NAMEFITO-LAMP");
		USART_Clear_All();
		if (USART_Wait_For("OK") == 2) {
			USART_Send("AT+PIN5625");
			USART_Clear_All();
			if (USART_Wait_For("OK") == 2) {
			} else
				return 0; /* A bit of indian code */
		} else
			return 0;
	} else
		return 0;

	return 1;
}

int USART_BT_Parse_Command(char * cParameter) {

	int i = -1;
	char * cCmdStart = 0;

	if (USART_RECEIVED) {

		while (++i < sizeof(cCommands) / sizeof(*cCommands)) {

			if ((cCmdStart = strstr((const char *) cUSARTBuffer, cCommands[i]))) { /* OMFG */
				strcpy(cParameter, cCmdStart + strlen(cCommands[i])); /* OMG */
				break;
			}

		}

		USART_Clear_All();

	}

	return i;
}

void InitializeBT(void) {
	if (!USART_BT_Connected()) {
		NVIC_DisableIRQ(USART3_IRQn);
		USART3->CR1 = 0;
		RCC->APB1ENR &= ~(RCC_APB1ENR_USART3EN);
	}
}

int main(void) {

	SysTick_Config(SystemCoreClock / (1000U) - 1UL);
	InitializeInterrupts();
	InitializeGPIO();
	InitializeI2C();
	InitializeLightSersor();
	InitializePWMs();
	InitializeRTC();
	InitializeUSART();
	InitializeBT();

	/*SetTime(0,33,14);*/

	SetAlarms(START_WORKHOUR, 0, END_WORKHOUR, 0);

	/*	uint16_t i = 0;
	 int8_t i_ = 1;

	 while(1) {

	 SetPWMPulseWidth(i);
	 Delay(5);
	 if(i==PWM_PERIOD) { i_=-1; Delay(1000);}
	 if(i==0) {i_=1; Delay(1000);}

	 i+=i_;

	 }*/

	Callibrate(0);

	/* We just started, so we need to know if the lamp has to be working right now */
	if (IsWorkHoursNow()) {

		f_States |= 1UL << 5; // Set workhours bit
		uw_TargetLuxes = uw_MaxLuxes;
		StartDimTo(PWM_PERIOD);

	} else {

		uw_TargetLuxes = 0;
		StartDimTo(0);
		f_States &= ~(1UL << 5); // Clear workhours bit

	}

	int16_t w_LuxDiff = 0;
	uint32_t dw_TicksWhenStartedToTune = 0;
	char sCommandParameters[32] = { 0 };
	char sSeconds[3] = { 0 }, sMinutes[3] = { 0 }, sHours[3] = { 0 };
	uint8_t bSeconds = 0, bMinutes = 0, bHours = 0;

	uint8_t bSM = 0, bSH = 0, bEH = 0, bEM = 0;
	char sSM[3] = { 0 }, sSH[3] = { 0 }, sEH[3] = { 0 }, sEM[3] = { 0 };

	for (;;) {

		/* USART Commands */
		switch (USART_BT_Parse_Command(sCommandParameters)) {
		case SETDATE: {

			if (strlen(sCommandParameters) >= 7) {

				sHours[0] = sCommandParameters[0];
				sHours[1] = sCommandParameters[1];
				sHours[2] = 0;
				sMinutes[0] = sCommandParameters[3];
				sMinutes[1] = sCommandParameters[4];
				sMinutes[2] = 0;
				sSeconds[0] = sCommandParameters[6];
				sSeconds[1] = sCommandParameters[7];
				sSeconds[2] = 0;

				bHours = atoi(sHours);
				bMinutes = atoi(sMinutes);
				bSeconds = atoi(sSeconds);

				SetTime(bSeconds, bMinutes, bHours);

			}

		}
			;
		case GETDATE: {
			GetTime(&bSeconds, &bMinutes, &bHours);
			sprintf(sCommandParameters, "DATE=%02u:%02u:%02u\r\n",
					(uint16_t) bHours, (uint16_t) bMinutes,
					(uint16_t) bSeconds);
			USART_Send(sCommandParameters);
			break;
		}
			;

		case SETALARMS: {
			if (strlen(sCommandParameters) >= 9) {

				sSH[0] = sCommandParameters[0];
				sSH[1] = sCommandParameters[1];
				sSH[2] = 0;
				sSM[0] = sCommandParameters[2];
				sSM[1] = sCommandParameters[3];
				sSM[2] = 0;
				sEH[0] = sCommandParameters[5];
				sEH[1] = sCommandParameters[6];
				sEH[2] = 0;
				sEM[0] = sCommandParameters[7];
				sEM[1] = sCommandParameters[8];
				sEM[2] = 0;
				bSH = atoi(sSH);
				bSM = atoi(sSM);
				bEH = atoi(sEH);
				bEM = atoi(sEM);
				SetAlarms(bSH, bSM, bEH, bEM);
			}

		}
			;
		case GETALARMS: {

			GetAlarms(&bSH, &bSM, &bEH, &bEM);
			sprintf(sCommandParameters, "ALARMS=%02u%02u:%02u%02u\r\n", bSH,
					bSM, bEH, bEM);
			USART_Send(sCommandParameters);

			break;
		}
			;
		case POWERON: {
			f_States &= ~(1UL); // It was powered on, so clean shutdown flag.
			uw_TargetLuxes = uw_MaxLuxes;
			break;
		}
			;
		case POWEROFF: {
			uw_TargetLuxes = 0;
			f_States |= 1UL; // Lamp was shut down permanently
			StartDimTo(0);
			break;
		}
			;
		case GETLUX: {
			sprintf(sCommandParameters, "LUX=%u\r\n", uw_CurrentLuxes);
			USART_Send(sCommandParameters);
			break;
		}
			;
		case SETMAXLUX: {
			if (strlen(sCommandParameters) >= 1) {
				uw_MaxLuxes = strtol(sCommandParameters, NULL, 10);
			}
		}
			;
		case GETMAXLUX: {
			sprintf(sCommandParameters, "MAXLUX=%u\r\n", uw_MaxLuxes);
			USART_Send(sCommandParameters);
			break;
		}
			;
		default: {
			break;
		}
			;
		}

		/* Lux Regulator */
		if (!(DIMNEEDED)) { //We should check and correct light only if dimming was finished

			Delay(LIGHT_SERSOR_MEASUREMENT_TIME);
			uw_CurrentLuxes = GetLux();

			if (uw_CurrentLuxes < 65535) { // If LuxMeter works

				w_LuxDiff = uw_CurrentLuxes - uw_TargetLuxes;

				if (!uw_TargetLuxes) {

					dw_TicksWhenStartedToTune = 0;
					StartDimTo(0);

				} else if (abs(w_LuxDiff) > 20) { // We will continue to tune the light until we got into 20 Luxes window

					if (!dw_TicksWhenStartedToTune) {

						dw_TicksWhenStartedToTune = Ticks();

					}

					if (Ticks() - dw_TicksWhenStartedToTune > TUNE_TIMEOUT
							&& uw_TargetLuxes) {
						dw_TicksWhenStartedToTune = 0;
						Callibrate(1);
						continue;

					}

					ub_PWMIncrementSize =
							abs(w_LuxDiff) > 200 ?
									b_QuickIncrementAmmount :
									b_SlowIncrementAmmount;
					ub_TicksBetweenDim = ub_PWMIncrementSize > 1 ? 5 : 2; //Theese numbers are almost identical to time, so we will reuse them as delays between dim

					if (w_LuxDiff < 0) {

						if (TIM1->CCR1 + ub_PWMIncrementSize < PWM_PERIOD)
							StartDimTo(TIM1->CCR1 + ub_PWMIncrementSize);

					} else {

						if (TIM1->CCR1 > ub_PWMIncrementSize)
							StartDimTo(TIM1->CCR1 - ub_PWMIncrementSize);

					}
				} else
					dw_TicksWhenStartedToTune = 0;
			}
		}
	};
}
