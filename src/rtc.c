#include "stm32f3xx.h"
#include "rtc.h"
#include "main.h"

void DisableRTCProtection(void) {

	/* Enable write access to RTC domain registers */
	PWR->CR |= PWR_CR_DBP;

	/* Write key to the WPR */
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

}

void EnableRTCProtection(void) {

	/* Write wrong key to WRP register */
	RTC->WPR = 0xFF;

	/* Disable write access to RTC domain registers */
	PWR->CR &= ~PWR_CR_DBP;

}
void InitializeRTC(void) {

	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	if (!(RTC->ISR & RTC_ISR_INITS)) {

		/* Enabling write access to RTC domain control registers */
		PWR->CR |= PWR_CR_DBP;

		/* Reset backup */
		RCC->BDCR |= RCC_BDCR_BDRST;
		RCC->BDCR &= ~RCC_BDCR_BDRST;

		/* Enable LSE and use it*/
		RCC->BDCR |= RCC_BDCR_LSEON;

		while (!(RCC->BDCR & RCC_BDCR_LSERDY))
			;

		RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;
		RCC->BDCR |= RCC_BDCR_RTCEN;

		DisableRTCProtection();

		/* Initialize calendar */

		/* Set the INIT bit in the RTC ISR */
		RTC->ISR |= RTC_ISR_INIT;
		while (!(RTC->ISR & RTC_ISR_INITF))
			;

		RTC->PRER = 0x007F00FF;

		/* 24 hour time format */
		RTC->CR &= ~RTC_CR_FMT;

		/* Initialize minimal date and time */
		RTC->TR = (INIT_SECONDS / 10) << RTC_TR_ST_Pos
				| (INIT_SECONDS % 10) << RTC_TR_SU_Pos
				| (INIT_MINUTES / 10) << RTC_TR_MNT_Pos
				| (INIT_MINUTES % 10) << RTC_TR_MNU_Pos
				| (INIT_HOURS / 10) << RTC_TR_HT_Pos
				| (INIT_HOURS % 10) << RTC_TR_HU_Pos;
		RTC->DR = (INIT_DAY / 10) << RTC_DR_DT_Pos
				| (INIT_DAY % 10) << RTC_DR_DU_Pos
				| (INIT_MONTH / 10) << RTC_DR_MT_Pos
				| (INIT_MONTH % 10) << RTC_DR_MU_Pos
				| (INIT_YEAR / 10) << RTC_DR_YT_Pos
				| (INIT_YEAR % 10) << RTC_DR_YU_Pos;

		/* Clear the INIT bit in the RTC ISR */
		RTC->ISR &= ~RTC_ISR_INIT;

		EnableRTCProtection();

	}

}

void SetTime(uint8_t seconds, uint8_t minutes, uint8_t hours) {

	DisableRTCProtection();

	/* Set the INIT bit in the RTC ISR */
	RTC->ISR |= RTC_ISR_INIT;
	while (!(RTC->ISR & RTC_ISR_INITF))
		;

	RTC->TR = (seconds / 10) << RTC_TR_ST_Pos | (seconds % 10) << RTC_TR_SU_Pos
			| (minutes / 10) << RTC_TR_MNT_Pos
			| (minutes % 10) << RTC_TR_MNU_Pos | (hours / 10) << RTC_TR_HT_Pos
			| (hours % 10) << RTC_TR_HU_Pos;

	/* Clear the INIT bit in the RTC ISR */
	RTC->ISR &= ~RTC_ISR_INIT;

	EnableRTCProtection();
}

void SetAlarms(uint8_t start_hour, uint8_t start_minutes, uint8_t end_hour, uint8_t end_minutes) {

	DisableRTCProtection();

	/* Disable Alarms and their interrupts */
	RTC->CR &= ~(RTC_CR_ALRAE | RTC_CR_ALRBE | RTC_CR_ALRAIE | RTC_CR_ALRBIE);

	/* Wait until their registers can be written */
	while (!(RTC->ISR & (RTC_ISR_ALRAWF | RTC_ISR_ALRBWF)))
		;

	/* Start Workhours */
	RTC->ALRMAR = RTC_ALRMAR_MSK4 | ((start_hour / 10) << RTC_ALRMAR_HT_Pos)
			| ((start_hour % 10) << RTC_ALRMAR_HU_Pos)
			| ((start_minutes / 10) << RTC_ALRMAR_MNT_Pos)
			| ((start_minutes % 10) << RTC_ALRMAR_MNU_Pos);

	/* End Workhours */
	RTC->ALRMBR = RTC_ALRMBR_MSK4 | ((end_hour / 10) << RTC_ALRMBR_HT_Pos)
			| ((end_hour % 10) << RTC_ALRMBR_HU_Pos)
			| ((end_minutes / 10) << RTC_ALRMBR_MNT_Pos)
			| ((end_minutes % 10) << RTC_ALRMBR_MNU_Pos);

	/* Enable Alarms and their interrupts */
	RTC->CR |= (RTC_CR_ALRAE | RTC_CR_ALRBE | RTC_CR_ALRAIE | RTC_CR_ALRBIE);

	EnableRTCProtection();

}

uint8_t IsHoursBetweenNow(uint8_t min, uint8_t max) {

	/* Workhours can be only on the same day */
	if (min > max)
		return 0;

	/* Wait until TR and DR will be synced with shadow registers */
	while (!(RTC->ISR & RTC_ISR_RSF))
		;

	/* Read time(TR) register and check data(DR) register
	 * to unlock theese registers for further syncing
	 * with their shadow registers */
	uint32_t TR = RTC->TR;
	(void) RTC->DR;

	/* Since that we accessed calendar registers quickly (less than in two RTCCLK periods), we have to clear RTC_ISR_RSF bit */
	RTC->ISR &= ~RTC_ISR_RSF;

	uint8_t hoursNow = ((TR & RTC_TR_HT_Msk) >> RTC_TR_HT_Pos) * 10
			+ ((TR & RTC_TR_HU_Msk) >> RTC_TR_HU_Pos);

	return hoursNow >= min && hoursNow <= max;

}

