/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @author  Ac6
 * @version V1.0
 * @date    02-Feb-2015
 * @brief   Default Interrupt Service Routines.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "stm32f3xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32f3xx_it.h"
#include "rtc.h"

#include "main.h"

extern volatile uint8_t f_States;

extern volatile uint16_t uw_PWMTargetWidth;
extern volatile uint8_t ub_TicksBetweenDim;
extern volatile uint16_t uw_TargetLuxes;
extern volatile uint16_t uw_MaxLuxes;

extern void SetPWMPulseWidth(uint32_t width);
extern uint8_t IsWorkHoursNow();
extern void StartDimTo(uint16_t width);

uint16_t ButtonPushedTicks = 0;
uint32_t uw_DimTicks = 0;

volatile uint32_t uwTicks;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint32_t Ticks() {
	return uwTicks;
}

void HAL_SYSTICK_Callback(void) {

	if (!(SHUTDOWN) && (WORKHOURS) && !(DIMNEEDED)) { // We will read movement sensor only if the lamp was not shut down by hand and workhours are now, and we are not changing power

		if (GPIOB->IDR & MOVEMENT_PIN) {

			uw_TargetLuxes = uw_MaxLuxes / 2; /*If someone is near the lamp, set it's brightness to half of the power*/

		} else {

			uw_TargetLuxes = uw_MaxLuxes;

		}

	}

	if (GPIOA->IDR & TOUCH_PIN) {

		ButtonPushedTicks++;

	} else {

		if (ButtonPushedTicks > LONG_PRESS_PERIOD) { //Long touch is the permanent shutdown for the lamp

			SetPWMPulseWidth(PWM_PERIOD); // Light it to max, so the user could understand that the sensor touch worked
			uw_TargetLuxes = 0;
			f_States |= 1UL; // Lamp was shut down permanently
			StartDimTo(0);

		} else if (ButtonPushedTicks > DEBOUNCE_PERIOD) {

			StartDimTo(TIM1->CCR1 ? 0 : PWM_PERIOD); //Flip current on/off state.

			if (uw_PWMTargetWidth) {

				f_States &= ~(1UL); // It was powered on, so clean shutdown flag.
				uw_TargetLuxes = uw_MaxLuxes;

			} else
				uw_TargetLuxes = 0;

		}

		ButtonPushedTicks = 0;
	}

	if ((DIMNEEDED)) { //Regulator started to tune the light

		if ((Ticks() - uw_DimTicks) > ub_TicksBetweenDim) {

			uw_DimTicks = Ticks();

			if ( TIM1->CCR1 > uw_PWMTargetWidth) {

				SetPWMPulseWidth( TIM1->CCR1 - 1);

			} else

				SetPWMPulseWidth( TIM1->CCR1 + 1);

		}

		if (uw_PWMTargetWidth == TIM1->CCR1)

			f_States &= ~(1UL << 1); //We have got to our target PWM width, disable dimming

	}

}

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) {
	uwTicks++;
	HAL_SYSTICK_Callback();
}

void RTC_ALARM_IT_IRQ_IRQHandler(void) {

	EXTI->PR |= EXTI_PR_PR17; /*This bit is cleared by writing a ‘1’ to the bit.*/

	PWR->CR  |= PWR_CR_DBP; /* Enabling write access to RTC registers */

	if ( RTC->ISR & RTC_ISR_ALRAF) { /* Workhours started */

		RTC->ISR &= ~RTC_ISR_ALRAF;

		uw_TargetLuxes = uw_MaxLuxes;

		StartDimTo(PWM_PERIOD);

		f_States |= 1UL << 5; // Set workhours bit
		f_States &= ~1UL; //Clear shutdown bit

	} else if (RTC->ISR & RTC_ISR_ALRBF) { /* Workhours ended */

		RTC->ISR &= ~RTC_ISR_ALRBF;

		uw_TargetLuxes = 0;

		StartDimTo(0);

		f_States &= ~(1UL << 5); // Clear workhours bit

	}

	PWR->CR  &= ~PWR_CR_DBP; /* Disabling write access to RTC registers */

}
