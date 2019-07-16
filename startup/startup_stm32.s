/**
  ******************************************************************************
  * @file      startup_stm32.s dedicated to STM32F301C8Tx device
  * @author    Ac6
  * @version   V1.0.0
  * @date      2019-07-02
  ******************************************************************************
  */


.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit
  
/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call the clock system intitialization function.*/
  bl  SystemInit
/* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl main

LoopForever:
    b LoopForever


.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler
/******************************************************************************
*
* The STM32F301C8Tx vector table.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
   .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors


g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word	MemManage_Handler
  .word	BusFault_Handler
  .word	UsageFault_Handler
  .word	0
  .word	0
  .word	0
  .word	0
  .word	SVC_Handler
  .word	DebugMon_Handler
  .word	0
  .word	PendSV_Handler
  .word	SysTick_Handler
  .word	WWDG_IRQ_IRQHandler          			/* Window Watchdog interrupt                                         */
  .word	PVD_IRQ_IRQHandler           			/* Power voltage detector through EXTI line detection interrupt      */
  .word	TAMP_IRQHandler              			/* Tamper and timestamp through EXTI19 line                          */
  .word	RTC_WKUP_IRQ_IRQHandler      			/* RTC                                                               */
  .word	FLASH_IRQHandler             			/* Flash global interrupt                                            */
  .word	RCC_IRQHandler               			/* RCC global interrupt                                              */
  .word	EXTI0_IRQ_IRQHandler         			/* EXTI Line 0 interrupt                                             */
  .word	EXTI1_IRQ_IRQHandler         			/* EXTI Line1 interrupt                                              */
  .word	EXTI2_RI_IRQ_IRQHandler      			/* EXTI Line 2 and routing interface interrupt                       */
  .word	EXTI3_IRQ_IRQHandler         			/* EXTI Line1 interrupt                                              */
  .word	EXTI4_IRQ_IRQHandler         			/* EXTI Line4 interrupt                                              */
  .word	DMA1_CH1_IRQHandler          			/* DMA1 channel 1 interrupt                                          */
  .word	DMA1_CH2_IRQHandler          			/* DMA1 channel 2 interrupt                                          */
  .word	DMA1_CH3_IRQHandler          			/* DMA1 channel 3 interrupt                                          */
  .word	DMA1_CH4_IRQHandler          			/* DMA1 channel 4 interrupt                                          */
  .word	DMA1_CH5_IRQHandler          			/* DMA1 channel 5 interrupt                                          */
  .word	DMA1_CH6_IRQHandler          			/* DMA1 channel 6 interrupt                                          */
  .word	DMA1_CH7_IRQHandler          			/* DMA1 channel 7 interrupt                                          */
  .word	ADC1_IRQ_IRQHandler          			/* ADC1 interrupt                                                    */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	EXTI5_9_IRQ_IRQHandler       			/* EXTI Line[9:5] interrupts                                         */
  .word	TIM15_IRQ_IRQHandler         			/* Timer 15 global interrupt                                         */
  .word	TIM16_IRQ_IRQHandler         			/* Timer 16 global interrupt                                         */
  .word	TIM17_IRQ_IRQHandler         			/* Timer 17 global interrupt                                         */
  .word	0                            			/* Reserved                                                          */
  .word	TIM2_IRQHandler              			/* Timer 2 global interrupt                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	I2C1_EV_IRQ_IRQHandler       			/* I2C1_EV global interrupt/EXTI Line[3:2] interrupts                */
  .word	I2C1_ER_IRQ_IRQHandler       			/* I2C1_ER                                                           */
  .word	I2C2_EV_IRQ_IRQHandler       			/* I2C2_EV global interrupt/EXTI Line[4:2] interrupts                */
  .word	I2C2_ER_IRQ_IRQHandler       			/* I2C2_ER                                                           */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	USART1_IRQ_IRQHandler        			/* USART1 global interrupt/EXTI25 (USART1 wakeup event)              */
  .word	USART2_IRQ_IRQHandler        			/* USART2 global interrupt/EXTI26 (USART1 wakeup event)              */
  .word	USART3_IRQ_IRQHandler        			/* USART3 global interrupt/EXTI28 (USART1 wakeup event)              */
  .word	EXTI15_10_IRQ_IRQHandler     			/* EXTI Line[15:10] interrupts                                       */
  .word	RTC_ALARM_IT_IRQ_IRQHandler  			/* RTC alarm interrupt                                               */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	TIM6_DAC1_IRQHandler         			/* TIM6 global, DAC1 Cahnnel1 and Cahnnel2 underrun error Interrupts */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	COMP2_IRQ_IRQHandler         			/* COMP2 interrupt combined with EXTI Lines                          */
  .word	COMP4_6_IRQ_IRQHandler       			/* COMP4 & COMP6 interrupts combined with                            */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	I2C3_EV_EXTI27_IRQ_IRQHandler			/* I2C3 event interrupt & EXTI Line27 interrupt                      */
  .word	I2C3_ER_IRQ_IRQHandler       			/* I2C3 error interrupt                                              */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	0                            			/* Reserved                                                          */
  .word	FPU_IRQHandler               			/* Floating point interrupt                                          */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  	.weak	NMI_Handler
	.thumb_set NMI_Handler,Default_Handler

  	.weak	HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler

  	.weak	MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler

  	.weak	BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler

	.weak	UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler

	.weak	SVC_Handler
	.thumb_set SVC_Handler,Default_Handler

	.weak	DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler
	
	.weak	PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler

	.weak	SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler

	.weak	WWDG_IRQ_IRQHandler
	.thumb_set WWDG_IRQ_IRQHandler,Default_Handler
	
	.weak	PVD_IRQ_IRQHandler
	.thumb_set PVD_IRQ_IRQHandler,Default_Handler
	
	.weak	TAMP_IRQHandler
	.thumb_set TAMP_IRQHandler,Default_Handler
	
	.weak	RTC_WKUP_IRQ_IRQHandler
	.thumb_set RTC_WKUP_IRQ_IRQHandler,Default_Handler
	
	.weak	FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,Default_Handler
	
	.weak	RCC_IRQHandler
	.thumb_set RCC_IRQHandler,Default_Handler
	
	.weak	EXTI0_IRQ_IRQHandler
	.thumb_set EXTI0_IRQ_IRQHandler,Default_Handler
	
	.weak	EXTI1_IRQ_IRQHandler
	.thumb_set EXTI1_IRQ_IRQHandler,Default_Handler
	
	.weak	EXTI2_RI_IRQ_IRQHandler
	.thumb_set EXTI2_RI_IRQ_IRQHandler,Default_Handler
	
	.weak	EXTI3_IRQ_IRQHandler
	.thumb_set EXTI3_IRQ_IRQHandler,Default_Handler
	
	.weak	EXTI4_IRQ_IRQHandler
	.thumb_set EXTI4_IRQ_IRQHandler,Default_Handler
	
	.weak	DMA1_CH1_IRQHandler
	.thumb_set DMA1_CH1_IRQHandler,Default_Handler
	
	.weak	DMA1_CH2_IRQHandler
	.thumb_set DMA1_CH2_IRQHandler,Default_Handler
	
	.weak	DMA1_CH3_IRQHandler
	.thumb_set DMA1_CH3_IRQHandler,Default_Handler
	
	.weak	DMA1_CH4_IRQHandler
	.thumb_set DMA1_CH4_IRQHandler,Default_Handler
	
	.weak	DMA1_CH5_IRQHandler
	.thumb_set DMA1_CH5_IRQHandler,Default_Handler
	
	.weak	DMA1_CH6_IRQHandler
	.thumb_set DMA1_CH6_IRQHandler,Default_Handler
	
	.weak	DMA1_CH7_IRQHandler
	.thumb_set DMA1_CH7_IRQHandler,Default_Handler
	
	.weak	ADC1_IRQ_IRQHandler
	.thumb_set ADC1_IRQ_IRQHandler,Default_Handler
	
	.weak	EXTI5_9_IRQ_IRQHandler
	.thumb_set EXTI5_9_IRQ_IRQHandler,Default_Handler
	
	.weak	TIM15_IRQ_IRQHandler
	.thumb_set TIM15_IRQ_IRQHandler,Default_Handler
	
	.weak	TIM16_IRQ_IRQHandler
	.thumb_set TIM16_IRQ_IRQHandler,Default_Handler
	
	.weak	TIM17_IRQ_IRQHandler
	.thumb_set TIM17_IRQ_IRQHandler,Default_Handler
	
	.weak	TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler
	
	.weak	I2C1_EV_IRQ_IRQHandler
	.thumb_set I2C1_EV_IRQ_IRQHandler,Default_Handler
	
	.weak	I2C1_ER_IRQ_IRQHandler
	.thumb_set I2C1_ER_IRQ_IRQHandler,Default_Handler
	
	.weak	I2C2_EV_IRQ_IRQHandler
	.thumb_set I2C2_EV_IRQ_IRQHandler,Default_Handler
	
	.weak	I2C2_ER_IRQ_IRQHandler
	.thumb_set I2C2_ER_IRQ_IRQHandler,Default_Handler
	
	.weak	USART1_IRQ_IRQHandler
	.thumb_set USART1_IRQ_IRQHandler,Default_Handler
	
	.weak	USART2_IRQ_IRQHandler
	.thumb_set USART2_IRQ_IRQHandler,Default_Handler
	
	.weak	USART3_IRQ_IRQHandler
	.thumb_set USART3_IRQ_IRQHandler,Default_Handler
	
	.weak	EXTI15_10_IRQ_IRQHandler
	.thumb_set EXTI15_10_IRQ_IRQHandler,Default_Handler
	
	.weak	RTC_ALARM_IT_IRQ_IRQHandler
	.thumb_set RTC_ALARM_IT_IRQ_IRQHandler,Default_Handler
	
	.weak	TIM6_DAC1_IRQHandler
	.thumb_set TIM6_DAC1_IRQHandler,Default_Handler
	
	.weak	COMP2_IRQ_IRQHandler
	.thumb_set COMP2_IRQ_IRQHandler,Default_Handler
	
	.weak	COMP4_6_IRQ_IRQHandler
	.thumb_set COMP4_6_IRQ_IRQHandler,Default_Handler
	
	.weak	I2C3_EV_EXTI27_IRQ_IRQHandler
	.thumb_set I2C3_EV_EXTI27_IRQ_IRQHandler,Default_Handler
	
	.weak	I2C3_ER_IRQ_IRQHandler
	.thumb_set I2C3_ER_IRQ_IRQHandler,Default_Handler
	
	.weak	FPU_IRQHandler
	.thumb_set FPU_IRQHandler,Default_Handler
	
	.weak	SystemInit

/************************ (C) COPYRIGHT Ac6 *****END OF FILE****/
