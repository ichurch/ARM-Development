/**
 ******************************************************************************
 * @file           : i2c1_gpioa_init.s
 * @author         : Ian Church
 * @brief          : itoa
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

.syntax				unified
.cpu cortex-m4 @ STM32L432KC

.thumb
.align

.include "../../include/Inc/stm32l432kc.i"
.section .text

@
@ Initialize GPIOs
@
@
@ Initialize GPIO - Alternate function
@
.globl i2c_gpioa_init
.type i2c_gpioa_init, %function
i2c_gpioa_init:
	@ Enable clock to GPIOA
	ldr		r0, =RCC_AHB2ENR
	ldr		r1, [r0]
	orr		r1, #RCC_AHB2ENR_GPIOA_EN
	str		r1, [r0]

	@ Set OTYPE
	ldr		r0, =GPIOA_OTYPER
	ldr		r1, [r0]
	orr		r1, #GPIOx_ODR_OT9
	orr		r1, #GPIOx_ODR_OT10
	str		r1, [r0]

	@ Set OSPEED
	ldr		r0, =GPIOA_OSPEED
	ldr		r1, [r0]
	orr		r1, #(GPIO_SPEED_FREQ_VERY_HIGH << GPIO_OSPEED_OSPEED9_Pos)
	orr		r1, #(GPIO_SPEED_FREQ_VERY_HIGH << GPIO_OSPEED_OSPEED10_Pos)
	str		r1, [r0]

	@ Set PUPDR
	ldr		r0, =GPIOA_PUPDR
	ldr		r1, [r0]
	orr		r1, #(GPIO_PUPD_PULL_UP << GPIO_PUPDR_PUPD9_Pos)
	orr		r1, #(GPIO_PUPD_PULL_UP << GPIO_PUPDR_PUPD10_Pos)
	str		r1, [r0]

	@ Set Alternate function modes
	ldr		r0, =GPIOA_AFRH
	ldr		r1, [r0]

	bic		r1, #GPIOx_AFRH_AFSEL9
	orr		r1, #GPIOx_AFRH_AFSEL9_AF4		@ Set AF4 on pa9. This sets pa9 to i2c1 scl

	bic		r1, #GPIOx_AFRH_AFSEL10
	orr		r1, #GPIOx_AFRH_AFSEL10_AF4		@ Set AF4 on pa10. This sets pa10 to i2c1 sda
	str		r1, [r0]

	@ Set GPIOA Port modes
	ldr		r0, =GPIOA_MODER
	ldr		r1, [r0]

	bic		r1, #GPIOx_MODER_P9				// Clear moder
	orr		r1, #GPIOx_MODER_P9_Altfn		// pa9 - Alternate function

	bic		r1, #GPIOx_MODER_P10			// Clear moder
	orr		r1, #GPIOx_MODER_P10_Altfn		// pa10 - Alternate function

	str		r1, [r0]

	bx		lr
	.pool
