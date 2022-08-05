/**
 ******************************************************************************
 * @file           : i2c1_init.s
 * @author         : Ian Church
 * @brief          : initialize i2c1
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
@ Initialize i2c1
@
@
@ Initialize i2c1
@
.globl i2c1_enable
.type i2c1_enable, %function
i2c1_enable:
	ldr		r0, =RCC_CCIPR
	ldr		r1, [r0]	// Get initial state of RCC_CCIPR
	bic		r1, #RCC_I2C1SEL
	orr		r1, #RCC_I2C1SEL_SYSCLK
	str		r1, [r0]

	ldr		r0, =RCC_APB1ENR1
	ldr		r1, [r0]	// Get initial state of RCC_APB1ENR
	orr		r1, #RCC_APB1ENR1_I2C1EN
	str		r1, [r0]

	ldr		r0, =RCC_APB1RSTR1
	ldr		r1, [r0]	// Reset i2c1
	orr		r1, #RCC_APB1RSTR1_I2C1RST
	str		r1, [r0]

	ldr		r0, =RCC_APB1RSTR1
	ldr		r1, [r0]	// Complete Reset i2c1
	bic		r1, #RCC_APB1RSTR1_I2C1RST
	str		r1, [r0]

	@ Disable i2c1
	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_CR1_OFFSET]	// Get initial state of I2C_CR1
	bic		r1, #I2C_CR1_PE
	str		r1, [r0, #I2C_CR1_OFFSET]

	@ Configure ANFOFF and DNF
	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_CR1_OFFSET]	// Get initial state of I2C_CR1
	bic		r1, #I2C_CR1_ANFOFF
	bic		r1, #I2C_CR1_DNF
	str		r1, [r0, #I2C_CR1_OFFSET]

// 0x00707cbb
	@ Configure PRESC, SDADEL, SCLDEL, SCLH, SCLL
	mov		r1, #0
	movt	r1, #0
	orr		r1, (7 << I2C_TIMINGR_PRESC_Pos)
	orr		r1, (49 << I2C_TIMINGR_SCLL_Pos)
	orr		r1, (49 << I2C_TIMINGR_SCLH_Pos)
	bic		r1, #I2C_TIMINGR_SCLDEL
	orr		r1, (14 << I2C_TIMINGR_SCLDEL_Pos)
	orr		r1, (15 << I2C_TIMINGR_SDADEL_Pos)
	ldr		r0, =I2C1_BASE
	str		r1, [r0, #I2C_TIMINGR_OFFSET]

	@ Configure nostretch
	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_CR1_OFFSET]	// Get initial state of I2C_CR1
	bic		r1, #I2C_CR1_NOSTRETCH
	str		r1, [r0, #I2C_CR1_OFFSET]

	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_OAR1_OFFSET]	// Get initial state of I2C_OAR1
	orr		r1, #I2C_OAR1_OA1EN
	orr		r1, (0x52 << I2C_OAR1_OA1_7_BIT_Pos)
	str		r1, [r0, #I2C_OAR1_OFFSET]

	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_CR2_OFFSET]	// Get initial state of I2C_CR2
	orr		r1, #I2C_CR2_AUTOEND
	bic		r1, #I2C_CR2_ADD10			/// set 7 bit addressing
	str		r1, [r0, #I2C_CR2_OFFSET]

	@ Enable i2c1
	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_CR1_OFFSET]	// Get initial state of I2C_CR1
	orr		r1, #I2C_CR1_PE
	str		r1, [r0, #I2C_CR1_OFFSET]

	bx		lr
	.pool
