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

//
// i2c_master_rx  r0 = device address, r1=pointer to data, r2 = size (number of bytes to receive)
//
.globl i2c_master_rx
.type i2c_master_rx, "function"
i2c_master_rx:
	push	{r4-r7, r11, lr}
	mov		r7, #0

	// i2c_transfer_config - set the i2c interface up for write
	// r0 - device address
	// r1 - size (number of bytes to send), r2 = direction (0 write to slave, 1 read from slave)
	// r2 - mode
	// r3 - request
	push	{r0-r3}
	mov		r1, r2			@ number of bytes
	mov		r2, #I2C_SOFTEND_MODE
	ldr		r3,	=I2C_GENERATE_START_READ
	bl		i2c_transfer_config
	pop		{r0-r3}

readloop:
	push	{r0-r3}
	bl		wait_on_rxne
	pop		{r0-r3}

	ldr		r5, =I2C1_BASE
	ldrb	r6, [r5, #I2C_RXDR_OFFSET]
	strb	r6, [r1], #1
	add		r7, r7, #1
	cmp		r7, r2
	bge		endrdloop
	b		readloop

endrdloop:

	pop    {r4-r7, r11, pc}

	.pool





.globl wait_on_rxne
.type wait_on_rxne, "function"
wait_on_rxne:
// Wait for ISR->RXNE
	push	{r4-r7, lr}
	ldr		r4, =#0xffff

// Wait for the RXNE flag
 wait_i2c1_rxne:
	cbz		r4, wt_rxne_timeout
	sub		r4, #1

	ldr		r5, =I2C1_BASE
	ldr		r6, [r5, #I2C_ISR_OFFSET]
	tst		r6, #(I2C_ISR_RXNE + I2C_ISR_NACKF + I2C_ISR_STOPF)

	beq		wait_i2c1_rxne

wt_rxne_timeout:
	// Clear NACK and STOP flag
	ldr		r5, =I2C1_BASE
	ldr		r6, [r5, #I2C_ICR_OFFSET]
	orr		r6, #(I2C_ICR_NACKCF + I2C_ICR_STOPCF)
	str		r6, [r5, #I2C_ICR_OFFSET]
	pop    {r4-r7, pc}

.pool

// Wait for ISR->STOPF or ISR->NACKF
// r0 - Flag (I2C_FLAG_TXE)
// r1 - flag status (SET, RESET)
// r2 - flag status
.type wait_on_flag, "function"
wait_on_flag:
	push	{r4, r5, lr}
	ldr		r4, =I2C1_BASE
	ldr		r5, [r4, #I2C_ISR_OFFSET]

	cmp		r1, #1
	beq		set

set:
	beq		wait_on_flag

	pop    {r4, r5, pc}

	.pool

