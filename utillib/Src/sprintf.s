/**
 ******************************************************************************
 * @file           : sprintf.s
 * @author         : Ian Church
 * @brief          : sprintf
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
	@ r0 - pointer to input buffer
	@ r1 - length
	@ r2 - pointer to buffer to return string representatio of the binary value
	@
.globl sprintf
.type sprintf, %function
sprintf:
	push	{r4-r6}

	@ r0 is a 32 bit value holding the binary
	@ take each 4 bit nibble, convert it to it's ascii equivalent

	mov		r6, #7
	mov		r5, #4
loop:
	mul		r3, r6, r5
	mov		r4, r0

	lsr		r4, r3
	and		r4, #0xf
	cmp		r4, #0x09
	ite		le
	addle	r4, #'0'
	addgt	r4, #('A' - 10)

	strb 	r4, [r1], #1

	cbz 	r6, done

	sub 	r6, r6, #1
	b		loop
done:

	pop 	{r4-r6}
	bx		lr

.pool
