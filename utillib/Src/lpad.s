/**
 ******************************************************************************
 * @file           : itoa.s
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
	@ itoa - convert binary integer to string
	@
	@ parameter usage
	@ r0 - Value to convert
	@ r1 - pointer to buffer for result
	@
.globl itoa
.type itoa, %function
itoa:
	PUSH	{r4-r6}
	MOV		r2, r0
	MOV		r3, r1
	MOV		r6, #10

loop1:
	CBZ 	r2, done
	UDIV 	r5, r2, r6
	MLS 	r4, r6, r5, r2
	ADD 	r4, r4, #0x30
	STRB 	r4, [r3], #1
	UDIV 	r2, r2, r6
	B 		loop1

done:
	MOV 	r4, #0
	STRB 	r4, [r3]
	SUB 	r3, r3, #1

loop2:
	CMP 	r3, r1
	BLE		exit
	LDRB 	r4, [r3]
	LDRB 	r5, [r1]
	STRB 	r4, [r1]
	STRB 	r5, [r3]
	ADD 	r1, r1, #1
	SUB 	r3, r3, #1
	B 		loop2

exit:
	POP 	{r4-r6}

	bx		lr
