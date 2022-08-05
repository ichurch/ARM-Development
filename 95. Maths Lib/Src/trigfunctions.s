/**
 ******************************************************************************
 * @file           : trigfunctions.s
 * @author         : Ian Church
 * @brief          : triganometric functions
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
 * Cortex M4 -> ARMv7E-M
 ******************************************************************************
 */

.syntax				unified
.cpu cortex-m4 @ STM32L432KC

.section .text

@
@ Sine function
from: http://www.coranac.com/2009/07/sines/
@
@ ARM assembly version, using n=13, p=15, A=12

@ A sine approximation via a third-order approx.
@ @param r0   Angle (with 2^15 units/circle)
@ @return     Sine value (Q12)
.align
.thumb
.type isin_S3a9, "function"
.global isin_S3a9
isin_S3a9:
    mov     r0, r0, lsl #(30-13)    @ x                 ; Q30
    teq     r0, r0, lsl #1
    it		mi
    rsbmi   r0, r0, #1<<31

    smulwt  r1, r0, r0              @ y=x*x             ; Q30*Q14/Q16 = Q28
    mov     r2, #3<<13              @ B_14=3/2
    sub     r1, r2, r1, asr #15     @ 3/2-y/2           ; Q14+Q28/Q14/2
    smulwt  r0, r1, r0              @                   ; Q14*Q14/Q16 = Q12

    bx      lr


@
@ ARM assembly version, using n=13, p=15, A=12
.type sine, "function"
.global	sine
sine:
	push 		{r4, lr}
	vpush.32 	{s16-s21}
	VMOV.F32 	s16, s0				// s16 = x
	VLDR.F32 	s18, =0 			// s18 = ret = e.e
	VMOV.F32 	s17, s16 			// s17 = pow = x
	VMOV.F32 	s19, s16 			// s19 = term = x
	VMOV.F32 	s20, #1 			// s2e = sign = 1
	VMOV.F32 	s21, #1 			// s21 = fact = 1
	MOVS 		r4, #1 				//  r4 = k = 1
	B 			check

loop:
	VMUL.F32 	s0, s20, s17 		// se = sign * power
	VDIV.F32 	s19, s0, s21 		// term = se/fact
	VADD.F32 	s18, s18, s19 		// ret += term
	VMUL.F32 	s0, s16, s16 		// se = x*x
	VMUL.F32 	s17, s0, s17		// pow *= x*x
	LSLS 		r0, r4, #1 			// re = 2*k
	ADDS 		r1, r0, #1 			// rl = 2*k + 1
	MULS 		r0, r1, r0			// re
	VMOV 		s0, r0
	VCVT.F32.S32 s0, s0				// convert to float
	VMUL.F32 	s21,s0,s21			// fact *= 2k*(2k+l)
	vldr.f32 	s0, =0
	VSUB.F32 	s20,s0,s20			// sign = e - sign
	ADDS 		r4, r4, #1			// k++

check:
	vabs.f32	s0, s19
	vldr.f32	s1, =label01
	vcmpe.f32	s0, s1
	VMRS 		APSR_nzcv, FPSCR	// copy NZCV f Lags
	BGE 		loop
	VMOV.F32	s0, s18				// return ret in se
	VPOP.32		{s16-s21}
	POP 		{r4, pc}


.data
label01: .float 0.00001










