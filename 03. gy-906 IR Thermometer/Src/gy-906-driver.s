/**
 ******************************************************************************
 * @file           : gy-906-driver.s
 * @author         : Ian Church
 * @brief          : gy-906/MLX90613 i2c driver
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Ian Church.
 * All rights reserved.</center></h2>
 *
 * connections:
 *
 ******************************************************************************
 */

.syntax				unified
.cpu cortex-m4 @ STM32L432KC

.thumb
.align

.include "../../include/Inc/stm32l432kc.i"

GY906_ADDR			=		0x5A

.section .text

.globl setsysclk
.globl pllenable
.globl hsienable
.globl uart2enable
.globl usart2write
.globl setGPIOA_USART2
.globl setMCO
.globl WaitForInterrupt
.globl DisableInterrupts
.globl EnableInterrupts
.globl setMCO_to_pa8
.globl itoh
.globl i2c_is_device_ready


.type main, "function"
.global main

main:
 	bl		setGPIOA_USART2
	bl		i2c_gpioa_init
	bl		hsienable
	bl		pllenable

	mov		r0, #RCC_CFGR_SW_PLL
	bl		setsysclk

	bl		uart2enable
	ldr		r0, =hello_string
	bl		usart2write

	bl		i2c1_enable

restart:

read_temp_loop:
	ldr		r0, =I2C1_BASE			// r0 - i2c base address
	mov		r1, GY906_ADDR			// r1 - device address
	lsl		r1, #1					// shift address left 1 bit
	mov		r2, #5					// number of trials
	mov		r3, #0x1000				// timeout
	bl		i2c_is_device_ready

	// r0 holds the status
	cmp		r0, #HAL_OK
	bne		no_device
	bl		display_device_rdy

no_device:
	@ simple delay
	ldr		r0, =0x00ffffff
	bl		simple_delay

	b		read_temp_loop
	.pool


display_addr:
	push	{r4-r7, lr}
	ldr		r0, =ox_string
	bl		usart2write

	ldr		r0, =addr
	ldr		r0, [r0]
	ldr		r1,	=myBuffer
	bl		itoh

	ldr		r0, =myBuffer
	bl		usart2write

	ldr		r0, =crlf
	bl		usart2write

	pop    {r4-r7, pc}

	.pool

display_device_rdy:
	push	{r4-r7, lr}
	ldr		r0, =device_rdy_str
	bl		usart2write
	pop    {r4-r7, pc}

	.pool

//
// Simple delay, r0 = lsw of delay r1 = msw of delay
//
.type simple_delay, "function"
simple_delay:
	push	{r4-r7, lr}
dlyup:
	cbz		r0, exdlyup
	sub		r0, #1
	b		dlyup
exdlyup:
	pop    {r4-r7, pc}

	.align

.data
hello_string:       .asciz              "Hello World - gy-906!\n\r"
add_string:       	.asciz              "ADD \n\r"
nack_string:       	.asciz              ": NACK\n\r"
ack_string:       	.asciz              ": ACK\n\r"
ox_string:       	.asciz              "0x"
device_rdy_str:    	.asciz              "device ready\n\r"
addr:				.word	0

output_string:		.asciz				"Address: "
crlf:				.asciz				"\n\r"

.bss
myBuffer:           .space              256

	.end
