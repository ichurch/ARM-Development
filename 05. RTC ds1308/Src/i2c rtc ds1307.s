/**
 ******************************************************************************
 * @file           : i2c rtc ds1307.c
 * @author         : Ian Church
 * @brief          : i2c Real Time Clock driver
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 Ian Church.
 * All rights reserved.</center></h2>
 *
 * connections:
 *
 *
 ******************************************************************************
 */

.syntax				unified
.cpu cortex-m4 @ STM32L432KC

.thumb
.align

.include "../../include/Inc/stm32l432kc.i"

COUNT				=	0
DS1307_ADDR			=	0x68			// rtc module

DS1307_TIME_REG_00	=	0x00
DS1307_TIME_REG_01	=	0x01
DS1307_TIME_REG_02	=	0x02
DS1307_TIME_REG_03	=	0x03
DS1307_TIME_REG_04	=	0x04
DS1307_TIME_REG_05	=	0x05
DS1307_TIME_REG_06	=	0x06
DS1307_CTRL_REG		=	0x07
DS1307_RAM_BASE		=	0x08
DS1307_RAM_TOP		=	0x3f

SQWE_FREQUENCY_1Hz		=	0x00
SQWE_FREQUENCY_4KHz		=	0x01
SQWE_FREQUENCY_8KHz		=	0x02
SQWE_FREQUENCY_32KHz	=	0x03

DS1307_CTRL_REG_SQWE	=	0x10

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
.globl i2c_gpioa_init
.globl i2c1_enable
.globl i2c1_start
.globl i2c1_senddata
.globl simple_delay

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

	mov		r0, #SQWE_FREQUENCY_32KHz
	bl		setCtrl

	mov		r0, #SQWE_FREQUENCY_8KHz
	bl		setCtrl

	mov		r0, #DS1307_TIME_REG_01			// Minutes
	mov		r1, #(4 << 4) + 0
	bl		setTime

	mov		r0, #DS1307_TIME_REG_02			// Hours
	mov		r1, #(1 << 4) + 0
	bl		setTime

	mov		r0, #DS1307_TIME_REG_04			// Date
	mov		r1, #(2 << 4) + 5
	bl		setTime

	mov		r0, #DS1307_TIME_REG_05			// Month
	mov		r1, #(0 << 4) + 7
	bl		setTime

	mov		r0, #DS1307_TIME_REG_06			// Year
	mov		r1, #(2 << 4) + 1
	bl		setTime

restart:
	// i2c_master_tx  r0=device address, r1=pointer to data to send, r2=size (number of bytes to send)
	mov		r0, #DS1307_ADDR
	mov		r1, #0				// Register to read
	mov		r2, #1				// Number of bytes to write
	bl		i2c_master_tx

	mov		r0, #DS1307_ADDR
	ldr		r1, =rtcData
	mov		r2, #0x7				// number of bytes to receive
	bl		i2c_master_rx

	// rtcData now contains 7 bytes of Timekeeper Register values
	ldr		r0, =output_string
	bl		usart2write

	// address is in r1
	ldr		r4, =rtcData
	ldr		r1, [r4]

	mov		r0, r1
	ldr		r1,	=myBuffer
	bl		itoh

	ldr		r0, =myBuffer
	bl		usart2write

	ldr		r0, =separator
	bl		usart2write

	// Seconds
	ldr		r1, =rtcData
	ldrb	r0, [r1], #1
	ldr		r2, =seconds
	str		r0, [r2]

	// Minutes
	ldrb	r0, [r1], #1
	ldr		r2, =minutes
	str		r0, [r2]

	// Hours
	ldrb	r0, [r1], #1
	ldr		r2, =hours
	str		r0, [r2]

	// Day
	ldrb	r0, [r1], #1
	ldr		r2, =day
	str		r0, [r2]

	// Date
	ldrb	r0, [r1], #1
	ldr		r2, =date
	str		r0, [r2]

	// Month
	ldrb	r0, [r1], #1
	ldr		r2, =month
	str		r0, [r2]

	// Year
	ldrb	r0, [r1], #1
	ldr		r2, =year
	str		r0, [r2]

	// read data is in Day
	ldr		r1, =date
	ldr		r0, [r1]
	ldr		r1,	=myBuffer
	bl		processRTCData
	bl		itoa
	ldr		r0, =myBuffer
	bl		usart2write

	// write hyphen
	ldr		r0, =hyphen
	bl		usart2write

	// read data is in Month
	ldr		r1, =month
	ldr		r0, [r1]
	ldr		r1,	=myBuffer
	bl		processRTCData
	bl		itoa
	ldr		r0, =myBuffer
	bl		usart2write

	// write hyphen
	ldr		r0, =hyphen
	bl		usart2write

	// display year
	ldr		r1, =year
	ldr		r0, [r1]
	ldr		r1,	=myBuffer
	bl		processRTCYear
	bl		itoa
	ldr		r0, =myBuffer
	bl		usart2write

	// write colon
	ldr		r0, =space
	bl		usart2write

	// read data is in rtcData: Hours
	ldr		r1, =hours
	ldr		r0, [r1]
	ldr		r1,	=myBuffer
	bl		processRTCData
	bl		itoa
	ldr		r0, =myBuffer
	bl		usart2write

	// write colon
	ldr		r0, =colon
	bl		usart2write

	// read data is in rtcData: Minutes
	ldr		r1, =minutes
	ldr		r0, [r1]
	ldr		r1,	=myBuffer
	bl		processRTCData
	bl		itoa
	ldr		r0, =myBuffer
	bl		usart2write

	// write colon
	ldr		r0, =colon
	bl		usart2write

	// read data is in rtcData: Seconds
	ldr		r1, =seconds
	ldr		r0, [r1]
	ldr		r1,	=myBuffer
	bl		processRTCData
	bl		itoa
	ldr		r0, =myBuffer
	bl		usart2write

	ldr		r0, =crlf
	bl		usart2write

	@ simple delay
	ldr		r0, =0x00fffff
	bl		simple_delay

	b		restart

	.pool


//
// print_register  r0 = device address, r1 = register to read, r2 = length,
//
.globl print_register
.type print_register, %function
print_register:
	push	{r4-r5, lr}
	push	{r0-r2}
	// Set up the register to read from
	ldr		r4, =send_buffer
	strb	r1, [r4]
	ldr		r1, =pwm_data
	mov		r2, #1

	// i2c_master_tx  r0 = device address, r1=pointer to data, r2 = size (number of bytes to send)
	bl		i2c_master_tx
	pop		{r0-r2}

	ldr		r1, =pwm_data
	bl		i2c_master_rx

	ldr		r1, =pwm_data
	ldr		r0, [r1]
	ldr		r1,	=myBuffer
	bl		itoh

	ldr		r0, =myBuffer
	bl		usart2write

	ldr		r0, =crlf
	bl		usart2write

	pop		{r4-r5, pc}


//
// processRTCData
// rtc data is bcd encoded, decode and save in buffer
// r0 - byte from rtc
// r1 - address of output buffer
//
.globl processRTCData
.type processRTCData, %function
processRTCData:
	mov		r2, r1				// output buffer address
	mov		r3,	r0				// save the value to be processed

	lsr		r0, #4
	and		r0, #0x07
	mov		r5, #10
	mul		r0, r5

	mov		r4, r3
	and		r4, #0x0f
	add		r0, r4

	strb	r0, [r2]

	bx		lr


//
// processRTCYear
// rtc data is bcd encoded, decode and save in buffer
// r0 - byte from rtc
// r1 - address of output buffer
//
.globl processRTCYear
.type processRTCYear, %function
processRTCYear:
	mov		r2, r1				// output buffer address
	mov		r3,	r0				// save the value to be processed

	lsr		r0, #4
	mov		r5, #10
	mul		r0, r5

	mov		r4, r3
	and		r4, #0x0f
	add		r0, r4

	strb	r0, [r2]

	bx		lr


//
// setCtrl  r0 = frequency
//
.globl setCtrl
.type setCtrl, %function
setCtrl:
	push	{r4-r5, lr}

	ldr		r4, =myBuffer
	mov		r5, #DS1307_CTRL_REG
	strb	r5, [r4], #1
	mov		r5, #DS1307_CTRL_REG_SQWE
	orr		r5, r0
	strb	r5, [r4]

	mov		r0, #DS1307_ADDR
	ldr		r1, =myBuffer
	mov		r2, #2					// number of bytes to send
	bl		i2c_master_tx

	pop		{r4-r5, pc}


	// r0 - register
	// r1 - value
.globl setTime
.type setTime, %function
setTime:
	push	{r4-r5, lr}

	ldr		r4, =myBuffer
	mov		r5, r0
	strb	r5, [r4], #1
	strb	r1, [r4]

	mov		r0, #DS1307_ADDR
	ldr		r1, =myBuffer
	mov		r2, #2					// number of bytes to send
	bl		i2c_master_tx

	pop		{r4-r5, pc}

	// r0 - register
	// r1 - value
.globl setMemory
.type setMemory, %function
setMemory:
	push	{r4-r5, lr}

	ldr		r4, =myBuffer
	strb	r0, [r4], #1
	strb	r1, [r4]

	mov		r0, #DS1307_ADDR
	ldr		r1, =myBuffer
	mov		r2, #2					// number of bytes to send
	bl		i2c_master_tx

	pop		{r4-r5, pc}

	.data
	.align

.data
hello_string:       .asciz              "Hello World!\n\r"
nack_string:       	.asciz              ": NACK\n\r"
ack_string:       	.asciz              ": ACK\n\r"
ox_string:       	.asciz              "0x"

output_string:		.asciz				"Data: "
separator:			.asciz				" -> "
colon:				.asciz				":"
space:				.asciz				" "
hyphen:				.asciz				"-"
crlf:				.asciz				"\n\r"

ptr:				.word				0
seconds:			.word				0
minutes:			.word				0
hours:				.word				0
day:				.word				0
date:				.word				0
month:				.word				0
year:				.word				0

.align
rcount:				.word				500

.bss
myBuffer:           .space              256

rtc_register:       .byte				0x00
rtcData:			.space				256
rtcDataProcessed:	.space				256
send_buffer:        .space              256
pwm_register:       .byte				0x00
pwm_data:			.space				256

mma_register:       .byte				0x0

	.end
