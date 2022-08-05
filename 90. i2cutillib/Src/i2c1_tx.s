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
// i2c_master_tx  r0 = device address, r1=pointer to data, r2 = size (number of bytes to send)
//
.globl i2c_master_tx
.type i2c_master_tx, "function"
i2c_master_tx:
	push	{r4, r5, r11, lr}

	push	{r0-r3}
	bl		i2c1_waitlineidle
	pop		{r0-r3}

	// i2c_transfer_config - set the i2c interface up for write
	// r0 - device address
	// r1 - size (number of bytes to send), r2 = direction (0 write to slave, 1 read from slave)
	// r2 - mode
	// r3 - request
	push	{r0-r3}
	mov		r1, r2							@ number of bytes
	mov		r2, #(1 << 25) 					@ I2C_AUTOEND_MODE
	ldr		r3,	=I2C_GENERATE_START_WRITE
	bl		i2c_transfer_config
	pop		{r0-r3}

	// loop sending the data
writeloop:
	cmp		r2, #0
	ble		endwrloop

	ldr		r4, =I2C1_BASE
	ldrb	r5, [r1], #1
	strb	r5, [r4, #I2C_TXDR_OFFSET]

	push	{r0-r3}
	bl		wait_i2c1_wait_txe
	pop		{r0-r3}

	sub		r2, r2, #1
	b		writeloop

endwrloop:
	pop    {r4, r5, r11, pc}

	.pool

//
// wait for line to be idle
//
.type i2c1_waitlineidle, "function"
i2c1_waitlineidle:
	ldr		r4, =#0xffff
wait_i2c1_idle:
	cbz		r4, wt_lineidle_timeout
	sub		r4, #1

	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_ISR_OFFSET]
	tst		r1, #I2C_ISR_BUSY			// if busy then wait
	bne		wait_i2c1_idle

	cbnz	r4, wt_lineidle_exit

	@ Enable i2c1
wt_lineidle_timeout:
	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_CR1_OFFSET]	// Get initial state of I2C_CR1
	bic		r1, #I2C_CR1_PE
	str		r1, [r0, #I2C_CR1_OFFSET]

	orr		r1, #I2C_CR1_PE
	str		r1, [r0, #I2C_CR1_OFFSET]

 wt_lineidle_exit:
	bx		lr
.pool



//
// Wait for TXE. Set to 1 when Transmit data register is empty
//
.type wait_i2c1_wait_txe, "function"
wait_i2c1_wait_txe:
	ldr		r4, =#0xffff
wait_txe_idle:
	cbz		r4, wt_txe_exit
	sub		r4, #1

	@ Enable i2c1
wt_txe_timeout:
	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_ISR_OFFSET]
	tst		r1, #(I2C_ISR_TXIS + I2C_ISR_NACKF + I2C_ISR_STOPF)			// if busy then wait
	beq		wait_txe_idle

	// Clear NACK and STOP flag
	ldr		r5, =I2C1_BASE
	ldr		r6, [r5, #I2C_ICR_OFFSET]
	orr		r6, #(I2C_ICR_NACKCF + I2C_ICR_STOPCF)
	str		r6, [r5, #I2C_ICR_OFFSET]

wt_txe_exit:
	bx		lr


/*
 * generate start on i2c, r0 - i2c base address, r1 = device address, r2 - addressing mode
 * assumes the device address has already been shifted left 1
 *
 * hi2c->Instance->CR2 = I2C_GENERATE_START(hi2c->Init.AddressingMode, DevAddress);
 *
 * #define I2C_GENERATE_START(__ADDMODE__,__ADDRESS__) (((__ADDMODE__) == I2C_ADDRESSINGMODE_7BIT) ? \
 *                                                    (uint32_t)((((uint32_t)(__ADDRESS__) & (I2C_CR2_SADD)) | \
 *                                                                (I2C_CR2_START) | (I2C_CR2_AUTOEND)) & \
 *                                                               (~I2C_CR2_RD_WRN)) : \
 *  10 bit addressing
 *                                                    (uint32_t)((((uint32_t)(__ADDRESS__) & (I2C_CR2_SADD)) | \
 *                                                                (I2C_CR2_ADD10) | (I2C_CR2_START)) & \
 *                                                               (~I2C_CR2_RD_WRN)))
*/
.globl i2c1_generate_start
.type i2c1_generate_start, %function
i2c1_generate_start:
	push	{r4, r5, r6}

	mov		r6, #I2C_CR2_SADD
	and		r5, r1, r6						@ set slave address

	orr		r5, r2
	cmp		r2, #I2C_ADDRESSINGMODE_7BIT
	ite		eq
	biceq	r5, #I2C_CR2_ADD10
	orrne	r5, #I2C_CR2_ADD10

	bic		r5, #I2C_CR2_RD_WRN
	orr		r5, #I2C_CR2_START
	orr		r5, #I2C_CR2_AUTOEND

	str		r5, [r0, #I2C_CR2_OFFSET]

	pop		{r4, r5, r6}
	bx		lr

	.pool

/*
 * generate start read on i2c, r0 - i2c base address, r1 = device address, r2 - addressing mode
*/
.globl i2c1_generate_start_read
.type i2c1_generate_start_read, %function
i2c1_generate_start_read:
	push	{r4, r5, r6}

	ldr		r5, [r0, #I2C_CR2_OFFSET]	// Get initial state of I2C_CR2
	orr		r5, #I2C_CR2_AUTOEND

	lsl		r1, #1
	mov		r6, #I2C_CR2_SADD
	bic		r5, r6
	orr		r5, r1 							@ set slave address
	lsl		r2, #I2C_CR2_NBYTES_Pos
	bic		r2, #I2C_CR2_ADD10				// 7 bit address mode
	orr		r5, r2
	cmp		r3, #0
	ite		eq
	biceq	r5, #I2C_CR2_RD_WRN
	orrne	r5, #I2C_CR2_RD_WRN
	str		r5, [r0, #I2C_CR2_OFFSET]

	@ StartE
	ldr		r5, [r0, #I2C_CR2_OFFSET]		// Get initial state of I2C_CR2
	orr		r5, #I2C_CR2_START				// generate start & place in master mode
	str		r5, [r0, #I2C_CR2_OFFSET]

	pop		{r4, r5, r6}
	bx		lr

	.pool


.globl i2c1_generate_start_write
.type i2c1_generate_start_write, %function
i2c1_generate_start_write:
	push	{r4, r5, r6}

	ldr		r5, [r0, #I2C_CR2_OFFSET]	// Get initial state of I2C_CR2
	orr		r5, #I2C_CR2_AUTOEND

	lsl		r1, #1
	mov		r6, #I2C_CR2_SADD
	bic		r5, r6
	orr		r5, r1 							@ set slave address
	lsl		r2, #I2C_CR2_NBYTES_Pos
	bic		r2, #I2C_CR2_ADD10				// 7 bit address mode
	orr		r5, r2
	cmp		r3, #0
	ite		eq
	biceq	r5, #I2C_CR2_RD_WRN
	orrne	r5, #I2C_CR2_RD_WRN
	str		r5, [r0, #I2C_CR2_OFFSET]

	@ StartE
	ldr		r5, [r0, #I2C_CR2_OFFSET]		// Get initial state of I2C_CR2
	orr		r5, #I2C_CR2_START				// generate start & place in master mode
	str		r5, [r0, #I2C_CR2_OFFSET]

	pop		{r4, r5, r6}
	bx		lr

	.pool

//
// wait for start to clear which indicates start has been sent
//
.type i2c1_waitstart, "function"
i2c1_waitstart:

wait_i2c1_start:
	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_CR2_OFFSET]
	tst		r1, #I2C_CR2_START			// if busy then wait
	bne		wait_i2c1_start
	bx		lr



//
// Generate a STOP bit
//
.globl i2c1_stop
.type i2c1_stop, "function"
i2c1_stop:
	push	{lr}
	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_CR2_OFFSET]
	orr		r1, #I2C_CR2_STOP
	str		r1, [r0, #I2C_CR2_OFFSET]

wait_i2c1_stop:
	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_ISR_OFFSET]
	tst		r1, #I2C_ISR_STOPF			// if busy then wait
	beq		wait_i2c1_stop

	// Clear the stop bit

	bl		i2c1_clear_stop
	pop		{pc}

//
// Clear the STOP flag
//
.globl i2c1_clear_stop
.type i2c1_clear_stop, "function"
i2c1_clear_stop:
	push	{lr}
	// Clear the stop bit
	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_ICR_OFFSET]
	orr		r1, #I2C_ICR_STOPCF
	str		r1, [r0, #I2C_ICR_OFFSET]

	pop		{pc}

//
// Clear the NACK flag
//
.globl i2c1_clear_nack
.type i2c1_clear_nack, "function"
i2c1_clear_nack:
	push	{lr}
	// Clear the NACK bit
	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_ICR_OFFSET]
	orr		r1, #I2C_ICR_NACKCF
	str		r1, [r0, #I2C_ICR_OFFSET]

	pop		{pc}


//
// i2c1 Send  (non-leaf function)
//
.globl i2c1_senddata
.type i2c1_senddata, %function
i2c1_senddata:
	push	{r4, r5, r11, lr}

	push	{r0}
	bl		i2c1_waitlineidle
	pop		{r0}

	mov		r1, #0			@ number of bytes
	mov		r2, #0			@ direction (1 read)
	bl		i2c1_generate_start_write

	@ Wait for ISR->STOPF or ISR->NACKF
wait_i2c1_tx:
	ldr		r0, =I2C1_BASE
	ldr		r1, [r0, #I2C_ISR_OFFSET]
	tst		r1, #(I2C_ISR_STOPF + I2C_ISR_NACKF)
	beq		wait_i2c1_tx

	pop    {r4, r5, r11, pc}

	.pool



/**
  * @brief  Checks if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  Trials Number of trials
  * @param  Timeout Timeout duration
  * @retval HAL status
  *
  * @r0 - I2C_BASE
  * @r1 - device address
  * @r2 - trials
  * @r3 - timeout
  * return
  * @r0 - status
  */
.globl i2c_is_device_ready
.type i2c_is_device_ready, %function
i2c_is_device_ready:
	push	{r11, lr}
	mov		fp, sp
	push	{r0, r1, r2, r3}  	@ save the arguments on the stack

	sub		sp, sp, #8  		@ allocate two more local variables
	push	{r4-r10}			@ save callee-saved registers

	ldr		r0, [fp, #-16]		@ reload the i2c base
	mov		r1, #I2C_ISR_BUSY
	bl		i2c_get_flag		@ See if the interface is BUSY
	cmp		r0, #SET
	bne		trials

	mov		r0, #HAL_BUSY
	b		exit_idr

trials:
	// Generate start
	// hi2c->Instance->CR2 = I2C_GENERATE_START(hi2c->Init.AddressingMode, DevAddress);

	@@ generate start on i2c1, r0 - i2c_base, r1 = device address, r2 = size (number of bytes to send), r3 = direction (0 write to slave, 1 read from slave)

	ldr		r0, [fp, #-16]		@ reload i2c_base
	ldr		r1, [fp, #-12]		@ reload device address
	mov		r2, #I2C_ADDRESSINGMODE_7BIT
	bl		i2c1_generate_start

	mov		r6, #1000

wait_stop:
	ldr		r0, [fp, #-16]		@ reload the i2c base
	mov		r1, #I2C_ISR_STOPF
	bl		i2c_get_flag
	cmp		r0, #SET
	beq		nack_or_stop_rcvd

	sub		r6, #1
	cmp		r6, #0
	bgt		wait_stop
	b		nack_rcvd

nack_or_stop_rcvd:
	ldr		r0, [fp, #-16]		@ reload the i2c base
	mov		r1, #I2C_ISR_NACKF	@ Check NACKF received
	bl		i2c_get_flag
	cmp		r0, #SET
	beq		nack_rcvd

	/* NACKF Not received, Wait until STOPF flag is reset */
	ldr		r0, [fp, #-16]		@ reload the i2c base
	mov		r1,	#I2C_FLAG_STOPF
	mov		r2, #RESET
	mov		r3, #1000
	bl		wait_on_flag_until_timeout

	// Clear the STOP Flag
	ldr		r0, [fp, #-16]		@ reload the i2c base
	mov		r1,	#I2C_FLAG_STOPF
	bl		i2c_clear_flag

	mov		r0, #HAL_OK
	b		exit_idr

nack_rcvd:
	/* Wait until STOPF flag is reset */
	ldr		r0, [fp, #-16]		@ reload the i2c base
	mov		r1,	#I2C_FLAG_STOPF
	mov		r2, #RESET
	mov		r3, #0xf000
	bl		wait_on_flag_until_timeout

	// Clear the NACK Flag
	ldr		r0, [fp, #-16]		@ reload the i2c base
	mov		r1,	#I2C_ISR_NACKF
	bl		i2c_clear_flag

	// Clear the STOP Flag
	ldr		r0, [fp, #-16]		@ reload the i2c base
	mov		r1,	#I2C_FLAG_STOPF
	bl		i2c_clear_flag

	mov		r0, #HAL_ERROR
	b		exit_idr

exit_idr:
	pop 	{r4-r10}     @ restore callee saved registers
	mov		sp, fp       @ reset stack pointer

	pop    {r11, pc}    /* End of the epilogue. Restoring Frame pointer from the stack, jumping to previously saved LR via direct load into PC */

	.pool

i2c_base:		.word	0


/** @brief  Check whether the specified I2C flag is set or not.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg @ref I2C_FLAG_TXE     Transmit data register empty
  *            @arg @ref I2C_FLAG_TXIS    Transmit interrupt status
  *            @arg @ref I2C_FLAG_RXNE    Receive data register not empty
  *            @arg @ref I2C_FLAG_ADDR    Address matched (slave mode)
  *            @arg @ref I2C_FLAG_AF      Acknowledge failure received flag
  *            @arg @ref I2C_FLAG_STOPF   STOP detection flag
  *            @arg @ref I2C_FLAG_TC      Transfer complete (master mode)
  *            @arg @ref I2C_FLAG_TCR     Transfer complete reload
  *            @arg @ref I2C_FLAG_BERR    Bus error
  *            @arg @ref I2C_FLAG_ARLO    Arbitration lost
  *            @arg @ref I2C_FLAG_OVR     Overrun/Underrun
  *            @arg @ref I2C_FLAG_PECERR  PEC error in reception
  *            @arg @ref I2C_FLAG_TIMEOUT Timeout or Tlow detection flag
  *            @arg @ref I2C_FLAG_ALERT   SMBus alert
  *            @arg @ref I2C_FLAG_BUSY    Bus busy
  *            @arg @ref I2C_FLAG_DIR     Transfer direction (slave mode)
  *
  * @retval The new state of __FLAG__ (SET or RESET).
  *
  * #define __HAL_I2C_GET_FLAG(__HANDLE__, __FLAG__) (((((__HANDLE__)->Instance->ISR) & \
  *                                                    (__FLAG__)) == (__FLAG__)) ? SET : RESET)
  * r0 - I2C_BASE
  * r1 - Flag to check
  * Return
  * r0 - The new state of the flag: SET or RESET
*/
i2c_get_flag:
	push	{r4, lr}

	ldr		r4, [r0, #I2C_ISR_OFFSET]	// Get initial state of I2C_CR2
	ands	r4, r1

	ite		eq							// eq: Z=1
	moveq	r0, #RESET
	movne	r0, #SET

	pop		{r4, pc}

	.pool

/** @brief  Clear the I2C pending flags which are cleared by writing 1 in a specific bit.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __FLAG__ specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg @ref I2C_FLAG_TXE     Transmit data register empty
  *            @arg @ref I2C_FLAG_ADDR    Address matched (slave mode)
  *            @arg @ref I2C_FLAG_AF      Acknowledge failure received flag
  *            @arg @ref I2C_FLAG_STOPF   STOP detection flag
  *            @arg @ref I2C_FLAG_BERR    Bus error
  *            @arg @ref I2C_FLAG_ARLO    Arbitration lost
  *            @arg @ref I2C_FLAG_OVR     Overrun/Underrun
  *            @arg @ref I2C_FLAG_PECERR  PEC error in reception
  *            @arg @ref I2C_FLAG_TIMEOUT Timeout or Tlow detection flag
  *            @arg @ref I2C_FLAG_ALERT   SMBus alert
  *
  * r0 - I2C_BASE
  * r1 - Flag to clear
  * @retval None
  *
  * #define __HAL_I2C_CLEAR_FLAG(__HANDLE__, __FLAG__) (((__FLAG__) == I2C_FLAG_TXE) ? \
  *                                                  ((__HANDLE__)->Instance->ISR |= (__FLAG__)) : \
  *                                                  ((__HANDLE__)->Instance->ICR = (__FLAG__)))
  */
i2c_clear_flag:
	push	{r4, lr}

	cmp		r1, #I2C_FLAG_TXE
	ittte	eq
	ldreq	r4, [r0, #I2C_ISR_OFFSET]
	orreq	r4, r1
	streq	r4, [r0, #I2C_ISR_OFFSET]
	strne	r1, [r0, #I2C_ICR_OFFSET]

	pop		{r4, pc}

	.pool


/**
  * @brief  This function handles I2C Communication Timeout. It waits
  *                until a flag is no longer in the specified status.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Flag Specifies the I2C flag to check.
  * @param  Status The actual Flag status (SET or RESET).
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  * r0 - I2C_BASE								[fp, #-20]
  * r1 - Flag to check							[fp, #-16]
  * r2 - The status of the flag to check		[fp, #-12]
  * r3 - the timeout loop count					[fp, #-8]
  */
wait_on_flag_until_timeout:
	push	{r11, lr}
	mov		fp, sp
	push	{r0-r4}  			@ save the arguments on the stack

	sub		sp, sp, #8  		@ allocate two more local variables
	push	{r4-r10}			@ save callee-saved registers

	ldr		r0, [fp, #-8]		@ Get the loop count
	str		r0, [fp, #-24]		@ Initialize the loop counter and store in local variable

wait_flag_loop:
	ldr		r0, [fp, #-20]		@ reload the i2c base
	ldr		r1, [fp, #-16]		@ reload the flag to check
	bl		i2c_get_flag

	ldr		r1, [fp, #-12]		@ reload the flag status to check
	cmp		r0, r1
	bne		flag_not_set

	mov		r0, #HAL_OK
	b		exit_wait_flag

flag_not_set:
	ldr		r0, [fp, #-24]		@ load the loop counter
	sub		r0, #1
	str		r0, [fp, #-24]		@ save the updated loop counter
	cmp		r0, #0

	bgt		wait_flag_loop

	mov		r0, #HAL_ERROR
	b		exit_wait_flag

exit_wait_flag:

	pop 	{r4-r10}     		@ restore callee saved registers
	mov		sp, fp       		@ reset stack pointer

	pop    {r11, pc}    		/* End of the epilogue. Restoring Frame pointer from the stack, jumping to previously saved LR via direct load into PC */

	.pool
