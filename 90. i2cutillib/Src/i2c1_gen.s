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

	/**
  * @brief  Handles I2Cx communication when starting transfer or during transfer (TC or TCR flag are set).
  * @param  hi2c I2C handle.
  * @param  DevAddress Specifies the slave address to be programmed.
  * @param  Size Specifies the number of bytes to be programmed.
  *   This parameter must be a value between 0 and 255.
  * @param  Mode New state of the I2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg @ref I2C_RELOAD_MODE Enable Reload mode .
  *     @arg @ref I2C_AUTOEND_MODE Enable Automatic end mode.
  *     @arg @ref I2C_SOFTEND_MODE Enable Software end mode.
  * @param  Request New state of the I2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg @ref I2C_NO_STARTSTOP Don't Generate stop and start condition.
  *     @arg @ref I2C_GENERATE_STOP Generate stop condition (Size should be set to 0).
  *     @arg @ref I2C_GENERATE_START_READ Generate Restart for read request.
  *     @arg @ref I2C_GENERATE_START_WRITE Generate Restart for write request.
  * @retval None

static void I2C_TransferConfig(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t Size, uint32_t Mode,
							   uint32_t Request)
*/

// i2c_transfer_config
// r0 - device address
// r1 - size (number of bytes to send), r2 = direction (0 write to slave, 1 read from slave)
// r2 - mode  		(I2C_AUTOEND_MODE, I2C_RELOAD_MODE, I2C_SOFTEND_MODE)
// r3 - request		(I2C_GENERATE_START_WRITE, I2C_GENERATE_START_READ, I2C_NO_STARTSTOP, I2C_GENERATE_STOP)
.globl i2c_transfer_config
.type i2c_transfer_config, "function"
i2c_transfer_config:

  /* update CR2 register */
//  MODIFY_REG(hi2c->Instance->CR2,
//			 ((I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | (I2C_CR2_RD_WRN & (uint32_t)(Request >> (31U - I2C_CR2_RD_WRN_Pos))) | I2C_CR2_START | I2C_CR2_STOP)), \
//			 (uint32_t)(((uint32_t)DevAddress & I2C_CR2_SADD) | (((uint32_t)Size << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | (uint32_t)Mode | (uint32_t)Request));

	push	{r4-r6}

	ldr		r4, =I2C1_BASE
	ldr		r5, [r4, #I2C_CR2_OFFSET]

	ldr		r6, =(I2C_CR2_SADD + I2C_CR2_NBYTES + I2C_CR2_RELOAD + I2C_CR2_AUTOEND + I2C_CR2_START + I2C_CR2_STOP + I2C_CR2_RD_WRN)
	bic		r5, r6					// clear CR2 register

	orr		r5, r2					// Set the mode
	orr		r5, r3					// Set the request

	lsl		r0, #1					// shift slave address 1 bit to the left
	orr		r5, r0 							@ set slave address
	lsl		r1, #I2C_CR2_NBYTES_Pos
	bic		r1, #I2C_CR2_ADD10
	orr		r5, r1

	str		r5, [r4, #I2C_CR2_OFFSET]

	pop		{r4-r6}
	bx		lr

