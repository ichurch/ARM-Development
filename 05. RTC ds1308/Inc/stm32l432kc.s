/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Ian Church
 * @brief          : DAC 1
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 Ian Church.
 * All rights reserved.</center></h2>
 *
 *
 ******************************************************************************
 */

.syntax				unified
.cpu cortex-m4 @ STM32L432KC

@ Copied from stm32l432xx.h
NonMaskableInt_IRQn         = -14    /*!< 2 Cortex-M4 Non Maskable Interrupt                                */
HardFault_IRQn              = -13    /*!< 3 Cortex-M4 Hard Fault Interrupt                                  */
MemoryManagement_IRQn       = -12    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
BusFault_IRQn               = -11    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
UsageFault_IRQn             = -10    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
SVCall_IRQn                 = -5     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
DebugMonitor_IRQn           = -4     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
PendSV_IRQn                 = -2     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
SysTick_IRQn                = -1     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
WWDG_IRQn                   = 0      /*!< Window WatchDog Interrupt                                         */
PVD_PVM_IRQn                = 1      /*!< PVD/PVM3/PVM4 through EXTI Line detection Interrupts              */
TAMP_STAMP_IRQn             = 2      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
RTC_WKUP_IRQn               = 3      /*!< RTC Wakeup interrupt through the EXTI line                        */
FLASH_IRQn                  = 4      /*!< FLASH global Interrupt                                            */
RCC_IRQn                    = 5      /*!< RCC global Interrupt                                              */
EXTI0_IRQn                  = 6      /*!< EXTI Line0 Interrupt                                              */
EXTI1_IRQn                  = 7      /*!< EXTI Line1 Interrupt                                              */
EXTI2_IRQn                  = 8      /*!< EXTI Line2 Interrupt                                              */
EXTI3_IRQn                  = 9      /*!< EXTI Line3 Interrupt                                              */
EXTI4_IRQn                  = 10     /*!< EXTI Line4 Interrupt                                              */
DMA1_Channel1_IRQn          = 11     /*!< DMA1 Channel 1 global Interrupt                                   */
DMA1_Channel2_IRQn          = 12     /*!< DMA1 Channel 2 global Interrupt                                   */
DMA1_Channel3_IRQn          = 13     /*!< DMA1 Channel 3 global Interrupt                                   */
DMA1_Channel4_IRQn          = 14     /*!< DMA1 Channel 4 global Interrupt                                   */
DMA1_Channel5_IRQn          = 15     /*!< DMA1 Channel 5 global Interrupt                                   */
DMA1_Channel6_IRQn          = 16     /*!< DMA1 Channel 6 global Interrupt                                   */
DMA1_Channel7_IRQn          = 17     /*!< DMA1 Channel 7 global Interrupt                                   */
ADC1_IRQn                   = 18     /*!< ADC1 global Interrupt                                             */
CAN1_TX_IRQn                = 19     /*!< CAN1 TX Interrupt                                                 */
CAN1_RX0_IRQn               = 20     /*!< CAN1 RX0 Interrupt                                                */
CAN1_RX1_IRQn               = 21     /*!< CAN1 RX1 Interrupt                                                */
CAN1_SCE_IRQn               = 22     /*!< CAN1 SCE Interrupt                                                */
EXTI9_5_IRQn                = 23     /*!< External Line[9:5] Interrupts                                     */
TIM1_BRK_TIM15_IRQn         = 24     /*!< TIM1 Break interrupt and TIM15 global interrupt                   */
TIM1_UP_TIM16_IRQn          = 25     /*!< TIM1 Update Interrupt and TIM16 global interrupt                  */
TIM1_TRG_COM_IRQn           = 26     /*!< TIM1 Trigger and Commutation Interrupt                            */
TIM1_CC_IRQn                = 27     /*!< TIM1 Capture Compare Interrupt                                    */
TIM2_IRQn                   = 28     /*!< TIM2 global Interrupt                                             */
I2C1_EV_IRQn                = 31     /*!< I2C1 Event Interrupt                                              */
I2C1_ER_IRQn                = 32     /*!< I2C1 Error Interrupt                                              */
SPI1_IRQn                   = 35     /*!< SPI1 global Interrupt                                             */
USART1_IRQn                 = 37     /*!< USART1 global Interrupt                                           */
USART2_IRQn                 = 38     /*!< USART2 global Interrupt                                           */
EXTI15_10_IRQn              = 40     /*!< External Line[15:10] Interrupts                                   */
RTC_Alarm_IRQn              = 41     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
SPI3_IRQn                   = 51     /*!< SPI3 global Interrupt                                             */
TIM6_DAC_IRQn               = 54     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
TIM7_IRQn                   = 55     /*!< TIM7 global interrupt                                             */
DMA2_Channel1_IRQn          = 56     /*!< DMA2 Channel 1 global Interrupt                                   */
DMA2_Channel2_IRQn          = 57     /*!< DMA2 Channel 2 global Interrupt                                   */
DMA2_Channel3_IRQn          = 58     /*!< DMA2 Channel 3 global Interrupt                                   */
DMA2_Channel4_IRQn          = 59     /*!< DMA2 Channel 4 global Interrupt                                   */
DMA2_Channel5_IRQn          = 60     /*!< DMA2 Channel 5 global Interrupt                                   */
COMP_IRQn                   = 64     /*!< COMP1 and COMP2 Interrupts                                        */
LPTIM1_IRQn                 = 65     /*!< LP TIM1 interrupt                                                 */
LPTIM2_IRQn                 = 66     /*!< LP TIM2 interrupt                                                 */
USB_IRQn                    = 67     /*!< USB event Interrupt                                               */
DMA2_Channel6_IRQn          = 68     /*!< DMA2 Channel 6 global interrupt                                   */
DMA2_Channel7_IRQn          = 69     /*!< DMA2 Channel 7 global interrupt                                   */
LPUART1_IRQn                = 70     /*!< LP UART1 interrupt                                                */
QUADSPI_IRQn                = 71     /*!< Quad SPI global interrupt                                         */
I2C3_EV_IRQn                = 72     /*!< I2C3 event interrupt                                              */
I2C3_ER_IRQn                = 73     /*!< I2C3 error interrupt                                              */
SAI1_IRQn                   = 74     /*!< Serial Audio Interface 1 global interrupt                         */
SWPMI1_IRQn                 = 76     /*!< Serial Wire Interface 1 global interrupt                          */
TSC_IRQn                    = 77     /*!< Touch Sense Controller global interrupt                           */
RNG_IRQn                    = 80     /*!< RNG global interrupt                                              */
FPU_IRQn                    = 81     /*!< FPU global interrupt                                              */
CRS_IRQn                    = 82      /*!< CRS global interrupt                                              */

@
@ SysTick is defined in the Cortex-M4 doc
@
NVIC_ST_CTRL_R			=		0xE000E010 	// SysTick Control and Status Register
NVIC_ST_RELOAD_R		=		0xE000E014	// SysTick Reload Value Register
NVIC_ST_CURRENT_R		=		0xE000E018 	// SysTick Current Value Register
NVIC_ST_CALIB_R			=		0xE000E01C 	// SysTick Calibration Value Register

NVIC_ST_CTRL_COUNT		=		0x00010000	// Count flag
NVIC_ST_CLK_SRC			=		0x00000004	// Clock source CPU
NVIC_ST_CTRL_INT_EN		=		0x00000002	// Tick interrupt enable
NVIC_ST_CTRL_ENAblE		=		0x00000001	// Enable the timer
NVIC_ST_RELOAD_M		=		0x00FFFFFF	// Counter load value

@ NVIC Registers
NVIC_ISER0				=		0xE000E100	@ Enable for external interrupt #0-31
NVIC_ISER1				=		0xE000E104	@ Enable for external interrupt #32-63
NVIC_ISER2				=		0xE000E108	@ Enable for external interrupt #64-95

NVIC_ICER0				=		0xE000E180	@ CLear Enable for external interrupt #0-31
NVIC_ICER1				=		0xE000E184	@ Clear Enable for external interrupt #32-63
NVIC_ICER2				=		0xE000E188	@ Clear Enable for external interrupt #64-95

NVIC_ISPR0				=		0xE000E200	@ Pending for external interrupt #0-31
NVIC_ISPR1				=		0xE000E204	@ Pending for external interrupt #32-63
NVIC_ISPR2				=		0xE000E208	@ Pending for external interrupt #64-95

NVIC_ICPR0				=		0xE000E280	@ Clear Pending for external interrupt #0-31
NVIC_ICPR1				=		0xE000E284	@ Clear Pending for external interrupt #32-63
NVIC_ICPR2				=		0xE000E288	@ Clear Pending for external interrupt #64-95


PERIPH_BASE			=		0x40000000
AHB1PERIPH_BASE		=		PERIPH_BASE + 0x00020000
AHB2PERIPH_BASE		=		PERIPH_BASE + 0x08000000

APB1PERIPH_BASE		=		PERIPH_BASE


RCC_BASE			=		AHB1PERIPH_BASE + 0x1000
GPIOB_BASE			=		AHB2PERIPH_BASE + 0x0400

@
@ RCC_CR Clock Control Register Definitions
@
RCC_OFFSET			=		0x00
RCC_CR				=		RCC_BASE + RCC_OFFSET

RCC_CR_MSION		=		1 << 0
RCC_CR_MSIRDY		=		1 << 1
RCC_CR_MSIPLLEN		=		1 << 2
RCC_CR_MSIRGSEL		=		1 << 3
RCC_CR_HSION		=		1 << 8
RCC_CR_HSIRDY		=		1 << 10
RCC_CR_PLLON		=		1 << 24
RCC_CR_PLLRDY		=		1 << 25

RCC_CR_MSIRANGE_Pos		=	4
RCC_CR_MSIRANGE_Msk		=	(0x0f << RCC_CR_MSIRANGE_Pos)
RCC_CR_MSIRANGE			= 	RCC_CR_MSIRANGE_Msk
RCC_CR_MSIRANGE_100K	=	(0x00 << RCC_CR_MSIRANGE_Pos)
RCC_CR_MSIRANGE_200K	=	(0x01 << RCC_CR_MSIRANGE_Pos)
RCC_CR_MSIRANGE_400K	=	(0x02 << RCC_CR_MSIRANGE_Pos)
RCC_CR_MSIRANGE_800K	=	(0x03 << RCC_CR_MSIRANGE_Pos)

RCC_CR_MSIRANGESEL_Pos	=	3
RCC_CR_MSIRGSEL_CR		=	(0x01 << RCC_CR_MSIRANGESEL_Pos)

@
@ RCC_PLLCFGR
@
RCC_PLLCFGR_OFFSET	=		0x0c
RCC_PLLCFGR			=		RCC_BASE + RCC_PLLCFGR_OFFSET

RCC_PLLCFGR_PLLR_2	=		0x00 << 25
RCC_PLLCFGR_PLLR_4	=		0x01 << 25
RCC_PLLCFGR_PLLR_6	=		0x02 << 25
RCC_PLLCFGR_PLLR_8	=		0x03 << 25

RCC_PLLCFGR_PLLREN	=		0x01 << 24

RCC_PLLCFGR_PLLQ_2	=		0x00 << 21
RCC_PLLCFGR_PLLQ_4	=		0x01 << 21
RCC_PLLCFGR_PLLQ_6	=		0x02 << 21
RCC_PLLCFGR_PLLQ_8	=		0x03 << 21

RCC_PLLCFGR_PLLQEN	=		0x01 << 20

RCC_PLLCFGR_PLLP_7	=		0x0	<< 17
RCC_PLLCFGR_PLLP_17	=		0x1	<< 17

RCC_PLLCFGR_PLLPEN	=		0x1	<< 16

RCC_PLLCFGR_PLLN_10	=		0x0A00

RCC_PLLCFGR_PLLM_1	=		0x00 << 4
RCC_PLLCFGR_PLLM_2	=		0x01 << 4
RCC_PLLCFGR_PLLM_3	=		0x02 << 4
RCC_PLLCFGR_PLLM_4	=		0x03 << 4
RCC_PLLCFGR_PLLM_5	=		0x04 << 4
RCC_PLLCFGR_PLLM_6	=		0x05 << 4
RCC_PLLCFGR_PLLM_7	=		0x06 << 4
RCC_PLLCFGR_PLLM_8	=		0x07 << 4

RCC_PLLCFGR_PLLSRC_MSI	=	0x01 << 0
RCC_PLLCFGR_PLLSRC_HSI16=	0x02 << 0
RCC_PLLCFGR_PLLSRC_HSE	=	0x03 << 0


AHB2ENR_OFFSET		=		0x4c
RCC_AHB2ENR			=		RCC_BASE + AHB2ENR_OFFSET

APB1ENR_OFFSET		=		0x58
RCC_APB1ENR			=		RCC_BASE + APB1ENR_OFFSET

RCC_APB1ENR1_OFFSET	=		0x58
RCC_APB1ENR1		=		RCC_BASE + RCC_APB1ENR1_OFFSET

APB2ENR_OFFSET		=		0x60
RCC_APB2ENR			=		RCC_BASE + APB2ENR_OFFSET

@
@ APB2ENR
@
RCC_APB2ENR_SYSCFGEN=		(0x01 << 1)
RCC_APB2ENR_FWEN	=		(0x01 << 7)
RCC_APB2ENR_SDMMC1EN=		(0x01 << 10)
RCC_APB2ENR_TIM1EN	=		(0x01 << 11)
RCC_APB2ENR_SPI1EN	=		(0x01 << 12)
RCC_APB2ENR_TIM15EN	=		(0x01 << 16)
RCC_APB2ENR_TIM16EN	=		(0x01 << 17)
RCC_APB2ENR_SAI1EN	=		(0x01 << 21)
RCC_APB2ENR_DFSDM1EN=		(0x01 << 24)

@
@ APB1ENR1
@
RCC_APB1ENR1_TIM2EN		=		(0x01 << 0)
RCC_APB1ENR1_TIM3EN		=		(0x01 << 1)
RCC_APB1ENR1_TIM6EN		=		(0x01 << 4)
RCC_APB1ENR1_TIM7EN		=		(0x01 << 5)
RCC_APB1ENR1_RTCAPBEN	=		(0x01 << 10)
RCC_APB1ENR1_WWDGEN		=		(0x01 << 11)
RCC_APB1ENR1_USART2EN	=		(0x01 << 17)
RCC_APB1ENR1_I2C1EN		=		(0x01 << 21)
RCC_APB1ENR1_I2C3EN		=		(0x01 << 23)
RCC_APB1ENR1_CRSEN		=		(0x01 << 24)
RCC_APB1ENR1_PWREN		=		(0x01 << 28)
RCC_APB1ENR1_DAC1		=		(0x01 << 29)
RCC_APB1ENR1_OPAMPEN	=		(0x01 << 30)
RCC_APB1ENR1_LPTIM1EN	=		(0x01 << 31)




@
@ Clock Configuration Register
@
RCC_CFGR_OFFSET		=		0x08
RCC_CFGR			=		RCC_BASE + RCC_CFGR_OFFSET

RCC_CFGR_SW_MSI			=	0x00 << 0
RCC_CFGR_SW_HSI16		=	0x01 << 0
RCC_CFGR_SW_HSE			=	0x02 << 0
RCC_CFGR_SW_PLL			=	0x03 << 0

RCC_CFGR_MCOSEL_Pos		=	24
RCC_CFGR_MCOSEL_Msk		=	(0x07 << RCC_CFGR_MCOSEL_Pos)
RCC_CFGR_MCOSEL			=	RCC_CFGR_MCOSEL_Msk
RCC_CFGR_MCOSEL_SYSCLK	=	(0x01 << RCC_CFGR_MCOSEL_Pos)
RCC_CFGR_MCOSEL_MSI		=	(0x02 << RCC_CFGR_MCOSEL_Pos)
RCC_CFGR_MCOSEL_HSI16	=	(0x03 << RCC_CFGR_MCOSEL_Pos)
RCC_CFGR_MCOSEL_HSE		=	(0x04 << RCC_CFGR_MCOSEL_Pos)
RCC_CFGR_MCOSEL_PLL		=	(0x05 << RCC_CFGR_MCOSEL_Pos)
RCC_CFGR_MCOSEL_LSI		=	(0x06 << RCC_CFGR_MCOSEL_Pos)
RCC_CFGR_MCOSEL_LSE		=	(0x07 << RCC_CFGR_MCOSEL_Pos)
RCC_CFGR_MCOSEL_HSI48	=	(0x08 << RCC_CFGR_MCOSEL_Pos)		@  Select HSI48 as MCO output



RCC_CFGR_MCOPRE_1		=	0x00 << 28		@ MCO Prescaller / 1
RCC_CFGR_MCOPRE_2		=	0x01 << 28		@ MCO Prescaller / 2
RCC_CFGR_MCOPRE_4		=	0x02 << 28		@ MCO Prescaller / 4
RCC_CFGR_MCOPRE_8		=	0x03 << 28		@ MCO Prescaller / 8
RCC_CFGR_MCOPRE_16		=	0x04 << 28		@ MCO Prescaller / 16

@
@ Clock Recovery Register - This controls HSI48 the 48Mhz internal clock
@
RCC_CRRCR_OFFSET	=		0x98
RCC_CRRCR			=		RCC_BASE + RCC_CRRCR_OFFSET

RCC_CRRCR_HSI48RDY	=		1<<1			@ RCC_CRRCR->HSI48RDY
RCC_CRRCR_HSI48ON	=		1<<0			@ RCC_CRRCR->HSI48ON

RCC_CCIPR_OFFSET	=		0x88			@ Peripherals independant clock configuration register
RCC_CCIPR			=		RCC_BASE + RCC_CCIPR_OFFSET

RCC_CCIPR_ADCSEL	=		0x03 << 28

@
@ ADC Definitions
@
ADC_BASE			=		0x50040000

ADC_ISR_OFFSET		=		0x00
ADC_ISR				=		ADC_BASE + ADC_ISR_OFFSET		@ Interrupt and status register

ADC_ISR_ADRDY		=		0x01							@ Bit 1, 1 => ADC Ready

ADC_CFGR_OFFSET		=		0x0c
ADC_CFGR			=		ADC_BASE + ADC_CFGR_OFFSET

ADC_CFGR_DISCEN		=		1 << 16							@ Discontinuous mode enable
ADC_CFGR_DISCNUM1	=		0								@ Discontinuous mode channel count 0 => 1 channel
ADC_CFGR_CONT		=		1 << 13
ADC_CFGR_OVRMOD1	=		1 << 12							@ OVRMOD => 1, ADC_DR is overwritten

@
@ ADC_CCR - Common Control Register
@
ADC_CCR_OFFSET		=		0x08
ADC_CCR				=		ADC_BASE + ADC_CCR_OFFSET + 0x300

ADC_CCR_CH17SEL		=		1<<23							@ Select temperature sensor
ADC_CCR_PRESC6		=		0x03 << 18						@ ADC Clock prescaler / 6

ADC_CR_OFFSET		=		0x08
ADC_CR				=		ADC_BASE + ADC_CR_OFFSET
ADC_CR_ADSTART		=		1 << 2
ADC_CR_ADEN			=		1 << 0
ADC_CR_DEEPPWD		=		1 << 29
ADC_CR_ADVREGEN		=		1 << 28


ADC_SMPR1_OFFSET	=		0x14
ADC_SMPR1			=		ADC_BASE + ADC_SMPR1_OFFSET

ADC_SMPR2_OFFSET	=		0x18
ADC_SMPR2			=		ADC_BASE + ADC_SMPR2_OFFSET

ADC_SMPR2_SMP17		=		0x011 << 21

ADC_DIFSEL_OFFSET	=		0xB0
ADC_DIFSEL			=		ADC_BASE + ADC_DIFSEL_OFFSET

ADC_SQR1_OFFSET		=		0x30
ADC_SQR1			=		ADC_BASE + ADC_SQR1_OFFSET

ADC_SQR1_SQ1		=		17 << 6		@ Select channel 17 as first channel in s=ence

TS_CAL1				=		0x1FFF75A8
TS_CAL2				=		0x1FFF75CA

@
@ ADC
@
ADC_DR_OFFSET		=		0x40
ADC_DR				=		ADC_BASE + ADC_DR_OFFSET

RCC_AHB2ENR_ADCEN	=		1<<13   @ ADC Clock enable


@
@ DAC Definitions
@

RCC_APB1ENR1_DAC1EN	=		1 << 29

DAC_BASE			= 		0x40007400
DAC_CR_OFFSET		=		0x0
DAC_CR				=		DAC_BASE + DAC_CR_OFFSET

DAC_DHR12R1_OFFSET	=		0x08
DAC_DHR12R1			=		DAC_BASE + DAC_DHR12R1_OFFSET

DAC_CR_EN1			=		1 << 0


@
@ GPIO defs
@
GPIOA_BASE			=		AHB2PERIPH_BASE + 0x0000
GPIOB_BASE			=		AHB2PERIPH_BASE + 0x0400
GPIOC_BASE			=		AHB2PERIPH_BASE + 0x0800
GPIOD_BASE			=		AHB2PERIPH_BASE + 0x0C00
GPIOE_BASE			=		AHB2PERIPH_BASE + 0x1000
GPIOH_BASE			=		AHB2PERIPH_BASE + 0x1C00

GPIO_MODER_OFFSET	=		0x00
GPIO_AFRL_OFFSET	=		0x20

GPIOA_MODER			=		GPIOA_BASE + GPIO_MODER_OFFSET
GPIOA_AFRL			=		GPIOA_BASE + GPIO_AFRL_OFFSET
GPIOB_MODER			=		GPIOB_BASE + GPIO_MODER_OFFSET
GPIOB_AFRL			=		GPIOB_BASE + GPIO_AFRL_OFFSET

GPIO_ODR_OFFSET		=		0x14

GPIOA_ODR			=		GPIOA_BASE + GPIO_ODR_OFFSET
GPIOB_ODR			=		GPIOB_BASE + GPIO_ODR_OFFSET

GPIOx_MODER_P0_Pos		=	0
GPIOx_MODER_P0_Msk		=	(0x03 << GPIOx_MODER_P0_Pos)
GPIOx_MODER_P0			=	GPIOx_MODER_P0_Msk
GPIOx_MODER_P0_Input	=	(0x00 << GPIOx_MODER_P0_Pos)
GPIOx_MODER_P0_Output	=	(0x01 << GPIOx_MODER_P0_Pos)
GPIOx_MODER_P0_Altfn	=	(0x02 << GPIOx_MODER_P0_Pos)
GPIOx_MODER_P0_Analog	=	(0x03 << GPIOx_MODER_P0_Pos)

GPIOx_MODER_P1_Pos		=	2
GPIOx_MODER_P1_Msk		=	(0x03 << GPIOx_MODER_P1_Pos)
GPIOx_MODER_P1			=	GPIOx_MODER_P1_Msk
GPIOx_MODER_P1_Input	=	(0x00 << GPIOx_MODER_P1_Pos)
GPIOx_MODER_P1_Output	=	(0x01 << GPIOx_MODER_P1_Pos)
GPIOx_MODER_P1_Altfn	=	(0x02 << GPIOx_MODER_P1_Pos)
GPIOx_MODER_P1_Analog	=	(0x03 << GPIOx_MODER_P1_Pos)

GPIOx_MODER_P2_Pos		=	4
GPIOx_MODER_P2_Msk		=	(0x03 << GPIOx_MODER_P2_Pos)
GPIOx_MODER_P2			=	GPIOx_MODER_P2_Msk
GPIOx_MODER_P2_Input	=	(0x00 << GPIOx_MODER_P2_Pos)
GPIOx_MODER_P2_Output	=	(0x01 << GPIOx_MODER_P2_Pos)
GPIOx_MODER_P2_Altfn	=	(0x02 << GPIOx_MODER_P2_Pos)
GPIOx_MODER_P2_Analog	=	(0x03 << GPIOx_MODER_P2_Pos)

GPIOx_MODER_P3_Pos		=	6
GPIOx_MODER_P3_Msk		=	(0x03 << GPIOx_MODER_P3_Pos)
GPIOx_MODER_P3			=	GPIOx_MODER_P3_Msk
GPIOx_MODER_P3_Input	=	(0x00 << GPIOx_MODER_P3_Pos)
GPIOx_MODER_P3_Output	=	(0x01 << GPIOx_MODER_P3_Pos)
GPIOx_MODER_P3_Altfn	=	(0x02 << GPIOx_MODER_P3_Pos)
GPIOx_MODER_P3_Analog	=	(0x03 << GPIOx_MODER_P3_Pos)

GPIOx_MODER_P4_Pos		=	8
GPIOx_MODER_P4_Msk		=	(0x03 << GPIOx_MODER_P4_Pos)
GPIOx_MODER_P4			=	GPIOx_MODER_P4_Msk
GPIOx_MODER_P4_Input	=	(0x00 << GPIOx_MODER_P4_Pos)
GPIOx_MODER_P4_Output	=	(0x01 << GPIOx_MODER_P4_Pos)
GPIOx_MODER_P4_Altfn	=	(0x02 << GPIOx_MODER_P4_Pos)
GPIOx_MODER_P4_Analog	=	(0x03 << GPIOx_MODER_P4_Pos)

GPIOx_MODER_P5_Pos		=	10
GPIOx_MODER_P5_Msk		=	(0x03 << GPIOx_MODER_P5_Pos)
GPIOx_MODER_P5			=	GPIOx_MODER_P5_Msk
GPIOx_MODER_P5_Input	=	(0x00 << GPIOx_MODER_P5_Pos)
GPIOx_MODER_P5_Output	=	(0x01 << GPIOx_MODER_P5_Pos)
GPIOx_MODER_P5_Altfn	=	(0x02 << GPIOx_MODER_P5_Pos)
GPIOx_MODER_P5_Analog	=	(0x03 << GPIOx_MODER_P5_Pos)

GPIOx_MODER_P6_Pos		=	12
GPIOx_MODER_P6_Msk		=	(0x03 << GPIOx_MODER_P6_Pos)
GPIOx_MODER_P6			=	GPIOx_MODER_P6_Msk
GPIOx_MODER_P6_Input	=	(0x00 << GPIOx_MODER_P6_Pos)
GPIOx_MODER_P6_Output	=	(0x01 << GPIOx_MODER_P6_Pos)
GPIOx_MODER_P6_Altfn	=	(0x02 << GPIOx_MODER_P6_Pos)
GPIOx_MODER_P6_Analog	=	(0x03 << GPIOx_MODER_P6_Pos)

GPIOx_MODER_P7_Pos		=	14
GPIOx_MODER_P7_Msk		=	(0x03 << GPIOx_MODER_P7_Pos)
GPIOx_MODER_P7			=	GPIOx_MODER_P7_Msk
GPIOx_MODER_P7_Input	=	(0x00 << GPIOx_MODER_P7_Pos)
GPIOx_MODER_P7_Output	=	(0x01 << GPIOx_MODER_P7_Pos)
GPIOx_MODER_P7_Altfn	=	(0x02 << GPIOx_MODER_P7_Pos)
GPIOx_MODER_P7_Analog	=	(0x03 << GPIOx_MODER_P7_Pos)

GPIOx_MODER_P8_Pos		=	16
GPIOx_MODER_P8_Msk		=	(0x03 << GPIOx_MODER_P8_Pos)
GPIOx_MODER_P8			=	GPIOx_MODER_P8_Msk
GPIOx_MODER_P8_Input	=	(0x00 << GPIOx_MODER_P8_Pos)
GPIOx_MODER_P8_Output	=	(0x01 << GPIOx_MODER_P8_Pos)
GPIOx_MODER_P8_Altfn	=	(0x02 << GPIOx_MODER_P8_Pos)
GPIOx_MODER_P8_Analog	=	(0x03 << GPIOx_MODER_P8_Pos)

GPIOx_MODER_P9_Pos		=	18
GPIOx_MODER_P9_Msk		=	(0x03 << GPIOx_MODER_P9_Pos)
GPIOx_MODER_P9			=	GPIOx_MODER_P9_Msk
GPIOx_MODER_P9_Input	=	(0x00 << GPIOx_MODER_P9_Pos)
GPIOx_MODER_P9_Output	=	(0x01 << GPIOx_MODER_P9_Pos)
GPIOx_MODER_P9_Altfn	=	(0x02 << GPIOx_MODER_P9_Pos)
GPIOx_MODER_P9_Analog	=	(0x03 << GPIOx_MODER_P9_Pos)

GPIOx_MODER_P10_Pos		=	20
GPIOx_MODER_P10_Msk		=	(0x03 << GPIOx_MODER_P10_Pos)
GPIOx_MODER_P10			=	GPIOx_MODER_P10_Msk
GPIOx_MODER_P10_Input	=	(0x00 << GPIOx_MODER_P10_Pos)
GPIOx_MODER_P10_Output	=	(0x01 << GPIOx_MODER_P10_Pos)
GPIOx_MODER_P10_Altfn	=	(0x02 << GPIOx_MODER_P10_Pos)
GPIOx_MODER_P10_Analog	=	(0x03 << GPIOx_MODER_P10_Pos)

GPIOx_MODER_P11_Pos		=	22
GPIOx_MODER_P11_Msk		=	(0x03 << GPIOx_MODER_P11_Pos)
GPIOx_MODER_P11			=	GPIOx_MODER_P11_Msk
GPIOx_MODER_P11_Input	=	(0x00 << GPIOx_MODER_P11_Pos)
GPIOx_MODER_P11_Output	=	(0x01 << GPIOx_MODER_P11_Pos)
GPIOx_MODER_P11_Altfn	=	(0x02 << GPIOx_MODER_P11_Pos)
GPIOx_MODER_P11_Analog	=	(0x03 << GPIOx_MODER_P11_Pos)

GPIOx_MODER_P12_Pos		=	24
GPIOx_MODER_P12_Msk		=	(0x03 << GPIOx_MODER_P12_Pos)
GPIOx_MODER_P12			=	GPIOx_MODER_P12_Msk
GPIOx_MODER_P12_Input	=	(0x00 << GPIOx_MODER_P12_Pos)
GPIOx_MODER_P12_Output	=	(0x01 << GPIOx_MODER_P12_Pos)
GPIOx_MODER_P12_Altfn	=	(0x02 << GPIOx_MODER_P12_Pos)
GPIOx_MODER_P12_Analog	=	(0x03 << GPIOx_MODER_P12_Pos)

GPIOx_MODER_P13_Pos		=	26
GPIOx_MODER_P13_Msk		=	(0x03 << GPIOx_MODER_P13_Pos)
GPIOx_MODER_P13			=	GPIOx_MODER_P13_Msk
GPIOx_MODER_P13_Input	=	(0x00 << GPIOx_MODER_P13_Pos)
GPIOx_MODER_P13_Output	=	(0x01 << GPIOx_MODER_P13_Pos)
GPIOx_MODER_P13_Altfn	=	(0x02 << GPIOx_MODER_P13_Pos)
GPIOx_MODER_P13_Analog	=	(0x03 << GPIOx_MODER_P13_Pos)

GPIOx_MODER_P14_Pos		=	28
GPIOx_MODER_P14_Msk		=	(0x03 << GPIOx_MODER_P14_Pos)
GPIOx_MODER_P14			=	GPIOx_MODER_P14_Msk
GPIOx_MODER_P14_Input	=	(0x00 << GPIOx_MODER_P14_Pos)
GPIOx_MODER_P14_Output	=	(0x01 << GPIOx_MODER_P14_Pos)
GPIOx_MODER_P14_Altfn	=	(0x02 << GPIOx_MODER_P14_Pos)
GPIOx_MODER_P14_Analog	=	(0x03 << GPIOx_MODER_P14_Pos)

GPIOx_MODER_P15_Pos		=	30
GPIOx_MODER_P15_Msk		=	(0x03 << GPIOx_MODER_P15_Pos)
GPIOx_MODER_P15			=	GPIOx_MODER_P15_Msk
GPIOx_MODER_P15_Input	=	(0x00 << GPIOx_MODER_P15_Pos)
GPIOx_MODER_P15_Output	=	(0x01 << GPIOx_MODER_P15_Pos)
GPIOx_MODER_P15_Altfn	=	(0x02 << GPIOx_MODER_P15_Pos)
GPIOx_MODER_P15_Analog	=	(0x03 << GPIOx_MODER_P15_Pos)

GPIOx_ODR_OD0			=	(0x01 << 0)
GPIOx_ODR_OD1			=	(0x01 << 1)
GPIOx_ODR_OD2			=	(0x01 << 2)
GPIOx_ODR_OD3			=	(0x01 << 3)
GPIOx_ODR_OD4			=	(0x01 << 4)
GPIOx_ODR_OD5			=	(0x01 << 5)
GPIOx_ODR_OD6			=	(0x01 << 6)
GPIOx_ODR_OD7			=	(0x01 << 7)
GPIOx_ODR_OD8			=	(0x01 << 8)
GPIOx_ODR_OD9			=	(0x01 << 9)
GPIOx_ODR_OD10			=	(0x01 << 10)
GPIOx_ODR_OD11			=	(0x01 << 11)
GPIOx_ODR_OD12			=	(0x01 << 12)
GPIOx_ODR_OD13			=	(0x01 << 13)
GPIOx_ODR_OD14			=	(0x01 << 14)
GPIOx_ODR_OD15			=	(0x01 << 15)


GPIO_SPEED_FREQ_LOW			=	0x00000000
GPIO_SPEED_FREQ_MEDIUM		=	0x00000001
GPIO_SPEED_FREQ_HIGH		=	0x00000002
GPIO_SPEED_FREQ_VERY_HIGH	=	0x00000003


GPIOx_AFRL_AFSEL0_Pos		= 0
GPIOx_AFRL_AFSEL0_Msk		= (0x0f << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0			= GPIOx_AFRL_AFSEL0_Msk
GPIOx_AFRL_AFSEL0_AF0		= (0x00 << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF1		= (0x01 << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF2		= (0x02 << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF3		= (0x03 << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF4		= (0x04 << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF5		= (0x05 << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF6		= (0x06 << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF7		= (0x07 << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF8		= (0x08 << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF9		= (0x09 << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF10		= (0x0a << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF11		= (0x0b << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF12		= (0x0c << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF13		= (0x0d << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF14		= (0x0e << GPIOx_AFRL_AFSEL0_Pos)
GPIOx_AFRL_AFSEL0_AF15		= (0x0f << GPIOx_AFRL_AFSEL0_Pos)

GPIOx_AFRL_AFSEL6_Pos		= 24
GPIOx_AFRL_AFSEL6_Msk		= (0x0f << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6			= GPIOx_AFRL_AFSEL6_Msk
GPIOx_AFRL_AFSEL6_AF0		= (0x00 << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF1		= (0x01 << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF2		= (0x02 << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF3		= (0x03 << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF4		= (0x04 << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF5		= (0x05 << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF6		= (0x06 << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF7		= (0x07 << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF8		= (0x08 << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF9		= (0x09 << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF10		= (0x0a << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF11		= (0x0b << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF12		= (0x0c << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF13		= (0x0d << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF14		= (0x0e << GPIOx_AFRL_AFSEL6_Pos)
GPIOx_AFRL_AFSEL6_AF15		= (0x0f << GPIOx_AFRL_AFSEL6_Pos)

@
@ TIMx control register 1 (TIMx_CR1)(x = 2 to 3)
@
TIMx_CR1_OFFSET				= 0x00
TIMx_CR1					= APB1PERIPH_BASE + TIMx_CR1_OFFSET

TIMx_CR1_CEN				= 0x01 << 0		@ Counter enable
TIMx_CR1_UDIS				= 0x01 << 1		@ Update disable
TIMx_CR1_URS				= 0x01 << 2
TIMx_CR1_OPM				= 0x01 << 3
TIMx_CR1_DIR				= 0x01 << 4
TIMx_CR1_CMS_1				= 0x00 << 5
TIMx_CR1_CMS_2				= 0x01 << 5
TIMx_CR1_CMS_3				= 0x02 << 5
TIMx_CR1_CMS_4				= 0x03 << 5

TIMx_CR1_ARPE				= 0x01 << 7
TIMx_CR1_CKD_1				= 0x01 << 8
TIMx_CR1_CKD_2				= 0x02 << 8
TIMx_CR1_CKD_3				= 0x03 << 8
TIMx_CR1_UIFREMAP			= 0x01 << 11

@
@ TIMx control register 2 (TIMx_CR2)(x = 2 to 3)
@
TIMx_CR2_OFFSET				= 0x04
TIMx_CR2					= APB1PERIPH_BASE + TIMx_CR2_OFFSET

TIMx_CR2_CCDS				= 0x01 << 3
TIMx_CR2_MMS_Reset			= 0x00 << 4
TIMx_CR2_MMS_Enable			= 0x01 << 4
TIMx_CR2_MMS_Update			= 0x02 << 4
TIMx_CR2_MMS_Compare		= 0x03 << 4
TIMx_CR2_MMS_Compare_1		= 0x04 << 4
TIMx_CR2_MMS_Compare_2		= 0x05 << 4
TIMx_CR2_MMS_Compare_3 		= 0x06 << 4
TIMx_CR2_MMS_Compare_4		= 0x07 << 4

TIMx_CR2_TI1S				= 0x01 << 7

@
@ TIMx slave mode control register (TIMx_SMCR)(x = 2 to 3)
@
TIMx_SMCR_OFFSET			= 0x08
TIMx_SMCR					= APB1PERIPH_BASE + TIMx_SMCR_OFFSET

TIMx_SMCR_SMS				= 0x01 << 0


@
@ TIMx DMA/Interrupt enable register (TIMx_DIER)(x = 2 to 3)
@
TIMx_DIER_OFFSET			= 0x0c
TIMx_DIER					= APB1PERIPH_BASE + TIMx_DIER_OFFSET

TIMx_DIER_UIE				= 0x01 << 0			@ Update interrupt enable
TIMx_DIER_CC1IE				= 0x01 << 1
TIMx_DIER_CC2IE				= 0x01 << 2
TIMx_DIER_CC3IE				= 0x01 << 3
TIMx_DIER_CC4IE				= 0x01 << 4			@ Capture/Compare 4 interrupt enable
TIMx_DIER_TIE				= 0x01 << 6			@ Trigger interrupt enable
TIMx_DIER_UDE				= 0x01 << 8			@ Update DMA request enable
TIMx_DIER_CC1DE				= 0x01 << 9			@ Capture/Compare 1 DMA request enable
TIMx_DIER_CC2DE				= 0x01 << 10		@ Capture/Compare 2 DMA request enable
TIMx_DIER_CC3DE				= 0x01 << 11		@ Capture/Compare 3 DMA request enable
TIMx_DIER_CC4DE				= 0x01 << 12		@ Capture/Compare 4 DMA request enable
TIMx_DIER_TDE				= 0x01 << 14		@ Trigger DMA Request enable

@
@ TIMx status register (TIMx_SR)(x = 2 to 3)
@
TIMx_SR_OFFSET				= 0x10
TIMx_SR						= APB1PERIPH_BASE + TIMx_SR_OFFSET

TIMx_SR_UIF					= 0x01 << 0			@ Update interrupt flag
TIMx_SR_CC1IF				= 0x01 << 1			@ Capture/Compare 1 interrupt flag
TIMx_SR_CC2IF				= 0x01 << 2			@ Capture/Compare 2 interrupt flag
TIMx_SR_CC3IF				= 0x01 << 3			@ Capture/Compare 3 interrupt flag
TIMx_SR_CC4IF				= 0x01 << 4			@ Capture/Compare 4 interrupt flag
TIMx_SR_TIF					= 0x01 << 6			@ Trigger interrupt flag
TIMx_SR_CC1OF				= 0x01 << 9			@ Capture/Compare 1 overcapture flag
TIMx_SR_CC2OF				= 0x01 << 10		@ Capture/Compare 2 overcapture flag
TIMx_SR_CC3OF				= 0x01 << 11		@ Capture/Compare 3 overcapture flag
TIMx_SR_CC4OF				= 0x01 << 12		@ Capture/Compare 4 overcapture flag

@
@ TIMx event generation register (TIMx_EGR)(x = 2 to 3)
@
TIMx_EGR_OFFSET				= 0x14
TIMx_EGR					= APB1PERIPH_BASE + TIMx_EGR_OFFSET

TIMx_EGR_UG					= 0x01 << 0			@ Trigger generation
TIMx_EGR_CC1G				= 0x01 << 1
TIMx_EGR_CC2G				= 0x01 << 2
TIMx_EGR_CC3G				= 0x01 << 3
TIMx_EGR_CC4G				= 0x01 << 4
TIMx_EGR_TG					= 0x01 << 5

@
@ TIMx capture/compare mode register 1 (TIMx_CCMR1)(x = 2 to 3)
@
TIMx_CCMR1_OFFSET			= 0x14
TIMx_CCMR1					= APB1PERIPH_BASE + TIMx_CCMR1_OFFSET


@
@ TIMx capture/compare mode register 2 (TIMx_CCMR2)(x = 2 to 3)
@
TIMx_CCMR2_OFFSET			= 0x1c
TIMx_CCMR2					= APB1PERIPH_BASE + TIMx_CCMR2_OFFSET

@
@ TIMx capture/compare enable register (TIMx_CCER)(x = 2 to 3)
@
TIMx_CCER_OFFSET			= 0x1c
TIMx_CCER					= APB1PERIPH_BASE + TIMx_CCER_OFFSET

@
@ TIMx counter (TIMx_CNT)(x = 2 to 3)
@
TIMx_CNT_OFFSET				= 0x24
TIMx_CNT					= APB1PERIPH_BASE + TIMx_CNT_OFFSET

@
@ TIMx prescaler (TIMx_PSC)(x = 2 to 3)
@
TIMx_PSC_OFFSET				= 0x28
TIMx_PSC					= APB1PERIPH_BASE + TIMx_PSC_OFFSET

@
@ TIMx auto-reload register (TIMx_ARR)(x = 2 to 3)
@
TIMx_ARR_OFFSET				= 0x2c
TIMx_ARR					= APB1PERIPH_BASE + TIMx_ARR_OFFSET

@
@ TIMx capture/compare register 2 (TIMx_CCR2)(x = 2 to 3)
@
TIMx_CCR2_OFFSET			= 0x38
TIMx_CCR2					= APB1PERIPH_BASE + TIMx_CCR2_OFFSET

@
@ TIMx capture/compare register 3 (TIMx_CCR3)(x = 2 to 3)
@
TIMx_CCR3_OFFSET			= 0x3c
TIMx_CCR3					= APB1PERIPH_BASE + TIMx_CCR3_OFFSET

@
@ TIMx capture/compare register 4 (TIMx_CCR4)(x = 2 to 3)
@
TIMx_CCR4_OFFSET			= 0x40
TIMx_CCR4					= APB1PERIPH_BASE + TIMx_CCR4_OFFSET

@
@ TIMx DMA control register (TIMx_DCR)(x = 2 to 3)
@
TIMx_DCR_OFFSET				= 0x48
TIMx_DCR					= APB1PERIPH_BASE + TIMx_DCR_OFFSET

@
@ TIMx DMA address for full transfer (TIMx_DMAR)(x = 2 to 3)
@
TIMx_DMAR_OFFSET			= 0x4c
TIMx_DMAR					= APB1PERIPH_BASE + TIMx_DMAR_OFFSET

@
@ TIM2 option register 1 (TIM2_OR1)
@
TIM2_OR1_OFFSET				= 0x50
TIM2_OR1					= APB1PERIPH_BASE + TIM2_OR1_OFFSET

@
@ TIM2 option register 2 (TIM2_OR2)
@
TIM2_OR2_OFFSET				= 0x60
TIM2_OR2					= APB1PERIPH_BASE + TIM2_OR2_OFFSET


@
@ TIM15
@
TIM15_BASE					= 0x40014000

@
@ TIM16
@
TIM16_BASE					= 0x40014400
TIM16_CR1_OFFSET			= 0x00
TIM16_CR1					= TIM16_BASE + TIM16_CR1_OFFSET

TIM16_CR1_CEN				= 0x01 << 0
TIM16_CR1_UDIS				= 0x01 << 1
TIM16_CR1_URS				= 0x01 << 2
TIM16_CR1_ARPE				= 0x01 << 7
TIM16_CR1_CKD1				= 0x00 << 8
TIM16_CR1_CKD2				= 0x01 << 8
TIM16_CR1_CKD3				= 0x02 << 8
TIM16_CR1_CKD4				= 0x03 << 8
TIM16_CR1_UIFREMAP			= 0x01 << 11

TIM16_CR2_OFFSET			= 0x04
TIM16_CR2					= TIM16_BASE + TIM16_CR2_OFFSET
TIM16_CR2_CCPC				= 0x01 << 0
TIM16_CR2_CCUS				= 0x01 << 2
TIM16_CR2_CCDS				= 0x01 << 3
TIM16_CR2_OIS1				= 0x01 << 8
TIM16_CR2_OIS1N				= 0x01 << 9

TIM16_DIER_OFFSET			= 0x0c
TIM16_DIER					= TIM16_BASE + TIM16_DIER_OFFSET
TIM16_DIER_UIE				= 0x01 << 0
TIM16_DIER_CC1IE			= 0x01 << 1
TIM16_DIER_COMIE			= 0x01 << 5
TIM16_DIER_BIE				= 0x01 << 7
TIM16_DIER_UDE				= 0x01 << 8
TIM16_DIER_CC1DE			= 0x01 << 9
TIM16_DIER_COMDE			= 0x01 << 13

TIM16_SR_OFFSET				= 0x10
TIM16_SR					= TIM16_BASE + TIM16_SR_OFFSET
TIM16_SR_UIF				= 0x01 << 0
TIM16_SR_CC1IF				= 0x01 << 1
TIM16_SR_COMIF				= 0x01 << 5
TIM16_SR_BIF				= 0x01 << 7
TIM16_SR_CC1OF				= 0x01 << 9

TIM16_EGR_OFFSET			= 0x14
TIM16_EGR					= TIM16_BASE + TIM16_EGR_OFFSET
TIM16_EGR_UG				= 0x01 << 0
TIM16_EGR_CC1G				= 0x01 << 1
TIM16_EGR_COMG				= 0x01 << 5
TIM16_EGR_BG				= 0x01 << 7

TIM16_CCMR1_OFFSET			= 0x18
TIM16_CCMR1					= TIM16_BASE + TIM16_CCMR1_OFFSET
TIM16_CCMR1_CC1S			= 0x01 << 0
TIM16_CCMR1_OC1FE			= 0x01 << 2
TIM16_CCMR1_OC1PE			= 0x01 << 3

TIM16_CCMR1_IC1PSC0			= 0x00 << 2
TIM16_CCMR1_IC1PSC1			= 0x01 << 2
TIM16_CCMR1_IC1PSC2			= 0x02 << 2
TIM16_CCMR1_IC1PSC3			= 0x03 << 2

TIM16_CCMR1_OC1M_PWM1		= 0x06 << 4
TIM16_CCMR1_OC1M_PWM2		= 0x07 << 4
TIM16_CCMR1_OC1M_2			= 0x01 << 16

TIM16_CCMR1_IC1F_Pos		= 4
TIM16_CCMR1_IC1F_Msk		= (0x0f << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F			= TIM16_CCMR1_IC1F_Msk
TIM16_CCMR1_IC1F_0			= (0x00 << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_1			= (0x01 << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_2			= (0x02 << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_3			= (0x03 << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_4			= (0x04 << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_5			= (0x05 << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_6			= (0x06 << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_7			= (0x07 << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_8			= (0x08 << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_9			= (0x09 << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_10			= (0x0a << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_11			= (0x0b << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_12			= (0x0c << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_13			= (0x0d << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_14			= (0x0e << TIM16_CCMR1_IC1F_Pos)
TIM16_CCMR1_IC1F_15			= (0x0f << TIM16_CCMR1_IC1F_Pos)

TIM16_CCER_OFFSET			= 0x20
TIM16_CCER					= TIM16_BASE + TIM16_CCER_OFFSET
TIM16_CCER_CC1E				= 0x01 << 0
TIM16_CCER_CC1P				= 0x01 << 1
TIM16_CCER_CC1NE			= 0x01 << 2
TIM16_CCER_CC1NP			= 0x00 << 3

TIM16_CNT_OFFSET			= 0x24
TIM16_CNT					= TIM16_BASE + TIM16_CNT_OFFSET

TIM16_PSC_OFFSET			= 0x28
TIM16_PSC					= TIM16_BASE + TIM16_PSC_OFFSET

TIM16_ARR_OFFSET			= 0x2c
TIM16_ARR					= TIM16_BASE + TIM16_ARR_OFFSET

TIM16_RCR_OFFSET			= 0x30
TIM16_RCR					= TIM16_BASE + TIM16_RCR_OFFSET

TIM16_CCR1_OFFSET			= 0x34
TIM16_CCR1					= TIM16_BASE + TIM16_CCR1_OFFSET

TIM16_BDTR_OFFSET			= 0x44
TIM16_BDTR					= TIM16_BASE + TIM16_BDTR_OFFSET
TIM16_BDTR_DTG				= 0x00 << 0
TIM16_BDTR_LOCK				= 0x00 << 8
TIM16_BDTR_OSSI				= 0x01 << 10
TIM16_BDTR_OSSR				= 0x01 << 11
TIM16_BDTR_BKE				= 0x01 << 12
TIM16_BDTR_BKP				= 0x01 << 13
TIM16_BDTR_AOE				= 0x01 << 14
TIM16_BDTR_MOE				= 0x01 << 15

TIM16_DCR_OFFSET			= 0x48
TIM16_DCR					= TIM16_BASE + TIM16_DCR_OFFSET
TIM16_DCR_DBA				= 0x00 << 0
TIM16_DCR_DBL				= 0x00 << 8

TIM16_OR1_OFFSET			= 0x50
TIM16_OR1					= TIM16_BASE + TIM16_OR1_OFFSET
TIM16_OR1_TI1_RMP_IO		= 0x00 << 0
TIM16_OR1_TI1_RMP_LSI		= 0x01 << 0
TIM16_OR1_TI1_RMP_LSE		= 0x02 << 0
TIM16_OR1_TI1_RMP_RTC_WU	= 0x03 << 0
TIM16_OR1_TI1_RMP_MSI		= 0x04 << 0
TIM16_OR1_TI1_RMP_HSE32		= 0x05 << 0
TIM16_OR1_TI1_RMP_MCO		= 0x06 << 0

TIM16_OR2_OFFSET			= 0x60
TIM16_OR2					= TIM16_BASE + TIM16_OR2_OFFSET
TIM16_OR2_BKINE				= 0x01 << 0
TIM16_OR2_BKCMPIE			= 0x01 << 1
TIM16_OR2_BKCMP2E			= 0x01 << 2
TIM16_OR2_BKINP				= 0x01 << 9
TIM16_OR2_BKCMP1P			= 0x01 << 10
TIM16_OR2_BKCMP2P			= 0x01 << 11


@
@ RCC_AHB2ENR AHB2 Peripheral clock enable register values
@
RCC_AHB2ENR_GPIOA_EN	=		1<<0   	@ GPIOA Enable
RCC_AHB2ENR_GPIOB_EN	=		1<<1   	@ GPIOB Enable
RCC_AHB2ENR_GPIOC_EN	=		1<<2   	@ GPIOC Enable
RCC_AHB2ENR_GPIOD_EN	=		1<<3   	@ GPIOD Enable
RCC_AHB2ENR_GPIOE_EN	=		1<<4   	@ GPIOE Enable
RCC_AHB2ENR_GPIOH_EN	=		1<<7   	@ GPIOH Enable

RCC_AHB2ENR_ADCEN		=		1<<13   @ ADC Enable
RCC_AHB2ENR_AESEN		=		1<<16	@ AES Enable
RCC_AHB2ENR_RNGEN		=		1<<18	@ RNG Enable


MODER3_OUT			=		1<<6	// Bit 6 is MODER3[1:0]     01 = output
MODER3_OUT			=		1<<6	// Bit 6 is MODER3[1:0]     01 = output

LED_ON				=		GPIOx_ODR_OD3	@ User LED3 Connected to GPIO PB3

PA6_ON				=		GPIOx_ODR_OD6

PA8_ON				=		GPIOx_ODR_OD8
