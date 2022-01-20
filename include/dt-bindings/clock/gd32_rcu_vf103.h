/*
 * Copyright (c) 2022 TOKITA Hiroshi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_GD32_RCU_VF103_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_GD32_RCU_VF103_H_

/**
 * Encode RCU register offset and configuration bit.
 *
 * - 0..5: bit number
 * - 6..14: offset
 * - 15: reserved
 *
 * @param reg RCU register name (expands to GD32_{reg}_OFFSET)
 * @param bit Configuration bit
 */
#define GD32_RCU_CONFIG(reg, bit) \
	(((GD32_ ## reg ## _OFFSET) << 6U) | (bit))

/**
 * @name Register offsets
 * @{
 */

#define GD32_AHBEN_OFFSET        0x14U
#define GD32_APB1EN_OFFSET       0x1CU
#define GD32_APB2EN_OFFSET       0x18U
#define GD32_BDCTL_OFFSET        0x20U

/** @} */

/**
 * @name Clock enable/disable definitions for peripherals
 * @{
 */

/* AHB peripherals */
#define GD32_RCU_DMA0       GD32_RCU_CONFIG(AHBEN, 0U)
#define GD32_RCU_DMA1       GD32_RCU_CONFIG(AHBEN, 1U)
#define GD32_RCU_CRC        GD32_RCU_CONFIG(AHBEN, 6U)
#define GD32_RCU_EXMC       GD32_RCU_CONFIG(AHBEN, 8U)
#define GD32_RCU_USBFS      GD32_RCU_CONFIG(AHBEN, 12U)

/* APB1 peripherals */
#define GD32_RCU_TIMER1     GD32_RCU_CONFIG(APB1EN, 0U)
#define GD32_RCU_TIMER2     GD32_RCU_CONFIG(APB1EN, 1U)
#define GD32_RCU_TIMER3     GD32_RCU_CONFIG(APB1EN, 2U)
#define GD32_RCU_TIMER4     GD32_RCU_CONFIG(APB1EN, 3U)
#define GD32_RCU_TIMER5     GD32_RCU_CONFIG(APB1EN, 4U)
#define GD32_RCU_TIMER6     GD32_RCU_CONFIG(APB1EN, 5U)
#define GD32_RCU_WWDGT      GD32_RCU_CONFIG(APB1EN, 11U)
#define GD32_RCU_SPI1       GD32_RCU_CONFIG(APB1EN, 14U)
#define GD32_RCU_SPI2       GD32_RCU_CONFIG(APB1EN, 15U)
#define GD32_RCU_USART1     GD32_RCU_CONFIG(APB1EN, 17U)
#define GD32_RCU_USART2     GD32_RCU_CONFIG(APB1EN, 18U)
#define GD32_RCU_UART3      GD32_RCU_CONFIG(APB1EN, 19U)
#define GD32_RCU_UART4      GD32_RCU_CONFIG(APB1EN, 20U)
#define GD32_RCU_I2C0       GD32_RCU_CONFIG(APB1EN, 21U)
#define GD32_RCU_I2C1       GD32_RCU_CONFIG(APB1EN, 22U)
#define GD32_RCU_CAN0       GD32_RCU_CONFIG(APB1EN, 25U)
#define GD32_RCU_CAN1       GD32_RCU_CONFIG(APB1EN, 26U)
#define GD32_RCU_BKPI       GD32_RCU_CONFIG(APB1EN, 27U)
#define GD32_RCU_PMU        GD32_RCU_CONFIG(APB1EN, 28U)
#define GD32_RCU_DAC        GD32_RCU_CONFIG(APB1EN, 29U)
#define GD32_RCU_RTC        GD32_RCU_CONFIG(BDCTL, 15U)

/* APB2 peripherals */
#define GD32_RCU_AF         GD32_RCU_CONFIG(APB2EN, 0U)
#define GD32_RCU_GPIOA      GD32_RCU_CONFIG(APB2EN, 2U)
#define GD32_RCU_GPIOB      GD32_RCU_CONFIG(APB2EN, 3U)
#define GD32_RCU_GPIOC      GD32_RCU_CONFIG(APB2EN, 4U)
#define GD32_RCU_GPIOD      GD32_RCU_CONFIG(APB2EN, 5U)
#define GD32_RCU_GPIOE      GD32_RCU_CONFIG(APB2EN, 6U)
#define GD32_RCU_ADC0       GD32_RCU_CONFIG(APB2EN, 9U)
#define GD32_RCU_ADC1       GD32_RCU_CONFIG(APB2EN, 10U)
#define GD32_RCU_TIMER0     GD32_RCU_CONFIG(APB2EN, 11U)
#define GD32_RCU_SPI0       GD32_RCU_CONFIG(APB2EN, 12U)
#define GD32_RCU_USART0     GD32_RCU_CONFIG(APB2EN, 14U)


/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_GD32_RCU_VF103_H_ */
