/*
 * Copyright (c) 2023 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT qorvo_pac55xx_usart

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>

/* System and clock controller (FIXME: use syscon) */
#define SCC_BASE DT_REG_ADDR(DT_NODELABEL(scc))

/* UART register offsets */
#define UARTXRBR 0x00U
#define UARTXTHR 0x04U
#define UARTXDLR 0x08U
#define UARTXFCR 0x14U
#define UARTXLCR 0x18U
#define UARTXLSR 0x20U

/* UARTXRBR fields/flags */
#define UARTXRBR_RDR  BIT(0U)
#define UARTXRBR_OE   BIT(1U)
#define UARTXRBR_PE   BIT(2U)
#define UARTXRBR_FE   BIT(3U)
#define UARTXRBR_TEMT BIT(6U)
#define UARTXRBR_RXFE BIT(7U)

/* UARTXFCR fields/flags */
#define UARTXFCR_FIFOEN    BIT(0U)
#define UARTXFCR_RXFIFORST BIT(1U)
#define UARTXFCR_TXFIFORST BIT(2U)

/* UARTXLCR fields/flags */
#define UARTXLCR_WLS  GENMASK(1U, 0U)
#define UARTXLCR_SBS  GENMASK(2U, 2U)
#define UARTXLCR_PEN  BIT(3U)
#define UARTXLCR_PSEL GENMASK(5U, 4U)

struct pac55xx_uart_config {
	uint32_t reg;
	uint8_t mode_flag;
	struct uart_config init_config;
	const struct pinctrl_dev_config *pcfg;
};

static int pac55xx_uart_poll_in(const struct device *dev, unsigned char *c)
{
	const struct pac55xx_uart_config *const config = dev->config;

	if ((sys_read32(config->reg + UARTXLSR) & UARTXRBR_RDR) == 0U) {
		return -1;
	}

	*c = sys_read8(config->reg + UARTXRBR);

	return 0;
}

static void pac55xx_uart_poll_out(const struct device *dev, unsigned char c)
{
	const struct pac55xx_uart_config *const config = dev->config;

	sys_write8(c, config->reg + UARTXTHR);

	while ((sys_read32(config->reg + UARTXLSR) & UARTXRBR_TEMT) == 0U) {
		;
	}
}

static int pac55xx_uart_configure(const struct device *dev, const struct uart_config *cfg)
{
	const struct pac55xx_uart_config *const config = dev->config;
	uint8_t val = 0U;

	switch (cfg->data_bits) {
	case UART_CFG_DATA_BITS_5:
		val |= FIELD_PREP(UARTXLCR_WLS, 0U);
		break;
	case UART_CFG_DATA_BITS_6:
		val |= FIELD_PREP(UARTXLCR_WLS, 1U);
		break;
	case UART_CFG_DATA_BITS_7:
		val |= FIELD_PREP(UARTXLCR_WLS, 2U);
		break;
	case UART_CFG_DATA_BITS_8:
		val |= FIELD_PREP(UARTXLCR_WLS, 3U);
		break;
	default:
		return -ENOTSUP;
	}

	switch (cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		val |= FIELD_PREP(UARTXLCR_SBS, 0U);
		break;
	case UART_CFG_STOP_BITS_1_5:
		if (cfg->data_bits != UART_CFG_DATA_BITS_5) {
			return -ENOTSUP;
		}
		val |= FIELD_PREP(UARTXLCR_SBS, 1U);
		break;
	case UART_CFG_STOP_BITS_2:
		if (cfg->data_bits == UART_CFG_DATA_BITS_5) {
			return -ENOTSUP;
		}
		val |= FIELD_PREP(UARTXLCR_SBS, 1U);
		break;
	default:
		return -ENOTSUP;
	}

	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		break;
	case UART_CFG_PARITY_ODD:
		val |= UARTXLCR_PEN | FIELD_PREP(UARTXLCR_PSEL, 0U);
		break;
	case UART_CFG_PARITY_EVEN:
		val |= UARTXLCR_PEN | FIELD_PREP(UARTXLCR_PSEL, 1U);
		break;
	default:
		return -ENOTSUP;
	}

	sys_write8(val, config->reg + UARTXLCR);

	/* FIXME: use clock controller, so that PCLKDIV can be handled */
	sys_write16(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / (16U * cfg->baudrate),
		    config->reg + UARTXDLR);

	return 0;
}

static int pac55xx_uart_err_check(const struct device *dev)
{
	const struct pac55xx_uart_config *const config = dev->config;
	uint32_t status;
	int errors = 0;

	status = sys_read32(config->reg + UARTXLSR);

	if (status & UARTXRBR_OE) {
		errors |= UART_ERROR_OVERRUN;
	}

	if (status & UARTXRBR_OE) {
		errors |= UART_ERROR_PARITY;
	}

	if (status & UARTXRBR_FE) {
		errors |= UART_ERROR_FRAMING;
	}

	return errors;
}

static int pac55xx_uart_init(const struct device *dev)
{
	const struct pac55xx_uart_config *const config = dev->config;
	uint32_t val;
	int ret;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* configure UART mode (FIXME: use syscon) */
	val = sys_read32(SCC_BASE) | BIT(config->mode_flag);
	sys_write32(val, SCC_BASE);

	ret = pac55xx_uart_configure(dev, &config->init_config);
	if (ret < 0) {
		return ret;
	}

	sys_write8(UARTXFCR_FIFOEN | UARTXFCR_RXFIFORST | UARTXFCR_TXFIFORST,
		   config->reg + UARTXFCR);

	return 0;
}

static const struct uart_driver_api pac55xx_uart_driver_api = {
	.poll_in = pac55xx_uart_poll_in,
	.poll_out = pac55xx_uart_poll_out,
	.configure = pac55xx_uart_configure,
	.err_check = pac55xx_uart_err_check,
};

#define PAC55XX_USART_DEFINE(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static const struct pac55xx_uart_config pac55xx_uart_config_##n = {                        \
		.reg = DT_INST_REG_ADDR(n),                                                        \
		.mode_flag = DT_INST_PROP(n, qorvo_mode_flag),                                     \
		.init_config = UART_CONFIG_DT_INST_INIT(n),                                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &pac55xx_uart_init, NULL, NULL, &pac55xx_uart_config_##n,         \
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,                           \
			      &pac55xx_uart_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PAC55XX_USART_DEFINE)
