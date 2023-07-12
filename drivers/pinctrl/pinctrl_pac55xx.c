/*
 * Copyright (c) 2023 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/pac55xx-pinctrl.h>

/* GPIO and DPM block */
#define SCC_BASE DT_REG_ADDR(DT_PARENT(DT_NODELABEL(pinctrl)))

#define PXMUXSEL(port) (SCC_BASE + 0x0CU + 4U * (port))
#define PXPUEN(port)   (SCC_BASE + 0x28U + 4U * (port))
#define PXPDEN(port)   (SCC_BASE + 0x44U + 4U * (port))
#define PXDS(port)     (SCC_BASE + 0x60U + 4U * (port))

#define PXMUXSEL_PX_POS(pin) (4U * (pin))
#define PXMUXSEL_PX_MSK      0x7U
#define PXPUEN_PX_POS(pin)   (pin)
#define PXPUEN_PX_MSK        0x1U
#define PXPDEN_PX_POS(pin)   (pin)
#define PXPDEN_PX_MSK        0x1U
#define PXDS_PXDS_POS(pin)   (4U * (pin))
#define PXDS_PXDS_MSK        0x7U
#define PXDS_PXST_POS(pin)   (3U + 4U * (pin))
#define PXDS_PXST_MSK        0x1U

/* GPIO block */
#define GPIO_BASE DT_REG_ADDR(DT_NODELABEL(gpioa))
#define GPIO_SIZE DT_REG_SIZE(DT_NODELABEL(gpioa))

#define GPIOXMODE(port) (GPIO_BASE + GPIO_SIZE * (port))

#define GPIOXMODE_PX_POS(pin) (2U * (pin))
#define GPIOXMODE_PX_MSK      0x3U

/* pin configuration helpers */
#define PINCFG_GET_PORT(cfg) (((cfg) >> PAC55XX_PORT_POS) & PAC55XX_PORT_MSK)
#define PINCFG_GET_PIN(cfg)  (((cfg) >> PAC55XX_PIN_POS) & PAC55XX_PIN_MSK)
#define PINCFG_GET_MUX(cfg)  (((cfg) >> PAC55XX_MUX_POS) & PAC55XX_MUX_MSK)
#define PINCFG_GET_PU(cfg)   (((cfg) >> PAC55XX_PU_POS) & PAC55XX_PU_MSK)
#define PINCFG_GET_PD(cfg)   (((cfg) >> PAC55XX_PD_POS) & PAC55XX_PD_MSK)
#define PINCFG_GET_ST(cfg)   (((cfg) >> PAC55XX_ST_POS) & PAC55XX_ST_MSK)
#define PINCFG_GET_DS(cfg)   (((cfg) >> PAC55XX_DS_POS) & PAC55XX_DS_MSK)
#define PINCFG_GET_MODE(cfg) (((cfg) >> PAC55XX_MODE_POS) & PAC55XX_MODE_MSK)

static void pinctrl_configure_pin(pinctrl_soc_pin_t cfg)
{
	uint8_t port, pin;
	uint32_t val;

	port = PINCFG_GET_PORT(cfg);
	pin = PINCFG_GET_PIN(cfg);

	/* multiplexer */
	val = sys_read32(PXMUXSEL(port));
	val &= ~(PXMUXSEL_PX_MSK << PXMUXSEL_PX_POS(pin));
	val |= PINCFG_GET_MUX(cfg) << PXMUXSEL_PX_POS(pin);
	sys_write32(val, PXMUXSEL(port));

	/* pull-up */
	val = sys_read32(PXPUEN(port));
	val &= ~(PXPUEN_PX_MSK << PXPUEN_PX_POS(pin));
	val |= PINCFG_GET_PU(cfg) << PXPUEN_PX_POS(pin);
	sys_write32(val, PXPUEN(port));

	/* pull-down */
	val = sys_read32(PXPDEN(port));
	val &= ~(PXPDEN_PX_MSK << PXPDEN_PX_POS(pin));
	val |= PINCFG_GET_PD(cfg) << PXPDEN_PX_POS(pin);
	sys_write32(val, PXPDEN(port));

	/* drive strength and schmitt-trigger */
	val = sys_read32(PXDS(port));
	val &= ~((PXDS_PXDS_MSK << PXDS_PXDS_POS(pin)) | (PXDS_PXST_MSK << PXDS_PXST_POS(pin)));
	val |= (PINCFG_GET_DS(cfg) << PXDS_PXDS_POS(pin)) |
	       (PINCFG_GET_ST(cfg) << PXDS_PXST_POS(pin));
	sys_write32(val, PXDS(port));

	/* GPIO mode */
	val = sys_read32(GPIOXMODE(port));
	val &= ~(GPIOXMODE_PX_MSK << GPIOXMODE_PX_POS(pin));
	val |= (~PINCFG_GET_MODE(cfg) & GPIOXMODE_PX_MSK) << GPIOXMODE_PX_POS(pin);
	sys_write32(val, GPIOXMODE(port));
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(pins[i]);
	}

	return 0;
}
