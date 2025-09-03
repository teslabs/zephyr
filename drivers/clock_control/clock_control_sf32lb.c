/*
 * Copyright (c) 2025 Core Devices LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_rcc_clk

#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/dt-bindings/clock/sf32lb-clocks-common.h>

#include <bf0_hal.h>

struct clock_control_sf32lb_config {
	uint32_t base;
	bool clk_hxt48_enabled;
};

static int clock_control_sf32lb_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct clock_control_sf32lb_config *config = dev->config;
	uint16_t id = *(uint16_t *)sys;

	sys_set_bit(config->base + FIELD_GET(SF32LB_CLOCK_OFFSET_MSK, id),
		    FIELD_GET(SF32LB_CLOCK_BIT_MSK, id));

	return 0;
}

static int clock_control_sf32lb_off(const struct device *dev, clock_control_subsys_t sys)
{
	const struct clock_control_sf32lb_config *config = dev->config;
	uint16_t id = *(uint16_t *)sys;

	sys_clear_bit(config->base + FIELD_GET(SF32LB_CLOCK_OFFSET_MSK, id),
		      FIELD_GET(SF32LB_CLOCK_BIT_MSK, id));

	return 0;
}

static enum clock_control_status clock_control_sf32lb_get_status(const struct device *dev,
								 clock_control_subsys_t sys)
{
	const struct clock_control_sf32lb_config *config = dev->config;
	uint16_t id = *(uint16_t *)sys;

	if (sys_test_bit(config->base + FIELD_GET(SF32LB_CLOCK_OFFSET_MSK, id),
			 FIELD_GET(SF32LB_CLOCK_BIT_MSK, id)) != 0) {
		return CLOCK_CONTROL_STATUS_ON;
	}

	return CLOCK_CONTROL_STATUS_OFF;
}

static DEVICE_API(clock_control, clock_control_sf32lb_api) = {
	.on = clock_control_sf32lb_on,
	.off = clock_control_sf32lb_off,
	.get_status = clock_control_sf32lb_get_status,
};

static int clock_control_sf32lb_init(const struct device *dev)
{
	const struct clock_control_sf32lb_config *config = dev->config;

	if (config->clk_hxt48_enabled) {
		HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
		HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HXT48);

		/* FIXME */
		HAL_RCC_HCPU_EnableDLL1(240000000);
		HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);
		HAL_RCC_HCPU_SetDiv(1, 1, 6);
	}

	return 0;
}

static const struct clock_control_sf32lb_config config = {
	.base = DT_REG_ADDR(DT_INST_PARENT(0)),
	.clk_hxt48_enabled = DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, clk_hxt48), okay),
};

DEVICE_DT_INST_DEFINE(0, clock_control_sf32lb_init, NULL, NULL, &config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_sf32lb_api);
