/*
 * Copyright (c) 2025 Core Devices LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/cache.h>

#include <bf0_hal.h>

uint32_t SystemCoreClock = 48000000UL;

void soc_early_init_hook(void)
{
	sys_cache_instr_enable();
	sys_cache_data_enable();

	HAL_RCC_CalibrateRC48();

	HAL_PMU_Init();
}
