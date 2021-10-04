/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "co_timer_zephyr.h"

static void DrvTimerInit(uint32_t freq)
{
	(void)freq;

	/* TODO: initialize timer, clear counter and keep timer stopped */
}

static void DrvTimerStart(void)
{
	/* TODO: start hardware timer */
}

static uint8_t DrvTimerUpdate(void)
{
	/* TODO: return 1 if timer event is elapsed, otherwise 0 */
	return (0u);
}

static uint32_t DrvTimerDelay(void)
{
	/* TODO: return current timer counter value */
	return (0u);
}

static void DrvTimerReload(uint32_t reload)
{
	(void)reload;

	/* TODO: reload timer counter value with given reload value */
}

static void DrvTimerStop(void)
{
	/* TODO: stop timer and clear counter value */
}

const CO_IF_TIMER_DRV ZephyrTimerDriver = {
	DrvTimerInit,
	DrvTimerReload,
	DrvTimerDelay,
	DrvTimerStop,
	DrvTimerStart,
	DrvTimerUpdate
};
