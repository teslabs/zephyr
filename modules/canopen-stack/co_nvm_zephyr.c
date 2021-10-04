/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "co_nvm_zephyr.h"

static void DrvNvmInit(void)
{
	/* TODO: initialize the non-volatile memory */
}

static uint32_t DrvNvmRead(uint32_t start, uint8_t *buffer, uint32_t size)
{
	(void)start;
	(void)buffer;
	(void)size;

	/* TODO: read a memory block from non-volatile memory into given buffer */
	return (0u);
}

static uint32_t DrvNvmWrite(uint32_t start, uint8_t *buffer, uint32_t size)
{
	(void)start;
	(void)buffer;
	(void)size;

	/* TODO: write content of given buffer into non-volatile memory */
	return (0u);
}

const CO_IF_NVM_DRV ZephyrNvmDriver = {
	DrvNvmInit,
	DrvNvmRead,
	DrvNvmWrite
};
