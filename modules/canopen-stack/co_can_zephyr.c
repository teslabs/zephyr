/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "co_can_zephyr.h"

static void DrvCanInit(void)
{
	/* TODO: initialize the CAN controller (don't enable communication) */
}

static void DrvCanEnable(uint32_t baudrate)
{
	(void)baudrate;

	/* TODO: set the given baudrate to the CAN controller */
}

static int16_t DrvCanSend(CO_IF_FRM *frm)
{
	(void)frm;

	/* TODO: wait for free CAN message slot and send the given CAN frame */
	return (0u);
}

static int16_t DrvCanRead (CO_IF_FRM *frm)
{
	(void)frm;

	/* TODO: wait for a CAN frame and read CAN frame from the CAN controller */
	return (0u);
}

static void DrvCanReset(void)
{
	/* TODO: reset CAN controller while keeping baudrate */
}

static void DrvCanClose(void)
{
	/* TODO: remove CAN controller from CAN network */
}

const CO_IF_CAN_DRV DummyCanDriver = {
	DrvCanInit,
	DrvCanEnable,
	DrvCanRead,
	DrvCanSend,
	DrvCanReset,
	DrvCanClose
};
