/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CO_CAN_ZEPHYR_H_
#define CO_CAN_ZEPHYR_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
* INCLUDES
******************************************************************************/

#include <co_if.h>

/** Instance of the Zephyr CAN driver. */
extern const CO_IF_CAN_DRV ZephyrCanDriver;

#ifdef __cplusplus
}
#endif

#endif /* CO_CAN_ZEPHYR_H_ */
