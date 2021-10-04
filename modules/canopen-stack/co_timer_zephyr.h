/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CO_TIMER_ZEPHYR_H_
#define CO_TIMER_ZEPHYR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <co_if.h>

/** Instance of the Zephyr timer driver. */
extern const CO_IF_TIMER_DRV ZephyrTimerDriver;

#ifdef __cplusplus
}
#endif

#endif /* CO_TIMER_ZEPHYR_H_ */
