/*
 * Copyright (c) 2023 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ARM_QORVO_PAC_PAC55XX_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_QORVO_PAC_PAC55XX_PINCTRL_SOC_H_

#include <stdint.h>

#include <zephyr/devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

/**
 * @brief Type for Qorvo PAC55XX pin.
 *
 * Bits:
 * - 0-8:   PAC55XX_PINMUX() bit field (port, pin, mux).
 * - 9:     pull-up
 * - 10:    pull-down
 * - 11-13: drive strength
 * - 14:    schmitt trigger
 * - 15-16: mode.
 * - 17-31: reserved.
 *
 * Values used by the mux, drive strength and mode pins match those expected in
 * the hardware registers. Mode values, however, are inverted so that we can use
 * uninitialized (00b) as high-impedance input (11b).
 */
typedef uint32_t pinctrl_soc_pin_t;

/**
 * @name PAC55XX pinctrl_soc_pin_t bit field positions/masks/values.
 * @{
 */

/** Pull-up flag position */
#define PAC55XX_PU_POS 9U
/** Pull-up flag mask */
#define PAC55XX_PU_MSK 0x1U
/** Pull-down flag position */
#define PAC55XX_PD_POS 10U
/** Pull-down flag mask */
#define PAC55XX_PD_MSK 0x1U
/** Drive-strength position */
#define PAC55XX_DS_POS 11U
/** Drive-strength mask */
#define PAC55XX_DS_MSK 0x7U
/** Schmitt-trigger flag position */
#define PAC55XX_ST_POS 14U
/** Schmitt-trigger flag mask */
#define PAC55XX_ST_MSK 0x1U
/** Pin mode position */
#define PAC55XX_MODE_POS 15U
/** Pin mode mask */
#define PAC55XX_MODE_MSK 0x3U
/** Pin mode: push-pull output (inverted value)*/
#define PAC55XX_MODE_PP  0x2U
/** Pin mode: open-drain output (inverted value) */
#define PAC55XX_MODE_OD  0x1U

/** @} */

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param prop Property name.
 * @param idx Property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	(DT_PROP_BY_IDX(node_id, prop, idx) |                                                      \
	 ((PAC55XX_PU_MSK * DT_PROP(node_id, bias_pull_up)) << PAC55XX_PU_POS) |                   \
	 ((PAC55XX_PD_MSK * DT_PROP(node_id, bias_pull_down)) << PAC55XX_PD_POS) |                 \
	 ((PAC55XX_ST_MSK * DT_PROP(node_id, input_schmitt_enable)) << PAC55XX_ST_POS) |           \
	 ((DT_ENUM_IDX(node_id, drive_strength) & PAC55XX_DS_MSK) << PAC55XX_DS_POS) |             \
	 ((PAC55XX_MODE_PP * DT_PROP(node_id, drive_push_pull)) << PAC55XX_MODE_POS) |             \
	 ((PAC55XX_MODE_OD * DT_PROP(node_id, drive_open_drain)) << PAC55XX_MODE_POS)),

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{                                                                                          \
		DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux,    \
				       Z_PINCTRL_STATE_PIN_INIT)                                   \
	}

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_QORVO_PAC_PAC55XX_PINCTRL_SOC_H_ */
