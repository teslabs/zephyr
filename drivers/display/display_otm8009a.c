/*
 * Copyright (c) 2020 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT orisetech_otm8009a

#include "display_otm8009a.h"

#include <drivers/display.h>
#include <drivers/gpio.h>
#include <drivers/mipi_dsi.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(display_otm8009a, CONFIG_DISPLAY_LOG_LEVEL);

struct otm8009a_config {
	const char *mipi_dsi_label;
	uint8_t channel;
	const char *reset_label;
	gpio_pin_t reset_pin;
	gpio_dt_flags_t reset_flags;
	const char *bl_label;
	gpio_pin_t bl_pin;
	gpio_dt_flags_t bl_flags;
	uint8_t data_lanes;
	uint32_t pixfmt;
	uint16_t rotation;
};

struct otm8009a_data {
	const struct device *mipi_dsi;
	const struct device *reset;
	const struct device *bl;
	uint16_t xres;
	uint16_t yres;
	enum display_orientation orientation;
};

static ssize_t otm8009a_mcs_write(const struct device *dsi, uint8_t channel,
				  uint16_t cmd, const void *buf, size_t len)
{
	ssize_t r;
	uint8_t scmd;

	scmd = cmd & 0xFFU;
	r = mipi_dsi_dcs_write(dsi, channel, OTM8009A_MCS_ADRSFT, &scmd, 1U);
	if (r < 0) {
		return r;
	}

	return mipi_dsi_dcs_write(dsi, channel, cmd >> 8U, buf, len);
}

static ssize_t otm8009a_configure(const struct device *dev)
{
	const struct otm8009a_config *config = dev->config;
	struct otm8009a_data *data = dev->data;

	ssize_t r;
	uint8_t buf[16];

	static const uint8_t pgamma[] = { 0x00, 0x09, 0x0F, 0x0E, 0x07, 0x10,
					  0x0B, 0x0A, 0x04, 0x07, 0x0B, 0x08,
					  0x0F, 0x10, 0x0A, 0x01 };
	static const uint8_t ngamma[] = { 0x00, 0x09, 0x0F, 0x0E, 0x07, 0x10,
					  0x0B, 0x0A, 0x04, 0x07, 0x0B, 0x08,
					  0x0F, 0x10, 0x0A, 0x01 };

	/* enter command 2 mode to access manufacturer registers (ref. 5.3) */
	buf[0] = 0x80U;
	buf[1] = 0x09U;
	buf[2] = 0x01U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_CMD2_ENA1, buf, 3U);
	if (r < 0) {
		return r;
	}

	/* enter Orise command 2 mode */
	buf[0] = 0x80U;
	buf[1] = 0x09U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_CMD2_ENA2, buf, 2U);
	if (r < 0) {
		return r;
	}

	/* source driver precharge control */
	/* XXX: source output level during porch and non-display area to GND */
	buf[0] = 0x30U;
	buf[1] = 0x83U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_SD_PCH_CTRL, buf, 2U);
	if (r < 0) {
		return r;
	}

	/* not documented */
	buf[0] = 0x40U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_NO_DOC1, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* power control settings 4 for DC voltage settings */
	/* XXX: enable GVDD test mode */
	buf[0] = 0x04U;
	buf[1] = 0xA9U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PWR_CTRL4, buf, 2U);
	if (r < 0) {
		return r;
	}

	/* power control settings 2 for normal mode */
	/* XXX: set pump 4 vgh voltage from 15.0v down to 13.0v */
	/* XXX: set pump 5 vgh voltage from -12.0v downto -9.0v */
	/* XXX: set pump 4&5 x6 (ONLY VALID when PUMP4_EN_ASDM_HV = "0") */
	/* XXX: change pump4 clock ratio from 1 line to 1/2 line */
	buf[0] = 0x96U;
	buf[1] = 0x34U; /**/
	buf[2] = 0x01U; /**/
	buf[3] = 0x33U;
	buf[4] = 0x33U; /**/
	buf[5] = 0x34U; /**/
	buf[6] = 0x33U; /**/
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PWR_CTRL2, buf, 7U);
	if (r < 0) {
		return r;
	}

	/* panel driving mode */
	/* XXX: column inversion */
	buf[0] = 0x50U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_P_DRV_M, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* VCOM voltage setting */
	/* XXX: VCOM Voltage settings from -1.0000v downto -1.2625v */
	buf[0] = 0x4EU;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_VCOMDC, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* oscillator adjustment for idle/normal mode */
	/** XXX: 65Hz */
	buf[0] = 0x66U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_OSC_ADJ, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* RGB video mode setting */
	buf[0] = 0x08U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_RGB_VID_SET, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* GVDD/NGVDD */
	buf[0] = 0x79U;
	buf[1] = 0x79U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_GVDDSET, buf, 2U);
	if (r < 0) {
		return r;
	}

	/* source driver timing setting */
	buf[0] = 0x0DU;
	buf[1] = 0x1BU; /**/
	buf[2] = 0x02U;
	buf[3] = 0x01U;
	buf[4] = 0x3CU;
	buf[5] = 0x08U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_SD_CTRL, buf, 6U);
	if (r < 0) {
		return r;
	}

	/* panel type setting */
	buf[0] = 0x00U;
	buf[1] = 0x01U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PANSET, buf, 2U);
	if (r < 0) {
		return r;
	}

	/* GOA VST setting */
	buf[0] = 0x85U;
	buf[1] = 0x01U;
	buf[2] = 0x00U;
	buf[3] = 0x84U;
	buf[4] = 0x01U;
	buf[5] = 0x00U;
	buf[6] = 0x81U;
	buf[7] = 0x01U;
	buf[8] = 0x28U;
	buf[9] = 0x82U;
	buf[10] = 0x01U;
	buf[11] = 0x28U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_GOAVST, buf, 12U);
	if (r < 0) {
		return r;
	}

	/* GOA CLKA1 setting */
	buf[0] = 0x18U;
	buf[1] = 0x04U;
	buf[2] = 0x03U;
	buf[3] = 0x39U;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_GOACLKA1, buf, 7U);
	if (r < 0) {
		return r;
	}

	/* GOA CLKA2 setting */
	buf[0] = 0x18U;
	buf[1] = 0x03U;
	buf[2] = 0x03U;
	buf[3] = 0x3AU;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_GOACLKA2, buf, 7U);
	if (r < 0) {
		return r;
	}

	/* GOA CLKA3 setting */
	buf[0] = 0x18U;
	buf[1] = 0x02U;
	buf[2] = 0x03U;
	buf[3] = 0x3BU;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_GOACLKA3, buf, 7U);
	if (r < 0) {
		return r;
	}

	/* GOA CLKA4 setting */
	buf[0] = 0x18U;
	buf[1] = 0x01U;
	buf[2] = 0x03U;
	buf[3] = 0x3CU;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_GOACLKA4, buf, 7U);
	if (r < 0) {
		return r;
	}

	/* GOA ECLK */
	buf[0] = 0x01U;
	buf[1] = 0x01U;
	buf[2] = 0x20U;
	buf[3] = 0x20U;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_GOAECLK, buf, 6U);
	if (r < 0) {
		return r;
	}

	/** GOA Other Options 1 */
	buf[0] = 0x01U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_GOAPT1, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* GOA Signal Toggle Option Setting */
	buf[0] = 0x02U;
	buf[1] = 0x00U;
	buf[2] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_GOATGOPT, buf, 3U);
	if (r < 0) {
		return r;
	}

	/* not documented */
	buf[0] = 0x00;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_NO_DOC2, buf, 3U);
	if (r < 0) {
		return r;
	}

	/* Panel Control Setting 1 */
	buf[0] = 0x00U;
	buf[1] = 0x00U;
	buf[2] = 0x00U;
	buf[3] = 0x00U;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x00U;
	buf[7] = 0x00U;
	buf[8] = 0x00U;
	buf[9] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PANCTRLSET1, buf, 10U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x00U;
	buf[1] = 0x00U;
	buf[2] = 0x00U;
	buf[3] = 0x00U;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x00U;
	buf[7] = 0x00U;
	buf[8] = 0x00U;
	buf[9] = 0x00U;
	buf[10] = 0x00U;
	buf[11] = 0x00U;
	buf[12] = 0x00U;
	buf[13] = 0x00U;
	buf[14] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PANCTRLSET2, buf, 15U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x00U;
	buf[1] = 0x00U;
	buf[2] = 0x00U;
	buf[3] = 0x00U;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x00U;
	buf[7] = 0x00U;
	buf[8] = 0x00U;
	buf[9] = 0x00U;
	buf[10] = 0x00U;
	buf[11] = 0x00U;
	buf[12] = 0x00U;
	buf[13] = 0x00U;
	buf[14] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PANCTRLSET3, buf, 15U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x00U;
	buf[1] = 0x00U;
	buf[2] = 0x00U;
	buf[3] = 0x00U;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x00U;
	buf[7] = 0x00U;
	buf[8] = 0x00U;
	buf[9] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PANCTRLSET4, buf, 10U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x00U;
	buf[1] = 0x04U;
	buf[2] = 0x04U;
	buf[3] = 0x04U;
	buf[4] = 0x04U;
	buf[5] = 0x04U;
	buf[6] = 0x00U;
	buf[7] = 0x00U;
	buf[8] = 0x00U;
	buf[9] = 0x00U;
	buf[10] = 0x00U;
	buf[11] = 0x00U;
	buf[12] = 0x00U;
	buf[13] = 0x00U;
	buf[14] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PANCTRLSET5, buf, 15U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x00U;
	buf[1] = 0x00U;
	buf[2] = 0x00U;
	buf[3] = 0x00U;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x04U;
	buf[7] = 0x04U;
	buf[8] = 0x04U;
	buf[9] = 0x04U;
	buf[10] = 0x04U;
	buf[11] = 0x00U;
	buf[12] = 0x00U;
	buf[13] = 0x00U;
	buf[14] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PANCTRLSET6, buf, 15U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x00U;
	buf[1] = 0x00U;
	buf[2] = 0x00U;
	buf[3] = 0x00U;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x00U;
	buf[7] = 0x00U;
	buf[8] = 0x00U;
	buf[9] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PANCTRLSET7, buf, 10U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0xFFU;
	buf[1] = 0xFFU;
	buf[2] = 0xFFU;
	buf[3] = 0xFFU;
	buf[4] = 0xFFU;
	buf[5] = 0xFFU;
	buf[6] = 0xFFU;
	buf[7] = 0xFFU;
	buf[8] = 0xFFU;
	buf[9] = 0xFFU;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PANCTRLSET8, buf, 10U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x00U;
	buf[1] = 0x26U;
	buf[2] = 0x09U;
	buf[3] = 0x0BU;
	buf[4] = 0x01U;
	buf[5] = 0x25U;
	buf[6] = 0x00U;
	buf[7] = 0x00U;
	buf[8] = 0x00U;
	buf[9] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PANU2D1, buf, 10U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x00U;
	buf[1] = 0x00U;
	buf[2] = 0x00U;
	buf[3] = 0x00U;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x00U;
	buf[7] = 0x00U;
	buf[8] = 0x00U;
	buf[9] = 0x00U;
	buf[10] = 0x00U;
	buf[11] = 0x26U;
	buf[12] = 0x0AU;
	buf[13] = 0x0CU;
	buf[14] = 0x02U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PANU2D2, buf, 15U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x25U;
	buf[1] = 0x00U;
	buf[2] = 0x00U;
	buf[3] = 0x00U;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x00U;
	buf[7] = 0x00U;
	buf[8] = 0x00U;
	buf[9] = 0x00U;
	buf[10] = 0x00U;
	buf[11] = 0x00U;
	buf[12] = 0x00U;
	buf[13] = 0x00U;
	buf[14] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PANU2D3, buf, 15U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x00U;
	buf[1] = 0x25U;
	buf[2] = 0x0CU;
	buf[3] = 0x0AU;
	buf[4] = 0x02U;
	buf[5] = 0x26U;
	buf[6] = 0x00U;
	buf[7] = 0x00U;
	buf[8] = 0x00U;
	buf[9] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PAND2U1, buf, 10U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x00U;
	buf[1] = 0x00U;
	buf[2] = 0x00U;
	buf[3] = 0x00U;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x00U;
	buf[7] = 0x00U;
	buf[8] = 0x00U;
	buf[9] = 0x00U;
	buf[10] = 0x00U;
	buf[11] = 0x25U;
	buf[12] = 0x0BU;
	buf[13] = 0x09U;
	buf[14] = 0x01U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PAND2U2, buf, 15U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x26U;
	buf[1] = 0x00U;
	buf[2] = 0x00U;
	buf[3] = 0x00U;
	buf[4] = 0x00U;
	buf[5] = 0x00U;
	buf[6] = 0x00U;
	buf[7] = 0x00U;
	buf[8] = 0x00U;
	buf[9] = 0x00U;
	buf[10] = 0x00U;
	buf[11] = 0x00U;
	buf[12] = 0x00U;
	buf[13] = 0x00U;
	buf[14] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PAND2U3, buf, 15U);
	if (r < 0) {
		return r;
	}

	/* power control setting 1 */
	/* XXX: Pump 1 min and max DM */
	buf[0] = 0x08U;
	buf[1] = 0x66U;
	buf[2] = 0x83U; /**/
	buf[3] = 0x00U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PWR_CTRL1, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* not documented */
	buf[0] = 0x06U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_NO_DOC3, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* PWM parameter 3 */
	/* XXX: Freq: 19.5 KHz */
	buf[0] = 0x06U;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_PWM_PARA3, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* gamma correction 2.2+ */
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_GMCT2_2P, pgamma, sizeof(pgamma));
	if (r < 0) {
		return r;
	}

	/* gamma correction 2.2- */
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_GMCT2_2N, ngamma, sizeof(ngamma));
	if (r < 0) {
		return r;
	}

	/* exit command 2 mode */
	buf[0] = 0xFFU;
	buf[1] = 0xFFU;
	buf[2] = 0xFFU;
	r = otm8009a_mcs_write(data->mipi_dsi, config->channel,
			       OTM8009A_MCS_CMD2_ENA1, buf, 3U);
	if (r < 0) {
		return r;
	}

	/* exit sleep mode */
	r = mipi_dsi_dcs_write(data->mipi_dsi, config->channel,
			       MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0U);
	if (r < 0) {
		return r;
	}

	k_sleep(K_MSEC(OTM8009A_EXIT_SLEEP_MODE_WAIT_TIME));

	/* set pixel color format */
	buf[0] = MIPI_DCS_PIXEL_FORMAT_24BIT;
	r = mipi_dsi_dcs_write(data->mipi_dsi, config->channel,
			       MIPI_DCS_SET_PIXEL_FORMAT, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* configure address mode */
	if (data->orientation == DISPLAY_ORIENTATION_NORMAL) {
		buf[0] = 0x00U;
	} else if (data->orientation == DISPLAY_ORIENTATION_ROTATED_90) {
		buf[0] = MIPI_DCS_ADDRESS_MODE_MIRROR_X | MIPI_DCS_ADDRESS_MODE_SWAP_XY;
	} else if (data->orientation == DISPLAY_ORIENTATION_ROTATED_180) {
		buf[0] = MIPI_DCS_ADDRESS_MODE_MIRROR_X | MIPI_DCS_ADDRESS_MODE_MIRROR_Y;
	} else if (data->orientation == DISPLAY_ORIENTATION_ROTATED_270) {
		buf[0] = MIPI_DCS_ADDRESS_MODE_MIRROR_Y | MIPI_DCS_ADDRESS_MODE_SWAP_XY;
	}

	r = mipi_dsi_dcs_write(data->mipi_dsi, config->channel,
			       MIPI_DCS_SET_ADDRESS_MODE, buf, 1U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x00U;
	buf[1] = 0x00U;
	buf[2] = (data->xres - 1U) >> 8U;
	buf[3] = (data->xres - 1U) & 0xFFU;
	r = mipi_dsi_dcs_write(data->mipi_dsi, config->channel,
			       MIPI_DCS_SET_COLUMN_ADDRESS, buf, 4U);
	if (r < 0) {
		return r;
	}

	buf[0] = 0x00U;
	buf[1] = 0x00U;
	buf[2] = (data->yres - 1U) >> 8U;
	buf[3] = (data->yres - 1U) & 0xFFU;
	r = mipi_dsi_dcs_write(data->mipi_dsi, config->channel,
			       MIPI_DCS_SET_PAGE_ADDRESS, buf, 4U);
	if (r < 0) {
		return r;
	}

	/* backlight control */
	buf[0] = OTM8009A_WRCTRLD_BCTRL | OTM8009A_WRCTRLD_DD |
		 OTM8009A_WRCTRLD_BL;
	r = mipi_dsi_dcs_write(data->mipi_dsi, config->channel,
			       MIPI_DCS_WRITE_CONTROL_DISPLAY, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* adaptive brightness control */
	buf[0] = OTM8009A_WRCABC_UI;
	r = mipi_dsi_dcs_write(data->mipi_dsi, config->channel,
			       MIPI_DCS_WRITE_POWER_SAVE, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* adaptive brightness control minimum brightness */
	buf[0] = 0xFFU;
	r = mipi_dsi_dcs_write(data->mipi_dsi, config->channel,
			       MIPI_DCS_SET_CABC_MIN_BRIGHTNESS, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* brightness */
	buf[0] = 0xFFU;
	r = mipi_dsi_dcs_write(data->mipi_dsi, config->channel,
			       MIPI_DCS_SET_DISPLAY_BRIGHTNESS, buf, 1U);
	if (r < 0) {
		return r;
	}

	/* turn on the display */
	r = mipi_dsi_dcs_write(data->mipi_dsi, config->channel,
			       MIPI_DCS_SET_DISPLAY_ON, NULL, 0U);
	if (r < 0) {
		return r;
	}

	/* trigger display write (from data coming by DSI bus) */
	r = mipi_dsi_dcs_write(data->mipi_dsi, config->channel,
			       MIPI_DCS_WRITE_MEMORY_START, NULL, 0U);
	if (r < 0) {
		return r;
	}

	return r;
}

static int otm8009a_write(const struct device *dev, const uint16_t x,
			  const uint16_t y,
			  const struct display_buffer_descriptor *desc,
			  const void *buf)
{
	struct otm8009a_data *data = dev->data;

	if (desc->width > desc->pitch || x + desc->pitch > data->xres ||
	    y + desc->height > data->yres) {
		return -EINVAL;
	}

	return 0;
}

static int otm8009a_read(const struct device *dev, const uint16_t x,
			 const uint16_t y,
			 const struct display_buffer_descriptor *desc,
			 void *buf)
{
	return -ENOTSUP;
}

static void *otm8009a_get_framebuffer(const struct device *dev)
{
	return NULL;
}

static int otm8009a_blanking_off(const struct device *dev)
{
	const struct otm8009a_config *config = dev->config;
	struct otm8009a_data *data = dev->data;

	return gpio_pin_set(data->bl, config->bl_pin, 1);
}

static int otm8009a_blanking_on(const struct device *dev)
{
	const struct otm8009a_config *config = dev->config;
	struct otm8009a_data *data = dev->data;

	return gpio_pin_set(data->bl, config->bl_pin, 0);
}

static int otm8009a_set_brightness(const struct device *dev,
				   const uint8_t brightness)
{
	const struct otm8009a_config *config = dev->config;
	struct otm8009a_data *data = dev->data;

	return mipi_dsi_dcs_write(data->mipi_dsi, config->channel,
				  MIPI_DCS_SET_DISPLAY_BRIGHTNESS, &brightness,
				  1U);
}

static int otm8009a_set_contrast(const struct device *dev,
				 const uint8_t contrast)
{
	return -ENOTSUP;
}

static void otm8009a_get_capabilities(const struct device *dev,
				      struct display_capabilities *capabilities)
{
	const struct otm8009a_config *config = dev->config;
	struct otm8009a_data *data = dev->data;

	memset(capabilities, 0, sizeof(*capabilities));

	capabilities->x_resolution = data->xres;
	capabilities->y_resolution = data->yres;
	capabilities->current_orientation = data->orientation;
	capabilities->supported_pixel_formats = config->pixfmt;
	capabilities->current_pixel_format = config->pixfmt;
}

static int
otm8009a_set_pixel_format(const struct device *dev,
			  const enum display_pixel_format pixel_format)
{
	LOG_ERR("Runtime pixel format change not supported");
	return -ENOTSUP;
}

static int
otm8009a_set_orientation(const struct device *dev,
			 const enum display_orientation orientation)
{
	LOG_ERR("Runtime orientation change not supported");
	return -ENOTSUP;
}

static const struct display_driver_api otm8009a_api = {
	.blanking_on = otm8009a_blanking_on,
	.blanking_off = otm8009a_blanking_off,
	.write = otm8009a_write,
	.read = otm8009a_read,
	.get_framebuffer = otm8009a_get_framebuffer,
	.set_brightness = otm8009a_set_brightness,
	.set_contrast = otm8009a_set_contrast,
	.get_capabilities = otm8009a_get_capabilities,
	.set_pixel_format = otm8009a_set_pixel_format,
	.set_orientation = otm8009a_set_orientation,
};

static int otm8009a_init(const struct device *dev)
{
	const struct otm8009a_config *config = dev->config;
	struct otm8009a_data *data = dev->data;

	int r;
	struct mipi_dsi_device mdev;

	/* MIPI-DSI host */
	data->mipi_dsi = device_get_binding(config->mipi_dsi_label);
	if (data->mipi_dsi == NULL) {
		LOG_ERR("Could not obtain MIPI-DSI host device");
		return -ENODEV;
	}

	/* reset */
	data->reset = device_get_binding(config->reset_label);
	if (data->reset != NULL) {
		r = gpio_pin_configure(data->reset, config->reset_pin,
				       GPIO_OUTPUT_INACTIVE |
					       config->reset_flags);
		if (r < 0) {
			LOG_ERR("Could not configure reset GPIO (%d)", r);
			return r;
		}

		/* reset display */
		gpio_pin_set(data->reset, config->reset_pin, 1);
		k_sleep(K_MSEC(OTM8009A_RESET_TIME));
		gpio_pin_set(data->reset, config->reset_pin, 0);
		k_sleep(K_MSEC(OTM8009A_WAKE_TIME));
	}

	/* back light */
	data->bl = device_get_binding(config->bl_label);
	if (data->bl != NULL) {
		r = gpio_pin_configure(data->bl, config->bl_pin,
				       GPIO_OUTPUT_ACTIVE | config->bl_flags);
		if (r < 0) {
			LOG_ERR("Could not configure back-light GPIO (%d)", r);
			return r;
		}
	}

	/* store x/y resolution & rotation */
	if (config->rotation == 0U) {
		data->xres = OTM8009A_WIDTH;
		data->yres = OTM8009A_HEIGHT;
		data->orientation = DISPLAY_ORIENTATION_NORMAL;
	} else if (config->rotation == 90U) {
		data->xres = OTM8009A_HEIGHT;
		data->yres = OTM8009A_WIDTH;
		data->orientation = DISPLAY_ORIENTATION_ROTATED_90;
	} else if (config->rotation == 180U) {
		data->xres = OTM8009A_WIDTH;
		data->yres = OTM8009A_HEIGHT;
		data->orientation = DISPLAY_ORIENTATION_ROTATED_180;
	} else if (config->rotation == 270U) {
		data->xres = OTM8009A_HEIGHT;
		data->yres = OTM8009A_WIDTH;
		data->orientation = DISPLAY_ORIENTATION_ROTATED_270;
	}

	/* attach to MIPI-DSI host */
	mdev.data_lanes = config->data_lanes;
	mdev.pixfmt = config->pixfmt;
	mdev.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM;

	mdev.timings.hactive = data->xres;
	mdev.timings.hbp = OTM8009A_HBP;
	mdev.timings.hfp = OTM8009A_HFP;
	mdev.timings.hsync = OTM8009A_HSYNC;
	mdev.timings.vactive = data->yres;
	mdev.timings.vbp = OTM8009A_VBP;
	mdev.timings.vfp = OTM8009A_VFP;
	mdev.timings.vsync = OTM8009A_VSYNC;

	r = mipi_dsi_attach(data->mipi_dsi, config->channel, &mdev);
	if (r < 0) {
		LOG_ERR("Could not attach to MIPI-DSI host");
		return r;
	}

	r = otm8009a_configure(dev);
	if (r < 0) {
		LOG_ERR("Could not configure display");
		return r;
	}

	return 0;
}

#define OTM8009A_INIT(n)                                                       \
	static const struct otm8009a_config otm8009a_config_##n = {            \
		.mipi_dsi_label = DT_INST_BUS_LABEL(n),                        \
		.channel = DT_INST_REG_ADDR(n),                                \
		.reset_label = UTIL_AND(DT_INST_NODE_HAS_PROP(n, reset_gpios), \
					DT_INST_GPIO_LABEL(n, reset_gpios)),   \
		.reset_pin = UTIL_AND(DT_INST_NODE_HAS_PROP(n, reset_gpios),   \
				      DT_INST_GPIO_PIN(n, reset_gpios)),       \
		.reset_flags = UTIL_AND(DT_INST_NODE_HAS_PROP(n, reset_gpios), \
					DT_INST_GPIO_FLAGS(n, reset_gpios)),   \
		.bl_label = UTIL_AND(DT_INST_NODE_HAS_PROP(n, bl_gpios),       \
				     DT_INST_GPIO_LABEL(n, bl_gpios)),         \
		.bl_pin = UTIL_AND(DT_INST_NODE_HAS_PROP(n, bl_gpios),         \
				   DT_INST_GPIO_PIN(n, bl_gpios)),             \
		.bl_flags = UTIL_AND(DT_INST_NODE_HAS_PROP(n, bl_gpios),       \
				     DT_INST_GPIO_FLAGS(n, bl_gpios)),         \
		.data_lanes = DT_INST_PROP(n, data_lanes),                     \
		.pixfmt = DT_INST_PROP(n, pixel_format),                       \
		.rotation = DT_INST_PROP(n, rotation),                         \
	};                                                                     \
                                                                               \
	static struct otm8009a_data otm8009a_data_##n;                         \
                                                                               \
	DEVICE_AND_API_INIT(otm8009a_##n, DT_INST_LABEL(n), otm8009a_init,     \
			    &otm8009a_data_##n, &otm8009a_config_##n,          \
			    POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY,     \
			    &otm8009a_api);

DT_INST_FOREACH_STATUS_OKAY(OTM8009A_INIT)
