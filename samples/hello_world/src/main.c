/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>

#include "hal/nrf_clock.h"
#include "hal/nrf_gpio.h"
#include "nrfx_i2s.h"

static nrfx_i2s_buffers_t s_i2s_bufs;
static const nrfx_i2s_t s_i2s = NRFX_I2S_INSTANCE(0);
static nrfx_i2s_config_t s_i2s_cfg = NRFX_I2S_DEFAULT_CONFIG(
	 NRF_GPIO_PIN_MAP(1, 15), NRF_GPIO_PIN_MAP(1, 12), NRF_GPIO_PIN_MAP(1, 14),
	NRF_GPIO_PIN_MAP(1, 13), NRF_I2S_PIN_NOT_CONNECTED);

#define SINE_WAVE_TOTAL_SAMPLES 32

/* Stereo sine wave data (L, R, L, R, ...) */
int16_t sine_wave[SINE_WAVE_TOTAL_SAMPLES] = {
	0x5555,     0xaaaa,     3134,  3134,  5791,  5791,  7567,  7567,  8191,  8191,  7567,
	7567,  5791,  5791,  3134,  3134,  0,     0,     -3134, -3134, -5791, -5791,
	-7567, -7567, -8191, -8191, -7567, -7567, -5791, -5791, -3134, -3134};

static void prv_data_handler(nrfx_i2s_buffers_t const *p_released, uint32_t status)
{
}

static void prv_playback(void)
{
	nrfx_err_t err;

	// MCLK: 4MHz, sample rate: ~16KHz (4Mhz / 256 = 15625Hz)
	s_i2s_cfg.mck_setup = NRF_I2S_MCK_32MDIV8;
	s_i2s_cfg.ratio = NRF_I2S_RATIO_256X;
	s_i2s_cfg.channels = NRF_I2S_CHANNELS_STEREO;

	err = nrfx_i2s_init(&s_i2s, &s_i2s_cfg, prv_data_handler);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	s_i2s_bufs.p_tx_buffer = (uint32_t *)sine_wave;
	s_i2s_bufs.buffer_size = SINE_WAVE_TOTAL_SAMPLES / 2;
	err = nrfx_i2s_start(&s_i2s, &s_i2s_bufs, 0);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	k_msleep(3000);

	nrfx_i2s_stop(&s_i2s);
	nrfx_i2s_uninit(&s_i2s);
}

void i2s_isr_shim(const void *ctx)
{
	nrfx_i2s_0_irq_handler();
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	IRQ_CONNECT(I2S_IRQn, 0, i2s_isr_shim, NULL, 0);
	irq_enable(I2S_IRQn);

	nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
	while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED)) {
	}
	nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);

	printf("I2S playback example\n");
	prv_playback();
	printf("I2S playback finished\n");

	return 0;
}
