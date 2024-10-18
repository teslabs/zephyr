/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nrfx_gpiote.h>
#include <nrfx_comp.h>
#include <nrfx_i2s.h>
#include <nrfx_pwm.h>
#include <nrfx_spim.h>
#include <nrfx_spis.h>
#include <nrfx_twim.h>
#include <nrfx_twis.h>
#include <nrfx_uarte.h>

int main(void)
{
	nrfx_gpiote_t gpiote20 = NRFX_GPIOTE_INSTANCE(20);
	nrfx_gpiote_t gpiote30 = NRFX_GPIOTE_INSTANCE(30);

	nrfx_i2s_t i2s20 = NRFX_I2S_INSTANCE(20);

	nrfx_pwm_t pwm20 = NRFX_PWM_INSTANCE(20);
	nrfx_pwm_t pwm21 = NRFX_PWM_INSTANCE(21);
	nrfx_pwm_t pwm22 = NRFX_PWM_INSTANCE(22);

	nrfx_spim_t  spim21  = NRFX_SPIM_INSTANCE(21);
	nrfx_spis_t  spis21  = NRFX_SPIS_INSTANCE(21);
	nrfx_twim_t  twim21  = NRFX_TWIM_INSTANCE(21);
	nrfx_twis_t  twis21  = NRFX_TWIS_INSTANCE(21);
	nrfx_uarte_t uarte21 = NRFX_UARTE_INSTANCE(21);

	nrfx_spim_t  spim22  = NRFX_SPIM_INSTANCE(22);
	nrfx_spis_t  spis22  = NRFX_SPIS_INSTANCE(22);
	nrfx_twim_t  twim22  = NRFX_TWIM_INSTANCE(22);
	nrfx_twis_t  twis22  = NRFX_TWIS_INSTANCE(22);
	nrfx_uarte_t uarte22 = NRFX_UARTE_INSTANCE(22);

	nrfx_spim_t  spim30  = NRFX_SPIM_INSTANCE(30);
	nrfx_spis_t  spis30  = NRFX_SPIS_INSTANCE(30);
	nrfx_twim_t  twim30  = NRFX_TWIM_INSTANCE(30);
	nrfx_twis_t  twis30  = NRFX_TWIS_INSTANCE(30);
	nrfx_uarte_t uarte30 = NRFX_UARTE_INSTANCE(30);

	nrfx_gpiote_init(&gpiote20, 0);
	nrfx_comp_init(NULL, NULL);
	nrfx_i2s_init(&i2s20, NULL, NULL);
	nrfx_pwm_init(&pwm20, NULL, NULL, NULL);
	nrfx_spim_init(&spim21, NULL, NULL, NULL);
	nrfx_spis_init(&spis21, NULL, NULL, NULL);
	nrfx_twim_init(&twim21, NULL, NULL, NULL);
	nrfx_twis_init(&twis21, NULL, NULL);
	nrfx_uarte_init(&uarte21, NULL, NULL);

	(void)gpiote20;
	(void)gpiote30;
	(void)i2s20;
	(void)pwm20;
	(void)pwm21;
	(void)pwm22;
	(void)spim21;
	(void)spis21;
	(void)twim21;
	(void)twis21;
	(void)spim22;
	(void)spis22;
	(void)twim22;
	(void)twis22;
	(void)uarte22;
	(void)spim30;
	(void)spis30;
	(void)twim30;
	(void)twis30;
	(void)uarte30;

	return 0;
}
