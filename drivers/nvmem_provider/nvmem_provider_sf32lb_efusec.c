/*
 * Copyright (c) 2025 Core Devices LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_efusec

#include <zephyr/device.h>
#include <zephyr/drivers/nvmem_provider.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <register.h>

LOG_MODULE_REGISTER(nvmem_sf32lb_efusec, CONFIG_NVMEM_PROVIDER_LOG_LEVEL);

/* EFUSE timing parameters (from HAL) */
#define EFUSE_PGM_THPCK_NS (20)
#define EFUSE_PGM_TCKHP_US (10)
#define EFUSE_RD_TIM_NS    (500)
#define EFUSE_PCLK_LIMIT   (120000000)

/* EFUSE bank configuration */
#define EFUSE_BANK_SIZE  32
#define EFUSE_BANK_NUM   4
#define EFUSE_TOTAL_SIZE (EFUSE_BANK_SIZE * EFUSE_BANK_NUM)

/* Timeout calculations (in iterations) */
#define EFUSE_WRITE_TIMEOUT_FACTOR (480000) /* 10ms per bit at 48MHz */
#define EFUSE_READ_TIMEOUT_FACTOR  (48000)  /* 1ms per bit at 48MHz */

struct nvmem_sf32lb_efusec_config {
	EFUSEC_TypeDef *regs;
	const struct device *clk_dev;
};

struct nvmem_sf32lb_efusec_data {
	struct k_mutex lock;
};

static int nvmem_sf32lb_efusec_init_timing(const struct device *dev)
{
	const struct nvmem_sf32lb_efusec_config *config = dev->config;
	EFUSEC_TypeDef *regs = config->regs;
	uint32_t pclk;
	uint32_t pgm_tckhp, pgm_thpck, rd_thrck;
	uint32_t pgm_tckhp_ns;

	pclk = sf32lb_hcpu_get_pclk1_freq();
	if (pclk > EFUSE_PCLK_LIMIT) {
		LOG_ERR("PCLK frequency %u Hz exceeds limit %u Hz", pclk, EFUSE_PCLK_LIMIT);
		return -EINVAL;
	}

	/* Calculate pgm_thpck */
	pgm_thpck = (uint64_t)EFUSE_PGM_THPCK_NS * pclk / (1000 * 1000000) + 1;
	pgm_thpck = FIELD_PREP(EFUSEC_TIMR_THPCK_Msk, pgm_thpck);
	if (pgm_thpck > EFUSEC_TIMR_THPCK) {
		LOG_ERR("pgm_thpck calculation overflow");
		return -EINVAL;
	}

	/* Calculate pgm_tckhp */
	pgm_tckhp = ((uint64_t)EFUSE_PGM_TCKHP_US * pclk + 500000) / 1000000;
	pgm_tckhp_ns = (uint64_t)pgm_tckhp * 1000000 * 1000 / pclk;
	if (pgm_tckhp_ns > 11000) {
		pgm_tckhp -= 1;
	} else if (pgm_tckhp_ns < 9000) {
		pgm_tckhp += 1;
	}
	pgm_tckhp = FIELD_PREP(EFUSEC_TIMR_TCKHP_Msk, pgm_tckhp);
	if (pgm_tckhp > EFUSEC_TIMR_TCKHP) {
		LOG_ERR("pgm_tckhp calculation overflow");
		return -EINVAL;
	}

	/* Calculate rd_thrck */
	rd_thrck = (uint64_t)EFUSE_RD_TIM_NS * pclk / (1000 * 1000000) + 1;
	rd_thrck = FIELD_PREP(EFUSEC_TIMR_THRCK_Msk, rd_thrck);
	if (rd_thrck > EFUSEC_TIMR_THRCK) {
		LOG_ERR("rd_thrck calculation overflow");
		return -EINVAL;
	}

	regs->TIMR = pgm_thpck | pgm_tckhp | rd_thrck;

	LOG_DBG("EFUSE timing initialized: TIMR=0x%08x", regs->TIMR);

	return 0;
}

static int nvmem_sf32lb_efusec_read(const struct device *dev, unsigned int offset, void *data,
				    size_t size)
{
	const struct nvmem_sf32lb_efusec_config *config = dev->config;
	struct nvmem_sf32lb_efusec_data *drv_data = dev->data;
	EFUSEC_TypeDef *regs = config->regs;
	uint8_t *buf = (uint8_t *)data;
	uint16_t bit_offset = offset * 8;
	int byte_off = (bit_offset >> 3) % EFUSE_BANK_SIZE;
	int bank = (bit_offset >> 8);
	volatile uint32_t *rd_reg;
	uint32_t timeout;
	uint32_t ready = 0;
	uint32_t val;
	uint32_t word_size;

	/* Validate parameters */
	if ((size > EFUSE_BANK_SIZE) || ((byte_off + size) > EFUSE_BANK_SIZE) ||
	    (size & 3) ||        /* Must be multiple of 4 */
	    (bit_offset & 31)) { /* Must be 32-bit aligned */
		LOG_ERR("Invalid read parameters: offset=%u, size=%zu", offset, size);
		return -EINVAL;
	}

	if (offset + size > EFUSE_TOTAL_SIZE) {
		LOG_ERR("Read exceeds device bounds");
		return -EINVAL;
	}

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	/* Adjust LDO voltage for SF32LB52X */
	uint32_t org = hwp_pmuc->HPSYS_VOUT;
	uint32_t vout = org + 3;

	if (vout > 0xf) {
		vout = 0xf;
	}
	if (vout < 0xe) {
		vout = 0xe;
	}
	hwp_pmuc->HPSYS_VOUT = vout;
	k_usleep(20);

	/* Select bank and enable READ mode */
	regs->CR = FIELD_PREP(EFUSEC_CR_BANKSEL_Msk, bank);

	/* Start read */
	regs->CR |= EFUSEC_CR_EN;

	/* Wait for completion */
	timeout = (uint32_t)size * 8 * EFUSE_READ_TIMEOUT_FACTOR;
	while (((regs->SR & EFUSEC_SR_DONE) == 0) && (ready < timeout)) {
		ready++;
	}

	regs->SR |= EFUSEC_SR_DONE;

	if (ready >= timeout) {
		LOG_ERR("Read timeout");
		hwp_pmuc->HPSYS_VOUT = org;
		k_mutex_unlock(&drv_data->lock);
		return -ETIMEDOUT;
	}

	/* Read data from bank registers */
	rd_reg = &regs->BANK0_DATA0;
	rd_reg += (bank << 3); /* Each bank has 8 registers */
	rd_reg += (byte_off >> 2);
	word_size = size >> 2;

	for (uint32_t i = 0; i < word_size; i++) {
		val = rd_reg[i];
		buf[0] = val & 0xFF;
		buf[1] = (val >> 8) & 0xFF;
		buf[2] = (val >> 16) & 0xFF;
		buf[3] = (val >> 24) & 0xFF;
		buf += 4;
	}

	/* Restore LDO voltage */
	hwp_pmuc->HPSYS_VOUT = org;

	k_mutex_unlock(&drv_data->lock);

	return 0;
}

static int nvmem_sf32lb_efusec_write(const struct device *dev, unsigned int offset,
				     const void *data, size_t size)
{
	const struct nvmem_sf32lb_efusec_config *config = dev->config;
	struct nvmem_sf32lb_efusec_data *drv_data = dev->data;
	EFUSEC_TypeDef *regs = config->regs;
	const uint8_t *buf = (const uint8_t *)data;
	uint16_t bit_offset = offset * 8;
	int byte_off = (bit_offset >> 3) % EFUSE_BANK_SIZE;
	int bank = (bit_offset >> 8);
	volatile uint32_t *pg_reg;
	uint32_t timeout;
	uint32_t ready = 0;
	uint32_t word_size;

	/* Validate parameters */
	if ((size > EFUSE_BANK_SIZE) || ((byte_off + size) > EFUSE_BANK_SIZE) ||
	    (size & 3) ||        /* Must be multiple of 4 */
	    (bit_offset & 31)) { /* Must be 32-bit aligned */
		LOG_ERR("Invalid write parameters: offset=%u, size=%zu", offset, size);
		return -EINVAL;
	}

	if (offset + size > EFUSE_TOTAL_SIZE) {
		LOG_ERR("Write exceeds device bounds");
		return -EINVAL;
	}

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	/* Clear PGM_DATA to avoid programming unexpected bits */
	pg_reg = &regs->PGM_DATA0;
	for (uint32_t i = 0; i < 8; i++) {
		pg_reg[i] = 0;
	}

	/* Adjust LDO voltage for SF32LB52X */
	uint32_t org = hwp_pmuc->HPSYS_VOUT;
	uint32_t vout = org + 3;

	if (vout > 0xf) {
		vout = 0xf;
	}
	hwp_pmuc->HPSYS_VOUT = vout;

	/* Enable EFUSE VDD */
	uint32_t anau_org = hwp_hpsys_cfg->ANAU_CR;

	hwp_hpsys_cfg->ANAU_CR =
		(anau_org & ~HPSYS_CFG_ANAU_CR_EFUSE_VDD_PD) | HPSYS_CFG_ANAU_CR_EFUSE_VDD_EN;

	/* Enable LDO */
	regs->ANACR |= EFUSEC_ANACR_LDO_EN;
	k_usleep(50);

	/* Select bank and enable PGM mode */
	regs->CR = FIELD_PREP(EFUSEC_CR_BANKSEL_Msk, bank) | EFUSEC_CR_MODE;

	/* Write data to PGM registers */
	pg_reg += (byte_off >> 2);
	word_size = size >> 2;

	for (uint32_t i = 0; i < word_size; i++) {
		pg_reg[i] = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
		buf += 4;
	}

	/* Start program */
	regs->CR |= EFUSEC_CR_EN;

	/* Wait for completion */
	timeout = (uint32_t)size * 8 * EFUSE_WRITE_TIMEOUT_FACTOR;
	while (((regs->SR & EFUSEC_SR_DONE) == 0) && (ready < timeout)) {
		ready++;
	}

	regs->SR |= EFUSEC_SR_DONE;

	/* Disable LDO */
	regs->ANACR &= ~EFUSEC_ANACR_LDO_EN;

	/* Restore LDO voltage */
	hwp_pmuc->HPSYS_VOUT = org;

	/* Restore EFUSE VDD */
	hwp_hpsys_cfg->ANAU_CR = anau_org;

	k_mutex_unlock(&drv_data->lock);

	if (ready >= timeout) {
		LOG_ERR("Write timeout");
		return -ETIMEDOUT;
	}

	return 0;
}

static int nvmem_sf32lb_efusec_init(const struct device *dev)
{
	struct nvmem_sf32lb_efusec_data *data = dev->data;
	int ret;

	k_mutex_init(&data->lock);

	ret = nvmem_sf32lb_efusec_init_timing(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize EFUSE timing");
		return ret;
	}

	return 0;
}

static const struct nvmem_provider_driver_api nvmem_sf32lb_efusec_api = {
	.read = nvmem_sf32lb_efusec_read,
	.write = nvmem_sf32lb_efusec_write,
};

#define NVMEM_SF32LB_EFUSEC_DEFINE(inst)                                                           \
	static const struct nvmem_sf32lb_efusec_config nvmem_sf32lb_efusec_config_##inst = {       \
		.regs = (EFUSEC_TypeDef *)DT_INST_REG_ADDR(inst),                                  \
	};                                                                                         \
                                                                                                   \
	static struct nvmem_sf32lb_efusec_data nvmem_sf32lb_efusec_data_##inst;                    \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, nvmem_sf32lb_efusec_init, NULL,                                \
			      &nvmem_sf32lb_efusec_data_##inst,                                    \
			      &nvmem_sf32lb_efusec_config_##inst, POST_KERNEL,                     \
			      CONFIG_NVMEM_PROVIDER_INIT_PRIORITY, &nvmem_sf32lb_efusec_api);

DT_INST_FOREACH_STATUS_OKAY(NVMEM_SF32LB_EFUSEC_DEFINE)
