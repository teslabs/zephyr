/*
 * Copyright (c) 2020 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_dsi

#include <device.h>
#include <irq.h>
#include <soc.h>

#include <drivers/clock_control/stm32_clock_control.h>
#include <drivers/mipi_dsi.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(dsi_stm32, CONFIG_MIPI_DSI_LOG_LEVEL);

/** STM32 MIPI-DSI host driver configuration. */
struct dsi_stm32_config {
	DSI_TypeDef *dsi;
	struct stm32_pclken pclken_dsi;
	LTDC_TypeDef *ltdc;
	struct stm32_pclken pclken_ltdc;
	uint32_t ltdc_irqn;
	uint8_t pll_ndiv;
	uint8_t pll_idf;
	uint8_t pll_odf;
	uint8_t txeckdiv;
};

/** STM32 MIPI-DSI host driver data. */
struct dsi_stm32_data {
	DSI_HandleTypeDef dsi;
	LTDC_HandleTypeDef ltdc;
};

__stm32_sdram2_section volatile uint32_t fb[800 * 480];

/** Connect LTDC IRQ */
static void ltdc_stm32_irq_connect(void);

static inline uint32_t ltdc_pixfmt_from_bpp(uint32_t bpp)
{
	switch (bpp) {
	case 32U:
		return LTDC_PIXEL_FORMAT_ARGB8888;
	case 24U:
		return LTDC_PIXEL_FORMAT_RGB888;
	case 16U:
		return LTDC_PIXEL_FORMAT_RGB565;
	case 8U:
		return LTDC_PIXEL_FORMAT_L8;
	default:
		LOG_WRN("Unsupported bpp, defaulting to ARGB8888 format");
		return LTDC_PIXEL_FORMAT_ARGB8888;
	}
}

static inline uint32_t dsi_pixfmt_from_mipi(uint32_t pixfmt)
{
	switch (pixfmt) {
	case MIPI_DSI_PIXFMT_RGB888:
		return DSI_RGB888;
	case MIPI_DSI_PIXFMT_RGB666:
	case MIPI_DSI_PIXFMT_RGB666_PACKED:
		return DSI_RGB666;
	case MIPI_DSI_PIXFMT_RGB565:
		return DSI_RGB565;
	default:
		LOG_WRN("Unsupported color format, defaulting to RGB888");
		return DSI_RGB888;
	}
}

/** LTDC IRQ handler. */
static void ltdc_stm32_isr(const struct device *dev)
{
}

static int dsi_stm32_attach(const struct device *dev,
			    uint8_t channel,
			    const struct mipi_dsi_device *mdev)
{
	const struct dsi_stm32_config *config = dev->config;
	struct dsi_stm32_data *data = dev->data;

	LTDC_InitTypeDef *ltdc_init;
	LTDC_LayerCfgTypeDef ltdc_lcfg;
	DSI_PLLInitTypeDef pll_init;
	DSI_VidCfgTypeDef vid_cfg;

	if (!(mdev->mode_flags & MIPI_DSI_MODE_VIDEO)) {
		return -ENOTSUP;
	}

	/* TODO: remove, just for testing FB output */
	for (size_t i = 0; i < ARRAY_SIZE(fb); i++) {
		fb[i] = 0xFF00FF00UL;
	}

	for (size_t i = 0; i < 100; i++) {
		for (size_t j = 0; j < 50; j++) {
			fb[j * 800 + i] = 0xFFFF0000UL;
		}
	}

	/* initialize LTDC */
	ltdc_init = &data->ltdc.Init;

	ltdc_init->HSPolarity = LTDC_HSPOLARITY_AL;
	ltdc_init->VSPolarity = LTDC_VSPOLARITY_AL;
	ltdc_init->DEPolarity = LTDC_DEPOLARITY_AL;
	ltdc_init->PCPolarity = LTDC_PCPOLARITY_IPC;

	ltdc_init->HorizontalSync = mdev->timings.hsync - 1U;
	ltdc_init->AccumulatedHBP = ltdc_init->HorizontalSync + mdev->timings.hbp;
	ltdc_init->AccumulatedActiveW = ltdc_init->AccumulatedHBP + mdev->timings.hactive;
	ltdc_init->TotalWidth = ltdc_init->AccumulatedActiveW + mdev->timings.hfp;

	ltdc_init->VerticalSync = mdev->timings.vsync - 1U;
	ltdc_init->AccumulatedVBP = ltdc_init->VerticalSync + mdev->timings.vbp;
	ltdc_init->AccumulatedActiveH = ltdc_init->AccumulatedVBP + mdev->timings.vactive;
	ltdc_init->TotalHeigh = ltdc_init->AccumulatedActiveH + mdev->timings.vfp;

	if (HAL_LTDC_Init(&data->ltdc) != HAL_OK) {
		LOG_ERR("Could not initialize LTDC");
		return -EIO;
	}

	/* configure LTDC layer */
	ltdc_lcfg.WindowX0 = 0;
	ltdc_lcfg.WindowX1 = mdev->timings.hactive;
	ltdc_lcfg.WindowY0 = 0;
	ltdc_lcfg.WindowY1 = mdev->timings.vactive;
	ltdc_lcfg.PixelFormat = ltdc_pixfmt_from_bpp(32U);
	ltdc_lcfg.Alpha = 0xFFU;
	ltdc_lcfg.Alpha0 = 0x00U;
	ltdc_lcfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
	ltdc_lcfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
	ltdc_lcfg.FBStartAdress = (uint32_t)fb;
	ltdc_lcfg.ImageWidth = mdev->timings.hactive;
	ltdc_lcfg.ImageHeight = mdev->timings.vactive;
	ltdc_lcfg.Backcolor.Blue = 0U;
	ltdc_lcfg.Backcolor.Green = 0U;
	ltdc_lcfg.Backcolor.Red = 0U;

	if (HAL_LTDC_ConfigLayer(&data->ltdc, &ltdc_lcfg, 0U) != HAL_OK) {
		LOG_ERR("Could not configure LTDC layer");
		return -EIO;
	}

	/* configure DSI-PHY */
	data->dsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
	data->dsi.Init.NumberOfLanes = mdev->data_lanes - 1U;
	data->dsi.Init.TXEscapeCkdiv = config->txeckdiv;

	pll_init.PLLNDIV = config->pll_ndiv;
	pll_init.PLLIDF = config->pll_idf;
	pll_init.PLLODF = config->pll_odf >> 1U;
	if (config->pll_odf >= 4U) {
		pll_init.PLLODF = 3U;
	}

	if (HAL_DSI_Init(&data->dsi, &pll_init) != HAL_OK) {
		LOG_ERR("Could not initialize DSI");
		return -EIO;
	}

	/* configure video mode */
	vid_cfg.VirtualChannelID = channel;

	/* pixel format */
	vid_cfg.ColorCoding = dsi_pixfmt_from_mipi(mdev->pixfmt);
	if (mdev->pixfmt == MIPI_DSI_PIXFMT_RGB666_PACKED) {
		vid_cfg.LooselyPacked = DSI_LOOSELY_PACKED_ENABLE;
	} else {
		vid_cfg.LooselyPacked = DSI_LOOSELY_PACKED_DISABLE;
	}

	/* video mode */
	if (mdev->mode_flags & MIPI_DSI_MODE_VIDEO_BURST) {
		vid_cfg.Mode = DSI_VID_MODE_BURST;
	} else if (mdev->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) {
		vid_cfg.Mode = DSI_VID_MODE_NB_PULSES;
	} else {
		vid_cfg.Mode = DSI_VID_MODE_NB_EVENTS;
	}

	vid_cfg.PacketSize = mdev->timings.hactive;
	vid_cfg.NumberOfChunks = 0;
	vid_cfg.NullPacketSize = 0xFFFU;
	vid_cfg.HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
	vid_cfg.VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
	vid_cfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
	vid_cfg.HorizontalSyncActive = (mdev->timings.hsync * 62500U) / 27429U;
	vid_cfg.HorizontalBackPorch = (mdev->timings.hbp * 62500U) / 27429U;
	vid_cfg.HorizontalLine = ((mdev->timings.hactive + mdev->timings.hsync +
				   mdev->timings.hbp + mdev->timings.hfp) *
				  62500U) /
				 27429U;
	vid_cfg.VerticalSyncActive = mdev->timings.vsync;
	vid_cfg.VerticalBackPorch = mdev->timings.vbp;
	vid_cfg.VerticalFrontPorch = mdev->timings.vfp;
	vid_cfg.VerticalActive = mdev->timings.vactive;

	if (mdev->mode_flags & MIPI_DSI_MODE_LPM) {
		vid_cfg.LPCommandEnable = DSI_LP_COMMAND_ENABLE;
	} else {
		vid_cfg.LPCommandEnable = DSI_LP_COMMAND_DISABLE;
	}
	vid_cfg.LPLargestPacketSize = 4;
	vid_cfg.LPVACTLargestPacketSize = 4;

	vid_cfg.LPHorizontalFrontPorchEnable = DSI_LP_HFP_ENABLE;
	vid_cfg.LPHorizontalBackPorchEnable = DSI_LP_HBP_ENABLE;
	vid_cfg.LPVerticalActiveEnable = DSI_LP_VACT_ENABLE;
	vid_cfg.LPVerticalFrontPorchEnable = DSI_LP_VFP_ENABLE;
	vid_cfg.LPVerticalBackPorchEnable = DSI_LP_VBP_ENABLE;
	vid_cfg.LPVerticalSyncActiveEnable = DSI_LP_VSYNC_ENABLE;

	vid_cfg.FrameBTAAcknowledgeEnable = DSI_FBTAA_ENABLE;

	if (HAL_DSI_ConfigVideoMode(&data->dsi, &vid_cfg) != HAL_OK) {
		LOG_ERR("Could not configure DSI in video mode");
		return -EIO;
	}

	if (HAL_DSI_ConfigFlowControl(&data->dsi, DSI_FLOW_CONTROL_BTA) != HAL_OK) {
		LOG_ERR("Could not configure flow control");
		return -EIO;
	}

	if (HAL_DSI_Start(&data->dsi) != HAL_OK) {
		LOG_ERR("Could not start DSI");
		return -EIO;
	}

	return 0;
}

static ssize_t dsi_stm32_transfer(const struct device *dev, uint8_t channel,
				  struct mipi_dsi_msg *msg)
{
	struct dsi_stm32_data *data = dev->data;

	HAL_StatusTypeDef s;
	uint8_t param_1 = 0U;
	uint8_t param_2 = 0U;

	switch (msg->type) {
	/* read */
	case MIPI_DSI_DCS_READ:
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
		s = HAL_DSI_Read(&data->dsi, channel, msg->rx_buf, msg->rx_len,
				 msg->type, msg->cmd, (uint8_t *)msg->tx_buf);
		break;
	/* short writes (DCS) */
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
		param_1 = msg->cmd;

		if (msg->tx_len >= 1U) {
			param_2 = ((uint8_t *)msg->tx_buf)[0];
		}

		s = HAL_DSI_ShortWrite(&data->dsi, channel, msg->type,
				       param_1, param_2);
		break;
	/* short writes (non-DCS) */
	case MIPI_DSI_V_SYNC_START:
	case MIPI_DSI_V_SYNC_END:
	case MIPI_DSI_H_SYNC_START:
	case MIPI_DSI_H_SYNC_END:
	case MIPI_DSI_END_OF_TRANSMISSION:
	case MIPI_DSI_COLOR_MODE_OFF:
	case MIPI_DSI_COLOR_MODE_ON:
	case MIPI_DSI_SHUTDOWN_PERIPHERAL:
	case MIPI_DSI_TURN_ON_PERIPHERAL:
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
		if (msg->tx_len >= 1U) {
			param_1 = ((uint8_t *)msg->tx_buf)[0];
		}

		if (msg->tx_len >= 2U) {
			param_2 = ((uint8_t *)msg->tx_buf)[1];
		}

		s = HAL_DSI_ShortWrite(&data->dsi, channel, msg->type,
				       param_1, param_2);
		break;
	/* long writes */
	case MIPI_DSI_NULL_PACKET:
	case MIPI_DSI_BLANKING_PACKET:
	case MIPI_DSI_GENERIC_LONG_WRITE:
	case MIPI_DSI_DCS_LONG_WRITE:
	case MIPI_DSI_LOOSELY_PACKED_PIXEL_STREAM_YCBCR20:
	case MIPI_DSI_PACKED_PIXEL_STREAM_YCBCR24:
	case MIPI_DSI_PACKED_PIXEL_STREAM_YCBCR16:
	case MIPI_DSI_PACKED_PIXEL_STREAM_30:
	case MIPI_DSI_PACKED_PIXEL_STREAM_36:
	case MIPI_DSI_PACKED_PIXEL_STREAM_YCBCR12:
	case MIPI_DSI_PACKED_PIXEL_STREAM_16:
	case MIPI_DSI_PACKED_PIXEL_STREAM_18:
	case MIPI_DSI_PIXEL_STREAM_3BYTE_18:
	case MIPI_DSI_PACKED_PIXEL_STREAM_24:
		s = HAL_DSI_LongWrite(&data->dsi, channel, msg->type,
				      msg->tx_len, msg->cmd,
				      (uint8_t *)msg->tx_buf);
		break;
	default:
		LOG_ERR("Unsupported message type (%d)", msg->type);
		return -ENOTSUP;
	}

	if (s != HAL_OK) {
		LOG_ERR("Transmission failed");
		return -EIO;
	}

	return msg->tx_len;
}

static struct mipi_dsi_driver_api dsi_stm32_api = {
	.attach = dsi_stm32_attach,
	.transfer = dsi_stm32_transfer,
};

static int dsi_stm32_init(const struct device *dev)
{
	const struct dsi_stm32_config *config = dev->config;
	struct dsi_stm32_data *data = dev->data;

	int r;
	const struct device *clk;

	/* enable DSI and LTDC peripherals clock */
	clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	r = clock_control_on(clk,
			     (clock_control_subsys_t *)&config->pclken_dsi);
	if (r < 0) {
		LOG_ERR("Could not initialize DSI clock (%d)", r);
		return r;
	}

	r = clock_control_on(clk,
			     (clock_control_subsys_t *)&config->pclken_ltdc);
	if (r < 0) {
		LOG_ERR("Could not initialize LTDC clock (%d)", r);
		return r;
	}

	data->dsi.Instance = config->dsi;
	data->ltdc.Instance = config->ltdc;

	ltdc_stm32_irq_connect();

	return 0;
}

static const struct dsi_stm32_config config = {
	.dsi = (DSI_TypeDef *)DT_INST_REG_ADDR(0),
	.pclken_dsi = { .bus = DT_INST_CLOCKS_CELL(0, bus),
			.enr = DT_INST_CLOCKS_CELL(0, bits) },
	.ltdc = (LTDC_TypeDef *)DT_REG_ADDR(DT_INST_PHANDLE(0, st_ltdc)),
	.pclken_ltdc = { .bus = DT_CLOCKS_CELL(DT_INST_PHANDLE(0, st_ltdc),
					       bus),
			 .enr = DT_CLOCKS_CELL(DT_INST_PHANDLE(0, st_ltdc),
					       bits) },
	.ltdc_irqn = DT_IRQN(DT_INST_PHANDLE(0, st_ltdc)),
	.pll_ndiv = DT_INST_PROP_BY_IDX(0, st_pll, 0),
	.pll_idf = DT_INST_PROP_BY_IDX(0, st_pll, 1),
	.pll_odf = DT_INST_PROP_BY_IDX(0, st_pll, 2),
	.txeckdiv = DT_INST_PROP(0, st_txeckdiv),
};

static struct dsi_stm32_data data;

DEVICE_DT_INST_DEFINE(0, &dsi_stm32_init, device_pm_control_nop, &data, &config,
		      POST_KERNEL, CONFIG_MIPI_DSI_INIT_PRIORITY,
		      &dsi_stm32_api);

static void ltdc_stm32_irq_connect()
{
	IRQ_CONNECT(DT_IRQN(DT_INST_PHANDLE(0, st_ltdc)),
		    DT_IRQ(DT_INST_PHANDLE(0, st_ltdc), priority),
		    ltdc_stm32_isr, DEVICE_DT_INST_GET(0), 0);
}
