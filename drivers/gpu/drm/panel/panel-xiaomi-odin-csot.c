// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM panel driver for Xiaomi MIX 4 CSOT K8 (42-02-0a) Novatek AMOLED panel
 */

#include <linux/atomic.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_encoder.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

#define XIAOMI_ODIN_CSOT_BL_FIRST_MS	80
#define XIAOMI_ODIN_CSOT_BL_RETRY_MS	200
#define XIAOMI_ODIN_CSOT_BL_SYNC_COUNT	5

struct xiaomi_odin_csot {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
	struct delayed_work bl_work;
	struct mutex lhbm_lock;	/* protects lhbm_mode and DSI writes */
	atomic_t bl_sync_rem;
	unsigned int lhbm_mode;
	bool prepared;
};

static const struct regulator_bulk_data xiaomi_odin_csot_supplies[] = {
	{ .supply = "vddio" },
	{ .supply = "vddd" },
	{ .supply = "avdd", .init_load_uA = 10000 },
};

static inline struct xiaomi_odin_csot *
to_xiaomi_odin_csot(struct drm_panel *panel)
{
	return container_of(panel, struct xiaomi_odin_csot, panel);
}

static void xiaomi_odin_csot_reset(struct xiaomi_odin_csot *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

#define NT37701_CMD_SWITCH_PAGE	0xf0

#define nt37701_switch_page(dsi_ctx, page) \
	mipi_dsi_dcs_write_seq_multi((dsi_ctx), NT37701_CMD_SWITCH_PAGE, \
				     0x55, 0xaa, 0x52, 0x08, (page))

static void xiaomi_odin_csot_on(struct mipi_dsi_multi_context *dsi_ctx,
				unsigned int vrefresh)
{
	dsi_ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x99, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x90, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x91,
				     0xab, 0x28, 0x00, 0x0c, 0xc2, 0x00,
				     0x03, 0x1c, 0x01, 0x7e, 0x00, 0x0f,
				     0x08, 0xbb, 0x04, 0x3d, 0x10, 0xf0);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x03, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MIPI_DCS_WRITE_MEMORY_START, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
				     0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_set_tear_on_multi(dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_set_column_address_multi(dsi_ctx, 0x0000, 0x0437);
	mipi_dsi_dcs_set_page_address_multi(dsi_ctx, 0x0000, 0x095f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x20);

	/* Select scan refresh rate */
	nt37701_switch_page(dsi_ctx, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x03);
	switch (vrefresh) {
	case 60:
		mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xbf, 0x02);
		mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x2f, 0x01);
		break;
	case 90:
		mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xbf, 0x05);
		mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x2f, 0x02);
		break;
	case 120:
	default:
		mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xbf, 0x03);
		mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x2f, 0x03);
		break;
	}

	nt37701_switch_page(dsi_ctx, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc3,
				     0x9b, 0x01, 0x92, 0xd0, 0x20, 0x01, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcd, 0x05, 0x61);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd2, 0x00, 0x00, 0x11);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd2, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x0f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd2, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x09);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd2, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xce, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x61);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xf3, 0x80);

	nt37701_switch_page(dsi_ctx, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc0, 0x4e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xbe, 0x88);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb5, 0x32);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x0b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb5, 0x33, 0x23, 0x2b);

	nt37701_switch_page(dsi_ctx, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcb, 0x06);

	nt37701_switch_page(dsi_ctx, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x29);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd9, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd9, 0x80);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x07);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb2, 0x07, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x17);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb2, 0x0f, 0xff);

	nt37701_switch_page(dsi_ctx, 0x07);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc0, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc9,
				     0x21, 0x00, 0x27, 0xd9, 0x27, 0xd9, 0x00,
				     0x00, 0x3f, 0xe0, 0x8e, 0xc6, 0x3f, 0xe0,
				     0x8e, 0xc6, 0x80, 0x06, 0x33, 0xd5, 0xf1,
				     0x00, 0x16, 0x13, 0x00, 0x7b, 0x78, 0x33,
				     0xd0, 0x27, 0xd9, 0x0f, 0x9b, 0x9b, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xca,
				     0x27, 0x00, 0x27, 0xd9, 0x27, 0xd9, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xe0,
				     0x8e, 0xc6, 0x80, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x7c, 0x13, 0x00, 0xe1, 0x78, 0x33,
				     0xd0, 0x00, 0x00, 0x03, 0x65, 0x9b, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcb,
				     0x2d, 0x00, 0x27, 0xd9, 0x27, 0xd9, 0x00,
				     0x00, 0x3f, 0xe0, 0x8e, 0xc6, 0x80, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x16, 0x79, 0x00, 0x7b, 0xde, 0x33,
				     0x70, 0x00, 0x00, 0x0c, 0x9b, 0x65, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcc,
				     0x2b, 0x00, 0x27, 0xd9, 0x27, 0xd9, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00,
				     0x00, 0x00, 0x7f, 0xf9, 0xcc, 0x2a, 0x0f,
				     0x00, 0x7c, 0x79, 0x00, 0xe1, 0xde, 0x33,
				     0x7f, 0xd8, 0x27, 0x00, 0x65, 0x65, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00);

	nt37701_switch_page(dsi_ctx, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xec, 0x00, 0xff);

	nt37701_switch_page(dsi_ctx, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc5, 0x10);
	nt37701_switch_page(dsi_ctx, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc0, 0x4e, 0xf3);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc0, 0x21);

	nt37701_switch_page(dsi_ctx, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc2, 0x14);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb1, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb2, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb8, 0x0f, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb8, 0x07, 0x70, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc7, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb7, 0x00, 0x40, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x0b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xca, 0xff);

	/* Under-display camera (CUP) zone compensation */
	nt37701_switch_page(dsi_ctx, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc1,
				     0xa7, 0x00, 0x3b, 0x81, 0x90, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc1,
				     0x64, 0x89, 0xa4, 0xbb, 0xcf, 0xe1, 0xf1);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc2,
				     0x88, 0x00, 0x00, 0x88, 0x00, 0x00, 0x88,
				     0x00, 0x00, 0x88, 0x00, 0x00, 0x88, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc3,
				     0x88, 0x00, 0x00, 0x88, 0x00, 0x00, 0x88,
				     0x00, 0x00, 0x88, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc4,
				     0x44, 0x00, 0x00, 0x44, 0x00, 0x00, 0x44,
				     0x00, 0x00, 0x04, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc5,
				     0x88, 0x00, 0x00, 0x88, 0x00, 0x00, 0x88,
				     0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc6, 0x11);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc7,
				     0x50, 0x50, 0x60, 0x70, 0x70, 0x80, 0x80,
				     0x80);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc8,
				     0x50, 0x50, 0x60, 0x70, 0x80, 0x80, 0xa0,
				     0xa0);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc9,
				     0x20, 0x20, 0x30, 0x40, 0x50, 0x70, 0xb0,
				     0xb0);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xca,
				     0x00, 0x14, 0x27, 0x00, 0x4e, 0x9d, 0x21,
				     0x3b, 0x74, 0xb8, 0xb0, 0x3e, 0x0f, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcb,
				     0x00, 0x13, 0x25, 0x00, 0x4a, 0x95, 0x21,
				     0x29, 0x52, 0xa8, 0x42, 0x94, 0x0f, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcd,
				     0x00, 0x0f, 0x1f, 0x00, 0x3d, 0x7a, 0x10,
				     0xf5, 0xea, 0xcb, 0x5c, 0x69, 0x0f, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xce,
				     0x22, 0x00, 0x00, 0x22, 0x00, 0x00, 0x22,
				     0x00, 0x00, 0x22, 0x00, 0x00, 0x22, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x0f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xce,
				     0x22, 0x00, 0x00, 0x22, 0x00, 0x00, 0x22,
				     0x00, 0x00, 0x22, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x1b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xce,
				     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				     0xff, 0xff, 0xff, 0xff, 0xff, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcf,
				     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				     0xff, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xff,
				     0xff, 0xff, 0xff, 0xff, 0xff, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x22);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd0,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd2, 0x00, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd2,
				     0x10, 0x00, 0x87, 0x00, 0x2c, 0x00, 0x00,
				     0x00, 0x00);

	mipi_dsi_dcs_exit_sleep_mode_multi(dsi_ctx);
	mipi_dsi_msleep(dsi_ctx, 85);
	mipi_dsi_dcs_set_display_on_multi(dsi_ctx);
	mipi_dsi_msleep(dsi_ctx, 20);
}

static int xiaomi_odin_csot_off(struct xiaomi_odin_csot *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);

	return dsi_ctx.accum_err;
}

static unsigned int xiaomi_odin_csot_get_vrefresh(struct drm_panel *panel)
{
	struct drm_encoder *encoder = panel->bridge.encoder;

	if (encoder && encoder->crtc && encoder->crtc->state) {
		unsigned int vrefresh =
			drm_mode_vrefresh(&encoder->crtc->state->adjusted_mode);
		if (vrefresh)
			return vrefresh;
	}

	return 120;
}

static int xiaomi_odin_csot_prepare(struct drm_panel *panel)
{
	struct xiaomi_odin_csot *ctx = to_xiaomi_odin_csot(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	struct drm_dsc_picture_parameter_set pps;
	unsigned int vrefresh = xiaomi_odin_csot_get_vrefresh(panel);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(xiaomi_odin_csot_supplies),
				    ctx->supplies);
	if (ret < 0)
		return ret;

	usleep_range(5000, 6000);
	xiaomi_odin_csot_reset(ctx);

	xiaomi_odin_csot_on(&dsi_ctx, vrefresh);
	/*
	 * Novatek NT37701 10bpc/8bpp DSC 1.1 QP table uses range_max_qp[0] = 8
	 * rather than the pre-SCR default of 4.
	 */
	ctx->dsc.rc_range_params[0].range_max_qp = 8;
	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	if (dsi_ctx.accum_err) {
		ret = dsi_ctx.accum_err;
		goto err_off;
	}

	ctx->prepared = true;

	return 0;

err_off:
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(xiaomi_odin_csot_supplies),
			       ctx->supplies);
	return ret;
}

static int xiaomi_odin_csot_unprepare(struct drm_panel *panel)
{
	struct xiaomi_odin_csot *ctx = to_xiaomi_odin_csot(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	atomic_set(&ctx->bl_sync_rem, 0);
	cancel_delayed_work_sync(&ctx->bl_work);
	mutex_lock(&ctx->lhbm_lock);
	ctx->prepared = false;
	ctx->lhbm_mode = 0;
	mutex_unlock(&ctx->lhbm_lock);

	ret = xiaomi_odin_csot_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to power off panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(xiaomi_odin_csot_supplies),
			       ctx->supplies);

	return 0;
}

static int xiaomi_odin_csot_enable(struct drm_panel *panel)
{
	struct xiaomi_odin_csot *ctx = to_xiaomi_odin_csot(panel);

	/*
	 * This NT37701 panel ignores DCS 0x51 brightness updates sent before
	 * the first pixel frame is transferred to GRAM. Since drm_panel_enable()
	 * runs before msm_atomic_commit_tail() flushes the first frame (~18 ms
	 * on warm unblank, ~70-95 ms during cold-boot fbcon registration),
	 * re-apply the backlight brightness across bounded post-enable passes.
	 */
	atomic_set(&ctx->bl_sync_rem, XIAOMI_ODIN_CSOT_BL_SYNC_COUNT);
	mod_delayed_work(system_dfl_wq, &ctx->bl_work,
			 msecs_to_jiffies(XIAOMI_ODIN_CSOT_BL_FIRST_MS));

	return 0;
}

static int xiaomi_odin_csot_disable(struct drm_panel *panel)
{
	struct xiaomi_odin_csot *ctx = to_xiaomi_odin_csot(panel);

	atomic_set(&ctx->bl_sync_rem, 0);
	cancel_delayed_work_sync(&ctx->bl_work);

	return 0;
}

static const struct drm_display_mode xiaomi_odin_csot_modes[] = {
	{
		/* 120 Hz */
		.clock = (1080 + 40 + 10 + 20) * (2400 + 652 + 2 + 500) * 120 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 40,
		.hsync_end = 1080 + 40 + 10,
		.htotal = 1080 + 40 + 10 + 20,
		.vdisplay = 2400,
		.vsync_start = 2400 + 652,
		.vsync_end = 2400 + 652 + 2,
		.vtotal = 2400 + 652 + 2 + 500,
		.width_mm = 70,
		.height_mm = 154,
		.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
	},
	{
		/* 90 Hz */
		.clock = (1080 + 40 + 10 + 20) * (2400 + 1336 + 2 + 1000) * 90 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 40,
		.hsync_end = 1080 + 40 + 10,
		.htotal = 1080 + 40 + 10 + 20,
		.vdisplay = 2400,
		.vsync_start = 2400 + 1336,
		.vsync_end = 2400 + 1336 + 2,
		.vtotal = 2400 + 1336 + 2 + 1000,
		.width_mm = 70,
		.height_mm = 154,
		.type = DRM_MODE_TYPE_DRIVER,
	},
	{
		/* 60 Hz */
		.clock = (1080 + 40 + 10 + 20) * (2400 + 2704 + 2 + 2000) * 60 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 40,
		.hsync_end = 1080 + 40 + 10,
		.htotal = 1080 + 40 + 10 + 20,
		.vdisplay = 2400,
		.vsync_start = 2400 + 2704,
		.vsync_end = 2400 + 2704 + 2,
		.vtotal = 2400 + 2704 + 2 + 2000,
		.width_mm = 70,
		.height_mm = 154,
		.type = DRM_MODE_TYPE_DRIVER,
	},
};

static int xiaomi_odin_csot_get_modes(struct drm_panel *panel,
				      struct drm_connector *connector)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(xiaomi_odin_csot_modes); i++) {
		const struct drm_display_mode *m = &xiaomi_odin_csot_modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode)
			return -ENOMEM;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = xiaomi_odin_csot_modes[0].width_mm;
	connector->display_info.height_mm = xiaomi_odin_csot_modes[0].height_mm;

	return ARRAY_SIZE(xiaomi_odin_csot_modes);
}

static const struct drm_panel_funcs xiaomi_odin_csot_panel_funcs = {
	.prepare = xiaomi_odin_csot_prepare,
	.unprepare = xiaomi_odin_csot_unprepare,
	.enable = xiaomi_odin_csot_enable,
	.disable = xiaomi_odin_csot_disable,
	.get_modes = xiaomi_odin_csot_get_modes,
};

static int xiaomi_odin_csot_set_lhbm(struct xiaomi_odin_csot *ctx,
				     unsigned int mode)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u16 brightness;

	guard(mutex)(&ctx->lhbm_lock);

	if (!ctx->prepared)
		return -EAGAIN;

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	if (mode == 0) {
		u8 bl_cmd[3];

		brightness = backlight_get_brightness(ctx->panel.backlight);
		bl_cmd[0] = MIPI_DCS_SET_DISPLAY_BRIGHTNESS;
		bl_cmd[1] = brightness >> 8;
		bl_cmd[2] = brightness & 0xff;
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx,
					     MIPI_DCS_WRITE_CONTROL_DISPLAY,
					     0x20);
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, bl_cmd,
						sizeof(bl_cmd));
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x87, 0x00, 0x00, 0x04);
		if (!dsi_ctx.accum_err)
			ctx->lhbm_mode = 0;
		return dsi_ctx.accum_err;
	}

	nt37701_switch_page(&dsi_ctx, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x08);
	switch (mode) {
	case 1:
		/* White 1000-nit FOD circle */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd0,
					     0x0b, 0x07, 0x09, 0x7b,
					     0x0c, 0x1d);
		break;
	case 2:
		/* White 110-nit low-brightness FOD circle */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd0,
					     0x07, 0x62, 0x06, 0x74,
					     0x07, 0xf7);
		break;
	case 3:
		/* Green 500-nit FOD circle */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd0,
					     0x00, 0x00, 0x09, 0x89,
					     0x00, 0x00);
		break;
	default:
		return -EINVAL;
	}

	nt37701_switch_page(&dsi_ctx, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0x00, 0x44);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x20);
	/* FOD optical sensor center coordinates: (539, 1802) */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x88, 0x02, 0x1b, 0x07, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x87, 0x0f, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x87, 0x0f, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
				     0x0f, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x87, 0x05);
	mipi_dsi_msleep(&dsi_ctx, 33);

	if (!dsi_ctx.accum_err)
		ctx->lhbm_mode = mode;

	return dsi_ctx.accum_err;
}

static ssize_t lhbm_mode_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct xiaomi_odin_csot *ctx = dev_get_drvdata(dev);
	unsigned int mode;

	mutex_lock(&ctx->lhbm_lock);
	mode = ctx->lhbm_mode;
	mutex_unlock(&ctx->lhbm_lock);

	return sysfs_emit(buf, "%u\n", mode);
}

static ssize_t lhbm_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct xiaomi_odin_csot *ctx = dev_get_drvdata(dev);
	unsigned int mode;
	int ret;

	ret = kstrtouint(buf, 0, &mode);
	if (ret)
		return ret;

	ret = xiaomi_odin_csot_set_lhbm(ctx, mode);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(lhbm_mode);

static struct attribute *xiaomi_odin_csot_attrs[] = {
	&dev_attr_lhbm_mode.attr,
	NULL
};
ATTRIBUTE_GROUPS(xiaomi_odin_csot);

static int xiaomi_odin_csot_bl_update_status(struct backlight_device *bl)
{
	struct xiaomi_odin_csot *ctx = bl_get_data(bl);
	struct mipi_dsi_device *dsi = ctx->dsi;
	u16 brightness = backlight_get_brightness(bl);

	guard(mutex)(&ctx->lhbm_lock);

	if (!ctx->prepared || ctx->lhbm_mode)
		return 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	return mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
}

static void xiaomi_odin_csot_bl_work(struct work_struct *work)
{
	struct xiaomi_odin_csot *ctx =
		container_of(work, struct xiaomi_odin_csot, bl_work.work);

	if (!ctx->prepared)
		return;

	backlight_update_status(ctx->panel.backlight);

	if (atomic_fetch_add_unless(&ctx->bl_sync_rem, -1, 0) > 1)
		mod_delayed_work(system_dfl_wq, &ctx->bl_work,
				 msecs_to_jiffies(XIAOMI_ODIN_CSOT_BL_RETRY_MS));
}

static const struct backlight_ops xiaomi_odin_csot_bl_ops = {
	.update_status = xiaomi_odin_csot_bl_update_status,
};

static struct backlight_device *
xiaomi_odin_csot_create_backlight(struct xiaomi_odin_csot *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 1024,
		.max_brightness = 4095,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, ctx,
					      &xiaomi_odin_csot_bl_ops, &props);
}

static int xiaomi_odin_csot_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct xiaomi_odin_csot *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct xiaomi_odin_csot, panel,
				   &xiaomi_odin_csot_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_regulator_bulk_get_const(dev,
					    ARRAY_SIZE(xiaomi_odin_csot_supplies),
					    xiaomi_odin_csot_supplies,
					    &ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset GPIO\n");

	ctx->dsi = dsi;
	mutex_init(&ctx->lhbm_lock);
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;
	ctx->panel.backlight = xiaomi_odin_csot_create_backlight(ctx);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	ret = devm_delayed_work_autocancel(dev, &ctx->bl_work,
					   xiaomi_odin_csot_bl_work);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to init backlight work\n");

	/*
	 * Qualcomm ABL leaves vddio/vddd/avdd enabled (regulator-boot-on) with
	 * the NT37701 DDIC and its internal AMOLED ELVDD/ELVSS charge pumps
	 * active at 60 Hz when msm_mdss_reset() cuts the DSI link. Power down
	 * and discharge the rails while DSI is still in LP-00 and RESET# is
	 * asserted so the first prepare() performs a clean 0 V cold power-on.
	 */
	ret = regulator_bulk_enable(ARRAY_SIZE(xiaomi_odin_csot_supplies),
				    ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Failed to claim boot supplies\n");
	regulator_bulk_disable(ARRAY_SIZE(xiaomi_odin_csot_supplies),
			       ctx->supplies);
	msleep(80);

	drm_panel_add(&ctx->panel);

	dsi->dsc = &ctx->dsc;
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.slice_height = 12;
	ctx->dsc.slice_width = 1080;
	ctx->dsc.slice_count = 1;
	ctx->dsc.bits_per_component = 10;
	ctx->dsc.bits_per_pixel = 8 << 4;
	ctx->dsc.block_pred_enable = true;

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void xiaomi_odin_csot_remove(struct mipi_dsi_device *dsi)
{
	struct xiaomi_odin_csot *ctx = mipi_dsi_get_drvdata(dsi);

	atomic_set(&ctx->bl_sync_rem, 0);
	cancel_delayed_work_sync(&ctx->bl_work);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id xiaomi_odin_csot_of_match[] = {
	{ .compatible = "xiaomi,odin-csot-nt37701" },
	{ }
};
MODULE_DEVICE_TABLE(of, xiaomi_odin_csot_of_match);

static struct mipi_dsi_driver xiaomi_odin_csot_driver = {
	.probe = xiaomi_odin_csot_probe,
	.remove = xiaomi_odin_csot_remove,
	.driver = {
		.name = "panel-xiaomi-odin-csot",
		.of_match_table = xiaomi_odin_csot_of_match,
		.dev_groups = xiaomi_odin_csot_groups,
	},
};
module_mipi_dsi_driver(xiaomi_odin_csot_driver);

MODULE_DESCRIPTION("DRM Panel Driver for Xiaomi MIX 4 CSOT NT37701 AMOLED Panel");
MODULE_LICENSE("GPL");
