// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5KHMX 108MP CMOS image sensor driver
 *
 * Copyright (c) 2026 Alexandru Andries <alex.andries.aa@gmail.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define S5KHMX_MCLK_FREQ_19P2MHZ	(19200000UL)
#define S5KHMX_LINK_FREQ_601P6MHZ	(601600000ULL)
#define S5KHMX_LINK_FREQ_1798P4MHZ	(1798400000ULL)
#define S5KHMX_PIXEL_RATE_896MHZ	(896000000ULL)
#define S5KHMX_PIXEL_RATE_1143P5MHZ	(1143466667ULL)
#define S5KHMX_DATA_LANES		3

#define S5KHMX_NATIVE_WIDTH		12048
#define S5KHMX_NATIVE_HEIGHT		9040
#define S5KHMX_PIXEL_ARRAY_LEFT		8
#define S5KHMX_PIXEL_ARRAY_TOP		8
#define S5KHMX_PIXEL_ARRAY_WIDTH	12032
#define S5KHMX_PIXEL_ARRAY_HEIGHT	9024

#define S5KHMX_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5KHMX_CHIP_ID			0x1ad0

#define S5KHMX_REG_CTRL_MODE		CCI_REG16(0x0100)
#define S5KHMX_MODE_STREAMING		BIT(8)
#define S5KHMX_VFLIP			BIT(1)
#define S5KHMX_HFLIP			BIT(0)

#define S5KHMX_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5KHMX_EXPOSURE_MIN		6
#define S5KHMX_EXPOSURE_STEP		1
#define S5KHMX_EXPOSURE_DEFAULT		0x0400
#define S5KHMX_EXPOSURE_MARGIN		20

#define S5KHMX_REG_AGAIN		CCI_REG16(0x0204)
#define S5KHMX_AGAIN_MIN		32
#define S5KHMX_AGAIN_MAX		1024
#define S5KHMX_AGAIN_STEP		1
#define S5KHMX_AGAIN_DEFAULT		32

#define S5KHMX_REG_VTS			CCI_REG16(0x0340)
#define S5KHMX_VTS_MAX			0xffff

#define S5KHMX_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define to_s5khmx(_sd)			container_of(_sd, struct s5khmx, sd)

static const s64 s5khmx_link_freq_menu[] = {
	S5KHMX_LINK_FREQ_601P6MHZ,
	S5KHMX_LINK_FREQ_1798P4MHZ,
};

static const u32 s5khmx_mbus_formats[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
};

struct s5khmx_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5khmx_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u32 link_freq_idx;
	u64 pixel_rate;
	struct v4l2_rect crop;
	const struct s5khmx_reg_list reg_list;
};

static const char * const s5khmx_test_pattern_menu[] = {
	"Disabled",
	"Solid colour",
	"Colour bars",
	"Fade to grey colour bars",
	"PN9",
};

static const char * const s5khmx_supply_names[] = {
	"vdda",
	"vddd",
	"vddio",
};

#define S5KHMX_NUM_SUPPLIES		ARRAY_SIZE(s5khmx_supply_names)

struct s5khmx {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[S5KHMX_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;

	const struct s5khmx_mode *mode;

	struct device_node *ois_node;
	struct i2c_client *ois_client;
};

static const struct cci_reg_sequence s5khmx_sw_reset_regs[] = {
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x0000), 0x0002 },
	{ CCI_REG16(0x0000), 0x1ad0 },
	{ CCI_REG16(0x6010), 0x0001 },
};

static const struct cci_reg_sequence s5khmx_init_regs[] = {
	{ CCI_REG16(0x6214), 0xff7d },
	{ CCI_REG16(0x6218), 0x0000 },
	{ CCI_REG16(0x0a02), 0x01f4 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0xbc90 },
	{ CCI_REG16(0x6f12), 0x0a49 },
	{ CCI_REG16(0x6f12), 0x10b5 },
	{ CCI_REG16(0x6f12), 0x0848 },
	{ CCI_REG16(0x6f12), 0x0a4a },
	{ CCI_REG16(0x6f12), 0xc1f8 },
	{ CCI_REG16(0x6f12), 0xec06 },
	{ CCI_REG16(0x6f12), 0x101a },
	{ CCI_REG16(0x6f12), 0xa1f8 },
	{ CCI_REG16(0x6f12), 0xf006 },
	{ CCI_REG16(0x6f12), 0xfff7 },
	{ CCI_REG16(0x6f12), 0x2ffe },
	{ CCI_REG16(0x6f12), 0x0748 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x0749 },
	{ CCI_REG16(0x6f12), 0x436f },
	{ CCI_REG16(0x6f12), 0x0748 },
	{ CCI_REG16(0x6f12), 0x9847 },
	{ CCI_REG16(0x6f12), 0x0749 },
	{ CCI_REG16(0x6f12), 0x0860 },
	{ CCI_REG16(0x6f12), 0x10bd },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0xbd54 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x6a70 },
	{ CCI_REG16(0x6f12), 0x2001 },
	{ CCI_REG16(0x6f12), 0xfa00 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x5d20 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0xbce1 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0xb427 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0xbd50 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x70b5 },
	{ CCI_REG16(0x6f12), 0x0d4c },
	{ CCI_REG16(0x6f12), 0x0d4e },
	{ CCI_REG16(0x6f12), 0x2089 },
	{ CCI_REG16(0x6f12), 0x3080 },
	{ CCI_REG16(0x6f12), 0x0d4d },
	{ CCI_REG16(0x6f12), 0x0b49 },
	{ CCI_REG16(0x6f12), 0x4e39 },
	{ CCI_REG16(0x6f12), 0xb5f8 },
	{ CCI_REG16(0x6f12), 0x8803 },
	{ CCI_REG16(0x6f12), 0x0880 },
	{ CCI_REG16(0x6f12), 0x0120 },
	{ CCI_REG16(0x6f12), 0x891e },
	{ CCI_REG16(0x6f12), 0x0880 },
	{ CCI_REG16(0x6f12), 0x2068 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x11f8 },
	{ CCI_REG16(0x6f12), 0x6089 },
	{ CCI_REG16(0x6f12), 0x3080 },
	{ CCI_REG16(0x6f12), 0x6068 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x0cf8 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x0ff8 },
	{ CCI_REG16(0x6f12), 0xc5f8 },
	{ CCI_REG16(0x6f12), 0x8003 },
	{ CCI_REG16(0x6f12), 0x70bd },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0xbd40 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0xf450 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x8fb0 },
	{ CCI_REG16(0x6f12), 0x4bf2 },
	{ CCI_REG16(0x6f12), 0x6b6c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4bf2 },
	{ CCI_REG16(0x6f12), 0x0f6c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x3800 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0600 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0011 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x1fc8 },
	{ CCI_REG16(0x6f12), 0x0067 },
	{ CCI_REG16(0x602a), 0x1fd0 },
	{ CCI_REG16(0x6f12), 0x0056 },
	{ CCI_REG16(0x602a), 0x1ffa },
	{ CCI_REG16(0x6f12), 0x0082 },
	{ CCI_REG16(0x602a), 0x2036 },
	{ CCI_REG16(0x6f12), 0x0018 },
	{ CCI_REG16(0x6f12), 0x05ea },
	{ CCI_REG16(0x6f12), 0x05fe },
	{ CCI_REG16(0x6f12), 0x0608 },
	{ CCI_REG16(0x6f12), 0x0018 },
	{ CCI_REG16(0x6f12), 0x05ea },
	{ CCI_REG16(0x6f12), 0x0608 },
	{ CCI_REG16(0x6f12), 0x05fe },
	{ CCI_REG16(0x6f12), 0x060a },
	{ CCI_REG16(0x6f12), 0x060a },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x207a },
	{ CCI_REG16(0x6f12), 0x01a8 },
	{ CCI_REG16(0x602a), 0x2138 },
	{ CCI_REG16(0x6f12), 0x005e },
	{ CCI_REG16(0x602a), 0x23c6 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x602a), 0x23d8 },
	{ CCI_REG16(0x6f12), 0x035e },
	{ CCI_REG16(0x602a), 0x23b8 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x4480 },
	{ CCI_REG16(0x6f12), 0x530d },
	{ CCI_REG16(0x602a), 0x273c },
	{ CCI_REG16(0x6f12), 0x04d2 },
	{ CCI_REG16(0x602a), 0x3338 },
	{ CCI_REG16(0x6f12), 0x0027 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x602a), 0x363a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0xf44a), 0x0014 },
	{ CCI_REG16(0xf446), 0x000d },
	{ CCI_REG16(0xf450), 0x0011 },
	{ CCI_REG16(0xf452), 0x002e },
	{ CCI_REG16(0xf65e), 0x3f00 },
	{ CCI_REG16(0x6b76), 0xb000 },
	{ CCI_REG16(0x0bc6), 0x0000 },
	{ CCI_REG16(0x602a), 0x125a },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x3680 },
	{ CCI_REG16(0x6f12), 0x193c },
	{ CCI_REG16(0x602a), 0xba2e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0xba44 },
	{ CCI_REG16(0x6f12), 0x7ac0 },
	{ CCI_REG16(0x6f12), 0x7ad0 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0003 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x0003 },
	{ CCI_REG16(0x602a), 0xba54 },
	{ CCI_REG16(0x6f12), 0x0dfb },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x04ae },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6028), 0x2001 },
	{ CCI_REG16(0x602a), 0xe660 },
	{ CCI_REG16(0x6f12), 0x7abd },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0xe666 },
	{ CCI_REG16(0x6f12), 0x0006 },
	{ CCI_REG16(0x6f12), 0x04ba },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x008f },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x200a },
	{ CCI_REG16(0x6f12), 0x0090 },
	{ CCI_REG16(0x602a), 0xe690 },
	{ CCI_REG16(0x6f12), 0x0090 },
	{ CCI_REG16(0x6f12), 0x0091 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x602a), 0xe6b4 },
	{ CCI_REG16(0x6f12), 0x0092 },
	{ CCI_REG16(0x6f12), 0x2279 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x200a },
	{ CCI_REG16(0x6f12), 0x0134 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x200a },
	{ CCI_REG16(0x6f12), 0x0134 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x602a), 0xe6d8 },
	{ CCI_REG16(0x6f12), 0x227a },
	{ CCI_REG16(0x6f12), 0x233f },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x200a },
	{ CCI_REG16(0x6f12), 0x00c6 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x3e5c },
	{ CCI_REG16(0x6f12), 0x0141 },
	{ CCI_REG16(0x602a), 0x49ee },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x49e2 },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x602a), 0x4b2a },
	{ CCI_REG16(0x6f12), 0x0018 },
	{ CCI_REG16(0x602a), 0x4b42 },
	{ CCI_REG16(0x6f12), 0x0018 },
	{ CCI_REG16(0x602a), 0x4b5a },
	{ CCI_REG16(0x6f12), 0x0018 },
	{ CCI_REG16(0x602a), 0x4b72 },
	{ CCI_REG16(0x6f12), 0x0018 },
	{ CCI_REG16(0x602a), 0x4b36 },
	{ CCI_REG16(0x6f12), 0x001c },
	{ CCI_REG16(0x602a), 0x4b4e },
	{ CCI_REG16(0x6f12), 0x001c },
	{ CCI_REG16(0x602a), 0x4b66 },
	{ CCI_REG16(0x6f12), 0x001c },
	{ CCI_REG16(0x602a), 0x4b7e },
	{ CCI_REG16(0x6f12), 0x001c },
	{ CCI_REG16(0x602a), 0x4b96 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x602a), 0x4bae },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x602a), 0x4bc6 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x602a), 0x4bde },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x602a), 0x4ba2 },
	{ CCI_REG16(0x6f12), 0x0014 },
	{ CCI_REG16(0x602a), 0x4bba },
	{ CCI_REG16(0x6f12), 0x0014 },
	{ CCI_REG16(0x602a), 0x4bd2 },
	{ CCI_REG16(0x6f12), 0x0014 },
	{ CCI_REG16(0x602a), 0x4bea },
	{ CCI_REG16(0x6f12), 0x0014 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x0106), 0x017a },
	{ CCI_REG16(0x0bcc), 0x0000 },
};

static const struct cci_reg_sequence s5khmx_mode_3008x2256_regs[] = {
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x0136), 0x1300 },
	{ CCI_REG16(0x013e), 0x00c8 },
	{ CCI_REG16(0x0304), 0x0003 },
	{ CCI_REG16(0x0306), 0x010c },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x0302), 0x0003 },
	{ CCI_REG16(0x0300), 0x0002 },
	{ CCI_REG16(0x030e), 0x0003 },
	{ CCI_REG16(0x0310), 0x00bc },
	{ CCI_REG16(0x0312), 0x0002 },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x030a), 0x0002 },
	{ CCI_REG16(0x602a), 0x23be },
	{ CCI_REG16(0x6f12), 0x0096 },
	{ CCI_REG16(0x0344), 0x0008 },
	{ CCI_REG16(0x0346), 0x0008 },
	{ CCI_REG16(0x0348), 0x2f07 },
	{ CCI_REG16(0x034a), 0x2347 },
	{ CCI_REG16(0x0350), 0x0000 },
	{ CCI_REG16(0x0352), 0x0000 },
	{ CCI_REG16(0x034c), 0x0bc0 },
	{ CCI_REG16(0x034e), 0x08d0 },
	{ CCI_REG16(0x0900), 0x0124 },
	{ CCI_REG16(0x0400), 0x1010 },
	{ CCI_REG16(0x0404), 0x2000 },
	{ CCI_REG16(0x0380), 0x0002 },
	{ CCI_REG16(0x0382), 0x0002 },
	{ CCI_REG16(0x0384), 0x0004 },
	{ CCI_REG16(0x0386), 0x0004 },
	{ CCI_REG16(0x602a), 0x38e6 },
	{ CCI_REG16(0x6f12), 0x0108 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x120e },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x1216 },
	{ CCI_REG16(0x6f12), 0x0e15 },
	{ CCI_REG16(0x602a), 0x4630 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x03b8 },
	{ CCI_REG16(0x0342), 0x3ea0 },
	{ CCI_REG16(0x0340), 0x0948 },
	{ CCI_REG16(0x0114), 0x0201 },
	{ CCI_REG16(0x0118), 0x0001 },
	{ CCI_REG16(0x011c), 0x0101 },
	{ CCI_REG16(0x081c), 0x000c },
	{ CCI_REG16(0x081e), 0x0c00 },
	{ CCI_REG16(0x602a), 0x127a },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x602a), 0x1f2e },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x39a2 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x602a), 0x3902 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x0d00), 0x0101 },
	{ CCI_REG16(0x0d02), 0x0101 },
	{ CCI_REG16(0x0d04), 0x0102 },
	{ CCI_REG16(0x602a), 0x4afe },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x4310 },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x6f12), 0x0402 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x4318 },
	{ CCI_REG16(0x6f12), 0x0003 },
	{ CCI_REG16(0x602a), 0x4396 },
	{ CCI_REG16(0x6f12), 0x0077 },
	{ CCI_REG16(0x602a), 0x43c0 },
	{ CCI_REG16(0x6f12), 0x0204 },
	{ CCI_REG16(0x602a), 0x434e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x2920 },
	{ CCI_REG16(0x6f12), 0x03c0 },
	{ CCI_REG16(0x6f12), 0x06c8 },
	{ CCI_REG16(0x6f12), 0x03d0 },
	{ CCI_REG16(0x6f12), 0x06d8 },
	{ CCI_REG16(0x6f12), 0x03e0 },
	{ CCI_REG16(0x6f12), 0x06e8 },
	{ CCI_REG16(0x6f12), 0x03f0 },
	{ CCI_REG16(0x6f12), 0x06f8 },
	{ CCI_REG16(0x602a), 0x23ce },
	{ CCI_REG16(0x6f12), 0x009c },
	{ CCI_REG16(0x602a), 0x275a },
	{ CCI_REG16(0x6f12), 0x0700 },
	{ CCI_REG16(0x602a), 0x44ec },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x3e36 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x5dd0 },
	{ CCI_REG16(0x6f12), 0x8370 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x8372 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x8374 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x8318 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x831c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x839e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x83a2 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x8418 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x841c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5f60 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5f66 },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x602a), 0x44b2 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x44e4 },
	{ CCI_REG16(0x6f12), 0x0433 },
	{ CCI_REG16(0x6f12), 0x0433 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x44dc },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x07c0 },
	{ CCI_REG16(0x6f12), 0x07c0 },
	{ CCI_REG16(0x602a), 0x43f2 },
	{ CCI_REG16(0x6f12), 0x0123 },
	{ CCI_REG16(0x602a), 0x43fa },
	{ CCI_REG16(0x6f12), 0x3210 },
	{ CCI_REG16(0x602a), 0x4412 },
	{ CCI_REG16(0x6f12), 0x0123 },
	{ CCI_REG16(0x602a), 0x441a },
	{ CCI_REG16(0x6f12), 0x3210 },
	{ CCI_REG16(0x602a), 0x3f82 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x3af0 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x3fff },
	{ CCI_REG16(0x6f12), 0x3fff },
	{ CCI_REG16(0x602a), 0x3972 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x3fff },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x3fff },
	{ CCI_REG16(0x0b08), 0x0101 },
	{ CCI_REG16(0x602a), 0x3954 },
	{ CCI_REG16(0x6f12), 0x0bc0 },
	{ CCI_REG16(0x602a), 0x3958 },
	{ CCI_REG16(0x6f12), 0x08d0 },
	{ CCI_REG16(0x602a), 0xba2c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0xba42 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0xba30 },
	{ CCI_REG16(0x6f12), 0xf880 },
	{ CCI_REG16(0x6f12), 0xfd80 },
	{ CCI_REG16(0x6028), 0x2001 },
	{ CCI_REG16(0x602a), 0xe664 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0xf472), 0x0010 },
};

static const struct cci_reg_sequence s5khmx_mode_6016x4512_regs[] = {
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x0136), 0x1300 },
	{ CCI_REG16(0x013e), 0x00c8 },
	{ CCI_REG16(0x0304), 0x0003 },
	{ CCI_REG16(0x0306), 0x00d2 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x0302), 0x0003 },
	{ CCI_REG16(0x0300), 0x0002 },
	{ CCI_REG16(0x030e), 0x0003 },
	{ CCI_REG16(0x0310), 0x0119 },
	{ CCI_REG16(0x0312), 0x0001 },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x030a), 0x0002 },
	{ CCI_REG16(0x602a), 0x23be },
	{ CCI_REG16(0x6f12), 0x0094 },
	{ CCI_REG16(0x0344), 0x0008 },
	{ CCI_REG16(0x0346), 0x0008 },
	{ CCI_REG16(0x0348), 0x2f07 },
	{ CCI_REG16(0x034a), 0x2347 },
	{ CCI_REG16(0x0350), 0x0000 },
	{ CCI_REG16(0x0352), 0x0000 },
	{ CCI_REG16(0x034c), 0x1780 },
	{ CCI_REG16(0x034e), 0x11a0 },
	{ CCI_REG16(0x0900), 0x0122 },
	{ CCI_REG16(0x0400), 0x1010 },
	{ CCI_REG16(0x0404), 0x1000 },
	{ CCI_REG16(0x0380), 0x0002 },
	{ CCI_REG16(0x0382), 0x0002 },
	{ CCI_REG16(0x0384), 0x0002 },
	{ CCI_REG16(0x0386), 0x0002 },
	{ CCI_REG16(0x602a), 0x38e6 },
	{ CCI_REG16(0x6f12), 0x0208 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x120e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1216 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x602a), 0x4630 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x0342), 0x1908 },
	{ CCI_REG16(0x0340), 0x1232 },
	{ CCI_REG16(0x0114), 0x0201 },
	{ CCI_REG16(0x0118), 0x0001 },
	{ CCI_REG16(0x011c), 0x0101 },
	{ CCI_REG16(0x081c), 0x000c },
	{ CCI_REG16(0x081e), 0x0c00 },
	{ CCI_REG16(0x602a), 0x127a },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x602a), 0x1f2e },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x39a2 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x602a), 0x3902 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x0d00), 0x0101 },
	{ CCI_REG16(0x0d02), 0x0101 },
	{ CCI_REG16(0x0d04), 0x0102 },
	{ CCI_REG16(0x602a), 0x4afe },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x4310 },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x6f12), 0x0201 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x4318 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x4396 },
	{ CCI_REG16(0x6f12), 0x0077 },
	{ CCI_REG16(0x602a), 0x43c0 },
	{ CCI_REG16(0x6f12), 0x0108 },
	{ CCI_REG16(0x602a), 0x434e },
	{ CCI_REG16(0x6f12), 0x0202 },
	{ CCI_REG16(0x6f12), 0x0302 },
	{ CCI_REG16(0x602a), 0x2920 },
	{ CCI_REG16(0x6f12), 0x03c0 },
	{ CCI_REG16(0x6f12), 0x03c8 },
	{ CCI_REG16(0x6f12), 0x06d0 },
	{ CCI_REG16(0x6f12), 0x06d8 },
	{ CCI_REG16(0x6f12), 0x03e0 },
	{ CCI_REG16(0x6f12), 0x03e8 },
	{ CCI_REG16(0x6f12), 0x06f0 },
	{ CCI_REG16(0x6f12), 0x06f8 },
	{ CCI_REG16(0x602a), 0x23ce },
	{ CCI_REG16(0x6f12), 0x009c },
	{ CCI_REG16(0x602a), 0x275a },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x44ec },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x3e36 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x5dd0 },
	{ CCI_REG16(0x6f12), 0x8370 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x8372 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x8374 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x8f4c },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x8f5a },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x8f5c },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x8f56 },
	{ CCI_REG16(0x6f12), 0x0433 },
	{ CCI_REG16(0x6f12), 0x8f58 },
	{ CCI_REG16(0x6f12), 0x0433 },
	{ CCI_REG16(0x6f12), 0x8f4e },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x8f50 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x8f52 },
	{ CCI_REG16(0x6f12), 0x07c0 },
	{ CCI_REG16(0x6f12), 0x8f54 },
	{ CCI_REG16(0x6f12), 0x07c0 },
	{ CCI_REG16(0x6f12), 0x8318 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x831c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x839e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x83a2 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x8418 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x841c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5f60 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5f66 },
	{ CCI_REG16(0x6f12), 0x0012 },
	{ CCI_REG16(0x602a), 0x44b2 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x44e4 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x44dc },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x43f2 },
	{ CCI_REG16(0x6f12), 0x3210 },
	{ CCI_REG16(0x602a), 0x43fa },
	{ CCI_REG16(0x6f12), 0x0123 },
	{ CCI_REG16(0x602a), 0x4412 },
	{ CCI_REG16(0x6f12), 0x0123 },
	{ CCI_REG16(0x602a), 0x441a },
	{ CCI_REG16(0x6f12), 0x3210 },
	{ CCI_REG16(0x602a), 0x3f82 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x3af0 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x3fff },
	{ CCI_REG16(0x6f12), 0x3fff },
	{ CCI_REG16(0x602a), 0x3972 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x3fff },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x3fff },
	{ CCI_REG16(0x0b08), 0x0101 },
	{ CCI_REG16(0x602a), 0x3954 },
	{ CCI_REG16(0x6f12), 0x1780 },
	{ CCI_REG16(0x602a), 0x3958 },
	{ CCI_REG16(0x6f12), 0x11a0 },
	{ CCI_REG16(0x602a), 0xba2c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0xba42 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0xba30 },
	{ CCI_REG16(0x6f12), 0xf880 },
	{ CCI_REG16(0x6f12), 0xfd80 },
	{ CCI_REG16(0x6028), 0x2001 },
	{ CCI_REG16(0x602a), 0xe664 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0xf472), 0x0015 },
};

static const struct s5khmx_mode s5khmx_supported_modes[] = {
	{
		.width = 3008,
		.height = 2256,
		.hts = 16032,
		.vts = 2376,
		.link_freq_idx = 0,
		.pixel_rate = S5KHMX_PIXEL_RATE_1143P5MHZ,
		.crop = {
			.left = S5KHMX_PIXEL_ARRAY_LEFT,
			.top = S5KHMX_PIXEL_ARRAY_TOP,
			.width = S5KHMX_PIXEL_ARRAY_WIDTH,
			.height = S5KHMX_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.regs = s5khmx_mode_3008x2256_regs,
			.num_regs = ARRAY_SIZE(s5khmx_mode_3008x2256_regs),
		},
	},
	{
		.width = 6016,
		.height = 4512,
		.hts = 6408,
		.vts = 4658,
		.link_freq_idx = 1,
		.pixel_rate = S5KHMX_PIXEL_RATE_896MHZ,
		.crop = {
			.left = S5KHMX_PIXEL_ARRAY_LEFT,
			.top = S5KHMX_PIXEL_ARRAY_TOP,
			.width = S5KHMX_PIXEL_ARRAY_WIDTH,
			.height = S5KHMX_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.regs = s5khmx_mode_6016x4512_regs,
			.num_regs = ARRAY_SIZE(s5khmx_mode_6016x4512_regs),
		},
	},
};

static int s5khmx_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5khmx *s5khmx =
		container_of(ctrl->handler, struct s5khmx, ctrl_handler);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		s64 exposure_max = s5khmx->mode->height + ctrl->val -
				   S5KHMX_EXPOSURE_MARGIN;
		s64 exposure_val = clamp_t(s64, s5khmx->exposure->val,
					   S5KHMX_EXPOSURE_MIN, exposure_max);

		__v4l2_ctrl_modify_range(s5khmx->exposure, S5KHMX_EXPOSURE_MIN,
					 exposure_max, S5KHMX_EXPOSURE_STEP,
					 exposure_val);
	}

	if (!pm_runtime_get_if_in_use(s5khmx->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(s5khmx->regmap, S5KHMX_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(s5khmx->regmap, S5KHMX_REG_AGAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(s5khmx->regmap, S5KHMX_REG_VTS,
			  s5khmx->mode->height + ctrl->val, &ret);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(s5khmx->regmap, S5KHMX_REG_TEST_PATTERN,
			  ctrl->val, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(s5khmx->dev);
	pm_runtime_put_autosuspend(s5khmx->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5khmx_ctrl_ops = {
	.s_ctrl = s5khmx_set_ctrl,
};

static int s5khmx_init_controls(struct s5khmx *s5khmx)
{
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5khmx->ctrl_handler;
	const struct s5khmx_mode *mode = s5khmx->mode;
	s64 hblank, vblank, exposure_max;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 10);
	if (ret)
		return ret;

	s5khmx->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL, V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(s5khmx_link_freq_menu) - 1,
				       mode->link_freq_idx,
				       s5khmx_link_freq_menu);
	if (s5khmx->link_freq)
		s5khmx->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5khmx->pixel_rate =
		v4l2_ctrl_new_std(ctrl_hdlr, NULL, V4L2_CID_PIXEL_RATE,
				  S5KHMX_PIXEL_RATE_896MHZ,
				  S5KHMX_PIXEL_RATE_1143P5MHZ, 1,
				  mode->pixel_rate);
	if (s5khmx->pixel_rate)
		s5khmx->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = mode->hts - mode->width;
	s5khmx->hblank = v4l2_ctrl_new_std(ctrl_hdlr, NULL, V4L2_CID_HBLANK,
					   hblank, hblank, 1, hblank);
	if (s5khmx->hblank)
		s5khmx->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	s5khmx->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5khmx_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   S5KHMX_VTS_MAX - mode->height, 1,
					   vblank);

	exposure_max = mode->vts - S5KHMX_EXPOSURE_MARGIN;
	s5khmx->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5khmx_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5KHMX_EXPOSURE_MIN, exposure_max,
					     S5KHMX_EXPOSURE_STEP,
					     S5KHMX_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5khmx_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5KHMX_AGAIN_MIN, S5KHMX_AGAIN_MAX,
			  S5KHMX_AGAIN_STEP, S5KHMX_AGAIN_DEFAULT);

	s5khmx->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5khmx_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (s5khmx->hflip)
		s5khmx->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	s5khmx->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5khmx_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (s5khmx->vflip)
		s5khmx->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5khmx_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5khmx_test_pattern_menu) - 1,
				     0, 0, s5khmx_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(s5khmx->dev, &props);
	if (!ret)
		v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5khmx_ctrl_ops,
						&props);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ret;
	}

	s5khmx->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

static void s5khmx_ois_get(struct s5khmx *s5khmx)
{
	struct i2c_client *ois;

	if (!s5khmx->ois_node || s5khmx->ois_client)
		return;

	ois = of_find_i2c_device_by_node(s5khmx->ois_node);
	if (!ois)
		return;

	if (pm_runtime_resume_and_get(&ois->dev) < 0) {
		put_device(&ois->dev);
		return;
	}

	s5khmx->ois_client = ois;
}

static void s5khmx_ois_put(struct s5khmx *s5khmx)
{
	if (!s5khmx->ois_client)
		return;

	pm_runtime_put_sync_suspend(&s5khmx->ois_client->dev);
	put_device(&s5khmx->ois_client->dev);
	s5khmx->ois_client = NULL;
}

static int s5khmx_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5khmx *s5khmx = to_s5khmx(sd);
	const struct s5khmx_reg_list *reg_list = &s5khmx->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(s5khmx->dev);
	if (ret < 0)
		return ret;

	ret = 0;
	cci_multi_reg_write(s5khmx->regmap, s5khmx_sw_reset_regs,
			    ARRAY_SIZE(s5khmx_sw_reset_regs), &ret);
	if (ret)
		goto error;

	usleep_range(15 * USEC_PER_MSEC, 16 * USEC_PER_MSEC);

	cci_multi_reg_write(s5khmx->regmap, s5khmx_init_regs,
			    ARRAY_SIZE(s5khmx_init_regs), &ret);
	cci_multi_reg_write(s5khmx->regmap, reg_list->regs,
			    reg_list->num_regs, &ret);
	cci_write(s5khmx->regmap, CCI_REG16(0x6028), 0x4000, &ret);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(s5khmx->sd.ctrl_handler);
	if (ret)
		goto error;

	cci_write(s5khmx->regmap, S5KHMX_REG_CTRL_MODE,
		  S5KHMX_MODE_STREAMING |
		  (s5khmx->vflip->val ? S5KHMX_VFLIP : 0) |
		  (s5khmx->hflip->val ? S5KHMX_HFLIP : 0), &ret);
	if (ret)
		goto error;

	s5khmx_ois_get(s5khmx);

	return 0;

error:
	dev_err(s5khmx->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_mark_last_busy(s5khmx->dev);
	pm_runtime_put_autosuspend(s5khmx->dev);

	return ret;
}

static int s5khmx_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5khmx *s5khmx = to_s5khmx(sd);
	int ret;

	s5khmx_ois_put(s5khmx);

	ret = cci_write(s5khmx->regmap, S5KHMX_REG_CTRL_MODE, 0, NULL);
	if (ret)
		dev_err(s5khmx->dev, "failed to stop streaming: %d\n", ret);
	else
		usleep_range(1000, 1500);

	pm_runtime_mark_last_busy(s5khmx->dev);
	pm_runtime_put_autosuspend(s5khmx->dev);

	return ret;
}

static u32 s5khmx_get_format_code(struct s5khmx *s5khmx)
{
	unsigned int i;

	i = (s5khmx->vflip->val ? 2 : 0) | (s5khmx->hflip->val ? 1 : 0);

	return s5khmx_mbus_formats[i];
}

static void s5khmx_update_pad_format(struct s5khmx *s5khmx,
				     const struct s5khmx_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5khmx_get_format_code(s5khmx);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5khmx_set_pad_format(struct v4l2_subdev *sd,
				 const struct v4l2_subdev_client_info *ci,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5khmx *s5khmx = to_s5khmx(sd);
	s64 hblank, vblank, exposure_max;
	const struct s5khmx_mode *mode;

	mode = v4l2_find_nearest_size(s5khmx_supported_modes,
				      ARRAY_SIZE(s5khmx_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	s5khmx_update_pad_format(s5khmx, mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || s5khmx->mode == mode)
		goto set_format;

	__v4l2_ctrl_s_ctrl(s5khmx->link_freq, mode->link_freq_idx);
	__v4l2_ctrl_s_ctrl_int64(s5khmx->pixel_rate, mode->pixel_rate);

	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(s5khmx->hblank, hblank, hblank, 1, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(s5khmx->vblank, vblank,
				 S5KHMX_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s5khmx->vblank, vblank);

	exposure_max = mode->vts - S5KHMX_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(s5khmx->exposure, S5KHMX_EXPOSURE_MIN,
				 exposure_max, S5KHMX_EXPOSURE_STEP,
				 S5KHMX_EXPOSURE_DEFAULT);
	__v4l2_ctrl_s_ctrl(s5khmx->exposure, S5KHMX_EXPOSURE_DEFAULT);

	if (s5khmx->sd.ctrl_handler->error)
		return s5khmx->sd.ctrl_handler->error;

	s5khmx->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int s5khmx_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5khmx *s5khmx = to_s5khmx(sd);

	if (code->index > 0)
		return -EINVAL;

	code->code = s5khmx_get_format_code(s5khmx);

	return 0;
}

static int s5khmx_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5khmx *s5khmx = to_s5khmx(sd);

	if (fse->index >= ARRAY_SIZE(s5khmx_supported_modes))
		return -EINVAL;

	if (fse->code != s5khmx_get_format_code(s5khmx))
		return -EINVAL;

	fse->min_width = s5khmx_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5khmx_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5khmx_get_selection(struct v4l2_subdev *sd,
				const struct v4l2_subdev_client_info *ci,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct s5khmx *s5khmx = to_s5khmx(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5KHMX_NATIVE_WIDTH;
		sel->r.height = S5KHMX_NATIVE_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = S5KHMX_PIXEL_ARRAY_LEFT;
		sel->r.top = S5KHMX_PIXEL_ARRAY_TOP;
		sel->r.width = S5KHMX_PIXEL_ARRAY_WIDTH;
		sel->r.height = S5KHMX_PIXEL_ARRAY_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP:
		sel->r = s5khmx->mode->crop;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5khmx_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct s5khmx *s5khmx = to_s5khmx(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = s5khmx->mode->width,
			.height = s5khmx->mode->height,
		},
	};

	s5khmx_set_pad_format(sd, NULL, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops s5khmx_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5khmx_pad_ops = {
	.set_fmt = s5khmx_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = s5khmx_get_selection,
	.enum_mbus_code = s5khmx_enum_mbus_code,
	.enum_frame_size = s5khmx_enum_frame_size,
	.enable_streams = s5khmx_enable_streams,
	.disable_streams = s5khmx_disable_streams,
};

static const struct v4l2_subdev_ops s5khmx_subdev_ops = {
	.video = &s5khmx_video_ops,
	.pad = &s5khmx_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5khmx_internal_ops = {
	.init_state = s5khmx_init_state,
};

static const struct media_entity_operations s5khmx_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int s5khmx_identify_sensor(struct s5khmx *s5khmx)
{
	u64 val;
	int ret;

	ret = cci_read(s5khmx->regmap, S5KHMX_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(s5khmx->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != S5KHMX_CHIP_ID) {
		dev_err(s5khmx->dev, "chip id mismatch: %x!=%llx\n",
			S5KHMX_CHIP_ID, val);
		return -ENODEV;
	}

	return 0;
}

static int s5khmx_check_hwcfg(struct s5khmx *s5khmx)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5khmx->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_CPHY,
	};
	unsigned long freq_bitmap;
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5KHMX_DATA_LANES) {
		dev_err(s5khmx->dev, "Invalid number of data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(s5khmx->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5khmx_link_freq_menu,
				       ARRAY_SIZE(s5khmx_link_freq_menu),
				       &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int s5khmx_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5khmx *s5khmx = to_s5khmx(sd);
	int ret;

	gpiod_set_value_cansleep(s5khmx->reset_gpio, 1);

	ret = regulator_bulk_enable(S5KHMX_NUM_SUPPLIES, s5khmx->supplies);
	if (ret)
		return ret;

	ret = clk_prepare_enable(s5khmx->mclk);
	if (ret)
		goto disable_regulators;

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(s5khmx->reset_gpio, 0);
	usleep_range(5 * USEC_PER_MSEC, 10 * USEC_PER_MSEC);

	return 0;

disable_regulators:
	regulator_bulk_disable(S5KHMX_NUM_SUPPLIES, s5khmx->supplies);

	return ret;
}

static int s5khmx_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5khmx *s5khmx = to_s5khmx(sd);

	gpiod_set_value_cansleep(s5khmx->reset_gpio, 1);

	clk_disable_unprepare(s5khmx->mclk);

	regulator_bulk_disable(S5KHMX_NUM_SUPPLIES, s5khmx->supplies);

	return 0;
}

static int s5khmx_probe(struct i2c_client *client)
{
	struct s5khmx *s5khmx;
	unsigned long freq;
	unsigned int i;
	int ret;

	s5khmx = devm_kzalloc_obj(&client->dev, *s5khmx);
	if (!s5khmx)
		return -ENOMEM;

	s5khmx->dev = &client->dev;
	v4l2_i2c_subdev_init(&s5khmx->sd, client, &s5khmx_subdev_ops);

	s5khmx->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5khmx->regmap))
		return dev_err_probe(s5khmx->dev, PTR_ERR(s5khmx->regmap),
				     "failed to init CCI\n");

	s5khmx->mclk = devm_v4l2_sensor_clk_get(s5khmx->dev, NULL);
	if (IS_ERR(s5khmx->mclk))
		return dev_err_probe(s5khmx->dev, PTR_ERR(s5khmx->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(s5khmx->mclk);
	if (freq != S5KHMX_MCLK_FREQ_19P2MHZ)
		return dev_err_probe(s5khmx->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = s5khmx_check_hwcfg(s5khmx);
	if (ret)
		return dev_err_probe(s5khmx->dev, ret,
				     "failed to check HW configuration\n");

	s5khmx->reset_gpio = devm_gpiod_get_optional(s5khmx->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(s5khmx->reset_gpio))
		return dev_err_probe(s5khmx->dev, PTR_ERR(s5khmx->reset_gpio),
				     "cannot get reset GPIO\n");

	for (i = 0; i < S5KHMX_NUM_SUPPLIES; i++)
		s5khmx->supplies[i].supply = s5khmx_supply_names[i];

	ret = devm_regulator_bulk_get(s5khmx->dev, S5KHMX_NUM_SUPPLIES,
				      s5khmx->supplies);
	if (ret)
		return dev_err_probe(s5khmx->dev, ret,
				     "failed to get supply regulators\n");

	s5khmx->ois_node = of_parse_phandle(s5khmx->dev->of_node, "lens-ois", 0);

	ret = s5khmx_power_on(s5khmx->dev);
	if (ret) {
		of_node_put(s5khmx->ois_node);
		return ret;
	}

	ret = s5khmx_identify_sensor(s5khmx);
	if (ret) {
		dev_err_probe(s5khmx->dev, ret, "failed to find sensor\n");
		goto power_off;
	}

	s5khmx->mode = &s5khmx_supported_modes[0];
	ret = s5khmx_init_controls(s5khmx);
	if (ret) {
		dev_err_probe(s5khmx->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	s5khmx->sd.state_lock = s5khmx->ctrl_handler.lock;
	s5khmx->sd.internal_ops = &s5khmx_internal_ops;
	s5khmx->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5khmx->sd.entity.ops = &s5khmx_subdev_entity_ops;
	s5khmx->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5khmx->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5khmx->sd.entity, 1, &s5khmx->pad);
	if (ret) {
		dev_err_probe(s5khmx->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&s5khmx->sd);
	if (ret < 0) {
		dev_err_probe(s5khmx->dev, ret,
			      "failed to init subdev\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(s5khmx->dev);
	pm_runtime_enable(s5khmx->dev);

	ret = v4l2_async_register_subdev_sensor(&s5khmx->sd);
	if (ret < 0) {
		dev_err_probe(s5khmx->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(s5khmx->dev, 1000);
	pm_runtime_use_autosuspend(s5khmx->dev);
	pm_runtime_idle(s5khmx->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&s5khmx->sd);
	pm_runtime_disable(s5khmx->dev);
	pm_runtime_set_suspended(s5khmx->dev);

media_entity_cleanup:
	media_entity_cleanup(&s5khmx->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(s5khmx->sd.ctrl_handler);

power_off:
	s5khmx_power_off(s5khmx->dev);
	of_node_put(s5khmx->ois_node);

	return ret;
}

static void s5khmx_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5khmx *s5khmx = to_s5khmx(sd);

	s5khmx_ois_put(s5khmx);
	of_node_put(s5khmx->ois_node);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(s5khmx->dev);

	if (!pm_runtime_status_suspended(s5khmx->dev)) {
		s5khmx_power_off(s5khmx->dev);
		pm_runtime_set_suspended(s5khmx->dev);
	}
}

static DEFINE_RUNTIME_DEV_PM_OPS(s5khmx_pm_ops, s5khmx_power_off,
				 s5khmx_power_on, NULL);

static const struct of_device_id s5khmx_of_match[] = {
	{ .compatible = "samsung,s5khmx" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5khmx_of_match);

static struct i2c_driver s5khmx_i2c_driver = {
	.driver = {
		.name = "s5khmx",
		.pm = pm_ptr(&s5khmx_pm_ops),
		.of_match_table = s5khmx_of_match,
	},
	.probe = s5khmx_probe,
	.remove = s5khmx_remove,
};

module_i2c_driver(s5khmx_i2c_driver);

MODULE_AUTHOR("Alexandru Andries <alex.andries.aa@gmail.com>");
MODULE_DESCRIPTION("Samsung S5KHMX 108MP image sensor driver");
MODULE_LICENSE("GPL");
