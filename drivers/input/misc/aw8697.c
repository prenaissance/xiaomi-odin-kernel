// SPDX-License-Identifier: GPL-2.0-only
/*
 * Awinic AW8697 LRA Haptic Driver
 *
 * Based on aw86927.c and vendor Awinic AW8697 reference code:
 *   Copyright (c) 2018-2021 AWINIC Technology CO., LTD
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#define AW8697_REG_ID			0x00
#define AW8697_CHIPID			0x97
#define AW8697_SOFTRST			0xaa

#define AW8697_REG_SYSST		0x01

#define AW8697_REG_SYSINT		0x02
#define AW8697_SYSINT_BSTERRI		BIT(7)
#define AW8697_SYSINT_OVI		BIT(6)
#define AW8697_SYSINT_UVLI		BIT(5)
#define AW8697_SYSINT_FF_AEI		BIT(4)
#define AW8697_SYSINT_FF_AFI		BIT(3)
#define AW8697_SYSINT_OCDI		BIT(2)
#define AW8697_SYSINT_OTI		BIT(1)
#define AW8697_SYSINT_DONEI		BIT(0)

#define AW8697_REG_SYSINTM		0x03
#define AW8697_SYSINTM_BSTERROFF	BIT(7)
#define AW8697_SYSINTM_OVOFF		BIT(6)
#define AW8697_SYSINTM_UVLOOFF		BIT(5)
#define AW8697_SYSINTM_FF_AEOFF		BIT(4)
#define AW8697_SYSINTM_FF_AFOFF		BIT(3)
#define AW8697_SYSINTM_OCDOFF		BIT(2)
#define AW8697_SYSINTM_OTOFF		BIT(1)
#define AW8697_SYSINTM_DONEOFF		BIT(0)

#define AW8697_REG_SYSCTRL		0x04
#define AW8697_SYSCTRL_WAVDAT_MODE_MASK	GENMASK(7, 6)
#define AW8697_SYSCTRL_WAVDAT_MODE_2X	0
#define AW8697_SYSCTRL_RAMINIT_EN	BIT(5)
#define AW8697_SYSCTRL_PLAY_MODE_MASK	GENMASK(3, 2)
#define AW8697_SYSCTRL_PLAY_MODE_RAM	0
#define AW8697_SYSCTRL_BST_MODE_MASK	BIT(1)
#define AW8697_SYSCTRL_BST_MODE_BYPASS	0
#define AW8697_SYSCTRL_BST_MODE_BOOST	1
#define AW8697_SYSCTRL_STANDBY		BIT(0)

#define AW8697_REG_GO			0x05
#define AW8697_GO_ENABLE		BIT(0)

#define AW8697_REG_WAVSEQ1		0x07
#define AW8697_REG_WAVSEQ2		0x08
#define AW8697_REG_WAVLOOP1		0x0f
#define AW8697_WAVLOOP1_SEQ1_MASK	GENMASK(7, 4)
#define AW8697_WAVLOOP_INFINITE		0x0f

#define AW8697_REG_DBGCTRL		0x20
#define AW8697_DBGCTRL_INT_MODE_MASK	BIT(2)
#define AW8697_DBGCTRL_INT_MODE_EDGE	BIT(2)

#define AW8697_REG_BASE_ADDRH		0x21
#define AW8697_REG_BASE_ADDRL		0x22
#define AW8697_REG_FIFO_AEH		0x23
#define AW8697_REG_FIFO_AEL		0x24
#define AW8697_REG_FIFO_AFH		0x25
#define AW8697_REG_FIFO_AFL		0x26

#define AW8697_REG_PWMPRC		0x2d
#define AW8697_PWMPRC_PRC_EN		BIT(7)

#define AW8697_REG_PWMDBG		0x2e
#define AW8697_PWMDBG_PWM_MODE_MASK	GENMASK(6, 5)
#define AW8697_PWMDBG_PWM_24K		2

#define AW8697_REG_BSTDBG1		0x31
#define AW8697_REG_BSTDBG2		0x32
#define AW8697_REG_BSTDBG3		0x33

#define AW8697_REG_BSTCFG		0x34
#define AW8697_BSTCFG_PEAKCUR_MASK	GENMASK(2, 0)
#define AW8697_BSTCFG_PEAKCUR_3P5A	5

#define AW8697_REG_ANADBG		0x35
#define AW8697_ANADBG_IOC_MASK		GENMASK(3, 2)
#define AW8697_ANADBG_IOC_4P65A		3

#define AW8697_REG_DATDBG		0x39

#define AW8697_REG_BSTDBG4		0x3a
#define AW8697_BSTDBG4_BSTVOL_MASK	GENMASK(5, 1)
#define AW8697_BSTDBG4_BST_8000MV	0x10

#define AW8697_REG_PRLVL		0x3e
#define AW8697_PRLVL_PR_EN		BIT(7)

#define AW8697_REG_RAMADDRH		0x40
#define AW8697_REG_RAMADDRL		0x41
#define AW8697_REG_RAMDATA		0x42

#define AW8697_REG_GLB_STATE		0x46
#define AW8697_GLB_STATE_MASK		GENMASK(3, 0)
#define AW8697_GLB_STATE_STANDBY	0

#define AW8697_REG_BST_AUTO		0x47
#define AW8697_BST_AUTO_AUTOSW		BIT(2)

#define AW8697_REG_TSET			0x4d
#define AW8697_REG_R_SPARE		0x5d

#define AW8697_REG_DETCTRL		0x5f
#define AW8697_DETCTRL_PROTECT_NO_ACT	BIT(5)
#define AW8697_DETCTRL_VBAT_GO		BIT(1)
#define AW8697_DETCTRL_DIAG_GO		BIT(0)

#define AW8697_REG_ADCTEST		0x66
#define AW8697_ADCTEST_VBAT_HW_COMP	BIT(6)

#define AW8697_REG_BEMF_VTHH_H		0x74
#define AW8697_REG_BEMF_VTHH_L		0x75
#define AW8697_REG_BEMF_VTHL_H		0x76
#define AW8697_REG_BEMF_VTHL_L		0x77

#define AW8697_RAM_BASE_ADDR		0x0800
#define AW8697_RAM_VERSION		0x85

struct aw8697_sram_header {
	u8 version;
	__be16 start_addr;
	__be16 end_addr;
} __packed;

/*
 * 72-sample two's-complement sine wave: round(84 * sin(2 * pi * i / 72)).
 * At 24 kHz PWM with 2x interpolation (12 kHz SRAM sample rate), 72 samples
 * produce a 166.7 Hz continuous drive matching typical X-axis LRA resonance.
 */
static const u8 aw8697_sine_waveform[] = {
	0x00, 0x07, 0x0f, 0x16, 0x1d, 0x23, 0x2a, 0x30, 0x36, 0x3b, 0x40, 0x45,
	0x49, 0x4c, 0x4f, 0x51, 0x53, 0x54, 0x54, 0x54, 0x53, 0x51, 0x4f, 0x4c,
	0x49, 0x45, 0x40, 0x3b, 0x36, 0x30, 0x2a, 0x23, 0x1d, 0x16, 0x0f, 0x07,
	0x00, 0xf9, 0xf1, 0xea, 0xe3, 0xdd, 0xd6, 0xd0, 0xca, 0xc5, 0xc0, 0xbb,
	0xb7, 0xb4, 0xb1, 0xaf, 0xad, 0xac, 0xac, 0xac, 0xad, 0xaf, 0xb1, 0xb4,
	0xb7, 0xbb, 0xc0, 0xc5, 0xca, 0xd0, 0xd6, 0xdd, 0xe3, 0xea, 0xf1, 0xf9,
};

static const struct aw8697_sram_header aw8697_sram_hdr = {
	.version = AW8697_RAM_VERSION,
	.start_addr = cpu_to_be16(AW8697_RAM_BASE_ADDR +
				  sizeof(struct aw8697_sram_header)),
	.end_addr = cpu_to_be16(AW8697_RAM_BASE_ADDR +
				sizeof(struct aw8697_sram_header) +
				ARRAY_SIZE(aw8697_sine_waveform) - 1),
};

struct aw8697_data {
	struct work_struct play_work;
	struct device *dev;
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct mutex lock; /* Serializes I2C playback and stop sequences */
	u16 level;
};

static const struct regmap_config aw8697_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
	.max_register = 0x7f,
};

static void aw8697_hw_reset(struct aw8697_data *haptics)
{
	gpiod_set_value_cansleep(haptics->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(haptics->reset_gpio, 0);
	usleep_range(5000, 6000);
}

static int aw8697_enter_standby(struct aw8697_data *haptics)
{
	int err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_SYSINTM,
				 AW8697_SYSINTM_UVLOOFF, AW8697_SYSINTM_UVLOOFF);
	if (err)
		return err;

	return regmap_update_bits(haptics->regmap, AW8697_REG_SYSCTRL,
				  AW8697_SYSCTRL_STANDBY, AW8697_SYSCTRL_STANDBY);
}

static int aw8697_wait_standby(struct aw8697_data *haptics)
{
	unsigned int val;

	return regmap_read_poll_timeout(haptics->regmap, AW8697_REG_GLB_STATE, val,
					FIELD_GET(AW8697_GLB_STATE_MASK, val) ==
						AW8697_GLB_STATE_STANDBY,
					2000, 200000);
}

static int aw8697_stop(struct aw8697_data *haptics)
{
	unsigned int state;
	int err;

	err = regmap_read(haptics->regmap, AW8697_REG_GLB_STATE, &state);
	if (!err && FIELD_GET(AW8697_GLB_STATE_MASK, state) ==
	    AW8697_GLB_STATE_STANDBY)
		return aw8697_enter_standby(haptics);

	err = regmap_update_bits(haptics->regmap, AW8697_REG_GO,
				 AW8697_GO_ENABLE, 0);
	if (err)
		return err;

	aw8697_wait_standby(haptics);

	return aw8697_enter_standby(haptics);
}

static int aw8697_play_sine(struct aw8697_data *haptics, u16 level)
{
	unsigned int dummy;
	u8 gain;
	int err;

	err = aw8697_stop(haptics);
	if (err)
		return err;

	gain = clamp_t(u32, ((u32)level * 0x80) / 0xffff, 0x18, 0x80);

	err = regmap_write(haptics->regmap, AW8697_REG_WAVSEQ1, 1);
	if (err)
		return err;

	err = regmap_write(haptics->regmap, AW8697_REG_WAVSEQ2, 0);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_WAVLOOP1,
				 AW8697_WAVLOOP1_SEQ1_MASK,
				 FIELD_PREP(AW8697_WAVLOOP1_SEQ1_MASK,
					    AW8697_WAVLOOP_INFINITE));
	if (err)
		return err;

	err = regmap_write(haptics->regmap, AW8697_REG_DATDBG, gain);
	if (err)
		return err;

	/* Enter active RAM mode with boost converter enabled */
	err = regmap_update_bits(haptics->regmap, AW8697_REG_SYSCTRL,
				 AW8697_SYSCTRL_PLAY_MODE_MASK |
				 AW8697_SYSCTRL_BST_MODE_MASK |
				 AW8697_SYSCTRL_STANDBY,
				 FIELD_PREP(AW8697_SYSCTRL_PLAY_MODE_MASK,
					    AW8697_SYSCTRL_PLAY_MODE_RAM) |
				 FIELD_PREP(AW8697_SYSCTRL_BST_MODE_MASK,
					    AW8697_SYSCTRL_BST_MODE_BOOST));
	if (err)
		return err;

	regmap_read(haptics->regmap, AW8697_REG_SYSINT, &dummy);

	err = regmap_update_bits(haptics->regmap, AW8697_REG_SYSINTM,
				 AW8697_SYSINTM_UVLOOFF, 0);
	if (err)
		return err;

	return regmap_update_bits(haptics->regmap, AW8697_REG_GO,
				  AW8697_GO_ENABLE, AW8697_GO_ENABLE);
}

static void aw8697_play_work(struct work_struct *work)
{
	struct aw8697_data *haptics =
		container_of(work, struct aw8697_data, play_work);
	u16 level;
	int err;

	mutex_lock(&haptics->lock);
	level = READ_ONCE(haptics->level);
	if (level)
		err = aw8697_play_sine(haptics, level);
	else
		err = aw8697_stop(haptics);
	mutex_unlock(&haptics->lock);

	if (err)
		dev_err(haptics->dev, "Failed to update playback state: %d\n", err);
}

static int aw8697_haptics_play(struct input_dev *dev, void *data,
			       struct ff_effect *effect)
{
	struct aw8697_data *haptics = input_get_drvdata(dev);
	u16 level;

	level = effect->u.rumble.strong_magnitude;
	if (!level)
		level = effect->u.rumble.weak_magnitude;

	if (READ_ONCE(haptics->level) == level)
		return 0;

	WRITE_ONCE(haptics->level, level);
	queue_work(system_highpri_wq, &haptics->play_work);

	return 0;
}

static void aw8697_close(struct input_dev *input)
{
	struct aw8697_data *haptics = input_get_drvdata(input);
	int err;

	cancel_work_sync(&haptics->play_work);

	mutex_lock(&haptics->lock);
	WRITE_ONCE(haptics->level, 0);
	err = aw8697_stop(haptics);
	mutex_unlock(&haptics->lock);

	if (err)
		dev_err(haptics->dev, "Failed to stop haptics on close: %d\n", err);
}

static int aw8697_offset_calibration(struct aw8697_data *haptics)
{
	unsigned int val;
	int err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_SYSCTRL,
				 AW8697_SYSCTRL_RAMINIT_EN,
				 AW8697_SYSCTRL_RAMINIT_EN);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_DETCTRL,
				 AW8697_DETCTRL_DIAG_GO,
				 AW8697_DETCTRL_DIAG_GO);
	if (err)
		goto out_raminit_off;

	err = regmap_read_poll_timeout(haptics->regmap, AW8697_REG_DETCTRL, val,
				       !(val & AW8697_DETCTRL_DIAG_GO),
				       1000, 100000);

out_raminit_off:
	regmap_update_bits(haptics->regmap, AW8697_REG_SYSCTRL,
			   AW8697_SYSCTRL_RAMINIT_EN, 0);
	return err;
}

static int aw8697_haptic_init(struct aw8697_data *haptics)
{
	int err;

	err = aw8697_enter_standby(haptics);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_SYSCTRL,
				 AW8697_SYSCTRL_WAVDAT_MODE_MASK,
				 FIELD_PREP(AW8697_SYSCTRL_WAVDAT_MODE_MASK,
					    AW8697_SYSCTRL_WAVDAT_MODE_2X));
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_PWMDBG,
				 AW8697_PWMDBG_PWM_MODE_MASK,
				 FIELD_PREP(AW8697_PWMDBG_PWM_MODE_MASK,
					    AW8697_PWMDBG_PWM_24K));
	if (err)
		return err;

	err = regmap_write(haptics->regmap, AW8697_REG_BSTDBG1, 0x30);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_REG_BSTDBG2, 0xeb);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_REG_BSTDBG3, 0xd4);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_REG_TSET, 0x12);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_REG_R_SPARE, 0x68);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_ANADBG,
				 AW8697_ANADBG_IOC_MASK,
				 FIELD_PREP(AW8697_ANADBG_IOC_MASK,
					    AW8697_ANADBG_IOC_4P65A));
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_BSTCFG,
				 AW8697_BSTCFG_PEAKCUR_MASK,
				 FIELD_PREP(AW8697_BSTCFG_PEAKCUR_MASK,
					    AW8697_BSTCFG_PEAKCUR_3P5A));
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_BSTDBG4,
				 AW8697_BSTDBG4_BSTVOL_MASK,
				 FIELD_PREP(AW8697_BSTDBG4_BSTVOL_MASK,
					    AW8697_BSTDBG4_BST_8000MV));
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_DETCTRL,
				 AW8697_DETCTRL_PROTECT_NO_ACT,
				 AW8697_DETCTRL_PROTECT_NO_ACT);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_PWMPRC,
				 AW8697_PWMPRC_PRC_EN, 0);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_PRLVL,
				 AW8697_PRLVL_PR_EN, 0);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_BST_AUTO,
				 AW8697_BST_AUTO_AUTOSW, 0);
	if (err)
		return err;

	err = aw8697_offset_calibration(haptics);
	if (err)
		return err;

	/*
	 * Disable hardware VBAT gain compensation: REG_VBAT is not actively
	 * sampled here (leaving uninitialized low ADC values that clip/overdrive
	 * the waveform), and the 8.0 V boost converter already regulates drive.
	 */
	err = regmap_update_bits(haptics->regmap, AW8697_REG_ADCTEST,
				 AW8697_ADCTEST_VBAT_HW_COMP, 0);
	if (err)
		return err;

	err = regmap_write(haptics->regmap, AW8697_REG_BEMF_VTHH_H, 0x00);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_REG_BEMF_VTHH_L, 0x08);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_REG_BEMF_VTHL_H, 0x03);
	if (err)
		return err;

	return regmap_write(haptics->regmap, AW8697_REG_BEMF_VTHL_L, 0xf8);
}

static int aw8697_ram_init(struct aw8697_data *haptics)
{
	const u8 *hdr = (const u8 *)&aw8697_sram_hdr;
	u16 base = AW8697_RAM_BASE_ADDR;
	u16 ae = base >> 2;
	u16 af = base - ae;
	size_t i;
	int err;

	err = regmap_update_bits(haptics->regmap, AW8697_REG_SYSCTRL,
				 AW8697_SYSCTRL_RAMINIT_EN,
				 AW8697_SYSCTRL_RAMINIT_EN);
	if (err)
		return err;

	err = regmap_write(haptics->regmap, AW8697_REG_BASE_ADDRH, base >> 8);
	if (err)
		goto out_raminit_off;
	err = regmap_write(haptics->regmap, AW8697_REG_BASE_ADDRL, base & 0xff);
	if (err)
		goto out_raminit_off;

	err = regmap_write(haptics->regmap, AW8697_REG_FIFO_AEH, ae >> 8);
	if (err)
		goto out_raminit_off;
	err = regmap_write(haptics->regmap, AW8697_REG_FIFO_AEL, ae & 0xff);
	if (err)
		goto out_raminit_off;
	err = regmap_write(haptics->regmap, AW8697_REG_FIFO_AFH, af >> 8);
	if (err)
		goto out_raminit_off;
	err = regmap_write(haptics->regmap, AW8697_REG_FIFO_AFL, af & 0xff);
	if (err)
		goto out_raminit_off;

	err = regmap_write(haptics->regmap, AW8697_REG_RAMADDRH, base >> 8);
	if (err)
		goto out_raminit_off;
	err = regmap_write(haptics->regmap, AW8697_REG_RAMADDRL, base & 0xff);
	if (err)
		goto out_raminit_off;

	/*
	 * AW8697 increments its internal SRAM pointer on single-byte writes to
	 * RAMDATA, whereas multi-byte I2C writes advance the register address.
	 */
	for (i = 0; i < sizeof(aw8697_sram_hdr); i++) {
		err = regmap_write(haptics->regmap, AW8697_REG_RAMDATA, hdr[i]);
		if (err)
			goto out_raminit_off;
	}

	for (i = 0; i < ARRAY_SIZE(aw8697_sine_waveform); i++) {
		err = regmap_write(haptics->regmap, AW8697_REG_RAMDATA,
				   aw8697_sine_waveform[i]);
		if (err)
			goto out_raminit_off;
	}

out_raminit_off:
	regmap_update_bits(haptics->regmap, AW8697_REG_SYSCTRL,
			   AW8697_SYSCTRL_RAMINIT_EN, 0);
	return err;
}

static irqreturn_t aw8697_irq(int irq, void *data)
{
	struct aw8697_data *haptics = data;
	unsigned int val;
	int err;

	err = regmap_read(haptics->regmap, AW8697_REG_SYSINT, &val);
	if (err)
		return IRQ_NONE;

	if (val & AW8697_SYSINT_OVI)
		dev_err(haptics->dev, "Over-voltage protection triggered\n");
	if (val & AW8697_SYSINT_UVLI)
		dev_warn(haptics->dev, "Under-voltage lock-out triggered\n");
	if (val & AW8697_SYSINT_OCDI)
		dev_err(haptics->dev, "Over-current protection triggered\n");
	if (val & AW8697_SYSINT_OTI)
		dev_err(haptics->dev, "Over-temperature protection triggered\n");

	return IRQ_HANDLED;
}

static int aw8697_probe(struct i2c_client *client)
{
	struct aw8697_data *haptics;
	unsigned int chip_id, dummy;
	int err;

	haptics = devm_kzalloc(&client->dev, sizeof(*haptics), GFP_KERNEL);
	if (!haptics)
		return -ENOMEM;

	haptics->dev = &client->dev;
	haptics->client = client;
	mutex_init(&haptics->lock);
	INIT_WORK(&haptics->play_work, aw8697_play_work);

	i2c_set_clientdata(client, haptics);

	haptics->regmap = devm_regmap_init_i2c(client, &aw8697_regmap_config);
	if (IS_ERR(haptics->regmap))
		return dev_err_probe(haptics->dev, PTR_ERR(haptics->regmap),
				     "Failed to initialize regmap\n");

	haptics->reset_gpio = devm_gpiod_get(haptics->dev, "reset",
					     GPIOD_OUT_HIGH);
	if (IS_ERR(haptics->reset_gpio))
		return dev_err_probe(haptics->dev, PTR_ERR(haptics->reset_gpio),
				     "Failed to acquire reset GPIO\n");

	aw8697_hw_reset(haptics);

	err = regmap_read(haptics->regmap, AW8697_REG_ID, &chip_id);
	if (err)
		return dev_err_probe(haptics->dev, err,
				     "Failed to read chip ID\n");

	if (chip_id != AW8697_CHIPID)
		return dev_err_probe(haptics->dev, -ENODEV,
				     "Unexpected chip ID 0x%02x\n", chip_id);

	err = regmap_write(haptics->regmap, AW8697_REG_ID, AW8697_SOFTRST);
	if (err)
		return dev_err_probe(haptics->dev, err,
				     "Failed software reset\n");

	usleep_range(2000, 3000);

	regmap_read(haptics->regmap, AW8697_REG_SYSINT, &dummy);

	err = regmap_update_bits(haptics->regmap, AW8697_REG_DBGCTRL,
				 AW8697_DBGCTRL_INT_MODE_MASK,
				 AW8697_DBGCTRL_INT_MODE_EDGE);
	if (err)
		return dev_err_probe(haptics->dev, err,
				     "Failed to configure IRQ edge mode\n");

	err = regmap_write(haptics->regmap, AW8697_REG_SYSINTM,
			   AW8697_SYSINTM_BSTERROFF |
			   AW8697_SYSINTM_UVLOOFF |
			   AW8697_SYSINTM_FF_AEOFF |
			   AW8697_SYSINTM_FF_AFOFF |
			   AW8697_SYSINTM_DONEOFF);
	if (err)
		return dev_err_probe(haptics->dev, err,
				     "Failed to configure IRQ mask\n");

	if (client->irq > 0) {
		err = devm_request_threaded_irq(haptics->dev, client->irq, NULL,
						aw8697_irq,
						IRQF_ONESHOT,
						"aw8697-haptics", haptics);
		if (err)
			return dev_err_probe(haptics->dev, err,
					     "Failed to request IRQ\n");
	}

	err = aw8697_haptic_init(haptics);
	if (err)
		return dev_err_probe(haptics->dev, err,
				     "Failed to initialize haptic registers\n");

	err = aw8697_ram_init(haptics);
	if (err)
		return dev_err_probe(haptics->dev, err,
				     "Failed to initialize haptic SRAM\n");

	haptics->input_dev = devm_input_allocate_device(haptics->dev);
	if (!haptics->input_dev)
		return -ENOMEM;

	haptics->input_dev->name = "aw8697-haptics";
	haptics->input_dev->close = aw8697_close;

	input_set_drvdata(haptics->input_dev, haptics);
	input_set_capability(haptics->input_dev, EV_FF, FF_RUMBLE);

	err = input_ff_create_memless(haptics->input_dev, NULL,
				      aw8697_haptics_play);
	if (err)
		return dev_err_probe(haptics->dev, err,
				     "Failed to create FF memless device\n");

	err = input_register_device(haptics->input_dev);
	if (err)
		return dev_err_probe(haptics->dev, err,
				     "Failed to register input device\n");

	return 0;
}

static int aw8697_suspend(struct device *dev)
{
	struct aw8697_data *haptics = dev_get_drvdata(dev);
	int err;

	cancel_work_sync(&haptics->play_work);

	mutex_lock(&haptics->lock);
	WRITE_ONCE(haptics->level, 0);
	err = aw8697_stop(haptics);
	mutex_unlock(&haptics->lock);

	return err;
}

static int aw8697_resume(struct device *dev)
{
	struct aw8697_data *haptics = dev_get_drvdata(dev);
	unsigned int base_h = 0;
	int err;

	mutex_lock(&haptics->lock);
	err = regmap_read(haptics->regmap, AW8697_REG_BASE_ADDRH, &base_h);
	if (!err && base_h != (AW8697_RAM_BASE_ADDR >> 8)) {
		dev_info(dev, "SRAM config lost across suspend, reinitializing\n");
		err = aw8697_haptic_init(haptics);
		if (!err)
			err = aw8697_ram_init(haptics);
	}
	mutex_unlock(&haptics->lock);

	return err;
}

static DEFINE_SIMPLE_DEV_PM_OPS(aw8697_pm_ops, aw8697_suspend, aw8697_resume);

static const struct of_device_id aw8697_of_id[] = {
	{ .compatible = "awinic,aw8697" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw8697_of_id);

static struct i2c_driver aw8697_driver = {
	.driver = {
		.name = "aw8697-haptics",
		.of_match_table = aw8697_of_id,
		.pm = pm_sleep_ptr(&aw8697_pm_ops),
	},
	.probe = aw8697_probe,
};
module_i2c_driver(aw8697_driver);

MODULE_DESCRIPTION("Awinic AW8697 LRA Haptic Driver");
MODULE_LICENSE("GPL");
