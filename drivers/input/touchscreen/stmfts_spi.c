// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics FingerTip S (FTS5) SPI touchscreen driver
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define STMFTS_SPI_FIFO_CMD_READALL	0x87
#define STMFTS_SPI_CMD_SCAN_MODE	0xa0
#define STMFTS_SPI_CMD_HW_REG_W		0xfa
#define STMFTS_SPI_CMD_HW_REG_R		0xfb

#define STMFTS_SPI_SCAN_ACTIVE		0x00
#define STMFTS_SPI_SCAN_LOW_POWER	0x01
#define STMFTS_SPI_ACTIVE_MULTI_TOUCH	0x01

#define STMFTS_SPI_EVT_NOEVENT		0x00
#define STMFTS_SPI_EVT_CONTROLLER_READY	0x03
#define STMFTS_SPI_EVT_ENTER_POINT	0x13
#define STMFTS_SPI_EVT_MOTION_POINT	0x23
#define STMFTS_SPI_EVT_LEAVE_POINT	0x33
#define STMFTS_SPI_EVT_ERROR		0xf3

#define STMFTS_SPI_ERR_WATCHDOG		0x06
#define STMFTS_SPI_ERR_ESD		0xf0

#define STMFTS_SPI_TOUCH_FINGER		0x01
#define STMFTS_SPI_TOUCH_GLOVE		0x02
#define STMFTS_SPI_TOUCH_STYLUS		0x03
#define STMFTS_SPI_TOUCH_PALM		0x04

#define STMFTS_SPI_EVENT_SIZE		8
#define STMFTS_SPI_FIFO_DEPTH		32
#define STMFTS_SPI_FIFO_MAX_BYTES	(STMFTS_SPI_EVENT_SIZE * STMFTS_SPI_FIFO_DEPTH)
#define STMFTS_SPI_MAX_SLOTS		10
#define STMFTS_SPI_CS_DELAY_US		10

#define STMFTS_SPI_DEFAULT_MAX_X	10800
#define STMFTS_SPI_DEFAULT_MAX_Y	24000

struct stmfts_spi_ts {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct regulator *vdd;
	struct regulator *avdd;
	struct gpio_desc *reset_gpio;
	struct mutex io_lock; /* Protects SPI transfers and DMA buffers */
	bool super_res;
	bool suspended;

	u8 tx_buf[16] ____cacheline_aligned;
	u8 rx_buf[STMFTS_SPI_FIFO_MAX_BYTES + 8] ____cacheline_aligned;
};

static int stmfts_spi_write(struct stmfts_spi_ts *ts, const u8 *cmd, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->tx_buf,
		.len = len,
		.delay = {
			.value = STMFTS_SPI_CS_DELAY_US,
			.unit = SPI_DELAY_UNIT_USECS,
		},
	};
	struct spi_message msg;
	int ret;

	if (len > sizeof(ts->tx_buf))
		return -EINVAL;

	mutex_lock(&ts->io_lock);
	memcpy(ts->tx_buf, cmd, len);
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(ts->spi, &msg);
	mutex_unlock(&ts->io_lock);

	return ret;
}

static int stmfts_spi_write_read(struct stmfts_spi_ts *ts,
				 const u8 *cmd, size_t cmd_len,
				 u8 *out, size_t out_len, bool dummy_byte)
{
	size_t rx_len = out_len + (dummy_byte ? 1 : 0);
	struct spi_transfer xfers[2] = {
		{
			.tx_buf = ts->tx_buf,
			.len = cmd_len,
		},
		{
			.rx_buf = ts->rx_buf,
			.len = rx_len,
			.delay = {
				.value = STMFTS_SPI_CS_DELAY_US,
				.unit = SPI_DELAY_UNIT_USECS,
			},
		},
	};
	struct spi_message msg;
	int ret;

	if (cmd_len > sizeof(ts->tx_buf) || rx_len > sizeof(ts->rx_buf))
		return -EINVAL;

	mutex_lock(&ts->io_lock);
	memcpy(ts->tx_buf, cmd, cmd_len);
	memset(ts->rx_buf, 0, rx_len);
	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);
	ret = spi_sync(ts->spi, &msg);
	if (!ret)
		memcpy(out, ts->rx_buf + (dummy_byte ? 1 : 0), out_len);
	mutex_unlock(&ts->io_lock);

	return ret;
}

static int stmfts_spi_set_scan_mode(struct stmfts_spi_ts *ts, u8 mode, u8 settings)
{
	u8 cmd[3] = { STMFTS_SPI_CMD_SCAN_MODE, mode, settings };
	size_t len = (mode == STMFTS_SPI_SCAN_LOW_POWER) ? 2 : 3;

	return stmfts_spi_write(ts, cmd, len);
}

static void stmfts_spi_release_all(struct stmfts_spi_ts *ts)
{
	int i;

	for (i = 0; i < STMFTS_SPI_MAX_SLOTS; i++) {
		input_mt_slot(ts->input, i);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	}
	input_mt_report_pointer_emulation(ts->input, false);
	input_sync(ts->input);
}

static int stmfts_spi_system_reset(struct stmfts_spi_ts *ts)
{
	static const u8 spi4_cmd[6] = {
		STMFTS_SPI_CMD_HW_REG_W, 0x20, 0x00, 0x00, 0x2d, 0x02
	};
	static const u8 sw_rst_cmd[6] = {
		STMFTS_SPI_CMD_HW_REG_W, 0x20, 0x00, 0x00, 0x24, 0x81
	};
	static const u8 read_fifo_cmd = STMFTS_SPI_FIFO_CMD_READALL;
	u8 evt[STMFTS_SPI_EVENT_SIZE];
	int attempt, poll, ret;

	for (attempt = 0; attempt < 3; attempt++) {
		if (ts->reset_gpio) {
			gpiod_set_value_cansleep(ts->reset_gpio, 1);
			usleep_range(10000, 12000);
			gpiod_set_value_cansleep(ts->reset_gpio, 0);
		} else {
			stmfts_spi_write(ts, spi4_cmd, sizeof(spi4_cmd));
			usleep_range(1000, 2000);
			stmfts_spi_write(ts, sw_rst_cmd, sizeof(sw_rst_cmd));
		}

		usleep_range(15000, 18000);

		for (poll = 0; poll < 50; poll++) {
			ret = stmfts_spi_write_read(ts, &read_fifo_cmd, 1,
						    evt, sizeof(evt), true);
			if (!ret && evt[0] == STMFTS_SPI_EVT_CONTROLLER_READY)
				return 0;
			usleep_range(10000, 12000);
		}

		/* Ensure 4-wire SPI mode is latched before retrying */
		stmfts_spi_write(ts, spi4_cmd, sizeof(spi4_cmd));
		usleep_range(1000, 2000);
	}

	return -ETIMEDOUT;
}

static int stmfts_spi_read_chip_id(struct stmfts_spi_ts *ts, u8 id[2])
{
	static const u8 cmd[5] = {
		STMFTS_SPI_CMD_HW_REG_R, 0x20, 0x00, 0x00, 0x00
	};

	return stmfts_spi_write_read(ts, cmd, sizeof(cmd), id, 2, true);
}

static int stmfts_spi_start_sensing(struct stmfts_spi_ts *ts)
{
	static const u8 read_fifo_cmd = STMFTS_SPI_FIFO_CMD_READALL;
	u8 evt[STMFTS_SPI_EVENT_SIZE];
	int i, ret;

	ret = stmfts_spi_set_scan_mode(ts, STMFTS_SPI_SCAN_ACTIVE, 0x00);
	if (ret)
		return ret;

	usleep_range(10000, 12000);

	/* Drain any stale status/echo events before enabling active scan */
	for (i = 0; i < STMFTS_SPI_FIFO_DEPTH; i++) {
		if (stmfts_spi_write_read(ts, &read_fifo_cmd, 1,
					  evt, sizeof(evt), true))
			break;
		if (evt[0] == STMFTS_SPI_EVT_NOEVENT || evt[0] == 0xff)
			break;
	}

	return stmfts_spi_set_scan_mode(ts, STMFTS_SPI_SCAN_ACTIVE,
					STMFTS_SPI_ACTIVE_MULTI_TOUCH);
}

static irqreturn_t stmfts_spi_irq(int irq, void *data)
{
	static const u8 read_fifo_cmd = STMFTS_SPI_FIFO_CMD_READALL;
	struct stmfts_spi_ts *ts = data;
	u8 fifo[STMFTS_SPI_FIFO_MAX_BYTES];
	unsigned int remaining, total, i;
	unsigned long slot_dirty = 0;
	bool report_sync = false;
	bool need_recovery = false;
	int ret;

	if (ts->suspended)
		return IRQ_HANDLED;

	ret = stmfts_spi_write_read(ts, &read_fifo_cmd, 1,
				    fifo, STMFTS_SPI_EVENT_SIZE, true);
	if (ret)
		return IRQ_HANDLED;

	remaining = fifo[7] & 0x1f;
	if (remaining > STMFTS_SPI_FIFO_DEPTH - 1)
		remaining = STMFTS_SPI_FIFO_DEPTH - 1;

	if (remaining > 0) {
		ret = stmfts_spi_write_read(ts, &read_fifo_cmd, 1,
					    &fifo[STMFTS_SPI_EVENT_SIZE],
					    remaining * STMFTS_SPI_EVENT_SIZE,
					    true);
		if (ret)
			remaining = 0;
	}

	total = 1 + remaining;
	for (i = 0; i < total; i++) {
		const u8 *evt = &fifo[i * STMFTS_SPI_EVENT_SIZE];
		u8 evt_id = evt[0];
		u8 touch_type, slot;
		unsigned int x, y, major, minor;

		if (evt_id == STMFTS_SPI_EVT_NOEVENT || evt_id == 0xff)
			break;

		if (evt_id == STMFTS_SPI_EVT_ERROR) {
			if (evt[1] == STMFTS_SPI_ERR_ESD ||
			    evt[1] == STMFTS_SPI_ERR_WATCHDOG)
				need_recovery = true;
			continue;
		}

		/*
		 * Pointer events pack the upper 2 bits of touch_major into
		 * evt[0] bits [3:2], so mask them out when matching the ID.
		 */
		switch (evt_id & 0xf3) {
		case STMFTS_SPI_EVT_ENTER_POINT:
		case STMFTS_SPI_EVT_MOTION_POINT:
			touch_type = evt[1] & 0x0f;
			slot = (evt[1] & 0xf0) >> 4;
			if (slot >= STMFTS_SPI_MAX_SLOTS)
				break;

			if (test_bit(slot, &slot_dirty)) {
				input_mt_report_pointer_emulation(ts->input, false);
				input_sync(ts->input);
				slot_dirty = 0;
				report_sync = false;
			}

			/* Reject palm contacts rather than reporting them as finger taps */
			if (touch_type == STMFTS_SPI_TOUCH_PALM) {
				input_mt_slot(ts->input, slot);
				input_mt_report_slot_state(ts->input,
							   MT_TOOL_FINGER, false);
				__set_bit(slot, &slot_dirty);
				report_sync = true;
				break;
			}

			if (touch_type != STMFTS_SPI_TOUCH_FINGER &&
			    touch_type != STMFTS_SPI_TOUCH_GLOVE &&
			    touch_type != STMFTS_SPI_TOUCH_STYLUS)
				break;

			if (ts->super_res) {
				x = get_unaligned_le16(&evt[2]);
				y = get_unaligned_le16(&evt[4]);
			} else {
				x = ((unsigned int)(evt[3] & 0x0f) << 8) | evt[2];
				y = ((unsigned int)evt[4] << 4) | ((evt[3] & 0xf0) >> 4);
			}

			major = ((evt[0] & 0x0c) << 2) | ((evt[6] & 0xf0) >> 4);
			minor = ((evt[7] & 0xc0) >> 2) | (evt[6] & 0x0f);

			input_mt_slot(ts->input, slot);
			input_mt_report_slot_state(ts->input,
						   (touch_type == STMFTS_SPI_TOUCH_STYLUS) ?
						   MT_TOOL_PEN : MT_TOOL_FINGER,
						   true);
			touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, major);
			input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, minor);
			__set_bit(slot, &slot_dirty);
			report_sync = true;
			break;

		case STMFTS_SPI_EVT_LEAVE_POINT:
			if (evt[1] == 0xb5)
				break;
			slot = (evt[1] & 0xf0) >> 4;
			if (slot >= STMFTS_SPI_MAX_SLOTS)
				break;

			if (test_bit(slot, &slot_dirty)) {
				input_mt_report_pointer_emulation(ts->input, false);
				input_sync(ts->input);
				slot_dirty = 0;
				report_sync = false;
			}

			input_mt_slot(ts->input, slot);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
			__set_bit(slot, &slot_dirty);
			report_sync = true;
			break;

		default:
			break;
		}
	}

	if (report_sync) {
		input_mt_report_pointer_emulation(ts->input, false);
		input_sync(ts->input);
	}

	if (need_recovery) {
		dev_warn(&ts->spi->dev, "controller error event, resetting\n");
		stmfts_spi_release_all(ts);
		if (!stmfts_spi_system_reset(ts))
			stmfts_spi_start_sensing(ts);
	}

	return IRQ_HANDLED;
}

static int stmfts_spi_power_on(struct stmfts_spi_ts *ts)
{
	int ret;

	ret = regulator_enable(ts->vdd);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	ret = regulator_enable(ts->avdd);
	if (ret) {
		regulator_disable(ts->vdd);
		return ret;
	}

	/* Allow analog 3.3 V rail and touch matrix baseline to settle cleanly */
	usleep_range(20000, 25000);
	return 0;
}

static void stmfts_spi_power_off(void *data)
{
	struct stmfts_spi_ts *ts = data;

	if (ts->reset_gpio)
		gpiod_set_value_cansleep(ts->reset_gpio, 1);

	regulator_disable(ts->avdd);
	regulator_disable(ts->vdd);
}

static int stmfts_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct stmfts_spi_ts *ts;
	u32 max_x = STMFTS_SPI_DEFAULT_MAX_X;
	u32 max_y = STMFTS_SPI_DEFAULT_MAX_Y;
	u8 chip_id[2] = { 0 };
	int ret;

	if (!spi->irq)
		return dev_err_probe(dev, -EINVAL, "missing IRQ\n");

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "failed to setup SPI\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	ts->super_res = device_property_read_bool(dev, "st,super-resolution");
	mutex_init(&ts->io_lock);
	spi_set_drvdata(spi, ts);

	ts->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ts->vdd))
		return dev_err_probe(dev, PTR_ERR(ts->vdd), "failed to get vdd\n");

	ts->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(ts->avdd))
		return dev_err_probe(dev, PTR_ERR(ts->avdd), "failed to get avdd\n");

	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset GPIO\n");

	ret = stmfts_spi_power_on(ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable regulators\n");

	ret = devm_add_action_or_reset(dev, stmfts_spi_power_off, ts);
	if (ret)
		return ret;

	ret = stmfts_spi_system_reset(ts);
	if (ret) {
		stmfts_spi_read_chip_id(ts, chip_id);
		return dev_err_probe(dev, ret,
				     "controller reset timed out (chip_id=%02x %02x)\n",
				     chip_id[0], chip_id[1]);
	}

	stmfts_spi_read_chip_id(ts, chip_id);

	device_property_read_u32(dev, "touchscreen-size-x", &max_x);
	device_property_read_u32(dev, "touchscreen-size-y", &max_y);

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;

	ts->input->name = "stmfts-spi";
	ts->input->id.bustype = BUS_SPI;

	input_set_capability(ts->input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(ts->input, EV_ABS, ABS_MT_POSITION_Y);
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, max_x - 1, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, max_y - 1, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);

	touchscreen_parse_properties(ts->input, true, &ts->prop);

	ret = input_mt_init_slots(ts->input, STMFTS_SPI_MAX_SLOTS,
				  INPUT_MT_DIRECT);
	if (ret)
		return ret;

	ret = input_register_device(ts->input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input device\n");

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, stmfts_spi_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"stmfts-spi", ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	ret = stmfts_spi_start_sensing(ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to start active scan\n");

	enable_irq(spi->irq);

	dev_info(dev, "STMicroelectronics FTS5 SPI ready (chip_id=%02x %02x, %ux%u)\n",
		 chip_id[0], chip_id[1], ts->prop.max_x + 1, ts->prop.max_y + 1);

	return 0;
}

static int stmfts_spi_suspend(struct device *dev)
{
	struct stmfts_spi_ts *ts = dev_get_drvdata(dev);

	ts->suspended = true;
	disable_irq(ts->spi->irq);
	stmfts_spi_set_scan_mode(ts, STMFTS_SPI_SCAN_ACTIVE, 0x00);
	stmfts_spi_release_all(ts);

	return 0;
}

static int stmfts_spi_resume(struct device *dev)
{
	struct stmfts_spi_ts *ts = dev_get_drvdata(dev);
	int ret;

	ret = stmfts_spi_system_reset(ts);
	if (!ret)
		ret = stmfts_spi_start_sensing(ts);

	ts->suspended = false;
	enable_irq(ts->spi->irq);

	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(stmfts_spi_pm_ops,
				stmfts_spi_suspend, stmfts_spi_resume);

static const struct of_device_id stmfts_spi_of_match[] = {
	{ .compatible = "st,fts-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, stmfts_spi_of_match);

static const struct spi_device_id stmfts_spi_id[] = {
	{ "fts-spi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, stmfts_spi_id);

static struct spi_driver stmfts_spi_driver = {
	.driver = {
		.name = "stmfts-spi",
		.of_match_table = stmfts_spi_of_match,
		.pm = pm_sleep_ptr(&stmfts_spi_pm_ops),
	},
	.id_table = stmfts_spi_id,
	.probe = stmfts_spi_probe,
};
module_spi_driver(stmfts_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics FingerTip S (FTS5) SPI Touchscreen Driver");
MODULE_LICENSE("GPL");
