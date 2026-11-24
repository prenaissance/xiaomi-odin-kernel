// SPDX-License-Identifier: GPL-2.0
/*
 * ON Semiconductor LC898128 optical image stabilization (OIS) driver
 *
 * Manages runtime power, MCU boot verification, and servo/OIS stabilization
 * mode control using the pre-programmed on-chip flash firmware.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define LC898128_POWER_DELAY_US		15000
#define LC898128_POLL_INTERVAL_US	2000
#define LC898128_POLL_TIMEOUT_US	100000

#define LC898128_REG_HALL_X		CCI_REG32(0x0178)
#define LC898128_REG_HALL_Y		CCI_REG32(0x017c)
#define LC898128_REG_GYRO_COEFF		CCI_REG32(0x080c)
#define LC898128_REG_GYRO_CFG0		CCI_REG32(0x7000)
#define LC898128_REG_GYRO_CFG1		CCI_REG32(0x7001)
#define LC898128_REG_FW_VERSION		CCI_REG32(0x8000)
#define LC898128_REG_IO_ADDR		CCI_REG32(0xc000)
#define LC898128_REG_IO_DATA		CCI_REG32(0xd000)
#define LC898128_REG_SERVO_STATUS	CCI_REG32(0xf010)
#define LC898128_REG_OIS_ENABLE		CCI_REG32(0xf012)
#define LC898128_REG_GYRO_START		CCI_REG32(0xf015)
#define LC898128_REG_GYRO_INIT		CCI_REG32(0xf019)
#define LC898128_REG_CMD_STATUS		CCI_REG16(0xf100)
#define LC898128_REG_CMD_STATUS2	CCI_REG16(0xf102)

#define LC898128_IO_SYSDSP_REMAP	0x00d000ac
#define LC898128_IO_CHIP_MODE		0x00e050d4
#define LC898128_CHIP_MODE_OK		0x0000000a
#define LC898128_GYRO_START_VAL		0x00000102
#define LC898128_GYRO_CFG1_VAL		0x00000024
#define LC898128_GYRO_COEFF_VAL		0x000c1b35

enum lc898128_mode {
	LC898128_MODE_OFF = 0,
	LC898128_MODE_CENTER = 1,
	LC898128_MODE_OIS = 2,
};

static const char * const lc898128_supply_names[] = {
	"vdd",
	"vio",
};

struct lc898128_device {
	struct v4l2_subdev sd;
	struct device *dev;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[ARRAY_SIZE(lc898128_supply_names)];
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *ois_ctrl;
	struct mutex lock; /* protects mode and I2C sequences */
	enum lc898128_mode mode;
	u32 fw_version;
};

static inline struct lc898128_device *sd_to_lc898128(struct v4l2_subdev *sd)
{
	return container_of(sd, struct lc898128_device, sd);
}

static int lc898128_io_read32(struct lc898128_device *lc, u32 io_addr, u64 *val)
{
	int ret;

	ret = cci_write(lc->regmap, LC898128_REG_IO_ADDR, io_addr, NULL);
	if (ret)
		return ret;

	return cci_read(lc->regmap, LC898128_REG_IO_DATA, val, NULL);
}

static int lc898128_poll_reg(struct lc898128_device *lc, u32 reg, u64 expected)
{
	u64 val = 0;
	int ret;

	ret = read_poll_timeout(cci_read, ret,
				ret || val == expected,
				LC898128_POLL_INTERVAL_US,
				LC898128_POLL_TIMEOUT_US, false,
				lc->regmap, reg, &val, NULL);
	if (ret)
		return ret;

	return val == expected ? 0 : -ETIMEDOUT;
}

static int lc898128_poll_io(struct lc898128_device *lc, u32 io_addr,
			    u64 expected)
{
	u64 val = 0;
	int ret;

	ret = read_poll_timeout(lc898128_io_read32, ret,
				ret || val == expected,
				LC898128_POLL_INTERVAL_US,
				LC898128_POLL_TIMEOUT_US, false,
				lc, io_addr, &val);
	if (ret)
		return ret;

	return val == expected ? 0 : -ETIMEDOUT;
}

static int lc898128_apply_mode_locked(struct lc898128_device *lc,
				      enum lc898128_mode mode)
{
	int ret = 0;

	if (mode == LC898128_MODE_OIS) {
		cci_write(lc->regmap, LC898128_REG_GYRO_INIT, 0x0, &ret);
		cci_write(lc->regmap, LC898128_REG_GYRO_START,
			  LC898128_GYRO_START_VAL, &ret);
		if (ret)
			return ret;

		ret = lc898128_poll_reg(lc, LC898128_REG_CMD_STATUS, 0x0);
		if (ret)
			return ret;

		ret = lc898128_poll_reg(lc, LC898128_REG_CMD_STATUS2, 0x0);
		if (ret)
			return ret;

		cci_write(lc->regmap, LC898128_REG_GYRO_CFG0, 0x1, &ret);
		cci_write(lc->regmap, LC898128_REG_GYRO_CFG1,
			  LC898128_GYRO_CFG1_VAL, &ret);
		cci_write(lc->regmap, LC898128_REG_GYRO_COEFF,
			  LC898128_GYRO_COEFF_VAL, &ret);
		cci_write(lc->regmap, LC898128_REG_OIS_ENABLE, 0x1, &ret);
		if (ret)
			return ret;

		return lc898128_poll_reg(lc, LC898128_REG_OIS_ENABLE, 0x1);
	}

	cci_write(lc->regmap, LC898128_REG_OIS_ENABLE, 0x0, &ret);
	if (ret)
		return ret;

	return lc898128_poll_reg(lc, LC898128_REG_CMD_STATUS, 0x0);
}

static int lc898128_power_up_locked(struct lc898128_device *lc)
{
	u64 fw_ver = 0;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(lc898128_supply_names),
				    lc->supplies);
	if (ret)
		return ret;

	fsleep(LC898128_POWER_DELAY_US);

	ret = lc898128_poll_io(lc, LC898128_IO_SYSDSP_REMAP, 0x1);
	if (ret) {
		dev_err(lc->dev, "MCU boot timeout: %d\n", ret);
		goto err_disable;
	}

	ret = lc898128_poll_io(lc, LC898128_IO_CHIP_MODE,
			       LC898128_CHIP_MODE_OK);
	if (ret) {
		dev_err(lc->dev, "Unexpected chip mode: %d\n", ret);
		goto err_disable;
	}

	ret = cci_read(lc->regmap, LC898128_REG_FW_VERSION, &fw_ver, NULL);
	if (ret)
		goto err_disable;

	lc->fw_version = (u32)fw_ver;

	ret = cci_write(lc->regmap, LC898128_REG_GYRO_INIT, 0x1, NULL);
	if (ret)
		goto err_disable;

	if (lc->mode != LC898128_MODE_OFF) {
		ret = lc898128_apply_mode_locked(lc, lc->mode);
		if (ret) {
			dev_err(lc->dev, "Failed to apply mode %d: %d\n",
				lc->mode, ret);
			goto err_disable;
		}
	}

	return 0;

err_disable:
	regulator_bulk_disable(ARRAY_SIZE(lc898128_supply_names), lc->supplies);
	return ret;
}

static int lc898128_power_down_locked(struct lc898128_device *lc)
{
	int ret;

	ret = cci_write(lc->regmap, LC898128_REG_OIS_ENABLE, 0x0, NULL);
	if (!ret)
		lc898128_poll_reg(lc, LC898128_REG_CMD_STATUS, 0x0);

	return regulator_bulk_disable(ARRAY_SIZE(lc898128_supply_names),
				      lc->supplies);
}

static int lc898128_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct lc898128_device *lc = container_of(ctrl->handler,
						  struct lc898128_device,
						  ctrl_handler);
	enum lc898128_mode new_mode;
	int ret = 0;

	if (ctrl->id != V4L2_CID_IMAGE_STABILIZATION)
		return -EINVAL;

	new_mode = ctrl->val ? LC898128_MODE_OIS : LC898128_MODE_CENTER;

	mutex_lock(&lc->lock);
	lc->mode = new_mode;
	if (pm_runtime_get_if_in_use(lc->dev) > 0) {
		ret = lc898128_apply_mode_locked(lc, new_mode);
		pm_runtime_put_autosuspend(lc->dev);
	}
	mutex_unlock(&lc->lock);

	return ret;
}

static const struct v4l2_ctrl_ops lc898128_ctrl_ops = {
	.s_ctrl = lc898128_set_ctrl,
};

static int lc898128_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int lc898128_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_mark_last_busy(sd->dev);
	pm_runtime_put_autosuspend(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops lc898128_internal_ops = {
	.open = lc898128_open,
	.close = lc898128_close,
};

static const struct v4l2_subdev_ops lc898128_ops = { };

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct lc898128_device *lc = sd_to_lc898128(sd);
	const char *str;

	mutex_lock(&lc->lock);
	switch (lc->mode) {
	case LC898128_MODE_OIS:
		str = "on";
		break;
	case LC898128_MODE_CENTER:
		str = "center";
		break;
	default:
		str = "off";
		break;
	}
	mutex_unlock(&lc->lock);

	return sysfs_emit(buf, "%s\n", str);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct lc898128_device *lc = sd_to_lc898128(sd);
	enum lc898128_mode new_mode;
	int ret = 0;

	if (sysfs_streq(buf, "on"))
		new_mode = LC898128_MODE_OIS;
	else if (sysfs_streq(buf, "center"))
		new_mode = LC898128_MODE_CENTER;
	else if (sysfs_streq(buf, "off"))
		new_mode = LC898128_MODE_OFF;
	else
		return -EINVAL;

	mutex_lock(&lc->lock);
	lc->mode = new_mode;
	if (pm_runtime_get_if_in_use(dev) > 0) {
		ret = lc898128_apply_mode_locked(lc, new_mode);
		pm_runtime_put_autosuspend(dev);
	}
	mutex_unlock(&lc->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(mode);

static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct lc898128_device *lc = sd_to_lc898128(sd);
	u64 ois_en = 0, servo = 0, hall_x = 0, hall_y = 0;
	bool active;
	int ret = 0;

	mutex_lock(&lc->lock);
	active = pm_runtime_get_if_in_use(dev) > 0;
	if (active) {
		cci_read(lc->regmap, LC898128_REG_OIS_ENABLE, &ois_en, &ret);
		cci_read(lc->regmap, LC898128_REG_SERVO_STATUS, &servo, &ret);
		cci_read(lc->regmap, LC898128_REG_HALL_X, &hall_x, &ret);
		cci_read(lc->regmap, LC898128_REG_HALL_Y, &hall_y, &ret);
		pm_runtime_put_autosuspend(dev);
	}
	mutex_unlock(&lc->lock);

	if (!active)
		return sysfs_emit(buf, "suspended fw=0x%08x mode=%d\n",
				  lc->fw_version, lc->mode);

	if (ret)
		return ret;

	return sysfs_emit(buf,
			  "active fw=0x%08x ois=%llu servo=0x%llx hall_x=0x%08llx hall_y=0x%08llx\n",
			  lc->fw_version, ois_en, servo, hall_x, hall_y);
}
static DEVICE_ATTR_RO(status);

static struct attribute *lc898128_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_status.attr,
	NULL
};
ATTRIBUTE_GROUPS(lc898128);

static int lc898128_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct lc898128_device *lc = sd_to_lc898128(sd);
	int ret;

	mutex_lock(&lc->lock);
	ret = lc898128_power_down_locked(lc);
	mutex_unlock(&lc->lock);

	return ret;
}

static int lc898128_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct lc898128_device *lc = sd_to_lc898128(sd);
	int ret;

	mutex_lock(&lc->lock);
	ret = lc898128_power_up_locked(lc);
	mutex_unlock(&lc->lock);

	return ret;
}

static int lc898128_probe(struct i2c_client *client)
{
	struct lc898128_device *lc;
	unsigned int i;
	int ret;

	lc = devm_kzalloc(&client->dev, sizeof(*lc), GFP_KERNEL);
	if (!lc)
		return -ENOMEM;

	lc->dev = &client->dev;
	lc->mode = LC898128_MODE_OIS;
	mutex_init(&lc->lock);

	lc->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(lc->regmap))
		return PTR_ERR(lc->regmap);

	for (i = 0; i < ARRAY_SIZE(lc898128_supply_names); i++)
		lc->supplies[i].supply = lc898128_supply_names[i];

	ret = devm_regulator_bulk_get(&client->dev,
				      ARRAY_SIZE(lc898128_supply_names),
				      lc->supplies);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "Failed to get regulators\n");

	v4l2_i2c_subdev_init(&lc->sd, client, &lc898128_ops);
	lc->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	lc->sd.internal_ops = &lc898128_internal_ops;
	lc->sd.entity.function = MEDIA_ENT_F_LENS;

	v4l2_ctrl_handler_init(&lc->ctrl_handler, 1);
	lc->ois_ctrl = v4l2_ctrl_new_std(&lc->ctrl_handler,
					 &lc898128_ctrl_ops,
					 V4L2_CID_IMAGE_STABILIZATION,
					 0, 1, 1, 1);
	if (lc->ctrl_handler.error) {
		ret = lc->ctrl_handler.error;
		v4l2_ctrl_handler_free(&lc->ctrl_handler);
		return ret;
	}
	lc->sd.ctrl_handler = &lc->ctrl_handler;

	ret = media_entity_pads_init(&lc->sd.entity, 0, NULL);
	if (ret)
		goto err_free_ctrl;

	/* Probe hardware once to verify factory flash MCU boots, then suspend */
	mutex_lock(&lc->lock);
	lc->mode = LC898128_MODE_OFF;
	ret = lc898128_power_up_locked(lc);
	if (!ret)
		lc898128_power_down_locked(lc);
	lc->mode = LC898128_MODE_OIS;
	mutex_unlock(&lc->lock);
	if (ret) {
		dev_err(&client->dev, "LC898128 probe failed: %d\n", ret);
		goto err_cleanup_media;
	}

	dev_info(&client->dev, "ON Semi LC898128 OIS detected (fw=0x%08x)\n",
		 lc->fw_version);

	pm_runtime_set_suspended(&client->dev);
	pm_runtime_set_autosuspend_delay(&client->dev, 1000);
	pm_runtime_use_autosuspend(&client->dev);
	pm_runtime_enable(&client->dev);

	ret = v4l2_async_register_subdev(&lc->sd);
	if (ret)
		goto err_pm_runtime;

	return 0;

err_pm_runtime:
	pm_runtime_disable(&client->dev);
	pm_runtime_dont_use_autosuspend(&client->dev);
err_cleanup_media:
	media_entity_cleanup(&lc->sd.entity);
err_free_ctrl:
	v4l2_ctrl_handler_free(&lc->ctrl_handler);
	return ret;
}

static void lc898128_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lc898128_device *lc = sd_to_lc898128(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&lc->ctrl_handler);
	media_entity_cleanup(&lc->sd.entity);

	pm_runtime_disable(&client->dev);
	pm_runtime_dont_use_autosuspend(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev)) {
		mutex_lock(&lc->lock);
		lc898128_power_down_locked(lc);
		mutex_unlock(&lc->lock);
	}
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id lc898128_of_table[] = {
	{ .compatible = "onnn,lc898128" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lc898128_of_table);

static DEFINE_RUNTIME_DEV_PM_OPS(lc898128_pm_ops,
				 lc898128_runtime_suspend,
				 lc898128_runtime_resume, NULL);

static struct i2c_driver lc898128_i2c_driver = {
	.driver = {
		.name = "lc898128",
		.pm = pm_ptr(&lc898128_pm_ops),
		.of_match_table = lc898128_of_table,
		.dev_groups = lc898128_groups,
	},
	.probe = lc898128_probe,
	.remove = lc898128_remove,
};
module_i2c_driver(lc898128_i2c_driver);

MODULE_DESCRIPTION("ON Semiconductor LC898128 OIS driver");
MODULE_LICENSE("GPL");
