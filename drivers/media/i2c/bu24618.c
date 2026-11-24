// SPDX-License-Identifier: GPL-2.0
/*
 * Rohm BU24618 optical image stabilization (OIS) driver
 *
 * Downloads RAM firmware (.prog, .coeff, .mem) on runtime power-up,
 * initializes the integrated SPI gyroscope bridge, and engages active
 * closed-loop optical image stabilization.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define BU24618_POWER_DELAY_US		15000
#define BU24618_POLL_INTERVAL_US	2000
#define BU24618_POLL_TIMEOUT_US		500000
#define BU24618_FW_CHUNK_BYTES		8

#define BU24618_ADDR_PROG		0x0000
#define BU24618_ADDR_COEFF		0x1b40
#define BU24618_ADDR_CALIB		0x1dc0
#define BU24618_ADDR_MEM		0x4000

#define BU24618_EEPROM_I2C_ADDR		0x50
#define BU24618_EEPROM_CALIB_OFF	0x220b
#define BU24618_CALIB_WORDS		19
#define BU24618_CALIB_BYTES		(BU24618_CALIB_WORDS * 2)

#define BU24618_REG_OIS_CTRL		CCI_REG8(0x6020)
#define BU24618_REG_OIS_MODE		CCI_REG8(0x6021)
#define BU24618_REG_GYRO_SPI_CTRL	CCI_REG8(0x6023)
#define BU24618_REG_STATUS		CCI_REG8(0x6024)
#define BU24618_REG_OIS_CFG		CCI_REG8(0x6025)
#define BU24618_REG_GYRO_IF_CTRL	CCI_REG8(0x602a)
#define BU24618_REG_GYRO_REG_ADDR	CCI_REG8(0x602c)
#define BU24618_REG_GYRO_REG_DATA	CCI_REG8(0x602d)
#define BU24618_REG_HALL_X		CCI_REG16(0x6040)
#define BU24618_REG_HALL_Y		CCI_REG16(0x6046)
#define BU24618_REG_INIT_CFG		CCI_REG8(0x614f)
#define BU24618_REG_SCENE_CFG		CCI_REG8(0x6157)
#define BU24618_REG_GYRO_SCALE_0	CCI_REG8(0x617d)
#define BU24618_REG_GYRO_SCALE_1	CCI_REG16(0x617e)
#define BU24618_REG_FILTER_CFG_0	CCI_REG8(0x618c)
#define BU24618_REG_FILTER_CFG_1	CCI_REG8(0x618d)
#define BU24618_REG_DSP_START		CCI_REG8(0xf006)
#define BU24618_REG_FW_VERSION		CCI_REG32(0xf008)
#define BU24618_REG_DL_MODE		CCI_REG8(0xf010)

#define BU24618_STATUS_READY		0x01
#define BU24618_CTRL_CENTER		0x01
#define BU24618_CTRL_OIS_ON		0x02

#define BU24618_DEFAULT_FW_PREFIX	"qcom/sm8350/xiaomi/odin/odin_wide_bu24618_ois"

enum bu24618_mode {
	BU24618_MODE_OFF = 0,
	BU24618_MODE_CENTER = 1,
	BU24618_MODE_OIS = 2,
};

static const char * const bu24618_supply_names[] = {
	"vdd",
	"vaf",
	"vio",
};

struct bu24618_device {
	struct v4l2_subdev sd;
	struct i2c_client *client;
	struct device *dev;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[ARRAY_SIZE(bu24618_supply_names)];
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *ois_ctrl;
	struct mutex lock; /* protects mode, firmware cache, and I2C sequences */
	const char *fw_prefix;
	const struct firmware *fw_prog;
	const struct firmware *fw_coeff;
	const struct firmware *fw_mem;
	enum bu24618_mode mode;
	u32 fw_checksum;
	u32 fw_version;
	u8 calib_data[BU24618_CALIB_BYTES];
	bool calib_valid;
};

static inline struct bu24618_device *sd_to_bu24618(struct v4l2_subdev *sd)
{
	return container_of(sd, struct bu24618_device, sd);
}

static int bu24618_poll_ready(struct bu24618_device *bu)
{
	u64 val = 0;
	int ret;

	ret = read_poll_timeout(cci_read, ret,
				ret || val == BU24618_STATUS_READY,
				BU24618_POLL_INTERVAL_US,
				BU24618_POLL_TIMEOUT_US, false,
				bu->regmap, BU24618_REG_STATUS, &val, NULL);
	if (ret)
		return ret;

	return val == BU24618_STATUS_READY ? 0 : -ETIMEDOUT;
}

static int bu24618_write_and_poll(struct bu24618_device *bu, u32 reg, u64 val)
{
	int ret;

	ret = cci_write(bu->regmap, reg, val, NULL);
	if (ret)
		return ret;

	return bu24618_poll_ready(bu);
}

static int bu24618_write_fw_blob(struct bu24618_device *bu, u16 base_addr,
				 const u8 *data, size_t len)
{
	u8 buf[2 + BU24618_FW_CHUNK_BYTES];
	size_t off = 0;
	int ret;

	while (off < len) {
		size_t chunk = min_t(size_t, BU24618_FW_CHUNK_BYTES, len - off);
		u16 addr = base_addr + off;

		buf[0] = addr >> 8;
		buf[1] = addr & 0xff;
		memcpy(&buf[2], data + off, chunk);

		ret = i2c_master_send(bu->client, buf, 2 + chunk);
		if (ret < 0)
			return ret;
		if (ret != 2 + chunk)
			return -EIO;

		off += chunk;
	}

	return 0;
}

static u32 bu24618_sum_bytes(const u8 *data, size_t len)
{
	u32 sum = 0;
	size_t i;

	for (i = 0; i < len; i++)
		sum += data[i];

	return sum;
}

static int bu24618_load_fw_cache_locked(struct bu24618_device *bu)
{
	char path[128];
	int ret;

	if (bu->fw_prog && bu->fw_coeff)
		return 0;

	snprintf(path, sizeof(path), "%s.prog", bu->fw_prefix);
	ret = request_firmware(&bu->fw_prog, path, bu->dev);
	if (ret) {
		dev_err(bu->dev, "Failed to load %s: %d\n", path, ret);
		return ret;
	}

	snprintf(path, sizeof(path), "%s.coeff", bu->fw_prefix);
	ret = request_firmware(&bu->fw_coeff, path, bu->dev);
	if (ret) {
		dev_err(bu->dev, "Failed to load %s: %d\n", path, ret);
		release_firmware(bu->fw_prog);
		bu->fw_prog = NULL;
		return ret;
	}

	snprintf(path, sizeof(path), "%s.mem", bu->fw_prefix);
	ret = request_firmware(&bu->fw_mem, path, bu->dev);
	if (ret) {
		dev_dbg(bu->dev, "Optional %s not found (%d)\n", path, ret);
		bu->fw_mem = NULL;
	}

	bu->fw_checksum = bu24618_sum_bytes(bu->fw_prog->data,
					    bu->fw_prog->size) +
			  bu24618_sum_bytes(bu->fw_coeff->data,
					    bu->fw_coeff->size);
	if (bu->fw_mem)
		bu->fw_checksum += bu24618_sum_bytes(bu->fw_mem->data,
						     bu->fw_mem->size);

	return 0;
}

static int bu24618_apply_mode_locked(struct bu24618_device *bu,
				     enum bu24618_mode mode)
{
	int ret;

	if (mode == BU24618_MODE_OIS) {
		ret = bu24618_write_and_poll(bu, BU24618_REG_OIS_MODE, 0x6b);
		if (ret)
			return ret;
		ret = bu24618_write_and_poll(bu, BU24618_REG_FILTER_CFG_1, 0x02);
		if (ret)
			return ret;
		ret = bu24618_write_and_poll(bu, BU24618_REG_FILTER_CFG_0, 0x0f);
		if (ret)
			return ret;
		ret = bu24618_write_and_poll(bu, BU24618_REG_SCENE_CFG, 0x09);
		if (ret)
			return ret;
		ret = bu24618_write_and_poll(bu, BU24618_REG_OIS_CFG, 0xe0);
		if (ret)
			return ret;

		return bu24618_write_and_poll(bu, BU24618_REG_OIS_CTRL,
					      BU24618_CTRL_OIS_ON);
	}

	return bu24618_write_and_poll(bu, BU24618_REG_OIS_CTRL,
				      BU24618_CTRL_CENTER);
}

static int bu24618_read_eeprom_calib_locked(struct bu24618_device *bu)
{
	size_t off = 0;

	if (bu->calib_valid)
		return 0;

	while (off < BU24618_CALIB_BYTES) {
		size_t chunk = min_t(size_t, BU24618_FW_CHUNK_BYTES,
				     BU24618_CALIB_BYTES - off);
		u16 reg = BU24618_EEPROM_CALIB_OFF + off;
		u8 reg_buf[2] = { reg >> 8, reg & 0xff };
		struct i2c_msg msgs[2] = {
			{
				.addr = BU24618_EEPROM_I2C_ADDR,
				.flags = 0,
				.len = sizeof(reg_buf),
				.buf = reg_buf,
			},
			{
				.addr = BU24618_EEPROM_I2C_ADDR,
				.flags = I2C_M_RD,
				.len = chunk,
				.buf = &bu->calib_data[off],
			},
		};
		int ret;

		ret = i2c_transfer(bu->client->adapter, msgs, 2);
		if (ret != 2) {
			dev_err(bu->dev,
				"Failed to read OIS EEPROM calibration @ 0x%04x: %d\n",
				reg, ret);
			return ret < 0 ? ret : -EIO;
		}
		off += chunk;
	}

	bu->calib_valid = true;
	return 0;
}

static int bu24618_write_calib_locked(struct bu24618_device *bu)
{
	unsigned int i;
	int ret;

	for (i = 0; i < BU24618_CALIB_WORDS; i++) {
		u16 val = ((u16)bu->calib_data[i * 2] << 8) |
			  bu->calib_data[i * 2 + 1];

		ret = cci_write(bu->regmap,
				CCI_REG16(BU24618_ADDR_CALIB + i * 2),
				val, NULL);
		if (ret)
			return ret;
	}

	return 0;
}

static int bu24618_power_up_locked(struct bu24618_device *bu)
{
	u64 fw_ver = 0;
	int ret;

	ret = bu24618_load_fw_cache_locked(bu);
	if (ret)
		return ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(bu24618_supply_names),
				    bu->supplies);
	if (ret)
		return ret;

	fsleep(BU24618_POWER_DELAY_US);

	/* Pre-init: wait ready and halt DSP for RAM firmware download */
	ret = bu24618_poll_ready(bu);
	if (ret) {
		dev_err(bu->dev, "Pre-init ready timeout: %d\n", ret);
		goto err_disable;
	}

	ret = cci_write(bu->regmap, BU24618_REG_DL_MODE, 0x00, NULL);
	if (ret)
		goto err_disable;
	fsleep(200);

	/* Download RAM firmware sections */
	ret = bu24618_write_fw_blob(bu, BU24618_ADDR_PROG,
				    bu->fw_prog->data, bu->fw_prog->size);
	if (ret) {
		dev_err(bu->dev, "Prog download failed: %d\n", ret);
		goto err_disable;
	}

	ret = bu24618_write_fw_blob(bu, BU24618_ADDR_COEFF,
				    bu->fw_coeff->data, bu->fw_coeff->size);
	if (ret) {
		dev_err(bu->dev, "Coeff download failed: %d\n", ret);
		goto err_disable;
	}

	if (bu->fw_mem) {
		ret = bu24618_write_fw_blob(bu, BU24618_ADDR_MEM,
					    bu->fw_mem->data,
					    bu->fw_mem->size);
		if (ret) {
			dev_err(bu->dev, "Mem download failed: %d\n", ret);
			goto err_disable;
		}
	}

	/* Verify hardware download checksum before applying calibration */
	ret = read_poll_timeout(cci_read, ret,
				ret || fw_ver == bu->fw_checksum,
				BU24618_POLL_INTERVAL_US,
				BU24618_POLL_TIMEOUT_US, false,
				bu->regmap, BU24618_REG_FW_VERSION,
				&fw_ver, NULL);
	if (ret) {
		dev_err(bu->dev, "FW checksum mismatch: got=0x%08llx exp=0x%08x\n",
			fw_ver, bu->fw_checksum);
		goto err_disable;
	}
	bu->fw_version = (u32)fw_ver;

	/* Load factory Hall/gyro calibration from module EEPROM into 0x1dc0..0x1de4 */
	ret = bu24618_read_eeprom_calib_locked(bu);
	if (ret)
		goto err_disable;

	ret = bu24618_write_calib_locked(bu);
	if (ret) {
		dev_err(bu->dev, "Failed to write calibration data: %d\n", ret);
		goto err_disable;
	}

	/* Start DSP */
	ret = cci_write(bu->regmap, BU24618_REG_DSP_START, 0x00, NULL);
	if (ret)
		goto err_disable;
	fsleep(1000);

	ret = bu24618_poll_ready(bu);
	if (ret) {
		dev_err(bu->dev, "Post-download ready timeout: %d\n", ret);
		goto err_disable;
	}

	/* Post-init: enable servo and configure integrated SPI gyroscope */
	ret = bu24618_write_and_poll(bu, BU24618_REG_INIT_CFG, 0x01);
	if (ret)
		goto err_disable;
	ret = bu24618_write_and_poll(bu, BU24618_REG_OIS_CTRL,
				     BU24618_CTRL_CENTER);
	if (ret)
		goto err_disable;
	ret = bu24618_write_and_poll(bu, BU24618_REG_GYRO_SCALE_0, 0x5e);
	if (ret)
		goto err_disable;
	ret = bu24618_write_and_poll(bu, BU24618_REG_GYRO_SCALE_1, 0x0048);
	if (ret)
		goto err_disable;
	ret = bu24618_write_and_poll(bu, BU24618_REG_GYRO_SPI_CTRL, 0x02);
	if (ret)
		goto err_disable;

	cci_write(bu->regmap, BU24618_REG_GYRO_REG_ADDR, 0x70, &ret);
	cci_write(bu->regmap, BU24618_REG_GYRO_REG_DATA, 0x21, &ret);
	cci_write(bu->regmap, BU24618_REG_GYRO_REG_ADDR, 0x71, &ret);
	cci_write(bu->regmap, BU24618_REG_GYRO_REG_DATA, 0x00, &ret);
	cci_write(bu->regmap, BU24618_REG_GYRO_REG_ADDR, 0x72, &ret);
	cci_write(bu->regmap, BU24618_REG_GYRO_REG_DATA, 0x00, &ret);
	if (ret)
		goto err_disable;

	ret = bu24618_poll_ready(bu);
	if (ret)
		goto err_disable;
	ret = bu24618_write_and_poll(bu, BU24618_REG_GYRO_IF_CTRL, 0x00);
	if (ret)
		goto err_disable;
	ret = bu24618_write_and_poll(bu, BU24618_REG_GYRO_SPI_CTRL, 0x00);
	if (ret)
		goto err_disable;

	if (bu->mode != BU24618_MODE_OFF) {
		ret = bu24618_apply_mode_locked(bu, bu->mode);
		if (ret) {
			dev_err(bu->dev, "Failed to apply mode %d: %d\n",
				bu->mode, ret);
			goto err_disable;
		}
	}

	dev_info_once(bu->dev, "Rohm BU24618 OIS firmware active (fw=0x%08x)\n",
		      bu->fw_version);

	return 0;

err_disable:
	regulator_bulk_disable(ARRAY_SIZE(bu24618_supply_names), bu->supplies);
	return ret;
}

static int bu24618_power_down_locked(struct bu24618_device *bu)
{
	bu24618_write_and_poll(bu, BU24618_REG_OIS_CTRL, BU24618_CTRL_CENTER);

	return regulator_bulk_disable(ARRAY_SIZE(bu24618_supply_names),
				      bu->supplies);
}

static int bu24618_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct bu24618_device *bu = container_of(ctrl->handler,
						 struct bu24618_device,
						 ctrl_handler);
	enum bu24618_mode new_mode;
	int ret = 0;

	if (ctrl->id != V4L2_CID_IMAGE_STABILIZATION)
		return -EINVAL;

	new_mode = ctrl->val ? BU24618_MODE_OIS : BU24618_MODE_CENTER;

	mutex_lock(&bu->lock);
	bu->mode = new_mode;
	if (pm_runtime_get_if_in_use(bu->dev) > 0) {
		ret = bu24618_apply_mode_locked(bu, new_mode);
		pm_runtime_put_autosuspend(bu->dev);
	}
	mutex_unlock(&bu->lock);

	return ret;
}

static const struct v4l2_ctrl_ops bu24618_ctrl_ops = {
	.s_ctrl = bu24618_set_ctrl,
};

static int bu24618_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int bu24618_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_mark_last_busy(sd->dev);
	pm_runtime_put_autosuspend(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops bu24618_internal_ops = {
	.open = bu24618_open,
	.close = bu24618_close,
};

static const struct v4l2_subdev_ops bu24618_ops = { };

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct bu24618_device *bu = sd_to_bu24618(sd);
	const char *str;

	mutex_lock(&bu->lock);
	switch (bu->mode) {
	case BU24618_MODE_OIS:
		str = "on";
		break;
	case BU24618_MODE_CENTER:
		str = "center";
		break;
	default:
		str = "off";
		break;
	}
	mutex_unlock(&bu->lock);

	return sysfs_emit(buf, "%s\n", str);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct bu24618_device *bu = sd_to_bu24618(sd);
	enum bu24618_mode new_mode;
	int ret = 0;

	if (sysfs_streq(buf, "on"))
		new_mode = BU24618_MODE_OIS;
	else if (sysfs_streq(buf, "center"))
		new_mode = BU24618_MODE_CENTER;
	else if (sysfs_streq(buf, "off"))
		new_mode = BU24618_MODE_OFF;
	else
		return -EINVAL;

	mutex_lock(&bu->lock);
	bu->mode = new_mode;
	if (pm_runtime_get_if_in_use(dev) > 0) {
		ret = bu24618_apply_mode_locked(bu, new_mode);
		pm_runtime_put_autosuspend(dev);
	}
	mutex_unlock(&bu->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(mode);

static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct bu24618_device *bu = sd_to_bu24618(sd);
	u64 ois_ctrl = 0, status = 0, hall_x = 0, hall_y = 0;
	bool active;
	int ret = 0;

	mutex_lock(&bu->lock);
	active = pm_runtime_get_if_in_use(dev) > 0;
	if (active) {
		cci_read(bu->regmap, BU24618_REG_OIS_CTRL, &ois_ctrl, &ret);
		cci_read(bu->regmap, BU24618_REG_STATUS, &status, &ret);
		cci_read(bu->regmap, BU24618_REG_HALL_X, &hall_x, &ret);
		cci_read(bu->regmap, BU24618_REG_HALL_Y, &hall_y, &ret);
		pm_runtime_put_autosuspend(dev);
	}
	mutex_unlock(&bu->lock);

	if (!active)
		return sysfs_emit(buf, "suspended fw=0x%08x mode=%d\n",
				  bu->fw_version, bu->mode);

	if (ret)
		return ret;

	return sysfs_emit(buf,
			  "active fw=0x%08x ctrl=0x%llx ready=0x%llx hall_x=0x%04llx hall_y=0x%04llx\n",
			  bu->fw_version, ois_ctrl, status, hall_x, hall_y);
}
static DEVICE_ATTR_RO(status);

static struct attribute *bu24618_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_status.attr,
	NULL
};
ATTRIBUTE_GROUPS(bu24618);

static int bu24618_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct bu24618_device *bu = sd_to_bu24618(sd);
	int ret;

	mutex_lock(&bu->lock);
	ret = bu24618_power_down_locked(bu);
	mutex_unlock(&bu->lock);

	return ret;
}

static int bu24618_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct bu24618_device *bu = sd_to_bu24618(sd);
	int ret;

	mutex_lock(&bu->lock);
	ret = bu24618_power_up_locked(bu);
	mutex_unlock(&bu->lock);

	return ret;
}

static int bu24618_probe(struct i2c_client *client)
{
	struct bu24618_device *bu;
	unsigned int i;
	int ret;

	bu = devm_kzalloc(&client->dev, sizeof(*bu), GFP_KERNEL);
	if (!bu)
		return -ENOMEM;

	bu->client = client;
	bu->dev = &client->dev;
	bu->mode = BU24618_MODE_OIS;
	mutex_init(&bu->lock);

	if (device_property_read_string(&client->dev, "firmware-name",
					&bu->fw_prefix))
		bu->fw_prefix = BU24618_DEFAULT_FW_PREFIX;

	bu->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(bu->regmap))
		return PTR_ERR(bu->regmap);

	for (i = 0; i < ARRAY_SIZE(bu24618_supply_names); i++)
		bu->supplies[i].supply = bu24618_supply_names[i];

	ret = devm_regulator_bulk_get(&client->dev,
				      ARRAY_SIZE(bu24618_supply_names),
				      bu->supplies);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "Failed to get regulators\n");

	v4l2_i2c_subdev_init(&bu->sd, client, &bu24618_ops);
	bu->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	bu->sd.internal_ops = &bu24618_internal_ops;
	bu->sd.entity.function = MEDIA_ENT_F_LENS;

	v4l2_ctrl_handler_init(&bu->ctrl_handler, 1);
	bu->ois_ctrl = v4l2_ctrl_new_std(&bu->ctrl_handler,
					 &bu24618_ctrl_ops,
					 V4L2_CID_IMAGE_STABILIZATION,
					 0, 1, 1, 1);
	if (bu->ctrl_handler.error) {
		ret = bu->ctrl_handler.error;
		v4l2_ctrl_handler_free(&bu->ctrl_handler);
		return ret;
	}
	bu->sd.ctrl_handler = &bu->ctrl_handler;

	ret = media_entity_pads_init(&bu->sd.entity, 0, NULL);
	if (ret)
		goto err_free_ctrl;

	pm_runtime_set_suspended(&client->dev);
	pm_runtime_set_autosuspend_delay(&client->dev, 1000);
	pm_runtime_use_autosuspend(&client->dev);
	pm_runtime_enable(&client->dev);

	ret = v4l2_async_register_subdev(&bu->sd);
	if (ret)
		goto err_pm_runtime;

	dev_info(&client->dev, "Rohm BU24618 OIS registered (fw=%s)\n",
		 bu->fw_prefix);

	return 0;

err_pm_runtime:
	pm_runtime_disable(&client->dev);
	pm_runtime_dont_use_autosuspend(&client->dev);
	media_entity_cleanup(&bu->sd.entity);
err_free_ctrl:
	v4l2_ctrl_handler_free(&bu->ctrl_handler);
	return ret;
}

static void bu24618_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bu24618_device *bu = sd_to_bu24618(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&bu->ctrl_handler);
	media_entity_cleanup(&bu->sd.entity);

	pm_runtime_disable(&client->dev);
	pm_runtime_dont_use_autosuspend(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev)) {
		mutex_lock(&bu->lock);
		bu24618_power_down_locked(bu);
		mutex_unlock(&bu->lock);
	}
	pm_runtime_set_suspended(&client->dev);

	release_firmware(bu->fw_mem);
	release_firmware(bu->fw_coeff);
	release_firmware(bu->fw_prog);
}

static const struct of_device_id bu24618_of_table[] = {
	{ .compatible = "rohm,bu24618" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bu24618_of_table);

static DEFINE_RUNTIME_DEV_PM_OPS(bu24618_pm_ops,
				 bu24618_runtime_suspend,
				 bu24618_runtime_resume, NULL);

static struct i2c_driver bu24618_i2c_driver = {
	.driver = {
		.name = "bu24618",
		.pm = pm_ptr(&bu24618_pm_ops),
		.of_match_table = bu24618_of_table,
		.dev_groups = bu24618_groups,
	},
	.probe = bu24618_probe,
	.remove = bu24618_remove,
};
module_i2c_driver(bu24618_i2c_driver);

MODULE_DESCRIPTION("Rohm BU24618 optical image stabilization driver");
MODULE_LICENSE("GPL");
