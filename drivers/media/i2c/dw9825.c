// SPDX-License-Identifier: GPL-2.0
/*
 * Dongwoon Anatech DW9825A closed-loop voice coil lens actuator driver
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define DW9825_MAX_FOCUS_POS	4095
#define DW9825_POS_SHIFT	4
#define DW9825_CTRL_STEPS	64
#define DW9825_CTRL_DELAY_US	1000
#define DW9825_POWER_DELAY_US	10000
#define DW9825_MODE_DELAY_US	3000

#define DW9825_REG_POS		CCI_REG16(0x00)
#define DW9825_REG_CONTROL	CCI_REG8(0x02)
#define DW9825_REG_INIT		CCI_REG8(0x04)

#define DW9825_MODE_ACTIVE	0x00
#define DW9825_MODE_SLEEP	0x20
#define DW9825_MODE_STANDBY	0x40
#define DW9825_INIT_ENABLE	0x01

static const char * const dw9825_supply_names[] = {
	"vdd",
	"vio",
};

struct dw9825_device {
	struct v4l2_subdev sd;
	struct device *dev;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[ARRAY_SIZE(dw9825_supply_names)];

	struct v4l2_ctrl_handler ctrls_vcm;
	struct v4l2_ctrl *focus;
};

static inline struct dw9825_device *sd_to_dw9825(struct v4l2_subdev *sd)
{
	return container_of(sd, struct dw9825_device, sd);
}

static int dw9825_set_position(struct dw9825_device *dw9825, u16 pos)
{
	return cci_write(dw9825->regmap, DW9825_REG_POS,
			 (u64)pos << DW9825_POS_SHIFT, NULL);
}

static int dw9825_power_down(struct dw9825_device *dw9825)
{
	int ret = 0;

	cci_write(dw9825->regmap, DW9825_REG_CONTROL, DW9825_MODE_STANDBY, &ret);
	fsleep(DW9825_CTRL_DELAY_US);
	cci_write(dw9825->regmap, DW9825_REG_CONTROL, DW9825_MODE_SLEEP, &ret);
	fsleep(DW9825_MODE_DELAY_US);
	if (ret)
		dev_err(dw9825->dev, "Failed to enter sleep mode: %d\n", ret);

	return regulator_bulk_disable(ARRAY_SIZE(dw9825_supply_names),
				      dw9825->supplies);
}

static int dw9825_power_up(struct dw9825_device *dw9825)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(dw9825_supply_names),
				    dw9825->supplies);
	if (ret)
		return ret;

	fsleep(DW9825_POWER_DELAY_US);

	cci_write(dw9825->regmap, DW9825_REG_CONTROL, DW9825_MODE_STANDBY, &ret);
	fsleep(DW9825_MODE_DELAY_US);
	cci_write(dw9825->regmap, DW9825_REG_INIT, DW9825_INIT_ENABLE, &ret);
	fsleep(DW9825_MODE_DELAY_US);
	cci_write(dw9825->regmap, DW9825_REG_CONTROL, DW9825_MODE_ACTIVE, &ret);
	fsleep(DW9825_MODE_DELAY_US);

	if (ret) {
		dev_err(dw9825->dev, "Failed to power up actuator: %d\n", ret);
		regulator_bulk_disable(ARRAY_SIZE(dw9825_supply_names),
				       dw9825->supplies);
		return ret;
	}

	return 0;
}

static int dw9825_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dw9825_device *dw9825 = container_of(ctrl->handler,
						    struct dw9825_device,
						    ctrls_vcm);
	int ret;

	if (!pm_runtime_get_if_in_use(dw9825->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_FOCUS_ABSOLUTE:
		ret = dw9825_set_position(dw9825, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(dw9825->dev);

	return ret;
}

static const struct v4l2_ctrl_ops dw9825_ctrl_ops = {
	.s_ctrl = dw9825_set_ctrl,
};

static int dw9825_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int dw9825_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put_autosuspend(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops dw9825_internal_ops = {
	.open = dw9825_open,
	.close = dw9825_close,
};

static const struct v4l2_subdev_ops dw9825_ops = { };

static int dw9825_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct dw9825_device *dw9825 = sd_to_dw9825(sd);
	int ret, val;

	for (val = dw9825->focus->val & ~(DW9825_CTRL_STEPS - 1);
	     val >= 0; val -= DW9825_CTRL_STEPS) {
		ret = dw9825_set_position(dw9825, val);
		if (ret) {
			dev_err_once(dev, "Failed to ramp down lens: %d\n", ret);
			break;
		}
		usleep_range(DW9825_CTRL_DELAY_US, DW9825_CTRL_DELAY_US + 100);
	}

	return dw9825_power_down(dw9825);
}

static int dw9825_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct dw9825_device *dw9825 = sd_to_dw9825(sd);
	int current_focus = dw9825->focus->val;
	int ret, val;

	ret = dw9825_power_up(dw9825);
	if (ret)
		return ret;

	for (val = current_focus % DW9825_CTRL_STEPS;
	     val <= current_focus; val += DW9825_CTRL_STEPS) {
		ret = dw9825_set_position(dw9825, val);
		if (ret) {
			dw9825_power_down(dw9825);
			return ret;
		}
		usleep_range(DW9825_CTRL_DELAY_US, DW9825_CTRL_DELAY_US + 100);
	}

	return 0;
}

static int dw9825_init_controls(struct dw9825_device *dw9825)
{
	struct v4l2_ctrl_handler *hdl = &dw9825->ctrls_vcm;
	int ret;

	v4l2_ctrl_handler_init(hdl, 1);

	dw9825->focus = v4l2_ctrl_new_std(hdl, &dw9825_ctrl_ops,
					  V4L2_CID_FOCUS_ABSOLUTE, 0,
					  DW9825_MAX_FOCUS_POS, 1, 0);
	if (hdl->error) {
		ret = hdl->error;
		dev_err(dw9825->dev, "Failed to init controls: %d\n", ret);
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	dw9825->sd.ctrl_handler = hdl;

	return 0;
}

static int dw9825_probe(struct i2c_client *client)
{
	struct dw9825_device *dw9825;
	unsigned int i;
	int ret;

	dw9825 = devm_kzalloc_obj(&client->dev, *dw9825);
	if (!dw9825)
		return -ENOMEM;

	dw9825->dev = &client->dev;

	dw9825->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(dw9825->regmap))
		return PTR_ERR(dw9825->regmap);

	for (i = 0; i < ARRAY_SIZE(dw9825_supply_names); i++)
		dw9825->supplies[i].supply = dw9825_supply_names[i];

	ret = devm_regulator_bulk_get(&client->dev,
				      ARRAY_SIZE(dw9825_supply_names),
				      dw9825->supplies);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "Failed to get regulators\n");

	v4l2_i2c_subdev_init(&dw9825->sd, client, &dw9825_ops);
	dw9825->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9825->sd.internal_ops = &dw9825_internal_ops;
	dw9825->sd.entity.function = MEDIA_ENT_F_LENS;

	ret = dw9825_init_controls(dw9825);
	if (ret)
		return ret;

	ret = media_entity_pads_init(&dw9825->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_free_ctrl;

	ret = dw9825_power_up(dw9825);
	if (ret)
		goto err_cleanup_media;

	pm_runtime_set_active(&client->dev);
	pm_runtime_get_noresume(&client->dev);
	pm_runtime_enable(&client->dev);

	ret = v4l2_async_register_subdev(&dw9825->sd);
	if (ret < 0)
		goto err_pm_runtime;

	pm_runtime_set_autosuspend_delay(&client->dev, 1000);
	pm_runtime_use_autosuspend(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return 0;

err_pm_runtime:
	pm_runtime_disable(&client->dev);
	pm_runtime_put_noidle(&client->dev);
	dw9825_power_down(dw9825);
err_cleanup_media:
	media_entity_cleanup(&dw9825->sd.entity);
err_free_ctrl:
	v4l2_ctrl_handler_free(&dw9825->ctrls_vcm);

	return ret;
}

static void dw9825_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9825_device *dw9825 = sd_to_dw9825(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&dw9825->ctrls_vcm);
	media_entity_cleanup(&dw9825->sd.entity);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		dw9825_power_down(dw9825);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id dw9825_of_table[] = {
	{ .compatible = "dongwoon,dw9825a" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dw9825_of_table);

static DEFINE_RUNTIME_DEV_PM_OPS(dw9825_pm_ops, dw9825_suspend, dw9825_resume,
				 NULL);

static struct i2c_driver dw9825_i2c_driver = {
	.driver = {
		.name = "dw9825",
		.pm = pm_sleep_ptr(&dw9825_pm_ops),
		.of_match_table = dw9825_of_table,
	},
	.probe = dw9825_probe,
	.remove = dw9825_remove,
};
module_i2c_driver(dw9825_i2c_driver);

MODULE_DESCRIPTION("Dongwoon DW9825A VCM lens actuator driver");
MODULE_LICENSE("GPL");
