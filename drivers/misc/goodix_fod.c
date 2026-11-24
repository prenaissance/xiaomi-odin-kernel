// SPDX-License-Identifier: GPL-2.0-only
/*
 * Goodix optical in-display fingerprint platform driver for Xiaomi MIX 4 (odin)
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netlink.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <net/sock.h>

#define GF_IOC_MAGIC			'g'
#define GF_IOC_INIT			_IOR(GF_IOC_MAGIC, 0, u8)
#define GF_IOC_EXIT			_IO(GF_IOC_MAGIC, 1)
#define GF_IOC_RESET			_IO(GF_IOC_MAGIC, 2)
#define GF_IOC_ENABLE_IRQ		_IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ		_IO(GF_IOC_MAGIC, 4)
#define GF_IOC_ENABLE_SPI_CLK		_IOW(GF_IOC_MAGIC, 5, u32)
#define GF_IOC_DISABLE_SPI_CLK		_IO(GF_IOC_MAGIC, 6)
#define GF_IOC_ENABLE_POWER		_IO(GF_IOC_MAGIC, 7)
#define GF_IOC_DISABLE_POWER		_IO(GF_IOC_MAGIC, 8)
#define GF_IOC_ENTER_SLEEP_MODE		_IO(GF_IOC_MAGIC, 10)
#define GF_IOC_GET_FW_INFO		_IOR(GF_IOC_MAGIC, 11, u8)
#define GF_IOC_REMOVE			_IO(GF_IOC_MAGIC, 12)

#define GF_NETLINK_PROTO		25
#define GF_NET_EVENT_IRQ		1

struct goodix_fod {
	struct device *dev;
	struct miscdevice misc;
	struct regulator *vdd;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *irq_gpio;
	struct sock *nl_sk;
	struct mutex lock;	/* serializes ioctl/sysfs GPIO and power ops */
	wait_queue_head_t irq_wq;
	atomic_t irq_count;
	int irq;
	u32 nl_pid;
	bool irq_enabled;
	bool power_enabled;
};

static struct goodix_fod *g_gf_dev;

static void goodix_fod_send_nl(struct goodix_fod *gf, u8 event)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	u32 pid;

	pid = READ_ONCE(gf->nl_pid);
	if (!gf->nl_sk || !pid)
		return;

	skb = nlmsg_new(sizeof(event), GFP_KERNEL);
	if (!skb)
		return;

	nlh = nlmsg_put(skb, 0, 0, NLMSG_DONE, sizeof(event), 0);
	if (!nlh) {
		kfree_skb(skb);
		return;
	}

	NETLINK_CB(skb).dst_group = 0;
	memcpy(nlmsg_data(nlh), &event, sizeof(event));
	nlmsg_unicast(gf->nl_sk, skb, pid);
}

static void goodix_fod_nl_recv(struct sk_buff *skb)
{
	struct nlmsghdr *nlh;

	if (!g_gf_dev || skb->len < nlmsg_total_size(0))
		return;

	nlh = nlmsg_hdr(skb);
	WRITE_ONCE(g_gf_dev->nl_pid, nlh->nlmsg_pid);
}

static void goodix_fod_hw_reset(struct goodix_fod *gf, unsigned int delay_ms)
{
	/* reset-gpios is GPIO_ACTIVE_LOW: 1 pulls LOW (assert), 0 pulls HIGH */
	gpiod_set_value_cansleep(gf->reset_gpio, 1);
	usleep_range(3000, 4000);
	gpiod_set_value_cansleep(gf->reset_gpio, 0);
	if (delay_ms)
		msleep(delay_ms);
}

static void goodix_fod_enable_irq(struct goodix_fod *gf)
{
	if (!gf->irq_enabled) {
		enable_irq(gf->irq);
		gf->irq_enabled = true;
	}
}

static void goodix_fod_disable_irq(struct goodix_fod *gf)
{
	if (gf->irq_enabled) {
		disable_irq(gf->irq);
		gf->irq_enabled = false;
	}
}

static irqreturn_t goodix_fod_irq_thread(int irq, void *data)
{
	struct goodix_fod *gf = data;

	atomic_inc(&gf->irq_count);
	wake_up_interruptible(&gf->irq_wq);
	goodix_fod_send_nl(gf, GF_NET_EVENT_IRQ);

	return IRQ_HANDLED;
}

static long goodix_fod_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct goodix_fod *gf = container_of(filp->private_data,
					     struct goodix_fod, misc);
	u8 nl_route = GF_NETLINK_PROTO;
	int ret = 0;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENOTTY;

	mutex_lock(&gf->lock);
	switch (cmd) {
	case GF_IOC_INIT:
		if (copy_to_user((void __user *)arg, &nl_route,
				 sizeof(nl_route)))
			ret = -EFAULT;
		break;
	case GF_IOC_EXIT:
		WRITE_ONCE(gf->nl_pid, 0);
		goodix_fod_disable_irq(gf);
		break;
	case GF_IOC_RESET:
		goodix_fod_hw_reset(gf, 10);
		break;
	case GF_IOC_ENABLE_IRQ:
		goodix_fod_enable_irq(gf);
		break;
	case GF_IOC_DISABLE_IRQ:
		goodix_fod_disable_irq(gf);
		break;
	case GF_IOC_ENABLE_POWER:
		if (!gf->power_enabled) {
			ret = regulator_enable(gf->vdd);
			if (!ret) {
				gf->power_enabled = true;
				usleep_range(10000, 11000);
			}
		}
		break;
	case GF_IOC_DISABLE_POWER:
		if (gf->power_enabled) {
			regulator_disable(gf->vdd);
			gf->power_enabled = false;
		}
		break;
	case GF_IOC_ENABLE_SPI_CLK:
	case GF_IOC_DISABLE_SPI_CLK:
	case GF_IOC_ENTER_SLEEP_MODE:
	case GF_IOC_REMOVE:
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	mutex_unlock(&gf->lock);

	return ret;
}

static __poll_t goodix_fod_poll(struct file *filp, poll_table *wait)
{
	struct goodix_fod *gf = container_of(filp->private_data,
					     struct goodix_fod, misc);

	poll_wait(filp, &gf->irq_wq, wait);
	if (atomic_read(&gf->irq_count) > 0)
		return EPOLLIN | EPOLLRDNORM;

	return 0;
}

static ssize_t goodix_fod_read(struct file *filp, char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct goodix_fod *gf = container_of(filp->private_data,
					     struct goodix_fod, misc);
	u32 val;

	if (count < sizeof(val))
		return -EINVAL;

	if (wait_event_interruptible(gf->irq_wq,
				     atomic_read(&gf->irq_count) > 0))
		return -ERESTARTSYS;

	val = atomic_xchg(&gf->irq_count, 0);
	if (copy_to_user(buf, &val, sizeof(val)))
		return -EFAULT;

	return sizeof(val);
}

static const struct file_operations goodix_fod_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = goodix_fod_ioctl,
	.poll = goodix_fod_poll,
	.read = goodix_fod_read,
};

static ssize_t irq_count_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct goodix_fod *gf = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", atomic_read(&gf->irq_count));
}
static DEVICE_ATTR_RO(irq_count);

static ssize_t irq_state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct goodix_fod *gf = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", gpiod_get_value_cansleep(gf->irq_gpio));
}
static DEVICE_ATTR_RO(irq_state);

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct goodix_fod *gf = dev_get_drvdata(dev);

	mutex_lock(&gf->lock);
	goodix_fod_hw_reset(gf, 10);
	mutex_unlock(&gf->lock);

	return count;
}
static DEVICE_ATTR_WO(reset);

static ssize_t irq_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct goodix_fod *gf = dev_get_drvdata(dev);
	bool enabled;

	mutex_lock(&gf->lock);
	enabled = gf->irq_enabled;
	mutex_unlock(&gf->lock);

	return sysfs_emit(buf, "%d\n", enabled ? 1 : 0);
}

static ssize_t irq_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct goodix_fod *gf = dev_get_drvdata(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&gf->lock);
	if (enable)
		goodix_fod_enable_irq(gf);
	else
		goodix_fod_disable_irq(gf);
	mutex_unlock(&gf->lock);

	return count;
}
static DEVICE_ATTR_RW(irq_enable);

static struct attribute *goodix_fod_attrs[] = {
	&dev_attr_irq_count.attr,
	&dev_attr_irq_state.attr,
	&dev_attr_irq_enable.attr,
	&dev_attr_reset.attr,
	NULL
};
ATTRIBUTE_GROUPS(goodix_fod);

static int goodix_fod_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct netlink_kernel_cfg nl_cfg = {
		.input = goodix_fod_nl_recv,
	};
	struct goodix_fod *gf;
	int ret;

	gf = devm_kzalloc(dev, sizeof(*gf), GFP_KERNEL);
	if (!gf)
		return -ENOMEM;

	gf->dev = dev;
	mutex_init(&gf->lock);
	init_waitqueue_head(&gf->irq_wq);
	platform_set_drvdata(pdev, gf);

	gf->vdd = devm_regulator_get(dev, "l11c-vdd");
	if (IS_ERR(gf->vdd))
		return dev_err_probe(dev, PTR_ERR(gf->vdd),
				     "Failed to get l11c-vdd regulator\n");

	ret = regulator_set_load(gf->vdd, 200000);
	if (ret)
		dev_warn(dev, "Failed to set l11c-vdd load: %d\n", ret);

	ret = regulator_enable(gf->vdd);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable l11c-vdd\n");
	gf->power_enabled = true;

	gf->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gf->reset_gpio)) {
		regulator_disable(gf->vdd);
		return dev_err_probe(dev, PTR_ERR(gf->reset_gpio),
				     "Failed to get reset GPIO\n");
	}

	gf->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(gf->irq_gpio)) {
		regulator_disable(gf->vdd);
		return dev_err_probe(dev, PTR_ERR(gf->irq_gpio),
				     "Failed to get irq GPIO\n");
	}

	gf->irq = gpiod_to_irq(gf->irq_gpio);
	if (gf->irq < 0) {
		regulator_disable(gf->vdd);
		return dev_err_probe(dev, gf->irq, "Failed to map irq GPIO\n");
	}

	/* Release sensor from reset cleanly at probe */
	goodix_fod_hw_reset(gf, 10);

	ret = devm_request_threaded_irq(dev, gf->irq, NULL,
					goodix_fod_irq_thread,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT |
					IRQF_NO_AUTOEN,
					"goodix_fod", gf);
	if (ret) {
		regulator_disable(gf->vdd);
		return dev_err_probe(dev, ret, "Failed to request IRQ %d\n",
				     gf->irq);
	}
	gf->irq_enabled = false;

	g_gf_dev = gf;
	gf->nl_sk = netlink_kernel_create(&init_net, GF_NETLINK_PROTO, &nl_cfg);
	if (!gf->nl_sk)
		dev_warn(dev, "Failed to create netlink socket %d\n",
			 GF_NETLINK_PROTO);

	gf->misc.minor = MISC_DYNAMIC_MINOR;
	gf->misc.name = "goodix_fp";
	gf->misc.fops = &goodix_fod_fops;
	gf->misc.parent = dev;

	ret = misc_register(&gf->misc);
	if (ret) {
		if (gf->nl_sk)
			netlink_kernel_release(gf->nl_sk);
		g_gf_dev = NULL;
		regulator_disable(gf->vdd);
		return dev_err_probe(dev, ret, "Failed to register /dev/goodix_fp\n");
	}

	dev_info(dev, "Goodix FOD platform driver probed (irq=%d)\n", gf->irq);
	return 0;
}

static void goodix_fod_remove(struct platform_device *pdev)
{
	struct goodix_fod *gf = platform_get_drvdata(pdev);

	misc_deregister(&gf->misc);
	if (gf->nl_sk)
		netlink_kernel_release(gf->nl_sk);
	g_gf_dev = NULL;
	if (gf->power_enabled)
		regulator_disable(gf->vdd);
}

static const struct of_device_id goodix_fod_of_match[] = {
	{ .compatible = "goodix,fingerprint" },
	{ }
};
MODULE_DEVICE_TABLE(of, goodix_fod_of_match);

static struct platform_driver goodix_fod_driver = {
	.probe = goodix_fod_probe,
	.remove = goodix_fod_remove,
	.driver = {
		.name = "goodix_fod",
		.of_match_table = goodix_fod_of_match,
		.dev_groups = goodix_fod_groups,
	},
};
module_platform_driver(goodix_fod_driver);

MODULE_DESCRIPTION("Goodix FOD Fingerprint Platform Driver");
MODULE_LICENSE("GPL");
