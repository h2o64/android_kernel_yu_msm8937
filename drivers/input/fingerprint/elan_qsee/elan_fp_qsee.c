#include <linux/module.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/err.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/time.h>
#include <linux/namei.h>
#include <linux/mount.h>
#include <linux/poll.h>
#include <linux/completion.h>
#include <linux/of_gpio.h>
#include <soc/qcom/scm.h>
#include "elan_fp_qsee.h"

#include <linux/notifier.h>
#include <linux/fb.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/regulator/consumer.h>
#include "../fp_drv/fp_drv.h"





#define VERSION_LOG	"ELAN FINGER PRINT V1.4.4.1"
#define ELAN_FP_NAME "elan_fp"
#define _ELAN_DEBUG_
#ifdef _ELAN_DEBUG_
static int elan_debug = 1;
#define ELAN_DEBUG(format, args ...) \
			do { \
					if (elan_debug) \
							printk("[ELAN] " format, ##args); \
			} while (0)
#else
#define ELAN_DEBUG(format, args ...)
#endif

#define GPIO_FP_ID	880
#define KEY_FP_INT			KEY_POWER
#define KEY_FP_INT2			KEY_1
#define SET_SPI_OWNER 0

struct completion cmd_done_irq;
static int factory_status = 0;
static DEFINE_MUTEX(elan_factory_mutex);
static struct fasync_struct *fasync_queue = NULL;
static int key_status = 0;

#define GF_VDD_MIN_UV      2800000
#define GF_VDD_MAX_UV	   2800000
#define GF_VIO_MIN_UV      1800000
#define GF_VIO_MAX_UV      1800000

struct efsa120s_data  {
	int 					irq_gpio;
	int						isr;
	int 					rst_gpio;
	int						irq_is_disable;
	struct miscdevice		efsa120_dev;	/* char device for ioctl */
	struct platform_device	*pdev;

	struct input_dev		*input_dev;
	spinlock_t				irq_lock;
	wait_queue_head_t		efsa_wait;
	struct wake_lock		wake_lock;
	u8 isPowerOn;
	struct regulator *vdd;
	struct regulator *vio;

	struct notifier_block notifier;
};

int elan_power_ctl(struct efsa120s_data *pdata, bool on);
int elan_request_irq(struct efsa120s_data *pdata);



#define MAX_LIB_BUF 256
static char lib_ver_buf[MAX_LIB_BUF] = "unknow";


#define POWER_NOTIFY
static int is_screen_poweroff = 0;



void efsa120s_irq_enable(void *_fp)
{
	struct efsa120s_data *fp = _fp;
	unsigned long irqflags = 0;
	ELAN_DEBUG("IRQ Enable = %d.\n", fp->isr);

	spin_lock_irqsave(&fp->irq_lock, irqflags);
	if (fp->irq_is_disable) {
		enable_irq(fp->isr);
		fp->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&fp->irq_lock, irqflags);
}

void efsa120s_irq_disable(void *_fp)
{
	struct efsa120s_data *fp = _fp;
	unsigned long irqflags;
	ELAN_DEBUG("IRQ Disable = %d.\n", fp->isr);

	spin_lock_irqsave(&fp->irq_lock, irqflags);
	if (!fp->irq_is_disable) {
		fp->irq_is_disable = 1;
		disable_irq_nosync(fp->isr);
	}
	spin_unlock_irqrestore(&fp->irq_lock, irqflags);
}

static ssize_t show_drv_version_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VERSION_LOG);
}
static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);

#ifdef _ELAN_DEBUG_
static ssize_t elan_debug_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(elan_debug) {
		elan_debug=0;
	} else {
		elan_debug=1;
	}
	return sprintf(buf, "[ELAN] elan debug %d\n", elan_debug);
}
static DEVICE_ATTR(elan_debug, S_IRUGO, elan_debug_value, NULL);
#endif

static struct attribute *efsa120s_attributes[] = {
	&dev_attr_drv_version.attr,
#ifdef _ELAN_DEBUG_
	&dev_attr_elan_debug.attr,
#endif
	NULL
};

static struct attribute_group efsa120s_attr_group = {
	.attrs = efsa120s_attributes,
};

static void efsa120s_reset(struct efsa120s_data *fp)
{
	/* Developement platform */
	gpio_direction_output(fp->rst_gpio, 0);
	mdelay(5);
	gpio_direction_output(fp->rst_gpio, 1);
	mdelay(50);
}

static int efsa120s_open(struct inode *inode, struct file *filp)
{
	struct efsa120s_data *fp = container_of(filp->private_data, struct efsa120s_data, efsa120_dev);
	filp->private_data = fp;
	ELAN_DEBUG("%s()\n", __func__);
	return 0;
}

static int efsa120s_close(struct inode *inode, struct file *filp)
{
	ELAN_DEBUG("%s()\n", __func__);
	return 0;
}

static long efsa120s_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct efsa120s_data *fp = filp->private_data;
	int ret = 0;

	ELAN_DEBUG("%s() : cmd = [%04X]\n", __func__, cmd);

	switch(cmd) {
	case ID_IOCTL_RESET:
		efsa120s_reset(fp);
		ELAN_DEBUG("ID_IOCTL_RESET\n");
		break;
	case ID_IOCTL_POLL_INIT:
		reinit_completion(&cmd_done_irq);
		ELAN_DEBUG("ID_IOCTL_POLL_INIT\n");
		break;
	case ID_IOCTL_POLL_EXIT:
		complete(&cmd_done_irq);
		ELAN_DEBUG("ID_IOCTL_POLL_EXIT\n");
		break;
	case ID_IOCTL_READ_FACTORY_STATUS:
		mutex_lock(&elan_factory_mutex);
		ELAN_DEBUG("READ_FACTORY_STATUS = %d", factory_status);
		mutex_unlock(&elan_factory_mutex);
		return factory_status;
		break;
	case ID_IOCTL_WRITE_FACTORY_STATUS:
		mutex_lock(&elan_factory_mutex);
		factory_status = (int __user)arg;
		ELAN_DEBUG("WRITE_FACTORY_STATUS = %d\n", factory_status);
		mutex_unlock(&elan_factory_mutex);
		break;
	case ID_IOCTL_EN_IRQ:
		efsa120s_irq_enable(fp);
		ELAN_DEBUG("ID_IOCTL_EN_IRQ\n");
		break;
	case ID_IOCTL_DIS_IRQ:
		efsa120s_irq_disable(fp);
		ELAN_DEBUG("ID_IOCTL_DIS_IRQ\n");
		break;
	case ID_IOCTL_POWER_SET:
		if(arg) {
			elan_power_ctl(fp, true);
		} else {
			elan_power_ctl(fp, false);
		}
		break;
	case IOCTL_READ_KEY_STATUS:
		ELAN_DEBUG("IOCTL_READ_KEY_STATUS,key_status = %d\n", key_status);
		return (key_status == 1 ? 1 : 2);
	case IOCTL_WRITE_KEY_STATUS:
		ELAN_DEBUG("IOCTL_WRITE_KEY_STATUS,arg = %ld\n", arg);
		key_status = arg;
		if (fasync_queue) {
			ELAN_DEBUG("IOCTL_WRITE_KEY_STATUS,kill_fasync to send key event\n");
			kill_fasync(&fasync_queue, SIGIO, POLL_IN);
		}
		break;
	case ID_IOCTL_REQUSEST_IRQ:
		if(arg) {
			if (elan_request_irq(fp)) {
				break;
			}
			full_fp_chip_name(ELAN_FP_NAME);
		}
		break;
	case ID_IOCTL_SET_VERSION:
		ret = copy_from_user(lib_ver_buf, (char *)arg, MAX_LIB_BUF);
		if (!ret) {
			full_fp_chip_info(lib_ver_buf);
		}
		break;
	case ID_IOCTL_GET_VERSION:
		ret = copy_to_user((char *)arg, lib_ver_buf, MAX_LIB_BUF);
		break;

	case ID_IOCTL_GET_SCREEN_STATUS:
		return is_screen_poweroff;

	default:
		ELAN_DEBUG("INVALID COMMAND\n");
		break;
	}
	return 0;
}

static unsigned int efsa120s_poll(struct file *file, poll_table *wait)
{
	struct efsa120s_data *fp = file->private_data;
	int mask=0;
	wait_for_completion_interruptible(&cmd_done_irq);
	poll_wait(file, &fp->efsa_wait, wait);




	mask |= POLLIN | POLLRDNORM;

	return mask;
}

static int elan_fp_fasync(int fd, struct file * filp, int on)
{

	return fasync_helper(fd, filp, on, &fasync_queue);
}

static const struct file_operations efsa120s_fops = {
	.owner 			= THIS_MODULE,
	.open 			= efsa120s_open,
	.unlocked_ioctl = efsa120s_ioctl,
	.poll			= efsa120s_poll,
	.release 		= efsa120s_close,
	.fasync 		= elan_fp_fasync,
};

#if SET_SPI_OWNER
static int set_pipe_ownership(void)
{
	const u32 TZ_BLSP_MODIFY_OWNERSHIP_ID = 3;
	const u32 TZBSP_TZ_ID = 3;
	int rc;
	struct scm_desc desc = {
		.arginfo = SCM_ARGS(2),
		.args[0] = 3,
		.args[1] = TZBSP_TZ_ID,
	};

	rc = scm_call2(SCM_SIP_FNID(SCM_SVC_TZ, TZ_BLSP_MODIFY_OWNERSHIP_ID), &desc);

	if(rc || desc.ret[0]) {
		ELAN_DEBUG("%s() FAIL\n", __func__);
		return -EINVAL;
	}
	ELAN_DEBUG("%s() Success\n", __func__);
	return 0;
}
#endif

static irqreturn_t efsa120s_irq_handler(int irq, void *_fp)
{
	struct efsa120s_data *fp = _fp;

	ELAN_DEBUG("%s()\n", __func__);
	/* input power keyevent */
	wake_lock_timeout(&fp->wake_lock,msecs_to_jiffies(1000));
#if 0
	input_report_key(fp->input_dev, KEY_FP_INT, 1); /* Added for KEY Event */
	input_sync(fp->input_dev);
	msleep(1);
	input_report_key(fp->input_dev, KEY_FP_INT, 0); /* Added for KEY Event */
	input_sync(fp->input_dev);
	msleep(1);
#endif

	complete(&cmd_done_irq);






	return IRQ_HANDLED;
}

static int efsa120s_setup_cdev(struct efsa120s_data *fp)
{

	fp->efsa120_dev.minor = MISC_DYNAMIC_MINOR;
	fp->efsa120_dev.name = "elan_fp";
	fp->efsa120_dev.fops = &efsa120s_fops;
	fp->efsa120_dev.mode = S_IFREG|S_IRWXUGO;
	if (misc_register(&fp->efsa120_dev) < 0) {
		ELAN_DEBUG("misc_register failed!!");
		return -1;
	} else {
		ELAN_DEBUG("misc_register finished!!");
	}
	return 0;
}

static int efsa120s_sysfs_create(struct efsa120s_data *sysfs)
{
	struct efsa120s_data *fp = platform_get_drvdata(sysfs->pdev);
	int error = 0;

	/* Register sysfs */
	error = sysfs_create_group(&fp->pdev->dev.kobj, &efsa120s_attr_group);
	if (error) {
		dev_err(&fp->pdev->dev, "[ELAN] Failed to create sysfs attributes, err: %d\n", error);
		goto fail_un;
	}
	return 0;
fail_un:
	/* Remove sysfs */
	sysfs_remove_group(&fp->pdev->dev.kobj, &efsa120s_attr_group);

	return error;
}

static char efsa120s_gpio_config(void *_fp)
{
	struct efsa120s_data *fp = _fp;
	int ret;

	/* Developement platform */

	ret = gpio_request(fp->irq_gpio, "efsa120-irq");
	ELAN_DEBUG("irq gpio_request ret = %d", ret);
	if (false/*ret < 0*/) {
		ELAN_DEBUG("%s() IRQ(%d) request fail, err=0x%x.\n", __func__, fp->irq_gpio, ret);
		ret = -ENODEV;
	} else {
		gpio_direction_input(fp->irq_gpio);
		fp->isr = gpio_to_irq(fp->irq_gpio);
		ELAN_DEBUG("%s() IRQ(%d) = ISR(%d) request success, err=0x%x.\n", __func__, fp->irq_gpio, fp->isr, ret);
	}


	ret =  gpio_request(fp->rst_gpio, "efsa120-reset");
	ELAN_DEBUG("reset gpio_request ret = %d", ret);
	if (false/*ret < 0*/) {
		gpio_free(fp->irq_gpio);
		free_irq(fp->isr, fp);
		ELAN_DEBUG("%s() RST%d request fail, err=0x%x.\n", __func__, fp->rst_gpio, ret);
		ret = -ENODEV;
	} else {
		ELAN_DEBUG("%s() RST%d request success, err=0x%x.\n", __func__, fp->rst_gpio, ret);
		gpio_direction_output(fp->rst_gpio, 0);
		mdelay(20);
		gpio_direction_output(fp->rst_gpio, 1);
		mdelay(20);
		ELAN_DEBUG("%s() Reset ...\n", __func__);
	}


	/*ret =  gpio_request(fp->fpid_gpio, "efsa120-fpid");
	if (ret < 0)
	{
		ELAN_DEBUG("%s() FPID%d request fail, err=0x%x.\n", __func__, fp->fpid_gpio, ret);
		ret = -ENODEV;
	}
	else
	{
		ELAN_DEBUG("%s() FPID%d request success, err=0x%x.\n", __func__, fp->fpid_gpio, ret);
	       fpid = gpio_get_value(fp->fpid_gpio);
		ELAN_DEBUG("%s() FPID = %d...\n", fpid,  __func__);
	}*/

	return ret;
}

static int elan_parse_dt(struct device *dev, struct efsa120s_data *pdata)
{

	struct device_node *np = dev->of_node;

	/* +++reset, irq gpio info+++ */
	pdata->rst_gpio = of_get_named_gpio(np, "qcom,reset-gpio", 0);
	ELAN_DEBUG("rst_gpio = %d\n", pdata->rst_gpio);
	if (pdata->rst_gpio < 0)
		return pdata->rst_gpio;

	pdata->irq_gpio = of_get_named_gpio(np, "qcom,irq-gpio", 0);
	ELAN_DEBUG("irq_gpio = %d\n", pdata->irq_gpio);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;


	/*pdata->fpid_gpio = of_get_named_gpio(np, "qcom,fpid-gpio", 0);
	ELAN_DEBUG("fpid_gpio = %d\n", pdata->fpid_gpio);
	if (pdata->fpid_gpio < 0)
		return pdata->irq_gpio;	*/

	/* ---reset, irq gpio info--- */

	/* ==optional== */
	/*
		pdata->osvcc_pin = of_get_named_gpio_flags(np,
				"osvcc-gpio", 0, NULL);

		pdata->vcc3v3 = of_get_named_gpio_flags(np,
				"vcc3v3-gpio", 0, NULL);
	*/
	ELAN_DEBUG("rst-gpio = %d, irq_pin = %d\n", pdata->rst_gpio, pdata->irq_gpio);

	return 0;
}

int elan_power_ctl(struct efsa120s_data *pdata, bool on)
{
	int rc = 0;

	if (on && (!pdata->isPowerOn)) {
		rc = regulator_enable(pdata->vdd);
		if (rc) {
			dev_err(&pdata->pdev->dev,
			        "elan Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		msleep(10);

		pdata->isPowerOn = 1;
		ELAN_DEBUG(" set PowerOn ok !\n");
	} else if (!on && (pdata->isPowerOn)) {

		rc = regulator_disable(pdata->vdd);
		if (rc) {
			dev_err(&pdata->pdev->dev,
			        "elan Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		pdata->isPowerOn = 0;
		ELAN_DEBUG(" set PowerDown !ok \n");
	} else {
		dev_warn(&pdata->pdev->dev,
		         "elan Ignore power status change from %d to %d\n",
		         on, pdata->isPowerOn);
	}
	return rc;
}

int elan_request_irq(struct efsa120s_data *pdata)
{
	int err = request_irq(pdata->isr, efsa120s_irq_handler,
	                      IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING,
	                      pdata->pdev->dev.driver->name, pdata);

	if (err) {
		ELAN_DEBUG("Failed to request IRQ %d.\n", err);
		return -1;
	}

	irq_set_irq_wake(pdata->isr, 1);
	spin_lock_init(&pdata->irq_lock);
	return 0;
}


int elan_power_init(struct efsa120s_data *pdata)
{
	int ret = 0;

	pdata->vdd = regulator_get(&pdata->pdev->dev, "vdd");
	if (IS_ERR(pdata->vdd)) {
		ret = PTR_ERR(pdata->vdd);
		printk("elan Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(pdata->vdd) > 0) {
		ret = regulator_set_voltage(pdata->vdd, GF_VDD_MIN_UV,
		                            GF_VDD_MAX_UV);
		if (ret) {
			printk("elan Regulator set_vtg failed vdd ret=%d\n", ret);
			goto reg_vdd_put;
		}
	}

	printk("elan Regulator set_vtg OK vdd ret=%d \n", ret);
	return 0;

reg_vdd_put:
	regulator_put(pdata->vdd);

	return ret;
}

int elan_power_deinit(struct efsa120s_data *pdata)
{
	int ret = 0;

	if (pdata->vdd) {
		if (regulator_count_voltages(pdata->vdd) > 0)
			regulator_set_voltage(pdata->vdd, 0, GF_VDD_MAX_UV);

		regulator_disable(pdata->vdd);
		regulator_put(pdata->vdd);
	}

	return ret;
}

#ifdef POWER_NOTIFY
static int elan_fb_state_chg_callback(struct notifier_block *nb,
                                      unsigned long val, void *data)
{
	struct fb_event *evdata = data;
	unsigned int blank;
	struct efsa120s_data *fp = container_of(nb, struct efsa120s_data, notifier);

	if (val != FB_EARLY_EVENT_BLANK) {
		return 0;
	}

	if (evdata && evdata->data && val == FB_EARLY_EVENT_BLANK && fp) {
		blank = *(int *)(evdata->data);
		switch (blank) {
		case FB_BLANK_POWERDOWN:
			is_screen_poweroff = 1;
			complete(&cmd_done_irq);


			break;
		case FB_BLANK_UNBLANK:
			is_screen_poweroff = 0;
			break;
		default:
			pr_info("%s defalut\n", __func__);
			break;
		}
	}
	return NOTIFY_OK;
}

static struct notifier_block elan_noti_block = {
	.notifier_call = elan_fb_state_chg_callback,
};
#endif

static int efsa120s_probe(struct platform_device *pdev)

{
	struct efsa120s_data *fp = NULL;
	struct input_dev *input_dev = NULL;
	int err = 0;

	ELAN_DEBUG("=====%s() Start=====\n", __func__);
	ELAN_DEBUG("%s GPIO_FP_ID : %d\n", VERSION_LOG, gpio_get_value(GPIO_FP_ID));

	init_completion(&cmd_done_irq);

	/* Allocate Device Data */
	fp = devm_kzalloc(&pdev->dev, sizeof(struct efsa120s_data), GFP_KERNEL);
	if(!fp)
		ELAN_DEBUG("alloc efsa120s data fail.\n");

	/* Init Poll Wait */
	init_waitqueue_head(&fp->efsa_wait);

	/* Parse Device Tree */
	err = elan_parse_dt(&pdev->dev, fp);

	/* Init Input Device */
	input_dev = input_allocate_device();
	if (!input_dev)
		ELAN_DEBUG("alloc input_dev fail.\n");

	fp->pdev = pdev;

	platform_set_drvdata(pdev, fp);


	err = elan_power_init(fp);

	input_dev->name = "efsa120s";
	input_dev->id.bustype = BUS_SPI;
	input_dev->dev.parent = &pdev->dev;
	input_set_drvdata(input_dev, fp);

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
	input_set_capability(input_dev, EV_KEY, KEY_FP_INT);
	input_set_capability(input_dev, EV_KEY, KEY_FP_INT2);

	fp->input_dev = input_dev;

	/* Init Sysfs */
	err = efsa120s_sysfs_create(fp);
	if(err < 0)
		ELAN_DEBUG("efsa120s sysfs fail.\n");

	/* Init Char Device */
	err = efsa120s_setup_cdev(fp);
	if(err < 0)
		ELAN_DEBUG("efsa120s setup device fail.\n");

	/* Register Input Device */
	err = input_register_device(input_dev);
	if(err) {
		ELAN_DEBUG("Unable to register input device, error: %d!\n", err);
		goto fp_probe_fail;
	}

	/* Init EFSA120S GPIO */
	err = efsa120s_gpio_config(fp);
	if(err < 0)
		ELAN_DEBUG("GPIO request fail (%d).\n", err);

	wake_lock_init(&fp->wake_lock, WAKE_LOCK_SUSPEND, "fp_wake_lock");

#ifdef POWER_NOTIFY
	fp->notifier = elan_noti_block;
	fb_register_client(&fp->notifier);
#endif
	/* Init IRQ FUNC */
	/*err = request_threaded_irq(fp->isr, NULL, efsa120s_irq_handler,
			IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			pdev->dev.driver->name, fp);*/


	/* Set Spi to TZ */
#if SET_SPI_OWNER
	err = set_pipe_ownership();
#endif

	ELAN_DEBUG("=====%s() End=====\n", __func__);
	return 0;

fp_probe_fail:
	platform_set_drvdata(pdev, NULL);

	input_free_device(input_dev);
	input_dev = NULL;
	kfree(fp);
	return -ENOMEM;
}


static int efsa120s_remove(struct platform_device *pdev)

{
#if 1
	struct efsa120s_data *fp = platform_get_drvdata(pdev);
#else
	struct efsa120s_data *fp = spi_get_drvdata(pdev);
#endif


	if (fp->isr)
		free_irq(fp->isr, fp);

	gpio_free(fp->irq_gpio);
	gpio_free(fp->rst_gpio);

	misc_deregister(&fp->efsa120_dev);
	input_free_device(fp->input_dev);
	elan_power_deinit(fp);

	kfree(fp);
#if 1
	platform_set_drvdata(pdev, NULL);
#else
	spi_set_drvdata(pdev, NULL);
#endif
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int efsa120s_suspend(struct device *dev)
{
	ELAN_DEBUG("efsa120s suspend!\n");
	return 0;
}

static int efsa120s_resume(struct device *dev)
{
	ELAN_DEBUG("efsa120s resume!\n");
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(efsa120s_pm_ops, efsa120s_suspend, efsa120s_resume);

#ifdef CONFIG_OF
static struct of_device_id efsa120s_metallica_table[] = {
	{ .compatible = "qcom,fingerprint",},
	{ },
};
#else
#define efsa120s_metallica_table NULL
#endif

#if 1
static struct platform_driver efsa120s_driver = {
#else
static struct spi_driver efsa120s_driver = {
#endif
	.driver = {
		.name 	= "efsa120s",
		.owner = THIS_MODULE,
		.pm 	= &efsa120s_pm_ops,
		.of_match_table = efsa120s_metallica_table,
	},
	.probe 	= efsa120s_probe,
	.remove = efsa120s_remove,
};

static int __init efsa120s_init(void)
{
	int status = 0;
	ELAN_DEBUG("=====%s() Start=====\n", __func__);
	status = platform_driver_register(&efsa120s_driver);

	ELAN_DEBUG("%s --->>>>>111>>>status = %d !\n",__func__,status);
	ELAN_DEBUG("%s --->>>>>222>>>status = %d !\n",__func__,status);
	if(status < 0)
		ELAN_DEBUG("%s FAIL !\n", __func__);

	ELAN_DEBUG("=====%s() End=====\n", __func__);

	return status;
}

static void __exit efsa120s_exist(void)
{
	platform_driver_unregister(&efsa120s_driver);


}

module_init(efsa120s_init);
module_exit(efsa120s_exist);

MODULE_AUTHOR("KennyKang <kenny.kang@emc.com.tw>");
MODULE_DESCRIPTION("ELAN SPI FingerPrint eFSA120S driver");
MODULE_VERSION(VERSION_LOG);
MODULE_LICENSE("GPL");
