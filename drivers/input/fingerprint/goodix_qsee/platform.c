#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

#define GF_VDD_MIN_UV      2800000
#define GF_VDD_MAX_UV	        3000000
#define GF_VIO_MIN_UV      1800000
#define GF_VIO_MAX_UV      1800000

/* GPIO pins reference.*/
int gf_parse_dts(struct gf_dev* gf_dev)
{
	int rc = 0;


	/*get irq resourece*/
	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"qcom,irq-gpio",0);

	if (gpio_is_valid(gf_dev->irq_gpio)) {
		rc = pinctrl_request_gpio(gf_dev->irq_gpio);
		if(rc) {
			printk("could not request irq gpio\n");
			return rc;
		}
	} else {
		printk("not valid irq gpio\n");
		return -EIO;

	}

	pinctrl_gpio_direction_input(gf_dev->irq_gpio);

	/*get reset resource*/
	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"qcom,reset-gpio",0);
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		rc = gpio_request(gf_dev->reset_gpio, "gf_reset-gpio");
		if(rc) {
			printk("could not request reset gpio\n");
			return rc;
		}
	} else {
		printk("not valid reset gpio\n");
		return -EIO;
	}
	printk("gf parser dt ok.\n");

	return rc;
}

void gf_cleanup(struct gf_dev	* gf_dev)
{
	pr_info("[info] %s\n",__func__);
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		gpio_free(gf_dev->pwr_gpio);
		pr_info("remove pwr_gpio success\n");
	}
}

/*power management*/
int gf_power_on(struct gf_dev* gf_dev)
{
	int rc = 0;
	rc = gf_power_ctl(gf_dev, true);
	return rc;

}

int gf_power_off(struct gf_dev* gf_dev)
{
	int rc = 0;
	rc = gf_power_ctl(gf_dev, false);
	return rc;
}

/********************************************************************
 *CPU output low level in RST pin to reset GF. This is the MUST action for GF.
 *Take care of this function. IO Pin driver strength / glitch and so on.
 ********************************************************************/
int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if(gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if(gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}


int gf_power_ctl(struct gf_dev* gf_dev, bool on)
{
	int rc = 0;

	if (on && (!gf_dev->isPowerOn)) {
		rc = regulator_enable(gf_dev->vdd);
		if (rc) {
			dev_err(&gf_dev->spi->dev,
			        "Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}
		msleep(10);
		gf_dev->isPowerOn = 1;
	} else if (!on && (gf_dev->isPowerOn)) {

		rc = regulator_disable(gf_dev->vdd);
		if (rc) {
			dev_err(&gf_dev->spi->dev,
			        "Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
		gf_dev->isPowerOn = 0;
	} else {
		dev_warn(&gf_dev->spi->dev,
		         "Ignore power status change from %d to %d\n",
		         on, gf_dev->isPowerOn);
	}
	return rc;
}

int gf_power_init(struct gf_dev* gf_dev)
{
	int ret = 0;

	gf_dev->vdd = regulator_get(&gf_dev->spi->dev, "vdd");
	if (IS_ERR(gf_dev->vdd)) {
		ret = PTR_ERR(gf_dev->vdd);
		dev_err(&gf_dev->spi->dev,
		        "Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(gf_dev->vdd) > 0) {
		ret = regulator_set_voltage(gf_dev->vdd, GF_VDD_MIN_UV,
		                            GF_VDD_MAX_UV);
		if (ret) {
			dev_err(&gf_dev->spi->dev,
			        "Regulator set_vtg failed vdd ret=%d\n", ret);
			goto reg_vdd_put;
		}
	}
	return 0;

reg_vdd_put:
	regulator_put(gf_dev->vdd);
	return ret;
}

int gf_power_deinit(struct gf_dev* gf_dev)
{
	int ret = 0;

	if (gf_dev->vdd) {
		if (regulator_count_voltages(gf_dev->vdd) > 0)
			regulator_set_voltage(gf_dev->vdd, 0, GF_VDD_MAX_UV);

		regulator_disable(gf_dev->vdd);
		regulator_put(gf_dev->vdd);
	}
	return ret;
}
