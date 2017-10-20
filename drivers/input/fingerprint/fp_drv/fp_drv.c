/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>

#include "fp_drv.h"


static int fp_probe(struct platform_device *pdev);
static int fp_remove(struct platform_device *pdev);


static struct platform_driver fp_driver = {
	.probe = fp_probe,
	.remove = fp_remove,
	.driver = {
		.name = "fp_drv",
	},
};

struct platform_device fp_device = {
	.name   	= "fp_drv",
	.id        	= -1,
};






static char m_dev_name[64];
static int has_exist = 0;


static char m_dev_info[64];
static int all_info_exist = 0;

static DECLARE_WAIT_QUEUE_HEAD(waiter);


int full_fp_chip_name(const char *name)
{
	__FUN();

	if((name == NULL) || (has_exist == 1)) {
		klog("----(name == NULL) || (has_exist == 1)--err!---\n");
		return -1;
	}

	memset(m_dev_name, 0, sizeof(m_dev_name));
	strcpy(m_dev_name, name);
	has_exist = 1;
	klog("---has_exist---:[%s]\n",  m_dev_name);
	return 0;
}

int full_fp_chip_info(const char *info)
{
	__FUN();

	if((info == NULL) || (all_info_exist == 1)) {
		klog("----(info == NULL) || (all_info_exist == 1)--err!---\n");
		return -1;
	}

	memset(m_dev_info, 0, sizeof(m_dev_info));
	strcpy(m_dev_info, info);
	all_info_exist = 1;
	klog("---m_dev_info---:[%s]\n",  m_dev_info);
	return 0;
}

static ssize_t info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(has_exist) {
		return sprintf(buf, "%s", m_dev_name);
	}
	return sprintf(buf, "%s", "unknow");
}

static ssize_t all_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(all_info_exist) {
		return sprintf(buf, "%s", m_dev_info);
	}
	return sprintf(buf, "%s", "unknow");
}

static DEVICE_ATTR(fp_drv_info, 0444, info_show, NULL);
static DEVICE_ATTR(fp_drv_all_info, 0444, all_info_show, NULL);

static int fp_probe(struct platform_device *pdev)
{
	__FUN();

	device_create_file(&pdev->dev, &dev_attr_fp_drv_info);
	device_create_file(&pdev->dev, &dev_attr_fp_drv_all_info);
	return 0;
}

static int fp_remove(struct platform_device *pdev)
{
	__FUN();
	device_remove_file(&pdev->dev, &dev_attr_fp_drv_info);
	device_remove_file(&pdev->dev, &dev_attr_fp_drv_all_info);
	return 0;
}

static int __init fp_drv_init(void)
{
	__FUN();

	if (platform_device_register(&fp_device) != 0) {
		klog( "device_register fail!.\n");
		return -1;

	}

	if (platform_driver_register(&fp_driver) != 0) {
		klog( "driver_register fail!.\n");
		return -1;
	}

	return 0;
}

static void __exit fp_drv_exit(void)
{
	__FUN();
	platform_driver_unregister(&fp_driver);
}


late_initcall(fp_drv_init);
module_exit(fp_drv_exit);







