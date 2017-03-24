/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define TINNO_DEV_INFO_NAME "qcom,dev-info"
extern struct proc_dir_entry *device_info_entry;

static struct of_device_id tinno_dev_info_match_table[] = {
	{	.compatible = TINNO_DEV_INFO_NAME,
	},
	{}
};

struct proc_dir_entry * creat_devinfo_file(const char *name,struct file_operations * fp)
{
	struct proc_dir_entry * file_entry;
	if(device_info_entry!=NULL)
	{
		file_entry=proc_create_data(name, S_IRUGO,device_info_entry,fp,NULL);
		return file_entry;
	}else{
		printk("device_info_entry is null! \n");
		return NULL;
	}
}
	
static int tinno_dev_info_probe(struct platform_device *pdev)
{
	printk("tinno_dev_info_probe \n");
	device_info_entry=proc_mkdir("Tinno_devinfo",NULL);
 	return 0;
}

static struct platform_driver  tinno_dev_info_driver = {
	.probe = tinno_dev_info_probe,
	.driver = {
		.name = "Tinno_Device_Info",
		.owner = THIS_MODULE,
		.of_match_table = tinno_dev_info_match_table,
	},
};

static int __init tinno_dev_info_init(void)
{
	return platform_driver_register(&tinno_dev_info_driver);
}

static void __exit tinno_dev_info_exit(void)
{
	return platform_driver_unregister(&tinno_dev_info_driver);
}

subsys_initcall(tinno_dev_info_init);
module_exit(tinno_dev_info_exit);

MODULE_DESCRIPTION(TINNO_DEV_INFO_NAME);
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" TINNO_DEV_INFO_NAME);
