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
#define DEV_NAME "TINNO,kernel"
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/cpu.h>
#include <linux/syscalls.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/export.h>
#include <linux/syscore_ops.h>
#include <linux/ftrace.h>
#include <linux/rtc.h>
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

extern int pm_get_wakelocks(bool show_active);
extern void pm_print_active_wakeup_sources(void);

#if defined(CONFIG_FB)
	 struct notifier_block fb_notif;
#endif

struct delayed_work w_work;
static void work_func(struct work_struct *work)
{
         
#if 0 //Tinno:CJ this is for debug IO wait
	struct task_struct *p;
	struct timespec ts;
	struct rtc_time tm;	

	//printk("----- work_func-----\n");
	if(nr_iowait() > 5)
	{
		printk("----- iowait log start -----\n");
		for_each_process(p)
		{
			if(p->in_iowait == 1)
			{
				getnstimeofday(&ts);
				rtc_time_to_tm(ts.tv_sec, &tm);
				printk("!!!WARNING!!! iowait %ld, process pid: %d, process name: %s, %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",nr_iowait(),p->pid, p->comm,tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
			}
		}
		printk("----- iowait log end -----\n");
	}
#endif 
	int wakelocks = pm_get_wakelocks(1);
	if(wakelocks == 0) // not wakelock ,but may be some wakeup source 
	{
		printk("Tinno:pm_get_wakelocks 0\n");
		pm_print_active_wakeup_sources();//Tinno:CJ
	}
	else
	{
		printk("pm_get_wakelocks %d\n",wakelocks);
		pm_print_active_wakeup_sources();//Tinno:CJ just debug , it will cause so much log		
	}

	schedule_delayed_work(&w_work,HZ*10);
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	//printk("Tinno: fb_notifier_callback\n");
	if (evdata && evdata->data) {
		if (event == FB_EARLY_EVENT_BLANK){
			//printk("Tinno: fb_notifier_callback FB_EARLY_EVENT_BLANK\n");
		}
		else if (event == FB_EVENT_BLANK) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK){
				printk("Tinno: fb_notifier_callback FB_BLANK_UNBLANK\n");
				cancel_delayed_work_sync(&w_work);

			}
			else if (*blank == FB_BLANK_POWERDOWN){
				printk("Tinno: fb_notifier_callback FB_BLANK_POWERDOWN\n");
				schedule_delayed_work(&w_work,HZ*10);

			}
			else{
				//printk("Tinno: fb_notifier_callback can't recongise evdata->data\n");
			}
		}
		else{
			//printk("Tinno: fb_notifier_callback can't recongise event\n");
		}
	}

	return 0;
}

static void configure_fb(void)
{
	int retval = 0;

	fb_notif.notifier_call = fb_notifier_callback;

	retval = fb_register_client(&fb_notif);
	if (retval)
		printk("Tinno:Unable to register fb_notifier: %d\n", retval);

	
	return;
}
#endif

static __init int work_init(void)
{
	INIT_DELAYED_WORK(&w_work,work_func);
#if defined(CONFIG_FB)
	configure_fb();
#else
	schedule_delayed_work(&w_work,HZ*10);
#endif
	return 0;    
}
static void work_exit(void)
{
#if defined(CONFIG_FB)
#else
	cancel_delayed_work_sync(&w_work);
#endif
}
module_init(work_init);
module_exit(work_exit);


MODULE_DESCRIPTION(DEV_NAME);
MODULE_LICENSE("GPL v2");
