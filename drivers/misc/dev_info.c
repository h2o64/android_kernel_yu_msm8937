#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>

//#define CONFIG_DEV_INFO_DEBUG

#ifdef CDBG
#undef CDBG
#endif

#ifdef CONFIG_DEV_INFO_DEBUG
#define CDBG(fmt, args...) pr_err("[dev_info] " fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define INFO_LEN  80

static struct kobject *dev_info;

static char main_camera[INFO_LEN];
static char sub_camera[INFO_LEN];
static char lcd_info[INFO_LEN];
static char tp_info[INFO_LEN];
//Begin <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun
#if defined(CONFIG_PROJECT_L5421)
int sunpanle_enbale = 0;
#endif
//ENE <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun

#if 0
static char main_camera_eeprom_info[INFO_LEN];
static char sub_camera_eeprom_info[INFO_LEN];
#endif

enum {
	MAIN_CAMERA, 
 	SUB_CAMERA, 
};

static int set_dev_info(const char *const str, int which_dev)
{
	int ret;
	ret = 0;

	if(str ==  NULL){
		CDBG("%s : str is NULL\n", __func__);
		return -1;
	}
	CDBG("%s : which_dev=%d\n", __func__, which_dev);
	switch(which_dev){
		case MAIN_CAMERA:
			ret = snprintf(main_camera, INFO_LEN, "%s", str);
			break;
		case SUB_CAMERA:
			ret = snprintf(sub_camera, INFO_LEN, "%s", str);
			break;
		default:
			break;
	}

	return ret;
}

int store_main_camera_info(const char *const str)
{
	CDBG("%s : str=%s\n", __func__, str);
	return set_dev_info(str, MAIN_CAMERA);
}
EXPORT_SYMBOL_GPL(store_main_camera_info);

int store_sub_camera_info(const char *const str)
{
	CDBG("%s : str=%s\n", __func__, str);
	return set_dev_info(str, SUB_CAMERA);
}
EXPORT_SYMBOL_GPL(store_sub_camera_info);

static ssize_t main_camera_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	CDBG("%s : main_camera=%s\n", __func__, main_camera);
	return snprintf(buf, INFO_LEN, "%s\n", main_camera);
}

static ssize_t sub_camera_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	CDBG("%s : sub_camera=%s\n", __func__, sub_camera);
	return snprintf(buf, INFO_LEN, "%s\n", sub_camera);
}

int store_lcd_info(const char *const str)
{
	CDBG("%s : str=%s\n", __func__, str);
	return snprintf(lcd_info, INFO_LEN, "%s", str);
}
EXPORT_SYMBOL_GPL(store_lcd_info);
static ssize_t lcd_info_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	CDBG("%s : lcd_info=%s\n", __func__, lcd_info);
	return snprintf(buf, INFO_LEN, "%s\n", lcd_info);
}

int store_tp_info(const char *const str)
{
	CDBG("%s : str=%s\n", __func__, str);
	return snprintf(tp_info, INFO_LEN, "%s", str);
}
EXPORT_SYMBOL_GPL(store_tp_info);
static ssize_t tp_info_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	CDBG("%s : tp_info=%s\n", __func__, tp_info);
	return snprintf(buf, INFO_LEN, "%s\n", tp_info);
}
//Begin <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun
#if defined(CONFIG_SUN_PANEL_FUNCTION)
static ssize_t sun_panle_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	CDBG("%s : sun_panle=%d\n", __func__, sunpanle_enbale);
	return snprintf(buf, INFO_LEN, "%d\n", sunpanle_enbale);
}
static ssize_t sun_panle_store(struct kobject *kobj, struct kobj_attribute *attr,
		      const char *buf, size_t count)
{
       CDBG("%s : sun_panle=%s\n", __func__, buf);
       sunpanle_enbale = buf[0] - 0x30;
	return count;//snprintf(sun_panle, INFO_LEN, "%s\n",buf );
}


int show_sub_sun_panle(void)
{
       CDBG("%s : sun_panle=%d\n", __func__, sunpanle_enbale);
	return sunpanle_enbale;//(sun_panle[0]!=0x30)?1:0;
}
EXPORT_SYMBOL_GPL(show_sub_sun_panle);
#endif
//Begin <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun

#if 0
int store_sub_camera_eeprom_info(const char *const str)
{
	CDBG("%s : str=%s\n", __func__, str);
	return snprintf(sub_camera_eeprom_info, INFO_LEN, "%s", str);
}
EXPORT_SYMBOL_GPL(store_sub_camera_eeprom_info);
static ssize_t sub_camera_eeprom_info_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	CDBG("%s : sub_camera_eeprom_info=%s\n", __func__, sub_camera_eeprom_info);
	return snprintf(buf, INFO_LEN, "%s\n", sub_camera_eeprom_info);
}

int store_main_camera_eeprom_info(const char *const str)
{
	CDBG("%s : str=%s\n", __func__, str);
	return snprintf(main_camera_eeprom_info, INFO_LEN, "%s", str);
}
EXPORT_SYMBOL_GPL(store_main_camera_eeprom_info);
static ssize_t main_camera_eeprom_info_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	CDBG("%s : tp_info=%s\n", __func__, main_camera_eeprom_info);
	return snprintf(buf, INFO_LEN, "%s\n", main_camera_eeprom_info);
}
#endif

static struct kobj_attribute main_camera_attribute =
	__ATTR(main_camera, 0555, main_camera_show, NULL);

static struct kobj_attribute sub_camera_attribute =
	__ATTR(sub_camera, 0555, sub_camera_show, NULL);

static struct kobj_attribute lcd_info_attribute =
	__ATTR(lcd_info, 0555, lcd_info_show, NULL);

static struct kobj_attribute tp_info_attribute =
	__ATTR(tp_info, 0555, tp_info_show, NULL);

//Begin <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun
#if defined(CONFIG_PROJECT_L5421)
static struct kobj_attribute sun_panle_attribute =
	__ATTR(sun_panle, 0666, sun_panle_show, sun_panle_store);
#endif
//Begin <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun
#if 0
static struct kobj_attribute main_camera_eeprom_info_attribute =
	__ATTR(main_camera_eeprom_info, 0555, main_camera_eeprom_info_show, NULL);

static struct kobj_attribute sub_camera_eeprom_info_attribute =
	__ATTR(sub_camera_eeprom_info, 0555, sub_camera_eeprom_info_show, NULL);
#endif

static struct attribute *attrs[] = {
	&main_camera_attribute.attr,
	&sub_camera_attribute.attr,
	&lcd_info_attribute.attr,
	&tp_info_attribute.attr,
	//Begin <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun
	#if defined(CONFIG_PROJECT_L5421)
	&sun_panle_attribute.attr,
	#endif
	//END <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun
#if 0	
	&main_camera_eeprom_info_attribute.attr,
	&sub_camera_eeprom_info_attribute.attr,	
#endif	
	NULL,
};


static struct attribute_group attr_group = {
	.attrs = attrs,
};

static int __init dev_info_init(void)
{
	int retval;

	dev_info = kobject_create_and_add("dev_info", kernel_kobj);
	if (!dev_info){
		pr_err("%s : kobject_create_and_add failed.\n", __func__);
		return -ENOMEM;
	}

	retval = sysfs_create_group(dev_info, &attr_group);
	if (retval){
		pr_err("%s : sysfs_create_group failed.\n", __func__);
		kobject_put(dev_info);
	}
	return retval;
}

static void __exit dev_info_exit(void)
{
	kobject_put(dev_info);
}

module_init(dev_info_init);
module_exit(dev_info_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("tinno");
