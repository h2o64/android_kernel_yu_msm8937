#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
    #include <linux/pm.h>
    #include <linux/earlysuspend.h>
#endif

#ifdef CONFIG_OF
	#include <linux/of.h>
	#include <linux/of_irq.h>
#endif

#include <linux/miscdevice.h>

#include "../fp_drv/fp_drv.h" //add by yinglong.tang

//LINE<JIRA_ID><DATE20160316><BUG_INFO>zenghaihui
extern int g_fp_match_flag;

#define ELAN_VDD_MIN_UV      2800000
#define ELAN_VDD_MAX_UV	        2800000
#define ELAN_VIO_MIN_UV      1800000
#define ELAN_VIO_MAX_UV      1800000



#define ELAN_FP_NAME "elan_fp"

#define ELAN_CHIP_ID 0x5a3c

#define SPI_ALINE_BYTE 4

#define SPI_MAX_SPEED 3*1000*1000

//#define PRINT_ELAN_INFO
#ifdef PRINT_ELAN_INFO 
	#define elan_info(fmt, args...) do{\
		if(debug_flage)\
                printk("[elan debug]:"fmt"\n", ##args);\
	}while(0);
#else
	#define elan_info(fmt, args...)
#endif

struct elan_fp_platform_data {
	unsigned short version;
	int intr_gpio;
	int rst_gpio;
};

struct elan_fp_data {
	int intr_gpio;
	int rst_gpio;
	int elan_irq;
	struct spi_device *spi;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_fp_wq;
	struct work_struct work;
#ifdef CONFIG_HAS_EARLYSUSPEND	
	struct early_suspend early_suspend;
#endif	
	struct miscdevice elan_fp_mdev;
	char *image_buf;
	char *tx_buf;
	char *rx_buf;
    //LINE<JIRA_ID><DATE20160328><add for power config>zenghaihui
    u8           isPowerOn;
    struct regulator *vdd;
    struct regulator *vio;
};

/*********************************ioctl data********************************************/
#define ELAN_IOCTLID	0xD0
#define IOCTL_RESET	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_IRQ_MASK _IOW(ELAN_IOCTLID, 2, int)
#define IOCTL_READ_MODE _IOW(ELAN_IOCTLID, 3, int)
#define IOCTL_WORK_MODE _IOW(ELAN_IOCTLID, 4, int)
#define IOCTL_SET_XY _IOW(ELAN_IOCTLID, 5, int)
#define IOCTL_SET_SCAN_FLAG _IOW(ELAN_IOCTLID, 6, int)
#define IOCTL_POWER_KEY _IOW(ELAN_IOCTLID, 7, int)
#define IOCTL_SPI_CONFIG _IOW(ELAN_IOCTLID, 8, int)
#define IOCTL_DEBUG_LOG_SWITCH _IOW(ELAN_IOCTLID, 9, int)
#define IOCTL_READ_KEY_STATUS _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_WRITE_KEY_STATUS _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_WAKE_UP_SUSPEND _IOW(ELAN_IOCTLID, 12, int)
#define IOCTL_REMALLOC_IMAGE_BUFFER _IOW(ELAN_IOCTLID, 13, int)

/*********************************global data********************************************/
static struct elan_fp_data *elan_fp = NULL;
static int elan_work_mode = 0;
static int elan_read_mode = 0;
static int elan_interrupt_cnt = 0;
static int image_ok_flag = 0;
static int elan_work_flag = 0;
static bool debug_flage = true;
static int key_status = 0;

static int max_width_pix = 120;
static int max_heigh_pix = 120;
static int total_byte = 120 * 120 * 2;

static DECLARE_WAIT_QUEUE_HEAD(elan_poll_wq);
static DECLARE_WAIT_QUEUE_HEAD(image_waiter);
static DECLARE_WAIT_QUEUE_HEAD(msg_wq);
static struct fasync_struct *fasync_queue = NULL;
static struct wake_lock elan_wake_lock;
static struct wake_lock elan_irq_wake_lock;

/************************************** function list**************************************/

//LINE<JIRA_ID><DATE20160328><BUG_INFO>zenghaihui
static int elan_power_ctl(struct elan_fp_data* gf_dev, bool on)
{
	int rc = 0;

	if (on && (!gf_dev->isPowerOn)) {
		rc = regulator_enable(gf_dev->vdd);
		if (rc) {
			dev_err(&gf_dev->spi->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

#if 1
		rc = regulator_enable(gf_dev->vio);
		if (rc) {
			dev_err(&gf_dev->spi->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(gf_dev->vdd);
			return rc;
		}
#endif
		msleep(10);

		gf_dev->isPowerOn = 1;
	} else if (!on && (gf_dev->isPowerOn)) {

		rc = regulator_disable(gf_dev->vdd);
		if (rc) {
			dev_err(&gf_dev->spi->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

#if 1
		rc = regulator_disable(gf_dev->vio);
		if (rc) {
			dev_err(&gf_dev->spi->dev,
				"Regulator vio disable failed rc=%d\n", rc);
		}
#endif

		gf_dev->isPowerOn = 0;
	} else {
		dev_warn(&gf_dev->spi->dev,
				"Ignore power status change from %d to %d\n",
				on, gf_dev->isPowerOn);
	}
	return rc;
}

static int elan_power_init(struct elan_fp_data* gf_dev)
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
		ret = regulator_set_voltage(gf_dev->vdd, ELAN_VDD_MIN_UV,
					   ELAN_VDD_MAX_UV);
		if (ret) {
			dev_err(&gf_dev->spi->dev,
				"Regulator set_vtg failed vdd ret=%d\n", ret);
			goto reg_vdd_put;
		}
	}

#if 1
	gf_dev->vio = regulator_get(&gf_dev->spi->dev, "vio");
	if (IS_ERR(gf_dev->vio)) {
		ret = PTR_ERR(gf_dev->vio);
		dev_err(&gf_dev->spi->dev,
			"Regulator get failed vio ret=%d\n", ret);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(gf_dev->vio) > 0) {
		ret = regulator_set_voltage(gf_dev->vio,
				ELAN_VIO_MIN_UV,
				ELAN_VIO_MAX_UV);
		if (ret) {
			dev_err(&gf_dev->spi->dev,
			"Regulator set_vtg failed vio ret=%d\n", ret);
			goto reg_vio_put;
		}
	}
#endif

	return 0;

#if 1
reg_vio_put:
	regulator_put(gf_dev->vio);
reg_vdd_set_vtg:
	if (regulator_count_voltages(gf_dev->vdd) > 0)
		regulator_set_voltage(gf_dev->vdd, 0, ELAN_VDD_MAX_UV);
#endif
reg_vdd_put:
	regulator_put(gf_dev->vdd);
	return ret;
}

static int elan_power_deinit(struct elan_fp_data* gf_dev)
{
    int ret = 0;

    if (gf_dev->vdd)
    {   
        if (regulator_count_voltages(gf_dev->vdd) > 0)
            regulator_set_voltage(gf_dev->vdd, 0, ELAN_VDD_MAX_UV);
        
        regulator_disable(gf_dev->vdd);
        regulator_put(gf_dev->vdd);
    }

#if 1
    if (gf_dev->vio)
    {   
        if (regulator_count_voltages(gf_dev->vio) > 0)
            regulator_set_voltage(gf_dev->vio, 0, ELAN_VIO_MAX_UV);
        
        regulator_disable(gf_dev->vio);
        regulator_put(gf_dev->vio);
    }
#endif    

    return ret;
}


static void elan_fp_reset(void)
{
	printk("[elan]:%s enter\n", __func__);
	gpio_set_value(elan_fp->rst_gpio, 0);
	mdelay(5);
	gpio_set_value(elan_fp->rst_gpio, 1);
	mdelay(50);
}

static void elan_fp_switch_irq(int on)
{
	printk("[elan] %s enter, irq = %d, on = %d\n", __func__, elan_fp->elan_irq, on);
	if (on){
		enable_irq(elan_fp->elan_irq);
	}
	else {
		disable_irq(elan_fp->elan_irq);
	}
}

static irqreturn_t elan_fp_irq_handler(int irq, void *dev_id)
{
	elan_info("%s enter\n", __func__);
	//elan_fp_switch_irq(0);
	wake_lock_timeout(&elan_irq_wake_lock, 5*HZ);
	queue_work(elan_fp->elan_fp_wq, &elan_fp->work);
	return IRQ_HANDLED;
}

static void elan_fp_gpio_int_config(struct elan_fp_data *fp)
{
	int err = -1;

	printk("[elan]:%s enter\n", __func__);

	printk("[elan_reset] %d\n", fp->rst_gpio);
	printk("[elan_intr] %d\n", fp->intr_gpio);
	printk("[elan_irq] %d\n", fp->elan_irq);

	//set reset output high
	gpio_request(fp->rst_gpio, "fp_reset");
	gpio_direction_output(fp->rst_gpio, 1);

	//set int pin input
	gpio_request(fp->intr_gpio, "fp_intr");
	gpio_direction_input(fp->intr_gpio);

	err = request_irq(fp->elan_irq, elan_fp_irq_handler, IRQF_TRIGGER_RISING, ELAN_FP_NAME, fp);
	if (err != 0){
		printk("[elan error] %s: request_irq %d failed\n",__func__, fp->elan_irq);
	}
	
	err = enable_irq_wake(fp->elan_irq);
	if (err){
		printk("[elan error] %s: enable_irq_wake %d failed\n",__func__, fp->elan_irq);
	}
}

static int elan_fp_spi_transfer(struct spi_device *spi, const char *txbuf, char *rxbuf, int len)
{
	struct spi_transfer t;
	struct spi_message m;
	
	memset(&t, 0, sizeof(t));
	spi_message_init(&m);
	t.tx_buf = txbuf;
	t.rx_buf = rxbuf;
	t.bits_per_word = 8;
	t.len = len;
	spi_message_add_tail(&t, &m);
	
	return spi_sync(spi, &m);
}

static int elan_fp_alloc_image_buffer(struct elan_fp_data *fp)
{
	int alloc_len = 0;

	if (fp == NULL) {
		printk("[elan error] %s: allocate elan_fp_data failed\n", __func__);
		return -ENOMEM;
	}

	if(fp->image_buf){
		kfree(fp->image_buf);		
	}
	
	//max len + cmd(1) + dummy(1) + 4Byte alignment(MTK DMA mode)
	alloc_len = (total_byte + 2 + (SPI_ALINE_BYTE-1))/ SPI_ALINE_BYTE * SPI_ALINE_BYTE;
	fp->image_buf = kzalloc(sizeof(char)*alloc_len, GFP_KERNEL);
		if (fp->image_buf == NULL) {
		printk("[elan error] %s: allocate image_buf failed\n", __func__);
		return -ENOMEM;
	}

	//max len + cmd(1) + dummy(1) + 4Byte alignment(MTK DMA mode)
	alloc_len = (max_width_pix*2 + 2 + (SPI_ALINE_BYTE-1))/ SPI_ALINE_BYTE * SPI_ALINE_BYTE;

	if(fp->tx_buf){
		kfree(fp->tx_buf);		
	}
	fp->tx_buf = kzalloc(sizeof(char)*alloc_len, GFP_KERNEL);
	if (fp->tx_buf == NULL) {
		printk("[elan error] %s: allocate tx_buf failed\n", __func__);
		return -ENOMEM;
	}
	
	if(fp->rx_buf){
		kfree(fp->rx_buf);		
	}
	fp->rx_buf = kzalloc(sizeof(char)*alloc_len, GFP_KERNEL);
	if (fp->rx_buf == NULL) {
		printk("[elan error] %s: allocate rx_buf failed\n", __func__);
		return -ENOMEM;
	}
	
	return 0;
}

static void elan_fp_free_image_buffer(struct elan_fp_data *fp)
{

	if(fp->image_buf){
		kfree(fp->image_buf);		
	}
    
	if(fp->tx_buf){
		kfree(fp->tx_buf);		
	}
	
	if(fp->rx_buf){
		kfree(fp->rx_buf);		
	}
	
	if(fp){
		kfree(fp);		
	}
}

static int elan_fp_open(struct inode *inode, struct file *filp)
{ 
	elan_info("%s enter", __func__);
	if (elan_fp == NULL){
		printk("elan_fp is NULL~~~");
	}
	return 0;
}

static int elan_fp_release(struct inode *inode, struct file *filp)
{    
	elan_info("%s enter", __func__);
	return 0;
}

ssize_t elan_fp_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
	ssize_t ret = 0;;
	
	elan_info("%s enter", __func__);
	
	if ( elan_read_mode == 1 ){
		//printk("[elan] register read \n");
		ret = copy_to_user(buff, elan_fp->rx_buf, count);
	}
	else{
		ret = wait_event_timeout(image_waiter, image_ok_flag!= 0, 1*HZ);
		if ( ret != 0 ){
			ret = copy_to_user(buff, elan_fp->image_buf, total_byte);
			elan_info("image is readly ");
			ret = 1;
		}
	}
	
	return image_ok_flag;
}

static ssize_t elan_fp_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{
	int ret = 0;
	
	elan_info("%s enter", __func__);
	memset(elan_fp->tx_buf, 0, max_width_pix);
	
   	ret = copy_from_user(elan_fp->tx_buf, buff, count);
	elan_fp_spi_transfer(elan_fp->spi, elan_fp->tx_buf, elan_fp->rx_buf, count);
	
	return 0;
}

static long elan_fp_ioctl( struct file *filp, unsigned int cmd, unsigned long arg)
{
	int buf[8] = {0};
	int ret = 0;
	
	elan_info("%s enter, cmd value %x",__func__, cmd);

	switch (cmd) {
		case IOCTL_RESET:
			elan_fp_reset();
			break;
		case IOCTL_IRQ_MASK:
			elan_fp_switch_irq(arg);
			break;
		case IOCTL_READ_MODE:
			elan_read_mode = arg;
			//printk("[elan] IOCTL_READ_MODE %x \n", elan_read_mode);
			break;
		case IOCTL_WORK_MODE:
			elan_work_mode = arg;
			//printk("[elan] IOCTL_WORK_MODE %x \n", elan_work_mode);
			break;
		case IOCTL_SET_XY:
			ret = copy_from_user(buf, (int *)arg, 2*sizeof(int));
			if(ret)
				break;
			max_width_pix = buf[0];
			max_heigh_pix = buf[1];
			total_byte = max_width_pix * max_heigh_pix * 2;
			//printk("[elan] max_width_pix: %d max_heigh_pix:%d \n", max_width_pix, max_heigh_pix);
			break;
		case IOCTL_SET_SCAN_FLAG:
			image_ok_flag = 0;
			elan_interrupt_cnt = 0;
			break;
		case IOCTL_POWER_KEY:
			input_report_key(elan_fp->input_dev, KEY_POWER, 1);
			input_report_key(elan_fp->input_dev, KEY_POWER, 0);
			input_sync(elan_fp->input_dev);
			break;
		case IOCTL_SPI_CONFIG:
			elan_fp->spi->max_speed_hz = arg;
			//printk("[elan] spi speed %ld \n", arg);
			break;
		case IOCTL_DEBUG_LOG_SWITCH:
			debug_flage = !debug_flage;
			break;
		case IOCTL_READ_KEY_STATUS:
			return key_status;
		case IOCTL_WRITE_KEY_STATUS:
			key_status = arg;
			if (fasync_queue){
				kill_fasync(&fasync_queue, SIGIO, POLL_IN);
			}
			break;
		case IOCTL_WAKE_UP_SUSPEND:
			if(arg){
				wake_lock(&elan_wake_lock);
			}
			else{
				wake_unlock(&elan_wake_lock);
			}
			break;
		case IOCTL_REMALLOC_IMAGE_BUFFER:
			//printk("[elan] reallco image buffer ! \n");
			elan_fp_alloc_image_buffer(elan_fp);
			break;
		default:
			break;
	}
	return ret;
}

static unsigned int elan_fp_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	
	poll_wait(file, &elan_poll_wq, wait);
	
	if(elan_work_flag > 0){
		mask = elan_work_flag;
	}
	
	elan_work_flag = 0;
	return mask;
}

static int elan_fp_fasync(int fd, struct file * filp, int on)
{
	//printk("%s enter \n",__func__);
	return fasync_helper(fd, filp, on, &fasync_queue);
}

static struct file_operations elan_fp_fops = {    
	.open = elan_fp_open,  
	.release =  elan_fp_release,
	.read = elan_fp_read,
	.write = elan_fp_write,
	.unlocked_ioctl = elan_fp_ioctl, 
	.compat_ioctl = elan_fp_ioctl,
	.poll = elan_fp_poll,
	.fasync = elan_fp_fasync,
};

static int elan_fp_suspend(struct spi_device *spi, pm_message_t mesg)
{
	printk("[elan]:%s enter\n", __func__);
	
	return 0;
}

static int elan_fp_resume(struct spi_device *spi)
{
	printk("[elan]:%s enter\n", __func__);
	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_fp_early_suspend(struct early_suspend *h)
{
	elan_fp_suspend(elan_fp->spi, PMSG_SUSPEND);
}

static void elan_fp_late_resume(struct early_suspend *h)
{
	elan_fp_resume(elan_fp->spi);
}
#endif

static void elan_fp_work(struct work_struct *work)
{
	int len = max_width_pix*2+2;
	
	elan_fp->tx_buf[0] = 0x10;

recv_one_frame_image:
	if (elan_work_mode & 0x01 || elan_work_mode & 0x08){
		elan_info("elan_fp_work ->woe mode WAKEUP or frame recve !\n");	
	}
	else{
		if (elan_work_mode & 0x02){
			//adc 8bit mode
			len = max_width_pix + 2;
			elan_fp_spi_transfer(elan_fp->spi, elan_fp->tx_buf, elan_fp->rx_buf, len);
		}
		else{
			//adc 14bit mode
			elan_fp_spi_transfer(elan_fp->spi, elan_fp->tx_buf, elan_fp->rx_buf, len);
		}
		//copy data to image, sheet 2byte(cmd+dummy)
		if (elan_interrupt_cnt < max_heigh_pix){
			memcpy(&elan_fp->image_buf[(len-2)*elan_interrupt_cnt], &elan_fp->rx_buf[2], len-2);
		}
	}
	
	if( elan_work_mode & 0x01 ){
		elan_interrupt_cnt = 0;
		elan_work_flag = 1;
		wake_up(&elan_poll_wq);
	}
	else{
		elan_interrupt_cnt++;
		elan_info("elan_interrupt_cnt = %d", elan_interrupt_cnt);
		
		if( (elan_interrupt_cnt == max_heigh_pix) ){
			elan_interrupt_cnt = 0;
			image_ok_flag = 1;
			wake_up(&image_waiter);
		}
		else if(elan_work_mode & 0x04){
			//one frame image mode
			goto recv_one_frame_image;
		}
		else if(elan_work_mode & 0x08){
			elan_info("recv alll pix in one transfer!\n");
			elan_fp->tx_buf[0] = 0x10;
			len = (max_width_pix*2+1)*max_heigh_pix+1;
			elan_fp_spi_transfer(elan_fp->spi, elan_fp->tx_buf, elan_fp->image_buf, len);
			for(elan_interrupt_cnt = 0; elan_interrupt_cnt < max_heigh_pix; elan_interrupt_cnt++){
				if(elan_interrupt_cnt == 0){         		
					memcpy(elan_fp->image_buf, &elan_fp->image_buf[2], max_width_pix*2);
				}
				else{						
					memcpy(&elan_fp->image_buf[max_width_pix*2*elan_interrupt_cnt], \
						&elan_fp->image_buf[(max_width_pix*2+1)*elan_interrupt_cnt + 2], max_width_pix*2);
				}
    		}
			elan_interrupt_cnt = 0;
			image_ok_flag = 1;
			wake_up(&image_waiter);
		}
	}
	
	//elan_fp_switch_irq(1);
}

static int elan_fp_detect_id(void)
{
	char *cmd = elan_fp->tx_buf;
	int i = 0;
	uint16_t chip_id = 0;

	//write to register
	cmd[0] = 0x80;
	cmd[1] = 0x5A;
	cmd[2] = 0x81;
	cmd[3] = (ELAN_CHIP_ID) >> 8;
	cmd[4] = 0x82;
	cmd[5] = (ELAN_CHIP_ID) & 0x00FF;
	for(i=0; i<6; i+=2)
		elan_fp_spi_transfer(elan_fp->spi, cmd+i, NULL, 2);

	//read register
	memset(elan_fp->tx_buf, 0, max_width_pix);
	cmd[0] = 0xC1;
	elan_fp_spi_transfer(elan_fp->spi, cmd, elan_fp->rx_buf, 4);
	
	for(i = 0; i < 4; i++)
		printk("%x ", elan_fp->rx_buf[i]);
	printk("\n");
	chip_id = (elan_fp->rx_buf[2] << 8) + elan_fp->rx_buf[3];

	if(chip_id == ELAN_CHIP_ID){
		printk("[elan] ELAN_CHIP_ID = %x detect ok \n", chip_id);
		cmd[0] = 0x80+0x2A;
		cmd[1] = 0x00;
		elan_fp_spi_transfer(elan_fp->spi, cmd, NULL, 2);
		printk("[elan] set power down ok \n");
		return 0;
	}
	else{
		printk("[elan] ELAN_CHIP_ID = %x detect error \n", chip_id);
		return -1;
	}
}

#ifdef CONFIG_OF
static int elan_fp_parse_dt(struct device *dev, struct elan_fp_platform_data *pdata)
{
    //int rc;
    struct device_node *np = dev->of_node;

#if 1
    /* reset, irq gpio info */

    pdata->rst_gpio = of_get_named_gpio(np,"qcom,reset-gpio",0);
    
    if (pdata->rst_gpio < 0){
        return pdata->rst_gpio;
    }
    pdata->intr_gpio = of_get_named_gpio(np,"qcom,irq-gpio",0);
    
    if (pdata->intr_gpio < 0){
        return pdata->intr_gpio;
    }
#endif
	
    return 0;
}
#endif


void elan_gpio_cleanup(struct elan_fp_data *fp)
{
	pr_info("[info] %s\n",__func__);
	if (gpio_is_valid(fp->intr_gpio))
	{
		gpio_free(fp->intr_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(fp->rst_gpio))
	{
		gpio_free(fp->rst_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

static int elan_fp_probe(struct spi_device *spi)
{
	struct elan_fp_data *fp = NULL;
	struct elan_fp_platform_data *pdata = spi->dev.platform_data;
	int err = 0;

	printk("[elan]:%s enter\n", __func__);


    //LINE<JIRA_ID><DATE20160316><BUG_INFO>zenghaihui
    if(1 == g_fp_match_flag)
    {
        pr_info("[elan]:%s   --fingerpirnt have been matched!!! \n", __func__);
        return -ENODEV;
    }


	spi->bits_per_word = 8;
	
	fp = kzalloc(sizeof(struct elan_fp_data), GFP_KERNEL);
	if (fp == NULL) {
		printk("[elan error] %s: allocate elan_fp_data failed\n", __func__);
		return -ENOMEM;
	}
	err = elan_fp_alloc_image_buffer(fp);
	if(err != 0){
		//elan_fp_free_image_buffer(fp);
		//return -ENOMEM;
            printk("[elan error] %s: allocate image buffer failed\n", __func__);
            goto alloc_image_buffer_fail;
	}
	
	fp->spi = spi;
	elan_fp = fp;

	
	fp->elan_fp_mdev.minor = MISC_DYNAMIC_MINOR;	
	fp->elan_fp_mdev.name = "elan_fp";
	fp->elan_fp_mdev.fops = &elan_fp_fops;
	fp->elan_fp_mdev.mode = S_IFREG|S_IRWXUGO; 
	if (misc_register(&fp->elan_fp_mdev) < 0)
	{
		printk("[elan] misc_register failed!!\n");
            goto misc_register_fail;
       }
	else
		printk("[elan] misc_register ok!!\n");
	
	//borad file config
	if (pdata != NULL) {
		printk("[elan] borad file config mode !! \n");
	}
#ifdef CONFIG_OF
	//dts file config
	else if (spi->dev.of_node) {
		printk("[elan] dts file config mode !! \n");		
		pdata = kzalloc(sizeof(struct elan_fp_platform_data), GFP_KERNEL);
        if (!pdata) {
            printk("[elan error] Failed to allocate elan_fp_platform_data memory\n");
            //return -ENOMEM;
            goto elan_fp_platform_data_fail;
        }
        
        err = elan_fp_parse_dt(&spi->dev, pdata);
        if (err) {
            printk("[elan error] DT parsing failed\n");
            //return -EINVAL;
            goto node_parse_fail;
        }
	}
#endif
	//user config
    else{
        printk("[elan error] user config error !!\n");
        //return -EINVAL;
        goto node_parse_fail;
    }


    //LINE<JIRA_ID><DATE20160328><BUG_INFO>zenghaihui
        
    err = elan_power_init(fp);
    if (err) {
        dev_err(&spi->dev, "Failed to init regulator\n");
        goto node_parse_fail;
    }
    fp->isPowerOn = false;
    err = elan_power_ctl(fp, true);
    if (err) {
        dev_err(&spi->dev, "Failed to power on device\n");
        goto elan_deinit_regulator;
    }
        


	if(pdata){
		fp->rst_gpio = pdata->rst_gpio;
        fp->intr_gpio = pdata->intr_gpio;
        fp->elan_irq = gpio_to_irq(fp->intr_gpio);
	}
	else{
		printk("[elan error] pdata is NULL !!! \n");
        //return -EINVAL;		
        goto node_parse_fail;
	}
	
	elan_fp_gpio_int_config(fp);
	printk("[elan] SPI max speed %d \n", spi->max_speed_hz);

	err = elan_fp_detect_id();
	if(err != 0){
		//elan_fp_free_image_buffer(fp);
		//return -ENODEV;		
            printk("[elan] chip match failed \n");
            goto elan_chip_match_fail;
	}

    
	INIT_WORK(&elan_fp->work, elan_fp_work);
	fp->elan_fp_wq = create_singlethread_workqueue(ELAN_FP_NAME); 
	if (!fp->elan_fp_wq) {
		printk("[elan error] %s: create_singlethread_workqueue failed\n", __func__);
		//return -EINVAL;
            goto create_thread_workqueue_fail;
	}
	printk("[elan]:%s elan_fp_gpio_int_config ok \n", __func__);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	fp->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	fp->early_suspend.suspend = elan_fp_early_suspend;
	fp->early_suspend.resume = elan_fp_late_resume;
	register_early_suspend(&fp->early_suspend);
#endif

	fp->input_dev = input_allocate_device();
    if (fp->input_dev == NULL) {
        printk("[elan error] Failed to allocate input device\n");
        //return -EINVAL;;
        goto input_allocate_fail;
    }

	fp->input_dev->evbit[0] = BIT(EV_KEY)|BIT_MASK(EV_REP);
	__set_bit(KEY_POWER, fp->input_dev->keybit);
	
	fp->input_dev->name = ELAN_FP_NAME;
	fp->input_dev->phys = "input/fp"; 
	fp->input_dev->id.bustype = BUS_SPI; 
	fp->input_dev->id.vendor = 0xDEAD; 
	fp->input_dev->id.product = 0xBEEF; 
	fp->input_dev->id.version = 2015;

	err = input_register_device(fp->input_dev);
	if (err) {
		input_free_device(fp->input_dev);
		printk("[elan error]%s: unable to register %s input device\n", __func__, fp->input_dev->name);
		//return err;
            goto input_register_device_fail;
	}
	
	wake_lock_init(&elan_wake_lock, WAKE_LOCK_SUSPEND, "elan_wake_lock");
	wake_lock_init(&elan_irq_wake_lock, WAKE_LOCK_SUSPEND, "elan_irq_wake_lock");
	printk("[elan]:++++++++++%s OK end ++++++++++++ \n", __func__);
	
    //LINE<JIRA_ID><DATE20160316><BUG_INFO>zenghaihui
    g_fp_match_flag = 1;

	full_fp_chip_name(ELAN_FP_NAME);//add by yinglong.tang.
    
	return 0;

    
input_register_device_fail:
    input_free_device(fp->input_dev);
    
input_allocate_fail:
    destroy_workqueue(fp->elan_fp_wq);
    
create_thread_workqueue_fail:
elan_chip_match_fail:
    disable_irq(fp->elan_irq);
    free_irq(fp->elan_irq, fp);
        
elan_deinit_regulator:
    elan_power_deinit(fp);
        
node_parse_fail:
    if(pdata)
    {
        kfree(pdata);
        pdata = NULL;
    }
    
elan_fp_platform_data_fail:
    misc_deregister(&fp->elan_fp_mdev);
    
misc_register_fail:
alloc_image_buffer_fail:
    elan_gpio_cleanup(fp);
    
    elan_fp_free_image_buffer(fp);
    fp = NULL;
    
    printk("[elan]:%s failed exit \n", __func__);
    
    return -ENODEV;
    
}

static int elan_fp_remove(struct spi_device *spi)
{
	printk("[elan]:%s enter\n", __func__);
        elan_power_deinit(elan_fp);
	return 0;
}

static const struct spi_device_id efp_id[] = {
	{ELAN_FP_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(spi, efp_id);

#ifdef CONFIG_OF
static struct of_device_id efsa120s_of_match[] = {
	{ .compatible = "qcom,fingerprint", },
	{}
};

MODULE_DEVICE_TABLE(of, efsa120s_of_match);
#endif

static struct spi_driver efp_driver = {
	.driver = {
		.name = ELAN_FP_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = efsa120s_of_match,
#endif		
	},
	.probe 	= elan_fp_probe,
	.remove = elan_fp_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = elan_fp_suspend,
	.resume = elan_fp_resume,
#endif
	.id_table = efp_id,
	
};

#if 0
static struct elan_fp_platform_data elan_fp_config = {
	.version 		= 0x0001,
	//.intr_gpio 		= S5PV210_GPH0(6),
	//.rst_gpio 		= S5PV210_GPH0(3),
};

static struct spi_board_info elan_fp_spi_devs[] __initdata = {
	{
		.modalias		= ELAN_FP_NAME,
		.bus_num 		= 0,
		.chip_select 	= 0,
		.max_speed_hz	= SPI_MAX_SPEED,
		.mode 			= SPI_MODE_0,
		.platform_data 	= &elan_fp_config,
	},
};
#endif

static int __init elan_fp_init(void)
{
    int status;
    
	printk("[elan]:%s enter\n", __func__);

    //LINE<JIRA_ID><DATE20160316><BUG_INFO>zenghaihui
    if(1 == g_fp_match_flag)
    {
        pr_info("[elan]:%s   --fingerpirnt have been matched!!! \n", __func__);
        return -ENODEV;
    }

	//spi_register_board_info(elan_fp_spi_devs, ARRAY_SIZE(elan_fp_spi_devs));
	
	//return spi_register_driver(&efp_driver);
	status =  spi_register_driver(&efp_driver);
    
    printk("[elan]:%s spi_register_driver status = %d \n",  __func__, status);

    return status;
}

static void __exit elan_fp_exit(void)
{
	printk("[elan]:%s enter\n", __func__);
	spi_unregister_driver(&efp_driver);
}

module_init(elan_fp_init);
module_exit(elan_fp_exit);

MODULE_DESCRIPTION("elan finger print driver");
MODULE_LICENSE("GPL");
