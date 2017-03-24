#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/usb.h>
//#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/power_supply.h>

#include <linux/gpio.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/completion.h>
#include "gf-spi.h"

#include "../fp_drv/fp_drv.h" //add by yinglong.tang

/*spi device name*/
#define SPI_DEV_NAME   "spidev"
/*device name after register in charater*/
#define DEV_NAME "goodix_fp"
#define GF_INPUT_NAME       "goodix_fp"
/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define	CHRD_DRIVER_NAME			"goodix"
#define	CLASS_NAME			    	"goodix-spi"
#define SPIDEV_MAJOR				154	/* assigned */
#define N_SPI_MINORS				32	/* ... up to 256 */
static DECLARE_BITMAP(minors, N_SPI_MINORS);


#if defined GF_X16M_TARGET_FW
	#if 1 // FW_UPDATE
	#define GF_PID "GFx16M"
	#endif
	#if (NAVIGATION == 1)
	#define GF_NAV_FRAME_LEN         (1680)    //navigation data
	#endif
#elif defined GF_X18M_TARGET_FW
	#if FW_UPDATE
	#define GF_PID "GFx18M"
	#endif
	#if (NAVIGATION == 1)
	#define GF_NAV_FRAME_LEN        (2160)     //navigation data len
	#endif
#else
#error "have't define any product ID"
#endif

/**************************debug******************************/
#define SPI_ASYNC   1

#define DEFAULT_DEBUG   (0x1<<0)
#define SUSPEND_DEBUG   (0x1<<1)
#define SPI_DEBUG       (0x1<<2)
#define TIME_DEBUG      (0x1<<3)
#define FLOW_DEBUG      (0x1<<4)
	
#define gf_debug(level, fmt, args...) do{ \
    if(/*g_debug & level*/1) {\
	pr_info("goodix gfx1xm  " fmt, ##args); \
    } \
}while(0)

#define FUNC_ENTRY()  gf_debug(FLOW_DEBUG, "gfx1xm:%s, entry\n", __func__)
#define FUNC_EXIT()  gf_debug(FLOW_DEBUG,"gfx1xm:%s, exit\n", __func__)

struct gf_key_map
{
    char *name;
    unsigned short val;
};

struct gf_key_map key_map[] = 
{
      {  "POWER",  KEY_POWER  },
      {  "HOME" ,  KEY_HOME   },
      {  "MENU" ,  KEY_MENU   },
      {  "BACK" ,  KEY_BACK   },
      #if (NAVIGATION == 1)
      {  "UP"   ,  KEY_UP     },
      {  "DOWN" ,  KEY_DOWN   },
      {  "LEFT" ,  KEY_LEFT   },
      {  "RIGHT",  KEY_RIGHT  },
      {  "FORCE",  KEY_F9     },
      {  "CLICK",  KEY_F19    },
      #endif
};

#define GF_KEY_POWER        KEY_POWER
//#define GF_KEY_HOME     KEY_HOME
#define GF_KEY_HOME     KEY_HOMEPAGE
#define GF_KEY_MENU     KEY_MENU
#define GF_KEY_BACK     KEY_BACK
#if (NAVIGATION == 1)
#define GF_UP_KEY       KEY_UP
#define GF_DOWN_KEY KEY_DOWN
#define GF_LEFT_KEY KEY_LEFT
#define GF_RIGHT_KEY    KEY_RIGHT
#define GF_KEY_FORCE    KEY_F9
#define GF_APP_SWITCH   KEY_F19
#endif

//LINE<JIRA_ID><DATE20160319><merge from P6601>zenghaihui       
//guomingyi.
static int ftm_gfx_irq_state = 0;

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
/*************************data stream***********************
 *	FRAME NO  | RING_H  | RING_L  |  DATA STREAM | CHECKSUM |
 *     1B      |   1B    |  1B     |    2048B     |  2B      |
 ************************************************************/
static unsigned bufsiz = 8 * (2048+5);
//unsigned long g_debug = DEFAULT_DEBUG;//FLOW_DEBUG;
static unsigned char g_mode_switch_flag = 0;


//LINE<JIRA_ID><DATE20160316><BUG_INFO>zenghaihui
int g_fp_match_flag = 0;
EXPORT_SYMBOL_GPL(g_fp_match_flag);

static int suspend_test_flag = 0;
//add by yinglong.tang
#define SCREEN_ON 1
#define SCREEN_OFF 0
static int gf_screen_status = SCREEN_ON;
//add by yinglong.tang

#define GF_PID_LEN 	6

#if FW_UPDATE

static unsigned char GF_FW[]=
{
#if defined GF_X16M_TARGET_FW
#include "inc/gf_fw_x16M.i"
#elif defined GF_X18M_TARGET_FW
#include "inc/gf_fw_x18M.i"
#else
#error "have't define any product for fw"
#endif
};
#define FW_LENGTH (42*1024)
unsigned char* fw = GF_FW;
#endif

#if CFG_UPDATE
static unsigned char GF_CFG[]=
{
#if defined GF_318M_TARGET_CFG
#include "inc/gf318m_cfg.i"
#elif defined GF_316M_TARGET_CFG
#include "inc/gf316m_cfg.i"
#elif defined GF_516M_TARGET_CFG
#include "inc/gf516m_cfg.i"
#elif defined GF_518M_TARGET_CFG
#include "inc/gf518m_cfg.i"
#elif defined GF_816M_TARGET_CFG
#include "inc/gf816m_cfg.i"
#else
#error "have't define any product for cfg"
#endif
};
unsigned char* cfg = GF_CFG;
#endif
struct wake_lock fg_wake_lock;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

//add by yinglong.tang
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data);

static void gf_configure_sleep(struct gf_dev *gf_dev_fb)
{
	int retval = 0;

	gf_dev_fb->notifier.notifier_call = fb_notifier_callback;

	retval = fb_register_client(&gf_dev_fb->notifier);
	if (retval)
		dev_err(&gf_dev_fb->spi->dev,
			"Unable to register notifier: %d\n", retval);
}

static int gf_suspend_fb(void) {
	printk(" goodix gfx1xm  %s======SCREEN_OFF=========.\n", __func__);
	gf_screen_status = SCREEN_OFF;
	return 0;
}
static int gf_resume_fb(void) {
	printk(" goodix gfx1xm  %s======SCREEN_ON=========.\n", __func__);
	gf_screen_status = SCREEN_ON;
	return 0;
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct gf_dev *gf_dev_fb =
		container_of(self, struct gf_dev, notifier);

	if (evdata && evdata->data && gf_dev_fb && gf_dev_fb->spi) {
             if (event == FB_EVENT_BLANK) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK)
				gf_resume_fb();
			else if (*blank == FB_BLANK_POWERDOWN)
				gf_suspend_fb() ;
		}
	}

	return 0;
}

#endif
//add by yinglong.tang



void print_16hex(u8 *config, u8 len)
{
#if 0
    u8 i,j = 0;
    printk("goodix gfx1xm  dump hex \n");
    for(i = 0 ; i< len ; i++) {
        printk("goodix gfx1xm  0x%x " , config[i]);
        if(j++ == 15) {
            j = 0;
            printk("\n");
        }
    }
    printk("\n");
#endif    
}

#if 0//(NAVIGATION == 1)
unsigned short checksum(u8 *data, u32 len)
{
    u16 checksum = 0;
    u32 i = 0;         
    for(i =0; i<len; i++) {
        checksum += data[i];
    }
    return checksum;
}
#endif

/* -------------------------------------------------------------------- */
/* devfs                                */
/* -------------------------------------------------------------------- */
#if 0
static ssize_t gf_debug_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
    printk("goodix gfx1xm  Show.\n");
    return 0;
}
static ssize_t gf_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int debug_level = 0;
    sscanf(buf, "%d", &debug_level);
    printk("goodix gfx1xm  Store. debug_level = %d\n", debug_level);
    return strnlen(buf, count);
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, gf_debug_show, gf_debug_store);

static struct attribute *gf_debug_attrs[] = {
    &dev_attr_debug_gf.attr,
    NULL
};

static const struct attribute_group gf_debug_attr_group = {
    .attrs = gf_debug_attrs,
    .name = "debug"
};
#endif

static void gf_enable_irq(struct gf_dev *gf_dev)
{
        if (gf_dev->irq_enabled) {
                pr_warn("goodix gfx1xm  IRQ has been enabled.\n");
        } else {
                enable_irq(gf_dev->spi->irq);
                gf_dev->irq_enabled = 1;
        }
}

static void gf_disable_irq(struct gf_dev *gf_dev)
{
        if (gf_dev->irq_enabled) {
                gf_dev->irq_enabled = 0;
                disable_irq(gf_dev->spi->irq);
        } else {
                pr_warn("goodix gfx1xm  IRQ has been disabled.\n");
        }
}


#ifdef SPI_ASYNC
static void gf_spi_complete(void *arg)
{
    complete(arg);
}
#endif //SPI_ASYNC

void gf_spi_setup(struct gf_dev *gf_dev, int max_speed_hz)
{

    gf_dev->spi->mode = SPI_MODE_0; //CPOL=CPHA=0
    gf_dev->spi->max_speed_hz = max_speed_hz; 
    gf_dev->spi->irq = gf_irq_num(gf_dev);
    gf_dev->spi->bits_per_word = 8;
    spi_setup(gf_dev->spi);
}

/**********************************************************
 *Message format:
 *	write cmd   |  ADDR_H |ADDR_L  |  data stream  |
 *    1B         |   1B    |  1B    |  length       |
 *
 * read buffer length should be 1 + 1 + 1 + data_length
 ***********************************************************/
int gf_spi_write_bytes(struct gf_dev *gf_dev,
	u16 addr, u32 data_len, u8 *tx_buf)
{
#ifdef SPI_ASYNC
    DECLARE_COMPLETION_ONSTACK(read_done);
#endif
    struct spi_message msg;
    struct spi_transfer *xfer;
    int ret = 0;

    xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
    if( xfer == NULL){
	pr_warn("goodix gfx1xm  No memory for command.\n");
	return -ENOMEM;
    }

    /*send gf command to device.*/
    spi_message_init(&msg);
    tx_buf[0] = GF_W;
    tx_buf[1] = (u8)((addr >> 8)&0xFF);
    tx_buf[2] = (u8)(addr & 0xFF);
    xfer[0].tx_buf = tx_buf;
    xfer[0].len = data_len + 3;
    xfer[0].delay_usecs = 5;
    spi_message_add_tail(xfer, &msg);
#ifdef SPI_ASYNC
    msg.complete = gf_spi_complete;
    msg.context = &read_done;

    spin_lock_irq(&gf_dev->spi_lock);
    ret = spi_async(gf_dev->spi, &msg);
    spin_unlock_irq(&gf_dev->spi_lock);
    if(ret == 0) {
	wait_for_completion(&read_done);
	if(msg.status == 0)
	    ret = msg.actual_length - GF_WDATA_OFFSET;
    }
#else
    ret = spi_sync(gf_dev->spi, &msg);
    if(ret == 0) {
	ret = msg.actual_length - GF_WDATA_OFFSET;
    }
#endif
    //gf_debug(SPI_DEBUG, "ret = %d, actual_length = %d\n", ret, msg.actual_length);
    kfree(xfer);
    if(xfer != NULL)
	xfer = NULL;

    return ret;
}

/*************************************************************
 *First message:
 *	write cmd   |  ADDR_H |ADDR_L  |
 *    1B         |   1B    |  1B    |
 *Second message:
 *	read cmd   |  data stream  |
 *    1B        |   length    |
 *
 * read buffer length should be 1 + 1 + 1 + 1 + data_length
 **************************************************************/
int gf_spi_read_bytes(struct gf_dev *gf_dev,
	u16 addr, u32 data_len, u8 *rx_buf)
{
#ifdef SPI_ASYNC
    DECLARE_COMPLETION_ONSTACK(write_done);
#endif //SPI_ASYNC
    struct spi_message msg;
    struct spi_transfer *xfer;
    int ret = 0;

    xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
    if( xfer == NULL){
	pr_warn("goodix gfx1xm  No memory for command.\n");
	return -ENOMEM;
    }

    /*send gf command to device.*/
    spi_message_init(&msg);
    rx_buf[0] = GF_W;
    rx_buf[1] = (u8)((addr >> 8)&0xFF);
    rx_buf[2] = (u8)(addr & 0xFF);
    xfer[0].tx_buf = rx_buf;
    xfer[0].len = 3;
    xfer[0].delay_usecs = 5;
    spi_message_add_tail(&xfer[0], &msg);

    /*if wanted to read data from gf. 
     *Should write Read command to device
     *before read any data from device.
     */
    //memset(rx_buf, 0xff, data_len);
    spi_sync(gf_dev->spi, &msg);
    spi_message_init(&msg);
    rx_buf[4] = GF_R;
    xfer[1].tx_buf = &rx_buf[4];

    xfer[1].rx_buf = &rx_buf[4];
    xfer[1].len = data_len + 1;
    xfer[1].delay_usecs = 5;

    spi_message_add_tail(&xfer[1], &msg);

#ifdef SPI_ASYNC
    msg.complete = gf_spi_complete;
    msg.context = &write_done;

    spin_lock_irq(&gf_dev->spi_lock);
    ret = spi_async(gf_dev->spi, &msg);
    spin_unlock_irq(&gf_dev->spi_lock);
    if(ret == 0) {
	wait_for_completion(&write_done);
	if(msg.status == 0)
	  return data_len;
    }
#else
    ret = spi_sync(gf_dev->spi, &msg);
    if(ret == 0){
        ret = data_len;
    }
#endif
    //gf_debug(SPI_DEBUG, "ret = %d, actual_length = %d\n", ret, msg.actual_length);
    kfree(xfer);
    if(xfer != NULL)
	xfer = NULL;
	
    return ret;
}

static int gf_spi_read_byte(struct gf_dev *gf_dev, u16 addr, u8 *value)
{
    int status = 0;
    mutex_lock(&gf_dev->buf_lock);

    status = gf_spi_read_bytes(gf_dev, addr, 1, gf_dev->buffer);
    *value = gf_dev->buffer[GF_RDATA_OFFSET];
    //gf_debug(SPI_DEBUG, "value = 0x%x, buffer[3] = 0x%x\n", *value, gf_dev->buffer[3]);
    mutex_unlock(&gf_dev->buf_lock);
    return status;
}
static int gf_spi_write_byte(struct gf_dev *gf_dev, u16 addr, u8 value)
{
    int status = 0;
    mutex_lock(&gf_dev->buf_lock);
    gf_dev->buffer[GF_WDATA_OFFSET] = value;
    status = gf_spi_write_bytes(gf_dev, addr, 1, gf_dev->buffer);
    mutex_unlock(&gf_dev->buf_lock);
    return status;
}

/**************************nagv***********************************/
#if (NAVIGATION == 1)
int get_nav_buf(struct gf_dev *gf_dev, struct gf_nagv *gf_nagv)
{
    unsigned char lbstatus = 0;
    int ret = 0;
    unsigned char frameNum = 0;
    unsigned char pNavFrameNum;

    if(gf_nagv->buf == NULL) {
        pr_info("goodix gfx1xm  get_package_data buf is NULL.\n");
        return -EINVAL;
    }
    /* read status and frame num buffered*/
    gf_spi_read_byte(gf_dev, GF_LBSTATUS_ADDR, &lbstatus);
    gf_spi_read_byte(gf_dev, GF_NAV_FRAMENUM_ADDR, &pNavFrameNum);
    pr_info("goodix gfx1xm  NAV: lbstatus: 0x%x, frame_num:0x%x\n", lbstatus, pNavFrameNum);

    frameNum = pNavFrameNum & 0x0F;
    if(frameNum)
    {/* read navigation data */
        pr_info("goodix gfx1xm  NAV: read %d frame data.\n", frameNum);
        mutex_lock(&gf_dev->buf_lock);
        gf_dev->spi->max_speed_hz=4*1000*1000;
        spi_setup(gf_dev->spi);
        if(lbstatus & GF_NAV_BUF_MASK)
            ret = gf_spi_read_bytes(gf_dev, GF_NAV_BUF1_ADDR, (frameNum*GF_NAV_FRAME_LEN), gf_dev->buffer);
        else
            ret = gf_spi_read_bytes(gf_dev, GF_NAV_BUF0_ADDR, (frameNum*GF_NAV_FRAME_LEN), gf_dev->buffer);

        gf_dev->spi->max_speed_hz=1*1000*1000;
        spi_setup(gf_dev->spi);
        mutex_unlock(&gf_dev->buf_lock);

        if(!ret) {
            pr_info("goodix gfx1xm  NAV:Failed to read data. len = %d. ret = %d, lbstatus = %d, nav_frm_num = %d\n",
                               GF_NAV_FRAME_LEN, ret, lbstatus, pNavFrameNum);
                return -EFAULT;
        }
/*
        unsigned int checksum16 = 0;
        unsigned char ck_r10 = 0;
        unsigned char ck_r11 = 0;
        unsigned char ck_r20 = 0;
        unsigned char ck_r21 = 0;

        checksum16 = checksum((unsigned char*)(gf_dev->buffer + GF_RDATA_OFFSET), 
                            (unsigned int)(GF_NAV_FRAME_LEN*frameNum));
        gf_spi_read_byte(gf_dev, 0x9A9A, &ck_r10);
        gf_spi_read_byte(gf_dev, 0x9A9B, &ck_r11);
        gf_spi_read_byte(gf_dev, 0xB49A, &ck_r20);
        gf_spi_read_byte(gf_dev, 0xB49B, &ck_r21);
        pr_info("goodix gfx1xm  NAV: checksum:0x%x, ck_r1:[0x%x-0x%x], ck_r2:[0x%x-0x%x].\n", checksum16, ck_r10, ck_r11, ck_r20, ck_r21);
*/     
        if(copy_to_user(gf_nagv->buf, (gf_dev->buffer + GF_RDATA_OFFSET), GF_NAV_FRAME_LEN*frameNum)){
            pr_err("goodix gfx1xm  Failed to copy data from kernel to user.\n");
            return -EFAULT;
        }
        if(copy_to_user(gf_nagv->frame_num, &pNavFrameNum, 1)){
            pr_err("goodix gfx1xm  Failed to copy data from kernel to user.\n");
            return -EFAULT;
        }
    }
    else
    {
        pr_info("goodix gfx1xm  NAV: no data to read.\n");
    }
 
    /*tell the hardware. Data has been read already.*/
    gf_spi_write_byte(gf_dev, GF_BUFFER_STATUS, 0);
    return 0;
}


int gf_read_nav_base(struct gf_dev *gf_dev, struct gf_nagv *gf_nagv)
{
    unsigned char status = 0;
    unsigned char lbstatus = 0;
    int retry = 0;
    int ret = 0;
    if(gf_nagv->buf == NULL)  {
        pr_info("goodix gfx1xm  get_nav_base is NULL.\n");
        return -EINVAL;
    }


    disable_irq(gf_dev->spi->irq);
    gf_spi_write_byte(gf_dev, GF_MODE_STATUS, GF_PRENAV_MODE);
    do
    {
        /* read status and frame num buffered*/
        gf_spi_read_byte(gf_dev, GF_BUFFER_STATUS, &status);
        gf_spi_read_byte(gf_dev, GF_LBSTATUS_ADDR, &lbstatus);
        //pr_info("goodix gfx1xm  NAV: retry: %d, status: 0x%x, lbstatus:0x%x\n", retry, status, lbstatus);

        if( (status & GF_BUF_STA_MASK) && (lbstatus & GF_NAV_MASK))
        {/* read navigation base */
            mutex_lock(&gf_dev->buf_lock);
            ret = gf_spi_read_bytes(gf_dev, GF_NAV_BUF0_ADDR, GF_NAV_FRAME_LEN, gf_dev->buffer);
            mutex_unlock(&gf_dev->buf_lock);

            if(!ret) {
                pr_info("goodix gfx1xm  NAV:Failed to read nav base. len = %d. ret = %d\n", GF_NAV_FRAME_LEN, ret);
                gf_spi_write_byte(gf_dev, GF_BUFFER_STATUS, status&0x7F);
                enable_irq(gf_dev->spi->irq);
                return -EFAULT;
            }
            else
            {
                gf_spi_write_byte(gf_dev, GF_PAUSE_ADDR, 1);
                if(copy_to_user(gf_nagv->buf, (gf_dev->buffer + GF_RDATA_OFFSET), GF_NAV_FRAME_LEN)){
                    pr_err("goodix gfx1xm  Failed to copy data from kernel to user.\n");
                    enable_irq(gf_dev->spi->irq);
                    return -EFAULT;
                }
                gf_spi_write_byte(gf_dev, GF_BUFFER_STATUS, status&0x7F);
                break;
            }
        }
        else
        {
            mdelay(3);
        }
    }while( retry++  < 20);
    pr_info("goodix gfx1xm   NAV base out: retry : %d\n", retry);
    gf_spi_write_byte(gf_dev, GF_MODE_STATUS, GF_IMAGE_MODE);
    enable_irq(gf_dev->spi->irq);
    return 0;
}
#endif
/**************************nagv end***********************************/
/*-------------------------------------------------------------------------*/
/* Read-only message with current device setup */
static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct gf_dev *gf_dev = filp->private_data;
    ssize_t			status = 0;
    //long int t1, t2;
    //FUNC_ENTRY();
    //pr_info("goodix gfx1xm  %s %d \n", __func__, __LINE__);
    if ((count > bufsiz)||(count == 0)) {
	//pr_warn("goodix gfx1xm  Max size for write buffer is %d. wanted length is %d\n", bufsiz, count);
	FUNC_EXIT();
	return -EMSGSIZE;
    }

    gf_dev = filp->private_data;
    mutex_lock(&gf_dev->fb_lock);
    mutex_lock(&gf_dev->buf_lock);
    gf_dev->spi->max_speed_hz=1*1000*1000;  //8*1000*1000;
    spi_setup(gf_dev->spi);
    status = gf_spi_read_bytes(gf_dev, GF_BUFFER_DATA, count, gf_dev->buffer);
    //gf_dev->spi->max_speed_hz=1*1000*1000;
    //spi_setup(gf_dev->spi);
    if(status > 0) {
	unsigned long missing = 0;
	missing = copy_to_user(buf, gf_dev->buffer + GF_RDATA_OFFSET, status);
	if(missing == status)
	    status = -EFAULT;
    } else {
	pr_err("goodix gfx1xm  Failed to read data from SPI device.\n");
	status = -EFAULT;
    }
    // t2 = ktime_to_us(ktime_get());
    //pr_info("goodix gfx1xm  read time use: %ld\n", t2-t1);
    mutex_unlock(&gf_dev->buf_lock);
    mutex_unlock(&gf_dev->fb_lock);

    return status;
}

/* Write-only message with current device setup */
static ssize_t gf_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
    struct gf_dev *gf_dev = filp->private_data;
    ssize_t			status = 0;
    //FUNC_ENTRY();
    if(count > bufsiz) {
	pr_warn("goodix gfx1xm  Max size for write buffer is %d\n", bufsiz);
	return -EMSGSIZE;
    } 

    mutex_lock(&gf_dev->fb_lock);
    mutex_lock(&gf_dev->buf_lock);
    status = copy_from_user(gf_dev->buffer + GF_WDATA_OFFSET, buf, count);
    if(status == 0) {
	gf_dev->spi->max_speed_hz=2*1000*1000;
	spi_setup(gf_dev->spi);
	status = gf_spi_write_bytes(gf_dev, GF_BUFFER_DATA, count, gf_dev->buffer);
    } else {
	pr_err("goodix gfx1xm  Failed to xfer data through SPI bus.\n");
	status = -EFAULT;
    }
    mutex_unlock(&gf_dev->buf_lock);
    mutex_unlock(&gf_dev->fb_lock);
    //FUNC_EXIT();
    return status;
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct gf_dev *gf_dev = NULL;
    struct gf_key gf_key = { 0 };
#if (NAVIGATION == 1)
    struct gf_nagv gf_nagv = { 0 };
#endif
    struct gf_ioc_transfer *ioc = NULL;
    int			err = 0;
    u32			tmp = 0;
    int 		retval = 0;
	int i =0;

	u8 *data_buf;   

    //printk("goodix gfx1xm  %s\n",__func__);
    //FUNC_ENTRY();

	
    if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
	return -ENOTTY;

    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
	err = !access_ok(VERIFY_WRITE,
		(void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
	err = !access_ok(VERIFY_READ,
		(void __user *)arg, _IOC_SIZE(cmd));
    if (err)
	return -EFAULT;

    gf_dev = (struct gf_dev *)filp->private_data;
    //pr_warn("goodix gfx1xm  goodix  gfx1xm_ioctl(%x)\n", cmd);         
    switch(cmd) {
	case GF_IOC_CMD:
	    ioc = kzalloc(sizeof(*ioc), GFP_KERNEL);
		
		#if 0
		for(i = 0; i < (ioc->len+5); i ++ )//debug
		{ 
						   pr_warn("goodix gfx1xm   gfx1xm_ioctl(%x)\n", gf_dev->buffer[i]);   
		}	
		#endif	 
	    /*copy command data from user to kernel.*/
	    if(copy_from_user(ioc, (struct gf_ioc_transfer*)arg, sizeof(*ioc))){
		pr_err("goodix gfx1xm  Failed to copy command from user to kernel.\n");
		retval = -EFAULT;
		break;
	    }

	    if((ioc->len > bufsiz)||(ioc->len == 0)) {
		pr_err("goodix gfx1xm  The request length[%d] is longer than supported maximum buffer length[%d].\n", 
			ioc->len, bufsiz);
		retval = -EMSGSIZE;
		break;
	    }

        mutex_lock(&gf_dev->fb_lock);
	    mutex_lock(&gf_dev->buf_lock);
	    gf_dev->spi->max_speed_hz=1*1000*1000;
	    spi_setup(gf_dev->spi);
	    if(ioc->cmd == GF_R) {
		/*if want to read data from hardware.*/
		//pr_info("goodix gfx1xm  Read data from 0x%x, len = 0x%x buf = 0x%p\n", ioc->addr, ioc->len, (void __user*)((unsigned long)(ioc->buf)));
		gf_spi_read_bytes(gf_dev, ioc->addr, ioc->len, gf_dev->buffer);
		if(copy_to_user((void __user*)((unsigned long)(ioc->buf)), gf_dev->buffer + GF_RDATA_OFFSET, ioc->len)) {
		    pr_err("goodix gfx1xm  Failed to copy data from kernel to user.\n");
		    retval = -EFAULT;
		    mutex_unlock(&gf_dev->buf_lock);
            mutex_unlock(&gf_dev->fb_lock);
		    break;
		}
		//print_16hex(ioc->buf, ioc->len);
	    } else if (ioc->cmd == GF_W) {
		/*if want to read data from hardware.*/
		//print_16hex(ioc->buf, ioc->len);
		//pr_err("goodix gfx1xm  Write data from 0x%x, len = 0x%x\n", ioc->addr, ioc->len);
		
		if(ioc->addr == 0x8043)
			{
				if (gf_dev->mode == GF_FF_MODE){
                        g_mode_switch_flag = 1;
                }
				data_buf = (void __user*)(unsigned long)ioc->buf;
				gf_dev->mode=data_buf[0];
				//pr_info("goodix gfx1xm  set mode 0x%x \n", gf_dev->mode);
			}

		if(copy_from_user(gf_dev->buffer + GF_WDATA_OFFSET,(void __user*)((unsigned long)(ioc->buf)), ioc->len)){
		    pr_err("goodix gfx1xm  Failed to copy data from user to kernel.\n");
		    retval = -EFAULT;
		    mutex_unlock(&gf_dev->buf_lock);
                    mutex_unlock(&gf_dev->fb_lock);
		    break;
		}

		gf_spi_write_bytes(gf_dev, ioc->addr, ioc->len, gf_dev->buffer);
	    } else {
		pr_err("goodix gfx1xm  Error command for gf.\n");
	    }
	    if(ioc != NULL) {
		kfree(ioc);
		ioc = NULL;
	    }
	    mutex_unlock(&gf_dev->buf_lock);
		mutex_unlock(&gf_dev->fb_lock);
		//pr_err("goodix gfx1xm  leave gf_ioctl.\n");
	    break;
	case GF_IOC_REINIT:
	    disable_irq(gf_dev->spi->irq);
	    gf_hw_reset(gf_dev);
	    enable_irq(gf_dev->spi->irq);
            gf_dev->irq_enabled = 1;

	    //gf_debug(FLOW_DEBUG,"wake-up gf\n");
	    break;
	case GF_IOC_SETSPEED:
	    retval = __get_user(tmp, (u32 __user*)arg);
	    if(tmp > 8*1000*1000) {
		pr_warn("goodix gfx1xm  The maximum SPI speed is 8MHz.\n");
		retval = -EMSGSIZE;
		break;
	    }
	    if(retval == 0) {
                mutex_lock(&gf_dev->fb_lock);		
     	        mutex_lock(&gf_dev->buf_lock);
		gf_dev->spi->max_speed_hz=tmp;
		spi_setup(gf_dev->spi);
	        mutex_unlock(&gf_dev->buf_lock);
                mutex_unlock(&gf_dev->fb_lock);
		//gf_debug(DEFAULT_DEBUG, "spi speed changed to %d\n", tmp);
	    }	
            break;
        case GF_IOC_POWER_OFF:
            disable_irq(gf_dev->spi->irq);
            mutex_lock(&gf_dev->fb_lock);	
            //POWER OFF	
            gf_power_off(gf_dev);
            mutex_unlock(&gf_dev->fb_lock);		

            //gf_debug(DEFAULT_DEBUG,"gf device disabled\n");
            break;

        case GF_IOC_POWER_ON:
            disable_irq(gf_dev->spi->irq);
            mutex_lock(&gf_dev->fb_lock);	
            //POWER ON	
            gf_power_on(gf_dev);
            mutex_unlock(&gf_dev->fb_lock);	

            enable_irq(gf_dev->spi->irq);
            gf_dev->irq_enabled = 1;
            //gf_debug(DEFAULT_DEBUG,"gf device enabled\n");
            break;
        case GF_IOC_DISABLE_IRQ:
            gf_disable_irq(gf_dev);
            break;
        case GF_IOC_ENABLE_IRQ:
            gf_enable_irq(gf_dev);
            break;
        case GF_IOC_SENDKEY:
            if (copy_from_user
                    (&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
                pr_warn("goodix gfx1xm  Failed to copy data from user space.\n");
                retval = -EFAULT;
                break;
            }

            for(i = 0; i< ARRAY_SIZE(key_map); i++) {
                if(key_map[i].val == gf_key.key){
                    input_report_key(gf_dev->input, gf_key.key, gf_key.value);
                    input_sync(gf_dev->input);
                    break;
                }   
            }   

            if(i == ARRAY_SIZE(key_map)) {
                pr_warn("goodix gfx1xm  key %d not support yet \n", gf_key.key);
                retval = -EFAULT; 
            } 

            break;
	#if (NAVIGATION == 1)
        case GF_IOC_GET_NAGV_BASE:
            if (copy_from_user
                    (&gf_nagv, (struct gf_nagv *)arg, sizeof(struct gf_nagv))) {
                pr_warn("goodix gfx1xm  Failed to copy data from user space.\n");
                retval = -EFAULT;
                break;
            }
            retval = gf_read_nav_base(gf_dev, &gf_nagv);
            break;
        case GF_IOC_GET_NAGV_DATA:
            if (copy_from_user
                    (&gf_nagv, (struct gf_nagv *)arg, sizeof(struct gf_nagv))) {
                pr_warn("goodix gfx1xm  Failed to copy data from user space.\n");
                retval = -EFAULT;
                break;
            }
            retval = get_nav_buf(gf_dev, &gf_nagv);
            break;
	#endif

    //LINE<JIRA_ID><DATE20160319><merge from P6601>zenghaihui            
    //guomingyi add start.
        case GFX1XM_IOC_SET_MODE:
            retval = __get_user(tmp, (u32 __user*)arg);
            if(tmp >= GF_IMAGE_MODE && tmp <= GF_KEY_MODE){
                //gfx1xm_spi_write_byte(gfx1xm_dev, 0x8043, tmp);
                gf_spi_write_byte(gf_dev, GF_MODE_STATUS, tmp);
                gf_dev->mode = tmp;
		pr_warn("goodix gfx1xm set mode for ESD:gf_dev->mode = %d\n", gf_dev->mode);
                //pr_info("goodix gfx1xm  [FTM]:set mode:[0x8043][%d]\n", tmp);
            }
        break;
        case GFX1XM_IOC_FTM:
        {
            int tmp1 = 0;
            retval = __get_user(tmp1, (u32 __user*)arg);
            tmp = ftm_gfx_irq_state;
            if(tmp1 == 0) {
                ftm_gfx_irq_state = 0;
            }
                retval = __put_user(tmp, (__u32 __user *)arg);
        }
        break;
    //guomingyi add end.
    
        default:
            pr_warn("goodix gfx1xm  gf doesn't support this command(%d)\n", cmd);
            break;
    }
    //FUNC_EXIT();
    return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
     return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static unsigned int gf_poll(struct file *filp, struct poll_table_struct *wait)
{
    struct gf_dev *gf_dev = filp->private_data;
    gf_spi_read_byte(gf_dev, GF_BUFFER_STATUS, &gf_dev->buf_status);
    if((gf_dev->buf_status & GF_BUF_STA_MASK) == GF_BUF_STA_READY) {
	return (POLLIN|POLLRDNORM);
    } else {
	//gf_debug(DEFAULT_DEBUG, "Poll no data.\n");
    }
    return 0;
}

#if CFG_UPDATE
static bool hw_config(struct gf_dev *gf_dev)
{

    mutex_lock(&gf_dev->buf_lock);
    memcpy(gf_dev->buffer + GF_WDATA_OFFSET, cfg, GF_CFG_LEN);
    gf_spi_write_bytes(gf_dev, GF_CFG_ADDR, GF_CFG_LEN, gf_dev->buffer);
    mutex_unlock(&gf_dev->buf_lock);

    return true;
}
#endif

#if FW_UPDATE
static int isUpdate(struct gf_dev *gf_dev)
{
    unsigned char version[16];
    unsigned int ver_fw = 0;
    unsigned int ver_file = 0;
    unsigned char* fw = GF_FW;
    unsigned char fw_running = 0;
    const unsigned char OFFSET = 7;

    msleep(300);
    gf_spi_read_byte(gf_dev, 0x41e4, &fw_running);
    pr_info("goodix gfx1xm  %s: 0x41e4 = 0x%x\n", __func__, fw_running);
    if(fw_running == 0xbe) {
	/*firmware running*/
	ver_file = (int)(fw[12] & 0xF0) <<12;
	ver_file |= (int)(fw[12] & 0x0F)<<8;
	ver_file |= fw[13];	//get the fw version in the i file;

	/*In case we want to upgrade to a special firmware. Such as debug firmware.*/
	if(ver_file != 0x5a5a) {
	    mutex_lock(&gf_dev->buf_lock);
	    gf_spi_read_bytes(gf_dev,0x8000,16,gf_dev->buffer);
	    memcpy(version, gf_dev->buffer + GF_RDATA_OFFSET, 16);
	    mutex_unlock(&gf_dev->buf_lock);
	    if(memcmp(version, GF_PID, GF_PID_LEN)) {
		pr_info("goodix gfx1xm  version: 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n", version[0], version[1],
			version[2], version[3], version[4], version[5]);
		return 1;
	    }
	    if((version[OFFSET]>9) || ((version[OFFSET + 1])>9)) {
                pr_info("goodix gfx1xm  version: 8-0x%x; 9-0x%x\n", version[OFFSET], version[OFFSET + 1]);
		return 1;
	    }

	    //get the current fw version
	    ver_fw  = (unsigned int)version[OFFSET] << 16;
	    ver_fw |= (unsigned int)version[OFFSET + 1] << 8;
	    ver_fw |= (unsigned int)version[OFFSET + 2];
	    pr_info("goodix gfx1xm  ver_fw: 0x%06x; ver_file:0x%06x\n", ver_fw, ver_file);

	    if(ver_fw == ver_file){
		/*If the running firmware is or ahead of the file's firmware. No need to do upgrade.*/
		return 0;
	    }
	}
	pr_info("goodix gfx1xm  Current Ver: 0x%x, Upgrade to Ver: 0x%x\n", ver_fw, ver_file);
    }else {
	/*no firmware.*/
	pr_info("goodix gfx1xm  No running firmware. Value = 0x%x\n", fw_running);
    }
    return 1;
}



static u8 is_9p_ready_ok(struct gf_dev *gf_dev)
{
   // u8 mp9pv[4] = {0x02, 0x08, 0x90, 0x00};
    u8 tmpBuf[16] = {0}; 
    u8 *ptr =NULL;
    u16 time_out = 0; 
    gf_spi_read_bytes(gf_dev, 0x4220, 4, tmpBuf);

    ptr = &tmpBuf[GF_RDATA_OFFSET];


    while(ptr[0] !=0x02 || ptr[1] !=0x08 || ptr[2] !=0x90 || ptr[3] !=0x00)
    {    
	time_out++;
	if (time_out > 200)
	{
	    return 0;
	}

	gf_spi_read_bytes(gf_dev, 0x4220, 4, tmpBuf);
	ptr = &tmpBuf[GF_RDATA_OFFSET];
    }    
    //printk("goodix gfx1xm  %s , timeout = %d\n",__func__,  time_out);
    return 1;
}

static int gf_fw_update_init(struct gf_dev *gf_dev)
{
    u8 retry_cnt = 5;     
    u8 value;

    while(retry_cnt--)
    {
	//set spi miso input pull up    
    	gf_miso_pullup();
    

	//reset and delay 5ms
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(5);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(1);

	//recover miso back spi
	gf_miso_backnal();


	gf_spi_setup(gf_dev, 1000*1000);


	if(!is_9p_ready_ok(gf_dev)){
	    pr_err("goodix gfx1xm  check 9p ver fail \n");
	    retry_cnt = 0xFF;
	    break;
	}


	mdelay(10);
	gf_spi_write_byte(gf_dev, 0x5081, 0x00);

	gf_spi_write_byte(gf_dev, 0x4180, 0x0C);
	gf_spi_read_byte(gf_dev, 0x4180, &value);
	if (value == 0x0C)             
	{             
	    pr_info("goodix gfx1xm  ########hold SS51 and DSP successfully!\n");
	    break;
	}
    }

    if(retry_cnt == 0xFF) {   
	pr_info("goodix gfx1xm  Faile to hold SS51 and DSP.\n");
	return 0;
    } else {
	pr_info("goodix gfx1xm  Hold retry_cnt=%d\n",retry_cnt);
	gf_spi_write_byte(gf_dev, 0x4010, 0);
	return 1;         
    }  
}

#endif


#if ESD_PROTECT
static void gf_timer_work(struct work_struct *work)
{	
    unsigned char value[4];
#if FW_UPDATE
    unsigned char* p_fw = GF_FW;
#endif
    struct gf_dev *gf_dev;
    int ret = 0;
    u8 mode = 0xFF;
    //FUNC_ENTRY();
    if(work == NULL)
    {
	pr_info("goodix gfx1xm  [info] %s wrong work\n",__func__);
	return;
    }
    gf_dev = container_of(work, struct gf_dev, spi_work);   

    mutex_lock(&gf_dev->fb_lock);

    //add by yinglong.tang
    if (gf_screen_status == SCREEN_OFF) {
        printk("goodix gfx1xm  [info] %s :gf_screen_status == SCREEN_OFF exit.\n",__func__);
        goto exit;
    }
    //add by yinglong.tang

    if ((gf_dev->isPowerOn == 0) || (gf_dev->mode == GF_FF_MODE) || (gf_dev->mode == GF_SLEEP_MODE)) {
        goto exit;
    }

    if(g_mode_switch_flag == 1) {
        g_mode_switch_flag = 0;
        goto exit;
    }

    //add by yinglong.tang
    if(suspend_test_flag == 1) {
        pr_info("goodix gfx1xm  [info] %s :suspend_test_flag =1\n",__func__);
        goto exit;
    } 
    //add by yinglong.tang

    mutex_lock(&gf_dev->buf_lock);
    ret = power_supply_is_system_supplied();
    //pr_info("goodix gfx1xm  BEN: power_supply ret = %d\n", ret);

    gf_dev->spi->max_speed_hz= 1000*1000;//SPI_SPEED_MIN;
    spi_setup(gf_dev->spi);
    mutex_unlock(&gf_dev->buf_lock);

    gf_spi_read_byte(gf_dev, 0x8040, &value[0]);
    gf_spi_read_byte(gf_dev, 0x8000, &value[1]);
    gf_spi_read_byte(gf_dev, 0x8043, &value[2]); //&& value[1] == 0x47 && value[2] == 0x56  
    //pr_info("goodix gfx1xm  timer_work: value[0] = 0x%x, 1:0x%x, 2:0x%x\n", value[0], value[1], value[2]);
    if(value[0] == 0xC6 && value[1] == 0x47){
	gf_spi_write_byte(gf_dev, 0x8040, 0xAA);
	mdelay(1);
    }else{
	gf_spi_read_byte(gf_dev, 0x8040, &value[0]);
	gf_spi_read_byte(gf_dev, 0x8000, &value[1]);
	gf_spi_read_byte(gf_dev, 0x8043, &value[2]); //&& value[1] == 0x47 && value[2] == 0x56  
	if(value[0] == 0xC6 && value[1] == 0x47){
	    gf_spi_write_byte(gf_dev, 0x8040, 0xAA);
	    mdelay(1);
	}else{
	    printk(KERN_ERR"goodix gfx1xm  ##Jason hardware works abnormal,do reset! 0x8040=0x%x 0x8000=0x%x 0x8046=0x%x\n"
			,value[0],value[1],value[2]);
	    disable_irq(gf_dev->spi->irq);
	    gf_hw_reset(gf_dev);

            gf_spi_read_byte(gf_dev, 0x8000, &value[0]);
            pr_info("goodix gfx1xm  [info] %s read 0x8000 finish value = 0x%x\n", __func__,value[0]);
#if FW_UPDATE
            if(value[0] != 0x47) {
                gf_spi_read_byte(gf_dev, 0x8000, &value[0]);
                if(value[0] != 0x47) {
		    /***********************************firmware update*********************************/
		    printk(KERN_ERR"goodix gfx1xm  [info] %s firmware update start\n", __func__);
		    del_timer_sync(&gf_dev->gf_timer);
		    if(gf_fw_update_init(gf_dev)) {
			gf_fw_update(gf_dev, p_fw, FW_LENGTH);
			gf_hw_reset(gf_dev);

		    }
		    gf_dev->gf_timer.expires = jiffies + 2 * HZ;
		    add_timer(&gf_dev->gf_timer);
		}
	    }
	#endif
	    //pr_info("goodix gfx1xm  [info] %s write 0xaa \n", __func__);
	    ret = gf_spi_write_byte(gf_dev, 0x8040, 0xAA);
	    if(!ret)
		pr_info("goodix gfx1xm  [info] %s write 0x8040 fail\n", __func__);
	
	#if CFG_UPDATE
	/***************************************update config********************************/
	    if(!hw_config(gf_dev))
		pr_info("goodix gfx1xm  [info] %s write config fail\n", __func__);
	#endif
	    enable_irq(gf_dev->spi->irq);
            gf_dev->irq_enabled = 1;
	}
    }
    /*if mode was changed by reset, we should set the mode  back to the primary mode*/
    gf_spi_read_byte(gf_dev, GF_MODE_STATUS,&mode); 
    if(mode != gf_dev->mode) {
	pr_info("goodix gfx1xm  [info] %s set mode back\n", __func__);
	gf_spi_write_byte(gf_dev, GF_MODE_STATUS, gf_dev->mode);
	gf_spi_read_byte(gf_dev, GF_MODE_STATUS, &mode);
	pr_info("goodix gfx1xm  [info] %s mode444 = %d\n", __func__, mode);
    }

exit:
    mutex_unlock(&gf_dev->fb_lock);
    mod_timer(&gf_dev->gf_timer, jiffies + 2*HZ);//??whether 2s is ok
    //FUNC_EXIT();
}


static void gf_timer_func(unsigned long arg)
{
    struct gf_dev* gf_dev = (struct gf_dev*)arg;
    schedule_work(&gf_dev->spi_work);	
}
#endif

static void gf_reg_key_kernel(struct gf_dev *gf_dev)
{
    int i;

    __set_bit(EV_KEY, gf_dev->input->evbit); //tell the kernel is key event
    for(i = 0; i< ARRAY_SIZE(key_map); i++) {
        __set_bit(key_map[i].val, gf_dev->input->keybit);
    }

    gf_dev->input->name = GF_INPUT_NAME;
    if (input_register_device(gf_dev->input))
        pr_warn("goodix gfx1xm  Failed to register GF as input device.\n");
}

static irqreturn_t gf_irq(int irq, void* handle)
{
    struct gf_dev *gf_dev = (struct gf_dev *)handle;
    u8 mode = 0x80;
    u8	status = 0, lbstatus = 0;

#if (NAVIGATION == 1)
    u8  ngv;
    u8  force_val;
#endif

    //printk(KERN_ERR"goodix gfx1xm  gf_irq ========= suspend_test_flag = %d \n",suspend_test_flag);
    if(suspend_test_flag == 1) {
        printk("goodix gfx1xm  suspend flag is true: return \n");
        return IRQ_HANDLED;
    }

    gf_spi_read_byte(gf_dev, GF_BUFFER_STATUS, &status);
#if (NAVIGATION == 1)
    gf_spi_read_byte(gf_dev, GF_LBSTATUS_ADDR, &lbstatus);
#endif
    gf_spi_read_byte(gf_dev, GF_MODE_STATUS, &mode);

    wake_lock_timeout(&fg_wake_lock, 10*HZ);
    if (lbstatus & (1<<7) ) {
        //senser reset
#if (NAVIGATION == 1)
        gf_spi_write_byte(gf_dev, GF_LBSTATUS_ADDR, lbstatus & 0x7F);
#endif
        gf_spi_write_byte(gf_dev, GF_MODE_STATUS, gf_dev->mode);
        //printk("goodix gfx1xm  Sensor rest, set mode %d\n", gf_dev->mode);
	    return IRQ_HANDLED;
    }
    if(!(status & GF_BUF_STA_MASK)) {
        //printk("goodix gfx1xm  Invalid IRQ status = 0x%x, lbstatus = 0x%x, mode = %d\n", status, lbstatus,mode);
	    return IRQ_HANDLED;
    }
 

    //printk("goodix gfx1xm  IRQ status = 0x%x, lbstatus = 0x%x, mode = %d\n", status, lbstatus, mode);

    
     //LINE<JIRA_ID><DATE20160319><merge from P6601>zenghaihui       
    //guomingyi add start.
       if(ftm_gfx_irq_state != 0) {
           ftm_gfx_irq_state = 0;
       }
    //guomingyi add end.
    
    switch(mode)
    {
	case GF_FF_MODE:
	    if((status & GF_HOME_KEY_MASK) && (status & GF_HOME_KEY_STA)){
		//pr_info("goodix gfx1xm  gf: wake device.\n");
		#if 0 //Remove by yinglong.tang
                input_report_key(gf_dev->input, GF_KEY_POWER, 1);
		input_sync(gf_dev->input);	    
                input_report_key(gf_dev->input, GF_KEY_POWER, 0);
		input_sync(gf_dev->input);
		#endif
	    //} else {
		//break;
	    }

	case GF_IMAGE_MODE:
#ifdef GF_FASYNC
	    if(gf_dev->async) {
		//pr_info("goodix gfx1xm  async \n");
		kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
	    }
#endif
	    break;
	case GF_KEY_MODE:
#if (NAVIGATION == 1)
            gf_spi_read_byte(gf_dev, GF_LBSTATUS_ADDR, &lbstatus); 
            printk("goodix gfx1xm  gf:Key mode: status = 0x%x lbstatus = 0x%x\n", status, lbstatus);

            if(lbstatus & GF_NGKEY_MASK) {
                if(lbstatus & GF_NVG_KEY_MASK) { //Navigator is enabled
                    ngv = (lbstatus & GF_NVG_KEY_STA) >> 4;
                    if (NGV_VALUE_UP == ngv) {
                        pr_info("goodix gfx1xm  UP");
                        input_report_key(gf_dev->input, GF_UP_KEY, 1);
                        input_sync(gf_dev->input);
                        input_report_key(gf_dev->input, GF_UP_KEY, 0);
                        input_sync(gf_dev->input);
                    } else if (NGV_VALUE_DOWN == ngv) {
                        pr_info("goodix gfx1xm  DOWN");
                        input_report_key(gf_dev->input, GF_DOWN_KEY, 1);
                        input_sync(gf_dev->input);
                        input_report_key(gf_dev->input, GF_DOWN_KEY, 0);
                        input_sync(gf_dev->input);
                    } else if (NGV_VALUE_LEFT == ngv) {
                        /*
                           pr_info("goodix gfx1xm  KEY_BACK");
                           gf_send_key(KEY_BACK, 1);   
                           gf_send_key(KEY_BACK, 0);   
                         */
                        pr_info("goodix gfx1xm  LEFT");
                        input_report_key(gf_dev->input, GF_KEY_MENU, 1);
                        input_sync(gf_dev->input);
                        input_report_key(gf_dev->input, GF_KEY_MENU, 0);
                        input_sync(gf_dev->input);
                    } else if (NGV_VALUE_RIGHT == ngv) {
                        pr_info("goodix gfx1xm  RIGHT");
                        /*gf_send_key(KEY_F17, 1);  
                          gf_send_key(KEY_F17, 0);    
                         */
                        input_report_key(gf_dev->input, GF_KEY_BACK, 1);
                        input_sync(gf_dev->input);
                        input_report_key(gf_dev->input, GF_KEY_BACK, 0);
                        input_sync(gf_dev->input);
                    }
                } 

                if (lbstatus&GF_FCE_KEY_MASK) { // force value is enabled.
                    gf_spi_read_byte(gf_dev, GF_FORCE_VALUE_ADDR, &force_val);
                    pr_info("goodix gfx1xm  FORCE VALUE. force_val : %d\n", force_val);
                    input_report_key(gf_dev->input, GF_KEY_FORCE, force_val);
                    input_sync(gf_dev->input);
                }
            } else if (status & GF_KEY_MASK) {
                pr_info("goodix gfx1xm  input home key \n");
                if (status & GF_HOME_KEY_MASK) {
                    input_report_key(gf_dev->input, GF_KEY_HOME, (status & GF_HOME_KEY_STA)>>4);
                    input_sync(gf_dev->input);
                }
            }
            //gf_spi_write_byte(gf_dev, GF_BUFFER_STATUS, (status & 0x7F));
#else
	    //printk(KERN_ERR"goodix gfx1xm  gf:Key mode: status = 0x%x\n", status);
	    if  ((status & GF_KEY_MASK) && (status & GF_BUF_STA_MASK)) {
            #if 0
		if (status & GF_HOME_KEY_MASK) {
            
            printk("goodix gfx1xm  gf:Key mode: GF_KEY_HOME \n");
                 #if 0 //Remove by yinglong.tang
		    input_report_key(gf_dev->input, GF_KEY_BACK, (status & GF_HOME_KEY_STA)>>4);
		    //input_report_key(gf_dev->input, GF_KEY_BACK, (status & GF_BACK_KEY_STA));
		    input_sync(gf_dev->input);
		    #endif
		}

		else if (status & GF_MENU_KEY_MASK){
            
            printk("goodix gfx1xm  gf:Key mode: GF_KEY_MENU \n");
                 #if 0 //Remove by yinglong.tang
		    input_report_key(gf_dev->input, GF_KEY_MENU, (status & GF_MENU_KEY_STA)>>2);
		    input_sync(gf_dev->input);
		    #endif

		}else if (status & GF_BACK_KEY_MASK){
            
            printk("goodix gfx1xm  gf:Key mode: GF_KEY_BACK \n");
                  #if 0 //Remove by yinglong.tang
		    input_report_key(gf_dev->input, GF_KEY_BACK, (status & GF_BACK_KEY_STA));
		    input_sync(gf_dev->input);
		    #endif
		}
            #else
            //LINE<JIRA_ID><DATE20160319><merge from P6601>zenghaihui            
            //guomingyi add start.
                if(status == 0xb0) {
                    ftm_gfx_irq_state = 0x1; //down.
                }
                else if(status == 0xa0) {
                    ftm_gfx_irq_state = 0x2; //up.
                }
                //gfx1xm_spi_write_byte(gfx1xm_dev, 0x8140, 0x00);  
                gf_spi_write_byte(gf_dev, GF_BUFFER_STATUS, 0x00);
#if 0 // GF_FASYNC

                if(gfx1xm_dev->async && ftm_gfx_irq_state) {
                    gfx1xm_debug(DEFAULT_DEBUG,"key async ");
                    kill_fasync(&gfx1xm_dev->async, SIGIO, POLL_IN);
                }
 #endif
            //guomingyi add end.
            #endif

	    }

#endif
            //LINE<JIRA_ID><DATE20160319><merge from P6601>zenghaihui            
	    if(gf_dev->async) {
           //if(gf_dev->async && ftm_gfx_irq_state) {
		//pr_info("goodix gfx1xm  async \n");
    		kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
	    }   

            break;
        case GF_SLEEP_MODE:
            //pr_warn("goodix gfx1xm  gf:Should not happen in sleep mode.\n");
            break;
        case GF_DEBUG_MODE:
#ifdef GF_FASYNC
	    if(gf_dev->async) {
		kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
	    }
#endif
            break;
	#if (NAVIGATION == 1)
        case GF_NAV_MODE:
            printk("goodix gfx1xm  Macky:IRQ status = 0x%x, lbstatus = 0x%x, mode = %d\n", status, lbstatus, mode);
            sendnlmsg("nagv");
            break;
	#endif
        default:
            pr_warn("goodix gfx1xm  gf:Unknown mode. mode = 0x%x\n", mode);
            break;

    }

    return IRQ_HANDLED;
}


static int gf_open(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev;
    int			status = -ENXIO;

    //FUNC_ENTRY();
	  //printk("goodix gfx1xm  %s\n",__func__);
	
	//printk("goodix gfx1xm  gf_open:gf_opengf_opengf_opengf_open.\n");
    mutex_lock(&device_list_lock);

    list_for_each_entry(gf_dev, &device_list, device_entry) {
	if(gf_dev->devt == inode->i_rdev) {
	    //gf_debug(DEFAULT_DEBUG, "Found\n");
	    status = 0;
	    break;
	}
    }

    if(status == 0){
	mutex_lock(&gf_dev->buf_lock);
	if( gf_dev->buffer == NULL) {
	    gf_dev->buffer = kzalloc(bufsiz + GF_RDATA_OFFSET, GFP_KERNEL);
	    if(gf_dev->buffer == NULL) {
		dev_dbg(&gf_dev->spi->dev, "goodix gfx1xm  open/ENOMEM\n");
		status = -ENOMEM;
	    }
	}
	mutex_unlock(&gf_dev->buf_lock);

	if(status == 0) {
	    gf_dev->users++;
	    filp->private_data = gf_dev;
	    nonseekable_open(inode, filp);
	    //gf_debug(DEFAULT_DEBUG, "Succeed to open device. irq = %d\n", gf_dev->spi->irq);
	    enable_irq(gf_dev->spi->irq);
            gf_dev->irq_enabled = 1;
	}
    } else {
	pr_err("goodix gfx1xm  No device for minor %d\n", iminor(inode));
    }
    gf_power_on(gf_dev);
    mutex_unlock(&device_list_lock);
    //FUNC_EXIT();
    return status;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
    struct gf_dev *gf_dev = filp->private_data;
    int ret;

    //FUNC_ENTRY();
    ret = fasync_helper(fd, filp, mode, &gf_dev->async);
    //FUNC_EXIT();
    return ret;
}
#endif

static int gf_release(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev;
    int    status = 0;
    //FUNC_ENTRY();
    mutex_lock(&device_list_lock);
    gf_dev = filp->private_data;
    filp->private_data = NULL;

    /*last close??*/
    gf_dev->users --;
    if(!gf_dev->users) {

	//gf_debug(DEFAULT_DEBUG, "disble_irq. irq = %d\n", gf_dev->spi->irq);
	disable_irq(gf_dev->spi->irq);
        gf_power_off(gf_dev);
    }
    mutex_unlock(&device_list_lock);
    //FUNC_EXIT();
    return status;
}

static const struct file_operations gf_fops = {
    .owner =	THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .write =	gf_write,
    .read =		gf_read,
    .unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT    
    .compat_ioctl   = gf_compat_ioctl,
#endif        
    .open =		gf_open,
    .release =	gf_release,
    .poll   = gf_poll,
#ifdef GF_FASYNC
    .fasync = gf_fasync,
#endif
};



/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *gf_spi_class;
struct gf_dev	 *g_tn_f_dev;

/*-------------------------------------------------------------------------*/

static int gf_probe(struct spi_device *spi)
{
    struct gf_dev	*gf_dev;
    int			status = -EINVAL ;
    int ret;
    unsigned long		minor;
    int err = 0;
    unsigned char version[16] = {0};
    FUNC_ENTRY();

    if(1 == g_fp_match_flag)
    {
        pr_info("goodix gfx1xm  gf_probe --fingerpirnt have been matched!!! \n");
        return -ENODEV;
    }
    
    /* Allocate driver data */
    gf_dev = kzalloc(sizeof(*gf_dev), GFP_KERNEL);
    if (!gf_dev){
	    pr_warn("goodix gfx1xm  Failed to alloc memory for gf device.\n");
		FUNC_EXIT();
		status = -ENOMEM;
		goto exit;
    }

    gf_dev->buffer = kzalloc(bufsiz + GF_RDATA_OFFSET, GFP_KERNEL);
    if(gf_dev->buffer == NULL) {
            status = -ENOMEM;   
        goto buf_alloc_fail;
    }
    spi_set_drvdata(spi, gf_dev);


	g_tn_f_dev = gf_dev;
    g_mode_switch_flag = 0; 
    /* Initialize the driver data */
    gf_dev->spi = spi;
    gf_dev->irq_enabled = 0;
    spin_lock_init(&gf_dev->spi_lock);
    mutex_init(&gf_dev->buf_lock);
    mutex_init(&gf_dev->fb_lock);
    INIT_LIST_HEAD(&gf_dev->device_entry);

	gf_dev->irq_gpio   = -EINVAL;
	gf_dev->reset_gpio = -EINVAL;
	gf_dev->pw_gpio = -EINVAL;
	gf_dev->miso_gpio = -EINVAL;
	
	if(gf_parse_dts(gf_dev)){
		
		status =  -EINVAL;
		goto res_clean;
	}
	
	ret = gf_power_init(gf_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to init regulator\n");
		goto res_clean;
	}
	gf_dev->isPowerOn = false;
	ret = gf_power_on(gf_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to power on device\n");
		goto err_deinit_regulator;
	}
    
    /*setup gf configurations.*/
    gf_debug(DEFAULT_DEBUG, "Setting gf device configuration.\n");
    /*SPI parameters.*/
    gf_spi_setup(gf_dev, 1000*1000);
    
    gf_hw_reset(gf_dev);

    ret = gf_spi_read_bytes(gf_dev,0x8000,16,gf_dev->buffer);
    memcpy(version, gf_dev->buffer + GF_RDATA_OFFSET, 16);		
    for(ret = 0; ret <16; ret++)
        printk("goodix gfx1xm  version[%d] = %x \n", ret,version[ret]);

    //LINE<JIRA_ID><DATE20160316><BUG_INFO>zenghaihui
    if(memcmp(version, GF_PID, GF_PID_LEN))
    {
        printk("goodix gfx1xm  chip match failed \n" );
        status = -EINVAL ;
        goto goodix_chip_match_fail;
    }

    
        BUILD_BUG_ON(N_SPI_MINORS > 256);
        status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
        if (status < 0){
            pr_warn("goodix gfx1xm  Failed to register char device!\n");
            goto err_register_char;
        }
        
        gf_spi_class = class_create(THIS_MODULE, CLASS_NAME);
        if (IS_ERR(gf_spi_class)) {
            pr_warn("goodix gfx1xm  Failed to create class.\n");
            //status = PTR_ERR(gf_spi_class);
            status = -EINVAL ;
            goto err_creat_class;
        }
        
        /*init netlink for kernel and user space*/
#if (NAVIGATION == 1)
        netlink_init();
#endif
    
    
    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
		struct device *dev;
		#if 0
		status = sysfs_create_group(&spi->dev.kobj,&gf_debug_attr_group);
	        if(status){
	            pr_err("goodix gfx1xm  Failed to create sysfs file.\n");
	            status =  -EINVAL;
			goto res_clean;
	        }
		#endif
		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_spi_class, &spi->dev, gf_dev->devt,
			gf_dev, DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    } else {
        dev_dbg(&spi->dev, "goodix gfx1xm  no minor number available!\n");
        status = -ENODEV;
		goto err_creat_class;
    }
		if(status ==0){
			set_bit(minor, minors);
	    	list_add(&gf_dev->device_entry, &device_list);		
		}
    mutex_unlock(&device_list_lock);
	

    
    if (status == 0){

		/*register device within input system.*/
		gf_dev->input = input_allocate_device();
		if(gf_dev->input == NULL) {
		    pr_err("goodix gfx1xm  Failed to allocate input device.\n");
		    status = -ENOMEM;
			goto input_dev_alloc_fail;
		}

        	gf_reg_key_kernel(gf_dev);
			

		#if FW_UPDATE
		if(isUpdate(gf_dev)) {
		    unsigned char* fw = GF_FW;     
		    /*Do upgrade action.*/         
		    if(gf_fw_update_init(gf_dev)) {
			gf_fw_update(gf_dev, fw, FW_LENGTH);
			gf_hw_reset(gf_dev);
			mdelay(100);
		    }
		}
		#endif
		
		#if CFG_UPDATE
		/*write config*/      
		if(!hw_config(gf_dev))       
		    pr_err("goodix gfx1xm  [info] %s write config fail\n", __func__);
		#endif

		#if 1
		err = request_threaded_irq(spi->irq, NULL, gf_irq, 
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			dev_name(&spi->dev), gf_dev);
		#endif
        

		if(!err) {
			device_init_wakeup(&spi->dev, 1);
		     enable_irq_wake(gf_dev->spi->irq);
		     disable_irq(gf_dev->spi->irq);
			 gf_debug(DEFAULT_DEBUG, "irq request 11 status[%d] irq[%d].\n",err,gf_dev->spi->irq);
		}


	#if ESD_PROTECT
		INIT_WORK(&gf_dev->spi_work, gf_timer_work);
		init_timer(&gf_dev->gf_timer); 
		gf_dev->gf_timer.function = gf_timer_func;
		gf_dev->gf_timer.expires = jiffies + 3*HZ;
		gf_dev->gf_timer.data = (unsigned long)gf_dev;
		add_timer(&gf_dev->gf_timer);  
	#endif // ESD_PROTECT 

		pr_info("goodix gfx1xm  GF installed.\n");
    }
    wake_lock_init(&fg_wake_lock, WAKE_LOCK_SUSPEND, "goodix_wake_lock");
	printk("goodix gfx1xm  gf_probe Ok end \n");
    
    //LINE<JIRA_ID><DATE20160316><BUG_INFO>zenghaihui
    g_fp_match_flag = 1;

    //add by yinglong.tang	
    #if defined(CONFIG_FB)
    gf_configure_sleep(gf_dev);
    #endif
    //add by yinglong.tang
    full_fp_chip_name(DEV_NAME);//add by yinglong.tang.
    
    FUNC_EXIT();
    return status;

input_dev_alloc_fail:	
    list_del(&gf_dev->device_entry);
    clear_bit(MINOR(gf_dev->devt), minors);
    device_destroy(gf_spi_class, gf_dev->devt);
    
    class_destroy(gf_spi_class);
    
err_creat_class:
    //unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
    unregister_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME);

err_register_char:
goodix_chip_match_fail:
err_deinit_regulator:
    gf_power_deinit(gf_dev);
    
res_clean:
    gf_cleanup(gf_dev);
    if(gf_dev->buffer != NULL)
    {
        kfree(gf_dev->buffer);
        gf_dev->buffer = NULL;
    }
    
buf_alloc_fail:
	kfree(gf_dev);

exit:

    pr_info("goodix gfx1xm  gf_probe failed exit \n");
    
	return status;


}

static int gf_remove(struct spi_device *spi)
{
    struct gf_dev	*gf_dev = spi_get_drvdata(spi);
    FUNC_ENTRY();

    /* make sure ops on existing fds can abort cleanly */
    if(gf_dev->spi->irq) {
	free_irq(gf_dev->spi->irq, gf_dev);
    }

#if ESD_PROTECT
    del_timer_sync(&gf_dev->gf_timer); 
    cancel_work_sync(&gf_dev->spi_work); //lbb del
#endif

    spin_lock_irq(&gf_dev->spi_lock);
    gf_dev->spi = NULL;
    spi_set_drvdata(spi, NULL);
    spin_unlock_irq(&gf_dev->spi_lock);
    /*
       if(gf_dev->spi_wq != NULL) {
       flush_workqueue(gf_dev->spi_wq);
       destroy_workqueue(gf_dev->spi_wq);
       }
     */
    /* prevent new opens */
    mutex_lock(&device_list_lock);
    list_del(&gf_dev->device_entry);
    device_destroy(gf_spi_class, gf_dev->devt);
    clear_bit(MINOR(gf_dev->devt), minors);

    gf_power_deinit(gf_dev);
    
	gf_cleanup(gf_dev);
    if (gf_dev->users == 0) {
	if(gf_dev->input != NULL)
	    input_unregister_device(gf_dev->input);

	if(gf_dev->buffer != NULL)
	    kfree(gf_dev->buffer);
	kfree(gf_dev);
    }
    mutex_unlock(&device_list_lock);

    FUNC_EXIT();
    return 0;
}

static int gf_suspend_test(struct device *dev)
{
    suspend_test_flag = 1;	
    enable_irq_wake(g_tn_f_dev->spi->irq);
    printk(KERN_ERR" goodix gfx1xm  gf_suspend_test.\n");
    //g_debug |= SUSPEND_DEBUG;
    return 0;
}

static int gf_resume_test(struct device *dev)
{
    suspend_test_flag = 0;
    disable_irq_wake(g_tn_f_dev->spi->irq);
    printk(KERN_ERR" goodix gfx1xm   gf_resume_test.\n");
    //g_debug &= ~SUSPEND_DEBUG;
    return 0;
}
static const struct dev_pm_ops gf_pm = {
    .suspend = gf_suspend_test,
    .resume = gf_resume_test
};
static struct of_device_id gx_match_table[] = {
        { .compatible = "qcom,fingerprint",},
        { },
};


static struct spi_driver gf_spi_driver = {
    .driver = {
	.name =		SPI_DEV_NAME,
	.owner =	THIS_MODULE,
	.pm = &gf_pm,
	.of_match_table = gx_match_table,
    },
    .probe =	gf_probe,
    .remove =	gf_remove,
    //.suspend = gf_suspend_test,
    //.resume = gf_resume_test,

    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};

/*-------------------------------------------------------------------------*/

static int __init gf_init(void)
{
    int status;
	FUNC_ENTRY();

    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */

    if(1 == g_fp_match_flag)
    {
        pr_info("goodix gfx1xm  gf_init  --fingerpirnt have been matched!!! \n");
        return -ENODEV;
    }

#if 1
        
        status = spi_register_driver(&gf_spi_driver);
        
        pr_info("goodix gfx1xm  gf_init  status = %d \n", status);
        
        if (status < 0) {
            pr_warn("goodix gfx1xm  Failed to register SPI driver.\n");
        }
        
#else

    BUILD_BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
    if (status < 0){
		pr_warn("goodix gfx1xm  Failed to register char device!\n");
		FUNC_EXIT();
		return status;
    }
	
    gf_spi_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(gf_spi_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
		pr_warn("goodix gfx1xm  Failed to create class.\n");
		FUNC_EXIT();
		return PTR_ERR(gf_spi_class);
    }
    status = spi_register_driver(&gf_spi_driver);
	
    pr_info("goodix gfx1xm  gf_init  status = %d \n", status);
    
    if (status < 0) {
		class_destroy(gf_spi_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
		pr_warn("goodix gfx1xm  Failed to register SPI driver.\n");
    }
    
	/*init netlink for kernel and user space*/
#if (NAVIGATION == 1)
    netlink_init();
#endif
#endif
	
    pr_info("goodix gfx1xm  gf_init exit\n");
    
    return status;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
   	FUNC_ENTRY();
#if 1

    spi_unregister_driver(&gf_spi_driver);

#else
    spi_unregister_driver(&gf_spi_driver);
    class_destroy(gf_spi_class);
    unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
#if (NAVIGATION == 1)
    netlink_exit();
#endif
#endif
}
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf-spi");


