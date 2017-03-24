#ifndef __GF_SPI_H
#define __GF_SPI_H


#define NAVIGATION		0
#define FW_UPDATE               	1
#define ESD_PROTECT             	1
#define CFG_UPDATE              	1
#define GF_X16M_TARGET_FW              	1
#define GF_316M_TARGET_CFG              	1

#include <linux/types.h>
#if (NAVIGATION == 1)
#include <linux/notifier.h>
#endif

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

/********************GF Mapping**********************/
#define GF_BASE             (0x8000)
#define GF_OFFSET(x)        (GF_BASE + x)

#define GF_VERSION	        GF_OFFSET(0)
#define GF_CONFIG_DATA      GF_OFFSET(0x40)
#define GF_CFG_ADDR	        GF_OFFSET(0x47)
#define GF_PAUSE_ADDR       GF_OFFSET(0x44)
#define GF_MODE_STATUS      GF_OFFSET(0x043)
//#define GF_MIXER_DATA     GF_OFFSET(0x140)
#define GF_BUFFER_STATUS	GF_OFFSET(0x140)
#define GF_KEY_DATA         GF_OFFSET(0x142)
#define GF_NOISE_DATA       GF_OFFSET(0x144)
#define GF_LONG_PRESS_STDP  GF_OFFSET(0x146)
#define GF_BUFFER_DATA      GF_OFFSET(0x141)
#if (NAVIGATION == 1)
#define GF_FORCE_VALUE_ADDR GF_OFFSET(0x146)
#define GF_LBSTATUS_ADDR    GF_OFFSET(0x141)
#define GF_NAV_FRAMENUM_ADDR    GF_OFFSET(0x146)
#define GF_NAV_BUF0_ADDR   GF_OFFSET(0x148)
#define GF_NAV_BUF1_ADDR       0x9B48
#endif

#define GF_BUF_STA_MASK     (0x1<<7)
#define	GF_BUF_STA_READY	(0x1<<7)
#define	GF_BUF_STA_BUSY     (0x0<<7)

#define	GF_IMAGE_MASK       (0x1<<6)
#define	GF_IMAGE_ENABLE     (0x1)
#define	GF_IMAGE_DISABLE	(0x0)

#if (NAVIGATION == 1)
#define GF_LONGPRESS_MASK   (0x1<<2)
#define GF_FCE_KEY_MASK (0x1<<3)
#define GF_NVG_KEY_MASK (0x1<<6)
#define GF_NVG_KEY_STA  (0x3<<4)
#define GF_NGKEY_MASK   (GF_NVG_KEY_MASK | GF_FCE_KEY_MASK | GF_LONGPRESS_MASK)
#endif

#define	GF_KEY_MASK	        (GF_HOME_KEY_MASK | \
                                 GF_MENU_KEY_MASK | \
                                 GF_BACK_KEY_MASK )

//home key
#define	GF_HOME_KEY_MASK	(0x1<<5)
#define	GF_HOME_KEY_ENABL   (0x1)
#define	GF_HOME_KEY_DISABLE (0x0)

#define	GF_HOME_KEY_STA     (0x1<<4)
//menu key
#define	GF_MENU_KEY_MASK    (0x1<<3)
#define	GF_MENU_KEY_ENABLE	(0x1)
#define	GF_MENU_KEY_DISABLE	(0x0)

#define	GF_MENU_KEY_STA	(0x1<<2)
//back key
#define	GF_BACK_KEY_MASK    (0x1<<1)
#define	GF_BACK_KEY_ENABLE  (0x1)
#define	GF_BACK_KEY_DISABLE (0x0)

#define	GF_BACK_KEY_STA     (0x1<<0)

#if (NAVIGATION == 1)
#define NGV_VALUE_UP        0x00 
#define NGV_VALUE_DOWN  0x01
#define NGV_VALUE_LEFT  0x02
#define NGV_VALUE_RIGHT     0x03
#endif


#define	GF_IMAGE_MODE       (0x00)
#define	GF_KEY_MODE	        (0x01)
#define GF_SLEEP_MODE       (0x02)
#define GF_FF_MODE		(0x03)
#if (NAVIGATION == 1)
#define GF_NAV_MODE         (0x10)
#define GF_PRENAV_MODE      (0x11)
#endif
#define GF_DEBUG_MODE       (0x56)

#if (NAVIGATION == 1)
/*low 8-bit status*/
#define GF_RST_MASK     (0x1<<7)
#define GF_NAV_MASK     (0x1<<6)
#define GF_NAV_BUF_MASK     (0x1<<5)
#define GF_LONG_PRESS_MASK     (0x1<<2)
#define GF_LONG_PRESS_STA     (0x1<<1)
#endif


/**********************GF ops****************************/
#define GF_W                0xF0
#define GF_R                0xF1
#define GF_WDATA_OFFSET         (0x3)
#define GF_RDATA_OFFSET         (0x5)
#define GF_CFG_LEN                  (249)   /*config data length*/

/**********************************************************/

#define GF_FASYNC 		1//If support fasync mechanism.
//#undef GF_FASYNC

/*************************************************************/
struct gf_dev {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct input_dev        *input;

	struct workqueue_struct *spi_wq;
	struct work_struct     spi_work;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned		users;
	u8			*buffer;		
	u8  		buf_status;
	u8 		mode;
	u8           isPowerOn;
	struct timer_list   gf_timer;

	unsigned 		irq_gpio;
	unsigned		reset_gpio;
	unsigned 		miso_gpio;
	unsigned 		pw_gpio;
	struct pinctrl *gf_pinctrl;
	struct pinctrl_state *pinctrl_state_default;
#ifdef GF_FASYNC
	struct  fasync_struct *async;
#endif
    struct notifier_block notifier;
    //char fb_black;
    struct mutex fb_lock;
    int irq_enabled;
    //LINE<JIRA_ID><DATE20160328><add for power config>zenghaihui
    struct regulator *vdd;
    struct regulator *vio;
};

struct gf_key {
        unsigned int key;
        int value;
};


/**********************IO Magic**********************/
#define  GF_IOC_MAGIC    'g'  //define magic number
struct gf_ioc_transfer {
	u8	cmd;
	u8 reserve;
	u16	addr;
	u32 len;
	u64 buf;	//u64 buf;
};
#if (NAVIGATION == 1)
struct gf_nagv {
	u8 *buf;
	u32 len;
        u8 *frame_num;
};
#endif
//define commands
/*read/write GF registers*/
#define  GF_IOC_CMD	_IOWR(GF_IOC_MAGIC, 1, struct gf_ioc_transfer)
#define  GF_IOC_REINIT	_IO(GF_IOC_MAGIC, 0)
#define  GF_IOC_SETSPEED	_IOW(GF_IOC_MAGIC, 2, u32)
#define  GF_IOC_POWER_OFF   _IO(GF_IOC_MAGIC, 3)
#define  GF_IOC_POWER_ON    _IO(GF_IOC_MAGIC, 4)
#define  GF_IOC_DISABLE_IRQ     _IO(GF_IOC_MAGIC, 5)
#define  GF_IOC_ENABLE_IRQ      _IO(GF_IOC_MAGIC, 6)
#define  GF_IOC_SENDKEY  _IOW(GF_IOC_MAGIC, 7, struct gf_key)
#if (NAVIGATION == 1)
#define  GF_IOC_GET_NAGV_BASE  _IOW(GF_IOC_MAGIC, 8, struct gf_nagv)
#define  GF_IOC_GET_NAGV_DATA  _IOW(GF_IOC_MAGIC, 9, struct gf_nagv)
#endif
//LINE<JIRA_ID><DATE20160319><merge from P6601>zenghaihui            
//guomingyi add start.
#define  GFX1XM_IOC_FTM	_IOW(GF_IOC_MAGIC, 101, int)
#define  GFX1XM_IOC_SET_MODE	_IOW(GF_IOC_MAGIC, 102, int)
//guomingyi add end.
//#define  GF_IOC_MAXNR    10

/*******************Refering to hardware platform*****************************/
#if FW_UPDATE
inline static void gf_miso_pullup(void)
{
	/*Config MISO pin, referring to platform.*/
	/*gpio_request_one(GF_MISO_PIN, GPIOF_IN, "gf_miso");
	s3c_gpio_cfgpin(GF_MISO_PIN, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GF_MISO_PIN, S3C_GPIO_PULL_UP);
	gpio_free(GF_MISO_PIN);*/
}

inline static void gf_miso_backnal(void)
{
	/*Config IRQ pin, referring to platform.*/	
	/*s3c_gpio_cfgpin(GF_MISO_PIN, S3C_GPIO_SFN(0x2));
	s3c_gpio_setpull(GF_MISO_PIN, S3C_GPIO_PULL_NONE);	*/
}
#endif

/********************************************************************
*CPU output low level in RST pin to reset GF. This is the MUST action for GF.
*Take care of this function. IO Pin driver strength / glitch and so on.
********************************************************************/
extern int gf_spi_read_bytes(struct gf_dev *gf_dev,
                                u16 addr, u32 data_len, u8 *rx_buf);

extern int gf_spi_write_bytes(struct gf_dev *gf_dev,
                                u16 addr, u32 data_len, u8 *tx_buf);
extern int gf_fw_update(struct gf_dev* gf_dev, unsigned char *buf, unsigned short len);
#if (NAVIGATION == 1)
extern int netlink_init(void);
extern void netlink_exit(void);
extern void sendnlmsg(char *message);
#endif
extern int gf_parse_dts(struct gf_dev* gf_dev);
extern void gf_cleanup(struct gf_dev* gf_dev);
extern int gf_power_on(struct gf_dev* gf_dev);
extern int gf_power_off(struct gf_dev* gf_dev);
extern int gf_hw_reset(struct gf_dev *gf_dev);
extern int gf_irq_num(struct gf_dev *gf_dev);
//LINE<JIRA_ID><DATE20160328><BUG_INFO>zenghaihui
extern int gf_power_ctl(struct gf_dev* gf_dev, bool on);
extern int gf_power_init(struct gf_dev* gf_dev);
extern int gf_power_deinit(struct gf_dev* gf_dev);

#endif
