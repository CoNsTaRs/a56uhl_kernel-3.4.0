/* HTC Android Linux Vibrator Device Driver
 *
 * Copyright (C) 2011 HTC Corporation.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VIB_DRV_2604_V102
 */

#define CONFIG_VIB_TRIGGERS 1
#ifdef CONFIG_VIB_TRIGGERS
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/vibtrig.h>
#include <linux/slab.h>
#include <linux/rwsem.h>
#endif
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include "../staging/android/timed_output.h"

#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/atomic.h>

#define VERSION					        "v 0.2"
#define VIB_DEVICE				"DRV2604_vibrator"
static unsigned char cali_data[3] = {0,0,0};
static uint32_t cali_node_data;
#define I2C_WRITE_RETRY_TIMES 2
static DEFINE_MUTEX(tp_wr_access);
static int ena_gpio;
static struct i2c_client *private_drv2604_client;
static bool drv2604_flag = 0;
static bool cali_flag = 0;
static bool read_cali_flag = 0;
static bool null_vib_flag = 0;

#define DRV2604_MAGIC 'A'
#define DRV2604_IOCTL_OFFSET_CALI               _IOR(DRV2604_MAGIC,      0x01, int)
#define DRV2604_IOCTL_GET_OFFSET                _IOR(DRV2604_MAGIC,      0x02, int)

#define CALIBRATION_DATA_PATH "/calibration_data"
#define SENSOR_FLASH_DATA "gyro_flash"


#define DBG_EVT_NONE		0x00000000	
#define DBG_EVT_INT		0x00000001	
#define DBG_EVT_TASKLET		0x00000002	

#define DBG_EVT_ALL		0xffffffff
#define DBG_EVT_MASK		(DBG_EVT_TASKLET)

#define MSG(evt, fmt, args...) \
do {	\
	if ((DBG_EVT_##evt) & DBG_EVT_MASK) { \
		printk(fmt, ##args); \
	} \
} while (0)

#define MSG_FUNC_ENTRY(f)	MSG(FUC, "<FUN_ENT>: %s\n", __func__)

#ifdef CONFIG_VIB_TRIGGERS
struct qpnp_vib {
	struct timed_output_dev timed_dev;
	struct vib_trigger_enabler enabler;
};
#endif

struct drv2604_chip {
	struct i2c_client	*client;
};
static struct workqueue_struct *vibrator_queue;
static struct work_struct vibrator_work;
static struct hrtimer vibe_timer;
static spinlock_t vibe_lock;
static int vibe_state;
atomic_t ldo_state;
static int shutdown_flag;

static int i2c_read_block(struct i2c_client *client,
		uint8_t addr, uint8_t *data, int length)
{
	s32 retry = 0;

	struct i2c_msg msg[] = {
	{
		.addr = client->addr,
		.flags = 0,
		.len = 1,
		.buf = &addr,
	},
	{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = length,
		.buf = data,
	}
	};
	for (retry = 0; retry < 10; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		msleep(10);
	}
	if (retry == 10) {
		printk(KERN_INFO "[VIB] %s: i2c_read_block retry over 10\n",__func__);
		return -EIO;
	}
	return 0;
}

static int i2c_write_block(struct i2c_client *client, uint8_t command,
	uint8_t *data, int length)
{
	int retry;
	int loop_i;
	uint8_t buf[length + 1];

	struct i2c_msg msg[] = {
		{
		.addr = client->addr,
		.flags = 0,
		.len = length + 1,
		.buf = buf,
		}
	};
	buf[0] = command;
	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i + 1] = data[loop_i];

	for (retry = 0; retry < I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}

	if (retry == I2C_WRITE_RETRY_TIMES) {
		printk(KERN_INFO "[VIB] %s: i2c_write_block retry over %d\n", __func__, I2C_WRITE_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

void drv2604_cali(void)
{
	struct i2c_client *client = private_drv2604_client;
	unsigned char data;
	unsigned char ret;
	int i =0;
	printk(KERN_INFO"[VIB]enter claibration \n");
	gpio_direction_output(ena_gpio, 1);
	udelay(250);

	data = 0;
	ret = i2c_write_block(client, 0x1, &data, 1);
	printk(KERN_INFO"[VIB] I2C WRITE %x to 0x1,ret =%d\n",data, ret); 
	data = 0x50;
	ret = i2c_write_block(client, 0x16, &data, 1);                    
	printk(KERN_INFO"[VIB] I2C WRITE %x to 0x16,ret =%d\n",data, ret);
	data = 0x85;
	ret = i2c_write_block(client, 0x17, &data, 1);                    
	printk(KERN_INFO"[VIB] I2C WRITE %x to 0x17,ret =%d\n",data, ret);
	data = 0xb6;
	ret = i2c_write_block(client, 0x1a, &data, 1);                    
	printk(KERN_INFO"[VIB] I2C WRITE %x to 0x1a,ret =%d\n",data, ret);
	data = 0x93;
	ret = i2c_write_block(client, 0x1b, &data, 1);                    
	printk(KERN_INFO"[VIB] I2C WRITE %d to 0x1b,ret =%d\n",data, ret);
	data = 0x80;
	ret = i2c_write_block(client, 0x1d, &data, 1);                    
	printk(KERN_INFO"[VIB] I2C WRITE %x to 0x1d,ret =%d\n",data, ret);
	data = 0x07;
	ret = i2c_write_block(client, 0x01, &data, 1);                    
	printk(KERN_INFO"[VIB] I2C WRITE %x to 0x1,ret =%d\n",data, ret);

	data = 0x20;
	ret = i2c_write_block(client, 0x1e, &data, 1);                    
	printk(KERN_INFO"[VIB] I2C WRITE %x to 0x1E,ret =%d\n",data, ret);

	data = 0x01;
	ret = i2c_write_block(client, 0x0c, &data, 1);                    
	printk(KERN_INFO"[VIB] I2C WRITE %x to 0xc,ret =%d\n",data, ret);
	data = 1;

	while (data ==1) {
		ret = i2c_read_block(client, 0x0c, &data, 1);
		printk(KERN_INFO"[VIB] I2C READ %x from 0xc,ret =%d\n",data, ret);
		msleep(100);
		++i;
		if ( i == 40) {
			printk(KERN_INFO"[VIB] I2C READ 0xc timeout \n");
			cali_flag = 0;
			return;
		}
	}

	ret = i2c_read_block(client, 0x00, &data, 1);
   	printk(KERN_INFO"[VIB] I2C READ %x from 0x0,ret =%d\n",data, ret);
	ret = i2c_read_block(client, 0x18, &cali_data[0], 1);
	printk(KERN_INFO"[VIB] I2C READ %x from 0x18,ret =%d\n",cali_data[0], ret);
	ret = i2c_read_block(client, 0x19, &cali_data[1], 1);
	printk(KERN_INFO"[VIB] I2C READ %x from 0x19,ret =%d\n",cali_data[1], ret);
	ret = i2c_read_block(client, 0x1a, &cali_data[2], 1);
	printk(KERN_INFO"[VIB] I2C READ %x from 0x1a,ret =%d\n",cali_data[2], ret);

	if (((data >> 3) & 1) == 0) {
		printk(KERN_INFO"[VIB] calibration 0x0 value valid\n");
		if( cali_data[0] == 0 || cali_data[1] == 0) {
			cali_flag = 0;
			null_vib_flag = 1;
			printk(KERN_INFO"[VIB] calibration fail\n");
		} else {
			cali_flag = 1;
			printk(KERN_INFO"[VIB] calibration success\n");
		}
	} else {
		cali_flag = 0;
		printk(KERN_INFO"[VIB] calibration fail\n");
	}

	cali_node_data = ((cali_data[0] << 16) | (cali_data[1] << 8) | cali_data[2]);

	data = (1<<6);
	i2c_write_block(client, 0x1, &data, 1);
	gpio_direction_output(ena_gpio, 0);
}

static ssize_t vib_calibration_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	cali_data[0] = (cali_node_data & (0xff << 16)) >> 16;
	cali_data[1] = (cali_node_data & (0xff << 8)) >> 8;
	cali_data[2] = (cali_node_data & (0xff));
	printk(KERN_INFO"[vib] 18:%x   19:%x    1a:%x\n", cali_data[0], cali_data[1], cali_data[2]);
	if (cali_flag) {
		printk(KERN_INFO"[VIB] calibration success\n");
		return snprintf(buf, 7, "%x%x%x\n", cali_data[0], cali_data[1], cali_data[2]);
	} else {
		if( null_vib_flag ) {
			printk(KERN_INFO"[VIB] calibration fail\n");
			return snprintf(buf, 2, "%d\n", 0);
		} else
			return snprintf(buf, 8, "-%x%x%x\n", cali_data[0], cali_data[1], cali_data[2]);
	}
}

static ssize_t vib_calibration_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int input;
	input = simple_strtoul(buf, NULL, 16);

	cali_flag = 0;
	if (input == 1)
		drv2604_cali();

	return count;
}

static DEVICE_ATTR(calibration, 0644, vib_calibration_show,
	vib_calibration_store);

static ssize_t vib_settings_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char vib_setting_data[9] = {0,0,0,0,0,0,0,0,0};
	struct i2c_client *client = private_drv2604_client;
	int ret = 0;
	printk(KERN_INFO"[VIB] cali_data:  %x  %x  %x\n", cali_data[0], cali_data[1], cali_data[2]);
	
	ret = i2c_read_block(client, 0x1, &vib_setting_data[0], 1);
	printk("[VIB] I2C READ %x from 0x1, ret = %d\n", vib_setting_data[0], ret);

	ret = i2c_read_block(client, 0x2, &vib_setting_data[1], 1);
	printk("[VIB] I2C READ %x from 0x1, ret = %d\n", vib_setting_data[0], ret);
	
	ret = i2c_read_block(client, 0x16, &vib_setting_data[2], 1);
	printk("[VIB] I2C READ %x from 0x16, ret = %d\n", vib_setting_data[1], ret);

	
	
	ret = i2c_read_block(client, 0x17, &vib_setting_data[3], 1);
	printk("[VIB] I2C READ %x from 0x17, ret = %d\n", vib_setting_data[2], ret);
	
	ret = i2c_read_block(client, 0x18, &vib_setting_data[4], 1);
	printk("[VIB] I2C READ %x from 0x18, ret = %d\n", vib_setting_data[3], ret);
	
	ret = i2c_read_block(client, 0x19, &vib_setting_data[5], 1);
	printk("[VIB] I2C READ %x from 0x19, ret = %d\n", vib_setting_data[4], ret);
	
	ret = i2c_read_block(client, 0x1a, &vib_setting_data[6], 1);
	printk("[VIB] I2C READ %x from 0x1a, ret = %d\n", vib_setting_data[5], ret);
	
	ret = i2c_read_block(client, 0x1b, &vib_setting_data[7], 1);
	printk("[VIB] I2C READ %x from 0x1b, ret = %d\n", vib_setting_data[6], ret);
	
	ret = i2c_read_block(client, 0x1d, &vib_setting_data[8], 1);
	printk("[VIB] I2C READ %x from 0x1d, ret = %d\n", vib_setting_data[7], ret);

	return snprintf(buf, 107, "DRV 2604 setting: 0x01: %x, 0x02: %x, 0x16: %x, 0x17: %x, 0x18: %x, 0x19: %x, 0x1a: %x, 0x1b: %x, 0x1d: %x\n",
			vib_setting_data[0], vib_setting_data[1], vib_setting_data[2], vib_setting_data[3], vib_setting_data[4],
			vib_setting_data[5], vib_setting_data[6], vib_setting_data[7], vib_setting_data[8]);
}

static ssize_t vib_settings_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long val;
	if ( kstrtoul(buf, 16, &val) != 0 ) {
		printk("[VIB] calibration value error\n");
		return -1;
	}
	cali_data[0] = (val & (0xff << 16)) >> 16;
	cali_data[1] = (val & (0xff << 8)) >> 8;
	cali_data[2] = (val & (0xff));
	printk (KERN_INFO"[VIB] 18:%x   19:%x    1a:%x\n", cali_data[0], cali_data[1], cali_data[2]);
	if ((cali_data[0] > 0) && (cali_data[0] < 0xff) && (cali_data[1] > 0) && (cali_data[1] < 0xff) && (cali_data[2] > 0) && (cali_data[2] < 0xff)) {
		cali_flag = 1;
		read_cali_flag = 1;
	} else {
		cali_flag = 0;
		read_cali_flag = 0;
	}
	cali_node_data = val;
	return count;
}

static DEVICE_ATTR(settings, 0644, vib_settings_show, vib_settings_store);

static int vibr_Enable(void)
{
	printk(KERN_INFO "[VIB] %s: ldo state: %d\n", __func__, atomic_read(&ldo_state));
	if (!atomic_read(&ldo_state))
		atomic_set(&ldo_state, 1);

	return 0;
}

static int vibr_Disable(void)
{
	struct i2c_client *client = private_drv2604_client;
	unsigned char data;

	printk(KERN_DEBUG "[VIB] %s: ldo state: %d\n", __func__, atomic_read(&ldo_state));
	if (atomic_read(&ldo_state)) {
		if (drv2604_flag) {
			data = (1<<6);
			i2c_write_block(client, 0x1, &data, 1);
			gpio_direction_output(ena_gpio, 0);
			printk("[VIB] drv2604 close\n");
		}
		atomic_set(&ldo_state, 0);
	}
	return 0;
}

static void update_vibrator(struct work_struct *work)
{
	if (!vibe_state)
		vibr_Disable();
	else
		vibr_Enable();
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return ktime_to_ms(r);
	} else
		return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	unsigned long flags;
	int ret = 0;

	if(drv2604_flag && (value != 0) ) {
		struct i2c_client *client = private_drv2604_client;
		unsigned char data;
		gpio_direction_output(ena_gpio, 1);
		udelay(250);

		data = 0x05;
		ret = i2c_write_block(client, 0x1, &data, 1);
		printk("[VIB] I2C WRITE %x to 0x1, ret = %d\n",data, ret);   
		printk(KERN_INFO"[VIB] Starting drv2604, value = %d\n", value);

		if (ret < 0) {
			printk ("[VIB] drv2604 I2C error, not using drv2604\n");
			drv2604_flag = 0;
		}

		if (shutdown_flag == 1) {
			data = (1<<6);
			i2c_write_block(client, 0x1, &data, 1);
			gpio_direction_output(ena_gpio, 0);
			printk(KERN_INFO"[VIB] shutdown drv2604 close\n");
		}
	}

	spin_lock_irqsave(&vibe_lock, flags);
	while (hrtimer_cancel(&vibe_timer)) {
		printk(KERN_INFO"[VIB] %s: try to cancel hrtimer\n", __func__);
	}

	if (value == 0 || shutdown_flag == 1) {
		if(drv2604_flag)
			gpio_direction_output(ena_gpio, 0);

		if(shutdown_flag)
			printk(KERN_INFO"[VIB] %s: shutdown_flag = %d\n", __func__, shutdown_flag);
		vibe_state = 0;
	} else {
		value = (value > 15000 ? 15000 : value);
		vibe_state = 1;
		hrtimer_start(&vibe_timer, ktime_set(value / 1000, (value % 1000) * 1000000), HRTIMER_MODE_REL);
	}

	spin_unlock_irqrestore(&vibe_lock, flags);
	printk(KERN_INFO"[VIB] %s: %d  flag: %d \n", __func__, value, drv2604_flag);
	update_vibrator(&vibrator_work);
}

#ifdef CONFIG_VIB_TRIGGERS
static void qpnp_vib_trigger_enable(struct vib_trigger_enabler *enabler, int value)
{
	struct qpnp_vib *vib;
	struct timed_output_dev *dev;

	vib = enabler->trigger_data;
	dev = &vib->timed_dev;

	printk(KERN_INFO "[VIB] %s: value = %d\n",__func__, value);

	vibrator_enable(dev, value);
}
#endif

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	vibe_state = 0;
	queue_work(vibrator_queue, &vibrator_work);
	printk(KERN_INFO "[VIB] hrtimer expire, queue vib work\n");
	return HRTIMER_NORESTART;
}

static struct timed_output_dev qcom_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static int vib_probe(struct platform_device *pdev)
{
	return 0;
}

static int vib_remove(struct platform_device *pdev)
{
	return 0;
}

static void vib_shutdown(struct platform_device *pdev)
{
	unsigned long flags;
	printk("[VIB] %s: enter!\n", __func__);
	spin_lock_irqsave(&vibe_lock, flags);
	shutdown_flag = 1;
	if (vibe_state) {
		printk("[VIB] %s: vibrator will disable\n", __func__);
		vibe_state = 0;
		vibr_Disable();
	}
	spin_unlock_irqrestore(&vibe_lock, flags);
}

static struct platform_driver vibrator_driver = {
	.probe = vib_probe,
	.remove = vib_remove,
	.shutdown = vib_shutdown,
	.driver = {
		   .name = VIB_DEVICE,
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device vibrator_device = {
	.name = "qcom_vibrator",
	.id = -1,
};

static ssize_t store_vibr_on(struct device *dev, struct device_attribute *attr, const char *buf,
			     size_t size)
{
	if (buf != NULL && size != 0) {
		if (buf[0] == '0') {
			vibr_Disable();
		} else {
			vibr_Enable();
		}
	}
	return size;
}

static DEVICE_ATTR(vibr_on, 0220, NULL, store_vibr_on);

static long Vib_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	
	struct device_node *sensor_offset;
	unsigned char *sensor_cali_data = NULL;
	int sensor_cali_size = 0, i = 0;
	char buf[10];
	printk(KERN_INFO"[VIB] %s: cmd = 0x%x\n", __func__, cmd);

	switch (cmd) {
		case DRV2604_IOCTL_OFFSET_CALI:
		{
			drv2604_cali();
			printk(KERN_INFO"[VIB] cali_data:  %x  %x  %x\n", cali_data[0], cali_data[1], cali_data[2]);
			printk(KERN_INFO"[VIB] cali_node_data:  %u\n", cali_node_data);
			snprintf( buf, sizeof(buf), "%u\n", cali_node_data);
			cali_data[0] = (cali_node_data & (0xff << 16)) >> 16;
			cali_data[1] = (cali_node_data & (0xff << 8)) >> 8;
			cali_data[2] = (cali_node_data & (0xff));
			printk(KERN_INFO"[vib] 18:%x   19:%x    1a:%x\n", cali_data[0], cali_data[1], cali_data[2]);
		}
		if ( copy_to_user(argp , &cali_node_data , sizeof(int)) ) {
			printk(KERN_DEBUG"[VIB] copy to user failed in DRV2604_IOCTL_OFFSET_CALI\n");
			return -1;
		}

		break;

		case DRV2604_IOCTL_GET_OFFSET:
		{
			
			sensor_offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
			if (sensor_offset) {
				sensor_cali_data = (char *)of_get_property(sensor_offset, SENSOR_FLASH_DATA,&sensor_cali_size);
				printk(KERN_INFO"[VIB] %s: sensor cali_size = %d\n", __func__, sensor_cali_size);
				if (sensor_cali_data) {
					for (i = 256; (i < sensor_cali_size) && (i < 259); i++) {
						printk(KERN_INFO"[VIB] gyro sensor cali_data[%d] = %02x\n", i, sensor_cali_data[i]);
					}
				}
			}
			
			cali_data[0] = (cali_node_data & (0xff << 16)) >> 16;
			cali_data[1] = (cali_node_data & (0xff << 8)) >> 8;
			cali_data[2] = (cali_node_data & (0xff));
			printk(KERN_INFO"[VIB] 18:%x   19:%x   1a:%x\n", cali_data[0], cali_data[1], cali_data[2]);
		}
		break;
	 }

	return 0;
}

static int Vib_open(struct inode *inode, struct file *file)
{
		return 0;
}

static int Vib_release(struct inode *inode, struct file *file)
{
		return 0;
}

static const struct file_operations vib_fops =
{
	.owner = THIS_MODULE,
	.open = Vib_open,
	.release = Vib_release,
	.unlocked_ioctl = Vib_ioctl,
};

static struct miscdevice vib_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "DRV2604-VIB",
	.fops = &vib_fops,
};



static int drv2604_vib_probe(struct i2c_client *client
		, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct drv2604_chip		*cdata;
	struct property *prop;
	struct device_node *dt = dev->of_node;
	
	int ret =0;
	int i = 0, err = 0;
	unsigned char data = 0;
	char data1[3] = {0};
	const char *parser_st[] = {"enable-gpio2"};
	struct device_node *sensor_offset;
	unsigned char *sensor_cali_data = NULL;
	int sensor_cali_size = 0;
#ifdef CONFIG_VIB_TRIGGERS
	struct vib_trigger_enabler *enabler;
	extern void vib_trigger_enabler_register(struct vib_trigger_enabler *enabler);
#endif

	null_vib_flag = 0;
	printk("[VIB][PROBE] drv2604 driver probe , CONFIG_VIB_TRIGGERS is %d +++\n", CONFIG_VIB_TRIGGERS);
	cdata = devm_kzalloc(&client->dev, sizeof(*cdata), GFP_KERNEL);

	err = misc_register(&vib_device);
	if (err) {
		dev_err(&client->dev, "Failed to register misc device !\n");
		goto exit_err;
	}

	if (!cdata) {
		ret = -ENOMEM;
		printk("[VIB][PROBE_ERR] failed on allocat cdata\n");
	}
	i2c_set_clientdata(client, cdata);
	private_drv2604_client = client;

	prop = of_find_property(dt, "enable-gpio2", NULL);
	if (prop) {
		of_property_read_u32(dt, "enable-gpio2", &ena_gpio);
		printk("[VIB] in of get: %d \n",ena_gpio);
	}
	ena_gpio = of_get_named_gpio(dt, parser_st[0], 0);
	printk("[VIB] en_gpio :%d\n",ena_gpio);

	ret = gpio_request(ena_gpio, "enable_gpio");
	if (ret) {
		dev_err(&client->dev, "[VIB] Failed to request gpio\n");
		return -ENODATA;
		ena_gpio = 0;
	}

	printk(KERN_INFO "[VIB] GPIO is %d\n", ena_gpio);
	gpio_direction_output(ena_gpio, 1);
	udelay(250);

	ret = i2c_read_block(client, 0x18, &data1[0], 1);
	if (ret == 0)
	{
		printk("[VIB] I2C READ %x from 0x18,ret =%d\n",data1[0], ret);
		drv2604_flag = 1;
	} else {
		printk ("[VIB] drv2604 I2C error, not using drv2604\n");
		drv2604_flag = 0;
	}

	if (drv2604_flag) {
		data = 0;
		ret = i2c_write_block(client, 0x1, &data, 1);		   
		printk("[VIB] I2C WRITE %x to 0x1,ret =%d\n",data, ret);
		data = 0x7f;
		ret = i2c_write_block(client, 0x2, &data, 1);
		printk("[VIB] I2C WRITE %x to 0x2,ret =%d\n",data, ret);   
		data = 0x50;
		ret = i2c_write_block(client, 0x16, &data, 1);             
		printk("[VIB] I2C WRITE %x to 0x16,ret =%d\n",data, ret);
		
		data = 0x85;
		ret = i2c_write_block(client, 0x17, &data, 1);             
		printk("[VIB] I2C WRITE %x to 0x17,ret =%d\n",data, ret);
		data = 0xb6;
		ret = i2c_write_block(client, 0x1a, &data, 1);             
		printk("[VIB] I2C WRITE %x to 0x1a,ret =%d\n",data, ret);
		data = 0x93;
		ret = i2c_write_block(client, 0x1b, &data, 1);             
		printk("[VIB] I2C WRITE %d to 0x1b,ret =%d\n",data, ret);
		data = 0x80;
		ret = i2c_write_block(client, 0x1d, &data, 1);             
		printk("[VIB] I2C WRITE %x to 0x1d,ret =%d\n",data, ret);
		data = (1<<6);
		i2c_write_block(client, 0x1, &data, 1);                    
		printk("[VIB] I2C WRITE %x to 0x1,ret =%d\n",data, ret);

		
		sensor_offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
		if (sensor_offset) {
			sensor_cali_data = (char *)of_get_property(sensor_offset, SENSOR_FLASH_DATA,&sensor_cali_size);
			printk(KERN_INFO"[VIB] %s: sensor cali_size = %d\n", __func__, sensor_cali_size);
			if (sensor_cali_data) {
				for (i = 256; (i < sensor_cali_size) && (i < 259); i++)
					printk(KERN_INFO"[VIB] vib cali_data[%d] = %02x\n", i, sensor_cali_data[i]);

				read_cali_flag = 0;
				ret = i2c_write_block(client, 0x18, &sensor_cali_data[258], 1);
				printk(KERN_INFO"[VIB] I2C WRITE %x from 0x18,ret =%d\n", sensor_cali_data[258], ret);
				ret = i2c_write_block(client, 0x19, &sensor_cali_data[257], 1);
				printk(KERN_INFO"[VIB] I2C WRITE %x from 0x19,ret =%d\n", sensor_cali_data[257], ret);
				ret = i2c_write_block(client, 0x1a, &sensor_cali_data[256], 1);
				printk(KERN_INFO"[VIB] I2C WRITE %x from 0x1a,ret =%d\n", sensor_cali_data[256], ret);
				cali_data[0] = sensor_cali_data[258];
				cali_data[1] = sensor_cali_data[257];
				cali_data[2] = sensor_cali_data[256];
			} else
				printk(KERN_INFO"[VIB] no vib cali_data\n");
		} else
			printk(KERN_INFO"[VIB] %s: no sensor cali data\n", __func__);

		
	}
	gpio_direction_output(ena_gpio, 0);

	printk("[VIB] %s: Qpnp vibrator driver register, version %s\n", __func__, VERSION);
#ifdef CONFIG_VIB_TRIGGERS
	enabler = kzalloc(sizeof(*enabler), GFP_KERNEL);
	if(!enabler)
	return -ENOMEM;
#endif
	ret = platform_device_register(&vibrator_device);
	if (ret != 0) {
		printk("[VIB] %s: Unable to register vibrator device (%d)\n", __func__, ret);
		return ret;
	}

	vibrator_queue = create_singlethread_workqueue(VIB_DEVICE);
	if (!vibrator_queue) {
		printk("[VIB] %s: Unable to create workqueue\n", __func__);
		return -ENODATA;
	}
	INIT_WORK(&vibrator_work, update_vibrator);

	spin_lock_init(&vibe_lock);
	shutdown_flag = 0;
	vibe_state = 0;
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&qcom_vibrator);

	ret = platform_driver_register(&vibrator_driver);

	if (ret) {
		printk("[VIB] %s: Unable to register vibrator driver (%d)\n", __func__, ret);
		return ret;
	}

	ret = device_create_file(qcom_vibrator.dev, &dev_attr_vibr_on);
	if (ret) {
		printk("[VIB] %s: device_create_file vibr_on fail!\n", __func__);
	}

	ret = device_create_file(qcom_vibrator.dev, &dev_attr_calibration);
	if (ret < 0) {
		pr_err("%s: failed on create attr calibration [%d]\n", __func__, i);
	}

	ret = device_create_file(qcom_vibrator.dev, &dev_attr_settings);
	if (ret < 0) {
		pr_err("%s: failed on create attr settings [%d]\n", __func__, i);
	}

#ifdef CONFIG_VIB_TRIGGERS
	enabler->name = "qpnp-vibrator";
	enabler->default_trigger = "vibrator";
	enabler->enable = qpnp_vib_trigger_enable;
	vib_trigger_enabler_register(enabler);
#endif

	printk("[VIB] %s: Done, flag is %d \n", __func__, drv2604_flag);

	return 0;

exit_err:
	return err;
}

static const struct i2c_device_id vibrator_i2c_id[] = {
	{ "DRV2604-VIB", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, vibrator_i2c_id);

static const struct of_device_id drv2604_mttable[] = {
	{ .compatible = "ti,drv2604l"},
	{ },
};

static struct i2c_driver vibrator_i2c_driver = {
	.driver = {
	.owner = THIS_MODULE,
	.name = "DRV2604-VIB",
	.of_match_table = drv2604_mttable,
	},
	.id_table = vibrator_i2c_id,
	.probe = drv2604_vib_probe,
	
};

static int __init drv260x_init(void)
{
	printk("[VIB] drv2604_%s Enter\n", __func__);
	return i2c_add_driver(&vibrator_i2c_driver);
}

late_initcall_sync(drv260x_init);

MODULE_AUTHOR("HTC touch team");
MODULE_DESCRIPTION("Qcom Vibrator Driver (VIB)");
MODULE_LICENSE("GPL");
