/* driver/leds/leds-lp5521_htc.c
 *
 * Copyright (C) 2010 HTC Corporation.
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
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/leds-lp5521_htc_vk.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>

#define LP5521_MAX_LEDS			9	
#define LED_DEBUG				1
#if LED_DEBUG
#define D(x...) printk(KERN_DEBUG "[LED]" x)
#define I(x...) printk(KERN_INFO "[LED]" x)
#else
#define D(x...)
#define I(x...)
#endif

static int led_rw_delay, chip_enable, vk_enable;
static struct i2c_client *private_lp5521_client = NULL;
static struct mutex	led_mutex;
static struct mutex	vk_led_mutex;
static struct workqueue_struct *g_led_work_queue;
static int VK_brightness;
static int last_pwm;
static uint8_t virtual_key_led_reset_flag = 0;
static u8 color_table[13] = {0};
static int use_color_table = 0;
static u8 current_table[13] = {0};
static int use_current_table = 0;
static int table_level_num = 0;
static struct led_i2c_platform_data *plat_data;
#define Mode_Mask (0xff << 24)
#define Red_Mask (0xff << 16)
#define Green_Mask (0xff << 8)
#define Blue_Mask 0xff

#define RG_Mask 0x3C
#define VK_Mask 0x03

#define VK_LED_FADE_LEVEL  16
#define VL_LED_FADE_TIME   125
#define VK_LED_SLEEP_TIME  115

static int gCurrent_param = 95;
module_param(gCurrent_param, int, 0600);

struct lp5521_led {
	int			id;
	u8			chan_nr;
	u8			led_current;
	u8			max_current;
	struct led_classdev	cdev;
	struct mutex 		led_data_mutex;
	struct work_struct 	led_work;
	struct work_struct 	led_work_multicolor;
	uint8_t 		Mode;
	uint8_t			Red;
	uint8_t 		Green;
	uint8_t 		Blue;
	int			VK_brightness;
	struct delayed_work	blink_delayed_work;
	struct wake_lock        led_wake_lock;
};

struct lp5521_chip {
	struct led_i2c_platform_data *pdata;
	struct mutex		led_i2c_rw_mutex; 
	struct i2c_client	*client;
	struct lp5521_led	leds[LP5521_MAX_LEDS];
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_init;
};

static int lp5521_parse_dt(struct device *, struct led_i2c_platform_data *);

static char *hex2string(uint8_t *data, int len)
{
	static char buf[LED_I2C_WRITE_BLOCK_SIZE*4];
	int i;

	i = LED_I2C_WRITE_BLOCK_SIZE -1;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}

static int i2c_write_block(struct i2c_client *client, uint8_t addr,
		uint8_t *data, int length)
{
	int retry;
	uint8_t buf[LED_I2C_WRITE_BLOCK_SIZE];
	int i;
	struct lp5521_chip *cdata;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	dev_dbg(&client->dev, "W [%02X] = %s\n",
			addr, hex2string(data, length));

	cdata = i2c_get_clientdata(client);
	if (length + 1 > LED_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "[LED] i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	for (i = 0; i < length; i++)
		buf[i+1] = data[i];

	mutex_lock(&cdata->led_i2c_rw_mutex);
	msleep(1);
	for (retry = 0; retry < I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(led_rw_delay);
	}
	if (retry >= I2C_WRITE_RETRY_TIMES) {
		dev_err(&client->dev, "[LED] i2c_write_block retry over %d times\n",
				I2C_WRITE_RETRY_TIMES);
		mutex_unlock(&cdata->led_i2c_rw_mutex);
		return -EIO;
	}
	mutex_unlock(&cdata->led_i2c_rw_mutex);

	return 0;
}


static int I2C_RxData_2(char *rxData, int length)
{
	uint8_t loop_i;

	struct i2c_msg msgs[] = {
		{
			.addr = private_lp5521_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = private_lp5521_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (loop_i = 0; loop_i < I2C_WRITE_RETRY_TIMES; loop_i++) {
		if (i2c_transfer(private_lp5521_client->adapter, msgs, 2) > 0)
			break;
		msleep(10);
	}

	if (loop_i >= I2C_WRITE_RETRY_TIMES) {
		printk(KERN_ERR "[LED] %s retry over %d times\n",
				__func__, I2C_WRITE_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int i2c_read_block(struct i2c_client *client,
		uint8_t cmd, uint8_t *pdata, int length)
{
	char buffer[3] = {0};
	int ret = 0, i;

	if (pdata == NULL)
		return -EFAULT;

	if (length > 2) {
		pr_err("[LED]%s: length %d> 2: \n", __func__, length);
		return ret;
	}
	buffer[0] = cmd;
	ret = I2C_RxData_2(buffer, length);
	if (ret < 0) {
		pr_err("[LED]%s: I2C_RxData fail \n", __func__);
		return ret;
	}

	for (i = 0; i < length; i++) {
		*(pdata+i) = buffer[i];
	}
	return ret;
}
#if 0
static int write_enable_register(struct i2c_client *client, uint8_t data, uint8_t vk_led)
{
	int ret = 0;
	uint8_t temp = 0, current_data = 0;

	if(!rgb_enable && !vk_enable && data == 0) {
		I("write_enable_register disable\n");
		ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
		return ret;
	}
	else
		temp = 0x40;

	ret = i2c_read_block(client, ENABLE_REGISTER, &current_data, 1);
	if(vk_led)
		temp |= (current_data & RG_Mask) | (data & VK_Mask);
	else
		temp |= (data & RG_Mask) | (current_data & VK_Mask);

	ret = i2c_write_block(client, ENABLE_REGISTER, &temp, 1);
	return ret;
}
static int write_operation_register(struct i2c_client *client, uint8_t data, uint8_t vk_led)
{
	int ret = 0;
	uint8_t temp = 0, current_data = 0;

	ret = i2c_read_block(client, OPRATION_REGISTER, &current_data, 1);

	if(vk_led)
		temp |= (current_data & RG_Mask) | (data & VK_Mask);
	else
		temp |= (data & RG_Mask) | (current_data & VK_Mask);

	ret = i2c_write_block(client, OPRATION_REGISTER, &temp, 1);
	return ret;
}
#endif
static int write_vk_led_program(struct i2c_client *client)
{
	int ret = 0, reg_index = 0;
	uint8_t data, step_time, target_pwm;
	uint8_t command_data[32] = {0};

	data = 0x15;
	ret = i2c_write_block(client, OPRATION_REGISTER, &data, 1);


	
	target_pwm = VK_LED_FADE_LEVEL * 1 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	
	target_pwm = VK_LED_FADE_LEVEL * 2 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	
	target_pwm = VK_LED_FADE_LEVEL * 3 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	
	target_pwm = VK_LED_FADE_LEVEL * 4 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	
	target_pwm = VK_LED_FADE_LEVEL * 5 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	
	target_pwm = VK_LED_FADE_LEVEL * 6 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	
	target_pwm = VK_LED_FADE_LEVEL * 7 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	
	target_pwm = VK_LED_FADE_LEVEL * 8 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	
	target_pwm = VK_LED_FADE_LEVEL * 1 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	
	target_pwm = VK_LED_FADE_LEVEL * 2 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	
	target_pwm = VK_LED_FADE_LEVEL * 3 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	
	target_pwm = VK_LED_FADE_LEVEL * 4 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	
	target_pwm = VK_LED_FADE_LEVEL * 5 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	
	target_pwm = VK_LED_FADE_LEVEL * 6 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	
	target_pwm = VK_LED_FADE_LEVEL * 7 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	
	target_pwm = VK_LED_FADE_LEVEL * 8 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); 
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	ret = i2c_write_block(client, CMD_ENG_1_BASE, command_data, 32);
	ret = i2c_write_block(client, CMD_ENG_2_BASE, command_data, 32);
	ret = i2c_write_block(client, CMD_ENG_3_BASE, command_data, 32);

	data = 0x2A;
	ret = i2c_write_block(client, OPRATION_REGISTER, &data, 1);

	return ret;
}

static void lp5521_led_enable(struct i2c_client *client, int blink_enable)
{
	int ret = 0;
	uint8_t data;

	char data1[1] = {0};

	I(" %s +++\n" , __func__);

	if (chip_enable) {
		I(" %s return, chip already enable\n" , __func__);
		return;
	}

	
	if (plat_data->ena_gpio) {
		ret = gpio_direction_output(plat_data->ena_gpio, 1);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
			gpio_free(plat_data->ena_gpio);
		}
	} 

	msleep(1);
#if 1
	if (blink_enable) {
		
		data = 0xFF;
		ret = i2c_write_block(client, RESET_CONTROL, &data, 1);
		msleep(20);
		ret = i2c_read_block(client, R_CURRENT_CONTROL, data1, 1);
		if (data1[0] != 0xaf) {
			I(" %s reset not ready %x\n" , __func__, data1[0]);
		}
	}
#endif
	chip_enable = 1;
	mutex_lock(&led_mutex);
	
	data = 0x40;
	ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
	udelay(550);
	
	data = 0x29;
	ret = i2c_write_block(client, CONFIG_REGISTER, &data, 1);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5521_led_disable(struct i2c_client *client)
{
	int ret = 0;
	I(" %s +++\n" , __func__);

	if (!chip_enable) {
		I(" %s return, chip already disable\n" , __func__);
		return;
	}

	if (plat_data->ena_gpio) {
		ret = gpio_direction_output(plat_data->ena_gpio, 0);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
			gpio_free(plat_data->ena_gpio);
		}
	}
	chip_enable = 0;

	I(" %s ---\n" , __func__);
}

static int virtual_key_led_change_pwm(struct i2c_client *client, int pwm_diff)
{
	int target_pwm_diff, ret = 0;
	uint8_t pc_addr1, pc_addr2, pwm_level, data, data_array[3];

	if(pwm_diff > 0){
		target_pwm_diff = pwm_diff;
		pc_addr1 = 0;
		pc_addr2 = 0;
	}
	else if (pwm_diff < 0){
		target_pwm_diff = -pwm_diff;
		pc_addr1 = 8;
		pc_addr2 = 8;
	}
	else {
		I("%s: pwm no changed, return.\n", __func__);
		return ret;
	}
	pwm_level = target_pwm_diff / VK_LED_FADE_LEVEL;
	if(pwm_level / 2) {
		pc_addr2 += (pwm_level / 2) - 1;
		pc_addr1 = pc_addr2 + (pwm_level % 2);
	}
	data_array[0] = pc_addr1;
	data_array[1] = pc_addr1;
	data_array[2] = pc_addr1;

	ret = i2c_write_block(client, ENG_1_PC_CONTROL, data_array, 3);
	data = 0x7F;
	ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
	if(pwm_level / 2) {
		msleep(VK_LED_SLEEP_TIME);
		data_array[0] = pc_addr2;
		data_array[1] = pc_addr2;
		data_array[2] = pc_addr2;

		ret = i2c_write_block(client, ENG_1_PC_CONTROL, data_array, 3);
		data = 0x7F;
		ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
	}
	msleep(VK_LED_SLEEP_TIME);

	return ret;
}

void virtual_key_led_reset_blink(int onoff)
{
	struct i2c_client *client = private_lp5521_client;
	int ret = 0, reg_index = 0;
	int target_pwm;
	uint8_t data;
	uint8_t command_data[10] = {0};

	if(!client)
		return;

	I("virtual_key_led_reset_blink +++, onoff = %d\n", onoff);

	if(onoff) {
		virtual_key_led_reset_flag = 1;
		lp5521_led_enable(client, 0);

		data = 0;
		ret = i2c_write_block(client, ENG_1_PC_CONTROL, &data, 1);
		ret = i2c_write_block(client, ENG_2_PC_CONTROL, &data, 1);
		ret = i2c_write_block(client, ENG_3_PC_CONTROL, &data, 1);

		data = 0x15;
		ret = i2c_write_block(client, OPRATION_REGISTER, &data, 1);

		
		command_data[reg_index++] = 0x40;
		command_data[reg_index++]  = 0x00;

		
		command_data[reg_index++] = 0x54;
		command_data[reg_index++]  = 0x00;

		
		command_data[reg_index++] = 0x40;
		command_data[reg_index++]  = 0xFF;

		
		command_data[reg_index++] = 0x54;
		command_data[reg_index++]  = 0x00;

		
		command_data[reg_index++] = 0x00;
		command_data[reg_index++]  = 0x00;

		ret = i2c_write_block(client, CMD_ENG_1_BASE, command_data, 10);
		ret = i2c_write_block(client, CMD_ENG_2_BASE, command_data, 10);
		ret = i2c_write_block(client, CMD_ENG_3_BASE, command_data, 10);

		data = 0x2A;
		ret = i2c_write_block(client, OPRATION_REGISTER, &data, 1);
		data = 0x6A;
		ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
	} else {
		virtual_key_led_reset_flag = 0;
		data = 0x40;
		ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
		write_vk_led_program(client);

		mutex_lock(&vk_led_mutex);
		target_pwm = VK_brightness / VK_LED_FADE_LEVEL * VK_LED_FADE_LEVEL;
		ret = virtual_key_led_change_pwm(client, (int)target_pwm);
		last_pwm = target_pwm;
		if(last_pwm)
			vk_enable = 1;
		else
			vk_enable = 0;
		mutex_unlock(&vk_led_mutex);

		if(!vk_enable)
			lp5521_led_disable(client);
	}
}

EXPORT_SYMBOL(virtual_key_led_reset_blink);

static void virtual_key_led_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_lp5521_client;
	struct lp5521_led *ldata;
	int ret, reg_index = 0;
	int target_pwm;
	uint8_t data, change_current = 0;

	ldata = container_of(work, struct lp5521_led, led_work);

	if(ldata->VK_brightness) {
		if(use_color_table && ldata->VK_brightness < table_level_num){
			if(use_current_table) {
				if(gCurrent_param != current_table[ldata->VK_brightness] && ldata->VK_brightness != 0) {
					gCurrent_param = current_table[ldata->VK_brightness];
					change_current = 1;
				}
			}
			ldata->VK_brightness = VK_brightness;
		}
		target_pwm = ldata->VK_brightness / VK_LED_FADE_LEVEL * VK_LED_FADE_LEVEL;
		if (vk_enable) {
		I(" %s virtual key already enable, change brightness\n" , __func__);

		if(change_current) {
			data = (u8)gCurrent_param;
			ret = i2c_write_block(client, R_CURRENT_CONTROL, &data, 1);
			ret = i2c_write_block(client, G_CURRENT_CONTROL, &data, 1);
			ret = i2c_write_block(client, B_CURRENT_CONTROL, &data, 1);
		}
		mutex_lock(&vk_led_mutex);
		ret = virtual_key_led_change_pwm(client, (int)(target_pwm - last_pwm));
		last_pwm = target_pwm;
		mutex_unlock(&vk_led_mutex);
		return;
		}

		lp5521_led_enable(client, 0);
		data = (u8)gCurrent_param;
		ret = i2c_write_block(client, R_CURRENT_CONTROL, &data, 1);
		ret = i2c_write_block(client, G_CURRENT_CONTROL, &data, 1);
		ret = i2c_write_block(client, B_CURRENT_CONTROL, &data, 1);
		write_vk_led_program(client);
		vk_enable = 1;
		reg_index = 0;

		mutex_lock(&vk_led_mutex);
		ret = virtual_key_led_change_pwm(client, (int)target_pwm);
		last_pwm = target_pwm;
		mutex_unlock(&vk_led_mutex);
	}else {
		if (!vk_enable) {
			I(" %s return, virtual key already disable\n" , __func__);
			return;
		}
		mutex_lock(&vk_led_mutex);
		ret = virtual_key_led_change_pwm(client, -last_pwm);
		last_pwm = 0;
		mutex_unlock(&vk_led_mutex);
		queue_delayed_work(g_led_work_queue, &ldata->blink_delayed_work, msecs_to_jiffies(VK_LED_SLEEP_TIME));
	}
}

static void led_fade_do_work(struct work_struct *work)
{
	struct i2c_client *client = private_lp5521_client;
	struct lp5521_led *ldata;

	ldata = container_of((struct delayed_work *)work, struct lp5521_led, blink_delayed_work);

	if(!ldata->VK_brightness) {
		vk_enable = 0;
		lp5521_led_disable(client);
	}
}

enum led_brightness lp5521_vk_led_get_brightness(struct led_classdev *led_cdev)
{
	return VK_brightness;
}

static void lp5521_vk_led_set_brightness(struct led_classdev *led_cdev,
					  enum led_brightness brightness)
{
	struct lp5521_led *ldata;

	ldata = container_of(led_cdev, struct lp5521_led, cdev);

	ldata->VK_brightness = brightness == LED_FULL? 256 : brightness;

	if(use_color_table && brightness < table_level_num){
		I("color_table[%d] = %x, current_table[%d] = %d\n", brightness, color_table[brightness], brightness, current_table[brightness]);
		brightness = color_table[brightness];
	}

	VK_brightness = brightness == LED_FULL? 256 : brightness;
	I(" %s , VK_brightness = %u\n" , __func__, VK_brightness);
	if(!virtual_key_led_reset_flag)
		queue_work(g_led_work_queue, &ldata->led_work);
	return;
}

#if 0
static ssize_t lp5521_led_i2c_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	char data[1] = {0};
	int i;
	struct i2c_client *client = private_lp5521_client;

	for (i = 0; i <= 0x6f; i++) {
		ret = i2c_read_block(client, i, data, 1);
		I(" %s i2c(%x) = 0x%x\n", __func__, i, data[0]);
	}
	return ret;
}

static ssize_t lp5521_led_i2c_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = private_lp5521_client;
	int i, ret;
	char *token[10];
	unsigned long ul_reg, ul_data = 0;
	uint8_t reg = 0, data;
	char value[1] = {0};
	struct led_i2c_platform_data *pdata;
	pdata = client->dev.platform_data;

	for (i = 0; i < 2; i++) {
		token[i] = strsep((char **)&buf, " ");
		D("%s: token[%d] = %s\n", __func__, i, token[i]);
	}
	ret = strict_strtoul(token[0], 16, &ul_reg);
	ret = strict_strtoul(token[1], 16, &ul_data);

	reg = ul_reg;
	data = ul_data;

	if (reg < 0x6F) {
		ret = i2c_write_block(client, reg, &data, 1);
		ret = i2c_read_block(client, reg, value, 1);
		I(" %s , ret = %d, Set REG=0x%x, data=0x%x\n" , __func__, ret, reg, data);
		ret = i2c_read_block(client, reg, value, 1);
		I(" %s , ret = %d, Get REG=0x%x, data=0x%x\n" , __func__, ret, reg, value[0]);
	}
	if (reg == 0x99) {
		if (data == 1) {
			I("%s , pull up enable pin\n", __func__);
			if (pdata->ena_gpio) {
				ret = gpio_direction_output(pdata->ena_gpio, 1);
				if (ret < 0) {
					pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
					gpio_free(pdata->ena_gpio);
				}
			}
		} else if (data == 0) {
			I("%s , pull down enable pin\n", __func__);
			if (pdata->ena_gpio) {
				ret = gpio_direction_output(pdata->ena_gpio, 1);
				if (ret < 0) {
					pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
					gpio_free(pdata->ena_gpio);
				}
			}
		}
	}
	return count;
}

static DEVICE_ATTR(i2c, 0644, lp5521_led_i2c_show, lp5521_led_i2c_store);
#endif
#if 0
static int lp5521_pinctrl_init(struct lp5521_chip *cdata, struct device *dev){

	int retval, ret;

	
	D("[LED]LP5521_pinctrl_init");

	cdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(cdata->pinctrl)) {
		retval = PTR_ERR(cdata->pinctrl);
		pr_err("[LED][lp5521 error]%s: Target does not use pinctrl\n", __func__);
		cdata->pinctrl = NULL;
		goto err_pinctrl_get;
	}
	cdata->gpio_state_init = pinctrl_lookup_state(cdata->pinctrl, "lp5521_init");
	if (IS_ERR_OR_NULL(cdata->gpio_state_init)) {
		pr_err("[LED][lp5521 error]%s: Cannot get pintctrl state\n", __func__);
		retval = PTR_ERR(cdata->gpio_state_init);
		cdata->pinctrl = NULL;
		return retval;
	}
	ret = pinctrl_select_state(cdata->pinctrl, cdata->gpio_state_init);
	if (ret) {
		pr_err("[LED][LP5521 error]%s: Cannot init INT gpio\n", __func__);
		return ret;
	}

	return 0;

err_pinctrl_get:
	cdata->pinctrl = NULL;
	return retval;
}
#endif

#define BLACK_ID 1
#define WHITE_ID 2

static void get_brightness_mapping_table(struct device_node *node)
{
	struct property *prop;
	int current_table_level_num;
	int color_ID = 0;
	const char* cmdline;
	char* temp_cmdline;
	prop = of_find_property(node, "vk-pwm-array",
					&table_level_num);
	if(!prop) {
		use_color_table = 0;
		return;
	}
	I("%s, vk-pwm-array table_level_num: %d\n", __func__, table_level_num);
	use_color_table = 1;
	memcpy(color_table, prop->value, table_level_num);

	prop = of_find_property(node, "vk-current-array",
					&current_table_level_num);
	if(!prop) {
	use_current_table = 0;
	} else {
	use_current_table = 1;
	memcpy(current_table, prop->value, current_table_level_num);
	}

	cmdline = kstrdup(saved_command_line, GFP_KERNEL);
	if (cmdline) {
		I("Get cmdline success\n");
		temp_cmdline = strstr(cmdline, "color_ID=");
		if(temp_cmdline == NULL) {
			I("No color_ID at devices\n");
			kfree(cmdline);
			return;
		}
		if(strncmp(temp_cmdline, "color_ID=DEF02", strlen("color_ID=DEF02")) == 0) {
			color_ID = WHITE_ID;
			I("color_ID match DEF02, use color_ID: %d\n", color_ID);
		} else if(strncmp(temp_cmdline, "color_ID=GRY01", strlen("color_ID=GRY01")) == 0) {
			color_ID = BLACK_ID;
			I("color_ID match GRY01, use color_ID: %d\n", color_ID);
		} else if(strncmp(temp_cmdline, "color_ID=GRY00", strlen("color_ID=GRY00")) == 0) {
			color_ID = BLACK_ID;
			I("color_ID match GRY00, use color_ID: %d\n", color_ID);
		} else {
			I("No color_ID matched\n");
		}
		kfree(cmdline);
	} else {
		I("Get cmdline failed\n");
	}

	if(color_ID == BLACK_ID) {
		prop = of_find_property(node, "vk-black-pwm-array",
			&table_level_num);
		if(!prop) {
			I("Not use color_table\n");
		} else {
			memcpy(color_table, prop->value, table_level_num);
		}
	} else if(color_ID == WHITE_ID) {
		prop = of_find_property(node, "vk-white-pwm-array",
			&table_level_num);
		if(!prop) {
			I("Not use color_table\n");
		} else {
			memcpy(color_table, prop->value, table_level_num);
		}
	}
}

static int lp5521_parse_dt(struct device *dev, struct led_i2c_platform_data *pdata)
{
	struct property *prop;
	struct device_node *dt = dev->of_node;
	prop = of_find_property(dt, "lp5521,lp5521_en", NULL);
	if (prop) {
		pdata->ena_gpio = of_get_named_gpio(dt, "lp5521,lp5521_en", 0);
	}
	prop = of_find_property(dt, "lp5521,num_leds", NULL);
	if (prop) {
		of_property_read_u32(dt, "lp5521,num_leds", &pdata->num_leds);
	}
	prop = of_find_property(dt, "lp5521,current_param", NULL);
	if (prop) {
		of_property_read_u32(dt, "lp5521,current_param", &gCurrent_param);
     	}
	return 0;
}

static int lp5521_led_probe(struct i2c_client *client
		, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct lp5521_chip		*cdata;
	struct led_i2c_platform_data *pdata;
	int ret =0;
	int i;

	printk("[LED][PROBE] led driver probe +++\n");

	
	cdata = kzalloc(sizeof(struct lp5521_chip), GFP_KERNEL);
	if (!cdata) {
		ret = -ENOMEM;
		printk("[LED][PROBE_ERR] failed on allocat cdata\n");
		goto err_cdata;
	}

	i2c_set_clientdata(client, cdata);
	cdata->client = client;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL) {
		ret = -ENOMEM;
		goto err_exit;
	}
	ret = lp5521_parse_dt(&client->dev, pdata);
#if 0
	
	if (lp5521_pinctrl_init(cdata, &client->dev) < 0) {
		pr_err("[LED] pinctrl setup failed");
	}
#endif
	led_rw_delay = 5;
	
	if (pdata->ena_gpio) {
		ret = gpio_request(pdata->ena_gpio, "led_enable");
		if (ret < 0) {
			pr_err("[LED] %s: gpio_request failed ena gpio %d\n", __func__, ret);
			goto err_request_ena_gpio;
		}
		ret = gpio_direction_output(pdata->ena_gpio, 1);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(pdata->ena_gpio);
			goto err_request_ena_gpio;
		}
	} 
	
	if (pdata->tri_gpio) {
		ret = gpio_request(pdata->tri_gpio, "led_trigger");
		if (ret < 0) {
			pr_err("[LED] %s: gpio_request failed led trigger %d\n", __func__, ret);
		}
		ret = gpio_direction_output(pdata->tri_gpio, 0);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(pdata->tri_gpio);
		}
	}
	private_lp5521_client = client;

	g_led_work_queue = create_singlethread_workqueue("led");
	if (!g_led_work_queue) {
		ret = -10;
		pr_err("[LED] %s: create workqueue fail %d\n", __func__, ret);
		goto err_create_work_queue;
	}
	for (i = 0; i < pdata->num_leds; i++) {
		I("VK probe, i = %d, num_leds = %d\n", i, pdata->num_leds);
		cdata->leds[i].cdev.name = "button-backlight";
		ret = led_classdev_register(dev, &cdata->leds[i].cdev);
		if (ret < 0) {
			dev_err(dev, "couldn't register led[%d]\n", i);
			goto err_register_attr_ModeRGB;
		}

		get_brightness_mapping_table(client->dev.of_node);

		cdata->leds[i].cdev.brightness_set = lp5521_vk_led_set_brightness;
		cdata->leds[i].cdev.brightness_get = lp5521_vk_led_get_brightness;

		INIT_WORK(&cdata->leds[i].led_work, virtual_key_led_work_func);
		
		INIT_DELAYED_WORK(&cdata->leds[i].blink_delayed_work, led_fade_do_work);
		mutex_init(&vk_led_mutex);
	}
	mutex_init(&cdata->led_i2c_rw_mutex);
	mutex_init(&led_mutex);
	plat_data = pdata;
	
#if 0
	
	data = 0x00;
	ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
	udelay(550);
	if (pdata->ena_gpio) {
		gpio_direction_output(pdata->ena_gpio, 0);
	} 
#endif
	printk("[LED][PROBE] led driver probe ---\n");

	return 0;

err_register_attr_ModeRGB:
err_create_work_queue:
err_request_ena_gpio:
	kfree(pdata);
err_exit:
	kfree(cdata);
err_cdata:
	return ret;
}

static int lp5521_led_remove(struct i2c_client *client)
{
	struct lp5521_chip *cdata;
	int i,ret;

	cdata = i2c_get_clientdata(client);

	ret = lp5521_parse_dt(&client->dev, plat_data);
	if (plat_data->ena_gpio) {
		gpio_direction_output(plat_data->ena_gpio, 0);
	} 
	for (i = 0; i < plat_data->num_leds; i++) {
		led_classdev_unregister(&cdata->leds[i].cdev);
	}
	destroy_workqueue(g_led_work_queue);
	kfree(cdata);

	return 0;
}


static const struct i2c_device_id led_i2c_id[] = {
	{ "LP5521-LED", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, led_i2c_id);

static const struct of_device_id lp5521_mttable[] = {
	{ .compatible = "LP5521-VK_LED"},
	{ },
};

static struct i2c_driver led_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "LP5521-VK_LED",
		.of_match_table = lp5521_mttable,
	},
	.id_table = led_i2c_id,
	.probe = lp5521_led_probe,
	.remove = lp5521_led_remove,
};
module_i2c_driver(led_i2c_driver);
MODULE_AUTHOR("<ShihHao_Shiung@htc.com>, <Dirk_Chang@htc.com>");
MODULE_DESCRIPTION("LP5521 LED driver");

