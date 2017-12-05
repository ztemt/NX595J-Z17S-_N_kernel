/*
* This file is part of the PA224 sensor driver.
* PA224 is combined proximity, and VCSEL.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*Reversion
*
*
*

when         	who         		Remark : what, where, why          		version
-----------   	------------     	-----------------------------------   	------------------
2015/11/17	Simon Hsueh	& Allen Hsiao		For PA224 interrupt mode and oil alg   	v1.1.1
==========================================================================================
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <asm/atomic.h>
#include <linux/wakelock.h>
#include <linux/ctype.h>


#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#ifdef SENSORS_CLASS_DEV
#include <linux/sensors.h>
#endif

#include "optical_key_pa224.h"



#define PA224_DRV_NAME "optical_key_left"
#define DRIVER_VERSION "1.3.0"

#define LOG_TAG "OPTICAL_KEY_LEFT_PA224"
#define DEBUG_ON //DEBUG SWITCH

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__

#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#ifdef  DEBUG_ON
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif

/* Oil */
#define TXC_ABS(x) (x) >= 0 ? (x):(x)*(-1)
#define TXC_SIZE(arr) (sizeof(arr)/sizeof(arr[0]))
#define TXC_SUM(arr, sum) \
do { \
	int i = 0; \
	int size = TXC_SIZE(arr); \
	for (i=0; i<size; i++) \
		sum += arr[i]; \
} while (0)

#define TXC_ABS_SUM(arr, sum) \
do { \
	int i = 0; \
	int size = TXC_SIZE(arr); \
	for (i = 0; i < size; i++) \
		sum += TXC_ABS(arr[i]); \
} while (0)

#define IS_CLOSE(arr, close) \
do { \
	int i = 0; \
	int size = TXC_SIZE(arr); \
	close = 1; \
	while (i < size && close == 1) { \
		if (arr[i] >= 0) \
			i++; \
		else \
			close = 0; \
	} \
}while (0)

#define IS_AWAY(arr, away) \
do { \
	int i = 0; \
	int size = TXC_SIZE(arr); \
	away = 1; \
	while (i < size && away == 1) { \
		if ( arr[i] <= 0) \
			i++; \
		else \
			away = 0; \
	}  \
}while (0)

#define INPUT_DEVICE_NAME       "optical_key_left"
#define MISC_DEV_NAME       "optical_key_dev"
#define THRESHOLDS_FILE_PATH    "/persist/sensors/optical_key/left/threshold"
#define CURVE_DATA_FILE		"/persist/sensors/optical_key/left/curve_data"
#define USER_DATA_FILE	"/persist/sensors/optical_key/left/user_data"

#define saturation_delay    100
#define sequence_dealy      15
#define OIL_EFFECT          35
#define ps_ary_size         1

static int far_ps_min = PA24_PS_OFFSET_MAX;
static int saturation_flag = 0;
static const int ps_steady = ps_ary_size + 4;
static u8 ps_seq_far[ps_ary_size];
static u8 ps_seq_oil[ps_ary_size];
static u8 ps_seq_near[ps_ary_size];
static int oil_occurred = 0;

static dev_t const optical_key_left_dev_t = MKDEV(MISC_MAJOR, 103);

struct class *optical_key_class;
EXPORT_SYMBOL(optical_key_class);

#ifdef SENSORS_CLASS_DEV
static struct sensors_classdev sensor_optical_key_left_cdev = {
	.name = "optical_key_left_cdev",
	.vendor = "txc",
	.version = 1,
	.handle = SENSOR_HANDLE_OPTICAL_KEY_LEFT,
	.type = SENSOR_TYPE_OPTICAL_KEY_LEFT,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif
/* Global Variant */
static int thresholds_file_got = 0;
//get from factory collection present or from file written before
static int curve_data_got = 0;
//get from user collection present or from file written before
static int user_data_got = 0;
static struct i2c_client *pa224_i2c_client = NULL;
static struct pa224_data *pdev_data = NULL;
//additional space for USER collection and CLEAR factory collection
static u8 curve_data[NUM_FACTORY_COLLECT + 2][2];
static u8 user_data, nearest_position;

/*----------------------------------------------------------------------------*/
/*
* internally used functions
*/
/* I2C Read */
static int i2c_read_reg(struct i2c_client *client, u8 reg, u8 *buf)
{
	int ret = 0;
	int i = 0;

	struct pa224_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->i2c_lock);

	for (i = 0; i < I2C_RETRY_TIMES; i++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0) {
			SENSOR_LOG_ERROR("failed to read i2c addr=%x\n", data->client->addr);
			msleep(I2C_RETRY_DELAY);
		} else {
			*buf = (u8) ret;
			mutex_unlock(&data->i2c_lock);
			return 0;
		}
	}
	mutex_unlock(&data->i2c_lock);

	return ret;
}
/* I2C Write */
static int i2c_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret = 0;
	int i = 0;

	struct pa224_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->i2c_lock);
	for (i = 0; i < I2C_RETRY_TIMES; i++)
	{
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret < 0) {
			SENSOR_LOG_ERROR("failed to write i2c addr=%x\n", 0x1E);
			msleep(I2C_RETRY_DELAY);
		} else {
			mutex_unlock(&data->i2c_lock);
			return 0;
		}
	}
	mutex_unlock(&data->i2c_lock);

	return ret;
}
/* Calibration file handle*/
static int read_file(char *filename, u8 *param, int count)
{
	struct file  *fop;
	mm_segment_t old_fs;
	int vfs_retval = -EINVAL;

	fop = filp_open(filename, O_RDONLY, 0444);
	if (IS_ERR(fop)) 
	{
		SENSOR_LOG_INFO("filp_open error!! Path = %s\n", filename);
		return -ERR_FILE_OPS;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fop->f_pos = 0;
	vfs_retval = vfs_read(fop, param, count,  &fop->f_pos);
	if (vfs_retval < 0) {
		SENSOR_LOG_ERROR("read file <%s>failed\n",filename);
	}

	set_fs(old_fs);
	filp_close(fop, NULL);
	return 0;
}
static ssize_t write_file(char *filename, u8 *param, int count)
{
	struct file  *fop;
	mm_segment_t old_fs;
	int vfs_retval = -EINVAL;

	fop = filp_open(filename, O_CREAT | O_RDWR | O_TRUNC, 0666);
	if (IS_ERR(fop)) {
		SENSOR_LOG_INFO("create file error!! Path = %s\n",filename);
		return -ERR_FILE_OPS;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_retval = vfs_write(fop, (char *)param, count, &fop->f_pos);
	if (vfs_retval < 0) {
		SENSOR_LOG_ERROR("write file <%s>failed\n",filename);
	}

	set_fs(old_fs);
	filp_close(fop, NULL);
	return 0;
}
static int get_curve_data(char *filename)
{
	u8 data[(NUM_FACTORY_COLLECT+1)*2], i;
	int res = read_file(filename, data, sizeof(data));
	if(!res)
	{
		for(i=0; i <= NUM_FACTORY_COLLECT; i++)
		{
			curve_data[i][0] = data[i*2];
			curve_data[i][1] = data[i*2 + 1];
		}
		curve_data_got = 1;
		SENSOR_LOG_INFO("get curve data success!\n");
	}
	return res;
}
static int write_curve_data(char *filename)
{
	u8 data[(NUM_FACTORY_COLLECT+1)*2],i,j;
	int res;
	for(i = 0,j = 0; i <= NUM_FACTORY_COLLECT; i++, j = 0)
	{
		data[i*2 + j++] = curve_data[i][0];
		data[i*2 + j] = curve_data[i][1];
	}

	res = write_file(filename, data, sizeof(data));
	if(!res)
		SENSOR_LOG_INFO("write curve data success!\n");
	return res;
}
static void load_thresholds_file(char *filename,struct i2c_client *client)
{
	struct pa224_data *data = i2c_get_clientdata(client);
	u8 param[2] = {0, 0};
	int result;

	result = read_file(filename, param, sizeof(param));
	if (result < 0) {
		data->near_threshold = NEAR_THRESHOLD_DEFAULT;
		data->far_threshold = FAR_THRESHOLD_DEFAULT;
	} 
	else
	{
		data->near_threshold = param[0];
		data->far_threshold = param[1];
	}
	SENSOR_LOG_INFO("loaded thresholds file, near_threshold = %d, far_threshold = %d\n",
				data->near_threshold, data->far_threshold);
	thresholds_file_got = 1;
}
static void report_first_event(struct pa224_data *data)
{
	if (data->touch_status != data->last_touch_status) {
		input_report_rel(data->optical_key_input_dev, REL_RZ, data->touch_status);
		input_sync(data->optical_key_input_dev);
		data->last_touch_status = data->touch_status;
		SENSOR_LOG_INFO("data->touch_status = %d\n", data->touch_status);
	}
	return;
}
static int enable_optical_key(struct i2c_client *client, int enable)
{
	struct pa224_data *data = i2c_get_clientdata(client);
	struct pa224_platform_data *pdata=data->platform_data;
	int i = 0, res = 0;
	u8 value = 0;

	data->enable= enable;
	if (enable)
	{
		if (!data->vdd_always_on) {
			if (pdata && pdata->power_on) 
			{
				pdata->power_on(true);
				pa224_init_client(client);
			}
		}
		i2c_write_reg(client, REG_CFG1 ,(data->current_level << 4) | (PA24_PS_PRST << 2));
		i2c_write_reg(client, REG_CFG2, (PA24_PS_MODE << 6) | (PA24_PS_SET << 2));
		i2c_write_reg(client, REG_CFG0, (enable << 1));
		msleep(PA24_PS_ENABLE_DELAY);
		
		if(!data->debug)
		{
			saturation_flag = 0;
			oil_occurred = 0;
			data->last_touch_status = TOUCH_UNKNOWN_DISTANCE;
			for (i = 0;i < ps_ary_size; i++)
			{
				ps_seq_far[i] = 255;
				ps_seq_oil[i] = 255;
				ps_seq_near[i]= 255;
			}

			if(!thresholds_file_got)
				load_thresholds_file(THRESHOLDS_FILE_PATH, client);
			/* ensure current status of optical key*/
			res= i2c_read_reg(client, REG_PS_DATA, &value);
			if (res < 0) {
				SENSOR_LOG_ERROR("i2c_read function result = %d\n",res);
				return -1;
			}
			if (value <= data->far_threshold)
				data->touch_status = TOUCH_FAR_DISTANCE;
			if (value >= data->near_threshold)
				data->touch_status = TOUCH_NEAR_DISTANCE;
			report_first_event(data);

			i2c_write_reg(client, REG_PS_TH, data->near_threshold);
			i2c_write_reg(client, REG_PS_TL, data->far_threshold);
		}
		pa224_irq_enable(data, true, false);
	}
	else
	{
		i2c_write_reg(client, REG_CFG0, (enable << 1));
		pa224_irq_enable(data, false, true);
		if (!data->vdd_always_on) 
		{
			if (pdata && pdata->power_on) 
			{
				pdata->power_on(false);
			}
		}
	}
	return 0;
}
/*
* return value
* -1: need naked calibration
* -2: need 3cm gray-card cailibraion
*/
static int pa224_get_ps_value(struct i2c_client *client)
{
	u8 regdata = 0;

	i2c_read_reg(client, REG_PS_DATA, &regdata);

	return regdata;
}

/*
* Initialization function
*/
static int pa224_init_client(struct i2c_client *client)
{
	struct pa224_data *data = i2c_get_clientdata(client);

	/* Dealy time setting */
	data->optical_key_poll_delay = PA24_PS_POLL_DELAY;
	data->enable_delay = PA24_PS_ENABLE_DELAY;

	/* Initialize Sensor */
	i2c_write_reg(client,REG_CFG1,
		(data->current_level << 4)| (PA24_PS_PRST << 2) );

	i2c_write_reg(client,REG_CFG3,
		(PA24_INT_TYPE	<< 6)| (PA24_PS_PERIOD << 3) );

	i2c_write_reg(client, REG_PS_SET, 0x82);

	i2c_write_reg(client, REG_CFG4, 0x0C);

	i2c_write_reg(client,REG_CFG2,
		((PA24_PS_MODE	<< 6)|(PA24_INT_TYPE << 2)));
	SENSOR_LOG_INFO("pa224_init_client ok\n");

	return 0;
}
static int pa224_get_object(struct i2c_client *client)
{
	struct pa224_data *data = i2c_get_clientdata(client);
	u8 psdata = pa224_get_ps_value(client);

	SENSOR_LOG_INFO("PS:%d\n", psdata);
	SENSOR_LOG_INFO("touch_status:%d\n", data->touch_status);
	SENSOR_LOG_INFO("far_threshold:%d\n", data->far_threshold);
	SENSOR_LOG_INFO("near_threshold:%d\n", data->near_threshold);
	switch (data->touch_status) {
		case TOUCH_NEAR_DISTANCE:
			if (psdata < data->far_threshold) {
				data->touch_status = TOUCH_FAR_DISTANCE;
				SENSOR_LOG_INFO("Object Far\n");
			}
			break;
		case TOUCH_FAR_DISTANCE:
			if (psdata > data->near_threshold) {
				data->touch_status = TOUCH_NEAR_DISTANCE;
				SENSOR_LOG_INFO("Object Near\n");
			}
			break;
	}

	return data->touch_status;
}
/*----------------------------------------------------------------------------*/

/* For HAL to Enable PS */
static ssize_t pa224_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->enable);
}
static ssize_t pa224_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int err = -1;

	SENSOR_LOG_INFO("enable ps sensor (%ld)\n", val);

	if ((val != 0) && (val != 1)) {
		SENSOR_LOG_INFO("enable ps sensor=%ld\n", val);
		return count;
	}

	mutex_lock(&data->dev_lock);
	err = enable_optical_key(client, val);
	if (err < 0) {
		mutex_unlock(&data->dev_lock);
		SENSOR_LOG_ERROR("enable_optical_key error\n");
		return count;
	}
	mutex_unlock(&data->dev_lock);

	return count;
}
#ifdef SENSORS_CLASS_DEV
static int pa224_ps_set_enable(struct sensors_classdev *sensors_cdev,
		u8 enable)
{
	struct pa224_data *data = container_of(sensors_cdev,
			struct pa224_data, optical_key_cdev);
	SENSOR_LOG_INFO("enter\n");
	if ((enable != 0) && (enable != 1)) {
		SENSOR_LOG_ERROR("invalid value(%d)\n", enable);
		return -EINVAL;
	}

	return enable_optical_key(data->client, enable);
}
#endif
static ssize_t pa224_show_ps_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->optical_key_poll_delay);	// return in micro-second
}

static ssize_t pa224_store_ps_poll_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	if (val < 10)
		val = 10;

	data->optical_key_poll_delay = (u8)val;

	return count;
}
/* Device init */
static ssize_t optical_key_init_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}
static ssize_t pa224_chip_name_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	sprintf(buf, "%s\n",PA224_DRV_NAME);
	return strlen(buf);
}
static ssize_t threshold_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d %d \n", data->near_threshold,data->far_threshold);
}
static ssize_t threshold_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	char *positon, *next_para_positon;
	u8 val, thresholds[2], res;

	val = simple_strtol(buf, &next_para_positon, 0);
	thresholds[0] = val;
	data->near_threshold = val;

	while(isspace(*next_para_positon))
		next_para_positon++;
	positon = next_para_positon;

	val = simple_strtol(positon,&next_para_positon, 0);
	thresholds[1] = val;
	data->far_threshold = val;
	
	i2c_write_reg(client, REG_PS_TH, data->near_threshold);
	i2c_write_reg(client, REG_PS_TL, data->far_threshold);
	thresholds_file_got = 1;
	SENSOR_LOG_INFO("reset near threshold = %d, far threshold = %d\n",
		data->near_threshold, data->far_threshold);
	res = write_file(THRESHOLDS_FILE_PATH, thresholds, sizeof(thresholds));
	if(!res)
		SENSOR_LOG_INFO("write user data to file succeed!\n");
	return count;
}

static ssize_t current_level_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "current level is %d\n", data->current_level);
}
static ssize_t current_level_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	u8 level = 0;
	struct pa224_data *data = dev_get_drvdata(dev);

	level = simple_strtol(buf, NULL, 0);
	if(level < 4 || level >7)
	{
		SENSOR_LOG_ERROR("illegel current level\n");
		return size;
	}
	data->current_level = level;
	SENSOR_LOG_INFO("reset current level = %d\n", data->current_level);
	return size;
}
static ssize_t debug_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	if(data->debug)
		return sprintf(buf, "true\n");
	else
		return sprintf(buf, "false\n");
}
static ssize_t debug_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int val;
	val = simple_strtol(buf,NULL, 0);
	if(val)
	{
		/*reset near and far threshold for continuous interrupts*/
		i2c_write_reg(client, REG_PS_TH, CONTINUOUS_INTERRUPT_NEAR);
		i2c_write_reg(client, REG_PS_TL, CONTINUOUS_INTERRUPT_FAR);
		data->debug = true;
	}
	else
	{
		i2c_write_reg(client, REG_PS_TH, data->near_threshold);
		i2c_write_reg(client, REG_PS_TL, data->far_threshold);
		data->debug = false;
	}
	SENSOR_LOG_INFO("debug = %d\n", data->debug);
	return count;
}
static ssize_t collect_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	if(data->collect_result == 0 ||data->collect_result == -3)
		return sprintf(buf, "%d:%d:%d\n", data->collect_result,
		curve_data[data->last_collect_count ][0],  curve_data[data->last_collect_count ][1]);
	else
		return sprintf(buf, "%d\n", data->collect_result);
}
static ssize_t collect_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 value = 0;

	data->collect = true;
	data->collect_finished = false;

	value = simple_strtol(buf, NULL, 0);
	if(value == USER_COLLECT)
	{
		/*couple with last "value == USER_COLLECT" to avoid write sysfs node of debug while user
		collect. In case APK crashed without shutdown debug*/
		i2c_write_reg(client, REG_PS_TH, CONTINUOUS_INTERRUPT_NEAR);
		i2c_write_reg(client, REG_PS_TL, CONTINUOUS_INTERRUPT_FAR);
		data->debug = true;
		
		data->collect_count = NUM_FACTORY_COLLECT + 1;
		curve_data[data->collect_count][0] = USER_COLLECT;
	}
	else if(value == CLEAR_FACTORY_COLLECT)
	{
		/*CLEAR factory collect must be carried out firstly*/
		data->collect_count = NUM_FACTORY_COLLECT;
		curve_data[data->collect_count][0] = CLEAR_FACTORY_COLLECT;
	}
	else
	{
		/*factory collect counted from NUM_FACTORY_COLLECT-1*/
		if(data->collect_count > NUM_FACTORY_COLLECT - 1)
		{
			data->collect = false;
			SENSOR_LOG_ERROR("beyong factory collect range!\n");
			return count;
		}
		if(value == data->last_collect_distance &&
			data->collect_count <= NUM_FACTORY_COLLECT-1)
			data->collect_count++;

		curve_data[data->collect_count][0] = value;
	}
	
	wait_event_interruptible(data->wait, data->collect_finished);
	if(value == USER_COLLECT)
	{
		i2c_write_reg(client, REG_PS_TH, data->near_threshold);
		i2c_write_reg(client, REG_PS_TL, data->far_threshold);
		data->debug = false;
	}

	return count;
}
static ssize_t curve_data_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	u8 data[(NUM_FACTORY_COLLECT+2)*4+1];
	int i, j;
	for(i = 0,j = 0; i <= NUM_FACTORY_COLLECT+1; i++, j = 0)
	{
		data[i*4 + j++] = curve_data[i][0];
		data[i*4 + j++] = ':';
		data[i*4 + j++] = curve_data[i][1];
		data[i*4 + j] = ',';
		j = 0;
		SENSOR_LOG_INFO("%d%c%d%c\n", data[i*4] , data[i*4 + 1],
			data[i*4 + 2], data[i*4 + 3]);
	}
	return 0;
}
static int seek_nearest_curve_data(u8 userdata)
{
	u8 position_globe, position_middle;
	u8 position_left, position_right;
	u8 delta_left, delta_right;

	position_globe = NUM_FACTORY_COLLECT - 1;
	position_middle = position_globe / 2;

	if(userdata > curve_data[0][1])
	{
		SENSOR_LOG_ERROR("user data more than maximun curve data\n");
		return -1;
	}
	if(userdata < curve_data[NUM_FACTORY_COLLECT][1])
	{
		SENSOR_LOG_ERROR("user data less than minimun curve data\n");
		return -1;
	}
	//binary chop
	while(1)
	{
		if(TXC_ABS(position_globe - position_middle) == 1)
			break;
		if(userdata >= curve_data[position_middle][1])
		{
			position_globe = position_middle;
			position_middle /= 2;
		}
		else
			position_middle += (position_globe - position_middle)/2;
	}
	//after while position_middle == 0 would never occur for the first "if" filter
	if(userdata >= curve_data[position_middle][1])
	{
		position_left = position_middle - 1;
		position_right = position_middle;
	}
	else
	{
		position_left = position_middle;
		position_right = position_middle + 1;
	}
	delta_left = TXC_ABS(userdata - curve_data[position_left][1]);
	delta_right = TXC_ABS(userdata - curve_data[position_right][1]);
	if(delta_left <= delta_right)
		nearest_position = position_left;
	else
		nearest_position = position_right;

	SENSOR_LOG_INFO("user data = %d, nearest curve data = %d", userdata,
		curve_data[nearest_position][1]);
	return 0;
}
static void calculate_threshold_userlized(u8 distance_A, u8 distance_B,
		u8 *near_threshold, u8 *far_threshold)
{
	u8 near_threshold_position, far_threshold_position;
	near_threshold_position = nearest_position + distance_A;
	far_threshold_position = nearest_position + distance_B;
	*near_threshold = curve_data[near_threshold_position][1];
	*far_threshold = curve_data[far_threshold_position][1];
	return;
}
static ssize_t calculate_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->calculate_result);
}
static ssize_t calculate_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	u8 val, thresholds[2];
	int res;

	if(!curve_data_got)
	{
		res = get_curve_data(CURVE_DATA_FILE);
		if(res)
		{
			SENSOR_LOG_ERROR("load curve data failed!\n");
			data->calculate_result = -1;
			return count;
		}
	}
	val = simple_strtoul(buf, NULL, 0);
	if(val == 1)
	{
	}
	if(val == 2)
	{
		if(!user_data_got)
		{
			res = read_file(USER_DATA_FILE, &user_data, sizeof(user_data));
			if(res)
			{
				SENSOR_LOG_ERROR("load curve data failed!\n");
				data->calculate_result = -2;
				return count;
			}
			user_data_got = 1;
		}
		res = seek_nearest_curve_data(user_data);
		if(res)
		{
			SENSOR_LOG_ERROR("seek nearest curve data for userdata failed!\n");
			data->calculate_result = -3;
			return count;
		}
		calculate_threshold_userlized(THRESHOLD_DISTANCE_A, THRESHOLD_DISTANCE_B,
				&thresholds[0], &thresholds[1]);
		data->near_threshold = thresholds[0];
		data->far_threshold = thresholds[1];
		thresholds_file_got = 1;
		SENSOR_LOG_INFO("userlized near threshold = %d, far threshold = %d\n",
			thresholds[0], thresholds[1]);
		res = write_file(THRESHOLDS_FILE_PATH, thresholds, sizeof(thresholds));
		if(!res)
			SENSOR_LOG_INFO("write thresholds to file succeed!\n");
		data->calculate_result = 0;
	}

	return count;
}
static struct device_attribute attrs_prox_device[] = {
	__ATTR(init, 0220, NULL, optical_key_init_store),
	__ATTR(enable, 0664, pa224_show_enable_ps_sensor, pa224_store_enable_ps_sensor),
	__ATTR(delay, 0664, pa224_show_ps_poll_delay, pa224_store_ps_poll_delay),
	__ATTR(chip_name, 0444, pa224_chip_name_show, NULL),
	__ATTR(debug, 0644, debug_show, debug_store),
	__ATTR(collect, 0644, collect_show, collect_store),
	__ATTR(curve_data, 0644, curve_data_show, NULL),
	__ATTR(calculate, 0644, calculate_show, calculate_store),
	__ATTR(threshold, 0644, threshold_show, threshold_store),
	__ATTR(current_level, 0644, current_level_show, current_level_store),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attrs_prox_device); i++)
	{
		if (device_create_file(dev, attrs_prox_device + i))
			return -ENODEV;
	}
	return 0;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attrs_prox_device); i++)
		device_remove_file(dev, attrs_prox_device + i);
	return;
}
/*work que function*/
static void pa224_work_func_proximity(struct work_struct *work)
{
	struct pa224_data *data = container_of(work,
						struct pa224_data, optical_key_dwork.work);
	int Pval;

	Pval = pa224_get_object(data->client);
	SENSOR_LOG_INFO("PS value: %d\n", Pval);

	input_report_rel(data->optical_key_input_dev, REL_DIAL, Pval ? TOUCH_FAR_DISTANCE: TOUCH_NEAR_DISTANCE);
	input_sync(data->optical_key_input_dev);

	if(PS_POLLING && data->enable)
		schedule_delayed_work(&data->optical_key_dwork, msecs_to_jiffies(data->optical_key_poll_delay));

}

static void pa224_get_ps_slope_array(u8 *ps_seq, int *slope, u8 ps, int arysize)
{
	int i;

	for (i=0; i<arysize-1; i++)
	{
		ps_seq[arysize-1-i] = ps_seq[arysize-1-i-1];
		if (ps_seq[arysize-1-i] == 0)
			ps_seq[arysize-1-i] = ps;
	}
	ps_seq[0] = (int)ps;

	for (i=0; i<arysize-1; i++)
	{
		slope[i] = (int)(ps_seq[i] - ps_seq[i+1]);
	}
	return;
}
static int debounce_factory_collection(u8 psdata, struct i2c_client *client)
{
	struct pa224_data *data = i2c_get_clientdata(client);
	int diff;
	data->debounce_count++;
	//debounced psdata gain fail for overtime
	if(data->debounce_count >= DEBOUNCE_COUNT_LIMIT)
		return -ERR_DEBOUNCE_OVERTIME;

	diff = psdata - data->last_psdata;
	//beyong debounce range , adjust "anchor" and redebounce
	if(diff > DEBOUNCE_RANGE || diff < -DEBOUNCE_RANGE)
	{
		data->lasting_count = 0;
		data->last_psdata = psdata;
	}
	else
		data->lasting_count++;

	//debounced psdata gain success
	if(data->lasting_count >= LASTING_COUNT_LIMIT)
		return SUCCESS;
	//need more psdata debounced
	return -ERR_MORE_DEBOUNCE;
}
static void handle_collection_debounce_result(int res, struct i2c_client *client)
{
	struct pa224_data *data = i2c_get_clientdata(client);
	//need more psdata debounce	
	if(res == -ERR_MORE_DEBOUNCE)
		return;

	data->last_collect_count = data->collect_count;
	
	/*debounced psdata gain failed for overtime*/
	if(res == -ERR_DEBOUNCE_OVERTIME)
	{
		data->collect_result = -ERR_DEBOUNCE_OVERTIME;
		SENSOR_LOG_ERROR("debounced psdata gain failed for overtime\n");
	}
	/*debounced psdata gain success*/
	if(res == SUCCESS)
	{
		SENSOR_LOG_INFO("debounce success, count = %d\n", data->collect_count);
		/*debounded psdata doesn't match curve pattem*/
		if(data->collect_count < NUM_FACTORY_COLLECT-1)
		{
			if(data->last_psdata < curve_data[data->collect_count + 1][1])
			{
				data->collect_result = -ERR_UNMATCH_CERVE_PATTERN;
				SENSOR_LOG_ERROR("debounced psdata unmatch expected curve pattern\n");
				return;
			}
		}

		data->collect_result = SUCCESS;
		curve_data[data->collect_count][1] = data->last_psdata;
		data->last_collect_distance = curve_data[data->collect_count][0];
		SENSOR_LOG_INFO("collect success, distance = %d, count = %d, data = %d\n",
				curve_data[data->collect_count][0], data->collect_count,
				curve_data[data->collect_count][1]);

		/*curve_data[NUM_FACTORY_COLLECT+1] is used for USER collect*/
		if(data->collect_count == NUM_FACTORY_COLLECT + 1)
		{
			if(data->last_psdata < curve_data[NUM_FACTORY_COLLECT][1]
				+VALID_USERDATA_OFFSET)
			{
				data->collect_result = -ERR_USER_NOT_APPROACH;
				SENSOR_LOG_ERROR("user didn't approach, collect fail!\n");
			}
			else
			{
				user_data = data->last_psdata;
				user_data_got = 1;
				res = write_file(USER_DATA_FILE, &user_data, sizeof(data));
				if(!res)
					SENSOR_LOG_ERROR("write user data to file succeed!\n");
			}
		}
		else
			data->collect_count--;
		/*normal factory collect reach the last one*/
		if(data->collect_count == -1)
		{
			data->collect_count = NUM_FACTORY_COLLECT-1;
			data->last_collect_distance = NUM_FACTORY_COLLECT-1;
			curve_data_got = 1;
			res = write_curve_data(CURVE_DATA_FILE);
			if(res)
				SENSOR_LOG_ERROR("write curve data to file failed!\n");
		}
	}

	data->last_psdata = 0;
	data->debounce_count = 0;
	data->lasting_count = 0;
	data->collect = false;
	data->collect_finished = true;

	SENSOR_LOG_ERROR("after handle count = %d\n",data->collect_count);

	wake_up_interruptible(&data->wait);
	return;
}

static int interrupt_handle(struct i2c_client *client)
{
	struct pa224_data *data = i2c_get_clientdata(client);
	int res;
	u8 psdata=0;
	u8 cfgdata=0;
	static int far_loop = 0;

	int slope[ps_ary_size-1];
	int sum = 0, abs_sum = 0, ps_sum = 0;

	res = i2c_read_reg(client, REG_PS_DATA, &psdata);
	if (res < 0) {
		SENSOR_LOG_ERROR("i2c_read function err res = %d\n",res);
		return -1;
	}

	if (data->debug)
	{
		input_report_rel(data->optical_key_input_dev, REL_MISC, psdata > 0 ? psdata : 1);
		input_sync(data->optical_key_input_dev);

		if(data->collect)
		{

			res = debounce_factory_collection(psdata, client);
			handle_collection_debounce_result(res, client);
			goto interrupt_handle_exit;
		}

		goto interrupt_handle_exit;
	}
	//SUNLIGHT
	if (psdata == 0) {

		if (data->touch_status == TOUCH_NEAR_DISTANCE) {
			i2c_write_reg(client,REG_CFG1,
						(data->current_level << 4)| (PA24_PS_PRST << 2) );
		}
		saturation_flag = 1;

		if (oil_occurred && far_ps_min < PA24_PS_OFFSET_MAX) {
			//data->near_threshold = far_ps_min + OIL_EFFECT + data->near_diff_cnt;
			//data->far_threshold = far_ps_min + OIL_EFFECT;
		} else if (!oil_occurred && far_ps_min < PA24_PS_OFFSET_MAX) {
			//data->near_threshold = far_ps_min + data->near_diff_cnt;
			//data->far_threshold = far_ps_min + data->near_diff_cnt - data->far_diff_cnt;//data->near_diff_cnt - PA24_NEAR_FAR_CNT;
		} else if (far_ps_min == PA24_PS_OFFSET_MAX) {
			//data->near_threshold = PA24_PS_OFFSET_MAX;
			//data->far_threshold = PA24_PS_OFFSET_MAX - 1;
		}
		//msleep(saturation_delay);
		SENSOR_LOG_INFO("Sun light!!, ht=%d, lt=%d, far_ps_min=%d\n", data->near_threshold, data->far_threshold, far_ps_min);
		data->touch_status = TOUCH_FAR_DISTANCE;
		goto interrupt_handle_exit;
	}
	//FARTHER AWAY
	if (psdata < data->far_threshold && data->touch_status == TOUCH_FAR_DISTANCE) {

		pa224_get_ps_slope_array(ps_seq_far, slope, psdata, ps_ary_size);
		TXC_SUM(ps_seq_far, ps_sum);
		TXC_SUM(slope, sum);
		TXC_ABS_SUM(slope, abs_sum);
		if (data->debug) {
		    //SENSOR_LOG_INFO("slope : %d %d %d\n", slope[2], slope[1], slope[0]);
		    //SENSOR_LOG_INFO("value : %d %d %d %d\n", ps_seq_far[3], ps_seq_far[2], ps_seq_far[1], ps_seq_far[0]);
			SENSOR_LOG_INFO("saturation_flag=%d\n", saturation_flag);
		}
		//If saturation happened, the average ps value must be greater than (far_ps_min-10) and also  steady
		if ( (saturation_flag && ps_sum/ps_ary_size >= ( far_ps_min > 10 ? (far_ps_min - 10) : far_ps_min ))
			|| !saturation_flag || (saturation_flag && far_ps_min == PA24_PS_OFFSET_MAX) )
		{
			//STEADY
			if (abs_sum < ps_steady) {
				if (saturation_flag)
					saturation_flag = 0;

				data->touch_status = TOUCH_FAR_DISTANCE;
				oil_occurred = 0;
				far_ps_min = ps_sum / ps_ary_size;
				//data->near_threshold = far_ps_min + data->near_diff_cnt;
				//data->far_threshold = far_ps_min > 15 ? (far_ps_min - 5) : 15;
				i2c_write_reg(client, REG_CFG3, (PA24_INT_TYPE << 6) | (PA24_PS_PERIOD << 3));
				if(data->debug)
					SENSOR_LOG_INFO("FAR, far_ps_min %3d high low : %3d %3d\n", far_ps_min, data->near_threshold, data->far_threshold);

				if (data->touch_status != data->last_touch_status)
				{
					input_report_rel(data->optical_key_input_dev, REL_HWHEEL, psdata > 0 ? psdata : 1);
					input_report_rel(data->optical_key_input_dev, REL_RZ, data->touch_status);
					input_sync(data->optical_key_input_dev);
					data->last_touch_status = data->touch_status;
					SENSOR_LOG_INFO("data->psdata = %d, data->touch_status = %d\n", 
						psdata, data->touch_status);
				}
				//pa224_report_event(data);
			}
		}
		//msleep(sequence_dealy);
	}
	//NEAR
	else if (psdata > data->near_threshold)
	{
		int i = 0;
		for (i = 0; i < ps_ary_size; i++) {
			res = i2c_read_reg(client, REG_PS_DATA, ps_seq_near+i);
			//if (i > 0)
				//slope[i-1] = (int)(ps_seq_near[i] - ps_seq_near[i-1]);
			//mdelay(5);
		}

		//pa224_get_ps_slope_array(ps_seq_near, slope, psdata, ps_ary_size);
		if (data->debug) {
		   // SENSOR_LOG_ERROR("slope : %d %d %d\n", slope[2], slope[1], slope[0]);
		    //SENSOR_LOG_ERROR("value : %d %d %d %d\n", ps_seq_near[3], ps_seq_near[2], ps_seq_near[1], ps_seq_near[0]);
		}
		TXC_ABS_SUM(slope, abs_sum);
		oil_occurred = 0;
		if (abs_sum < ps_steady) {
			data->touch_status = TOUCH_NEAR_DISTANCE;
			i2c_write_reg(client,REG_CFG1,
					(data->current_level << 4)| (1 << 2) );

			if (psdata >= 254) {
				far_loop = 0;
				oil_occurred = 1;
				//data->far_threshold = far_ps_min + OIL_EFFECT;
				//data->near_threshold = 0xFF;
			} else {
				//data->far_threshold = far_ps_min + (data->near_diff_cnt - data->far_diff_cnt);//(PA24_NEAR_FAR_CNT - 3));
				//data->near_threshold = 254;//(far_ps_min + OIL_EFFECT);
			}
			if (data->debug) {
				SENSOR_LOG_INFO("NER, far_ps_min:%3d psdata:%3d high low:%3d %3d\n", far_ps_min, psdata, data->near_threshold, data->far_threshold);
			}
			if (data->touch_status != data->last_touch_status)
			{
				input_report_rel(data->optical_key_input_dev, REL_HWHEEL, psdata > 0 ? psdata : 1);
				input_report_rel(data->optical_key_input_dev, REL_RZ, data->touch_status);
				input_sync(data->optical_key_input_dev);
				data->last_touch_status = data->touch_status;
				SENSOR_LOG_INFO("data->psdata = %d, data->touch_status = %d\n", psdata, data->touch_status);
			}
			//pa224_report_event(data);
		} else if (abs_sum > 20) {
			/*Flicker light*/
			i2c_write_reg(client, REG_CFG3, (PA24_INT_TYPE << 6)| (0 << 3));
			SENSOR_LOG_ERROR("Flicker light!!!!");
		}
	}
	//FAR AWAY
	if (psdata < data->far_threshold && data->touch_status == TOUCH_NEAR_DISTANCE)
	{
		if (oil_occurred) {
			far_loop++;
			pa224_get_ps_slope_array(ps_seq_oil, slope, psdata, ps_ary_size);
			TXC_SUM(ps_seq_oil, ps_sum);
			TXC_SUM(slope, sum);
			TXC_ABS_SUM(slope, abs_sum);
			if (data->debug) {
			  //  SENSOR_LOG_INFO("slope : %d %d %d\n", slope[2], slope[1], slope[0]);
			 //   SENSOR_LOG_INFO("value : %d %d %d %d\n", ps_seq_oil[3], ps_seq_oil[2], ps_seq_oil[1], ps_seq_oil[0]);
			}//STEADY
			if (abs_sum < ps_steady || far_loop > 10) {
				i2c_write_reg(client,REG_CFG1,
					(data->current_level << 4)| (PA24_PS_PRST << 2) );
				data->touch_status = TOUCH_FAR_DISTANCE;
				oil_occurred = 0;
				if (far_loop <= 10) {
					far_ps_min = ps_sum / ps_ary_size;
					//data->near_threshold = far_ps_min + data->near_diff_cnt;
					//data->far_threshold = far_ps_min > 5 ? (far_ps_min - 5) : 5;
				} else {
					far_ps_min = far_ps_min + 15;
					//data->near_threshold = far_ps_min + data->near_diff_cnt;
					//data->far_threshold = far_ps_min + (data->near_diff_cnt - data->far_diff_cnt);//PA24_NEAR_FAR_CNT);
				}
				i2c_write_reg(client, REG_CFG3, (PA24_INT_TYPE << 6)| (PA24_PS_PERIOD << 3));
				if (data->debug) {
					SENSOR_LOG_INFO("OIL to FAR, far_ps_min %3d high low : %3d %3d\n", far_ps_min, data->near_threshold, data->far_threshold);
				}
				if (data->touch_status != data->last_touch_status)
				{
					input_report_rel(data->optical_key_input_dev, REL_HWHEEL, psdata > 0 ? psdata : 1);
					input_report_rel(data->optical_key_input_dev, REL_RZ, data->touch_status);
					input_sync(data->optical_key_input_dev);
					data->last_touch_status = data->touch_status;
					SENSOR_LOG_INFO("data->psdata = %d, data->touch_status = %d\n", 
						psdata, data->touch_status);
				}
				//pa224_report_event(data);
			}
			//msleep(sequence_dealy);
		} else {
			i2c_write_reg(client,REG_CFG1,
					(data->current_level << 4)| (PA24_PS_PRST << 2));
			data->touch_status = TOUCH_FAR_DISTANCE;
			//data->near_threshold = far_ps_min + data->near_diff_cnt;
			//data->far_threshold = far_ps_min + data->near_diff_cnt - data->far_diff_cnt;//PA24_NEAR_FAR_CNT;
			if (data->debug) {
			    SENSOR_LOG_ERROR("FAR, far_ps_min %3d high low : %3d %3d\n", far_ps_min, data->near_threshold, data->far_threshold);
			}

			if (data->touch_status != data->last_touch_status)
			{
				input_report_rel(data->optical_key_input_dev, REL_HWHEEL, psdata > 0 ? psdata : 1);
				input_report_rel(data->optical_key_input_dev, REL_RZ, data->touch_status);
				input_sync(data->optical_key_input_dev);
				data->last_touch_status = data->touch_status;
				SENSOR_LOG_INFO("data->psdata = %d, data->touch_status = %d\n", psdata, data->touch_status);
			}
			//pa224_report_event(data);
		}
	}

interrupt_handle_exit:

	//i2c_write_reg(client, REG_PS_TL, data->far_threshold);
	//i2c_write_reg(client, REG_PS_TH, data->near_threshold);

	/* Clear PS INT FLAG */
	res = i2c_read_reg(client, REG_CFG2, &cfgdata);
	if (res < 0) {
		SENSOR_LOG_ERROR("i2c_read function err res = %d\n",res);
		return -ERR_DEV_OPS;
	}
	cfgdata = cfgdata & 0xFD;

	res = i2c_write_reg(client,REG_CFG2, cfgdata);
	if (res < 0) {
		SENSOR_LOG_ERROR("i2c_send function err res = %d\n",res);
		return -ERR_DEV_OPS;
	}

	res = i2c_read_reg(client, REG_CFG2, &cfgdata);
	if (cfgdata & 2) {
		cfgdata = cfgdata & 0xFD;
		res = i2c_write_reg(client,REG_CFG2, cfgdata);
		if (res < 0) {
			SENSOR_LOG_ERROR("i2c_send function err res = %d\n",res);
			return -1;
		}
	}
	return 0;
}

static void pa224_irq_enable(struct pa224_data *data, bool enable, bool flag_sync)
{
	if (enable == data->irq_enabled) {
		SENSOR_LOG_DEBUG("doubule %s irq %d\n",enable? "enable" : "disable",
			data->irq);
		return;
	} else {
			data->irq_enabled = enable;
	}

	if (enable) {
		enable_irq(data->irq);
	} else {
		if (flag_sync) {
			disable_irq(data->irq);
		} else {
			disable_irq_nosync(data->irq);
		}
	}
}

static void pa224_work_func_irq(struct work_struct *work)
{
	struct pa224_data *data;
	struct i2c_client *client;
	data = container_of((struct work_struct *)work, struct pa224_data, irq_dwork);
	client = data->client;
	/* Add Oil Alg */
	wake_lock_timeout(&data->pa224_wake_lock, msecs_to_jiffies(1000));
	interrupt_handle(client);
}

static irqreturn_t pa224_irq(int irq, void *handle)
{
	struct pa224_data *data = handle;
	struct i2c_client *client = data->client;

	wake_lock_timeout(&data->pa224_wake_lock, msecs_to_jiffies(1000));
	interrupt_handle(client);

	return IRQ_HANDLED;
}
/*Suspend/Resume*/
static int pa224_suspend(struct device *dev)
{
	SENSOR_LOG_ERROR("suspend\n");
	return 0;
}

static int pa224_resume(struct device *dev)
{
	SENSOR_LOG_ERROR("resume\n");
	return 0;
}

static int pa224_pinctrl_init(struct pa224_data *data, struct device *dev)
{
	int rc;

	data->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		SENSOR_LOG_ERROR("data->pinctrl is NULL\n");
		return PTR_ERR(data->pinctrl);
	}

	data->pin_default = pinctrl_lookup_state(data->pinctrl, "Optical_key_left_default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		SENSOR_LOG_ERROR("lookup default state failed\n");
		return PTR_ERR(data->pin_default);
	}

	data->pin_sleep = pinctrl_lookup_state(data->pinctrl, "Optical_key_left_sleep");
	if (IS_ERR_OR_NULL(data->pin_sleep)) {
		SENSOR_LOG_ERROR("lookup sleep state failed\n");
		return PTR_ERR(data->pin_sleep);
	}

	if (!IS_ERR_OR_NULL(data->pinctrl)) {
		rc = pinctrl_select_state(data->pinctrl, data->pin_default);
		if (rc) {
			SENSOR_LOG_ERROR("select default state failed\n");
			return rc;
		}
	}
	SENSOR_LOG_INFO("pinctrl init success\n");
	return 0;
}

static int sensor_regulator_configure(struct pa224_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				PA224_VDD_MAX_UV);

		regulator_put(data->vdd);
		regulator_disable(data->vdd);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			SENSOR_LOG_ERROR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if (!IS_ERR_OR_NULL(data->vdd)) {
			if (regulator_count_voltages(data->vdd) > 0) {
				rc = regulator_set_voltage(data->vdd,
					PA224_VDD_MIN_UV, PA224_VDD_MAX_UV);
				if (rc) {
					SENSOR_LOG_ERROR("Regulator set failed vdd rc=%d\n",
						rc);
					goto reg_vdd_put;
				}
			}

			rc = regulator_enable(data->vdd);
			if (rc) {
				SENSOR_LOG_ERROR(
					"Regulator enable vdd failed. rc=%d\n", rc);
				goto reg_vdd_put;
			}
		}
	}

	return 0;

reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}


static int sensor_regulator_power_on(struct pa224_data *data, bool on)
{
	int rc = 0;

	if (!on) {
		if (!IS_ERR_OR_NULL(data->vdd)) {
			rc = regulator_disable(data->vdd);
			if (rc) {
				SENSOR_LOG_ERROR(
					"Regulator vdd disable failed rc=%d\n", rc);
				return rc;
			}
		}
	} else {
		if (!IS_ERR_OR_NULL(data->vdd)) {
			rc = regulator_enable(data->vdd);
			if (rc) {
				SENSOR_LOG_ERROR(
					"Regulator vdd enable failed rc=%d\n", rc);
				return rc;
			}
		}
	}
	SENSOR_LOG_INFO("power %s\n", on ? "on":"off");
	mdelay(5);

	return rc;
}

static int sensor_platform_hw_power_on(bool on)
{
	if (pdev_data == NULL)
		return -ENODEV;

	sensor_regulator_power_on(pdev_data, on);

	return 0;
}

static int sensor_platform_hw_init(void)
{
	struct i2c_client *client;
	struct pa224_data *data;
	int error;

	if (pdev_data == NULL)
		return -ENODEV;

	data = pdev_data;
	client = data->client;

	error = sensor_regulator_configure(data, true);
	if (error) {
		SENSOR_LOG_ERROR("unable to configure regulator\n");
		return error;
	}

	if (gpio_is_valid(data->platform_data->irq_gpio)) {
		/* configure pa224 irq gpio */
		SENSOR_LOG_INFO("gpio value is %d \n", gpio_get_value(data->platform_data->irq_gpio));
		error = gpio_request_one(data->platform_data->irq_gpio,
				GPIOF_DIR_IN,
				"pa224_irq_gpio");
		if (error) {
			SENSOR_LOG_ERROR("unable to request gpio %d\n",
				data->platform_data->irq_gpio);
		}

		data->irq = client->irq =
			gpio_to_irq(data->platform_data->irq_gpio);
	} else {
		SENSOR_LOG_ERROR("irq gpio not provided\n");
	}
	return 0;
}

static void sensor_platform_hw_exit(void)
{
	struct pa224_data *data = pdev_data;

	if (data == NULL)
		return;

	//sensor_regulator_configure(data, false);

	if (gpio_is_valid(data->platform_data->irq_gpio))
		gpio_free(data->platform_data->irq_gpio);
}

static int sensor_parse_dt(struct device *dev,
		struct pa224_platform_data *pdata,
		struct pa224_data *data)
{
	struct device_node *np = dev->of_node;

	unsigned int tmp = 0;
	int rc = 0;

	/* set functions of platform data */
	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;
	pdata->power_on = sensor_platform_hw_power_on;

	/* irq gpio */
	rc = of_get_named_gpio(dev->of_node,"txc,irq-gpio", 0);
	if (rc < 0) {
		SENSOR_LOG_ERROR("Unable to read irq gpio\n");
		return rc;
	}
	pdata->irq_gpio = rc;
	SENSOR_LOG_INFO("irq gpio is %d\n", pdata->irq_gpio);

	/* vdd-always-on flag */
	rc = of_property_read_u32(np, "txc,vdd-always-on", &tmp);
	data->vdd_always_on = tmp;
	SENSOR_LOG_INFO("vdd always-on flag is %d\n", data->vdd_always_on);

	/* ps tuning data*/
	rc = of_property_read_u32(np, "txc,ps_threshold_low", &tmp);
	data->far_threshold= (!rc ? tmp : 30);
	SENSOR_LOG_INFO("ps_threshold_low is %d\n", data->far_threshold);

	rc = of_property_read_u32(np, "txc,ps_threshold_high", &tmp);
	data->near_threshold = (!rc ? tmp : 120);
	SENSOR_LOG_INFO("ps_threshold_low is %d\n", data->near_threshold);

	return 0;
}
static int pa224_read_device_id(struct pa224_data *data)
{
	int retry_times = 3;
	u8 device_id;
	int res = -1;
	struct i2c_client *client = data->client;
	while (retry_times--) {
		res = i2c_read_reg(client, REG_ID, &device_id);
		if (res >= 0) {
			SENSOR_LOG_INFO("device_id = %d\n", device_id);
			if (device_id == 0x11) {
				SENSOR_LOG_INFO("read device id success\n");
				return 0;
			}
		}
	}
	SENSOR_LOG_ERROR("read device id failed\n");
	return res;
}

static const struct i2c_device_id pa224_id[] = {
	{ PA224_DRV_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, pa224_id);

static struct of_device_id pa224_match_table[] = {
	{ .compatible = "Optical_key_left_pa224",},
	{ },
};

static const struct dev_pm_ops pa224_pm_ops = {
	.suspend	= pa224_suspend,
	.resume 	= pa224_resume,
};

static struct i2c_driver optical_key_left_driver = {
	.driver = {
		.name	= PA224_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = pa224_match_table,
		.pm = &pa224_pm_ops,
	},
	.probe	= pa224_probe,
	.remove	= pa224_remove,
	.id_table = pa224_id,
};
/*
* I2C init/probing/exit functions
*/

static int pa224_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct pa224_data *data = NULL;
	struct pa224_platform_data *pdata=client->dev.platform_data;
	int err = 0;

	SENSOR_LOG_DEBUG("probe start\n");
	data = kzalloc(sizeof(struct pa224_data), GFP_KERNEL);
	if (!data) {
		SENSOR_LOG_ERROR("kzalloc pa224_data failed\n");
		err = -ENOMEM;
		goto exit;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		SENSOR_LOG_ERROR("i2c_check_functionality error");
		goto exit;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct pa224_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			SENSOR_LOG_ERROR("Failed to allocate memory\n");
			err = -ENOMEM;
			goto exit_platform_failed;
		}

		client->dev.platform_data = pdata;

		/* Parse device tree */
		err = sensor_parse_dt(&client->dev, pdata, data);
		if (err) {
			SENSOR_LOG_ERROR("sensor_parse_dt() err\n");
			goto exit_platform_failed;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			SENSOR_LOG_ERROR("No platform data\n");
			err = -ENODEV;
			goto exit_platform_failed;
		}
	}
	/*init pin state*/
	err = pa224_pinctrl_init(data, &client->dev);
	if (err) {
		SENSOR_LOG_ERROR("init pinctrl state failed\n");
		data->pinctrl = NULL;
		data->pin_default = NULL;
		data->pin_sleep = NULL;
	}

	mutex_init(&data->i2c_lock);
	mutex_init(&data->dev_lock);
	pdev_data = data;

	init_waitqueue_head(&data->wait);
	data->debug = false;
	data->collect = false;
	data->collect_finished = false;

	data->collect_count = NUM_FACTORY_COLLECT-1;
	data->last_collect_count = NUM_FACTORY_COLLECT-1;
	data->lasting_count = 0;
	data->last_psdata = 0;
	data->last_collect_distance = 0;

	data->client = client;
	i2c_set_clientdata(client, data);

	data->platform_data  = pdata;
	pa224_i2c_client = client;

	/* h/w initialization */
	if (pdata->init)
		err = pdata->init();

	/*read device id*/
	err = pa224_read_device_id(data);
	if (err < 0) {
		SENSOR_LOG_ERROR("pa224_read_device_id failed\n");
		goto exit_no_dev_error;
	}
	optical_key_class = class_create(THIS_MODULE, "optical_key");
	data->optical_key_device = device_create(optical_key_class, NULL, optical_key_left_dev_t, 
		&optical_key_left_driver ,"left");
	if (IS_ERR(data->optical_key_device)) {
		err = PTR_ERR(data->optical_key_device);
		SENSOR_LOG_ERROR("create optical key device failed\n");
		goto create_optcial_key_device_failed;
	}

	dev_set_drvdata(data->optical_key_device, data);

	err = create_sysfs_interfaces(data->optical_key_device);
	if (err < 0) {
		SENSOR_LOG_ERROR("create sysfs interfaces failed\n");
		goto create_sysfs_interface_error;
	}
#ifdef SENSORS_CLASS_DEV
	/* Register to sensors class */
	data->optical_key_cdev = sensor_optical_key_left_cdev;
	data->optical_key_cdev.sensors_enable = pa224_ps_set_enable;
	data->optical_key_cdev.sensors_poll_delay = NULL,
	err = sensors_classdev_register(&client->dev, &data->optical_key_cdev);
	if (err) {
		SENSOR_LOG_ERROR("Unable to register to sensors class: %d\n",err);
		goto create_sysfs_interface_error;
	}
#endif
	/* allocate proximity input_device */
	data->optical_key_input_dev = input_allocate_device();
	if (IS_ERR_OR_NULL(data->optical_key_input_dev)) {
		err = -ENOMEM;
		SENSOR_LOG_INFO("could not allocate input device\n");
		goto exit_unregister_sensorclass;
	}

	input_set_drvdata(data->optical_key_input_dev, data);
	data->optical_key_input_dev->name = INPUT_DEVICE_NAME;
	data->optical_key_input_dev->id.bustype = BUS_I2C;
	set_bit(EV_REL, data->optical_key_input_dev->evbit);
	set_bit(REL_RZ,  data->optical_key_input_dev->relbit);
	set_bit(REL_MISC,  data->optical_key_input_dev->relbit);
	set_bit(REL_HWHEEL,  data->optical_key_input_dev->relbit);

	SENSOR_LOG_INFO("registering proximity input device\n");
	err = input_register_device(data->optical_key_input_dev);
	if (err < 0) {
		SENSOR_LOG_INFO("could not register input device\n");
		err = -ENOMEM;
		goto exit_unregister_sensorclass;
	}
	wake_lock_init(&data->pa224_wake_lock, WAKE_LOCK_SUSPEND ,"pa224_wake_lock");
	/*Device Initialize*/
	err = pa224_init_client(client);
	if (err < 0) {
		SENSOR_LOG_ERROR("init pa224 failed when probe\n");
		goto input_dev_exit;
	}

	if (PS_POLLING)
		INIT_DELAYED_WORK(&data->optical_key_dwork, pa224_work_func_proximity);
	else
		INIT_WORK(&data->irq_dwork, pa224_work_func_irq);

	data->irq_work_queue = create_singlethread_workqueue("pa224_work_queue");
	if (IS_ERR_OR_NULL(data->irq_work_queue)){
		err = -ENOMEM;
		SENSOR_LOG_ERROR( "cannot create work taos_work_queue, err = %d",err);
		goto input_dev_exit;
	}

	irq_set_irq_wake(client->irq, 1);
	/*Interrupt Regist*/
	if (!PS_POLLING ) {
		err = request_threaded_irq(data->irq, NULL, pa224_irq,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				PA224_DRV_NAME, data);

		if (err) {
			SENSOR_LOG_INFO("Could not get IRQ\n");
		}
	}

	pa224_irq_enable(data, false, true);
	if (!data->vdd_always_on) {
		if (pdata && pdata->power_on) {
			pdata->power_on(false);
		}
	}
	data->current_level = LED_CURR_DEFAULT; /*4:15mA | 5:12mA | 6:10mA | 7:7mA*/
	SENSOR_LOG_INFO("probe ok.\n");
	return 0;
input_dev_exit:
	if (!IS_ERR_OR_NULL(data->optical_key_input_dev)) {
		input_unregister_device(data->optical_key_input_dev);
		input_free_device(data->optical_key_input_dev);
	}
exit_unregister_sensorclass:
	wake_lock_destroy(&data->pa224_wake_lock);
#ifdef SENSORS_CLASS_DEV
	sensors_classdev_unregister(&data->optical_key_cdev);
#endif
create_sysfs_interface_error:
	remove_sysfs_interfaces(data->optical_key_device);
create_optcial_key_device_failed:
	data->optical_key_device = NULL;
	device_destroy(optical_key_class, optical_key_left_dev_t);
	class_destroy(optical_key_class);
exit_no_dev_error:
	if (pdata->power_on)
		pdata->power_on(false);
	if(pdata->exit)
		pdata->exit();
exit_platform_failed:
	kfree(data);
exit:
	return err;
}

static int pa224_remove(struct i2c_client *client)
{
	struct pa224_data *data = i2c_get_clientdata(client);
	struct pa224_platform_data *pdata = data->platform_data;
	struct device *dev = data->optical_key_device;

	if (data->enable)
		enable_optical_key(client, 0);

	input_unregister_device(data->optical_key_input_dev);
	input_free_device(data->optical_key_input_dev);
	remove_sysfs_interfaces(dev);
	wake_lock_destroy(&data->pa224_wake_lock);

	if (!PS_POLLING)
		free_irq(data->irq, client);

	if (pdata->power_on)
		pdata->power_on(false);

	if (pdata->exit)
		pdata->exit();

	mutex_destroy(&data->i2c_lock);
	mutex_destroy(&data->dev_lock);

	kfree(data);

	return 0;
}


static int __init pa224_init(void)
{
	return i2c_add_driver(&optical_key_left_driver);
}

static void __exit pa224_exit(void)
{
	i2c_del_driver(&optical_key_left_driver);
}

MODULE_AUTHOR("Sensor Team, TXC");
MODULE_DESCRIPTION("Optical key left sensor driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(pa224_init);
module_exit(pa224_exit);


