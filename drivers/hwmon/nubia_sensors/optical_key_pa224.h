/*
*  pa224.h - Linux kernel modules for proximity sensor
*
*  Copyright (c) 2013, All rights reserved.
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License version 2 and
*  only version 2 as published by the Free Software Foundation.

*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#ifndef __PA224_H__
#define __PA224_H__

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
/*Driver Parameters  */

/*pa224 als/ps Default*/
#define NEAR_THRESHOLD_DEFAULT	200
#define FAR_THRESHOLD_DEFAULT		190
#define CONTINUOUS_INTERRUPT_NEAR	0x80
#define CONTINUOUS_INTERRUPT_FAR	0x7f
#define NUM_FACTORY_COLLECT	9
#define USER_COLLECT	254
#define CLEAR_FACTORY_COLLECT	255
#define DEBOUNCE_COUNT_LIMIT	300
#define LASTING_COUNT_LIMIT		100
#define DEBOUNCE_RANGE 5
#define THRESHOLD_DISTANCE_A	2
#define THRESHOLD_DISTANCE_B	4
#define VALID_USERDATA_OFFSET	120

#define SUCCESS 0
#define ERR_MORE_DEBOUNCE 1
#define ERR_DEBOUNCE_OVERTIME 2
#define ERR_UNMATCH_CERVE_PATTERN 3
#define ERR_USER_NOT_APPROACH 1

#define PA24_PS_POLL_DELAY		100

#define PA24_PS_ENABLE_DELAY	30

#define PA24_PS_TH_HIGH		40
#define PA24_PS_TH_LOW		25
#define PA24_PS_TH_MIN		0	// Minimun value
#define PA24_PS_TH_MAX		255     // 8 bit MAX

#define TOUCH_NEAR_DISTANCE	3       //Near distance 0 cm
#define TOUCH_FAR_DISTANCE	10       //Far distance 1 cm
#define TOUCH_UNKNOWN_DISTANCE	0       //Unkown distance 1 cm

#define PA24_PS_UNCOVER_MIN_SAFE 2 	// to decide if sensor need naked calibration
#define PA24_PS_UNCOVER_MAX_SAFE 100   // provided to decide if sensor is qualified
#define PA24_PS_OFFSET_DEFAULT	1 	// for X-talk cannceling
#define PA24_PS_OFFSET_MAX		150

#define PA24_MIN_NEAR_CNT	15  // min 3cm count
#define PA24_MAX_NEAR_CNT	75  // max 3cm count
#define PA24_MANUAL_OFFSET	2

#define LED_CURR_DEFAULT		7 	/*4:15mA | 5:12mA | 6:10mA | 7:7mA*/
#define PA24_PS_PRST			0	// 0:1point 1:2points 2:4points 3:8points (for INT)
#define PA24_PS_SET				1	// 0:No intTOUCH_UNKNOWN_DISTANCEupt 1:PS intTOUCH_UNKNOWN_DISTANCEupt only
#define PA24_PS_MODE			0	// 0:OFFSET 1:NORMAL
#define PA24_INT_TYPE			0 	// 0:Window type 1:Hysteresis type for Auto Clear flag , if ALS use intTOUCH_UNKNOWN_DISTANCEupt mode,should use windows mode
#define PA24_PS_PERIOD			0	/* 0:6.25 ms | 1:12.5 ms | 2:25 ms | 3:50 ms | 4:100 ms | 5:200 ms | 6:400 ms | 7:800 ms */

#define PS_INT			0 	//gpio_to_irq(xxxx) GPIO Define
#define PS_POLLING		0	// 0:INT Mode 1:Polling Mode

#define I2C_RETRY_TIMES			3
#define I2C_RETRY_DELAY			10	//10ms

/* POWER SUPPLY VOLTAGE RANGE */
#define PA224_VDD_MIN_UV  2000000
#define PA224_VDD_MAX_UV  3300000
#define PA224_VIO_MIN_UV  1750000
#define PA224_VIO_MAX_UV  1950000

/* Analog voltage @2.7 V */
#define AVDD_VTG_MIN_UV		3000000
#define AVDD_VTG_MAX_UV		3000000
#define AVDD_ACTIVE_LOAD_UA	15000

/* Digital voltage @1.8 V */
#define VDDIO_VTG_DIG_MIN_UV	1800000
#define VDDIO_VTG_DIG_MAX_UV	1800000
#define VDDIO_ACTIVE_LOAD_DIG_UA	10000

#define VDDIO_I2C_VTG_MIN_UV		1800000
#define VDDIO_I2C_VTG_MAX_UV		1800000
#define VDDIO_I2C_LOAD_UA		10000


/*Driver Internel Use only */

#define PA224_I2C_ADDRESS        	0x1E  	//7 bit Address

/*pa12200001 als/ps sensor register map*/
#define REG_CFG0 			0X00  	// PS_ON(D1)
#define REG_CFG1 			0X01  	// LED_CURR(D6-4),PS_PRST(D3-2)
#define REG_CFG2 			0X02  	// PS_MODE(D6),CLEAR(D4),INT_SET(D3-2),PS_INT(D1)
#define REG_CFG3			0X03  	// INT_TYPE(D6),PS_PERIOD(D5-3)
#define REG_PS_TL			0X08  	// PS Threshold Low
#define REG_PS_TH			0X0A  	// PS Threshold High
#define REG_PS_DATA			0X0E  	// PS DATA
#define REG_PS_OFFSET			0X10  	// TBD
#define REG_PS_SET			0X11  	// 0x82
#define REG_CFG4			0X12    //0x0C
#define REG_ID				0x7f
#define PS_ACTIVE			0x02
#define PS_INT_ACTIVE			0x02


/*IOCTL Define*/
#define TXC_IOC_MAGIC 't'
#define PA24_IOCTL_PS_ENABLE		_IOW(TXC_IOC_MAGIC,1,int)
#define PA24_IOCTL_PS_GET_DATA		_IOR(TXC_IOC_MAGIC,2,int)
#define PA24_IOCTL_PS_CALIBRATION	_IOR(TXC_IOC_MAGIC,3,int)

#define forward_step 	15
#define backward_step 	5

enum
{
	ERR_NAKED_CAL = 1,
	ERR_THRES_CAL,
	ERR_FILE_OPS,
	ERR_DEV_OPS,
	ERR_OTHER,
};

struct pa224_platform_data {
	//unsigned int ps_th_low;
	//unsigned int ps_th_high;

	int irq_gpio;  /* proximity/light-sensor- external irq*/

	int (*power)(unsigned char onoff);
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(bool);
};


struct pa224_data {
	struct i2c_client *client;
	struct mutex i2c_lock;
	struct mutex dev_lock;

	struct device *optical_key_device;
	struct pa224_platform_data *platform_data;

	struct input_dev *optical_key_input_dev;
	struct delayed_work optical_key_dwork;
	struct workqueue_struct *irq_work_queue;
	struct work_struct irq_dwork;
	struct wake_lock pa224_wake_lock;
	unsigned int optical_key_poll_delay;
	unsigned int enable_delay;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	struct regulator *vdd;
	//struct regulator *vio;
#ifdef SENSORS_CLASS_DEV
	struct sensors_classdev optical_key_cdev;
#endif
	/*vdd flag*/
	unsigned int vdd_always_on;

	int enable;
	bool debug;
	bool collect;
	int current_level;
	int collect_count;
	int collect_result;
	int calculate_result;
	u8 last_psdata;
	u8 last_collect_count;
	unsigned int last_collect_distance;
	unsigned int debounce_count;  //reaches 300 means debounced psdata gain fail
	unsigned int lasting_count;	//reaches 100 means debounced psdata gain success

	unsigned int far_threshold;
	unsigned int near_threshold;

	wait_queue_head_t wait;
	bool collect_finished;

	int near_diff_cnt;
	int far_diff_cnt;

	int irq;
	bool irq_enabled;

	/* PS Calibration */
	unsigned int crosstalk;
	unsigned int crosstalk_base;

	/* PS status */
	/* Object status, near = 3, far = 10 */
	unsigned int touch_status;
	unsigned int last_touch_status;
};

static int pa224_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int pa224_remove(struct i2c_client *client);
//static void pa224_report_event(struct pa224_data *data);
static void pa224_irq_enable(struct pa224_data *data, bool enable, bool flag_sync);
static int pa224_init_client(struct i2c_client *client);
#endif
