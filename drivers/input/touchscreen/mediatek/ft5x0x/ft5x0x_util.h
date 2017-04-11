

extern struct i2c_client *i2c_client;
extern struct tpd_device *tpd;
/***********************************************
//	SET RESET PIN
*/
#define TPD_PIN_RST 	(0)
#define TPD_PIN_EINT 	(1)
//#define TPD_PROXIMITY

static void ft5x0x_set_rst(bool bSet, int nDelay)
{
	tpd_gpio_output(TPD_PIN_RST, bSet?1:0);
	if (nDelay) mdelay(nDelay);
}
#ifdef TPD_PROXIMITY
#include "../../../../misc/mediatek/alsps/inc/alsps.h"
#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <hwmsen_helper.h>
#endif
#ifdef TPD_PROXIMITY

#define APS_ERR(fmt,arg...)           	//printk("<<proximity>> "fmt"\n",##arg)

#define TPD_PROXIMITY_DEBUG(fmt,arg...) //printk("<<proximity>> "fmt"\n",##arg)

#define TPD_PROXIMITY_DMESG(fmt,arg...) //printk("<<proximity>> "fmt"\n",##arg)

static u8 tpd_proximity_flag 			= 0;

static u8 tpd_proximity_flag_one 		= 0; //add for tpd_proximity by wangdongfang

static u8 tpd_proximity_detect 		= 1;//0-->close ; 1--> far away

#endif
/***********************************************
//	SET POWER TOUCHPANEL
*/
extern struct tpd_device* tpd;
static void ft5x0x_power(bool bOn)
{
	int retval;
	if (!tpd->reg)
	{
		tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
		retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
	}

	if  (bOn)	{
		retval = regulator_enable(tpd->reg);
	}else{
		retval = regulator_disable(tpd->reg);
	}
}

/***********************************************
//	GET PANEL PRESSURE STSTE
*/
#define TPD_SUPPORT_POINTS	2
struct touch_info
{
	int y[TPD_SUPPORT_POINTS];
	int x[TPD_SUPPORT_POINTS];
	int p[TPD_SUPPORT_POINTS];
	int id[TPD_SUPPORT_POINTS];
	int count;
};
static int tpd_touchinfo(struct i2c_client* i2c_client, struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
	char data[40] = {0};
	u8 report_rate = 0;
	u16 high_byte, low_byte;

	i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &(data[0]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x18, 8, &(data[24]));

	i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[32]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x88, 1, &report_rate);

	TPD_DEBUG("FW version=%x]\n",data[32]);

	if(report_rate < 8)
	{
		report_rate = 0x8;
		if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate)) < 0)
			TPD_DMESG("I2C write report rate error, line: %d\n", __LINE__);
	}

	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if((data[0] & 0x70) != 0)
		return false;

	memcpy(pinfo, cinfo, sizeof(struct touch_info));
	memset(cinfo, 0, sizeof(struct touch_info));
	for(i = 0; i < TPD_SUPPORT_POINTS; i++)
		cinfo->p[i] = 1;	//Put up

	/*get the number of the touch points*/
	cinfo->count = data[2] & 0x0f;

	TPD_DEBUG("Number of touch points = %d\n", cinfo->count);
	TPD_DEBUG("Procss raw data...\n");

	for(i = 0; i < cinfo->count; i++)
	{
		cinfo->p[i] = (data[3 + 6 * i] >> 6) & 0x0003; //event flag
		cinfo->id[i] = data[3 + 6 * i + 2] >> 4; //touch id

		/*get the X coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 1];
		low_byte &= 0x00FF;
		cinfo->x[i] = high_byte | low_byte;

		/*get the Y coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i + 2];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 3];
		low_byte &= 0x00FF;
		cinfo->y[i] = high_byte | low_byte;

		TPD_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i , cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
	}

	return true;
};

#ifdef TPD_PROXIMITY
int tpd_read_ps(void)
{
	tpd_proximity_detect;
	return 0;
}

static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}

static int tpd_enable_ps(int enable)
{
	u8 state;
	int ret = -1;

	i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
	//printk("[proxi_fts]read: 999 0xb0's value is 0x%02X\n", state);

	if (enable){
		state |= 0x01;
		tpd_proximity_flag = 1;
		TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is on\n");
	}else{
		state &= 0x00;
		tpd_proximity_flag = 0;
		TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is off\n");
	}

	ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &state);
	TPD_PROXIMITY_DEBUG("[proxi_fts]write: 0xB0's value is 0x%02X\n", state);
	return 0;
}

int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,

		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *sensor_data;
	TPD_DEBUG("[proxi_fts]command = 0x%02X\n", command);

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;
		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value)
				{
					if((tpd_enable_ps(1) != 0))
					{
						APS_ERR("enable ps fail: %d\n", err);
						return -1;
					}
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						APS_ERR("disable ps fail: %d\n", err);
						return -1;
					}
				}
			}
			break;
		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (struct hwm_sensor_data *)buff_out;
				if((err = tpd_read_ps()))
				{
					err = -1;;
				}
				else
				{
					sensor_data->values[0] = tpd_get_ps_value();
					TPD_PROXIMITY_DEBUG("huang sensor_data->values[0] 1082 = %d\n", sensor_data->values[0]);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				}
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	return err;
}
#endif
/***********************************************
//	CHECK KEYS
*/

extern void tpd_button(unsigned int x, unsigned int y, unsigned int down);
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag = 0;

static void tpd_down(int x, int y, int id)
{
    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(tpd->dev);

	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
	{
		tpd_button(x, y, 1);
	}
}

static void tpd_up(int x, int y)
{
    TPD_DEBUG("%s x:%d y:%d\n", __func__, x, y);
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_mt_sync(tpd->dev);

	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
	{
		tpd_button(x, y, 0);
	}
}

static int touch_event_handler(void *unused)
{
	#ifdef TPD_PROXIMITY
	int err;
	struct hwm_sensor_data sensor_data;
	u8 proximity_status;
	#endif
	u8 state;
	int i=0;
	struct touch_info cinfo, pinfo;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };

	sched_setscheduler(current, SCHED_RR, &param);

	do
	{
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;
		set_current_state(TASK_RUNNING);

#ifdef FTS_GESTRUE
	if (touch_getsure_event_handler(i2c_client)) continue;
#endif
	 #ifdef TPD_PROXIMITY

		 if (tpd_proximity_flag == 1)
		 {

			i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
            TPD_PROXIMITY_DEBUG("proxi_fts 0xB0 state value is 1131 0x%02X\n", state);
			if(!(state&0x01))
			{
				tpd_enable_ps(1);
			}
			i2c_smbus_read_i2c_block_data(i2c_client, 0x01, 1, &proximity_status);
            TPD_PROXIMITY_DEBUG("proxi_fts 0x01 value is 1139 0x%02X\n", proximity_status);
			if (proximity_status == 0xC0)
			{
				tpd_proximity_detect = 0;
			}
			else if(proximity_status == 0xE0)
			{
				tpd_proximity_detect = 1;
			}

			TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);
			if ((err = tpd_read_ps()))
			{
				TPD_PROXIMITY_DMESG("proxi_fts read ps data 1156: %d\n", err);
			}
			sensor_data.values[0] = tpd_get_ps_value();
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			//if ((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
			//{
			//	TPD_PROXIMITY_DMESG(" proxi_5206 call hwmsen_get_interrupt_data failed= %d\n", err);
			//}
		}

		#endif


		TPD_DEBUG("touch_event_handler start \n");
		if(tpd_touchinfo(i2c_client, &cinfo, &pinfo) == 0)
			continue;

		if(cinfo.count > 0)
		{
		    for(i =0; i < cinfo.count; i++)
		    {
		         tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
		    }
		}else{
		    tpd_up(cinfo.x[0], cinfo.y[0]);
		}
	   	input_sync(tpd->dev);

	} while (!kthread_should_stop());

	TPD_DEBUG("touch_event_handler exit \n");

	return 0;
}

/***********************************************
//	ATTACH TOUCHPANEL EVENT
*/
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	TPD_DEBUG("TPD interrupt has been triggered\n");
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}
static void tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	struct task_struct *thread;
	unsigned int touch_irq;
	int retval;

	node = of_find_matching_node(node, touch_of_match);
	if (node)
	{
		tpd_gpio_as_int(TPD_PIN_EINT);
		touch_irq = irq_of_parse_and_map(node, 0);
		retval = request_irq(touch_irq, tpd_eint_interrupt_handler,
						IRQF_TRIGGER_FALLING, TPD_DEVICE, NULL);
	}

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
}
