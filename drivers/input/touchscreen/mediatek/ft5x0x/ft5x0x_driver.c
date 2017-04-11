#include <linux/interrupt.h>
#include <mt_boot_common.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

//#include "flash_on.h"
#define FTS_GESTRUE
//#define TPD_CLOSE_POWER_IN_SLEEP

#include "tpd.h"

#ifdef FTS_GESTRUE
#include "ft5x0x_getsure.h"
#endif

#include "ft5x0x_util.h"



extern struct tpd_device *tpd;
extern int tpd_v_magnify_x;
extern int tpd_v_magnify_y;

struct i2c_client *i2c_client = NULL;

static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);
	return 0;
}
#define TPD_MAX_RESET_COUNT	3
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	char data;
	int reset_count;
	int retval = 0;
	i2c_client = client;

	TPD_DMESG("mtk_tpd: tpd_probe ft5x0x \n");
	for (reset_count = 0; reset_count < TPD_MAX_RESET_COUNT; ++reset_count)
	{
		ft5x0x_set_rst(false, 5);
		ft5x0x_power(true);
		ft5x0x_set_rst(true, 20);

		if(i2c_smbus_read_i2c_block_data(client, 0x00, 1, &data) >= 0)
		{
			tpd_irq_registration();
			// Extern variable MTK touch driver
		    tpd_load_status = 1;

			TPD_DMESG("Touch Panel Device Probe %s\n", (retval < 0) ? "FAIL" : "PASS");
			#ifdef TPD_PROXIMITY
				struct hwmsen_object obj_ps;
				obj_ps.polling = 0;	/* 0--interrupt mode;1--polling mode; */
				obj_ps.sensor_operate = tpd_ps_operate;
				s32 err_hw = hwmsen_attach(ID_PROXIMITY, &obj_ps);
				if (err_hw)
					TPD_DEBUG("hwmsen attach fail, return:%d.", err_hw);
			#endif
			return 0;
		}
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
	};

	return -1;
}

static int tpd_remove(struct i2c_client *client)
{
	TPD_DEBUG("TPD removed\n");
	return 0;
}

static const struct i2c_device_id ft5x0x_tpd_id[] = {{"ft5x0x",0},{}};
static const struct of_device_id ft5x0x_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};

MODULE_DEVICE_TABLE(of, ft5x0x_dt_match);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.of_match_table = ft5x0x_dt_match,
		.name = "ft5x0x",
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = ft5x0x_tpd_id,
	.detect = tpd_detect,
};
/********************************************/
static int tpd_local_init(void)
{
#ifdef FTS_GESTRUE
	fts_i2c_Init();
#endif
//	TPD_DMESG("Focaltech FT5x0x I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
	if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}

	tpd_button_setting(tpd_dts_data.tpd_key_num,
			tpd_dts_data.tpd_key_local,
			tpd_dts_data.tpd_key_dim_local);

	if (tpd_dts_data.touch_filter.enable)
	{
		tpd_v_magnify_x = tpd_dts_data.touch_filter.VECLOCITY_THRESHOLD[0];
		tpd_v_magnify_y = tpd_dts_data.touch_filter.VECLOCITY_THRESHOLD[1];
	}


	TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

static void tpd_resume(struct device *h)
{
	static char data;
#ifdef FTS_GESTRUE
	if (tpd_getsure_resume(i2c_client)) return;
#endif

#ifdef TPD_CLOSE_POWER_IN_SLEEP
   	TPD_DMESG("TPD wake up\n");
	ft5x0x_set_rst(false, 5);
	ft5x0x_power(true);
	ft5x0x_set_rst(true, 20);
#else
//	data = 0x00;
//	i2c_smbus_write_i2c_block_data(i2c_client, 0xd0, 1, &data);
#endif

//	tpd_up(0,0);
//	input_sync(tpd->dev);
	TPD_DMESG("TPD wake up done\n");
}

static void tpd_suspend(struct device *h)
{
	static char data;

#ifdef FTS_GESTRUE
	if (tpd_getsure_suspend(i2c_client)) return;
#endif
	//return;

	TPD_DEBUG("TPD enter sleep\n");
	data = 0x3;
	i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode

#ifdef TPD_CLOSE_POWER_IN_SLEEP
	ft5x0x_power(false);
#endif
}


static struct device_attribute *ft5x0x_attrs[] = {

#ifdef FTS_GESTRUE
	 &dev_attr_tpgesture,
	 &dev_attr_tpgesture_status,
#endif
};

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "FT5x0x",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
	.attrs = {
		.attr = ft5x0x_attrs,
		.num  = ARRAY_SIZE(ft5x0x_attrs),
	},
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	printk("MediaTek FT5x0x touch panel driver init\n");
	tpd_get_dts_info();
	if(tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add FT5x0x driver failed\n");

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	TPD_DMESG("MediaTek FT5x0x touch panel driver exit\n");
       	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
