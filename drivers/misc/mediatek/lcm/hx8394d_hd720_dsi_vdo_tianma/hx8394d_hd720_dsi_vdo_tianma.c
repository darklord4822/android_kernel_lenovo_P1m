#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#endif

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#include <platform/mt_i2c.h>
#include <platform/upmu_common.h>
#include "ddp_hal.h"
#else
#endif

#include "lcm_drv.h"
#include <mt-plat/mt_gpio.h>
#include <mt-plat/mt_gpio_core.h>
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))
#define UDELAY(n) 											(lcm_util.udelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE  0

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             								0xFC
#define REGFLAG_UDELAY             								0xFB

#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER
#define REGFLAG_RESET_LOW       								0xFE
#define REGFLAG_RESET_HIGH      								0xFF


#define HX8394D_HD720_ID  (0x94)

static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef BUILD_LK
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

#define _LCM_DEBUG_

#ifdef BUILD_LK
#define printk printf
#endif

#ifdef _LCM_DEBUG_
#define lcm_debug(fmt, args...) printk(fmt, ##args)
#else
#define lcm_debug(fmt, args...) do { } while (0)
#endif

#ifdef _LCM_INFO_
#define lcm_info(fmt, args...) printk(fmt, ##args)
#else
#define lcm_info(fmt, args...) do { } while (0)
#endif
#define lcm_err(fmt, args...) printk(fmt, ##args)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_init_setting[] = {
	/*
	Note :

	Data ID will depends on the following rule.

	count of parameters > 1	=> Data ID = 0x39
	count of parameters = 1	=> Data ID = 0x15
	count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag

	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	{0xB9,  0x03 ,{0xFF, 0x83, 0x94}}, //FF 83 94
	{0xBA,  0x0b ,{0x73, 0x43, 0x20, 0x65,
					0xB2, 0x09, 0x09, 0x40,
					0x10, 0x00, 0x00}}, //73 43 20 65 B2 09 09 40 10 00 00
	{0xB1,  0x0f ,{0x6c, 0x11, 0x11, 0x37,
					0x04, 0x11, 0xf1, 0x80,
					0x9f, 0x94, 0x23, 0x80,
					0xc0, 0xd2, 0x18}}, //6c 11 11 37 04 11 F1 80 9f 94 23 80 C0 D2 18
	{0xB2,  0x0f ,{0x00, 0x64, 0x0e, 0x0d,
					0x32, 0x23, 0x08, 0x08,
					0x1c, 0x4d, 0x00, 0x00,
					0x30, 0x44, 0x48}}, //00 64 0E 0D 32 23 08 08 1C 4D 00 00 30 44 48
	{0xB4,  0x0c ,{0x00, 0xff, 0x03, 0x46,
					0x03, 0x46, 0x03, 0x46,
					0x03, 0x46, 0x01, 0x68,
					0x01, 0x68}}, //00 FF 03 46 03 46 03 46 01 68 01 68
	{0xBF,  0x03 ,{0x41, 0x0e, 0x01}}, //41 0E 01
	{0xD3,  0x25 ,{0x00, 0x07, 0x00, 0x00,
					0x00, 0x10, 0x00, 0x32,
					0x10, 0x05, 0x00, 0x05,
					0x32, 0x10, 0x00, 0x00,
					0x00, 0x32, 0x10, 0x00,
					0x00, 0x00, 0x36, 0x03,
					0x09, 0x09, 0x37, 0x00,
					0x00, 0x37, 0x00, 0x00,
					0x00, 0x00, 0x0a, 0x00,
					0x01,}}, //00 07 00 00 00 10 00 32 10 05 00 05 32 10 00 00 00 32 10 00 00 00 36 03 09 09 37 00 00 37 00 00 00 00 0A 00 01

	{0xD5,  0x2C ,{0x02, 0x03, 0x00, 0x01,
					0x06, 0x07, 0x04, 0x05,
					0x20, 0x21, 0x22, 0x23,
					0x18, 0x18, 0x18, 0x18,
					0x18, 0x18, 0x18, 0x18,
					0x18, 0x18, 0x18, 0x18,
					0x18, 0x18, 0x18, 0x18,
					0x18, 0x18, 0x18, 0x18,
					0x18, 0x18, 0x18, 0x18,
					0x18, 0x18, 0x24, 0x25,
					0x18, 0x18, 0x19, 0x19,}}, //02 03 00 01 06 07 04 05 20 21 22 23 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 24 25 18 18 19 19

	{REGFLAG_DELAY, 0x0a, {}},

	{0xD6,  0x2C ,{0x05, 0x04, 0x07, 0x06,
					0x01, 0x00, 0x03, 0x02,
					0x23, 0x22, 0x21, 0x20,
					0x18, 0x18, 0x18, 0x18,
					0x18, 0x18, 0x58, 0x58,
					0x18, 0x18, 0x18, 0x18,
					0x18, 0x18, 0x18, 0x18,
					0x18, 0x18, 0x18, 0x18,
					0x18, 0x18, 0x18, 0x18,
					0x18, 0x18, 0x25, 0x24,
					0x19, 0x19, 0x18, 0x18,}}, //05 04 07 06 01 00 03 02 23 22 21 20 18 18 18 18 18 18 58 58 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 25 24 19 19 18 18
	{REGFLAG_DELAY, 0x0a, {}},

#if 0
	{0xE0, 0x2a ,{0x02, 0x0a, 0x0a, 0x29,
					0x28, 0x3f, 0x20, 0x3f,
					0x0c, 0x0e, 0x0f, 0x17,
					0x0e, 0x12, 0x14, 0x13,
					0x14, 0x08, 0x12, 0x16,
					0x19, 0x02, 0x0a, 0x0a,
					0x29, 0x28, 0x3f, 0x20,
					0x3f, 0x0c, 0x0e, 0x0f,
					0x17, 0x0e, 0x12, 0x14,
					0x13, 0x14, 0x08, 0x12,
					0x16, 0x19,}}, //02 0a 0a 29 28 3F 20 3F 0c 0e 0f 17 0e 12 14 13 14 08 12 16 19 02 0a 0a 29 28 3f 20 3f 0c 0e 0f 17 0e 12 14 13 14 08 12 16 19
#else
	{0xE0, 0x2a ,{0x02, 0x10, 0x14, 0x2c,
					0x2f, 0x3f, 0x22, 0x40,
					0x06, 0x0a, 0x0c, 0x17,
					0x0e, 0x12, 0x14, 0x13,
					0x14, 0x08, 0x12, 0x14,
					0x1b, 0x02, 0x10, 0x14,
					0x2c, 0x2f, 0x3f, 0x22,
					0x40, 0x06, 0x0a, 0x0c,
					0x17, 0x0e, 0x12, 0x14,
					0x13, 0x14, 0x08, 0x12,
					0x14, 0x1b,}},
#endif

	{0xCC,  0x01 ,{0x09}}, //09
	{REGFLAG_DELAY, 0x0a, {}},

	{0xC7,  0x04 ,{0x00, 0xc0, 0x40, 0xc0}}, //00 C0 40 C0
	{0xC0,  0x02 ,{0x30, 0x14}}, //30 14
	{0xBC,  0x01 ,{0x07}}, //07
	{0x51,  0x01 ,{0xff}}, //FF
	{0x53,  0x01 ,{0x24}}, //24
	{0x55,  0x01 ,{0x00}}, //00
	{0xE4,	0x02 ,{0x00, 0x01}},//52 01
	{0x35,  0x01 ,{0x00}}, //00
	{0x11,  0x00 ,{}}, //00
	{REGFLAG_DELAY, 0xc8, {}},
	{0x29,  0x00 ,{}}, //00
	{REGFLAG_DELAY, 0x0a, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 150, {}},

	// Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 0, {}},
	{REGFLAG_DELAY, 60, {}},

	// Sleep Mode On
	{0x10, 0, {}},
	{REGFLAG_DELAY, 100, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void init_lcm_registers(void)
{
	unsigned int data_array[16];
	data_array[0]=0x00043902;
	data_array[1]=0x9483ffb9;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x000c3902;
	data_array[1]=0x204373ba;
	data_array[2]=0x0909B265;
	data_array[3]=0x00001040;
	dsi_set_cmdq(data_array, 4, 1);
	MDELAY(1);

	//BAh,1st para=73,2nd para=43,7th para=09,8th para=40,9th para=10,10th para=00,11th para=00
	data_array[0]=0x00103902;
	data_array[1]=0x11116Cb1;
	data_array[2]=0xF1110437;//vgl=-5.55*2
	data_array[3]=0x23949F80;
	data_array[4]=0x18D2C080;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(5);

	data_array[0]=0x000C3902;
	data_array[1]=0x0E6400b2;
	data_array[2]=0x0823320D;
	data_array[3]=0x004D1C08;
	data_array[4]=0x48443000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);

	data_array[0]=0x000D3902;
	data_array[1]=0x03FF00b4;
	data_array[2]=0x03460346;
	data_array[3]=0x01680346;
	data_array[4]=0x00000068;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);

	data_array[0]=0x00043902;
	data_array[1]=0x010E41BF;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00263902;
	data_array[1]=0x000700D3;
	data_array[2]=0x00100000;
	data_array[3]=0x00051032;
	data_array[4]=0x00103205;
	data_array[5]=0x10320000;
	data_array[6]=0x36000000;
	data_array[7]=0x37090903;   
	data_array[8]=0x00370000;
	data_array[9]=0x0A000000;
	data_array[10]=0x00000100;
	dsi_set_cmdq(data_array, 11, 1);
	MDELAY(1);

	data_array[0]=0x002D3902;
	data_array[1]=0x000302d5;
	data_array[2]=0x04070601;
	data_array[3]=0x22212005;
	data_array[4]=0x18181823;
	data_array[5]=0x18181818;
	data_array[6]=0x18181818;
	data_array[7]=0x18181818;   
	data_array[8]=0x18181818;
	data_array[9]=0x18181818;
	data_array[10]=0x24181818;
	data_array[11]=0x19181825;
	data_array[12]=0x00000019;
	dsi_set_cmdq(data_array, 13, 1);
	MDELAY(5);//recommond 10ms

	data_array[0]=0x002D3902;
	data_array[1]=0x070405D6;
	data_array[2]=0x03000106;
	data_array[3]=0x21222302;
	data_array[4]=0x18181820;
	data_array[5]=0x58181818;
	data_array[6]=0x18181858;
	data_array[7]=0x18181818;   
	data_array[8]=0x18181818;
	data_array[9]=0x18181818;
	data_array[10]=0x25181818;
	data_array[11]=0x18191924;
	data_array[12]=0x00000018;
	dsi_set_cmdq(data_array, 13, 1);
	MDELAY(5);//recommond 10ms

/*  data_array[0]=0x002b3902;// gamma 2.0
	data_array[1]=0x120e02e0;
	data_array[2]=0x1F3F2C27;
	data_array[3]=0x0C0A063C;
	data_array[4]=0x15120F17;
	data_array[5]=0x12071513;
	data_array[6]=0x0E021713;
	data_array[7]=0x3F2C2712;   
	data_array[8]=0x0A063C1F;
	data_array[9]=0x120F170C;
	data_array[10]=0x07151315;
	data_array[11]=0x00171312;
	dsi_set_cmdq(data_array, 12, 1);
	MDELAY(1);
*/

  data_array[0]=0x002b3902;// gamma 2.1
	data_array[1]=0x130f02e0;
	data_array[2]=0x203F2e2a;
	data_array[3]=0x0C0A063e;
	data_array[4]=0x15130F17;
	data_array[5]=0x12081513;
	data_array[6]=0x0f021713;
	data_array[7]=0x3F2e2a13;   
	data_array[8]=0x0A063e20;
	data_array[9]=0x130F170C;
	data_array[10]=0x08151315;
	data_array[11]=0x00171312;
	dsi_set_cmdq(data_array, 12, 1);
	MDELAY(1);



/*	data_array[0]=0x002b3902;//gamma 2.2
	data_array[1]=0x141002e0;
	data_array[2]=0x223F2F2c;
	data_array[3]=0x0C0A0640;
	data_array[4]=0x14120E17;
	data_array[5]=0x12081413;
	data_array[6]=0x10021B14;
	data_array[7]=0x3F2F2C14;   
	data_array[8]=0x0A064022;
	data_array[9]=0x120E170C;
	data_array[10]=0x08141314;
	data_array[11]=0x001B1412;
	dsi_set_cmdq(data_array, 12, 1);
	MDELAY(1);
*/
	data_array[0]=0x00023902;
	data_array[1]=0x000009cc;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(5);//recommond 10ms

	data_array[0]=0x00053902;
	data_array[1]=0x40C000c7;
	data_array[2]=0x000000C0;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(5);//recommond 5ms

	//data_array[0]=0x00033902;
	//data_array[1]=0x007F69b6;//VCOM
	//dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00033902;
	data_array[1]=0x001430C0;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00023902;
	data_array[1]=0x000007BC;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00023902; //Pin 6 PWM NC at main board
	data_array[1]=0x0000FF51;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00023902;//Pin 6 PWM NC at main board
	data_array[1]=0x00002453;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00023902;//Pin 6 PWM NC at main board
	data_array[1]=0x00000055;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(5);

	data_array[0]=0x00033902;//Enable ce
	data_array[1]=0x000100E4;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(5);//recommond 5ms

	data_array[0]=0x00023902;
	data_array[1]=0x00000035;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);

	data_array[0]= 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(200);

	data_array[0]= 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);   

	//data_array[0]=0x00023902;
	//data_array[1]=0x000062b6;//VCOM
	//dsi_set_cmdq(data_array, 2, 1);
	//MDELAY(10);	
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++)
	{
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY :
				MDELAY(table[i].count);
				break;

			case REGFLAG_UDELAY :
				UDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE :
				break;

			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
#endif

	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 12;
	params->dsi.vertical_frontporch					= 16;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 24;
	params->dsi.horizontal_backporch				= 100;
	params->dsi.horizontal_frontporch				= 50;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	//params->dsi.LPX=8; 

	// Bit rate calculation
	params->dsi.PLL_CLOCK = 229;

	params->dsi.clk_lp_per_line_enable = 0;
	
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x09;
	params->dsi.lcm_esd_check_table[0].count        = 3;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
	params->dsi.lcm_esd_check_table[0].para_list[1] = 0x73;
	params->dsi.lcm_esd_check_table[0].para_list[2] = 0x06;

	params->dsi.lcm_esd_check_table[1].cmd		= 0xD9;
	params->dsi.lcm_esd_check_table[1].count	= 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;
}

static void lcm_init_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
// guanyuwei@wt, 20150528,  no need support power for this i2c device in wt98735
#if 0
#ifdef BUILD_LK
	pmic_set_register_value(PMIC_RG_VGP1_EN,1);
#else
	printk("%s, begin\n", __func__);
	hwPowerOn(MT6328_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");	
	printk("%s, end\n", __func__);
#endif
#endif
#endif
}

static void lcm_suspend_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#if 0
#ifdef BUILD_LK
	pmic_set_register_value(PMIC_RG_VGP1_EN,0);
#else
	printk("%s, begin\n", __func__);
	hwPowerDown(MT6328_POWER_LDO_VGP1, "LCM_DRV");	
	printk("%s, end\n", __func__);
#endif
#endif
#endif
}

static void lcm_resume_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#if 0
#ifdef BUILD_LK
	pmic_set_register_value(PMIC_RG_VGP1_EN,1);
#else
	printk("%s, begin\n", __func__);
	hwPowerOn(MT6328_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");	
	printk("%s, end\n", __func__);
#endif
#endif
#endif
}

static void set_vsp_vsn_pin(int value)
{
	if (value) {
        gpio_direction_output(3, GPIO_OUT_ONE);
		MDELAY(15);
		gpio_direction_output(4, GPIO_OUT_ONE);
		MDELAY(15);
	} else {
        gpio_direction_output(4, GPIO_OUT_ZERO);
		MDELAY(15);
		gpio_direction_output(3, GPIO_OUT_ZERO);
		MDELAY(15);
	}
}

static void set_reset_gpio_pin(int value)
{
	if(value) {
		gpio_direction_output(146, GPIO_OUT_ONE);
	} else {
		gpio_direction_output(146, GPIO_OUT_ZERO);
	}
}

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0x0a;
	int ret=0;

#ifndef CONFIG_FPGA_EARLY_PORTING
	set_vsp_vsn_pin(1);
#endif

#ifdef GPIO_LCD_MAKER_ID
	mt_set_gpio_mode(GPIO_LCD_MAKER_ID, GPIO_LCD_MAKER_ID_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_LCD_MAKER_ID, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_LCD_MAKER_ID, GPIO_DIR_IN);
#endif
	
	lcm_debug("%s %d\n", __func__,__LINE__);
//	SET_RESET_PIN(0);
	set_reset_gpio_pin(0);
	MDELAY(5);
//	SET_RESET_PIN(1);
	set_reset_gpio_pin(1);
	MDELAY(125);
//	push_table(lcm_init_setting, sizeof(lcm_init_setting) / sizeof(struct LCM_setting_table), 1);
	init_lcm_registers();
}

static void lcm_suspend(void)
{
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);  
	MDELAY(10);

//	SET_RESET_PIN(0);
	set_reset_gpio_pin(0);
	MDELAY(10);

	set_vsp_vsn_pin(0);
}

static void lcm_resume(void)
{
	lcm_init(); 
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
  unsigned char cmd = 0x0;
	unsigned char data = 0x0a;
	int ret=0;
	unsigned int data_array[2];
	unsigned char buffer;
	unsigned int array[16];

#ifndef CONFIG_FPGA_EARLY_PORTING
	set_vsp_vsn_pin(1);
#endif
	lcm_debug("%s %d\n", __func__,__LINE__);

	set_reset_gpio_pin(1);	
  MDELAY(5);
	set_reset_gpio_pin(0);
	MDELAY(10);
//	SET_RESET_PIN(1);
	set_reset_gpio_pin(1);
	MDELAY(125);

	data_array[0]= 0x00043902;
	data_array[1]= (0x94<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00023902;
	data_array[1]= (0x33<<8)|0xba;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00043902;
	data_array[1]= (0x94<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00013700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0xf4, &buffer, 1);

	#ifdef BUILD_LK
		printf("%s, LK debug: hx8394d id = 0x%08x\n", __func__, buffer);
    #else
		printk("%s, kernel debug: hx8394d id = 0x%08x\n", __func__, buffer);
    #endif

	return (buffer == HX8394D_HD720_ID ? 1 : 0);

}

#if 0
static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	char  buffer;
	read_reg_v2(0x0a, &buffer, 1);
	printk("%s, kernel debug: reg = 0x%08x\n", __func__, buffer);

	return FALSE;
	
#else
	return FALSE;
#endif

}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();

	return TRUE;
}
#endif

static void* lcm_switch_mode(int mode)
{
#ifndef BUILD_LK
//customization: 1. V2C config 2 values, C2V config 1 value; 2. config mode control register
	if(mode == 0)
	{//V2C
		lcm_switch_mode_cmd.mode = CMD_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;// mode control addr
		lcm_switch_mode_cmd.val[0]= 0x13;//enabel GRAM firstly, ensure writing one frame to GRAM
		lcm_switch_mode_cmd.val[1]= 0x10;//disable video mode secondly
	}
	else
	{//C2V
		lcm_switch_mode_cmd.mode = SYNC_PULSE_VDO_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;
		lcm_switch_mode_cmd.val[0]= 0x03;//disable GRAM and enable video mode
	}
	return (void*)(&lcm_switch_mode_cmd);
#else
	return NULL;
#endif
}

static int bl_Cust_Max = 1023;
static int bl_Cust_Min = 28;

static int setting_max = 1023;
static int setting_min = 40;

static int lcm_brightness_mapping(int level)
{
   int mapped_level;

   if(level >= setting_min){
	 mapped_level = ((bl_Cust_Max-bl_Cust_Min)*level+(setting_max*bl_Cust_Min)-(setting_min*bl_Cust_Max))/(setting_max-setting_min);
   }else{
	 mapped_level = (bl_Cust_Min*level)/setting_min;
   } 

   #ifdef BUILD_LK
   printf("level= %d, lcm_brightness_mapping= %d\n", level, mapped_level);
   #else
   printk("level= %d, lcm_brightness_mapping= %d\n", level, mapped_level);
   #endif

  return mapped_level;
}

#ifdef CONFIG_WT_GAMMA_PQ_WITH_MULTI_LCM	
DISP_PQ_PARAM_LCD lcm_pq_standard_tianma = 
{	
	u4SHPGain:2,
	u4SatGain:4,
	u4HueAdj:{9,9,12,12},
	u4SatAdj:{0,9,12,13},   // AE3 tuning for K35 LCM
	u4Contrast:4,
	u4Brightness:4
};
DISP_PQ_PARAM_LCD lcm_pq_vivid_tianma = 
{	
	u4SHPGain:2,
	u4SatGain:9,
	u4HueAdj:{9,9,12,12},
	u4SatAdj:{18,18,18,18},
	u4Contrast:4,
	u4Brightness:4
};	
				
void lcm_get_pq_standard_param_tianma(DISP_PQ_PARAM_LCD *pq_data)	
{
	if(pq_data != NULL)
		memcpy(pq_data, &lcm_pq_standard_tianma, sizeof(DISP_PQ_PARAM_LCD));
}
void lcm_get_pq_vivid_param_tianma(DISP_PQ_PARAM_LCD *pq_data)	
{
	if(pq_data != NULL)
		memcpy(pq_data, &lcm_pq_vivid_tianma, sizeof(DISP_PQ_PARAM_LCD));
}
#endif

LCM_DRIVER hx8394d_hd720_dsi_vdo_tianma_lcm_drv = 
{
	.name			= "hx8394d_hd720_dsi_tianma_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.init_power		= lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	//.esd_check 	= lcm_esd_check,
	//.esd_recover	= lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
#ifdef CONFIG_WT_BRIGHTNESS_MAPPING_WITH_LCM
	.cust_mapping = lcm_brightness_mapping,
#endif
#ifdef CONFIG_WT_GAMMA_PQ_WITH_MULTI_LCM	
	.pq_standard_param = lcm_get_pq_standard_param_tianma,	
	.pq_vivid_param    = lcm_get_pq_vivid_param_tianma,	
#endif    
};
