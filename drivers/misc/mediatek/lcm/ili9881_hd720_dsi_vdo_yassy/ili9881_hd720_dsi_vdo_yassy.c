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
#define LCM_DSI_CMD_MODE									0

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0xFC
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER


#define LCM_ID_ILI9881                                      (0x9881)

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
	//CCMON
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
     

	{ 0xFF, 0x03, {0x98, 0x81, 0x03}},
    { 0x01, 0x01, {0x00}},
    { 0x02, 0x01, {0x00}},
    { 0x03, 0x01, {0x53}},
    { 0x04, 0x01, {0x14}},
    { 0x05, 0x01, {0x00}},
    { 0x06, 0x01, {0x06}},
    { 0x07, 0x01, {0x01}},
    { 0x08, 0x01, {0x00}},
    { 0x09, 0x01, {0x01}},
    { 0x0A, 0x01, {0x19}},
    { 0x0B, 0x01, {0x01}},
    { 0x0C, 0x01, {0x00}},
    { 0x0D, 0x01, {0x00}},
    { 0x0E, 0x01, {0x00}},
    { 0x0F, 0x01, {0x19}},
    { 0x10, 0x01, {0x19}},
    { 0x11, 0x01, {0x00}},
    { 0x12, 0x01, {0x00}},
    { 0x13, 0x01, {0x00}},
    { 0x14, 0x01, {0x00}},
    { 0x15, 0x01, {0x00}},
    { 0x16, 0x01, {0x00}},
    { 0x17, 0x01, {0x00}},
    { 0x18, 0x01, {0x00}},
    { 0x19, 0x01, {0x00}},
    { 0x1A, 0x01, {0x00}},
    { 0x1B, 0x01, {0x00}},
    { 0x1C, 0x01, {0x00}},
    { 0x1D, 0x01, {0x00}},
    { 0x1E, 0x01, {0x40}},
    { 0x1F, 0x01, {0x40}},
    { 0x20, 0x01, {0x02}},
    { 0x21, 0x01, {0x05}},
    { 0x22, 0x01, {0x02}},
    { 0x23, 0x01, {0x00}},
    { 0x24, 0x01, {0x87}},
    { 0x25, 0x01, {0x87}},
    { 0x26, 0x01, {0x00}},
    { 0x27, 0x01, {0x00}},
    { 0x28, 0x01, {0x3B}},
    { 0x29, 0x01, {0x03}},
    { 0x2A, 0x01, {0x00}},
    { 0x2B, 0x01, {0x00}},
    { 0x2C, 0x01, {0x00}},
    { 0x2D, 0x01, {0x00}},
    { 0x2E, 0x01, {0x00}},
    { 0x2F, 0x01, {0x00}},
    { 0x30, 0x01, {0x00}},
    { 0x31, 0x01, {0x00}},
    { 0x32, 0x01, {0x00}},
    { 0x33, 0x01, {0x00}},
    { 0x34, 0x01, {0x04}},
    { 0x35, 0x01, {0x00}},
    { 0x36, 0x01, {0x00}},
    { 0x37, 0x01, {0x00}},
    { 0x38, 0x01, {0x01}},
    { 0x39, 0x01, {0x01}},
    { 0x3A, 0x01, {0x40}},
    { 0x3B, 0x01, {0x40}},
    { 0x3C, 0x01, {0x00}},
    { 0x3D, 0x01, {0x00}},
    { 0x3E, 0x01, {0x00}},
    { 0x3F, 0x01, {0x00}},
    { 0x40, 0x01, {0x00}},
    { 0x41, 0x01, {0x88}},
    { 0x42, 0x01, {0x00}},
    { 0x43, 0x01, {0x00}},
    { 0x44, 0x01, {0x00}},
    { 0x50, 0x01, {0x01}},
    { 0x51, 0x01, {0x23}},
    { 0x52, 0x01, {0x45}},
    { 0x53, 0x01, {0x67}},
    { 0x54, 0x01, {0x89}},
    { 0x55, 0x01, {0xAB}},
    { 0x56, 0x01, {0x01}},
    { 0x57, 0x01, {0x23}},
    { 0x58, 0x01, {0x45}},
    { 0x59, 0x01, {0x67}},
    { 0x5A, 0x01, {0x89}},
    { 0x5B, 0x01, {0xAB}},
    { 0x5C, 0x01, {0xCD}},
    { 0x5D, 0x01, {0xEF}},
    { 0x5E, 0x01, {0x11}},
    { 0x5F, 0x01, {0x06}},
    { 0x60, 0x01, {0x0C}},
    { 0x61, 0x01, {0x0D}},
    { 0x62, 0x01, {0x0E}},
    { 0x63, 0x01, {0x0F}},
    { 0x64, 0x01, {0x02}},
    { 0x65, 0x01, {0x02}},
    { 0x66, 0x01, {0x02}},
    { 0x67, 0x01, {0x02}},
    { 0x68, 0x01, {0x02}},
    { 0x69, 0x01, {0x02}},
    { 0x6A, 0x01, {0x02}},
    { 0x6B, 0x01, {0x02}},
    { 0x6C, 0x01, {0x02}},
    { 0x6D, 0x01, {0x02}},
    { 0x6E, 0x01, {0x05}},
    { 0x6F, 0x01, {0x05}},
    { 0x70, 0x01, {0x05}},
    { 0x71, 0x01, {0x02}},
    { 0x72, 0x01, {0x01}},
    { 0x73, 0x01, {0x00}},
    { 0x74, 0x01, {0x08}},
    { 0x75, 0x01, {0x08}},
    { 0x76, 0x01, {0x0C}},
    { 0x77, 0x01, {0x0D}},
    { 0x78, 0x01, {0x0E}},
    { 0x79, 0x01, {0x0F}},
    { 0x7A, 0x01, {0x02}},
    { 0x7B, 0x01, {0x02}},
    { 0x7C, 0x01, {0x02}},
    { 0x7D, 0x01, {0x02}},
    { 0x7E, 0x01, {0x02}},
    { 0x7F, 0x01, {0x02}},
    { 0x80, 0x01, {0x02}},
    { 0x81, 0x01, {0x02}},
    { 0x82, 0x01, {0x02}},
    { 0x83, 0x01, {0x02}},
    { 0x84, 0x01, {0x05}},
    { 0x85, 0x01, {0x05}},
    { 0x86, 0x01, {0x05}},
    { 0x87, 0x01, {0x02}},
    { 0x88, 0x01, {0x01}},
    { 0x89, 0x01, {0x00}},
    { 0x8A, 0x01, {0x06}},
    { 0xFF, 0x03, {0x98, 0x81, 0x04}},
    { 0x6C, 0x01, {0x15}},
    { 0x6E, 0x01, {0x2B}},
    { 0x6F, 0x01, {0xB3}},
    { 0x3A, 0x01, {0xA4}},
    { 0x8D, 0x01, {0x15}},
    { 0x87, 0x01, {0xBA}},
    { 0x26, 0x01, {0x76}},
    { 0xB2, 0x01, {0xD1}},
    { 0xFF, 0x03, {0x98, 0x81, 0x01}},
    { 0x22, 0x01, {0x0A}},
    { 0x55, 0x01, {0xA5}},
    { 0x50, 0x01, {0xAE}},
    { 0x51, 0x01, {0xAF}},
    { 0x31, 0x01, {0x00}},
    { 0x60, 0x01, {0x14}},
    { 0xA0, 0x01, {0x1D}},
    { 0xA1, 0x01, {0x22}},
    { 0xA2, 0x01, {0x2B}},
    { 0xA3, 0x01, {0x0E}},
    { 0xA4, 0x01, {0x10}},
    { 0xA5, 0x01, {0x23}},
    { 0xA6, 0x01, {0x17}},
    { 0xA7, 0x01, {0x15}},
    { 0xA8, 0x01, {0x97}},
    { 0xA9, 0x01, {0x15}},
    { 0xAA, 0x01, {0x26}},
    { 0xAB, 0x01, {0x85}},
    { 0xAC, 0x01, {0x18}},
    { 0xAD, 0x01, {0x14}},
    { 0xAE, 0x01, {0x47}},
    { 0xAF, 0x01, {0x22}},
    { 0xB0, 0x01, {0x25}},
    { 0xB1, 0x01, {0x59}},
    { 0xB2, 0x01, {0x6B}},
    { 0xB3, 0x01, {0x39}},
    { 0xC0, 0x01, {0x1D}},
    { 0xC1, 0x01, {0x2D}},
    { 0xC2, 0x01, {0x38}},
    { 0xC3, 0x01, {0x09}},
    { 0xC4, 0x01, {0x0B}},
    { 0xC5, 0x01, {0x1E}},
    { 0xC6, 0x01, {0x16}},
    { 0xC7, 0x01, {0x1F}},
    { 0xC8, 0x01, {0x8F}},
    { 0xC9, 0x01, {0x20}},
    { 0xCA, 0x01, {0x2A}},
    { 0xCB, 0x01, {0x93}},
    { 0xCC, 0x01, {0x1F}},
    { 0xCD, 0x01, {0x20}},
    { 0xCE, 0x01, {0x55}},
    { 0xCF, 0x01, {0x22}},
    { 0xD0, 0x01, {0x2C}},
    { 0xD1, 0x01, {0x5A}},
    { 0xD2, 0x01, {0x67}},
    { 0xD3, 0x01, {0x39}},
    { 0xFF, 0x03, {0x98, 0x81, 0x00}},
    { 0x35, 0x01, {0x00}},
    { 0x11, 0x00, {0x00}},
    { REGFLAG_DELAY, 0x78, {0x00}},
    { 0x29, 0x00, {0x00}},
    { REGFLAG_DELAY, 0x14, {0x00}},
    { REGFLAG_END_OF_TABLE, 0x00, {0x00}}
};
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    //Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

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

		params->dbi.te_mode				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		//LCM_DBI_TE_MODE_DISABLED;
		//LCM_DBI_TE_MODE_VSYNC_ONLY;  
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING; 
		/////////////////////   
		//if(params->dsi.lcm_int_te_monitor)  
		//params->dsi.vertical_frontporch *=2;  
		//params->dsi.lcm_ext_te_monitor= 0;//TRUE; 
	//	params->dsi.noncont_clock= TRUE;//FALSE;   
	//	params->dsi.noncont_clock_period=2;
		params->dsi.cont_clock=1;    //modify  yudengwu
		////////////////////          
		params->dsi.mode   = BURST_VDO_MODE; 
		// DSI    /* Command mode setting */  
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;      
		//The following defined the fomat for data coming from LCD engine.  
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;   
		params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST; 
		params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;    
		params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;       
		// Video mode setting		   
		params->dsi.intermediat_buffer_num = 2;  
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;  
		params->dsi.packet_size=256;    
		// params->dsi.word_count=480*3;	
		//DSI CMD mode need set these two bellow params, different to 6577   
		// params->dsi.vertical_active_line=800;   
		params->dsi.vertical_sync_active				= 4; //4   
		params->dsi.vertical_backporch				       = 20;  //14  
		params->dsi.vertical_frontporch				       = 10;  //16  
		params->dsi.vertical_active_line				       = FRAME_HEIGHT;     
		params->dsi.horizontal_sync_active				= 60;   //4
		params->dsi.horizontal_backporch				= 100;  //60  
		params->dsi.horizontal_frontporch				= 80;    //60
		params->dsi.horizontal_blanking_pixel				= 60;   
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;  
		params->dsi.ssc_disable=1;
		params->dsi.PLL_CLOCK = 229;	   // 245;
        params->dsi.clk_lp_per_line_enable = 0;
		params->dsi.HS_TRAIL = 14;
		params->dsi.esd_check_enable = 0;
        params->dsi.customization_esd_check_enable = 0;
        params->dsi.lcm_esd_check_table[0].cmd                  = 9;
        params->dsi.lcm_esd_check_table[0].count                = 3;
        params->dsi.lcm_esd_check_table[0].para_list[0] = -128;
        params->dsi.lcm_esd_check_table[0].para_list[1] = 115;
        params->dsi.lcm_esd_check_table[1].cmd = 6;
        params->dsi.lcm_esd_check_table[1].count = -39;
        params->dsi.lcm_esd_check_table[1].para_list[0] = 1;
        params->dsi.lcm_esd_check_table[1].para_list[1] = -128;

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
    push_table(lcm_init_setting, sizeof(lcm_init_setting) / sizeof(struct LCM_setting_table), 1);
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

	return (buffer == LCM_ID_ILI9881 ? 1 : 0);

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

LCM_DRIVER ili9881_hd720_dsi_vdo_yassy_lcm_drv =
{
    .name           	= "ili9881_hd720_dsi_vdo_yassy",
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