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


#define OTM1284A_HD720_ID  (0x1284)

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
#if 0
static struct LCM_setting_table lcm_init_setting[] = {


	{0x00,  1 ,{0x00}}, 
	{0xff,  3 ,{0x12,0x84,0x01}}, 
	
	{0x00,  1 ,{0x80}}, 
	{0xff,  2 ,{0x12,0x84}},
	
	{0x00,  1 ,{0x92}}, 
	{0xff,  2 ,{0x30,0x02}},
	
	{0x00,  1 ,{0x80}}, 
	{0xc0,  9 ,{0x00,0x64,0x00,0x10,0x10,0x00,0x64,0x10,0x10}},
	
	{0x00,  1 ,{0x90}}, 
	{0xc0,  6 ,{0x00,0x5c,0x00,0x01,0x00,0x04}},	
	
	{0x00,  1 ,{0xb3}}, 
	{0xc0,  2 ,{0x00,0x55}},	
	
	{0x00,  1 ,{0x81}}, 
	{0xc1,  1 ,{0x55}},
	
	{0x00,  1 ,{0xd2}}, 
	{0xc1,  1 ,{0x03}},
	
	{0xc1,  1 ,{0x91}}, 
	{0xc1,  1 ,{0x9a}},		

	{0x00,  1 ,{0x90}}, 
	{0xc1,  1 ,{0x00}},
	
	{0xc1,  1 ,{0xa0}}, 
	{0xc1,  1 ,{0x02}},	

	{0x00,  1 ,{0xb6}}, 
	{0xc1,  1 ,{0x03}},	
	
	
	{0x00,  1 ,{0xa0}}, 
	{0xc4,  14 ,{0x05,0x10,0x04,0x02,0x05,0x15,0x11,0x05,0x10,0x07,0x02,0x05,0x15,0x11}},

	{0x00,  1 ,{0xb0}}, 
	{0xc4,  2 ,{0x00,0x00}},

	{0x00,  1 ,{0x91}}, 
	{0xc5,  2 ,{0xa6,0xd2}},

	{0x00,  1 ,{0x00}}, 
	{0xd8,  2 ,{0xc7,0xc7}},

	{0x00,  1 ,{0x00}}, 
	{0xd9,  1 ,{0x72}},
	
	{0x00,  1 ,{0xb3}}, 
	{0xc5,  1 ,{0x84}},

  {0x00,  1 ,{0x82}}, 
	{0xc4,  1 ,{0x0a}},
	
  {0x00,  1 ,{0xc6}}, 
	{0xb0,  1 ,{0x03}},	
	
  {0x00,  1 ,{0xc2}}, 
	{0xf5,  1 ,{0x40}},	
	
  {0x00,  1 ,{0xc3}}, 
	{0xf5,  1 ,{0x80}},	
	
  {0x00,  1 ,{0x00}}, 
	{0xd0,  1 ,{0x40}},	
	
  {0x00,  1 ,{0x00}}, 
	{0xd1,  2 ,{0x00,0x00}},	
	
  {0x00,  1 ,{0xb2}}, 
	{0xf5,  2 ,{0x00,0x00}},	
	
  {0x00,  1 ,{0xb4}}, 
	{0xf5,  2 ,{0x00,0x00}},

  {0x00,  1 ,{0xb6}}, 
	{0xf5,  2 ,{0x00,0x00}},
	
  {0x00,  1 ,{0xb8}}, 
	{0xf5,  2 ,{0x00,0x00}},	

  {0x00,  1 ,{0x94}}, 
	{0xf5,  2 ,{0x00,0x00}},

  {0x00,  1 ,{0xd2}}, 
	{0xf5,  2 ,{0x06,0x15}},

  {0x00,  1 ,{0xb4}}, 
	{0xc5,  1 ,{0xcc}},

  {0x00,  1 ,{0x90}}, 
	{0xf5,  4 ,{0x02,0x11,0x02,0x15}},
	
  {0x00,  1 ,{0x90}}, 
	{0xc5,  1 ,{0x50}},

  {0x00,  1 ,{0x94}}, 
	{0xc5,  1 ,{0x66}},

  {0x00,  1 ,{0x80}}, 
	{0xcb,  11 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

  {0x00,  1 ,{0x90}}, 
	{0xcb,  15 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

  {0x00,  1 ,{0xa0}}, 
	{0xcb,  15 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

  {0x00,  1 ,{0xb0}}, 
	{0xcb,  15 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},	
	
  {0x00,  1 ,{0xc0}}, 
	{0xcb,  15 ,{0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x05,0x00,0x00,0x00,0x00}},	
	
  {0x00,  1 ,{0xd0}}, 
	{0xcb,  15 ,{0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05}},	
	
  {0x00,  1 ,{0xe0}}, 
	{0xcb,  14 ,{0x05,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x00}},
	
  {0x00,  1 ,{0xf0}}, 
	{0xcb,  11 ,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},	
	
  {0x00,  1 ,{0x80}}, 
	{0xcc,  15 ,{0x29,0x2a,0x0a,0x0c,0x0e,0x10,0x12,0x14,0x06,0x00,0x08,0x00,0x00,0x00,0x00}},		
	
  {0x00,  1 ,{0x90}}, 
	{0xcc,  15 ,{0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x29,0x2a,0x09,0x0b,0x0d,0x0f,0x11,0x13}},	
	
  {0x00,  1 ,{0xa0}}, 
	{0xcc,  14 ,{0x05,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00}},	
	
  {0x00,  1 ,{0xb0}}, 
	{0xcc,  15 ,{0x29,0x2a,0x13,0x11,0x0f,0x0d,0x0b,0x09,0x01,0x00,0x07,0x00,0x00,0x00,0x00}},	
	
  {0x00,  1 ,{0xc0}}, 
	{0xcc,  15 ,{0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x29,0x2a,0x14,0x12,0x10,0x0e,0x0c,0x0a}},	
	
  {0x00,  1 ,{0xd0}}, 
	{0xcc,  14 ,{0x02,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00}},	
	
  {0x00,  1 ,{0x80}}, 
	{0xce,  12 ,{0x89,0x05,0x10,0x88,0x05,0x10,0x00,0x00,0x00,0x00,0x00,0x00}},		
	
  {0x00,  1 ,{0x90}}, 
	{0xce,  14 ,{0x54,0xfc,0x10,0x54,0xfd,0x10,0x55,0x00,0x10,0x55,0x01,0x10,0x00,0x00}},	
	
  {0x00,  1 ,{0xa0}}, 
	{0xce,  14 ,{0x58,0x07,0x04,0xfc,0x00,0x10,0x00,0x58,0x06,0x04,0xfd,0x00,0x10,0x00}},	
	
  {0x00,  1 ,{0xb0}}, 
	{0xce,  14 ,{0x58,0x05,0x04,0xfe,0x00,0x10,0x00,0x58,0x04,0x04,0xff,0x00,0x10,0x00}},
		
  {0x00,  1 ,{0xc0}}, 
	{0xce,  14 ,{0x58,0x03,0x05,0x00,0x00,0x10,0x00,0x58,0x02,0x05,0x01,0x00,0x10,0x00}},	
	
  {0x00,  1 ,{0xd0}}, 
	{0xce,  14 ,{0x58,0x01,0x05,0x02,0x00,0x10,0x00,0x58,0x00,0x05,0x03,0x00,0x10,0x00}},	
	
  {0x00,  1 ,{0x80}}, 
	{0xcf,  14 ,{0x50,0x00,0x05,0x04,0x00,0x10,0x00,0x50,0x01,0x05,0x05,0x00,0x10,0x00}},		
	
  {0x00,  1 ,{0x90}}, 
	{0xcf,  14 ,{0x50,0x02,0x05,0x06,0x00,0x10,0x00,0x50,0x03,0x05,0x07,0x00,0x10,0x00}},	
	
  {0x00,  1 ,{0xa0}}, 
	{0xcf,  14 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},	
	
  {0x00,  1 ,{0xb0}}, 
	{0xcf,  14 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},	
	
  {0x00,  1 ,{0xc0}}, 
	{0xcf,  11 ,{0x39,0x39,0x20,0x20,0x00,0x00,0x01,0x01,0x20,0x00,0x00}},	
	
  {0x00,  1 ,{0xb5}}, 
	{0xc5,  6 ,{0x0b,0x95,0xff,0x0b,0x95,0xff}},	
	
  {0x00,  1 ,{0x00}}, 
	{0xe1,  20 ,{0x00,0x2E,0x3D,0x49,0x5A,0x66,0x67,0x8E,0x7D,0x93,0x72,0x5E,0x72,0x4F,0x4D,0x41,0x32,0x25,0x19,0x00}},		
	
  {0x00,  1 ,{0x00}}, 
	{0xe2,  20 ,{0x00,0x2E,0x3D,0x49,0x5A,0x67,0x68,0x8F,0x7D,0x93,0x72,0x5F,0x72,0x4F,0x4D,0x41,0x32,0x25,0x19,0x00}},	
	
  {0x00,  1 ,{0x00}}, 
	{0xff,  3 ,{0xff,0xff,0xff}},		
	{0x11,  0 ,{}}, //00
	{REGFLAG_DELAY, 0xc8, {}},
	{0x29,  0 ,{}}, //00
	{REGFLAG_DELAY, 0x0a, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	
};
#endif
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
	data_array[0]=0x00023902;
	data_array[1]=0x00000000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00043902;
	data_array[1]=0x018412ff;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x00008000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00033902;
	data_array[1]=0x008412ff;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x00009200;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00033902;
	data_array[1]=0x000230ff;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x00008000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x000a3902;
	data_array[1]=0x006400c0;
	data_array[2]=0x64001010;
	data_array[3]=0x00001010;
	dsi_set_cmdq(data_array, 4, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00009000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00073902;
	data_array[1]=0x005c00c0;
	data_array[2]=0x00040001;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000b300;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00033902;
	data_array[1]=0x005500c0;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x00008100;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x000055c1;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000d200;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x000003c1;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x000091c1;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x00009ac1;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x00009000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x000000c1;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000a0c1;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x000002c1;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000b600;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x000003c1;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000a000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x000f3902;
	data_array[1]=0x041005c4;
	data_array[2]=0x11150502;
	data_array[3]=0x02071005;
	data_array[4]=0x00111505;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000b000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00033902;
	data_array[1]=0x000000c4;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x00009100;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00033902;
	data_array[1]=0x00d2a6c5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00000000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00033902;
	data_array[1]=0x00c7c7d8;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00000000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x000072d9;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000b300;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x000084c5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000bb00;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00008ac5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00008200;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00000ac4;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000c600;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x000003b0;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000c200;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x000040f5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);		
	data_array[0]=0x00023902;
	data_array[1]=0x0000c300;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x000085f5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00000000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x000040d0;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00000000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00033902;
	data_array[1]=0x000000d1;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000b200;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00033902;
	data_array[1]=0x000000f5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000b400;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00033902;
	data_array[1]=0x000000f5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000b600;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00033902;
	data_array[1]=0x000000f5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000b800;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00033902;
	data_array[1]=0x000000f5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x00009400;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00033902;
	data_array[1]=0x000000f5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000d200;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00033902;
	data_array[1]=0x001506f5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000b400;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000ccc5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00009000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00053902;
	data_array[1]=0x021102f5;
	data_array[2]=0x00000015;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00009000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x000050c5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00009400;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x000066c5;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00008000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000c3902;
	data_array[1]=0x000000cb;
	data_array[2]=0x00000000;
	data_array[3]=0x00000000;
	dsi_set_cmdq(data_array, 4, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00009000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00103902;
	data_array[1]=0x000000cb;
	data_array[2]=0x00000000;
	data_array[3]=0x00000000;
	data_array[4]=0x00000000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000a000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00103902;
	data_array[1]=0x000000cb;
	data_array[2]=0x00000000;
	data_array[3]=0x00000000;
	data_array[4]=0x00000000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000b000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00103902;
	data_array[1]=0x000000cb;
	data_array[2]=0x00000000;
	data_array[3]=0x00000000;
	data_array[4]=0x00000000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000c000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00103902;
	data_array[1]=0x050505cb;
	data_array[2]=0x05050505;
	data_array[3]=0x05000505;
	data_array[4]=0x00000000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000d000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00103902;
	data_array[1]=0x000000cb;
	data_array[2]=0x00000500;
	data_array[3]=0x05050505;
	data_array[4]=0x05050505;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000e000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000f3902;
	data_array[1]=0x050005cb;
	data_array[2]=0x00000000;
	data_array[3]=0x00000000;
	data_array[4]=0x00000005;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000f000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000c3902;
	data_array[1]=0xffffffcb;
	data_array[2]=0xffffffff;
	data_array[3]=0xffffffff;
	dsi_set_cmdq(data_array, 4, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00008000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00103902;
	data_array[1]=0x0a2a29cc;
	data_array[2]=0x12100e0c;
	data_array[3]=0x08000614;
	data_array[4]=0x00000000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00009000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00103902;
	data_array[1]=0x000000cc;
	data_array[2]=0x00000200;
	data_array[3]=0x0b092a29;
	data_array[4]=0x13110f0d;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000a000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000f3902;
	data_array[1]=0x070005cc;
	data_array[2]=0x00000000;
	data_array[3]=0x00000000;
	data_array[4]=0x00000001;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000b000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00103902;
	data_array[1]=0x132a29cc;
	data_array[2]=0x0b0d0f11;
	data_array[3]=0x07000109;
	data_array[4]=0x00000000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000c000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00103902;
	data_array[1]=0x000000cc;
	data_array[2]=0x00000500;
	data_array[3]=0x12142a29;
	data_array[4]=0x0a0c0e10;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000d000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000f3902;
	data_array[1]=0x080002cc;
	data_array[2]=0x00000000;
	data_array[3]=0x00000000;
	data_array[4]=0x00000006;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);		
	data_array[0]=0x00023902;
	data_array[1]=0x00008000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000d3902;
	data_array[1]=0x100589ce;
	data_array[2]=0x00100588;
	data_array[3]=0x00000000;
	data_array[4]=0x00000000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00009000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000f3902;
	data_array[1]=0x10fc54ce;
	data_array[2]=0x5510fd54;
	data_array[3]=0x01551000;
	data_array[4]=0x00000010;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000a000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000f3902;
	data_array[1]=0x040758ce;
	data_array[2]=0x001000fc;
	data_array[3]=0xfd040658;
	data_array[4]=0x00001000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000b000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000f3902;
	data_array[1]=0x040558ce;
	data_array[2]=0x001000fe;
	data_array[3]=0xff040458;
	data_array[4]=0x00001000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000c000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000f3902;
	data_array[1]=0x050358ce;
	data_array[2]=0x00100000;
	data_array[3]=0x01050258;
	data_array[4]=0x00001000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000d000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000f3902;
	data_array[1]=0x050158ce;
	data_array[2]=0x00100002;
	data_array[3]=0x03050058;
	data_array[4]=0x00001000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00008000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000f3902;
	data_array[1]=0x050050cf;
	data_array[2]=0x00100004;
	data_array[3]=0x05050150;
	data_array[4]=0x00001000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00009000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000f3902;
	data_array[1]=0x050250cf;
	data_array[2]=0x00100006;
	data_array[3]=0x07050350;
	data_array[4]=0x00001000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000a000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000f3902;
	data_array[1]=0x000000cf;
	data_array[2]=0x00000000;
	data_array[3]=0x00000000;
	data_array[4]=0x00000000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000b000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x000f3902;
	data_array[1]=0x000000cf;
	data_array[2]=0x00000000;
	data_array[3]=0x00000000;
	data_array[4]=0x00000000;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000c000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);		
	data_array[0]=0x000b3902;
	data_array[1]=0x203939cf;
	data_array[2]=0x01000020;
	data_array[3]=0x00002001;
	dsi_set_cmdq(data_array, 4, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x0000b500;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);		
	data_array[0]=0x00073902;
	data_array[1]=0xff950bc5;
	data_array[2]=0x00ff950b;
	dsi_set_cmdq(data_array,3, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x00000000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);		
	data_array[0]=0x00153902;
	data_array[1]=0x3d2e00e1;
	data_array[2]=0x67665a49;
	data_array[3]=0x72937d8e;
	data_array[4]=0x4d4f725e;
	data_array[5]=0x19253241;
	data_array[6]=0x00000000;
	dsi_set_cmdq(data_array,7, 1);
	MDELAY(1);
	data_array[0]=0x00023902;
	data_array[1]=0x00000000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);		
	data_array[0]=0x00153902;
	data_array[1]=0x3d2e00e2;
	data_array[2]=0x68675a49;
	data_array[3]=0x72937d8f;
	data_array[4]=0x4d4f725f;
	data_array[5]=0x19253241;
	data_array[6]=0x00000000;
	dsi_set_cmdq(data_array,7, 1);
	MDELAY(1);	
	data_array[0]=0x00023902;
	data_array[1]=0x00000000;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	data_array[0]=0x00043902;
	data_array[1]=0xffffffff;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);	
	
	data_array[0]=0x00023902;
	data_array[1]=0x00000036;
	dsi_set_cmdq(data_array, 2, 1);		
	
	data_array[0]=0x00023902;
	data_array[1]=0x00000035;
	dsi_set_cmdq(data_array, 2, 1);	
	 
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


	params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 


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
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
	params->dsi.lcm_esd_check_table[1].cmd		= 0x0e;
	params->dsi.lcm_esd_check_table[1].count	= 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;	
	params->dsi.lcm_esd_check_table[2].cmd		= 0xac;
	params->dsi.lcm_esd_check_table[2].count	= 1;
	params->dsi.lcm_esd_check_table[2].para_list[0] = 0x00;
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
	unsigned int data_array[3];
	data_array[0]= 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(60);

	data_array[0]= 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(100);  
//	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);  
	//MDELAY(10);

//	SET_RESET_PIN(0);
	set_reset_gpio_pin(0);
	MDELAY(10);

	set_vsp_vsn_pin(0);
}

static void lcm_resume(void)
{
	lcm_init(); 
}
         


static unsigned int lcm_compare_id(void)
{
    unsigned char cmd = 0x0;
	unsigned char data = 0x0a;
	unsigned int id = 0;
	int ret=0;
	unsigned char buffer[5];
	unsigned int array[16];

#ifndef CONFIG_FPGA_EARLY_PORTING
	set_vsp_vsn_pin(1);
#endif

	lcm_debug("%s %d\n", __func__,__LINE__);
	set_reset_gpio_pin(0);
	MDELAY(5);
	set_reset_gpio_pin(1);
	MDELAY(125);
	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xA1, buffer, 5);
	id = (buffer[2] << 8) | (buffer[3]) ;
#ifdef BUILD_LK
		printf("%s, buffer[0]=0x%x,buffer[1]=0x%x,buffer[2]=0x%x,buffer[3]=0x%x buffer[4]=0x%x  id = 0x%x\n",__func__,buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],id);
#endif
		return (OTM1284A_HD720_ID == id)?1:0;
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

LCM_DRIVER otm1284a_hd720_dsi_vdo_baolongda_lcm_drv = 
{
	.name			= "otm1284a_hd720_dsi_vdo_baolongda",
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
};
