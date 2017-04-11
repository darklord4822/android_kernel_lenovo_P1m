#define FTS_GESTRUE

#include "ft5x0x_i2c.h"
#include "accdet.h"

#ifdef FTS_GESTRUE
#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		0x30
#define GESTURE_W		0x31
#define GESTURE_M		0x32
#define GESTURE_E		0x33
#define GESTURE_C		0x34
#define GESTURE_S		0x46
#define GESTURE_V		0x54
#define GESTURE_Z		0x41
#define KEY_GESTURE          	KEY_POWER	/* customize gesture-key */

#define FTS_DMA_BUF_SIZE	1024


#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME  62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};
#endif

static char tpgesture_value[10]={};
static char tpgesture_status_value[5] = {};
static char tpgesture_status	= 1;
static int g_call_state 	= 0;

extern struct i2c_client *i2c_client;
extern struct tpd_device *tpd;

//extern void tpgesture_hander(void);
void tpgesture_hander(void)
{
}

static void check_gesture(int gesture_id)
{	
    //printk("kaka gesture_id==0x%x\n ",gesture_id);
    
	switch(gesture_id)
	{
	case GESTURE_DOUBLECLICK:
    		sprintf(tpgesture_value,"DOUBCLICK");
		tpgesture_hander();
		break;

	case GESTURE_UP:
    		sprintf(tpgesture_value,"UP");
		tpgesture_hander();
		break;

	case GESTURE_DOWN:
		sprintf(tpgesture_value,"DOWN");
		tpgesture_hander();
		break;

	case GESTURE_LEFT:
		sprintf(tpgesture_value,"LEFT");
		tpgesture_hander();
		break;

	case GESTURE_RIGHT:
		sprintf(tpgesture_value,"RIGHT");
		tpgesture_hander();
		break;

	case GESTURE_C:
		sprintf(tpgesture_value,"c");
		tpgesture_hander();
		break;

	case GESTURE_O:
		sprintf(tpgesture_value,"o");
		tpgesture_hander();
		break;

	case GESTURE_W:
		sprintf(tpgesture_value,"w");
		tpgesture_hander();
		break;

	case GESTURE_E:
		sprintf(tpgesture_value,"e");
		tpgesture_hander();
		break;

	case GESTURE_V:
		sprintf(tpgesture_value,"v");
		tpgesture_hander();
		break;

	case GESTURE_M:
		sprintf(tpgesture_value,"m");
		tpgesture_hander();
		break;

	case GESTURE_Z:
		sprintf(tpgesture_value,"z");
		tpgesture_hander();
		break;
	case GESTURE_S:
		sprintf(tpgesture_value,"s");
		tpgesture_hander();
		break;
	default:		
		break;
	}
}

static int fetch_object_sample(unsigned char *buf,short pointnum)
{
	return 0;
}

static int ft5x0x_read_Touchdata(struct i2c_client* i2c_client)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 4] = { 0 };
	int ret = -1;
	int i = 0;

	int gestrue_id = 0;
	short pointnum = 0;

//	ret = i2c_smbus_read_i2c_block_data(i2c_client, 0xd3, FTS_GESTRUE_POINTS_HEADER, &(buf[0]));
	buf[0] = 0xd3;
	ret = fts_i2c_Read(i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
	if (ret < 0)
	{
		//printk( "%s read touchdata failed.\n", __func__);
		return ret;
	}

	//printk("%s lsm--buf[0]=%x .\n",__func__,buf[0]);
	if(buf[0] != 0xfe)
	{
// pass dblclick
		gestrue_id =  buf[0];
		check_gesture(gestrue_id);
		return -1;
	}

	pointnum = (short)(buf[1]) & 0xff;
	//printk("%s lsm--pointnum=%d .\n",__func__, pointnum);

	buf[0] = 0xd3;
	ret = fts_i2c_Read(i2c_client, buf, 1, buf, pointnum * 4 + 2+6);
	if (ret < 0)
	{
//flash_on(1);
		//printk( "%s read touchdata failed.\n", __func__);
		return ret;
	}
// pass OK
//flash_on(2);
	gestrue_id = fetch_object_sample(buf, pointnum);
	for(i = 0;i < pointnum;i++)
	{
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
		    8 | (((s16) buf[1 + (4 * i)])& 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
		    8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
//printk( "Gesture touch pint x,y=%d,%d\n", coordinate_x[i], coordinate_y[i]);
	}
	check_gesture(gestrue_id);
	return -1;
}

 static int touch_getsure_event_handler(struct i2c_client* i2c_client)
{
 	u8 state;
	i2c_smbus_read_i2c_block_data(i2c_client, 0xd0, 1, &state);
	//printk("%s lsm--state=%x .\n",__func__,state);
	if(state !=1) return false;

	ft5x0x_read_Touchdata(i2c_client);
	return true;
}

static bool tpd_getsure_suspend(struct i2c_client* i2c_client)
 {
	static char data;

	//printk("[xy-tp]%d\n", tpgesture_status);

	if((g_call_state == CALL_ACTIVE) || (!tpgesture_status)) 
		return false;

	//printk("[xy-tp]:gesture mode\n");
	msleep(200);

	data	= 0x01;
	i2c_smbus_write_i2c_block_data(i2c_client, 0xd0, 1, &data);

	data	= 0xff;
	i2c_smbus_write_i2c_block_data(i2c_client, 0xd1, 1, &data);
	i2c_smbus_write_i2c_block_data(i2c_client, 0xd2, 1, &data);
	i2c_smbus_write_i2c_block_data(i2c_client, 0xd5, 1, &data);
	i2c_smbus_write_i2c_block_data(i2c_client, 0xd6, 1, &data);
	i2c_smbus_write_i2c_block_data(i2c_client, 0xd7, 1, &data);
	i2c_smbus_write_i2c_block_data(i2c_client, 0xd8, 1, &data);

	return true;
 }

static bool tpd_getsure_resume(struct i2c_client* i2c_client)
 {
	static char data = 0x0;
	TPD_DMESG("TPD wake up\n");
	if((g_call_state == CALL_ACTIVE) || (!tpgesture_status))
		return false;

	i2c_smbus_write_i2c_block_data(i2c_client, 0xD0, 1, &data);

	return true;
 }


static ssize_t show_tpgesture_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	 //printk("show tp gesture value is %s \n", tpgesture_value);
	 return sprintf(buf, "%s\n", tpgesture_value);
}
 
static ssize_t show_tpgesture_status_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	 //printk("show tp gesture status is %s\n", tpgesture_status_value);
	 return sprintf(buf, "%s\n", tpgesture_status_value);
}
 
static ssize_t store_tpgesture_status_value(struct device* dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	 if(!strncmp(buf, "on", 2))
	 {
		 sprintf(tpgesture_status_value,"on");
		 tpgesture_status = 1;//status --- on
	 }
	 else
	 {
		 sprintf(tpgesture_status_value,"off");
		 tpgesture_status = 0;//status --- off
	 }
 
	 return count;
}

static DEVICE_ATTR(tpgesture, 0664, show_tpgesture_value, NULL);
//	0666
static DEVICE_ATTR(tpgesture_status, 0664, show_tpgesture_status_value, store_tpgesture_status_value);


