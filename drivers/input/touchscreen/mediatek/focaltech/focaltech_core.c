/*
 *
 * FocalTech TouchScreen driver.
 * 
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
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


/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/dma-mapping.h>
#include "focaltech_core.h"
#include <linux/of.h>
#include <linux/of_irq.h>
unsigned int TPD_EINT_TOUCH_PANEL_NUM;
/*******************************************************************************
* 2.Private constant and macro definitions using #define
*******************************************************************************/
#define FTS_CTL_IIC
#define SYSFS_DEBUG
#define FTS_APK_DEBUG

#define GPIO_TP_AVDD_EN 	(GPIO21 | 0x80000000)    //GPIO21   ->TP_AVDD_EN
#define GPIO_CTP_EINT_PIN         (GPIO62 | 0x80000000)
#define GPIO_CTP_RST_PIN         (GPIO10 | 0x80000000)
#define GPIO_CTP_RST_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_CTP_EINT_PIN_M_EINT   GPIO_MODE_00

#define GTP_RST_PORT    0
#define GTP_INT_PORT    1
#define CONFIG_FOCALTECH_POWER_BY_PMIC

/*
for tp esd check
*/
#if FT_ESD_PROTECT
    #define TPD_ESD_CHECK_CIRCLE                                200
    static struct delayed_work gtp_esd_check_work;
    static struct workqueue_struct *gtp_esd_check_workqueue     = NULL;
    static int count_irq                                        = 0;
    static u8 run_check_91_register                             = 0;
    static unsigned long esd_check_circle                       = TPD_ESD_CHECK_CIRCLE;
    static void gtp_esd_check_func(struct work_struct *);
#endif

//#if FT_ESD_PROTECT
int apk_debug_flag=0;
//#endif
#ifdef TPD_PROXIMITY
    #include <linux/hwmsensor.h>
    #include <linux/hwmsen_dev.h>
    #include <linux/sensors_io.h>
#endif

#ifdef TPD_PROXIMITY
#define FT_PROXIMITY_ENABLE             1
#define APS_ERR(fmt,arg...)             printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
static u8 tpd_proximity_flag            = 0;
static u8 tpd_proximity_flag_one        = 0;
//0-->close ; 1--> far away
static u8 tpd_proximity_detect      = 1;
#endif

/*dma declare, allocate and release*/
#define __MSG_DMA_MODE__
#ifdef __MSG_DMA_MODE__
    u8 *g_dma_buff_va = NULL;
    dma_addr_t g_dma_buff_pa = 0;
#endif

#ifdef __MSG_DMA_MODE__

static void msg_dma_alloct(struct device *dev)
{
    g_dma_buff_va = (u8 *)dma_alloc_coherent(dev, 128, &g_dma_buff_pa, GFP_KERNEL);//DMA size 4096 for customer

    if(!g_dma_buff_va) {
        FTS_ERROR("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
    }
}

static void msg_dma_release(struct device *dev)
{
    if(g_dma_buff_va) {
        dma_free_coherent(dev, 128, g_dma_buff_va, g_dma_buff_pa);
        g_dma_buff_va = NULL;
        g_dma_buff_pa = 0;
        FTS_ERROR("[DMA][release] Allocate DMA I2C Buffer release!\n");
    }
}
#endif

#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif
/*******************************************************************************
* 3.Private enumerations, structures and unions using typedef
*******************************************************************************/
/*register driver and device info*/
static const struct i2c_device_id fts_tpd_id[] = {{"fts", 0}, {}};
//static struct i2c_board_info __initdata fts_i2c_tpd = {I2C_BOARD_INFO("fts", (0x70 >> 1))};

/*******************************************************************************
* 4.Static variables
*******************************************************************************/
struct i2c_client *fts_i2c_client = NULL;
struct input_dev *fts_input_dev = NULL;
struct task_struct *focaltechthread = NULL;
static int tpd_flag = 0;
static int tpd_halt = 0;
#if FTS_GESTRUE_EN
static bool gesture_enabled = false;
#endif
bool is_update = false;
/*******************************************************************************
* 5.Global variable or extern global variabls/functions
*******************************************************************************/
#ifdef FTS_USB_SWITCH
extern kal_bool upmu_is_chr_det(void);
extern void (*ctp_charge_mode_enable)(void);
void charge_mode_enable(void);
static struct work_struct set_charge_mode_work;
static void set_charge_mode_work_func(struct work_struct *work);
static int need_restore = -1;
static int usb_cable_state = 0;
#endif
#if FTS_GESTRUE_EN
int gesture_wakeup_flag;
#endif


/*******************************************************************************
* 6.Static function prototypes
*******************************************************************************/
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(i2c_rw_access);
//static void tpd_eint_interrupt_handler(void);
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *desc);
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
//extern void mt_eint_mask(unsigned int eint_num);
//extern void mt_eint_unmask(unsigned int eint_num);
//extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
//extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
//extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
//extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

static struct of_device_id tpd_of_match[] = {
	{ .compatible = "mediatek,CAP_TOUCH",},
	{ },
};


static struct i2c_driver tpd_i2c_driver = {
    .driver = {
        .name = "fts",
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = tpd_of_match,
#endif
    },
    .probe = tpd_probe,
    .remove = __devexit_p(tpd_remove),
    .id_table = fts_tpd_id,
    .detect = tpd_detect,
};



/*
*   open/release/(I/O) control tpd device
*
*/
//#define VELOCITY_CUSTOM_fts
#ifdef VELOCITY_CUSTOM_fts
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

/*for magnify velocity*/
#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X               10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y               10
#endif

#define TOUCH_IOC_MAGIC                     'A'
#define TPD_GET_VELOCITY_CUSTOM_X           _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y           _IO(TOUCH_IOC_MAGIC,1)

int g_v_magnify_x = TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y = TPD_VELOCITY_CUSTOM_Y;

/************************************************************************
* Name: tpd_misc_open
* Brief: open node
* Input: node, file point
* Output: no
* Return: fail <0
***********************************************************************/
static int tpd_misc_open(struct inode *inode, struct file *file)
{
    return nonseekable_open(inode, file);
}

/************************************************************************
* Name: tpd_misc_release
* Brief: release node
* Input: node, file point
* Output: no
* Return: 0
***********************************************************************/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
    return 0;
}

/************************************************************************
* Name: tpd_unlocked_ioctl
* Brief: I/O control for apk
* Input: file point, command
* Output: no
* Return: fail <0
***********************************************************************/
static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
                               unsigned long arg)
{
    void __user *data;
    long err = 0;

    if(_IOC_DIR(cmd) & _IOC_READ) {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));

    } else if(_IOC_DIR(cmd) & _IOC_WRITE) {
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }

    if(err) {
        FTS_ERROR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch(cmd) {
    case TPD_GET_VELOCITY_CUSTOM_X:
        data = (void __user *) arg;

        if(data == NULL) {
            err = -EINVAL;
            break;
        }

        if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x))) {
            err = -EFAULT;
            break;
        }

        break;

    case TPD_GET_VELOCITY_CUSTOM_Y:
        data = (void __user *) arg;

        if(data == NULL) {
            err = -EINVAL;
            break;
        }

        if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y))) {
            err = -EFAULT;
            break;
        }

        break;

    default:
        printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
        err = -ENOIOCTLCMD;
        break;
    }

    return err;
}

static struct file_operations tpd_fops = {
    //.owner = THIS_MODULE,
    .open = tpd_misc_open,
    .release = tpd_misc_release,
    .unlocked_ioctl = tpd_unlocked_ioctl,
};

static struct miscdevice tpd_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "touch",
    .fops = &tpd_fops,
};
#endif

#ifdef FTS_USB_SWITCH
static void set_charge_mode_work_func(struct work_struct *work)
{
    int ret = -1;

    if (upmu_is_chr_det() == KAL_TRUE) {
        FTS_INFO("USB cable plug in!\n");

        if (tpd_halt == 1) {
            need_restore = 1;

        } else {
            ret = fts_write_reg(fts_i2c_client, 0x8B, 1);
            usb_cable_state = 1;

            if (ret < 0)
                FTS_ERROR("Write reg(0x8B) failed!\n");
        }

    } else {
        FTS_INFO("USB cabel plug out!\n");

        if (tpd_halt == 1) {
            need_restore = 0;

        } else {
            ret = fts_write_reg(fts_i2c_client, 0x8B, 0);
            usb_cable_state = 0;

            if (ret < 0)
                FTS_ERROR("Write reg(0x8B) failed!\n");
        }
    }
}

void charge_mode_enable(void)
{
    schedule_work(&set_charge_mode_work);
}
#endif

int fts_i2c_read_testa(/*struct i2c_client *client,*/ unsigned char *writebuf,int writelen, unsigned char *readbuf, int readlen)
{
    return fts_i2c_read(fts_i2c_client, writebuf, writelen, readbuf, readlen);
}

int fts_i2c_write_testa(/*struct i2c_client *client,*/ unsigned char *writebuf, int writelen)
{
    return fts_i2c_write(fts_i2c_client, writebuf, writelen);
}

/************************************************************************
* Name: fts_i2c_read
* Brief: i2c read
* Input: i2c info, write buf, write len, read buf, read len
* Output: get data in the 3rd buf
* Return: fail <0
***********************************************************************/
int fts_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
    int ret = -1;

    //for DMA I2c transfer
    
    mutex_lock(&i2c_rw_access);
    
    if((NULL!=client) && (writelen>0) && (writelen<=128))
    {
        // DMA Write
        if(g_dma_buff_va==NULL){
            FTS_ERROR("%s:g_dma_buff_va==NULL\n", __func__);
            return -1;
            }
        memcpy(g_dma_buff_va, writebuf, writelen);
        client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;

        if((ret = i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen)) != writelen)
            FTS_ERROR("%s:i2c write failed!\n", __func__);

        client->addr = (client->addr & I2C_MASK_FLAG) & (~ I2C_DMA_FLAG);
    }

    // DMA Read 
    if((NULL!=client) && (readlen>0) && (readlen<=128))
    {
        client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
        ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa, readlen);
        memcpy(readbuf, g_dma_buff_va, readlen);
        client->addr = (client->addr & I2C_MASK_FLAG) & (~ I2C_DMA_FLAG);
    }

    mutex_unlock(&i2c_rw_access);
    return ret;
}

/************************************************************************
* Name: fts_i2c_write
* Brief: i2c write
* Input: i2c info, write buf, write len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
    int ret = -1;
    mutex_lock(&i2c_rw_access);
    
    if((NULL!=client) && (writelen>0) && (writelen<=128))
    {
        memcpy(g_dma_buff_va, writebuf, writelen);
        client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;

        if((ret = i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen)) != writelen)
        {
            FTS_ERROR("%s:i2c write failed!\n", __func__);
        }

        client->addr = (client->addr & I2C_MASK_FLAG) & (~ I2C_DMA_FLAG);
    }
    mutex_unlock(&i2c_rw_access);
    return ret;
}

/************************************************************************
* Name: fts_write_reg
* Brief: write register
* Input: i2c info, reg address, reg value
* Output: no
* Return: fail <0
***********************************************************************/
int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
    unsigned char buf[2] = {0};
    buf[0] = regaddr;
    buf[1] = regvalue;
    return fts_i2c_write(client, buf, sizeof(buf));
}

/************************************************************************
* Name: fts_read_reg
* Brief: read register
* Input: i2c info, reg address, reg value
* Output: get reg value
* Return: fail <0
***********************************************************************/
int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
    return fts_i2c_read(client, &regaddr, 1, regvalue, 1);
}

/************************************************************************
* Name: tpd_down
* Brief: down info
* Input: x pos, y pos, id number
* Output: no
* Return: no
***********************************************************************/
static void tpd_down(int x, int y, int p,int id)
{
    //FTS_DBG("%s:[%4d %4d %4d]\n", __func__, x, y, p);
    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 200);
    input_report_abs(tpd->dev, ABS_MT_PRESSURE, p);
    input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id); 
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);
#if 0
    if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
    {   
        tpd_button(x, y, 1);  
    }
#endif
}

/************************************************************************
* Name: tpd_up
* Brief: up info
* Input: x pos, y pos, count
* Output: no
* Return: no
***********************************************************************/
static void tpd_up(int x, int y, int p)
{
    //FTS_DBG("%s:[%4d %4d %4d]\n", __func__, x, y, p);
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_mt_sync(tpd->dev);
#if 0
    if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
    {   
        tpd_button(x, y, 0);
    }
#endif
}

/************************************************************************
* Name: fts_read_Touchdata
* Brief: report the point information
* Input: event info
* Output: get touch data in pinfo
* Return: success is zero
***********************************************************************/
int fts_palm_buf=0;
int fts_palm_flag=0;
static int tpd_touchinfo(struct ts_event *data)
{
  	u8 buf[POINT_READ_BUF] = { 0 };
    int ret = -1;
    int i = 0;
    u8 pointid = FTS_MAX_ID;

    if (tpd_halt) {
        FTS_DBG("%s return ..\n", __func__);
        return false;
    }

    mutex_lock(&i2c_access);
    ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, (3 + FTS_TOUCH_STEP * fts_updateinfo_curr.TPD_MAX_POINTS));
    mutex_unlock(&i2c_access);

    if (ret < 0) {
        FTS_ERROR("Read touchdata failed!\n");
        return false;
    }
#if 0
    for(i=0;i<fts_updateinfo_curr.TPD_MAX_POINTS;i++)
    {
        FTS_DBG(&fts_i2c_client->dev,"\n [fts]buf[%d] =(0x%02x)  \n", i,buf[i]);
    }
#endif
    memset(data, 0, sizeof(struct ts_event));
    data->touch_point = 0;  
    data->touch_point_num=buf[FT_TOUCH_POINT_NUM] & 0x0F;

    //printk("tpd  fts_updateinfo_curr.TPD_MAX_POINTS=%d fts_updateinfo_curr.chihID=%d \n", fts_updateinfo_curr.TPD_MAX_POINTS,fts_updateinfo_curr.CHIP_ID);
    for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++)
    {
        pointid = (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;
        if (pointid >= FTS_MAX_ID)
            break;
        else
            data->touch_point++;

        data->au16_x[i] = (s16)(buf[FTS_TOUCH_X_H_POS + FTS_TOUCH_STEP * i] & 0x0F) << 8 
                        | (s16) buf[FTS_TOUCH_X_L_POS + FTS_TOUCH_STEP * i];
        data->au16_y[i] = (s16)(buf[FTS_TOUCH_Y_H_POS + FTS_TOUCH_STEP * i] & 0x0F) << 8 
                        | (s16) buf[FTS_TOUCH_Y_L_POS + FTS_TOUCH_STEP * i];
        data->au8_touch_event[i] = buf[FTS_TOUCH_EVENT_POS + FTS_TOUCH_STEP * i] >> 6;
        data->au8_finger_id[i]   = buf[FTS_TOUCH_ID_POS    + FTS_TOUCH_STEP * i] >> 4;
        data->pressure[i]        = buf[FTS_TOUCH_XY_POS    + FTS_TOUCH_STEP * i];
        data->area[i]            = buf[FTS_TOUCH_MISC      + FTS_TOUCH_STEP * i];
        /* add by wuyujing, 800 is specific for palm */
        if (data->au16_x[i] == 800)
            fts_palm_flag=0;
    }

	return true;
}

/************************************************************************
* Name: fts_report_value
* Brief: report the point information
* Input: event info
* Output: no
* Return: success is zero
***********************************************************************/
static void fts_report_value(struct ts_event *data)
{
    int i = 0;
    int up_point = 0;
	
#ifdef MT_PROTOCOL_B    
    for (i = 0; i < data->touch_point; i++) 
    {
         input_mt_slot(tpd->dev, data->au8_finger_id[i]);
 
        if (data->au8_touch_event[i]== 0 || data->au8_touch_event[i] == 2)
        {
            input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,true);
            input_report_abs(tpd->dev, ABS_MT_PRESSURE,data->area[i]/*data->pressure[i]*/);
            input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,200/*data->area[i]*/);
            input_report_abs(tpd->dev, ABS_MT_POSITION_X,data->au16_x[i]);
            input_report_abs(tpd->dev, ABS_MT_POSITION_Y,data->au16_y[i]);

            if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
            {   
                tpd_button(data->au16_x[i], data->au16_y[i], 1);
            }
        }
        else
        {
            up_point++;
            input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,false);
        }                
    }
         
    if(data->touch_point == up_point)
    {
        input_report_key(tpd->dev, BTN_TOUCH, 0);
        if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
        {   
            tpd_button(data->au16_x[0], data->au16_y[0], 0);
        }
    }
    else
        input_report_key(tpd->dev, BTN_TOUCH, 1);
    
#else // MT_PROTOCOL_B
    if(fts_palm_flag == 0){
        fts_palm_flag = 1;
	pr_err("report palm_flag = %x \n",data->palm_flag);
	input_report_key(tpd->dev, KEY_PALM_LOCK, 1);
	input_sync(tpd->dev);
	input_report_key(tpd->dev, KEY_PALM_LOCK, 0);
	input_sync(tpd->dev);
#if 0
        input_report_key(tpd->dev, BTN_TOUCH, 1);
        input_report_abs(tpd->dev,ABS_MT_POSITION_X,100);
        input_report_abs(tpd->dev,ABS_MT_POSITION_Y,0);
        input_report_abs(tpd->dev, ABS_MT_PRESSURE, 1000);
        input_mt_sync(tpd->dev);
        input_sync(tpd->dev);
        input_report_key(tpd->dev, BTN_TOUCH, 0);
        input_mt_sync(tpd->dev);
        input_sync(tpd->dev);
#endif
    }else {
        for (i = 0; i < data->touch_point; i++)
        {
            if (data->au8_touch_event[i]== 0 || data->au8_touch_event[i] == 2)
            {
                tpd_down(data->au16_x[i], data->au16_y[i], data->area[i],data->au8_finger_id[i]);
            }
            else
            {
                up_point++;
                if ( data->touch_point == up_point )
                {
                    tpd_up(data->au16_x[0], data->au16_y[0], data->area[0]);
                }
            }
        }
    }
#endif//MT_PROTOCOL_B         
    input_sync(tpd->dev);
}
#ifdef TPD_PROXIMITY
/************************************************************************
* Name: tpd_read_ps
* Brief: read proximity value
* Input: no
* Output: no
* Return: 0
***********************************************************************/
int tpd_read_ps(void)
{
    tpd_proximity_detect;
    return 0;
}

/************************************************************************
* Name: tpd_get_ps_value
* Brief: get proximity value
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static int tpd_get_ps_value(void)
{
    return tpd_proximity_detect;
}

/************************************************************************
* Name: tpd_enable_ps
* Brief: enable proximity
* Input: enable or not
* Output: no
* Return: 0
***********************************************************************/
static int tpd_enable_ps(int enable)
{
    u8 state;
    int ret = -1;
    
    //i2c_smbus_read_i2c_block_data(fts_i2c_client, 0xB0, 1, &state);
    ret = fts_read_reg(fts_i2c_client, 0xB0, &state);

    if (ret<0) 
    {
        TPD_PROXIMITY_DEBUG("[proxi_fts] read value fail");
    }
    
    TPD_PROXIMITY_DEBUG("[proxi_fts]%s:read 0xb0's value is 0x%02X\n", __func__, state);

    if (enable)
    {
        state |= 0x01;
        tpd_proximity_flag = 1;
        TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is on\n");
    }
    else
    {
        state &= 0x00;
        tpd_proximity_flag = 0;
        TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is off\n");
    }

    //ret = i2c_smbus_write_i2c_block_data(fts_i2c_client, 0xB0, 1, &state);
    ret = fts_write_reg(fts_i2c_client, 0xB0, state);

    if (ret<0) 
    {
        TPD_PROXIMITY_DEBUG("[proxi_fts] write value fail");
    }

    TPD_PROXIMITY_DEBUG("[proxi_fts]write: 0xB0's value is 0x%02X\n", state);
    return 0;
}

/************************************************************************
* Name: tpd_ps_operate
* Brief: operate function for proximity
* Input: point, which operation, buf_in , buf_in len, buf_out , buf_out len, no use
* Output: buf_out
* Return: fail <0
***********************************************************************/
int tpd_ps_operate(void *self, uint32_t command, void *buff_in, int size_in,
                   void *buff_out, int size_out, int *actualout)
{
    int err = 0;
    int value;
    hwm_sensor_data *sensor_data;
    TPD_PROXIMITY_DEBUG("[proxi_fts]command = 0x%02X\n", command);

    switch (command) {
    case SENSOR_DELAY:
        if((buff_in == NULL) || (size_in < sizeof(int))) {
            APS_ERR("Set delay parameter error!\n");
            err = -EINVAL;
        }

        // Do nothing
        break;

    case SENSOR_ENABLE:
        if((buff_in == NULL) || (size_in < sizeof(int))) {
            APS_ERR("Enable sensor parameter error!\n");
            err = -EINVAL;

        } else {
            value = *(int *)buff_in;

            if(value) {
                if((tpd_enable_ps(1) != 0)) {
                    APS_ERR("enable ps fail: %d\n", err);
                    return -1;
                }

            } else {
                if((tpd_enable_ps(0) != 0)) {
                    APS_ERR("disable ps fail: %d\n", err);
                    return -1;
                }
            }
        }

        break;

    case SENSOR_GET_DATA:
        if((buff_out == NULL) || (size_out < sizeof(hwm_sensor_data))) {
            APS_ERR("get sensor data parameter error!\n");
            err = -EINVAL;

        } else {
            sensor_data = (hwm_sensor_data *)buff_out;

            if((err = tpd_read_ps())) {
                err = -1;;

            } else {
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

#if FT_ESD_PROTECT
 void esd_switch(s32 on)
 {
     if (1 == on) // switch on esd 
     {
         queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, esd_check_circle);
     }
     else // switch off esd
     {
         cancel_delayed_work(&gtp_esd_check_work);
     }
 }

 /************************************************************************
 * Name: force_reset_guitar
 * Brief: reset
 * Input: no
 * Output: no
 * Return: 0
 ***********************************************************************/
 static void force_reset_guitar(void)
 {
     s32 i;
     s32 ret;
     
     //mt_eint_mask(TPD_EINT_TOUCH_PANEL_NUM);    // disable interrupt 
     disable_irq(TPD_EINT_TOUCH_PANEL_NUM);
     mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
     mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
     mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
     msleep(10);
     FTS_INFO("%s\n", __func__);
#ifdef CONFIG_FOCALTECH_POWER_BY_PMIC
     regulator_disable(tpd->reg);
#else
     //hwPowerDown(MT6323_POWER_LDO_VGP1,  "TP");
     mt_set_gpio_mode(GPIO_TP_AVDD_EN, GPIO_MODE_00);
     mt_set_gpio_dir(GPIO_TP_AVDD_EN, GPIO_DIR_OUT);
     mt_set_gpio_out(GPIO_TP_AVDD_EN, GPIO_OUT_ZERO);
#endif
     msleep(200);
#ifdef CONFIG_FOCALTECH_POWER_BY_PMIC
     regulator_enable(tpd->reg);
#else
     //hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_2800, "TP");
    mt_set_gpio_mode(GPIO_TP_AVDD_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_TP_AVDD_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_TP_AVDD_EN, GPIO_OUT_ONE);

#endif
     //msleep(5);
 
     msleep(10);
     FTS_INFO("fts ic reset\n");
     mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
     mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
     mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
 
     mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
     mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
     mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
     mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
 
     msleep(300);
     
#ifdef TPD_PROXIMITY
     if (FT_PROXIMITY_ENABLE == tpd_proximity_flag) 
     {
         tpd_enable_ps(FT_PROXIMITY_ENABLE);
     }
#endif
     //mt_eint_unmask(TPD_EINT_TOUCH_PANEL_NUM);  // enable interrupt
     enable_irq(TPD_EINT_TOUCH_PANEL_NUM);
 }
 
  
#define A3_REG_VALUE                                0x54
#define RESET_91_REGVALUE_SAMECOUNT                 5
 static u8 g_old_91_Reg_Value                        = 0x00;
 static u8 g_first_read_91                           = 0x01;
 static u8 g_91value_same_count                      = 0;
 /************************************************************************
 * Name: gtp_esd_check_func
 * Brief: esd check function
 * Input: struct work_struct
 * Output: no
 * Return: 0
 ***********************************************************************/
 static void gtp_esd_check_func(struct work_struct *work)
 {
     int i;
     int ret = -1;
     u8 data, data_old;
     u8 flag_error = 0;
     int reset_flag = 0;
     u8 check_91_reg_flag = 0;
 
     if (tpd_halt ) {
         return;
     }
 //printk("\n zax 0 enter apk_debug_flag=%d \n",apk_debug_flag);
     if(is_update)
     {
         return;
     }
     //printk("\n zax 1 enter apk_debug_flag=%d \n",apk_debug_flag);
     if(apk_debug_flag) {
         //queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, esd_check_circle);
         FTS_INFO("Enter apk debug mode!\n");
         return;
     }
 
     run_check_91_register = 0;
 
     for (i = 0; i < 3; i++) {
         //ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0xA3, 1, &data);
         ret = fts_read_reg(fts_i2c_client, 0xA3, &data);
 
         if (ret < 0) {
             FTS_ERROR("Read value of 0xA3 failed!");
             //return ret;
         }
 
         if (ret == 1 && A3_REG_VALUE == data) {
             break;
         }
     }
 
     if (i >= 3) {
        force_reset_guitar();
        FTS_ERROR("i >= 3  ret = %d	A3_Reg_Value = 0x%02x\n ", ret, data);
        reset_flag = 1;
        goto FOCAL_RESET_A3_REGISTER;
     }
 
     //esd check for count
     //ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x8F, 1, &data);
     ret = fts_read_reg(fts_i2c_client, 0x8F, &data);
 
     if (ret < 0) {
         FTS_ERROR("Read value of 0x8F failed!");
         //return ret;
     }
 
     FTS_DBG("0x8F:%d, count_irq is %d\n", data, count_irq);
     flag_error = 0;
 
     if((count_irq - data) > 10) {
         if((data + 200) > (count_irq + 10) ) {
             flag_error = 1;
         }
     }
 
     if((data - count_irq ) > 10) {
         flag_error = 1;
     }
 
     if(1 == flag_error) {
        FTS_DBG("1 == flag_error...data=%d	count_irq = %d\n ", data, count_irq);
        force_reset_guitar();
        reset_flag = 1;
        goto FOCAL_RESET_INT;
     }
 
     run_check_91_register = 1;
     //ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x91, 1, &data);
     ret = fts_read_reg(fts_i2c_client, 0x91, &data);
 
     if (ret < 0) {
         FTS_ERROR("Read value of 0x91 failed!");
         //return ret;
     }
 
     FTS_DBG("0x91 register value = 0x%02x, old value = 0x%02x\n", data, g_old_91_Reg_Value);
 
     if(0x01 == g_first_read_91) {
         g_old_91_Reg_Value = data;
         g_first_read_91 = 0x00;
 
     } else {
         if(g_old_91_Reg_Value == data) {
             g_91value_same_count++;
             FTS_DBG("g_91value_same_count = %d\n", g_91value_same_count);
 
             if(RESET_91_REGVALUE_SAMECOUNT == g_91value_same_count) {
                 FTS_ERROR("g_91value_same_count = %d\n", RESET_91_REGVALUE_SAMECOUNT);
                 force_reset_guitar();
                 g_91value_same_count = 0;
                 reset_flag = 1;
             }
 
             //run_check_91_register = 1;
             esd_check_circle = TPD_ESD_CHECK_CIRCLE / 2;
             g_old_91_Reg_Value = data;
 
         } else {
             g_old_91_Reg_Value = data;
             g_91value_same_count = 0;
             //run_check_91_register = 0;
             esd_check_circle = TPD_ESD_CHECK_CIRCLE;
         }
     }
 
 FOCAL_RESET_INT:
 FOCAL_RESET_A3_REGISTER:
     count_irq = 0;
     data = 0;
     //fts_i2c_smbus_write_i2c_block_data(i2c_client, 0x8F, 1, &data);
     ret = fts_write_reg(fts_i2c_client, 0x8F, data);
 
     if (ret < 0) {
         FTS_ERROR("Write value of 0x8F failed!\n");
         //return ret;
     }
 
     if(0 == run_check_91_register) {
         g_91value_same_count = 0;
     }
 
#ifdef TPD_PROXIMITY
 
     if( (1 == reset_flag) && ( FT_PROXIMITY_ENABLE == tpd_proximity_flag) ) {
         if((tpd_enable_ps(FT_PROXIMITY_ENABLE) != 0)) {
             APS_ERR("enable ps fail\n");
             return -1;
         }
     }
 
#endif
     //end esd check for count
 
     if (!tpd_halt) {
         //queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
         queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, esd_check_circle);
     }
 
     return;
 }
#endif

/************************************************************************
* Name: touch_event_handler
* Brief: interrupt event from TP, and read/report data to Android system
* Input: no use
* Output: no
* Return: 0
***********************************************************************/
static int touch_event_handler(void *unused)
{
    struct ts_event pevent;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
#if FTS_GESTRUE_EN
    u8 state;
    int ret = 0;
#endif
#ifdef TPD_PROXIMITY
    int err;
    hwm_sensor_data sensor_data;
    u8 proximity_status;
#endif
    sched_setscheduler(current, SCHED_RR, &param);

    do {
        //mt_eint_unmask(TPD_EINT_TOUCH_PANEL_NUM);
        enable_irq(TPD_EINT_TOUCH_PANEL_NUM);
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, tpd_flag != 0);
                         
        tpd_flag = 0;
             
        set_current_state(TASK_RUNNING);
#if FTS_GESTRUE_EN
        if (gesture_wakeup_flag && gesture_enabled) 
		{
            //i2c_smbus_read_i2c_block_data(fts_i2c_client, 0xd0, 1, &state);
            ret = fts_read_reg(fts_i2c_client, 0xd0, &state);

            if (ret < 0) {
                FTS_ERROR("Read  state of gesture value failed!\n");
                //return ret;
            }

            FTS_ERROR("read Gestrue data state = %d\n", state);

            if(state == 1) {
                fts_read_Gestruedata();
                continue;
            }
        }
#endif
#ifdef TPD_PROXIMITY
        if (tpd_proximity_flag == 1) 
		{
            //i2c_smbus_read_i2c_block_data(fts_i2c_client, 0xB0, 1, &state);
            ret = fts_read_reg(fts_i2c_client, 0xB0, &state);

            if (ret < 0) {
                APS_ERR("[Focal][Touch] read value fail");
                //return ret;
            }

            TPD_PROXIMITY_DEBUG("proxi_fts 0xB0 state value is 1131 0x%02X\n", state);

            if(!(state & 0x01)) {
                tpd_enable_ps(1);
            }

            //i2c_smbus_read_i2c_block_data(fts_i2c_client, 0x01, 1, &proximity_status);
            ret = fts_read_reg(fts_i2c_client, 0x01, &proximity_status);

            if (ret < 0) {
                APS_ERR("[Focal][Touch] read value fail");
                //return ret;
            }

            TPD_PROXIMITY_DEBUG("proxi_fts 0x01 value is 1139 0x%02X\n", proximity_status);

            if (proximity_status == 0xC0) {
                tpd_proximity_detect = 0;

            } else if(proximity_status == 0xE0) {
                tpd_proximity_detect = 1;
            }

            TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);

            if ((err = tpd_read_ps())) {
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
#if FT_ESD_PROTECT
        esd_switch(0);apk_debug_flag = 1;
#endif
        if (tpd_touchinfo(&pevent)) 
        {
       		//FTS_DBG("point_num = %d\n",pevent.touch_point_num);
            TPD_DEBUG_SET_TIME;
            fts_report_value(&pevent);
        }

#if FT_ESD_PROTECT
        esd_switch(1);apk_debug_flag = 0;
#endif
    } while (!kthread_should_stop());

    return 0;
}
/************************************************************************
* Name: fts_reset_tp
* Brief: reset TP
* Input: pull low or high
* Output: no
* Return: 0
***********************************************************************/
void fts_reset_tp(int HighOrLow)
{
    
    if(HighOrLow)
        tpd_gpio_output(GTP_RST_PORT, 1);
    else
        tpd_gpio_output(GTP_RST_PORT, 0);    
}

/************************************************************************
* Name: tpd_detect
* Brief: copy device name
* Input: i2c info, board info
* Output: no
* Return: 0
***********************************************************************/
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, TPD_DEVICE);
    return 0;
}

/************************************************************************
* Name: tpd_eint_interrupt_handler
* Brief: deal with the interrupt event
* Input: no
* Output: no
* Return: no
***********************************************************************/
#if 0
static void tpd_eint_interrupt_handler(void)
{
    FTS_DBG("TPD interrupt has been triggered\n");
    disable_irq_nosync(TPD_EINT_TOUCH_PANEL_NUM);
    TPD_DEBUG_PRINT_INT;
    tpd_flag = 1;
#if FT_ESD_PROTECT
    count_irq ++;
#endif
    wake_up_interruptible(&waiter);
}
#endif
static irqreturn_t tpd_eint_interrupt_handler(int  irq, void *desc)
{
    TPD_DEBUG_PRINT_INT;
    tpd_flag = 1;
#if FT_ESD_PROTECT
    count_irq ++;
#endif
    disable_irq_nosync(TPD_EINT_TOUCH_PANEL_NUM);
    wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}

/************************************************************************
* Name: fts_init_gpio_hw
* Brief: initial gpio
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static void fts_init_gpio_hw(void)
{
    tpd_gpio_output(GTP_RST_PORT, 1);
}

/************************************************************************
* Name: device_parse_platform
* Brief: parse platform device
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static void device_parse_platform(void)
{
	unsigned int gpiopin, debounce;
	u32 ints[2] = {0, 0};
	int irq;
	struct device_node *node=NULL;
#if 0	
    //printk("add by sbli device_parse_platform\n");
    
	/* get gpio pin & debounce time */
	/*
	* kernel standard uses pin control to setup gpio
	* This example doesn't include pin control part.
	*/
	node = of_find_compatible_node(NULL, NULL, "mediatek, TOUCH_PANEL-eint");

	if(node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpiopin = ints[GPIOPIN];
		debounce = ints[DEBOUNCE];
		printk(KERN_ERR "%s, gpiopin=%d, debounce=%d microsecond\n",
			__func__, gpiopin, debounce);
		/* get irq # */
		
		irq = irq_of_parse_and_map(node, 0);
		if(!irq) {
			printk("can't irq_of_parse_and_map for abc!!\n");
			return -EINVAL;
		}
        printk("%s sbli irq=%d\n",__func__,irq);
		/* set debounce (optional) */
		mt_gpio_set_debounce(gpiopin, debounce);
		/* request irq for eint (either way) */
		TPD_EINT_TOUCH_PANEL_NUM=irq;
	}
#endif
	pr_err("%s\n", __func__);
	node = of_find_matching_node(node, touch_of_match);
	if(node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpiopin = ints[0];
		debounce = ints[1];
		printk(KERN_ERR "%s, gpiopin=%d, debounce=%d microsecond\n",
			__func__, gpiopin, debounce);
		/* get irq # */
		irq = irq_of_parse_and_map(node, 0);
		if(!irq) {
			printk("can't irq_of_parse_and_map for abc!!\n");
			return ;
		}
		/* set debounce (optional) */
		gpio_set_debounce(gpiopin, debounce);
		/* request irq for eint (either way) */
		TPD_EINT_TOUCH_PANEL_NUM = irq;
	}
}

/************************************************************************
* Name: tpd_probe
* Brief: driver entrance function for initial/power on/create channel
* Input: i2c info, device id
* Output: no
* Return: 0
***********************************************************************/
 static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {   
	int retval = 0;
	char data;
	u8 err=0;
	int reset_count = 0;
#ifdef TPD_PROXIMITY
	int err;
	struct hwmsen_object obj_ps;
#endif
	msg_dma_alloct(&client->dev);

reset_proc:
	fts_i2c_client = client;
	fts_i2c_client->addr = 0x38;
	fts_input_dev = tpd->dev;
	device_parse_platform();

	tpd_gpio_output(GTP_RST_PORT, 0);
	msleep(10);

    	// power on, need confirm with SA
#ifdef CONFIG_FOCALTECH_POWER_BY_PMIC
	pr_err("enable tp power by pmic\n");
	err = regulator_set_voltage(tpd->reg, 2800000, 2800000);
	err += regulator_enable(tpd->reg);

	if (err)
		FTS_ERROR("Power on ic error!");
#else
#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
	//hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
	mt_set_gpio_mode(GPIO_TP_AVDD_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_TP_AVDD_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_TP_AVDD_EN, GPIO_OUT_ONE);
#endif
#ifdef TPD_POWER_SOURCE_1800
	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif
#endif

	msleep(10);
	FTS_DBG("fts reset\n");
	tpd_gpio_output(GTP_RST_PORT, 1);
	tpd_gpio_as_int(GTP_INT_PORT);

	msleep(200);

	err = fts_read_reg(fts_i2c_client, 0x00, &data);
	FTS_DBG("gao_i2c:err %d,data:%d\n", err, data);
	if(err< 0 || data!=0)   // reg0 data running state is 0; other state is not 0
	{
		FTS_DBG("I2C transfer error, line: %d\n", __LINE__);
#ifdef CONFIG_FOCALTECH_POWER_BY_PMIC
		regulator_disable(tpd->reg);
		msleep(20);
#else
#ifdef TPD_POWER_SOURCE_CUSTOM
		hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
#else
		//hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
		mt_set_gpio_mode(GPIO_TP_AVDD_EN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_TP_AVDD_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_TP_AVDD_EN, GPIO_OUT_ZERO);
#endif
#endif

		if ( ++reset_count < 3 ) {
			goto reset_proc;
		}

#ifdef CONFIG_FOCALTECH_POWER_BY_PMIC
		regulator_put(tpd->reg);
#endif
		msg_dma_release(&client->dev);
		return -1; 
	}

	fts_init_gpio_hw();

	/*
	uc_reg_addr = FTS_REG_POINT_RATE;               
	fts_i2c_write(fts_i2c_client, &uc_reg_addr, 1);
	fts_i2c_read(fts_i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	printk("mtk_tpd[FTS] report rate is %dHz.\n",uc_reg_value * 10);

	uc_reg_addr = FTS_REG_FW_VER;
	fts_i2c_write(fts_i2c_client, &uc_reg_addr, 1);
	fts_i2c_read(fts_i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	printk("mtk_tpd[FTS] Firmware version = 0x%x\n", uc_reg_value);


	uc_reg_addr = FTS_REG_CHIP_ID;
	fts_i2c_write(fts_i2c_client, &uc_reg_addr, 1);
	retval=fts_i2c_read(fts_i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	printk("mtk_tpd[FTS] chip id is %d.\n",uc_reg_value);
	if(retval<0)
	{
		printk("mtk_tpd[FTS] Read I2C error! driver NOt load!! CTP chip id is %d.\n",uc_reg_value);
		return 0;
	}
	*/
    
	tpd_load_status = 1;
	//mt_eint_registration(TPD_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	//mt_eint_mask(TPD_EINT_TOUCH_PANEL_NUM);
    
	if (request_irq(TPD_EINT_TOUCH_PANEL_NUM, tpd_eint_interrupt_handler, IRQF_TRIGGER_NONE, "TOUCH_PANEL-eint", NULL) < 0){
		pr_err("%s request_irq failed!\n", __func__);
	}

	disable_irq(TPD_EINT_TOUCH_PANEL_NUM);

    
#ifdef VELOCITY_CUSTOM_fts
	if((err = misc_register(&tpd_misc_device))) {
		FTS_ERROR("tpd_misc_device register failed\n");
	}
#endif

	focaltechthread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(focaltechthread)) {
		retval = PTR_ERR(focaltechthread);
		FTS_ERROR("Failed to create kernel thread: %d\n", retval);
	}

#ifdef SYSFS_DEBUG
	fts_create_sysfs(fts_i2c_client);
#endif
	HidI2c_To_StdI2c(fts_i2c_client);
	fts_get_upgrade_array();
#ifdef FTS_CTL_IIC
	if (fts_rw_iic_drv_init(fts_i2c_client) < 0)
	FTS_ERROR("%s:Create fts control iic driver failed!\n", __func__);
#endif
#ifdef FTS_APK_DEBUG
	fts_create_apk_debug_channel(fts_i2c_client);
#endif

#ifdef TPD_AUTO_UPGRADE
	FTS_DBG("Enter CTP Auto Upgrade!\n");
	is_update = true;
	fts_ctpm_auto_upgrade(fts_i2c_client);
	is_update = false;
#endif

#ifdef FTS_USB_SWITCH
	INIT_WORK(&set_charge_mode_work, set_charge_mode_work_func);
	ctp_charge_mode_enable = charge_mode_enable;
	set_charge_mode_work_func(NULL);
#endif
#ifdef TPD_PROXIMITY
	{
		obj_ps.polling = 1; //0--interrupt mode;1--polling mode;
		obj_ps.sensor_operate = tpd_ps_operate;

		if ((err = hwmsen_attach(ID_PROXIMITY, &obj_ps))) {
			TPD_DEBUG("hwmsen attach fail, return:%d.", err);
		}
	}
#endif
#if FT_ESD_PROTECT
    INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
    queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
#endif
#if FTS_GESTRUE_EN
	fts_Gesture_init(tpd->dev);
#endif
#ifdef MT_PROTOCOL_B
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))
	input_mt_init_slots (tpd->dev, fts_updateinfo_curr.TPD_MAX_POINTS);
#endif
	input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, TPD_RES_X, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, TPD_RES_Y, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
#endif
	input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, 1080, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, 1920, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
    	FTS_INFO("Fts Touch Panel Device Probe %s!\n", (retval < 0) ? "FAIL" : "PASS");

	//mt_eint_unmask(TPD_EINT_TOUCH_PANEL_NUM);
	enable_irq(TPD_EINT_TOUCH_PANEL_NUM);
    
    return 0;
}

/************************************************************************
* Name: tpd_remove
* Brief: remove driver/channel
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
static int __devexit tpd_remove(struct i2c_client *client)
{
    msg_dma_release(&client->dev);
#ifdef FTS_CTL_IIC
    fts_rw_iic_drv_exit();
#endif
#ifdef SYSFS_DEBUG
    fts_remove_sysfs(client);
#endif
#if FT_ESD_PROTECT
    destroy_workqueue(gtp_esd_check_workqueue);
#endif
#ifdef FTS_APK_DEBUG
    fts_release_apk_debug_channel();
#endif
    FTS_INFO("Fts Touch Panel Driver Removed!\n");
    return 0;
}

/************************************************************************
* Name: tpd_local_init
* Brief: add driver info
* Input: no
* Output: no
* Return: fail <0
***********************************************************************/
static int tpd_local_init(void)
{
    //FTS_INFO("Focaltech fts I2C Touchscreen Driver (Built %s @ %s)\n", __data__, __time__);
#ifdef CONFIG_FOCALTECH_POWER_BY_PMIC
    tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");

    if (IS_ERR(tpd->reg)) {
        FTS_ERROR("Get regulator %s failed!\n", "vtouch");
        return PTR_ERR(tpd->reg);
    }
#endif
    if(i2c_add_driver(&tpd_i2c_driver)!=0)
    {
        FTS_DBG("fts unable to add i2c driver.\n");
        return -1;
    }

    if(tpd_load_status == 0) {
        FTS_DBG("fts add error touch panel driver.\n");
        i2c_del_driver(&tpd_i2c_driver);
        return -1;
    }

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 8, 0))
    //for linux 3.8
    input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, (CFG_MAX_TOUCH_POINTS - 1), 0, 0);
#endif
#ifdef TPD_HAVE_BUTTON
    // initialize tpd button data
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif
    FTS_INFO("End  of %s, %d\n", __FUNCTION__, __LINE__);
    tpd_type_cap = 1;
    return 0;
}

static void fts_release_all_finger(void)
{
    
#ifdef MT_PROTOCOL_B
    unsigned int finger_count=0;
    for (finger_count = 0; finger_count < fts_updateinfo_curr.TPD_MAX_POINTS; finger_count++) 
    {
        input_mt_slot(tpd->dev, finger_count);
        input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, 0);
    }
    input_mt_report_pointer_emulation(tpd->dev, false);
    input_sync(tpd->dev);
#else
    tpd_up(0,0,0);
#endif
}

/************************************************************************
* Name: tpd_resume
* Brief: system wake up 
* Input: no use
* Output: no
* Return: no
***********************************************************************/
static void tpd_resume( struct device *h )
{
    FTS_INFO("TPD wake up\n");
#ifdef TPD_PROXIMITY
    if (tpd_proximity_flag == 1) {
        if(tpd_proximity_flag_one == 1) {
            tpd_proximity_flag_one = 0;
            FTS_DBG(" tpd_proximity_flag_one \n");
            return;
        }
    }
#endif

#if FTS_GESTRUE_EN
    if (gesture_wakeup_flag) {
        FTS_INFO("Gesture mode disabled!\n");
        fts_write_reg(fts_i2c_client, 0xD0, 0x00);
        gesture_enabled = false;
    }
#endif

    tpd_gpio_output(GTP_RST_PORT, 0);
    msleep(1);
    tpd_gpio_output(GTP_RST_PORT, 1);

    enable_irq(TPD_EINT_TOUCH_PANEL_NUM);
    msleep(30);
    /* release all touches */
    fts_release_all_finger();
    
    tpd_halt = 0;
#ifdef FTS_USB_SWITCH
    if (need_restore != -1) {
        FTS_INFO("Usb cable was plug %s while in suspend mode!\n",
                 (need_restore ? "in" : "out"));
        fts_write_reg(fts_i2c_client, 0x8B, need_restore);
        need_restore = -1;

    } else {
        FTS_INFO("Usb cable was plug %s while in resume mode!\n",
                 (usb_cable_state ? "in" : "out"));
        fts_write_reg(fts_i2c_client, 0x8B, usb_cable_state);
    }
#endif
    
#if FT_ESD_PROTECT
    count_irq = 0;
    queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
#endif
    FTS_INFO("TPD wake up done\n");
}

/************************************************************************
* Name: tpd_suspend
* Brief: system sleep
* Input: no use
* Output: no
* Return: no
***********************************************************************/

static void tpd_suspend( struct device *h )
{
    static char data = 0x3;
    int ret = 0;
            
    FTS_INFO("TPD enter sleep\n");
    
    // release all point info
    fts_release_all_finger(); 
    
#ifdef TPD_PROXIMITY
    if (tpd_proximity_flag == 1) {
        tpd_proximity_flag_one = 1;
        return;
    }
#endif

#if FTS_GESTRUE_EN
    if (gesture_wakeup_flag) 
    {
        if (fts_updateinfo_curr.CHIP_ID==0x54 || 
            fts_updateinfo_curr.CHIP_ID==0x58 || 
            fts_updateinfo_curr.CHIP_ID==0x86 || 
            fts_updateinfo_curr.CHIP_ID==0x87)
        {
            FTS_INFO("Gesture mode enabled!\n");
            fts_write_reg(fts_i2c_client, 0xd1, 0xff);
            fts_write_reg(fts_i2c_client, 0xd2, 0xff);
            fts_write_reg(fts_i2c_client, 0xd5, 0xff);
            fts_write_reg(fts_i2c_client, 0xd6, 0xff);
            fts_write_reg(fts_i2c_client, 0xd7, 0xff);
            fts_write_reg(fts_i2c_client, 0xd8, 0xff);
        }
        fts_write_reg(fts_i2c_client, 0xd0, 0x01);
        gesture_enabled = true;
        return;
    }
#endif

#if FT_ESD_PROTECT
    cancel_delayed_work_sync(&gtp_esd_check_work);
#endif
    tpd_halt = 1;
    //mt_eint_mask(TPD_EINT_TOUCH_PANEL_NUM);
    disable_irq(TPD_EINT_TOUCH_PANEL_NUM);
    // TP enter sleep mode
    mutex_lock(&i2c_access);
    if ((fts_updateinfo_curr.CHIP_ID == 0x59)) {
        data = 0x02;
    }
    //i2c_smbus_write_i2c_block_data(fts_i2c_client, 0xA5, 1, &data);
    ret = fts_write_reg(fts_i2c_client, 0xA5, data);
    
    if (ret < 0) {
        FTS_ERROR("Write value of 0xA5 failed!\n");
    }
    mutex_unlock(&i2c_access);

    msleep(10);
    FTS_INFO("TPD enter sleep done!\n");
}

static struct tpd_driver_t tpd_device_driver = {
    .tpd_device_name = "fts",
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
};

/************************************************************************
* Name: tpd_suspend
* Brief:  called when loaded into kernel
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static int __init tpd_driver_init(void)
{
    FTS_INFO("MediaTek fts touch panel driver init\n");
    //i2c_register_board_info(IIC_PORT, &fts_i2c_tpd, 1);
    tpd_get_dts_info();
    if(tpd_driver_add(&tpd_device_driver) < 0)
        FTS_ERROR("Add fts driver failed!\n");

    return 0;
}

/************************************************************************
* Name: tpd_driver_exit
* Brief:  should never be called
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static void __exit tpd_driver_exit(void)
{
    FTS_INFO("MediaTek fts touch panel driver exit\n");
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
