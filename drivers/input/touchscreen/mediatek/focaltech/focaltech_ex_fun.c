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
*
* File Name: Focaltech_ex_fun.c
*
* Author: Xu YongFeng
*
* Created: 2015-01-29
*   
* Modify by mshl on 2015-03-20
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <asm/uaccess.h>

#include <linux/i2c.h>//iic
#include <linux/delay.h>//msleep

#include "focaltech_test_main.h"
#include "focaltech_test_ini.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/*create apk debug channel*/
#define PROC_UPGRADE							0
#define PROC_READ_REGISTER						1
#define PROC_WRITE_REGISTER					2
#define PROC_AUTOCLB							4
#define PROC_UPGRADE_INFO						5
#define PROC_WRITE_DATA						6
#define PROC_READ_DATA							7
#define PROC_SET_TEST_FLAG						8
#define FTS_DEBUG_DIR_NAME					"fts_debug"
#define PROC_NAME								"ftxxxx-debug"
#define WRITE_BUF_SIZE							1016
#define READ_BUF_SIZE							1016

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/


/*******************************************************************************
* Static variables
*******************************************************************************/
static unsigned char proc_operate_mode 			= PROC_UPGRADE;
static struct proc_dir_entry *fts_proc_entry;
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
//#if FT_ESD_PROTECT
extern int apk_debug_flag;
//#endif

extern unsigned int TPD_EINT_TOUCH_PANEL_NUM;
extern bool is_update;
/*******************************************************************************
* Static function prototypes
*******************************************************************************/

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	#if FT_ESD_PROTECT
//printk("\n  zax proc w 0 \n");
				esd_switch(0);apk_debug_flag = 1;
//printk("\n  zax v= %d \n",apk_debug_flag);

			#endif
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			FTS_DBG("%s\n", upgrade_file_path);	
						
			//#if FT_ESD_PROTECT
			//	esd_switch(0);apk_debug_flag = 1;
			//#endif
			disable_irq(fts_i2c_client->irq);
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			enable_irq(fts_i2c_client->irq);
			if (ret < 0) {
				#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
				dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
			//#if FT_ESD_PROTECT
			//	esd_switch(1);apk_debug_flag = 0;
			//#endif
		}
		break;
	//case PROC_SET_TEST_FLAG:
	
	//	break;
	case PROC_SET_TEST_FLAG:
		#if FT_ESD_PROTECT
		apk_debug_flag=writebuf[1];
		if(1==apk_debug_flag)
			esd_switch(0);
		else if(0==apk_debug_flag)
			esd_switch(1);
		printk("\n zax flag=%d \n",apk_debug_flag);
		#endif		
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		if(writelen>0)
		{
			ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
			if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
				dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
				return ret;
			}
		}
		break;
	default:
		break;
	}
	
#if FT_ESD_PROTECT
//printk("\n  zax proc w 1 \n");
				esd_switch(1);apk_debug_flag = 0;
//printk("\n  zax v= %d \n",apk_debug_flag);
			#endif
	return count;
}

/* interface of read proc */
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	unsigned char buf[READ_BUF_SIZE];
	#if FT_ESD_PROTECT
//printk("\n  zax proc r 0 \n");
		esd_switch(0);apk_debug_flag = 1;
//printk("\n  zax v= %d \n",apk_debug_flag);
	#endif
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		// after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	if (copy_to_user(buff, buf, num_read_chars)) {
		dev_err(&fts_i2c_client->dev, "%s:copy to user error\n", __func__);
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
		return -EFAULT;
	}
	#if FT_ESD_PROTECT
	//printk("\n  zax proc r 1 \n");
esd_switch(1);apk_debug_flag = 0;
//printk("\n  zax v= %d \n",apk_debug_flag);
	#endif
        //memcpy(buff, buf, num_read_chars);
	return num_read_chars;
}
static const struct file_operations fts_proc_fops = {
		.owner 	= THIS_MODULE,
		.read 	= fts_debug_read,
		.write 	= fts_debug_write,
		
};
#else
/* interface of write proc */
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static int fts_debug_write(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)
{
	//struct i2c_client *client = (struct i2c_client *)fts_proc_entry->data;
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = len;
	int writelen = 0;
	int ret = 0;
	
	#if FT_ESD_PROTECT
			//printk("\n  zax proc w 0 \n");	
esd_switch(0);apk_debug_flag = 1;
//printk("\n  zax v= %d \n",apk_debug_flag);
			#endif
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			FTS_DBG("%s\n", upgrade_file_path);
			//#if FT_ESD_PROTECT
			//	esd_switch(0);apk_debug_flag = 1;
			//#endif
			disable_irq(fts_i2c_client->irq);
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			enable_irq(fts_i2c_client->irq);
			if (ret < 0) {
				dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
				#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
				return ret;
			}
			//#if FT_ESD_PROTECT
			//	esd_switch(1);apk_debug_flag = 0;
			//#endif
		}
		break;
	//case PROC_SET_TEST_FLAG:
	
	//	break;
	case PROC_SET_TEST_FLAG:
		#if FT_ESD_PROTECT
		apk_debug_flag=writebuf[1];
		if(1==apk_debug_flag)
			esd_switch(0);
		else if(0==apk_debug_flag)
			esd_switch(1);
		printk("\n zax flag=%d \n",apk_debug_flag);
		#endif
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = len - 1;
		if(writelen>0)
		{
			ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
			if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
				dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
				return ret;
			}
		}
		break;
	default:
		break;
	}
	
	#if FT_ESD_PROTECT
		//printk("\n  zax proc w 1 \n");		
esd_switch(1);apk_debug_flag = 0;
//printk("\n  zax v= %d \n",apk_debug_flag);
			#endif
	return len;
}

/* interface of read proc */
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static int fts_debug_read( char *page, char **start,
	off_t off, int count, int *eof, void *data )
{
	//struct i2c_client *client = (struct i2c_client *)fts_proc_entry->data;
	int ret = 0;
	unsigned char buf[READ_BUF_SIZE];
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	#if FT_ESD_PROTECT
	//printk("\n  zax proc r 0 \n");
esd_switch(0);apk_debug_flag = 1;
//printk("\n  zax v= %d \n",apk_debug_flag);
	#endif
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		// after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	memcpy(page, buf, num_read_chars);
	#if FT_ESD_PROTECT
	//printk("\n  zax proc r 1 \n");
esd_switch(1);apk_debug_flag = 0;
//printk("\n  zax v= %d \n",apk_debug_flag);
	#endif
	return num_read_chars;
}
#endif
/************************************************************************
* Name: fts_create_apk_debug_channel
* Brief:  create apk debug channel
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_apk_debug_channel(struct i2c_client * client)
{
	#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
		fts_proc_entry = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);		
	#else
		fts_proc_entry = create_proc_entry(PROC_NAME, 0777, NULL);
	#endif
	if (NULL == fts_proc_entry) 
	{
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		
		return -ENOMEM;
	} 
	else 
	{
		dev_info(&client->dev, "Create proc entry success!\n");
		
		#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
			//fts_proc_entry->data = client;
			fts_proc_entry->write_proc = fts_debug_write;
			fts_proc_entry->read_proc = fts_debug_read;
		#endif
	}
	return 0;
}
/************************************************************************
* Name: fts_release_apk_debug_channel
* Brief:  release apk debug channel
* Input: no
* Output: no
* Return: no
***********************************************************************/
void fts_release_apk_debug_channel(void)
{
	
	if (fts_proc_entry)
		#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
            proc_remove(fts_proc_entry);
		#else
			remove_proc_entry(PROC_NAME, NULL);
		#endif
}

/************************************************************************
* Name: fts_tpfwver_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 fwver = 0;
	u8 vendor_id = 0x0;
	char *temp = buf;
	char *vendor = "null";

	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);  jacob use globle fts_wq_data data
	mutex_lock(&fts_input_dev->mutex);

	if (fts_read_reg(fts_i2c_client, FTS_REG_FW_VER, &fwver) < 0){
		dev_err(&fts_i2c_client->dev, "fw version read failed\n");
		mutex_unlock(&fts_input_dev->mutex);
		return -ENODEV;
	}

	if (fts_read_reg(fts_i2c_client, FTS_REG_VENDOR_ID, &vendor_id) < 0) {
		dev_err(&fts_i2c_client->dev, "vendor id read failed\n");
		mutex_unlock(&fts_input_dev->mutex);
		return -ENODEV;
	}

	if (FTS_VENDOR_LIANSI == vendor_id )
		vendor = "liansi";
	else if (FTS_VENDOR_OFILM == vendor_id)
		vendor = "ofilm";

	buf += sprintf(buf, "fwver:%02X\n", fwver);
	buf += sprintf(buf, "vid:%02X %s\n", vendor_id, vendor);

	mutex_unlock(&fts_input_dev->mutex);

	return (ssize_t)(buf - temp);;
}
/************************************************************************
* Name: fts_tpfwver_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpfwver_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tprwreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_store
* Brief:  read/write register
* Input: device, device attribute, char buf, char count
* Output: print register value
* Return: char count
***********************************************************************/
static ssize_t fts_tprwreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	/*u32 wmreg=0;*/
	long unsigned int wmreg=0;
	u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5]={0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&fts_input_dev->mutex);	
	num_read_chars = count - 1;
	if (num_read_chars != 2) 
	{
		if (num_read_chars != 4) 
		{
			dev_err(dev, "please input 2 or 4 character\n");
			goto error_return;
		}
	}
	memcpy(valbuf, buf, num_read_chars);
	retval = kstrtoul(valbuf, 16, &wmreg);
	if (0 != retval) 
	{
		dev_err(dev, "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
		goto error_return;
	}
	if (2 == num_read_chars) 
	{
		/*read register*/
		regaddr = wmreg;
		printk("[focal](0x%02x)\n", regaddr);
		if (fts_read_reg(client, regaddr, &regvalue) < 0)
			printk("[Focal] %s : Could not read the register(0x%02x)\n", __func__, regaddr);
		else
			printk("[Focal] %s : the register(0x%02x) is 0x%02x\n", __func__, regaddr, regvalue);
	} 
	else 
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if (fts_write_reg(client, regaddr, regvalue)<0)
			dev_err(dev, "[Focal] %s : Could not write the register(0x%02x)\n", __func__, regaddr);
		else
			dev_dbg(dev, "[Focal] %s : Write 0x%02x into register(0x%02x) successful\n", __func__, regvalue, regaddr);
	}
	error_return:
	mutex_unlock(&fts_input_dev->mutex);
	
	return count;
}
/************************************************************************
* Name: fts_fwupdate_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupdate_store
* Brief:  upgrade from *.i
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupdate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	//struct fts_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	//data = (struct fts_ts_data *) i2c_get_clientdata(client);
	#if FT_ESD_PROTECT
		esd_switch(0);apk_debug_flag = 1;
	#endif

	pr_err("%s\n", __func__);
	mutex_lock(&fts_input_dev->mutex);
	is_update = true;
	disable_irq(TPD_EINT_TOUCH_PANEL_NUM);

	fts_ctpm_auto_upgrade(client);
#if 0
	i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
	if (i_ret == 0)
	{
		msleep(300);
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		dev_dbg(dev, "%s [FTS] upgrade to new version 0x%x\n", __func__, uc_host_fm_ver);
	}
	else
	{
		dev_err(dev, "%s ERROR:[FTS] upgrade failed ret=%d.\n", __func__, i_ret);
	}
#endif
	is_update = false;
	enable_irq(TPD_EINT_TOUCH_PANEL_NUM);
	mutex_unlock(&fts_input_dev->mutex);
	#if FT_ESD_PROTECT
		esd_switch(1);apk_debug_flag = 0;
	#endif
	return count;
}
/************************************************************************
* Name: fts_fwupgradeapp_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupgradeapp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupgradeapp_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupgradeapp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';
	#if FT_ESD_PROTECT
		esd_switch(0);apk_debug_flag = 1;
	#endif
	mutex_lock(&fts_input_dev->mutex);
	
	disable_irq(client->irq);
	fts_ctpm_fw_upgrade_with_app_file(client, fwname);
	enable_irq(client->irq);
	
	mutex_unlock(&fts_input_dev->mutex);
	#if FT_ESD_PROTECT
		esd_switch(1);apk_debug_flag = 0;
	#endif
	return count;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	return -EPERM;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FOCALTECH_TEST_INFO  "File Version of  focaltech_test.c:  V1.0.0 2016-03-24"
#define FTS_DBG_EN
#ifdef FTS_DBG_EN	
#define FTS_COMMON_DBG(fmt, args...)	do {printk("[FTS] %s. line: %d.  "fmt"\n", __FUNCTION__, __LINE__, ##args);} while (0)
//#define FTS_COMMON_DBG(fmt, arg...)	printk("[FTS] %s. line: %d.  "fmt"\n", __FUNCTION__, __LINE__, ##arg)
#else
#define FTS_COMMON_DBG(fmt, args...) do{}while(0)
//#define FTS_COMMON_DBG(fmt, arg...) (void)(0)
#endif

//配置文件存放目录定义
#define FTS_INI_FILE_PATH "/system/etc/firmware/"  
#define FTS_TEST_BUFFER_SIZE		80*1024
#define FTS_TEST_PRINT_SIZE		128

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/


/*******************************************************************************
* Static variables
*******************************************************************************/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/


/*******************************************************************************
* Static function prototypes
*******************************************************************************/
static int fts_test_get_ini_size(char *config_name);
static int fts_test_read_ini_data(char *config_name, char *config_buf);
static int fts_test_save_test_data(char *file_name, char *data_buf, int iLen);
static int fts_test_get_testparam_from_ini(char *config_name);
static int fts_test_entry(char *ini_file_name);


/*******************************************************************************
* functions body
*******************************************************************************/

//获取配置文件大小, 用于分配内存读取配置
static int fts_test_get_ini_size(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode = NULL;
	//unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "%s%s", FTS_INI_FILE_PATH, config_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		FTS_COMMON_DBG("error occured while opening file %s.",  filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	//magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);

	return fsize;
}
//读取配置到内存
static int fts_test_read_ini_data(char *config_name, char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode = NULL;
	//unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	loff_t pos = 0;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FTS_INI_FILE_PATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		FTS_COMMON_DBG("error occured while opening file %s.",  filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	//magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}
//保存测试数据到SD卡 etc.
static int fts_test_save_test_data(char *file_name, char *data_buf, int iLen)
{
	struct file *pfile = NULL;
	
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	//sprintf(filepath, "%s%s", FTS_INI_FILE_PATH, file_name);
	sprintf(filepath, "%s%s", "/mnt/sdcard/", file_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_CREAT|O_RDWR, 0);
	if (IS_ERR(pfile)) {
		FTS_COMMON_DBG("error occured while opening file %s.",  filepath);
		return -EIO;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_write(pfile, data_buf, iLen, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}

//读取,解析配置文件,初始化测试变量
static int fts_test_get_testparam_from_ini(char *config_name)
{
	char *filedata = NULL;
	int ret = 0;

	int inisize = fts_test_get_ini_size(config_name);

	FTS_COMMON_DBG("inisize = %d \n ", inisize);
	if (inisize <= 0) {
		FTS_COMMON_DBG("%s ERROR:Get firmware size failed",  __func__);
		return -EIO;
	}

	filedata = kmalloc(inisize + 1, GFP_ATOMIC);
		
	if (fts_test_read_ini_data(config_name, filedata)) {
		FTS_COMMON_DBG("%s() - ERROR: request_firmware failed",  __func__);
		kfree(filedata);
		return -EIO;
	} else {
		FTS_COMMON_DBG("fts_test_read_ini_data successful");
	}

	ret = set_param_data(filedata);
	if(ret < 0)
		return ret;
	
	return 0;

}


/////////////////////////////////
//测试库调用总入口
///////////////////////////////////
static int fts_test_entry(char *ini_file_name)
{
        /* place holder for future use */
            char cfgname[128];
        char *testdata = NULL;
        char *printdata = NULL;
        int iTestDataLen=0;//库中测试数据实际长度,用于保存到文件
        int ret = 0;
        int icycle = 0, i =0;
        int print_index = 0;
        
    
        FTS_COMMON_DBG("");
        /*用于获取存放在库中的测试数据,注意分配空间大小.*/
        FTS_COMMON_DBG("Allocate memory, size: %d", FTS_TEST_BUFFER_SIZE);
        testdata = kmalloc(FTS_TEST_BUFFER_SIZE, GFP_ATOMIC);
        if(NULL == testdata)
        {
            //printk("kmalloc failed in function:%s",  __func__);
            FTS_COMMON_DBG("kmalloc failed in function:%s",  __func__);
            return -1;
        }
        printdata = kmalloc(FTS_TEST_PRINT_SIZE, GFP_ATOMIC);
        if(NULL == printdata)
        {
            //printk("kmalloc failed in function:%s",  __func__);
            FTS_COMMON_DBG("kmalloc failed in function:%s",  __func__);
            if(NULL != testdata) kfree(testdata);
            return -1;
        }
        /*初始化平台相关的I2C读写函数*/
        init_i2c_write_func(fts_i2c_write_testa);
        init_i2c_read_func(fts_i2c_read_testa);
    
        /*初始化指针内存*/
        ret = focaltech_test_main_init();
        if(ret < 0)
        {
            FTS_COMMON_DBG("focaltech_test_main_init() error.");
            goto TEST_ERR;
        }       
    
        /*读取解析配置文件*/
        memset(cfgname, 0, sizeof(cfgname));
        sprintf(cfgname, "%s", ini_file_name);  
        if(fts_test_get_testparam_from_ini(cfgname) <0)
        {
            FTS_COMMON_DBG("get testparam from ini failure");
            goto TEST_ERR;
        }
    
        /*根据测试配置开始测试*/
        if(true == start_test_tp())
            FTS_COMMON_DBG("tp test pass");
        else
            FTS_COMMON_DBG("tp test failure");
            
        /*获取测试库中的测试数，并保存*/
        iTestDataLen = get_test_data(testdata);
        //FTS_COMMON_DBG("\n%s", testdata);
    
        icycle = 0;
        /*打印触摸数据包 */
        FTS_COMMON_DBG("print test data: \n");
        for(i = 0; i < iTestDataLen; i++)
        {
            if(('\0' == testdata[i])//遇到结束符
                ||(icycle == FTS_TEST_PRINT_SIZE -2)//满足打印字符串长度要求
                ||(i == iTestDataLen-1)//已是最后一个字符
            )
            {
                if(icycle == 0)
                {
                    print_index++;
                }   
                else
                {
                    memcpy(printdata, testdata + print_index, icycle);
                    printdata[FTS_TEST_PRINT_SIZE-1] = '\0';
                    printk("%s", printdata);
                    print_index += icycle;
                    icycle = 0;
                }
            }
            else
            {
                icycle++;
            }
        }
        printk("\n");       
    
        fts_test_save_test_data("testdata.csv", testdata, iTestDataLen);
    
        /*释放内存等... */
        focaltech_test_main_exit();
        
            
        //mutex_unlock(&g_device_mutex);
        if(NULL != testdata) kfree(testdata);
        if(NULL != printdata) kfree(printdata);
        return 0;
        
    TEST_ERR:
        if(NULL != testdata) kfree(testdata);
        if(NULL != printdata) kfree(printdata);
        return -1;

}

static ssize_t fts_ic_detect_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	size_t len = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	printk("start fts_ic_detect_show \n");
	mutex_lock(&fts_input_dev->mutex);

	disable_irq(client->irq);

	rc =fts_test_entry("NX573J_FT5436_TEST.ini");
	if (rc < 0) {
		printk( "fts_ic_detect_show write error\n");
		len = snprintf(buf, PAGE_SIZE, "1\n");/*test failed*/
	}

	printk("fts_ic_detect_show write success\n");

	enable_irq(client->irq);

	mutex_unlock(&fts_input_dev->mutex);

	msleep(300);

	len = snprintf(buf, PAGE_SIZE, "0\n");//PASS
	return len;
}

extern int gesture_wakeup_flag;

static ssize_t fts_easy_wakeup_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    u32 cmd;
    if (sscanf(buf, "%x", &cmd) == 1){
        printk( "fts_easy_wakeup_gesture_store write cmd %x \n",cmd);
        if(cmd == 1)
            gesture_wakeup_flag=1;
        else 
            gesture_wakeup_flag=0;
        return count;
        }
    else {
        printk("fts_easy_wakeup_gesture_store cmd error\n");
		return -EINVAL;
        }
}

/****************************************/
/* sysfs */
/* get the fw version
*   example:cat ftstpfwver
*/
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, fts_tpfwver_show, fts_tpfwver_store);
/* upgrade from *.i
*   example: echo 1 > ftsfwupdate
*/
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, fts_fwupdate_show, fts_fwupdate_store);
/* read and write register
*   read example: echo 88 > ftstprwreg ---read register 0x88
*   write example:echo 8807 > ftstprwreg ---write 0x07 into register 0x88
*
*   note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, fts_tprwreg_show, fts_tprwreg_store);
/*  upgrade from app.bin
*    example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, fts_fwupgradeapp_show, fts_fwupgradeapp_store);
static DEVICE_ATTR(ftsgetprojectcode, S_IRUGO|S_IWUSR, fts_getprojectcode_show, fts_getprojectcode_store);
static DEVICE_ATTR(ic_detect, S_IRUSR|S_IXUSR|S_IRGRP|S_IXGRP|S_IROTH|S_IXOTH, fts_ic_detect_show,NULL);
static DEVICE_ATTR(easy_wakeup_gesture, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IXOTH, NULL, fts_easy_wakeup_gesture_store);

/* add your attr in here*/
static struct attribute *fts_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsgetprojectcode.attr,
    &dev_attr_ic_detect.attr,
    &dev_attr_easy_wakeup_gesture.attr,
	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

/************************************************************************
* Name: fts_create_sysfs
* Brief:  create sysfs for debug
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_sysfs(struct i2c_client * client)
{
	int err;
	err = sysfs_create_group(&client->dev.kobj, &fts_attribute_group);
	if (0 != err) 
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed.\n", __func__);
		sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
		return -EIO;
	} 
	else 
	{
		pr_info("fts:%s() - sysfs_create_group() succeeded.\n",__func__);
	}
	//HidI2c_To_StdI2c(client);
	return err;
}
/************************************************************************
* Name: fts_remove_sysfs
* Brief:  remove sys
* Input: i2c info
* Output: no
* Return: no
***********************************************************************/
int fts_remove_sysfs(struct i2c_client * client)
{
	sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
	return 0;
}
