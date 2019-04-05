/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* drivers/hwmon/mt6516/amit/tmd2771.c - TMD2771 ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/wakelock.h> 
#include <asm/io.h>
#include <linux/module.h>

#include <linux/hwmsen_helper.h>
#include <cust_eint.h>
#include <linux/hwmsensor.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_dev.h>
#include <alsps.h>
//#ifdef MT6516
//#include <mach/mt6516_devs.h>
//#include <mach/mt6516_typedefs.h>
//#include <mach/mt6516_gpio.h>
//#include <mach/mt6516_pll.h>
//#endif

//#ifdef MT6573
//#include <mach/mt6573_devs.h>
//#include <mach/mt6573_typedefs.h>
//#include <mach/mt6573_gpio.h>
//#include <mach/mt6573_pll.h>
//#endif

//#ifdef MT6575
//#include <mach/mt6575_devs.h>
//#include <mach/mt6575_typedefs.h>
//#include <mach/mt6575_gpio.h>
//#include <mach/mt6575_pm_ldo.h>
//#endif


//#ifdef MT6577
//#include <mach/mt_devs.h>
//#include <mach/mt_typedefs.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_pm_ldo.h>
//#endif
//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/eint.h>

#include <linux/proc_fs.h>


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>


//#ifdef MT6516
//#define POWER_NONE_MACRO MT6516_POWER_NONE
//#endif

//#ifdef MT6573
//#define POWER_NONE_MACRO MT65XX_POWER_NONE
//#endif

//#ifdef MT6575
//#define POWER_NONE_MACRO MT65XX_POWER_NONE
//#endif

//#ifdef MT6577
//#define POWER_NONE_MACRO MT65XX_POWER_NONE
//#endif
#define POWER_NONE_MACRO MT65XX_POWER_NONE


#include <linux/wakelock.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include "tmd2771_cust_alsps.h"
#include "tmd2771.h"
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define TMD2771_DEV_NAME     "TMD2771"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN()               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)                 
/******************************************************************************
 * extern functions
*******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);

/*----------------------------------------------------------------------------*/
static struct i2c_client *tmd2771_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id tmd2771_i2c_id[] = {{TMD2771_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_TMD2771={ I2C_BOARD_INFO("TMD2771", (0X72>>1))};
/*the adapter id & i2c address will be available in customization*/
//static unsigned short tmd2771_force[] = {0x02, 0X72, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const tmd2771_forces[] = { tmd2771_force, NULL };
//static struct i2c_client_address_data tmd2771_addr_data = { .forces = tmd2771_forces,};
/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int tmd2771_i2c_remove(struct i2c_client *client);
static int tmd2771_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int tmd2771_i2c_resume(struct i2c_client *client);

extern struct tmd2772_alsps_hw *tmd2771_get_cust_alsps_hw(void);
extern int hwmsen_alsps_sensor_add(struct sensor_init_info* obj) ;

long tmd2771_read_ps(struct i2c_client *client, u16 *data);
static int tmd2771_init_client_for_cali(struct i2c_client *client);
static int tmd2771_init_client(struct i2c_client *client);


static struct wake_lock ps_lock;//zhaoshaopeng add 

static struct tmd2772_alsps_hw cust_alsps_hw = {
    .i2c_num    = 2,
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    .i2c_addr   = {0x72, 0x48, 0x78, 0x00},
    /*Lenovo-sw chenlj2 add 2011-06-03,modify parameter below two lines*/
    .als_level  = { 4, 6, 10,  20,   30,   50,  80,  120,   200, 360, 400,   700,   1200 , 2000,3000},
    //    .als_level  = { 4, 10,  20,   30,   50,  80,  120,   200, 400,   700,   1200 , 2000},,  3000, 5000, 8000},
    //.als_value  = {10, 20,20,  120, 120, 280,  280,  280, 1600,  1600,  1600,  6000,  6000, 9000,  10240, 10240},
    .als_value  = {0,  20,  20,  120,  120,  280,  280,  1600, 1600,  6000,  6000,  9000, 9000, 10240,  10240, 10240},

    //.als_level  = { 0,  2,  4,   6,   8,  10,  20, 40, 80,  200,  400, 1000, 2000, 5000, 10000},
    //.als_value  = {0, 0, 50,  90, 130, 160,  225,  320,  640,  780,  980,  1600,  2600, 4600,  10240, 10240},
    .ps_threshold_high = 900,
    .ps_threshold_low = 750,
    .ps_threshold = 900,
};
//struct alsps_hw *get_cust_alsps_hw(void) {
struct tmd2772_alsps_hw *tmd2771_get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}
//fenggy mask int TMD2771_CMM_PPCOUNT_VALUE = 0x04;
int TMD2771_CMM_PPCOUNT_VALUE = 0x0c;
int ZOOM_TIME = 4;
//int TMD2771_CMM_CONTROL_VALUE = 0xE0;
int TMD2771_CMM_CONTROL_VALUE = 0x20;

int TMD2771_OFFSET = 0x00;

static u8  offset_data=0;
static u16 tmp_data=0;
#define CLOSE_AWAY_OFFSET 30

static struct tmd2771_priv *g_tmd2771_ptr = NULL;

#define OPEN_PROX_ARITHMETIC 1

 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct tmd2771_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct tmd2771_priv {
    struct tmd2772_alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct tmd2771_i2c_addr  addr;
    
    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver tmd2771_i2c_driver = {	
	.probe      = tmd2771_i2c_probe,
	.remove     = tmd2771_i2c_remove,
	.detect     = tmd2771_i2c_detect,

#if !defined(CONFIG_HAS_EARLYSUSPEND)  
	
	.suspend    = tmd2771_i2c_suspend,
	.resume     = tmd2771_i2c_resume,
#endif

	.id_table   = tmd2771_i2c_id,
//	.address_data = &tmd2771_addr_data,
	.driver = {
//		.owner          = THIS_MODULE,
		.name           = TMD2771_DEV_NAME,
	},
};

static struct tmd2771_priv *tmd2771_obj = NULL;
//static struct platform_driver tmd2771_alsps_driver;
/* Delete for auto detect feature */
//static struct platform_driver tmd2771_alsps_driver;
/* Delete end */

/* Add for auto detect feature */
static int  tmd2771_local_init(void);
static int tmd2771_remove(void);
static int tmd2771_init_flag =0;
static struct sensor_init_info tmd2771_init_info = {			
	.name = "tmd2771",			
	.init = tmd2771_local_init,			
	.uninit =tmd2771_remove,	
};/* Add end */


struct proc_dir_entry *proc_tmd2771_close_away;
int val_cali = 0;

/*----------------------------------------------------------------------------*/
int tmd2771_get_addr(struct tmd2772_alsps_hw *hw, struct tmd2771_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void tmd2771_power(struct tmd2772_alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "TMD2771")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "TMD2771")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}

/*----------------------------------------------------------------------*/





int tmd2771_read_mean(struct i2c_client *client, int n)
{
printk(" [alsps]  tmd2771_read_mean start \n ");
	struct tmd2771_priv *obj = i2c_get_clientdata(client);
	int prox_sum = 0, prox_mean = 0;
	int i, ret = 0;
	u16 prox_data[20];
	mdelay(10);
/////////////
		n=2;
	for(i = 0; i < n; i++)
	{
		if(ret = tmd2771_read_ps(tmd2771_i2c_client, &prox_data[i]))
		{
			printk("tmd2772_read_data_for_cali fail: %d\n", i);
			return ret;
		}
		prox_sum += prox_data[i];
		mdelay(10);
	}
	prox_mean = prox_sum/n;
	
printk(" [alsps]  tmd2771_read_mean end \n ");	
	
	return prox_mean;
}


int  tmd2771_3cm_ps_calibrate(){
APS_FUN();
	struct tmd2771_priv *obj = i2c_get_clientdata(tmd2771_i2c_client);
	struct i2c_client * client = tmd2771_i2c_client;

	tmd2771_init_client_for_cali(obj->client);
	int value=tmd2771_read_mean(tmd2771_i2c_client ,3);
	printk("tmd2771_3cm_ps_calibrate_value:%d\n",value);
	if(value<=400 || value>=900)
		goto err_out10;
///////////	
	ps_cali.valid=1;
	ps_cali.close=value-15;
	ps_cali.far_away=value-45;		
	printk(" [alsps] close =%d ,for_away=%d  value=%d   \n",ps_cali.close,ps_cali.far_away,value); 
	
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	
	tmd2771_obj->hw->ps_threshold_high = ps_cali.close;
			tmd2771_obj->hw->ps_threshold_low = ps_cali.far_away;
			atomic_set(&tmd2771_obj->ps_thd_val_high,  tmd2771_obj->hw->ps_threshold_high);
			atomic_set(&tmd2771_obj->ps_thd_val_low,  tmd2771_obj->hw->ps_threshold_low);
			
APS_FUN();	
	return ps_cali.valid;
	
err_out10:
	ps_cali.valid = 0;
	return ps_cali.valid;
///////////	
}



static int tmd2771_ps_calibrate()
{
	
	struct tmd2771_priv *obj = i2c_get_clientdata(tmd2771_i2c_client);
	struct i2c_client * client = tmd2771_i2c_client;
	int prox_sum = 0, prox_mean = 0, prox_max = 0;
	int prox_threshold_hi = 0, prox_threshold_lo = 0;
	int i, ret = 0;
	u16 prox_data[20];
	u8 buffer[2];
	int err,reg_value[10];
	int res, res_cali;
APS_FUN();	
			reg_value[0]=0;
			res = i2c_master_recv(client, reg_value, 0x1);
		//	if(res <= 0)
//				{
				//	goto EXIT_ERR;
		//		}
	printk("zengtao:0x%x == %d , %d, %s\n", reg_value[0],reg_value[0], __LINE__, __FUNCTION__);


	tmd2771_init_client_for_cali(obj->client);
	buffer[0] = TMD2771_CMM_OFFSET;    
	buffer[1] = 0x00;
	res = i2c_master_send(client, buffer, 0x2);
offset_data = 0;	
	if(res <= 0)
	{
	//	goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}



	prox_mean = tmd2771_read_mean(client, 10);
		if(prox_mean>800)
		{
			   res_cali = 0;
				  printk("tmd2771_ps_calibrate fail  \n");
				return -1;  
		}	
	
	prox_mean = tmd2771_read_mean(client, 10);
if((154-prox_mean)>=0)
	offset_data=(8*(154-prox_mean))/(5*TMD2771_CMM_PPCOUNT_VALUE);
else	
	offset_data=(8*(prox_mean-154))/(5*TMD2771_CMM_PPCOUNT_VALUE);	
	

	
	if((0 <=prox_mean)&&(prox_mean <154))//if prox_mean_clai is less than 200,plus prox_mean_clai
	{
		
		buffer[0] = TMD2771_CMM_OFFSET;
		offset_data = buffer[1] = 0x80 | offset_data;  // 0x80  not change. | 0x30  can change
		err= i2c_master_send(client, buffer, 0x2);
		if(err<= 0)
		{
			printk("prox_mean<50 error \n");
		}
		mdelay(100);//5ms
		printk("  [alsps]  <50 offset is %d  \n ",offset_data);
		prox_mean = tmd2771_read_mean(client, 10);
	}
	else if((154 <= prox_mean))//if prox_mean_clai is less than 200,plus prox_mean_clai
	{
		buffer[0] = TMD2771_CMM_OFFSET;
		offset_data = buffer[1] = 0x00 | offset_data;  // 0x80  not change.       | 0x30  can change
		err= i2c_master_send(client, buffer, 0x2);
		if(err<= 0)
		{
			printk("prox_mean<120 error \n");
		}
		mdelay(100);//5ms
		printk("  [alsps]  <120 offset is %d  \n ",offset_data);
		prox_mean = tmd2771_read_mean(client, 10);
	}
	
	else
	{
		offset_data = 0;
		printk("  [alsps]  %d  \n ",offset_data);
	}




	
	prox_mean = tmd2771_read_mean(client, 10);
		if((100<=prox_mean)&&(prox_mean<=300)  )
		{res_cali = 1;
			printk("tmd2771_ps_calibrate success  prox_mean is %d \n",prox_mean);
			   
		}
		else
		{
			res_cali = 0;
				  printk("tmd2771_ps_calibrate fail  \n");
		}

APS_FUN();		   
	printk("  [alsps]  offset is %d   end \n ",offset_data);  
	
	
	reg_value[0]=0;
	res = i2c_master_recv(client, reg_value, 0x1);
//	if(res <= 0)
//				{
//					goto EXIT_ERR;
//				}
	printk("zengtao:0x%x == %d , %d, %s\n", reg_value[0],reg_value[0], __LINE__, __FUNCTION__);

	return res_cali;
	
	
}

#define PROX_DATA_SAFE_RANGE_MAX_VALUE  22000
#define PROX_DATA_SAFE_RANGE_MIN_VALUE   1000
#define PROX_DATA_PROX_THRES_MAX_VALUE   28000
#define PROX_DATA_PROX_THRES_MIN_VALUE    2000

static int read_file(char *filename)
{
    int fd;
    char buf[1];
    struct file *filp;
    char bufs[100];
    int ret;
	int data_val=0;
 
    /* kernel memory access setting */
    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);
 
    /* open a file */
    filp = filp_open(filename, O_RDWR, 0664);
    if (IS_ERR(filp)) {
        printk("open error\n");
        return 0;
    }
    else {
        printk("open success\n");
    }
 
    /* write example */
  ///  printk("filp->f_pos = %d\n", (int)filp->f_pos);
  //  vfs_write(filp, bufs, strlen(bufs), &filp->f_pos);
  //  printk("filp->f_pos = %d\n", (int)filp->f_pos);
 
    /* read example */
    printk(KERN_DEBUG);

        int pos = (int)filp->f_pos;
        printk("filp->f_pos=%d \n: ", (int)filp->f_pos);
        memset(bufs,'\n',60);
        ret = vfs_read(filp, bufs, 60, &filp->f_pos);
        printk("char_cnt =%d \n: ", ret);
		
        if (ret != 0) {
            printk("filp->f_pos=%d\n", (int)filp->f_pos);
        }
        printk("\n");

 	ret= sscanf(bufs,"%d\n",&data_val);
	printk("data_val=%d\n", data_val);
    filp_close(filp, NULL);  /* filp_close(filp, current->files) ?  */
    /* restore kernel memory setting */

    set_fs(old_fs);
	return data_val;
}


int offset_start_flag = 0;
int cali_start_flag = 0;
int cali_3cm_status_flag = 0;

static ssize_t tmd2771_show_prox_data_safe_range_max(struct device_driver *ddri, char *buf)
{
	int res;
	printk("tmd2771_show_prox_data_safe_range_max\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "0x%04X\n", PROX_DATA_SAFE_RANGE_MAX_VALUE);     
	
}


static ssize_t tmd2771_show_prox_data_safe_range_min(struct device_driver *ddri, char *buf)
{
	int res;
	printk("tmd2771_show_prox_data_safe_range_min\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "0x%04X\n", PROX_DATA_SAFE_RANGE_MIN_VALUE);     
	
}



static long tmd2771_enable_ps_for_cali(struct i2c_client *client, int enable)
{
APS_FUN();
	struct tmd2771_data *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	u8 buffer[2];  	
	long res = 0;

             APS_LOG("tmd2771_enable_ps  = %x\n",enable);

	if(enable)
	{
	       databuf[0] = TMD2771_CMM_OFFSET;    
		databuf[1] = 0;//0x02
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2771_ERR_I2C;
		}
		val_cali = tmd2771_ps_calibrate();
		tmd2771_init_client(tmd2771_i2c_client);
	}
	else
    {
    	
                         
		if(res <= 0)
		{
			goto EXIT_ERR;
		}                
		
 	}
APS_FUN();
		return 0;
	
EXIT_ERR:
	APS_ERR("tmd2771_enable_ps fail\n");
	return res;
	}

 
static ssize_t tmd2771_show_data_val(struct device_driver *ddri, char *buf)
{
	int val;
	APS_FUN();
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
       tmd2771_init_client_for_cali(tmd2771_i2c_client);
	val = tmd2771_read_mean(tmd2771_i2c_client, 10);
       //int val = tmd2771_simple_read_ps (tmd2771_i2c_client);
       printk("tmd2771_show_data_val---val = %d",val);
	APS_FUN();   
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", val);     
}

int cal_flag_val = 0;
int cal_finish_flag = 0;

static ssize_t tmd2771_show_prox_offset_cal(struct device_driver *ddri, char *buf)
{
	int res;
	printk("tmd2771_show_prox_offset_cal---val_cali = %d\n", val_cali);
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "0x%04X\n", val_cali);     
	
}


static ssize_t tmd2771_store_prox_offset_cal(struct device_driver *ddri, const char *buf, size_t count)
{
}

static ssize_t tmd2771_show_prox_thres_max(struct device_driver *ddri, char *buf)
{
	int res;
	printk("tmd2771_show_prox_thres_max\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "0x%04X\n", PROX_DATA_PROX_THRES_MAX_VALUE);     
	
}

static ssize_t tmd2771_show_prox_thres_min(struct device_driver *ddri, char *buf)
{
	int res;
	printk("tmd2771_show_prox_thres_min\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "0x%04X\n", PROX_DATA_PROX_THRES_MIN_VALUE);     
	
}

static ssize_t tmd2771_store_prox_thres_max(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	printk("tmd2771_store_prox_thres_max\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &value))
	printk("tmd2771_store_prox_thres_max cal_finish_flag=%d\n",cal_finish_flag);
	cal_finish_flag = value;

	return count;    
}

static ssize_t tmd2771_store_prox_thres_min(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	printk("tmd2771_store_prox_thres_min\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &value))

	cal_flag_val = value;

	return count;    
}

static ssize_t tmd2771_show_prox_data_max(struct device_driver *ddri, char *buf)
{
	int res;
	printk("tmd2771_show_prox_data_max\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	

	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", 4000);     
	
}

static ssize_t tmd2771_show_prox_thres(struct device_driver *ddri, char *buf)
{
	int res;
	printk("tmd2771_show_prox_thres\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	

	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", tmd2771_obj->ps);     
	
}

static ssize_t tmd2771_show_prox_calibrate_start(struct device_driver *ddri, char *buf)
{
	int res;
	printk("tmd2771_show_prox_thres\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	
	return  0;//return scnprintf(buf, PAGE_SIZE, "0x%04X\n", calibrate_start_flag);     
	
}

static ssize_t tmd2771_store_prox_calibrate_start(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	printk("tmd2771_store_prox_calibrate_start\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &value))
	        printk("tmd2771_store_prox_calibrate_start calibrate_start_flag=%d\n",value);
	cali_start_flag = value;

	return count;    
}


static ssize_t tmd2771_show_prox_3cm_calibrate_status(struct device_driver *ddri, char *buf)
{
	int res;
	printk("tmd2771_show_prox_3cm_calibrate_status\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	
	return scnprintf(buf, PAGE_SIZE, "0x%x\n", cali_3cm_status_flag);     	
}

static ssize_t tmd2771_show_prox_3cm_calibrate_offset(struct device_driver *ddri, char *buf)
{
	int res;
	printk("tmd2771_show_prox_3cm_calibrate_offset\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	
	return scnprintf(buf, PAGE_SIZE, "0x%x\n", offset_data);     	
}
int offset_flag = 0;
static ssize_t tmd2771_store_prox_3cm_calibrate_offset(struct device_driver *ddri, char *buf)
{
	int res, offset_data;
	struct i2c_client * client = tmd2771_i2c_client;
	u8 buffer[2];  
	int flag = 0;
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	if(offset_flag == 0)
	{
		if(1 == sscanf(buf, "%d", &offset_data))
		{
			printk("offset_data = %d\n", offset_data);
		}
	
	buffer[0] = TMD2771_CMM_OFFSET;
	buffer[1] = offset_data; 
	res= i2c_master_send(client, buffer, 0x2);
	offset_flag =1;
	}
}

static ssize_t tmd2771_show_prox_3cm_calibrate_close(struct device_driver *ddri, char *buf)
{
	int res;
	printk("tmd2771_show_prox_3cm_calibrate_close\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	printk("tmd2771_show_prox_3cm_calibrate_close  %x??%d  \n", ps_cali.close, ps_cali.close);
	return scnprintf(buf, PAGE_SIZE, "0x%x\n", ps_cali.close);     	
}

static ssize_t tmd2771_store_prox_3cm_calibrate_close(struct device_driver *ddri, char *buf)
{
	int close;
	
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d", &close))
	{
		printk("---ps_threshold_high=%d\n", close);
	}
	ps_cali.close = close-15;
	ps_cali.far_away = ps_cali.close -CLOSE_AWAY_OFFSET-15;
printk(" [alsps] close =%d ,for_away=%d \n",ps_cali.close,ps_cali.far_away); 	
	tmd2771_obj->hw->ps_threshold_high = ps_cali.close;
	tmd2771_obj->hw->ps_threshold_low = ps_cali.far_away;
	atomic_set(&tmd2771_obj->ps_thd_val_high,  tmd2771_obj->hw->ps_threshold_high);
	atomic_set(&tmd2771_obj->ps_thd_val_low,  tmd2771_obj->hw->ps_threshold_low);
			

}

static ssize_t tmd2771_store_prox_3cm_calibrate_start(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	printk("tmd2771_store_prox_3cm_calibrate_start\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &value))
	printk("tmd2771_store_prox_3cm_calibrate_start calibrate_start_flag=%d\n",value);
	if(value == 1)
	{
		cali_3cm_status_flag = tmd2771_3cm_ps_calibrate();
	}
	return count;    
}



void tmd2771_store_prox_test(struct device_driver *ddri, const char *buf, size_t count){
APS_FUN();	
	int value;
	printk("tmd2771_store_prox_thres\n");
	int crosstalk=0;
	int threshold = 0;
	int cali_finish_flag = 0;
	
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}

	   cali_start_flag = 1;
	   
		printk("tmd2771_enable_als crosstalk=%d, threshold=%d , cali_finish_flag=%d\n",crosstalk,threshold,cali_finish_flag);
		if(cali_finish_flag ==1)
		{
		
		tmd2771_3cm_ps_calibrate();
		
		}
APS_FUN();		
}







static ssize_t tmd2771_store_prox_thres(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	printk("tmd2771_store_prox_thres\n");
	int crosstalk=0;
	int threshold = 0;
	int cali_finish_flag = 0;
	
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &value))
	{
		cali_finish_flag = value;
	}
	
	        cali_start_flag = 0;

	   printk("tmd2771_store_prox_calibrate_start cali_finish_flag=%d\n",cali_finish_flag);

	//	crosstalk = read_file("/persist/proxdata/crosstalk");
		threshold = read_file("/persist/proxdata/threshold");
	//	cali_finish_flag = read_file("/persist/proxdata/prox_thres");
		printk("tmd2771_enable_als crosstalk=%d, threshold=%d , cali_finish_flag=%d\n",crosstalk,threshold,cali_finish_flag);
		if(cali_finish_flag ==1)
		{
		
		tmd2771_3cm_ps_calibrate();
		//////////////////  hanguangce
			/*
			tmd2771_obj->hw->ps_threshold_high = ps_cali.close;
			tmd2771_obj->hw->ps_threshold_low = ps_cali.far_away;
			atomic_set(&tmd2771_obj->ps_thd_val_high,  tmd2771_obj->hw->ps_threshold_high);
			atomic_set(&tmd2771_obj->ps_thd_val_low,  tmd2771_obj->hw->ps_threshold_low);
			*/
		}
	return count;    
}

static ssize_t tmd2771_show_prox_offset_start(struct device_driver *ddri, char *buf)
{
	
}
static ssize_t tmd2771_store_prox_offset_start(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	
	struct tmd2771_priv *obj = i2c_get_clientdata(tmd2771_i2c_client);
	struct i2c_client * client = tmd2771_i2c_client;
	
	printk("tmd2771_store_prox_calibrate_start\n");
	if(!tmd2771_obj)
	{
		APS_ERR("tmd2771_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &value)){
		printk("tmd2771_store_prox_offset_start offset_start_flag=%d\n",offset_start_flag);
	offset_start_flag = value;
		
		tmd2771_enable_ps_for_cali(client,1); 
	}

	return count;    
}


static DRIVER_ATTR(prox_data_safe_range_max ,     0664, tmd2771_show_prox_data_safe_range_max , NULL);
static DRIVER_ATTR(prox_data_safe_range_min ,      0664, tmd2771_show_prox_data_safe_range_min, NULL);
static DRIVER_ATTR(prox_data_val ,  0664, tmd2771_show_data_val ,	NULL);
static DRIVER_ATTR(prox_offset_cal ,   0664, tmd2771_show_prox_offset_cal, tmd2771_store_prox_offset_cal);
static DRIVER_ATTR(prox_thres_max ,  0664, tmd2771_show_prox_thres_max, tmd2771_store_prox_thres_max);
static DRIVER_ATTR(prox_thres_min ,   0664, tmd2771_show_prox_thres_min,		tmd2771_store_prox_thres_min);
static DRIVER_ATTR(prox_data_max ,  0664, tmd2771_show_prox_data_max, NULL);
static DRIVER_ATTR(prox_thres ,    0664, tmd2771_show_prox_thres, tmd2771_store_prox_thres);
static DRIVER_ATTR(prox_calibrate_start  ,    0664, tmd2771_show_prox_calibrate_start, tmd2771_store_prox_calibrate_start);
static DRIVER_ATTR(prox_offset_start ,     0664, tmd2771_show_prox_offset_start, tmd2771_store_prox_offset_start);
static DRIVER_ATTR(prox_test ,     0664, NULL, tmd2771_store_prox_test);
static DRIVER_ATTR(prox_3cm_calibrate_start  ,    0664, NULL, tmd2771_store_prox_3cm_calibrate_start);
static DRIVER_ATTR(prox_3cm_calibrate_status  ,    0664, tmd2771_show_prox_3cm_calibrate_status, NULL);
static DRIVER_ATTR(prox_3cm_calibrate_offset  ,    0664, tmd2771_show_prox_3cm_calibrate_offset, tmd2771_store_prox_3cm_calibrate_offset);
static DRIVER_ATTR(prox_3cm_calibrate_close  ,    0664, tmd2771_show_prox_3cm_calibrate_close, tmd2771_store_prox_3cm_calibrate_close);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *tmd2771_attr_list[] = {
    &driver_attr_prox_data_safe_range_max,
    &driver_attr_prox_data_safe_range_min,    
    &driver_attr_prox_data_val,       
    &driver_attr_prox_offset_cal,
    &driver_attr_prox_thres_max,
    &driver_attr_prox_thres_min,
    &driver_attr_prox_data_max,
    &driver_attr_prox_thres,
    &driver_attr_prox_calibrate_start,
    &driver_attr_prox_3cm_calibrate_start,
    &driver_attr_prox_3cm_calibrate_status,
    &driver_attr_prox_3cm_calibrate_offset,
    &driver_attr_prox_3cm_calibrate_close,
	&driver_attr_prox_offset_start,
};

/*----------------------------------------------------------------------------*/
static int tmd2771_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(tmd2771_attr_list)/sizeof(tmd2771_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}
printk("[alsps ] tmd2771_create_attr \n");
	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, tmd2771_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", tmd2771_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int tmd2771_delete_attr(struct device_driver *driver)
	{
printk("[alsps ] tmd2771_create_attr \n");	
	int idx ,err = 0;
	int num = (int)(sizeof(tmd2771_attr_list)/sizeof(tmd2771_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, tmd2771_attr_list[idx]);
	}
	
	return err;
}

/*------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static long tmd2771_enable_als(struct i2c_client *client, int enable)
{
		struct tmd2771_priv *obj = i2c_get_clientdata(client);
		u8 databuf[2];	  
		long res = 0;
		//u8 buffer[1];
		//u8 reg_value[1];
		uint32_t testbit_PS;
		
	
		if(client == NULL)
		{
			APS_DBG("CLIENT CANN'T EQUL NULL\n");
			return -1;
		}
		
		#if 0	/*yucong MTK enable_als function modified for fixing reading register error problem 2012.2.16*/
		buffer[0]=TMD2771_CMM_ENABLE;
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, reg_value, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
		
		if(enable)
		{
			databuf[0] = TMD2771_CMM_ENABLE;	
			databuf[1] = reg_value[0] |0x0B;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			/*Lenovo-sw chenlj2 add 2011-06-03,modify ps to ALS below two lines */
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
			APS_DBG("tmd2771 power on\n");
		}
		else
		{
			databuf[0] = TMD2771_CMM_ENABLE;	
			databuf[1] = reg_value[0] &0xFD;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			/*Lenovo-sw chenlj2 add 2011-06-03,modify ps_deb_on to als_deb_on */
			atomic_set(&obj->als_deb_on, 0);
			APS_DBG("tmd2771 power off\n");
		}
		#endif
		#if 1
		/*yucong MTK enable_als function modified for fixing reading register error problem 2012.2.16*/
		testbit_PS = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
		if(enable)
		{
			if(testbit_PS){	
			databuf[0] = TMD2771_CMM_ENABLE;	
			databuf[1] = 0x2F;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			/*debug code for reading register value*/
			#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
			#endif
			}
			else{
			databuf[0] = TMD2771_CMM_ENABLE;	
			databuf[1] = 0x2B;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}

			/*debug code for reading register value*/
			#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
			#endif

			}
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
			APS_DBG("tmd2771 power on\n");
		}
		else
		{	
			if(testbit_PS){
			databuf[0] = TMD2771_CMM_ENABLE;	
			databuf[1] = 0x2D;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			}
			else{
			databuf[0] = TMD2771_CMM_ENABLE;	
			databuf[1] = 0x00;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			}
			/*Lenovo-sw chenlj2 add 2011-06-03,modify ps_deb_on to als_deb_on */
			atomic_set(&obj->als_deb_on, 0);
			APS_DBG("tmd2771 power off\n");
		}
		#endif
		#if 0 /*yucong add for debug*/
			buffer[0]=TMD2771_CMM_ENABLE;
			res = i2c_master_send(client, buffer, 0x1);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
		#endif
		
		return 0;
		
	EXIT_ERR:
		APS_ERR("tmd2771_enable_als fail\n");
		return res;
}

/*----------------------------------------------------------------------------*/
static long tmd2771_enable_ps(struct i2c_client *client, int enable)
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	long res = 0;
	u8 buffer[1];
	u8 reg_value[1];
	uint32_t testbit_ALS;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
#if 0	/*yucong MTK modified for fixing reading register error problem 2012.2.16*/
	buffer[0]=TMD2771_CMM_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	/*yucong MTK: lenovo orignal code*/
	if(enable)
	{
		databuf[0] = TMD2771_CMM_ENABLE;    
		databuf[1] = reg_value[0] |0x0d;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		APS_DBG("tmd2771 power on\n");

		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			if(1 == ps_cali.valid)
			{
				databuf[0] = TMD2771_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
				databuf[0] = TMD2771_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
				databuf[0] = TMD2771_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(ps_cali.close & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
				databuf[0] = TMD2771_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
			}
			else
			{
				databuf[0] = TMD2771_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)(480 & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
				databuf[0] = TMD2771_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)((480 & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
				databuf[0] = TMD2771_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(700 & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
				databuf[0] = TMD2771_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((700 & 0xFF00) >> 8);;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
		
			}
		
			databuf[0] = TMD2771_CMM_Persistence;
			databuf[1] = 0x20;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2771_ERR_I2C;
			}
			databuf[0] = TMD2771_CMM_ENABLE;	
			databuf[1] = reg_value[0] | 0x0d | 0x20;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2771_ERR_I2C;
			}
		
			mt_eint_unmask(CUST_EINT_ALS_NUM);
		}
	}
	else
	{
		databuf[0] = TMD2771_CMM_ENABLE;    
		databuf[1] = reg_value[0] &0xfb;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("tmd2771 power off\n");

		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			cancel_work_sync(&obj->eint_work);
			mt_eint_mask(CUST_EINT_ALS_NUM);
		}
	}
#endif
#if 1	
	/*yucong MTK: enable_ps function modified for fixing reading register error problem 2012.2.16*/
	testbit_ALS = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
	if(enable)
	{  
	       wake_lock(&ps_lock); //zhaoshaopeng add
		if(testbit_ALS){
		databuf[0] = TMD2771_CMM_ENABLE;    
		databuf[1] = 0x0F;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		/*debug code for reading register value*/
		#if 0
		res = i2c_master_recv(client, reg_value, 0x1);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
		#endif
		}else{
		databuf[0] = TMD2771_CMM_ENABLE;    
		databuf[1] = 0x0D;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
		/*debug code for reading register value*/
		#if 0
		res = i2c_master_recv(client, reg_value, 0x1);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
		#endif
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		APS_DBG("tmd2771 power on\n");

		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			if(1 == ps_cali.valid)
			{
				databuf[0] = TMD2771_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
				databuf[0] = TMD2771_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
				databuf[0] = TMD2771_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(ps_cali.close & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
				databuf[0] = TMD2771_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
			}
			else
			{
				databuf[0] = TMD2771_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)(750 & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
				databuf[0] = TMD2771_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)((750 & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
				databuf[0] = TMD2771_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(900 & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
				databuf[0] = TMD2771_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((900 & 0xFF00) >> 8);;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2771_ERR_I2C;
				}
		
			}
		
			databuf[0] = TMD2771_CMM_Persistence;
			databuf[1] = 0x20;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2771_ERR_I2C;
			}
			if(testbit_ALS){
			databuf[0] = TMD2771_CMM_ENABLE;    
			databuf[1] = 0x2F;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			/*debug code for reading register value*/
			#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
			#endif
			}else{
			databuf[0] = TMD2771_CMM_ENABLE;    
			databuf[1] = 0x2D;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			}
			/*debug code for reading register value*/
			#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
			#endif
		
			mt_eint_unmask(CUST_EINT_ALS_NUM);
		}
	}
	else
	{
	wake_unlock(&ps_lock);//zhaoshaopeng 
	/*yucong MTK: enable_ps function modified for fixing reading register error problem 2012.2.16*/
	if(testbit_ALS){
		databuf[0] = TMD2771_CMM_ENABLE;    
		databuf[1] = 0x2B;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}else{
		databuf[0] = TMD2771_CMM_ENABLE;    
		databuf[1] = 0x00;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("tmd2771 power off\n");

		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			cancel_work_sync(&obj->eint_work);
			mt_eint_mask(CUST_EINT_ALS_NUM);
		}
	}
#endif
	return 0;
	
EXIT_ERR:
	APS_ERR("tmd2771_enable_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
#ifdef OPEN_PROX_ARITHMETIC // fenggy open
static int tmd2771_enable(struct i2c_client *client, int enable)
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	/* modify to restore reg setting after cali ---liaoxl.lenovo */
	buffer[0]=TMD2771_CMM_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	if(enable)
	{
		databuf[0] = TMD2771_CMM_ENABLE;    
		databuf[1] = reg_value[0] | 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_DBG("tmd2771 power on\n");
	}
	else
	{
		databuf[0] = TMD2771_CMM_ENABLE;    
		databuf[1] = reg_value[0] & 0xFE;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		/*Lenovo-sw chenlj2 add 2011-06-03,close als_deb_on */
		atomic_set(&obj->als_deb_on, 0);
		APS_DBG("tmd2771 power off\n");
	}
	return 0;
	
EXIT_ERR:
	APS_ERR("tmd2771_enable fail\n");
	return res;
}
#endif

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static int tmd2771_check_and_clear_intr(struct i2c_client *client) 
{
	//struct tmd2771_priv *obj = i2c_get_clientdata(client);
	int res,intp,intl;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;

	buffer[0] = TMD2771_CMM_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong tmd2771_check_and_clear_intr status=0x%x\n", buffer[0]);
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;		
	}

	if(0 == res)
	{
		if((1 == intp) && (0 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
		}
		else if((0 == intp) && (1 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
		}
		else
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
		}
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		else
		{
			res = 0;
		}
	}

	return res;

EXIT_ERR:
	APS_ERR("tmd2771_check_and_clear_intr fail\n");
	return 1;
}
/*----------------------------------------------------------------------------*/

/*yucong add for interrupt mode support MTK inc 2012.3.7*/
static int tmd2771_check_intr(struct i2c_client *client) 
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);
	int res,intp,intl;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;

	buffer[0] = TMD2771_CMM_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//APS_ERR("tmd2771_check_and_clear_intr status=0x%x\n", buffer[0]);
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;		
	}

	return res;

EXIT_ERR:
	APS_ERR("tmd2771_check_intr fail\n");
	return 1;
}

static int tmd2771_clear_intr(struct i2c_client *client) 
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 buffer[2];

#if 0
	if((1 == intp) && (0 == intl))
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
	}
	else if((0 == intp) && (1 == intl))
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
	}
	else
#endif
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
	}
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}

	return res;

EXIT_ERR:
	APS_ERR("tmd2771_check_and_clear_intr fail\n");
	return 1;
}


/*-----------------------------------------------------------------------------*/
void tmd2771_eint_func(void)
{
	struct tmd2771_priv *obj = g_tmd2771_ptr;
	if(!obj)
	{
		return;
	}
	
	schedule_work(&obj->eint_work);
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int tmd2771_setup_eint(struct i2c_client *client)
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);        

	g_tmd2771_ptr = obj;
	

    mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, tmd2771_eint_func, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);
    return 0;
}

/*----------------------------------------------------------------------------*/

#ifdef OPEN_PROX_ARITHMETIC // fenggy open
static int tmd2771_init_client_for_cali(struct i2c_client *client)
{

	struct tmd2771_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = TMD2771_CMM_ENABLE;    
	databuf[1] = 0x05;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}
	
	databuf[0] = TMD2771_CMM_ATIME;    
	databuf[1] = 0xEE;//0xEE
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}

	databuf[0] = TMD2771_CMM_PTIME;    
	databuf[1] = 0xFF;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}

	databuf[0] = TMD2771_CMM_WTIME;    
	databuf[1] = 0xFF;//0xFF
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}

	

	databuf[0] = TMD2771_CMM_PPCOUNT;    
	databuf[1] = TMD2771_CMM_PPCOUNT_VALUE;//0x02
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}

	databuf[0] = TMD2771_CMM_CONTROL;    
	databuf[1] = TMD2771_CMM_CONTROL_VALUE;//0x22
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}
	databuf[0] = TMD2771_CMM_ENABLE;	
		databuf[1] = 0x0F;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2771_ERR_I2C;
		}

	return TMD2771_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;

}
#endif

static int tmd2771_init_client(struct i2c_client *client)
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = TMD2771_CMM_ENABLE;    
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}
	
	databuf[0] = TMD2771_CMM_ATIME;    
	databuf[1] = 0xC9;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}

	databuf[0] = TMD2771_CMM_PTIME;    
	databuf[1] = 0xFF;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}

	databuf[0] = TMD2771_CMM_WTIME;    
	databuf[1] = 0xEE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(0 == obj->hw->polling_mode_ps)
	{
		if(1 == ps_cali.valid)
		{
			databuf[0] = TMD2771_CMM_INT_LOW_THD_LOW;	
			databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2771_ERR_I2C;
			}
			databuf[0] = TMD2771_CMM_INT_LOW_THD_HIGH;	
			databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2771_ERR_I2C;
			}
			databuf[0] = TMD2771_CMM_INT_HIGH_THD_LOW;	
			databuf[1] = (u8)(ps_cali.close & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2771_ERR_I2C;
			}
			databuf[0] = TMD2771_CMM_INT_HIGH_THD_HIGH;	
			databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2771_ERR_I2C;
			}
		}
		else
		{
			databuf[0] = TMD2771_CMM_INT_LOW_THD_LOW;	
			databuf[1] = (u8)(750 & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2771_ERR_I2C;
			}
			databuf[0] = TMD2771_CMM_INT_LOW_THD_HIGH;	
			databuf[1] = (u8)((750 & 0xFF00) >> 8);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2771_ERR_I2C;
			}
			databuf[0] = TMD2771_CMM_INT_HIGH_THD_LOW;	
			databuf[1] = (u8)(900 & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2771_ERR_I2C;
			}
			databuf[0] = TMD2771_CMM_INT_HIGH_THD_HIGH;	
			databuf[1] = (u8)((900 & 0xFF00) >> 8);;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2771_ERR_I2C;
			}

		}

		databuf[0] = TMD2771_CMM_Persistence;
		databuf[1] = 0x20;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2771_ERR_I2C;
		}
		databuf[0] = TMD2771_CMM_ENABLE;	
		databuf[1] = 0x20;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2771_ERR_I2C;
		}

	}

	databuf[0] = TMD2771_CMM_CONFIG;    
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}

       /*Lenovo-sw chenlj2 add 2011-06-03,modified pulse 2  to 4 */
	databuf[0] = TMD2771_CMM_PPCOUNT;    
	databuf[1] = TMD2771_CMM_PPCOUNT_VALUE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}

        /*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16  to 1 */
	databuf[0] = TMD2771_CMM_CONTROL;    
	databuf[1] = TMD2771_CMM_CONTROL_VALUE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}
       /*ADD BY EMILY*/
	/*databuf[0] = TMD2771_CMM_OFFSET;    
	databuf[1] = TMD2771_OFFSET;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}
	*/
	
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if((res = tmd2771_setup_eint(client))!=0)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	if(res = tmd2771_check_and_clear_intr(client))
	{
		APS_ERR("check/clear intr: %d\n", res);
		//    return res;
	}
	
	return TMD2771_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
int tmd2771_read_als(struct i2c_client *client, u16 *data)
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);	 
	u16 c0_value, c1_value;	 
	u32 c0_nf, c1_nf;
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	u16 atio;
	//u16 als_value;
	int res = 0;
	u8 reg_value[1];
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	/*debug tag for yucong*/
	#if 0
	buffer[0]=TMD2771_CMM_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
	#endif
//get adc channel 0 value
	buffer[0]=TMD2771_CMM_C0DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2771_CMM_C0DATA_L = 0x%x\n", als_value_low[0]);

	buffer[0]=TMD2771_CMM_C0DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2771_CMM_C0DATA_H = 0x%x\n", als_value_high[0]);
	c0_value = als_value_low[0] | (als_value_high[0]<<8);
	c0_nf = obj->als_modulus*c0_value;
	//APS_DBG("c0_value=%d, c0_nf=%d, als_modulus=%d\n", c0_value, c0_nf, obj->als_modulus);

//get adc channel 1 value
	buffer[0]=TMD2771_CMM_C1DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2771_CMM_C1DATA_L = 0x%x\n", als_value_low[0]);	

	buffer[0]=TMD2771_CMM_C1DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2771_CMM_C1DATA_H = 0x%x\n", als_value_high[0]);	

	c1_value = als_value_low[0] | (als_value_high[0]<<8);
	c1_nf = obj->als_modulus*c1_value;	
	//APS_DBG("c1_value=%d, c1_nf=%d, als_modulus=%d\n", c1_value, c1_nf, obj->als_modulus);

	if((c0_value > c1_value) &&(c0_value < 50000))
	{  	/*Lenovo-sw chenlj2 add 2011-06-03,add {*/
		atio = (c1_nf*100)/c0_nf;

	//APS_DBG("atio = %d\n", atio);
	if(atio<30)
	{
		*data = (13*c0_nf - 24*c1_nf)/10000;
	}
	else if(atio>= 30 && atio<38) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (16*c0_nf - 35*c1_nf)/10000;
	}
	else if(atio>= 38 && atio<45)  /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (9*c0_nf - 17*c1_nf)/10000;
	}
	else if(atio>= 45 && atio<54) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (6*c0_nf - 10*c1_nf)/10000;
	}
	else
		*data = 0;
	/*Lenovo-sw chenlj2 add 2011-06-03,add }*/
    }
	else if (c0_value > 50000)
	{
		*data = 65535;
	}
	else if(c0_value == 0)
        {
                *data = 0;
        }
        else
	{
		APS_DBG("als_value is invalid!!\n");
		return -1;
	}	
	APS_DBG("als_value_lux = %d\n", *data);
	//printk("yucong: als_value_lux = %d\n", *data);
	return 0;	 

	
	
EXIT_ERR:
	APS_ERR("tmd2771_read_ps fail\n");
	return res;
}
int tmd2771_read_als_ch0(struct i2c_client *client, u16 *data)
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);	 
	u16 c0_value;	 
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	int res = 0;
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
//get adc channel 0 value
	buffer[0]=TMD2771_CMM_C0DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	buffer[0]=TMD2771_CMM_C0DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	c0_value = als_value_low[0] | (als_value_high[0]<<8);
	*data = c0_value;
	return 0;	 

	
	
EXIT_ERR:
	APS_ERR("tmd2771_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/

static int tmd2771_get_als_value(struct tmd2771_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		//APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		//APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
long tmd2771_read_ps(struct i2c_client *client, u16 *data)
{
	
printk(" [alsps]  tmd2771_read_ps start \n ");	
	struct tmd2771_priv *obj = i2c_get_clientdata(client);    
	//u16 ps_value;    
	u8 ps_value_low[1], ps_value_high[1];
	u8 buffer[1];
	long res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]=TMD2771_CMM_PDATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	buffer[0]=TMD2771_CMM_PDATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	*data = ps_value_low[0] | (ps_value_high[0]<<8);
	printk("  [als_ps]  ps_data= %d  ", *data);
	
	
printk(" [alsps]  tmd2771_read_ps end \n ");	
	//APS_DBG("ps_data=%d, low:%d  high:%d", *data, ps_value_low[0], ps_value_high[0]);
	return 0;    

EXIT_ERR:
	APS_ERR("tmd2771_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int tmd2771_get_ps_value(struct tmd2771_priv *obj, u16 ps)
{
	int val;// mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	static int val_temp=1;
	 /*Lenovo-sw chenlj2 add 2011-10-12 begin*/
	 u16 temp_ps[1];
	 /*Lenovo-sw chenlj2 add 2011-10-12 end*/
	 
	
	//APS_LOG("tmd2771_get_ps_value  1 %d," ,ps_cali.close);
	//APS_LOG("tmd2771_get_ps_value  2 %d," ,ps_cali.far_away);
	//APS_LOG("tmd2771_get_ps_value  3 %d,", ps_cali.valid);

	//APS_LOG("tmd2771_get_ps_value  ps %d,", ps);
    /*Lenovo-sw zhuhc delete 2011-10-12 begin*/
	//return 1;
    /*Lenovo-sw zhuhc delete 2011-10-12 end*/

        mdelay(160);
	tmd2771_read_ps(obj->client,temp_ps);
	if(ps_cali.valid == 1)
		{
			//APS_LOG("tmd2771_get_ps_value val_temp  = %d",val_temp);
			if((ps >ps_cali.close)&&(temp_ps[0] >ps_cali.close))
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
			}
			else if((ps <ps_cali.far_away)&&(temp_ps[0] < ps_cali.far_away))
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
			}
			else
				val = val_temp;

			//APS_LOG("tmd2771_get_ps_value val  = %d",val);
	}
	else
	{
			if((ps > atomic_read(&obj->ps_thd_val_high))&&(temp_ps[0]  > atomic_read(&obj->ps_thd_val_high)))
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
			}
			else if((ps < atomic_read(&obj->ps_thd_val_low))&&(temp_ps[0]  < atomic_read(&obj->ps_thd_val_low)))
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
			}
			else
			       val = val_temp;	
			
	}
	
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	
	else if (obj->als > 45000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}
printk(" [alsps] vcalid =%d , close= %d, for_away= %d",ps_cali.valid,ps_cali.close,ps_cali.far_away);
	if(!invalid)
	{
		//APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}


/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static void tmd2771_eint_work(struct work_struct *work)
{
	struct tmd2771_priv *obj = (struct tmd2771_priv *)container_of(work, struct tmd2771_priv, eint_work);
	int err;
	hwm_sensor_data sensor_data;
	u8 buffer[1];
	u8 reg_value[1];
	u8 databuf[2];
	int res = 0;

	if((err = tmd2771_check_intr(obj->client)))
	{
		APS_ERR("tmd2771_eint_work check intrs: %d\n", err);
	}
	else
	{
		//get raw data
		tmd2771_read_ps(obj->client, &obj->ps);
		//mdelay(160);
		tmd2771_read_als_ch0(obj->client, &obj->als);
//		APS_ERR("tmd2771_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
//		APS_ERR("tmd2771_eint_work ps_thd_val_low=0x%x,ps_thd_val_high=0x%x\n",
//			obj->ps_thd_val_low,obj->ps_thd_val_high);
		//printk("tmd2771_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		sensor_data.values[0] = tmd2771_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			
/*singal interrupt function add*/
#if 1
		if(intr_flag_value){
				//printk("yucong interrupt value ps will < 750");
				databuf[0] = TMD2771_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2771_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				
//				APS_ERR("tmd2771_eint_work ps_thd_val_low=0x%x",obj->ps_thd_val_low);
				
				databuf[0] = TMD2771_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2771_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((0xFF00) >> 8);;
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
		else{	
				//printk("yucong interrupt value ps will > 900");
				databuf[0] = TMD2771_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)(0 & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2771_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)((0 & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2771_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2771_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
#endif
		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
	tmd2771_clear_intr(obj->client);
	mt_eint_unmask(CUST_EINT_ALS_NUM);      
}


/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int tmd2771_open(struct inode *inode, struct file *file)
{
	file->private_data = tmd2771_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tmd2771_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}



#ifdef OPEN_PROX_ARITHMETIC // fenggy open
static void tmd2771_WriteCalibration(struct PS_CALI_DATA_STRUCT *data_cali)
{

	   APS_LOG("tmd2771_WriteCalibration  1 %d," ,data_cali->close);
		   APS_LOG("tmd2771_WriteCalibration  2 %d," ,data_cali->far_away);
		   APS_LOG("tmd2771_WriteCalibration  3 %d,", data_cali->valid);
		   
	  if(data_cali->valid == 1)
	  {
	      if(data_cali->close < 100)
	      	{
		  	ps_cali.close = 200;
			ps_cali.far_away= 150;
			ps_cali.valid = 1;
	      	}
		  else if(data_cali->close > 900)
		  {
		  	ps_cali.close = 900;
			ps_cali.far_away= 750;
			ps_cali.valid = 1;
	      	}
		  else
		  {
			  ps_cali.close = data_cali->close;
			ps_cali.far_away= data_cali->far_away;
			ps_cali.valid = 1;
		  }
	  }
	  

}
#endif

#ifdef OPEN_PROX_ARITHMETIC // fenggy open
static int tmd2771_read_data_for_cali(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali)
{
     int i=0 ,err = 0,j = 0;
	 u16 data[22]={0},sum = 0,data_cali=0;

	 for(i = 0;i<=20;i++)
	 {
	 		mdelay(5);//50
			if(err = tmd2771_read_ps(client,&data[i]))
			{
				APS_ERR("tmd2771_read_data_for_cali fail: %d\n", i); 
				break;
			}
//			else
//			{
//					sum += data[i];
//			}
			mdelay(55);//160
	 }
	 APS_ERR("tmd7221 20 data:\n");
	 
	 for(j = 1;j<=20;j++){
	 	APS_ERR("[data%d]:%d,",j,data[j]);
		sum += data[j];
	 }
	 if(i == 21)
	 {
			data_cali = sum/20;
			APS_ERR("tmd2771_read_data_for_cali data = %d",data_cali);
//			if(data_cali>600)
//				return -1;
			
			if(data_cali<=100)
			{
				ps_data_cali->close =data_cali*23/10;
				ps_data_cali->far_away = data_cali*20/10;
//				ps_data_cali->close =data_cali*16/10;
//				ps_data_cali->far_away = data_cali*13/10;

				ps_data_cali->valid =1;
			}
			else if(100<data_cali&&data_cali<300)
			{
				ps_data_cali->close = data_cali*21/10;
				ps_data_cali->far_away =data_cali*19/10;
//				ps_data_cali->close = data_cali*15/10;
//				ps_data_cali->far_away =data_cali*12/10;

				ps_data_cali->valid = 1;
			}
			else
			{
				ps_data_cali->close = data_cali*19/10;
				ps_data_cali->far_away =data_cali*18/10;
//				ps_data_cali->close = data_cali*14/10;
//				ps_data_cali->far_away =data_cali*11/10;

				ps_data_cali->valid = 1;
			}
		        //zhaoshaopeng  from fae 20120615
		       #if 1
            if(ps_data_cali->close > 1000)
            {
			  	ps_data_cali->close = 1000;
				ps_data_cali->far_away = 950;
				err= 0;
	        }
                     #else
			if(ps_data_cali->close > 900)
			{
				ps_data_cali->close = 500;
				ps_data_cali->far_away = 350;
				err= 0;
			}
			#endif
			//zhaoshaopeng end
			else  if(ps_data_cali->close < 300)
			{
			   ps_data_cali->close = 400;
			   ps_data_cali->far_away = 350;
			   err= 0;
			}

			ps_cali.close = ps_data_cali->close;
			ps_cali.far_away= ps_data_cali->far_away;
			ps_cali.valid = 1;

			tmd2771_obj->hw->ps_threshold_high = ps_cali.close;
			tmd2771_obj->hw->ps_threshold_low = ps_cali.far_away;
			atomic_set(&tmd2771_obj->ps_thd_val_high,  tmd2771_obj->hw->ps_threshold_high);
			atomic_set(&tmd2771_obj->ps_thd_val_low,  tmd2771_obj->hw->ps_threshold_low);
			
			APS_ERR("tmd2771_read_data_for_cali close  = %d %d %d ",ps_data_cali->close,ps_data_cali->far_away,ps_data_cali->valid);
	
	 	}
	 else
	 {
	 	ps_data_cali->valid = 0;
	 	err=  -1;
	 }
	 return err;
	 	

}


//fenggy add ?¡§23?¡§o???£¤¡§o?¨¤?¨º??¡§?2¡§|??£¤???¡è?¡ì
int tmd2771_get_ps_rawdata(void)
{
	struct tmd2771_priv *obj = tmd2771_obj;  
	struct PS_CALI_DATA_STRUCT ps_cali_temp; 
	long err = 0;
	//zhaoshaopeng
	//tmd2771_init_client_for_cali(obj->client);
	err = tmd2771_init_client_for_cali(obj->client);
	if(err)
	{
		APS_ERR("tmd2771_init_client_for_cali error,return\n");
		return -1;
	}
	else
	{
           err = 0;
	}
	err = tmd2771_read_data_for_cali(obj->client,&ps_cali_temp);
	if(err)
	{
		APS_ERR("tmd2771_read_data_for_cali error,set default vale\n");
		return -1;
	}
	err = tmd2771_init_client(obj->client);
	if(err)
	{
		APS_ERR("tmd2771_init_client error\n");
	   return -1;
	}	
	// tmd2771_enable_ps(obj->client, 1);
	err = tmd2771_enable(obj->client, 0);
	if(err)
	{
		APS_ERR("tmd2771_enable error\n");
	   return -1;
	}		
	return 0;
}

//fenggy end
#endif


/*----------------------------------------------------------------------------*/
static long tmd2771_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct tmd2771_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	struct PS_CALI_DATA_STRUCT ps_cali_temp;

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = tmd2771_enable_ps(obj->client, 1))
				{
		//			APS_ERR("enable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if(err = tmd2771_enable_ps(obj->client, 0))
				{
//					APS_ERR("disable ps fail: %d\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if(err = tmd2771_read_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}
			
			dat = tmd2771_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if(err = tmd2771_read_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = tmd2771_enable_als(obj->client, 1))
				{
//					APS_ERR("enable als fail: %d\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if(err = tmd2771_enable_als(obj->client, 0))
				{
//					APS_ERR("disable als fail: %d\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if(err = tmd2771_read_als(obj->client, &obj->als))
			{
				goto err_out;
			}

			dat = tmd2771_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if(err = tmd2771_read_als(obj->client, &obj->als))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

/*		case ALSPS_SET_PS_CALI:
			dat = (void __user*)arg;
			if(dat == NULL)
			{
				APS_LOG("dat == NULL\n");
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&ps_cali_temp,dat, sizeof(ps_cali_temp)))
			{
				APS_LOG("copy_from_user\n");
				err = -EFAULT;
				break;	  
			}
			tmd2771_WriteCalibration(&ps_cali_temp);
			APS_LOG(" ALSPS_SET_PS_CALI %d,%d,%d\t",ps_cali_temp.close,ps_cali_temp.far_away,ps_cali_temp.valid);
			break;
		case ALSPS_GET_PS_RAW_DATA_FOR_CALI:
			tmd2771_init_client_for_cali(obj->client);
			err = tmd2771_read_data_for_cali(obj->client,&ps_cali_temp);
			if(err)
			{
			   goto err_out;
			}
			tmd2771_init_client(obj->client);
			// tmd2771_enable_ps(obj->client, 1);
			tmd2771_enable(obj->client, 0);
			if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
*/
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}


/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	//struct tmd2771_priv *obj = i2c_get_clientdata(client);    
	//int err;
	APS_FUN();    
#if 0
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		if(err = tmd2771_enable_als(client, 0))
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		atomic_set(&obj->ps_suspend, 1);
		if(err = tmd2771_enable_ps(client, 0))
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		tmd2771_power(obj->hw, 0);
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_resume(struct i2c_client *client)
{
	//struct tmd2771_priv *obj = i2c_get_clientdata(client);        
	//int err;
	APS_FUN();
#if 0
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	tmd2771_power(obj->hw, 1);
	if(err = tmd2771_init_client(client))
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = tmd2771_enable_als(client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		if(err = tmd2771_enable_ps(client, 1))
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static void tmd2771_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct tmd2771_priv *obj = container_of(h, struct tmd2771_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	#if 1
	atomic_set(&obj->als_suspend, 1);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = tmd2771_enable_als(obj->client, 0))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
	}
	#endif
}
/*----------------------------------------------------------------------------*/
static void tmd2771_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct tmd2771_priv *obj = container_of(h, struct tmd2771_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

        #if 1
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = tmd2771_enable_als(obj->client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
	#endif
}

int tmd2771_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct tmd2771_priv *obj = (struct tmd2771_priv *)self;
	
	//APS_FUN();
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
					if(err = tmd2771_enable_ps(obj->client, 1))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
					#if 0	
					if(err = tmd2771_enable_als(obj->client, 1))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
					#endif
				}
				else
				{
					if(err = tmd2771_enable_ps(obj->client, 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
					#if 0
					if(err = tmd2771_enable_als(obj->client, 0))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
					#endif
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;	
				tmd2771_read_ps(obj->client, &obj->ps);
				
                                //mdelay(160);
				tmd2771_read_als_ch0(obj->client, &obj->als);
				APS_ERR("tmd2771_ps_operate als data=%d!\n",obj->als);
				sensor_data->values[0] = tmd2771_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int tmd2771_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct tmd2771_priv *obj = (struct tmd2771_priv *)self;

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
					if(err = tmd2771_enable_als(obj->client, 1))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if(err = tmd2771_enable_als(obj->client, 0))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				/*yucong MTK add for fixing know issue*/
				#if 1
				tmd2771_read_als(obj->client, &obj->als);
				if(obj->als == 0)
				{
					sensor_data->values[0] = 0;//zhaoshaopeng from -1;				
				}else{
					u16 b[2];
					int i;
					for(i = 0;i < 2;i++){
					tmd2771_read_als(obj->client, &obj->als);
					b[i] = obj->als;
					}
					(b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
					sensor_data->values[0] = tmd2771_get_als_value(obj, obj->als);
				}
				#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, TMD2771_DEV_NAME);
	return 0;
}

void tmd2771_read_data(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali)
{
     int i=0 ,err = 0,j = 0;
	 u16 data[22]={0},sum = 0,data_cali=0;

	 for(i = 0;i<=20;i++)
	 {
	 		mdelay(5);//50
			if(err = tmd2771_read_ps(client,&data[i]))
			{
				APS_ERR("tmd2771_read_data fail: %d\n", i); 
				break;
			}
			mdelay(55);//160
	 }
	 APS_ERR("tmd7221 20 data:\n");
	 
	 for(j = 1;j<=20;j++){
	 	APS_ERR("[data%d]:%d,",j,data[j]);
		sum += data[j];
	 }
	 if(i == 21)
	 {
	 		ps_data_cali->valid = 1;
			data_cali = sum/20;
			APS_ERR("tmd2771_read_data data = %d",data_cali);
			ps_data_cali->close = data_cali;
			ps_data_cali->far_away = data_cali;
//			if(data_cali<=100)
//			{
//				ps_data_cali->close =data_cali*16/10;
//				ps_data_cali->far_away = data_cali*13/10;

//				ps_data_cali->valid =1;
//			}
//			else if(100<data_cali&&data_cali<300)
//			{
//				ps_data_cali->close = data_cali*15/10;
//				ps_data_cali->far_away =data_cali*12/10;

//				ps_data_cali->valid = 1;
//			}
//			else
//			{
//				ps_data_cali->close = data_cali*14/10;
//				ps_data_cali->far_away =data_cali*11/10;

//				ps_data_cali->valid = 1;
//			}
//			
//			APS_ERR("tmd2771_read_data close  = %d,far_away = %d,valid = %d",ps_data_cali->close,ps_data_cali->far_away,ps_data_cali->valid);
	 }
	 else
	 {
	 	ps_data_cali->valid = 0;
	 }
}

int tmd2771_close_away_proc_write( struct file *filp, const char __user *buf,unsigned long len, void *data )
{
		char str_buf[256]={0};
		int close,away;
		
		copy_from_user(str_buf,buf,len);

		sscanf(str_buf,"%d:%d",&close,&away);
		
		printk("jacob test tmd2771_close_away_proc_write str_buf=%s,close=%d,away=%d\n",
			str_buf,close,away);

		
		ps_cali.close = close;
		ps_cali.far_away= away;
		ps_cali.valid = 1;
		
		tmd2771_obj->hw->ps_threshold_high = ps_cali.close;
		tmd2771_obj->hw->ps_threshold_low = ps_cali.far_away;
		atomic_set(&tmd2771_obj->ps_thd_val_high,  tmd2771_obj->hw->ps_threshold_high);
		atomic_set(&tmd2771_obj->ps_thd_val_low,  tmd2771_obj->hw->ps_threshold_low);

		return len;	
}

int tmd2771_close_away_proc_read(char *buf, char **start, off_t offset, int request, int *eof, void *data)
{
		int	 len=0;
		int err= 1;
		struct PS_CALI_DATA_STRUCT tmp_data;
		if(err = tmd2771_enable_ps(tmd2771_i2c_client, 1))
		{
	//		APS_ERR("enable ps fail: %ld\n", err); 
			goto err_out;
		}
		
		tmd2771_read_data(tmd2771_i2c_client,&tmp_data);
		tmd2771_enable_ps(tmd2771_i2c_client, 0);
			
		printk("jacob test tmd2771_close_away_proc_read valid=%d,close=%d\n",
			tmp_data.valid,tmp_data.close);
		
		len = sprintf(buf,"%d:%d",tmp_data.valid,tmp_data.close);
		return len;
err_out:
		tmp_data.valid = 0;
		len = sprintf(buf,"%d:%d",tmp_data.valid,tmp_data.close);
		return len;
}



/*----------------------------------------------------------------------------*/
static struct file_operations tmd2771_fops = {
	.owner = THIS_MODULE,
	.open = tmd2771_open,
	.release = tmd2771_release,
    .write = tmd2771_close_away_proc_write,
    .read = tmd2771_close_away_proc_read,	
	.unlocked_ioctl = tmd2771_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tmd2771_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &tmd2771_fops,
};
/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tmd2771_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	tmd2771_obj = obj;

	obj->hw = tmd2771_get_cust_alsps_hw();
	tmd2771_get_addr(obj->hw, &obj->addr);

	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	INIT_WORK(&obj->eint_work, tmd2771_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 200);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);  
	/*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16 to 1/5 accoring to actual thing */
	obj->als_modulus = (400*100*ZOOM_TIME)/(1*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100 //16
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	//zhaoshaopeng add
       wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");
	//zhaoshaopeng end
	tmd2771_i2c_client = client;

//fenggy add	
#ifdef	OPEN_PROX_ARITHMETIC
	err = tmd2771_get_ps_rawdata();
	if(err)
	{
		APS_ERR("tmd2771_get_ps_rawdata error ,set the defult vaule\n");
		if(err = tmd2771_init_client(client))
		{
			goto exit_init_failed;
		}
	}
#else	
	if(err = tmd2771_init_client(client))
	{
		goto exit_init_failed;
	}
#endif	
//fenggy add end


	APS_ERR("tmd2771_init_client() OK!\n");

	if(err = misc_register(&tmd2771_device))
	{
		APS_ERR("tmd2771_device register failed\n");
		goto exit_misc_device_register_failed;
	}

//	proc_tmd2771_close_away = proc_create( "tmd2771closeaway", 0777, NULL,&tmd2771_fops);
	//proc_tmd2771_close_away->write  = tmd2771_close_away_proc_write;
	//proc_tmd2771_close_away->read = tmd2771_close_away_proc_read;
	
	obj_ps.self = tmd2771_obj;
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(1 == obj->hw->polling_mode_ps)
	{
		obj_ps.polling = 1;
	}
	else
	{
		obj_ps.polling = 0;
	}

    
	if((err = tmd2771_create_attr(&(tmd2771_init_info.platform_diver_addr->driver))))
	{
printk("[alsps ] tmd2771  probe 3 \n");
	
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	
printk("[alsps ] tmd2771  probe 4 \n");

	obj_ps.self = tmd2771_obj;
	obj_ps.sensor_operate = tmd2771_ps_operate;
	if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = tmd2771_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = tmd2771_als_operate;
	if(err = hwmsen_attach(ID_LIGHT, &obj_als))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = tmd2771_early_suspend,
	obj->early_drv.resume   = tmd2771_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

	/* Add for auto detect feature */	
    tmd2771_init_flag = 0;

	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&tmd2771_device);
	exit_misc_device_register_failed:
	exit_init_failed:
//	cancel_delayed_work(&obj->eint_work);
	cancel_work_sync(&obj->eint_work);
	if(0 == obj->hw->polling_mode_ps)
	{
		mt_eint_mask(CUST_EINT_ALS_NUM);
	}
	//i2c_detach_client(client);
	//exit_kfree:
	kfree(obj);
	exit:
	tmd2771_i2c_client = NULL;  
	/* Add for auto detect feature */
	tmd2771_init_flag = -1;
//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_remove(struct i2c_client *client)
{
	int err;	
	
	if(err = tmd2771_delete_attr(&tmd2771_i2c_driver.driver))
	{
		APS_ERR("tmd2771_delete_attr fail: %d\n", err);
	} 

	remove_proc_entry("tmd2771closeaway",proc_tmd2771_close_away);

	if(err = misc_deregister(&tmd2771_device))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	tmd2771_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
#if 0 /* Delete for auto detect feature */
static int tmd2771_probe(struct platform_device *pdev) 
{
	struct tmd2772_alsps_hw *hw = tmd2771_get_cust_alsps_hw();

	tmd2771_power(hw, 1);    
	//tmd2771_force[0] = hw->i2c_num;
	//tmd2771_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",tmd2771_force[0],tmd2771_force[1]);
	if(i2c_add_driver(&tmd2771_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tmd2771_remove(struct platform_device *pdev)
{
	struct tmd2772_alsps_hw *hw = tmd2771_get_cust_alsps_hw();
	APS_FUN();    
	tmd2771_power(hw, 0);    
	i2c_del_driver(&tmd2771_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver tmd2771_alsps_driver = {
	.probe      = tmd2771_probe,
	.remove     = tmd2771_remove,    
	.driver     = {
		.name  = "als_ps",
//		.owner = THIS_MODULE,
	}
};
#else
static int  tmd2771_local_init(void)
{
	struct tmd2772_alsps_hw *hw = tmd2771_get_cust_alsps_hw();

	tmd2771_power(hw, 1); 
	APS_ERR("tmd2771_local_init...\n");
//	tmd2771_force[0] = hw->i2c_num;
//	tmd2771_force[1] = hw->i2c_addr[0];
//	APS_DBG("I2C = %d, addr =0x%x\n",tmd2771_force[0],tmd2771_force[1]);
	if(i2c_add_driver(&tmd2771_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	if(-1 == tmd2771_init_flag)		
	{	   		
		return -1;		
	}
	return 0;
}
static int tmd2771_remove(void)
{
	struct tmd2772_alsps_hw *hw = tmd2771_get_cust_alsps_hw();
	APS_FUN();    
	tmd2771_power(hw, 0);    
	i2c_del_driver(&tmd2771_i2c_driver);
	return 0;
}

#endif


/*----------------------------------------------------------------------------*/
static int __init tmd2771_init(void)
{
	APS_FUN();
	i2c_register_board_info(2, &i2c_TMD2771, 1);
#if 0 /* Modify for auto detect feature */	
	if(platform_driver_register(&tmd2771_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
#else	
	APS_ERR("tmd2771_init...\n");
	alsps_driver_add(&tmd2771_init_info);	
#endif	
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit tmd2771_exit(void)
{
	APS_FUN();
#if 0	/* Modify for auto detect feature */	
	platform_driver_unregister(&tmd2771_alsps_driver);
#endif
}
/*----------------------------------------------------------------------------*/
module_init(tmd2771_init);
module_exit(tmd2771_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("tmd2771 driver");
MODULE_LICENSE("GPL");
