/*****************************************************************************
 *
 * Filename:
 * ---------
 *     t4ka3mipiraw_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include <linux/types.h>


#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "t4ka3mipiraw_Sensor.h"

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

#define PFX "t4ka3_4lane_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineprintk(format, args...) xlog_printk(ANDROID_printkO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#define printk(format, args...)	xlog_printk(ANDROID_printkO   , PFX, "[%s] " format, __FUNCTION__, ##args)


static DEFINE_SPINLOCK(imgsensor_drv_lock);

static int flag = 0;

#define Vender 0x10 //for Seasons ID
//#define Vender 0x07 //for Oflim ID


#define T4KA3_OTP_PSEL 0x4A02
#define T4KA3_OTP_CTRL 0x4A00

#define T4KA3_OTP_DATA_BEGIN_ADDR 0x4A04
#define T4KA3_OTP_DATA_END_ADDR   0x4A43
//static uint16_t T4KA3_OTP_DATA[T4KA3_OTP_DATA_END_ADDR - T4KA3_OTP_DATA_BEGIN_ADDR + 1] = {0x00};
//static uint16_t T4KA3_OTP_DATA_BACKUP[T4KA3_OTP_DATA_END_ADDR - T4KA3_OTP_DATA_BEGIN_ADDR + 1] = {0x00};

//static kal_uint16 t4ka3_r_golden_value=0x54; //golden module  AWB R value
//static kal_uint16 t4ka3_g_golden_value=0x92; //golden module  AWB G value: (Gr+Gb)/2
//static kal_uint16 t4ka3_b_golden_value=0x5A; //golden module  AWB B value, 

static kal_uint16 t4ka3_r_golden_value=0x56; //golden module  AWB R value
static kal_uint16 t4ka3_g_golden_value=0x95; //golden module  AWB G value: (Gr+Gb)/2
static kal_uint16 t4ka3_b_golden_value=0x5b; //golden module  AWB B value, 

static kal_uint16 t4ka3_r_cur_value=0; //current module  AWB R value
static kal_uint16 t4ka3_g_cur_value=0; //current module  AWB G value: (Gr+Gb)/2
static kal_uint16 t4ka3_b_cur_value=0; //current module  AWB B value
static kal_uint16 t4ka3_r_gain=0; 
static kal_uint16 t4ka3_b_gain=0; 


static imgsensor_info_struct imgsensor_info = {
    .sensor_id = T4KA3MIPI_SENSOR_ID,        //record sensor id defined in Kd_imgsensor.h

    .checksum_value = 0xeab06ccb,        //checksum value for Camera Auto Test
	
	.pre = {
        .pclk = 259200000,                //record different mode's pclk
		.linelength  = 3440,				//record different mode's linelength
        .framelength = 2492,            //record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1632,		//record different mode's width of grabwindow
		.grabwindow_height = 1224,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 259200000,
		.linelength  = 3440,
        .framelength = 2492,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	.cap1 = {/*PIP capture*/ //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 259200000,
        .linelength = 3440,
        .framelength = 2492,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 240,	//less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps  
	},
	.normal_video = {
        .pclk = 259200000,
        .linelength = 3508,
        .framelength = 2488,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 300,
	},
	.hs_video = {/*slow motion*/
        .pclk = 259200000,
        .linelength = 3440,
        .framelength = 2492,
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		/*	 following for  MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	.slim_video = {
        .pclk = 259200000,
        .linelength = 3440,
        .framelength = 2492,
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1632,		//record different mode's width of grabwindow
		.grabwindow_height = 1224,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	
    .margin = 6,            //sensor framelength & shutter margin
	.min_shutter = 1,		//min shutter
	.max_frame_length = 0xffff,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
    .ae_sensor_gain_delay_frame = 1,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 1,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	
    .cap_delay_frame = 1,        //enter capture delay frame num
    .pre_delay_frame = 2,         //enter preview delay frame num
    .video_delay_frame = 2,        //enter video delay frame num
    .hs_video_delay_frame = 2,    //enter high speed video  delay frame num
    .slim_video_delay_frame = 2,//enter slim video delay frame num

    .isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,//sensor output first pixel color
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
    .i2c_addr_table = {0x6c, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,                //mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x4EA,                    //current shutter
    .gain = 0x40,                        //current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
    .current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x6c,//record current sensor's i2c write id
};


/* Sensor output window information */

static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = 
{{ 3264, 2448, 0, 0, 3264, 2448, 1632, 1224, 0, 0, 1632, 1224,  0, 0, 1632, 1224}, // Preview 
 { 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0, 0, 3264, 2448}, // capture 
 { 3264, 2448, 0, 0, 3264, 2448, 3264, 1836, 0, 0, 3264, 1836,  0, 0, 1632, 1224}, // video 
 { 3264, 2448, 0, 0, 3232, 2448, 3264, 2448, 0, 0, 1636 ,1224,  0, 0, 1632, 1224}, //hight speed video 
 { 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 1636, 1224,  0, 0, 1632, 1224}};// slim video

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}
/*
static int32_t t4ka3_otp_read_data(uint16_t* otp_data)
{
    kal_uint16 i = 0;
	
    for (i = 0; i <= (T4KA3_OTP_DATA_END_ADDR - T4KA3_OTP_DATA_BEGIN_ADDR); i++)
	{
        otp_data[i]=read_cmos_sensor(T4KA3_OTP_DATA_BEGIN_ADDR+i);
		printk("otp_data=0x%x\n",otp_data[i]);
    }

    return 0;
}
*/
static void set_dummy(void)
{
	printk("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);
  
}	/*	set_dummy  */
/*
static kal_uint32 return_sensor_id(void)
{
    return ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
}*/
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
//	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	printk("framerate = %d \n", framerate);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
//	kal_uint32 frame_length = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	//write_shutter(shutter);
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	/* OV Recommend Solution
	*  if shutter bigger than frame_length, should extend frame length first
	*/
	
	
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
		else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	}

	// Update Shutter
	write_cmos_sensor(0x0202, shutter >> 8);
	write_cmos_sensor(0x0203, shutter & 0xFF);
	
	printk("Exit! autoflicker_en =%d, margin =%d, min_shutter =%d, min_frame_length =%d, max_frame_length =%d\n",imgsensor.autoflicker_en, imgsensor_info.margin, imgsensor_info.min_shutter, imgsensor.min_frame_length, imgsensor_info.max_frame_length);
	printk("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}
/*
static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	
    reg_gain = ((gain / BASEGAIN) << 4) + ((gain % BASEGAIN) * 16 / BASEGAIN);
    reg_gain = reg_gain & 0xFFFF;
	return (kal_uint16)reg_gain;
}*/

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;
	kal_uint32 temp;
  
  temp = (kal_uint32)(gain)*128*1000;
  temp = temp/64/1000;
  reg_gain = (kal_uint16)(temp);
	//reg_gain = (gain/BASEGAIN)*128;  //0x80 is 1x gain
	
	if(reg_gain<128) reg_gain = 128;
	else if(reg_gain>1024) reg_gain = 1024;
		
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    printk("[t4kb3mipiraw ]ispgain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

    write_cmos_sensor(0x0234, (reg_gain>>8)& 0xFF);
    write_cmos_sensor(0x0235, reg_gain & 0xFF);
	printk("set_gain %x \n", gain);

    return gain;

}    /*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
#if 0
	printk("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);

	
	if (imgsensor.ihdr_en) {
		
		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)		
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;
			
			
		// Extend frame length first
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);

		write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
   		write_cmos_sensor(0x0203, le & 0xFF);

		write_cmos_sensor(0x0234, ((se) >> 8) & 0xFF);
        write_cmos_sensor(0x0235, (se) & 0xFF);

		set_gain(gain);
	}
#endif
}


/*
static void set_mirror_flip(kal_uint8 image_mirror)
{
	printk("image_mirror = %d\n", image_mirror);

//return 0;
*/
	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
/*
#if 0
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x06));
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x06));		
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
			break;
		default:
			printk("Error image_mirror setting\n");
	}
#endif

}
*/
/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/

static void sensor_init(void)
{
	printk("sensor_init Enter\n");
// T4KA3 Initial 3264x2448_30FPS_MIPI_4LANE(Binning)
// Register Information File
// made from Register setter ver600

	// begin
	write_cmos_sensor(0x0103,0x00);//software rst

	
	write_cmos_sensor(0x0101,0x00); // -/-/-/-/-/V_Flip/H_Mirror
	write_cmos_sensor(0x4136,0x18); // EXTCLK_FRQ_MHZ[15:8];
	write_cmos_sensor(0x4137,0x00); // EXTCLK_FRQ_MHZ[7:0];
	write_cmos_sensor(0x3094,0x01);//Reserved; // 2014-12-26  INIT_Ver.13  from Register setter ver600
	write_cmos_sensor(0x0233,0x01);//Reserved;
	write_cmos_sensor(0x4B06,0x01);//Reserved;
	write_cmos_sensor(0x4B07,0x01);//Reserved;
	write_cmos_sensor(0x3028,0x01);//Reserved;
	write_cmos_sensor(0x3032,0x14);//Reserved;
	write_cmos_sensor(0x305C,0x0C);//Reserved;
	write_cmos_sensor(0x306D,0x0A);//Reserved;
	write_cmos_sensor(0x3071,0xFA);//Reserved;
	write_cmos_sensor(0x307E,0x0A);//Reserved;
	write_cmos_sensor(0x307F,0xFC);//Reserved;
	write_cmos_sensor(0x3091,0x04);//Reserved;
	write_cmos_sensor(0x3092,0x60);//Reserved;
	write_cmos_sensor(0x3096,0xC0);//Reserved;
	write_cmos_sensor(0x3139,0x06);//Reserved;
	write_cmos_sensor(0x313A,0x06);//Reserved;
	write_cmos_sensor(0x313B,0x04);//Reserved;
	write_cmos_sensor(0x3143,0x02);//Reserved;
	write_cmos_sensor(0x314F,0x0E);//Reserved;
	write_cmos_sensor(0x3169,0x99);//Reserved;
	write_cmos_sensor(0x316A,0x99);//Reserved;
	write_cmos_sensor(0x3171,0x05);//Reserved;
	write_cmos_sensor(0x31A1,0xA7);//Reserved;
	write_cmos_sensor(0x31A2,0x9C);//Reserved;
	write_cmos_sensor(0x31A3,0x8F);//Reserved;
	write_cmos_sensor(0x31A4,0x75);//Reserved;
	write_cmos_sensor(0x31A5,0xEE);//Reserved;
	write_cmos_sensor(0x31A6,0xEA);//Reserved;
	write_cmos_sensor(0x31A7,0xE4);//Reserved;
	write_cmos_sensor(0x31A8,0xE4);//Reserved;
	write_cmos_sensor(0x31DF,0x05);//Reserved;
	write_cmos_sensor(0x31EC,0x1B);//Reserved;
	write_cmos_sensor(0x31ED,0x1B);//Reserved;
	write_cmos_sensor(0x31EE,0x1B);//Reserved;
	write_cmos_sensor(0x31F0,0x1B);//Reserved;
	write_cmos_sensor(0x31F1,0x1B);//Reserved;
	write_cmos_sensor(0x31F2,0x1B);//Reserved;
	write_cmos_sensor(0x3204,0x3F);//Reserved;
	write_cmos_sensor(0x3205,0x03);//Reserved;
	write_cmos_sensor(0x3210,0x01);//Reserved;
	write_cmos_sensor(0x3212,0x00);//Reserved;
	write_cmos_sensor(0x3216,0x68);//Reserved;
	write_cmos_sensor(0x3217,0x58);//Reserved;
	write_cmos_sensor(0x3218,0x58);//Reserved;
	write_cmos_sensor(0x321A,0x68);//Reserved;
	write_cmos_sensor(0x321B,0x60);//Reserved;
	write_cmos_sensor(0x3238,0x03);//Reserved;
	write_cmos_sensor(0x3239,0x03);//Reserved;
	write_cmos_sensor(0x323A,0x05);//Reserved;
	write_cmos_sensor(0x323B,0x06);//Reserved;
	write_cmos_sensor(0x3243,0x03);//-/-/-/-/-/-/LSC_EN/SHD_GRID_EN;
	write_cmos_sensor(0x3244,0x08);//Reserved;
	write_cmos_sensor(0x3245,0x01);//Reserved;
	write_cmos_sensor(0x3307,0x19);//Reserved;
	write_cmos_sensor(0x3308,0x19);//Reserved;
	write_cmos_sensor(0x3320,0x01);//Reserved;
	write_cmos_sensor(0x3326,0x15);//Reserved;
	write_cmos_sensor(0x3327,0x0D);//Reserved;
	write_cmos_sensor(0x3328,0x01);//Reserved;
	write_cmos_sensor(0x3380,0x01);//Reserved;
	write_cmos_sensor(0x3394,0x23);//Reserved;
	write_cmos_sensor(0x339E,0x07);//Reserved;
	write_cmos_sensor(0x3424,0x00);//Reserved;
	write_cmos_sensor(0x343C,0x01);//Reserved;
	write_cmos_sensor(0x35E0,0xB0);//Reserved;
	write_cmos_sensor(0x35E1,0x90);//Reserved;
	write_cmos_sensor(0x35E2,0x88);//Reserved;
	write_cmos_sensor(0x35E3,0xB0);//Reserved;
	write_cmos_sensor(0x35E4,0x90);//Reserved;
	write_cmos_sensor(0x35E5,0x08);//Reserved;
	write_cmos_sensor(0x3398,0x04);//Reserved;
	write_cmos_sensor(0x343A,0x10);//Reserved;
	write_cmos_sensor(0x339A,0x22);//Reserved;
	write_cmos_sensor(0x33B4,0x00);//Reserved;
    write_cmos_sensor(0x3393,0x01);//Reserved;
	write_cmos_sensor(0x33B3,0x6E);//Reserved;
	write_cmos_sensor(0x3433,0x06);//Reserved;
	write_cmos_sensor(0x3433,0x00);//Reserved;
	write_cmos_sensor(0x33B3,0x00);//Reserved;
	write_cmos_sensor(0x3393,0x03);//Reserved;
	write_cmos_sensor(0x33B4,0x03);//Reserved;
	write_cmos_sensor(0x343A,0x00);//Reserved;
	write_cmos_sensor(0x339A,0x00);//Reserved;
	write_cmos_sensor(0x3398,0x00);//Reserved;	//	2014-12-26
	write_cmos_sensor(0x3427,0x03);//Reserved; //add for discontinue mode A
	
	//write_cmos_sensor(0x3438,0x04);//

	write_cmos_sensor(0x0112,0x0A);//CSI_DATA_FORMAT[15:8];
	write_cmos_sensor(0x0113,0x0A);//CSI_DATA_FORMAT[7:0];				 ;
	write_cmos_sensor(0x0114,0x03);//CSI_LANE_MODE[1:0];
	write_cmos_sensor(0x4136,0x18);//EXTCLK_FRWQ_MHZ[15:8];
	write_cmos_sensor(0x4137,0x00);//EXTCLK_FRWQ_MHZ[7:0];
	write_cmos_sensor(0x0820,0x0A);//MSB_LBRATE[31:24];
	write_cmos_sensor(0x0821,0x20);//MSB_LBRATE[23:16];
	write_cmos_sensor(0x0822,0x00);//MSB_LBRATE[15:8];
	write_cmos_sensor(0x0823,0x00);//MSB_LBRATE[7:0];
	write_cmos_sensor(0x0301,0x0A);//-/-/-/VT_PIX_CLK_DIV[4:0];
	write_cmos_sensor(0x0303,0x01);//-/-/-/-/VT_SYS_CLK_DIV[3:0];
	write_cmos_sensor(0x0305,0x04);//-/-/-/-/PRE_PLL_CLK_DIV[3:0];
	write_cmos_sensor(0x0306,0x01);//-/-/-/-/-/PLL_MULTIPLIER[10:8];
	write_cmos_sensor(0x0307,0xB0);//PLL_MULTIPLIER[7:0];
	write_cmos_sensor(0x030B,0x01);//-/-/OP_SYS_CLK_DIV[5:0];

	write_cmos_sensor(0x0202,0x09);//
	write_cmos_sensor(0x0203,0xF0);//
	write_cmos_sensor(0x0234,0x00);//
	write_cmos_sensor(0x0235,0x80);//
	
	write_cmos_sensor(0x034C,0x0C);//X_OUTPUT_SIZE[15:8];
	write_cmos_sensor(0x034D,0xC0);//X_OUTPUT_SIZE[7:0];
	write_cmos_sensor(0x034E,0x09);//Y_OUTPUT_SIZE[15:8];
	write_cmos_sensor(0x034F,0x8E);//Y_OUTPUT_SIZE[7:0];
	write_cmos_sensor(0x0340,0x09);//FR_LENGTH_LINES[15:8];
	write_cmos_sensor(0x0341,0xBC);//FR_LENGTH_LINES[7:0];
	write_cmos_sensor(0x0342,0x0D);//LINE_LENGTH_PCK[15:8];
	write_cmos_sensor(0x0343,0x70);//LINE_LENGTH_PCK[7:0];
	write_cmos_sensor(0x0344,0x00);//X_ADDR_START[15:8];
	write_cmos_sensor(0x0345,0x00);//X_ADDR_START[7:0];
	write_cmos_sensor(0x0346,0x00);//Y_ADDR_START[15:8];
	write_cmos_sensor(0x0347,0x00);//Y_ADDR_START[7:0];
	write_cmos_sensor(0x0348,0x0c);//X_ADDR_END[15:8];
	write_cmos_sensor(0x0349,0xcf);//X_ADDR_END[7:0];
	write_cmos_sensor(0x034A,0x09);//Y_ADDR_END[15:8];
	write_cmos_sensor(0x034B,0x9F);//Y_ADDR_END[7:0];
	write_cmos_sensor(0x0408,0x00);//DCROP_XOFS[15:8];
	write_cmos_sensor(0x0409,0x08);//DCROP_XOFS[7:0];
	write_cmos_sensor(0x040A,0x00);//DCROP_YOFS[15:8];
	write_cmos_sensor(0x040B,0x0A);//DCROP_YOFS[7:0];
	write_cmos_sensor(0x040C,0x0c);//DCROP_WIDTH[15:8];
	write_cmos_sensor(0x040D,0xc0);//DCROP_WIDTH[7:0];
	write_cmos_sensor(0x040E,0x09);//DCROP_HIGT[15:8];
	write_cmos_sensor(0x040F,0x90);//DCROP_HIGT[7:0];
	write_cmos_sensor(0x0900,0x01);//-/-/-/-/-/-/-/BINNING_MODE;
	write_cmos_sensor(0x0901,0x11);//BINNING_TYPE[7:0];
	write_cmos_sensor(0x0902,0x00);//-/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	write_cmos_sensor(0x4220,0x00);//-/-/-/HDR_MODE[4:0];
	write_cmos_sensor(0x4222,0x01);//HDR_RATIO[7:0];
	write_cmos_sensor(0x3245,0x01);//Reserverd G_ON
	write_cmos_sensor(0x3380,0x01);//HDR_MODE2[0]
	write_cmos_sensor(0x0100,0x01);//MODE2_SELECT[0]

	printk("sensor_init Exit\n");

}	/*	sensor_init  */


static void preview_setting(void)
{
	printk("preview_setting Enter\n");
    write_cmos_sensor(0x0100,0x00);//standby mode
    mdelay(50);
   //t4ka3 preview size 1632x1224 30fps
   write_cmos_sensor(0x034C,0x06);//X_OUTPUT_SIZE[15:8];
   write_cmos_sensor(0x034D,0x60);//X_OUTPUT_SIZE[7:0];
   write_cmos_sensor(0x034E,0x04);//Y_OUTPUT_SIZE[15:8];
   write_cmos_sensor(0x034F,0xc6);//Y_OUTPUT_SIZE[7:0];
	 
   write_cmos_sensor(0x0340,((imgsensor_info.pre.framelength >> 8) & 0xFF));//; VTS H
   write_cmos_sensor(0x0341,(imgsensor_info.pre.framelength & 0xFF));//; VTS L
   write_cmos_sensor(0x0342,((imgsensor_info.pre.linelength >> 8) & 0xFF));//; HTS H
   write_cmos_sensor(0x0343,(imgsensor_info.pre.linelength & 0xFF));//; HTS L

   write_cmos_sensor(0x0344,0x00);//X_ADDR_START[15:8];
   write_cmos_sensor(0x0345,0x00);//X_ADDR_START[7:0];
   write_cmos_sensor(0x0346,0x00);//Y_ADDR_START[15:8];
   write_cmos_sensor(0x0347,0x00);//Y_ADDR_START[7:0];
   write_cmos_sensor(0x0348,0x0c);//X_ADDR_END[15:8];
   write_cmos_sensor(0x0349,0xcf);//X_ADDR_END[7:0];
   write_cmos_sensor(0x034A,0x09);//Y_ADDR_END[15:8];
   write_cmos_sensor(0x034B,0x9F);//Y_ADDR_END[7:0];
   write_cmos_sensor(0x0408,0x00);//DCROP_XOFS[15:8];
   write_cmos_sensor(0x0409,0x04);//DCROP_XOFS[7:0];
   write_cmos_sensor(0x040A,0x00);//DCROP_YOFS[15:8];
   write_cmos_sensor(0x040B,0x06);//DCROP_YOFS[7:0];
   write_cmos_sensor(0x040C,0x06);//DCROP_WIDTH[15:8];
   write_cmos_sensor(0x040D,0x60);//DCROP_WIDTH[7:0];
   write_cmos_sensor(0x040E,0x04);//DCROP_HIGT[15:8];
   write_cmos_sensor(0x040F,0xc8);//DCROP_HIGT[7:0];
   write_cmos_sensor(0x0900,0x01);//-/-/-/-/-/-/-/BINNING_MODE;
   write_cmos_sensor(0x0901,0x22);//BINNING_TYPE[7:0];
   write_cmos_sensor(0x0902,0x00);//-/-/-/-/-/-/BINNING_WEIGHTING[1:0];
   write_cmos_sensor(0x4220,0x00);//-/-/-/HDR_MODE[4:0];
   write_cmos_sensor(0x4222,0x01);//HDR_RATIO[7:0];
   write_cmos_sensor(0x3245,0x01);//Reserverd G_ON
   write_cmos_sensor(0x3380,0x01);//HDR_MODE2[0]
	   
    write_cmos_sensor(0x0100,0x01);//MODE2_SELECT[0]

	printk("preview_setting Exit\n");
	mdelay(40);

}	/*	preview_setting  */


static void capture_setting(kal_uint16 currefps)
{
	 //t4ka3 capture size 3264x2448 30fps
	  write_cmos_sensor(0x0100,0x00);//standby mode
    mdelay(10);
	 write_cmos_sensor(0x034C,0x0C);//X_OUTPUT_SIZE[15:8];
	 write_cmos_sensor(0x034D,0xC0);//X_OUTPUT_SIZE[7:0];
	 write_cmos_sensor(0x034E,0x09);//Y_OUTPUT_SIZE[15:8];
	 write_cmos_sensor(0x034F,0x8E);//Y_OUTPUT_SIZE[7:0];
	 
	 write_cmos_sensor(0x0340,((imgsensor_info.pre.framelength >> 8) & 0xFF));//; VTS H
	 write_cmos_sensor(0x0341,(imgsensor_info.pre.framelength & 0xFF));//; VTS L
	 write_cmos_sensor(0x0342,((imgsensor_info.pre.linelength >> 8) & 0xFF));//; HTS H
	 write_cmos_sensor(0x0343,(imgsensor_info.pre.linelength & 0xFF));//; HTS L
	 
	 write_cmos_sensor(0x0344,0x00);//X_ADDR_START[15:8];
	 write_cmos_sensor(0x0345,0x00);//X_ADDR_START[7:0];
	 write_cmos_sensor(0x0346,0x00);//Y_ADDR_START[15:8];
	 write_cmos_sensor(0x0347,0x00);//Y_ADDR_START[7:0];
	 write_cmos_sensor(0x0348,0x0c);//X_ADDR_END[15:8];
	 write_cmos_sensor(0x0349,0xcf);//X_ADDR_END[7:0];
	 write_cmos_sensor(0x034A,0x09);//Y_ADDR_END[15:8];
	 write_cmos_sensor(0x034B,0x9F);//Y_ADDR_END[7:0];
	 write_cmos_sensor(0x0408,0x00);//DCROP_XOFS[15:8];
	 write_cmos_sensor(0x0409,0x08);//DCROP_XOFS[7:0];
	 write_cmos_sensor(0x040A,0x00);//DCROP_YOFS[15:8];
	 write_cmos_sensor(0x040B,0x0A);//DCROP_YOFS[7:0];
	 write_cmos_sensor(0x040C,0x0c);//DCROP_WIDTH[15:8];
	 write_cmos_sensor(0x040D,0xc0);//DCROP_WIDTH[7:0];
	 write_cmos_sensor(0x040E,0x09);//DCROP_HIGT[15:8];
	 write_cmos_sensor(0x040F,0x90);//DCROP_HIGT[7:0];
	 write_cmos_sensor(0x0900,0x01);//-/-/-/-/-/-/-/BINNING_MODE;
	 write_cmos_sensor(0x0901,0x11);//BINNING_TYPE[7:0];
	 write_cmos_sensor(0x0902,0x00);//-/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	 write_cmos_sensor(0x4220,0x00);//-/-/-/HDR_MODE[4:0];
	 write_cmos_sensor(0x4222,0x01);//HDR_RATIO[7:0];
	 write_cmos_sensor(0x3245,0x01);//Reserverd G_ON
	 write_cmos_sensor(0x3380,0x01);//HDR_MODE2[0]

	 write_cmos_sensor(0x0100,0x01);//steam on //liuying
	 mdelay(40);
}

static void normal_video_setting(kal_uint16 currefps)
{
	 
	 //t4ka3 normal video size 1632x1224 30fps
	  write_cmos_sensor(0x0100,0x00);//standby mode
    mdelay(10);
	 write_cmos_sensor(0x034C,0x06);//X_OUTPUT_SIZE[15:8];
	 write_cmos_sensor(0x034D,0x60);//X_OUTPUT_SIZE[7:0];
	 write_cmos_sensor(0x034E,0x04);//Y_OUTPUT_SIZE[15:8];
	 write_cmos_sensor(0x034F,0xc6);//Y_OUTPUT_SIZE[7:0];
	 
	 write_cmos_sensor(0x0340,((imgsensor_info.pre.framelength >> 8) & 0xFF));//; VTS H
	 write_cmos_sensor(0x0341,(imgsensor_info.pre.framelength & 0xFF));//; VTS L
	 write_cmos_sensor(0x0342,((imgsensor_info.pre.linelength >> 8) & 0xFF));//; HTS H
	 write_cmos_sensor(0x0343,(imgsensor_info.pre.linelength & 0xFF));//; HTS L
	 
	 write_cmos_sensor(0x0344,0x00);//X_ADDR_START[15:8];
	 write_cmos_sensor(0x0345,0x00);//X_ADDR_START[7:0];
	 write_cmos_sensor(0x0346,0x00);//Y_ADDR_START[15:8];
	 write_cmos_sensor(0x0347,0x00);//Y_ADDR_START[7:0];
	 write_cmos_sensor(0x0348,0x0c);//X_ADDR_END[15:8];
	 write_cmos_sensor(0x0349,0xcf);//X_ADDR_END[7:0];
	 write_cmos_sensor(0x034A,0x09);//Y_ADDR_END[15:8];
	 write_cmos_sensor(0x034B,0x9F);//Y_ADDR_END[7:0];
	 write_cmos_sensor(0x0408,0x00);//DCROP_XOFS[15:8];
	 write_cmos_sensor(0x0409,0x04);//DCROP_XOFS[7:0];
	 write_cmos_sensor(0x040A,0x00);//DCROP_YOFS[15:8];
	 write_cmos_sensor(0x040B,0x06);//DCROP_YOFS[7:0];
	 write_cmos_sensor(0x040C,0x06);//DCROP_WIDTH[15:8];
	 write_cmos_sensor(0x040D,0x60);//DCROP_WIDTH[7:0];
	 write_cmos_sensor(0x040E,0x04);//DCROP_HIGT[15:8];
	 write_cmos_sensor(0x040F,0xc8);//DCROP_HIGT[7:0];
	 write_cmos_sensor(0x0900,0x01);//-/-/-/-/-/-/-/BINNING_MODE;
	 write_cmos_sensor(0x0901,0x22);//BINNING_TYPE[7:0];
	 write_cmos_sensor(0x0902,0x00);//-/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	 write_cmos_sensor(0x4220,0x00);//-/-/-/HDR_MODE[4:0];
	 write_cmos_sensor(0x4222,0x01);//HDR_RATIO[7:0];
	 write_cmos_sensor(0x3245,0x01);//Reserverd G_ON
	 write_cmos_sensor(0x3380,0x01);//HDR_MODE2[0]

	write_cmos_sensor(0x0100,0x01);//steam on //liuying
	mdelay(40);
	printk("E! currefps:%d\n",currefps);

}
static void hs_video_setting(void)
{
	 //t4ka3 hs video size 1632x1224 30fps
	  write_cmos_sensor(0x0100,0x00);//standby mode
    mdelay(10);
	 write_cmos_sensor(0x034C,0x06);//X_OUTPUT_SIZE[15:8];
	 write_cmos_sensor(0x034D,0x60);//X_OUTPUT_SIZE[7:0];
	 write_cmos_sensor(0x034E,0x04);//Y_OUTPUT_SIZE[15:8];
	 write_cmos_sensor(0x034F,0xc6);//Y_OUTPUT_SIZE[7:0];
	 
	 write_cmos_sensor(0x0340,((imgsensor_info.pre.framelength >> 8) & 0xFF));//; VTS H
	 write_cmos_sensor(0x0341,(imgsensor_info.pre.framelength & 0xFF));//; VTS L
	 write_cmos_sensor(0x0342,((imgsensor_info.pre.linelength >> 8) & 0xFF));//; HTS H
	 write_cmos_sensor(0x0343,(imgsensor_info.pre.linelength & 0xFF));//; HTS L
	 
	 write_cmos_sensor(0x0344,0x00);//X_ADDR_START[15:8];
	 write_cmos_sensor(0x0345,0x00);//X_ADDR_START[7:0];
	 write_cmos_sensor(0x0346,0x00);//Y_ADDR_START[15:8];
	 write_cmos_sensor(0x0347,0x00);//Y_ADDR_START[7:0];
	 write_cmos_sensor(0x0348,0x0c);//X_ADDR_END[15:8];
	 write_cmos_sensor(0x0349,0xcf);//X_ADDR_END[7:0];
	 write_cmos_sensor(0x034A,0x09);//Y_ADDR_END[15:8];
	 write_cmos_sensor(0x034B,0x9F);//Y_ADDR_END[7:0];
	 write_cmos_sensor(0x0408,0x00);//DCROP_XOFS[15:8];
	 write_cmos_sensor(0x0409,0x04);//DCROP_XOFS[7:0];
	 write_cmos_sensor(0x040A,0x00);//DCROP_YOFS[15:8];
	 write_cmos_sensor(0x040B,0x06);//DCROP_YOFS[7:0];
	 write_cmos_sensor(0x040C,0x06);//DCROP_WIDTH[15:8];
	 write_cmos_sensor(0x040D,0x60);//DCROP_WIDTH[7:0];
	 write_cmos_sensor(0x040E,0x04);//DCROP_HIGT[15:8];
	 write_cmos_sensor(0x040F,0xc8);//DCROP_HIGT[7:0];
	 write_cmos_sensor(0x0900,0x01);//-/-/-/-/-/-/-/BINNING_MODE;
	 write_cmos_sensor(0x0901,0x22);//BINNING_TYPE[7:0];
	 write_cmos_sensor(0x0902,0x00);//-/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	 write_cmos_sensor(0x4220,0x00);//-/-/-/HDR_MODE[4:0];
	 write_cmos_sensor(0x4222,0x01);//HDR_RATIO[7:0];
	 write_cmos_sensor(0x3245,0x01);//Reserverd G_ON
	 write_cmos_sensor(0x3380,0x01);//HDR_MODE2[0]

	write_cmos_sensor(0x0100,0x01);//steam on //liuying
	mdelay(40);
//	printk("E! currefps:%d\n",currefps);

}

static void slim_video_setting(void)
{
	 //t4ka3 slim video size 1632x1224 30fps
	   write_cmos_sensor(0x0100,0x00);//standby mode
    mdelay(10);
	 write_cmos_sensor(0x034C,0x06);//X_OUTPUT_SIZE[15:8];
	 write_cmos_sensor(0x034D,0x60);//X_OUTPUT_SIZE[7:0];
	 write_cmos_sensor(0x034E,0x04);//Y_OUTPUT_SIZE[15:8];
	 write_cmos_sensor(0x034F,0xc6);//Y_OUTPUT_SIZE[7:0];
	 
	 write_cmos_sensor(0x0340,((imgsensor_info.pre.framelength >> 8) & 0xFF));//; VTS H
	 write_cmos_sensor(0x0341,(imgsensor_info.pre.framelength & 0xFF));//; VTS L
	 write_cmos_sensor(0x0342,((imgsensor_info.pre.linelength >> 8) & 0xFF));//; HTS H
	 write_cmos_sensor(0x0343,(imgsensor_info.pre.linelength & 0xFF));//; HTS L
	 
	 write_cmos_sensor(0x0344,0x00);//X_ADDR_START[15:8];
	 write_cmos_sensor(0x0345,0x00);//X_ADDR_START[7:0];
	 write_cmos_sensor(0x0346,0x00);//Y_ADDR_START[15:8];
	 write_cmos_sensor(0x0347,0x00);//Y_ADDR_START[7:0];
	 write_cmos_sensor(0x0348,0x0c);//X_ADDR_END[15:8];
	 write_cmos_sensor(0x0349,0xcf);//X_ADDR_END[7:0];
	 write_cmos_sensor(0x034A,0x09);//Y_ADDR_END[15:8];
	 write_cmos_sensor(0x034B,0x9F);//Y_ADDR_END[7:0];
	 write_cmos_sensor(0x0408,0x00);//DCROP_XOFS[15:8];
	 write_cmos_sensor(0x0409,0x04);//DCROP_XOFS[7:0];
	 write_cmos_sensor(0x040A,0x00);//DCROP_YOFS[15:8];
	 write_cmos_sensor(0x040B,0x06);//DCROP_YOFS[7:0];
	 write_cmos_sensor(0x040C,0x06);//DCROP_WIDTH[15:8];
	 write_cmos_sensor(0x040D,0x60);//DCROP_WIDTH[7:0];
	 write_cmos_sensor(0x040E,0x04);//DCROP_HIGT[15:8];
	 write_cmos_sensor(0x040F,0xc8);//DCROP_HIGT[7:0];
	 write_cmos_sensor(0x0900,0x01);//-/-/-/-/-/-/-/BINNING_MODE;
	 write_cmos_sensor(0x0901,0x22);//BINNING_TYPE[7:0];
	 write_cmos_sensor(0x0902,0x00);//-/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	 write_cmos_sensor(0x4220,0x00);//-/-/-/HDR_MODE[4:0];
	 write_cmos_sensor(0x4222,0x01);//HDR_RATIO[7:0];
	 write_cmos_sensor(0x3245,0x01);//Reserverd G_ON
	 write_cmos_sensor(0x3380,0x01);//HDR_MODE2[0]

	write_cmos_sensor(0x0100,0x01);//steam on //liuying
	mdelay(40);
//	printk("E! currefps:%d\n",currefps);
	
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    printk("enable: %d\n", enable);

return 0;
    if (enable) {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x0601, 0x02);
    } else {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x0601, 0x00);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static void t4ka3_otp_read_enable(kal_uint8 enable)
{
   if(enable)
   	write_cmos_sensor(T4KA3_OTP_CTRL,0x01);
   else
   	write_cmos_sensor(T4KA3_OTP_CTRL,0x00);
}

static void t4ka3_otp_set_page(kal_uint16 page)
{
   write_cmos_sensor(T4KA3_OTP_PSEL,page);
}

static void t4ka3_otp_start_read(void)
{
  write_cmos_sensor(T4KA3_OTP_CTRL,(read_cmos_sensor(T4KA3_OTP_CTRL) | 0x80));
}

static void t4ka3_otp_checkMag(void)
{
  
  kal_uint8 checkMag;
  
  t4ka3_otp_set_page(0x0f);
  t4ka3_otp_start_read();
  checkMag = read_cmos_sensor(0x4A1C);

  if(checkMag != 0x99)
  	{
	  write_cmos_sensor(0x35EC,0x99);//Mag_ADJ[7:0]
	  printk("checkMag error! modify checkMag=0x%x\n",read_cmos_sensor(0x0205));
  	}
  else if(checkMag == 0x99)
  	{
  	  printk("checkMag correct!");
  	}
  
}

static int t4ka3_otp_update_lsc(void)
{
	 kal_uint8 lsc_flag;
	 kal_uint8 i;	 
	 kal_uint8 lsc_buff1[64];	 
	 kal_uint8 lsc_buff2[64];	 
	 kal_uint8 lsc_buff3[64];	 
	 kal_uint8 lsc_buff4[64];	 
	 kal_uint8 lsc_buffr[64];	 
	 kal_uint8 lsc_buffb[64];	 

	 kal_uint16 sum1 =0;
	 kal_uint16 sum2 =0;
	 
	 t4ka3_otp_set_page(0x00);
	 t4ka3_otp_start_read();
	 lsc_flag = read_cmos_sensor(0x4A04);
	 printk("lsc_flag=0x%x\n",lsc_flag);	
	 if(lsc_flag==0x01)
	 	{
	 	write_cmos_sensor(0x3243,0x03);// 1st lsc turn on
		
		printk("load 1st lsc\n");
	 	return 1;
	 	}
	 else if(lsc_flag==0x03)
	 	{
	 	write_cmos_sensor(0x3551,0x85);// 2nd lsc turn on
		printk("load 2nd lsc\n");
	 	return 2;
    }
   else if(lsc_flag==0x07)
   {
		t4ka3_otp_read_enable(1);
		
		t4ka3_otp_set_page(3);
		t4ka3_otp_start_read();
		for(i=0;i<64;i++)  
		{
			lsc_buff1[i] = read_cmos_sensor(T4KA3_OTP_DATA_BEGIN_ADDR+i);
		}
		
		t4ka3_otp_set_page(4);
		t4ka3_otp_start_read();
		for(i=0;i<64;i++)  
		{
			lsc_buff2[i] = read_cmos_sensor(T4KA3_OTP_DATA_BEGIN_ADDR+i);
		}
		
		t4ka3_otp_set_page(5);
		t4ka3_otp_start_read();
		for(i=0;i<64;i++)  
		{
			lsc_buff3[i] = read_cmos_sensor(T4KA3_OTP_DATA_BEGIN_ADDR+i);
		}

		t4ka3_otp_set_page(6);
		t4ka3_otp_start_read();
		for(i=0;i<64;i++)  
		{
			lsc_buff4[i] = read_cmos_sensor(T4KA3_OTP_DATA_BEGIN_ADDR+i);
		}

		for(i=0;i<64;i++)  
		{

			lsc_buffb[i] = lsc_buff1[i] | lsc_buff2[i];
			lsc_buffr[i] = lsc_buff3[i] | lsc_buff4[i];
		}

	//	for(i=0;i<64;i++)  
	//	{

	//		printk("lsc_buffb %d=0x%x\n",i,lsc_buffb[i]);
	//		printk("lsc_buffr %d=0x%x\n",i,lsc_buffr[i]);
	//	}
				

		for(i=0;i<63;i++)
		{
			sum1+=lsc_buffb[i];
			sum2+=lsc_buffr[i];
		}
		printk("sum1=0x%x\n",sum1);
		printk("sum2=0x%x\n",sum2);

		printk("lsc_buffb[63]=0x%x\n",lsc_buffb[63]);
		printk("lsc_buffr[63]=0x%x\n",lsc_buffr[63]);	
		
	//	if(((sum1&0xff)==lsc_buffb[63])&&((sum2&0xff)==lsc_buffr[63])) //for checking checksum
			for(i=0;i<12;i++)
			{
				write_cmos_sensor(0x3618+i, lsc_buffb[i]);
				write_cmos_sensor(0x363c+i, lsc_buffb[i+12]);
				write_cmos_sensor(0x3660+i, lsc_buffb[i+24]);
				write_cmos_sensor(0x3684+i, lsc_buffb[i+36]);
				write_cmos_sensor(0x360c+i, lsc_buffr[i]);
				write_cmos_sensor(0x3630+i, lsc_buffr[i+12]);
				write_cmos_sensor(0x3654+i, lsc_buffr[i+24]);
				write_cmos_sensor(0x3678+i, lsc_buffr[i+36]);
			}
			write_cmos_sensor(0x3692, lsc_buffb[48]);
			write_cmos_sensor(0x3695, lsc_buffb[49]);
			write_cmos_sensor(0x3698, lsc_buffb[50]);
			write_cmos_sensor(0x369b, lsc_buffb[51]);
			write_cmos_sensor(0x369e, lsc_buffb[52]);
			write_cmos_sensor(0x36a1, lsc_buffb[53]);
			write_cmos_sensor(0x36a4, lsc_buffb[54]);
			write_cmos_sensor(0x36a7, lsc_buffb[55]);
			write_cmos_sensor(0x36aa, lsc_buffb[56]);

			write_cmos_sensor(0x3694, lsc_buffr[48]);
			write_cmos_sensor(0x3697, lsc_buffr[49]);
			write_cmos_sensor(0x369a, lsc_buffr[50]);
			write_cmos_sensor(0x369d, lsc_buffr[51]);
			write_cmos_sensor(0x36a0, lsc_buffr[52]);
			write_cmos_sensor(0x36a3, lsc_buffr[53]);
			write_cmos_sensor(0x36a6, lsc_buffr[54]);
			write_cmos_sensor(0x36a9, lsc_buffr[55]);
			write_cmos_sensor(0x36ac, lsc_buffr[56]);

			write_cmos_sensor(0x3693, lsc_buffb[57]);
			write_cmos_sensor(0x3696, lsc_buffb[58]);
			write_cmos_sensor(0x3699, lsc_buffb[59]);
			write_cmos_sensor(0x369c, lsc_buffb[60]);			

			write_cmos_sensor(0x369f, lsc_buffr[57]);
			write_cmos_sensor(0x36a2, lsc_buffr[58]);
			write_cmos_sensor(0x36a5, lsc_buffr[59]);
			write_cmos_sensor(0x36a8, lsc_buffr[60]);
			
			write_cmos_sensor(0x36b0, lsc_buffb[61]);
			write_cmos_sensor(0x36b1, lsc_buffb[62]);

			write_cmos_sensor(0x36ab, lsc_buffr[61]);
			write_cmos_sensor(0x36ae, lsc_buffr[62]);	
        printk("load 3rd lsc\n");
		return 3;
     	}
    else
    	{
    		printk("Load LSC NG\n");
    	}
	 return  0;
	 
} 

static void t4ka3_otp_update_awb(void)
{  
  int flag;
  kal_uint16 check_sum=0x0000;
  kal_uint8  checksumpage=0x00;
//  kal_uint16 OTP_data[64];

  t4ka3_otp_set_page(0x01);
  t4ka3_otp_start_read();
  flag = read_cmos_sensor(0x4A06);
  printk("yzb----flag:%d\n",flag);
  flag = read_cmos_sensor(0x4A07);
   printk("yzb----flag:%d\n",flag);
   flag = read_cmos_sensor(0x4A0A);
   printk("yzb----flag:%d\n",flag);
  //--------------read OTP Add by joe 2015/9/21--------------//
  t4ka3_otp_set_page(0x00);
  t4ka3_otp_start_read();
  flag = read_cmos_sensor(0x4A08);
	mDELAY(2);  
  t4ka3_otp_set_page(0x02);
  t4ka3_otp_start_read();
 // t4ka3_otp_read_data(OTP_data); //For printf all AWB inf
  		if(flag == 0x00)
  			{
				printk("AWB is empty\n");
				return ; //return for breaking camear off
			}
			
		else if(flag == 0x01)
			{
				printk("first OTP\n");
				t4ka3_r_cur_value=((read_cmos_sensor(0x4A04) << 8) | read_cmos_sensor(0x4A05));
  				t4ka3_g_cur_value=((read_cmos_sensor(0x4A06) << 8) | read_cmos_sensor(0x4A07));
  				t4ka3_b_cur_value=((read_cmos_sensor(0x4A08) << 8) | read_cmos_sensor(0x4A09));
				checksumpage = read_cmos_sensor(0x4A0A);
  				printk("checksumpage=0x%x\n",checksumpage);
				  //checking awb check sum
   			//	for( i=0; i<6;  i++ )
  			//	 {
    				  check_sum =t4ka3_r_cur_value+ t4ka3_g_cur_value+t4ka3_b_cur_value;
	    				printk("check_sum=0x%x\n",check_sum);
   			//	 }
  			//	if(check_sum!=checksumpage)
  			//		{
			//			printk("check_sum error\n");
			//			return -1;
			//		}
				
			}
		else if(flag == 0x03)
			{
				printk("Second OTP\n");
				t4ka3_r_cur_value=((read_cmos_sensor(0x4A0C) << 8) | read_cmos_sensor(0x4A0D));
  				t4ka3_g_cur_value=((read_cmos_sensor(0x4A0E) << 8) | read_cmos_sensor(0x4A0F));
  				t4ka3_b_cur_value=((read_cmos_sensor(0x4A10) << 8) | read_cmos_sensor(0x4A11));
				checksumpage = read_cmos_sensor(0x4A12);
  				printk("checksumpage=0x%x\n",checksumpage);
				  //checking awb check sum
   				//for( i=0; i<6;  i++ )
  				// {
    				   check_sum =t4ka3_r_cur_value+ t4ka3_g_cur_value+t4ka3_b_cur_value;
	    				printk("check_sum=0x%x\n",check_sum);
   				// }
  			//	if(check_sum!=checksumpage)
  			//		{
			//			printk("check_sum error\n");
			//			return -1;
			//		}
				
				
			}
		else if(flag == 0x07)
			{
				printk("Third OTP\n");
				t4ka3_r_cur_value=((read_cmos_sensor(0x4A14) << 8) | read_cmos_sensor(0x4A15));
  				t4ka3_g_cur_value=((read_cmos_sensor(0x4A16) << 8) | read_cmos_sensor(0x4A17));
  				t4ka3_b_cur_value=((read_cmos_sensor(0x4A18) << 8) | read_cmos_sensor(0x4A19));
				checksumpage = read_cmos_sensor(0x4A1a);
  				printk("checksumpage=0x%x\n",checksumpage);
				  //checking awb check sum
   			////	for( i=0; i<6;  i++ )
  			//	 {
    				   check_sum =t4ka3_r_cur_value+ t4ka3_g_cur_value+t4ka3_b_cur_value;
	    				printk("check_sum=0x%x\n",check_sum);
   			//	 }
  			//	if(check_sum!=checksumpage)
  			//		{
			//			printk("check_sum error\n");
			//			return -1;
			//		}
				
			}
		else
			{
			    printk("OTP error\n");
				return ;
			}
			
		
  	
  //--------------END-----------------------------------//
  //t4ka3_r_cur_value=((read_cmos_sensor(0x4A04) << 8) | read_cmos_sensor(0x4A05));
  //t4ka3_g_cur_value=((read_cmos_sensor(0x4A06) << 8) | read_cmos_sensor(0x4A07));
  //t4ka3_b_cur_value=((read_cmos_sensor(0x4A08) << 8) | read_cmos_sensor(0x4A09));
  printk("t4ka3_r_cur_value=0x%x,t4ka3_g_cur_value=0x%x,t4ka3_b_cur_value=0x%x\n",t4ka3_r_cur_value,t4ka3_g_cur_value,t4ka3_b_cur_value);
  printk("t4ka3_r_golden_value=0x%x,t4ka3_g_golden_value=0x%x,t4ka3_b_golden_value=0x%x\n",t4ka3_r_golden_value,t4ka3_g_golden_value,t4ka3_b_golden_value);
  //checking awb check sum
  // for( i=0; i<6;  i++ )
  // {
  //    check_sum = check_sum + read_cmos_sensor(0x4A04+i);
//	    printk("check_sum=0x%x\n",check_sum);
  // }
  
     //for(i=0,sum=0x4a04;i<6;i++)
      //{
	   //sum+=i;  
	   //check_sum=check_sum + read_cmos_sensor(sum);  
      //}
   
  // checksumpage = read_cmos_sensor(0x4A0A);
  // printk("checksumpage=0x%x\n",checksumpage);
  //if((check_sum&0xFF)==checksumpage)
 // {
//	 printk("otp awb checksum ok!\n");
  //1. Calculate R gain, and B gain for AWB correction
  t4ka3_r_gain = 256*((t4ka3_r_golden_value*t4ka3_g_cur_value)*1000/(t4ka3_r_cur_value*t4ka3_g_golden_value))/1000;
  t4ka3_b_gain = 256*((t4ka3_b_golden_value*t4ka3_g_cur_value)*1000/(t4ka3_b_cur_value*t4ka3_g_golden_value))/1000;
    printk("((float)t4ka3_r_golden_value*t4ka3_g_cur_value)=%d,(t4ka3_r_cur_value *t4ka3_g_golden_value)=%d\n",(t4ka3_r_golden_value*t4ka3_g_cur_value),(t4ka3_r_cur_value *t4ka3_g_golden_value));
    printk("(t4ka3_b_golden_value*t4ka3_g_cur_value)=%d,(t4ka3_b_cur_value *t4ka3_g_golden_value)=%d\n",(t4ka3_b_golden_value*t4ka3_g_cur_value),(t4ka3_b_cur_value *t4ka3_g_golden_value));
	printk("t4ka3_r_gain=0x%x,t4ka3_b_gain=0x%x\n",t4ka3_r_gain,t4ka3_b_gain);
	
  //2.write R gain and B gain to KA3 register;
	write_cmos_sensor(0x0210, t4ka3_r_gain	>> 8);
	write_cmos_sensor(0x0211, t4ka3_r_gain	& 0xff);
	write_cmos_sensor(0x0212, t4ka3_b_gain >> 8);
	write_cmos_sensor(0x0213, t4ka3_b_gain & 0xff);
//	return 0;
 //  }
  //else
  // {
//	printk("otp awb checksum error!\n");
	//return -1;
  // }
   
		
}
static kal_uint16 venderID;
static kal_uint16 lensID;
static kal_uint16 Get_venderID(void)
{
	
	t4ka3_otp_read_enable(1);
	t4ka3_otp_set_page(0x01);
	t4ka3_otp_start_read();
	venderID = read_cmos_sensor(0x4A06);
	printk("t4ka3 venderID: 0x%x\n",venderID);
	lensID = read_cmos_sensor(0x4A0A);
	printk("t4ka3 lensID: 0x%x\n",lensID);
	t4ka3_otp_read_enable(0);
	return venderID;
}

static void t4ka3_otp_init_setting(void)
{  
   t4ka3_otp_read_enable(1);
   t4ka3_otp_checkMag();
   t4ka3_otp_update_lsc();
   t4ka3_otp_update_awb();
   t4ka3_otp_read_enable(0);
}
	
/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
extern int af_ofilm_af_test(void);
extern int af_sjc_af_test(void);
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			
			*sensor_id = ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
			if (*sensor_id == imgsensor_info.sensor_id)
				{
				if(Get_venderID()==0x10)
				{
					if(lensID == 8)
					{
						*sensor_id = T4KA3MIPI_SENSOR_ID;
						printk("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	
						return ERROR_NONE;
					}
					else
				   		 break;
				}	
			}	
	//		printk("Read sensor id fail, id: 0x%x    0x%x \n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	*sensor_id = 0xFFFFFFFF;
	return ERROR_SENSOR_CONNECT_FAIL;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0; 
	printk("PLATFORM:MT6735M,MIPI 4LANE\n");
	//printk("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");
	
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    mDELAY(2);
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
		
			sensor_id = ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
			if (sensor_id == imgsensor_info.sensor_id) {	
				printk("ofilm in open i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				
				break;
			}	
	//		printk("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}		 
	
	/* initail sequence write in  */
	sensor_init();
	t4ka3_otp_init_setting();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	printk("[T4KA3] close E\n");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("[T4KA3] privew E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	//mDELAY(40);
	printk("preview() done!\n");
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("[T4KA3] capture  E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            printk("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	printk("imgsensor.current_fps=%d, imgsensor.pclk =%d, imgsensor.line_length =%d, imgsensor.frame_length =%d, imgsensor.min_frame_length =%d\n",
				imgsensor.current_fps, imgsensor.pclk, imgsensor.line_length, imgsensor.frame_length, imgsensor.min_frame_length);
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps); 
	//capture_setting(240); 
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("T4KA3 normal_video  E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	//mDELAY(40);
	
	
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("t4ka3 hs_video  E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("T4KA3 slim E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
	
	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	printk("t4ka3  get_resolution E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth   	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

			break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			flag = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			flag = 1;
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
		default:
			printk("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	printk("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{   
     #if 0
     kal_uint32 reg[12];
	if (KAL_TRUE == imgsensor.test_pattern) return ERROR_NONE;	//add myself
	printk("enable = %x, framerate = %d \n", enable, framerate);
	 
	  reg[0] = read_cmos_sensor(0x0340);
	  reg[1] = read_cmos_sensor(0x0341);
	  reg[2] = read_cmos_sensor(0x0342);
	  reg[3] = read_cmos_sensor(0x0343);
	  printk("ciel_test_0x0340 = %x, ciel_test_0x0341 = %x ,ciel_test_0x0342 = %x, ciel_test_0x0343 = %x\n",  reg[0],reg[1],reg[2],reg[3]); 
	  reg[4] = read_cmos_sensor(0x034c);
	  reg[5] = read_cmos_sensor(0x034d);
	  reg[6] = read_cmos_sensor(0x034e);
	  reg[7] = read_cmos_sensor(0x034f);
	  printk("ciel_test_0x034c = %x, ciel_test_0x034d = %x ,ciel_test_0x034e = %x, ciel_test_0x034f = %x\n",  reg[4],reg[5],reg[6],reg[7]); 
	  reg[8] = read_cmos_sensor(0x0202);
	  reg[9] = read_cmos_sensor(0x0203);
	  reg[10] = read_cmos_sensor(0x0234);
	  reg[11] = read_cmos_sensor(0x0235);
	  printk("ciel_test_0x0202 = %x, ciel_test_0x0203 = %x ,ciel_test_0x0234 = %x, ciel_test_0x0235 = %x\n",  reg[8],reg[9],reg[10],reg[11]);       
	  #endif        
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_uint32 frame_length;
  
	printk("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:		
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();	
			break;		
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();	
			printk("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	printk("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}



static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
//    unsigned long long *feature_return_para=(unsigned long long *) feature_para;
	
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	printk("feature_id = %d\n", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:	 
            printk("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;		   
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
			break;
		case SENSOR_FEATURE_SET_GAIN:		
            set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
			break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32); 
			break; 
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            printk("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
		case SENSOR_FEATURE_SET_HDR:
            printk("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            printk("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
			wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		
			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;	  
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
			break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            printk("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2)); 
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));    
			break;
		default:
			break;
	}
  
	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};


UINT32 T4KA3_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
} 
		
