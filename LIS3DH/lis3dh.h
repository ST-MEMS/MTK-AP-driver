
/********************************* (C) COPYRIGHT 2018 STMicroelectronics ********************************
 *
 * File Name         : lis3dh.h
 * Authors           : William ZENG
 * Version           : V1.0.3
 * Date              : 07/24/2018
 * Description       : LIS3DH driver source file
 *
 *********************************************************************************************************
  * Copyright (c) 2018, STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *********************************************************************************************************
 * REVISON HISTORY
 *
 * VERSION | DATE          | DESCRIPTION
 *
 * 1.0.2   | 05/09/2018	   | modified driver to be compatible with Android O
 * 1.0.3   | 07/24/2018	   | optimized calibration functions
 * 
 ****************************************************************************************************/

#ifndef LIS3DH_H
#define LIS3DH_H
	 
#include <linux/ioctl.h>
#include <accel.h>
#include <hwmsensor.h>
#include <cust_acc.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE
	 
/* LIS3DH Register Map  (Please refer to LIS3DH Specifications) */
#define LIS3DH_REG_WHO_AM_I		0x0F
#define LIS3DH_REG_CTL_REG1		0x20
#define LIS3DH_REG_CTL_REG2		0x21
#define LIS3DH_REG_CTL_REG3		0x22
#define LIS3DH_REG_CTL_REG4		0x23
#define LIS3DH_REG_DATAX0		    0x28
#define LIS3DH_REG_OUT_X		    0x28
#define LIS3DH_REG_OUT_Y		    0x2A
#define LIS3DH_REG_OUT_Z		    0x2C


#define LIS3DH_FIXED_DEVID		0x33
	 
#define LIS3DH_BW_200HZ			0x60
#define LIS3DH_BW_100HZ			0x50 //400 or 100 on other choise //changed
#define LIS3DH_BW_50HZ			0x40
#define LIS3DH_BW_0HZ             0x00

#define	LIS3DH_FULLRANG_LSB		0XFF
	 
#define LIS3DH_MEASURE_MODE		0x08	//changed 
#define LIS3DH_DATA_READY			0x07    //changed
	 
//#define LIS3DH_FULL_RES			0x08
#define LIS3DH_RANGE_2G			0x00
#define LIS3DH_RANGE_4G			0x10
#define LIS3DH_RANGE_8G			0x20 //8g or 2g no ohter choise//changed
#define LIS3DH_RANGE_16G			0x30

#define LIS3DH_SELF_TEST			0x10 //changed
	 
#define LIS3DH_STREAM_MODE		0x80
#define LIS3DH_SAMPLES_15			0x0F
	 
#define LIS3DH_FS_8G_LSB_G		0x20
#define LIS3DH_FS_4G_LSB_G		0x10
#define LIS3DH_FS_2G_LSB_G		0x00
	 
#define LIS3DH_LEFT_JUSTIFY		0x04
#define LIS3DH_RIGHT_JUSTIFY		0x00
	 
#define LIS3DH_SUCCESS					0
#define LIS3DH_ERR_I2C					-1
#define LIS3DH_ERR_STATUS					-3
#define LIS3DH_ERR_SETUP_FAILURE			-4
#define LIS3DH_ERR_GETGSENSORDATA			-5
#define LIS3DH_ERR_IDENTIFICATION			-6

#define LIS3DH_BUFSIZE			256

#define LIS3DH_AXIS_X          0
#define LIS3DH_AXIS_Y          1
#define LIS3DH_AXIS_Z          2
#define LIS3DH_AXES_NUM        3
#define LIS3DH_DATA_LEN        6
#define LIS3DH_DEV_NAME        "LIS3DH"

//#define CONFIG_LIS3DH_LOWPASS   /*apply low pass filter on output*/       

#define CONFIG_LIS3DH_ACC_DRY

/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI    = 0X08,
    ADX_TRC_INFO    = 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][LIS3DH_AXES_NUM];
    int sum[LIS3DH_AXES_NUM];
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
struct lis3dh_acc {
    //struct i2c_client *client;
    struct acc_hw *lis3dh_acc_hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t                filter;
    s32                     cali_sw[LIS3DH_AXES_NUM];

    /*data*/
    s32                     offset[LIS3DH_AXES_NUM];
    s16                     data[LIS3DH_AXES_NUM];
    bool                    lis3dh_acc_power;
	int                     odr;
	int                     enabled;
#if defined(CONFIG_LIS3DH_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
};

struct lis3dh_data {
    struct i2c_client *client;
    struct lis3dh_acc lis3dh_acc_data;
    u8     reg_addr;
};

#define ST_TAG                  "[ST] "
#define ST_ERR(fmt, args...)    printk(KERN_ERR ST_TAG "%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define ST_LOG(fmt, args...)    printk(KERN_ERR ST_TAG fmt, ##args)

#if defined(DEBUG)
    #define ST_FUN(f)               printk(KERN_INFO ST_TAG "%s\n", __FUNCTION__)
	#define ST_DBG(fmt, args...)    printk(KERN_ERR ST_TAG fmt, ##args)
#else
	#define ST_FUN(f)
	#define ST_DBG(fmt, args...)
#endif

int lis3dh_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lis3dh_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);

void dumpReg(struct lis3dh_data *obj);
int lis3dh_set_interrupt(struct lis3dh_data *obj, u8 intenable);

int lis3dh_acc_set_power_mode(struct lis3dh_acc *acc_obj, bool enable);
int lis3dh_acc_init(struct lis3dh_acc *acc_obj, int reset_cali);

//extern struct i2c_client *lis3dh_i2c_client;
extern struct lis3dh_data *obj_i2c_data;
extern int sensor_suspend;
extern struct acc_init_info lis3dh_acc_init_info;
extern int lis3dh_acc_init_flag;

#endif
