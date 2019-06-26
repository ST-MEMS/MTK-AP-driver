/********************************* (C) COPYRIGHT 2019 STMicroelectronics ********************************
 *
 * File Name:   lps22hh.h
 * Authors:     William ZENG
 * Version:     V1.0.0
 * Date:        04/30/2019
 * Description: LPS22HH driver source file
 *
 *********************************************************************************************************
  * Copyright (c) 2019, STMicroelectronics.
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
 * 1.0.0   | 04/30/2019    | MTK driver initial version
 *
 ****************************************************************************************************/

#ifndef LPS22HH_H
#define LPS22HH_H
	 
#include <linux/ioctl.h>
#include <barometer.h>
#include <hwmsensor.h>
#include <cust_baro.h>

//#define POWER_NONE_MACRO MT65XX_POWER_NONE

#define DEBUG
#define KERNEL_4_4
//#define MISC_DEVICE_FACTORY		0
	 
/* LPS22HH Register Map  (Please refer to LPS22HH Specifications) */
#define LPS22HH_REG_WHO_AM_I		0x0F
#define LPS22HH_REG_CTRL_REG1		0x10
#define LPS22HH_REG_CTRL_REG2		0x11
#define LPS22HH_REG_CTRL_REG3		0x12
#define LPS22HH_REG_REF_P_L		    0x15
#define LPS22HH_REG_REF_P_H			0x16
#define LPS22HH_REG_PRESS_OUT_XL	0x28
#define LPS22HH_REG_PRESS_OUT_L		0x29
#define LPS22HH_REG_PRESS_OUT_H		0x2A
#define LPS22HH_REG_TEMP_OUT_L		0x2B
#define LPS22HH_REG_TEMP_OUT_H		0x2C

#define LPS22HH_FIXED_DEVID			0xB3

#define LPS22HH_ODR_200HZ				0x70
#define LPS22HH_ODR_100HZ				0x60
#define LPS22HH_ODR_75HZ				0x50
#define LPS22HH_ODR_50HZ				0x40
#define LPS22HH_ODR_25HZ				0x30
#define LPS22HH_ODR_10HZ            	0x20
#define LPS22HH_ODR_1HZ            		0x10
#define LPS22HH_ODR_0HZ             	0x00
	 
#define LPS22HH_SUCCESS						0
#define LPS22HH_ERR_I2C						-1
#define LPS22HH_ERR_STATUS					-3
#define LPS22HH_ERR_SETUP_FAILURE			-4
#define LPS22HH_ERR_GETGSENSORDATA			-5
#define LPS22HH_ERR_IDENTIFICATION			-6

#define LPS22HH_BUFSIZE						256

#define LPS22HH_AXES_NUM        3
#define LPS22HH_DATA_LEN        6
#define LPS22HH_DEV_NAME        "LPS22HH"

//#define CONFIG_LPS22HH_LOWPASS   /*apply low pass filter on output*/       
//#define CONFIG_LPS22HH_BARO_DRY

#define LPS22HH_BARO_DIV				100
#define LPS22HH_BARO_SENSITIVITY		4096
#define LPS22HH_TEMP_SENSITIVITY		100

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
    s32 raw[C_MAX_FIR_LENGTH];
    int sum;
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
struct lps22hh_baro {
    //struct i2c_client *client;
    struct baro_hw *lps22hh_baro_hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t                filter;
    s32                     cali_sw;

    /*data*/
    s32                     offset;
    s32                     data;
    bool                    lps22hh_baro_power;
	int                     odr;
	int                     enabled;
#if defined(CONFIG_LPS22HH_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
};

struct lps22hh_data {
    struct i2c_client *client;
    struct lps22hh_baro lps22hh_baro_data;
    u8     reg_addr;
};

#define ST_TAG                  "[STMEMS] "
#define ST_ERR(fmt, args...)    printk(KERN_ERR ST_TAG "%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define ST_LOG(fmt, args...)    printk(KERN_ERR ST_TAG "%s %d : "fmt, __FUNCTION__, __LINE__, ##args)

#if defined(DEBUG)
    	#define ST_FUN(f)               printk(KERN_INFO ST_TAG "%s\n", __FUNCTION__)
	#define ST_DBG(fmt, args...)    printk(KERN_ERR ST_TAG fmt, ##args)
#else
	#define ST_FUN(f)
	#define ST_DBG(fmt, args...)
#endif

int lps22hh_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lps22hh_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);

void dumpReg(struct lps22hh_data *obj);
int lps22hh_set_interrupt(struct lps22hh_data *obj, u8 intenable);

int lps22hh_baro_set_power_mode(struct lps22hh_baro *baro_obj, bool enable);
int lps22hh_baro_init(struct lps22hh_baro *baro_obj, int reset_cali);

//extern struct i2c_client *lps22hh_i2c_client;
extern struct lps22hh_data *obj_i2c_data;
extern int sensor_suspend;
extern struct baro_init_info lps22hh_baro_init_info;
extern int lps22hh_baro_init_flag;

#endif
