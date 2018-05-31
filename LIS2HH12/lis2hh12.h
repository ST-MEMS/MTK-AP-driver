
/********************************* (C) COPYRIGHT 2017 STMicroelectronics ********************************
 *
 * File Name:   lis2hh12.h
 * Authors:     William Zeng
 * Version:     V3.0.3
 * Date:        07/05/2017
 * Description: LIS2HH12 driver source file
 *
 *********************************************************************************************************
 * Copyright (c) 2017, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the STMicroelectronics nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************************************
 * REVISON HISTORY
 *
 * VERSION | DATE          | DESCRIPTION
 *
 * 3.0.2   | 06/22/2017    | MTK driver initial version
 * 3.0.3   | 07/05/2017    | fixed ATA test bug
 *
 ****************************************************************************************************/


#ifndef _LIS2HH12_H_
#define _LIS2HH12_H_
	 
#include <linux/ioctl.h>
#include <hwmsensor.h>
#include <accel.h>
#include <cust_acc.h>

#define DEBUG 						1
#define CONFIG_LIS2HH12_LOWPASS   /*apply low pass filter on output*/       

/* LIS2HH12 Register Map  (Please refer to LIS2HH12 Specifications) */
#define LIS2HH12_REG_WHO_AM_I 				0x0F
#define LIS2HH12_REG_ACT_THS 				0x1E
#define LIS2HH12_REG_ACT_DUR 				0x1F
#define LIS2HH12_REG_CTRL1 				0x20
#define LIS2HH12_REG_CTRL2 				0x21
#define LIS2HH12_REG_CTRL3 				0x22
#define LIS2HH12_REG_CTRL4 				0x23
#define LIS2HH12_REG_CTRL5 				0x24
#define LIS2HH12_REG_CTRL6 				0x25
#define LIS2HH12_REG_CTRL7 				0x26
#define LIS2HH12_REG_STATUS 				0x27
#define LIS2HH12_REG_OUT_X_L 				0x28
#define LIS2HH12_REG_OUT_X_H			 	0x29
#define LIS2HH12_REG_OUT_Y_L 				0x2A
#define LIS2HH12_REG_OUT_Y_H 				0x2B
#define LIS2HH12_REG_OUT_Z_L 				0x2C
#define LIS2HH12_REG_OUT_Z_H 				0x2D
#define LIS2HH12_REG_FIFO_CTRL 				0x2E
#define LIS2HH12_REG_FIFO_SRC 				0x2F
#define LIS2HH12_REG_IG_CFG1 				0x30
#define LIS2HH12_REG_IG_SRC1 				0x31
#define LIS2HH12_REG_IG_THS_X1 				0x32
#define LIS2HH12_REG_IG_THS_Y1 				0x33
#define LIS2HH12_REG_IG_THS_Z1 				0x34
#define LIS2HH12_REG_IG_DUR1 				0x35
#define LIS2HH12_REG_IG_CFG2 				0x36
#define LIS2HH12_REG_IG_SRC2 				0x37
#define LIS2HH12_REG_IG_THS2 				0x38
#define LIS2HH12_REG_IG_DUR2 				0x39
#define LIS2HH12_REG_XL_REFERENCE 			0x3A
#define LIS2HH12_REG_XH_REFERENCE 			0x3B
#define LIS2HH12_REG_YL_REFERENCE 			0x3C
#define LIS2HH12_REG_YH_REFERENCE	 		0x3D
#define LIS2HH12_REG_ZL_REFERENCE 			0x3E
#define LIS2HH12_REG_ZH_REFERENCE 			0x3F

/*  interrupt det mask*/
#define LIS2HH12_INT2_ON_INT1_MASK		 	0x20

#define LIS2HH12_EN			             	0x01
#define LIS2HH12_DIS			         	0x00

/*
*  register mask bit
*/
#define LIS2HH12_REG_CTRL2_MASK_FUNC_CFG_EN        	0x10
#define LIS2HH12_REG_CTRL1_MASK_ODR		        0x70
#define LIS2HH12_REG_CTRL4_MASK_FS			0x30
#define LIS2HH12_REG_CTRL10_C_MASK_FUNC_EN              0x04

/*
*  register value
*/
#define LIS2HH12_FIXED_DEVID_LIS2HH12			0x41

#define LIS2HH12_REG_FUNC_CFG_ACCESS_ENABLE             0x80
#define LIS2HH12_REG_FUNC_CFG_ACCESS_DISABLE            0x00

#define LIS2HH12_REG_CTRL1_ODR_0HZ			0x00
#define LIS2HH12_REG_CTRL1_ODR_10HZ			0x01
#define LIS2HH12_REG_CTRL1_ODR_50HZ			0x02
#define LIS2HH12_REG_CTRL1_ODR_100HZ			0x03
#define LIS2HH12_REG_CTRL1_ODR_200HZ			0x04
#define LIS2HH12_REG_CTRL1_ODR_400HZ			0x05
#define LIS2HH12_REG_CTRL1_ODR_800HZ			0x06

#define LIS2HH12_REG_CTRL1_FS_2G			0x00
#define LIS2HH12_REG_CTRL1_FS_4G			0x02
#define LIS2HH12_REG_CTRL1_FS_8G			0x03

#define LIS2HH12_REG_CTRL10_C_FUNC_ENABLE               0x04
#define LIS2HH12_REG_CTRL10_C_FUNC_DISABLE              0x00
#define LIS2HH12_REG_CTRL10_C_FUNC_PEDO_RST_STEP        0x02

/*
*  return value
*/
#define LIS2HH12_SUCCESS				0
#define LIS2HH12_ERR_I2C				-1
#define LIS2HH12_ERR_STATUS				-3
#define LIS2HH12_ERR_SETUP_FAILURE			-4
#define LIS2HH12_ERR_GETGSENSORDATA			-5
#define LIS2HH12_ERR_IDENTIFICATION			-6

#define LIS2HH12_BUFSIZE				256

#define LIS2HH12_AXIS_X          			0
#define LIS2HH12_AXIS_Y          			1
#define LIS2HH12_AXIS_Z          			2
#define LIS2HH12_AXES_NUM        			3
#define LIS2HH12_DATA_LEN        			6
#define LIS2HH12_DEV_NAME        			"LIS2HH12"

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
    u8 whole;
    u8 fraction;
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
    s16 raw[C_MAX_FIR_LENGTH][LIS2HH12_AXES_NUM];
    int sum[LIS2HH12_AXES_NUM];
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
struct lis2hh12_acc {
    u8 name[32];
    struct acc_hw *lis2hh12_acc_hw;
    struct hwmsen_convert cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t trace;
    atomic_t suspend;
    atomic_t selftest;
    atomic_t filter;
    s32 cali_sw[LIS2HH12_AXES_NUM];

    /*data*/
    s32 offset[LIS2HH12_AXES_NUM];
    s16 data[LIS2HH12_AXES_NUM];
    bool lis2hh12_acc_power;
    int odr;

#if defined(CONFIG_LIS2HH12_LOWPASS)
    atomic_t firlen;
    atomic_t fir_en;
    struct data_filter fir;
#endif 
};

struct lis2hh12_data {
    u8 name[32];
    u8 chip_id;
    struct i2c_client *client;
    struct lis2hh12_acc lis2hh12_acc_data;
    u8 reg_addr;
    int acc_enabled;
};

#define ST_TAG                  "STMEMS "
#define ST_ERR(fmt, args...)    printk(KERN_ERR ST_TAG "%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define ST_LOG(fmt, args...)    printk(KERN_ERR ST_TAG fmt, ##args)

#if defined(DEBUG)
#define ST_FUN(f)               printk(KERN_INFO ST_TAG "%s\n", __FUNCTION__)
#define ST_DBG(fmt, args...)    printk(KERN_ERR ST_TAG fmt, ##args)
#else
#define ST_FUN(f)
#define ST_DBG(fmt, args...)
#endif

extern struct lis2hh12_data *obj_i2c_data;

int lis2hh12_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lis2hh12_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lis2hh12_i2c_write_with_mask(struct i2c_client *client,u8 addr, u8 mask, u8 data);

void dumpReg(struct lis2hh12_data *obj);
int lis2hh12_set_interrupt(void);

int lis2hh12_acc_set_power_mode(struct lis2hh12_acc *acc_obj, bool enable);
int lis2hh12_acc_init(struct lis2hh12_acc *acc_obj, int reset_cali);

extern struct acc_init_info lis2hh12_acc_init_info;

#endif
