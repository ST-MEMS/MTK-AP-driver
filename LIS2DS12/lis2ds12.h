
/********************************* (C) COPYRIGHT 2017 STMicroelectronics ********************************
 *
 * File Name: 	lis2ds12.h
 * Authors: 	Ian Yang, William Zeng
 * Version: 	V3.0.3
 * Date: 	07/05/2017
 * Description: LIS2DS12 driver source file
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
 * 3.0.1   | 06/15/2017    | MTK driver initial version
 * 3.0.2   | 07/03/2017	   | add Kconfig
 * 3.0.3   | 07/05/2017	   | fixed ATA test bug
 *
 ****************************************************************************************************/


#ifndef _LIS2DS12_H_
#define _LIS2DS12_H_
	 
#include <linux/ioctl.h>
#include <hwmsensor.h>
#include <accel.h>
#include <step_counter.h>
#include <tilt_detector.h>
#include <cust_acc.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define DEBUG 1

#define CONFIG_STEP_COUNTER              1
#define CONFIG_STEP_DETECT               1
#define CONFIG_SIGNIFICANT_MOTION        1
#define CONFIG_TILT                      1

#define CONFIG_PEDOMETER_ALWAYS_ON       0   //this mean the peodometer still work even system in susupend .

#if (CONFIG_STEP_DETECT || CONFIG_SIGNIFICANT_MOTION || CONFIG_TILT)
#define CONFIG_HARDWARE_INTERRUPT        1   //if use above features, suggest use hardware interrupt 
#else
#define CONFIG_HARDWARE_INTERRUPT        0
#endif

#define ST_SENSOR_STEP_COUNTER           0
#define ST_SENSOR_STEP_DETECT            1
#define ST_SENSOR_SIGNIFICANT_MOTION     2
#define ST_SENSOR_TILT                   3
	 
/* LIS2DS12 Register Map  (Please refer to LIS2DS12 Specifications) */
#define LIS2DS12_REG_WHO_AM_I 				0x0F
#define LIS2DS12_REG_CTRL1 				0x20
#define LIS2DS12_REG_CTRL2 				0x21
#define LIS2DS12_REG_CTRL3 				0x22
#define LIS2DS12_REG_CTRL4 				0x23
#define LIS2DS12_REG_CTRL5 				0x24
#define LIS2DS12_REG_FIFO_CTRL 				0x25
#define LIS2DS12_REG_OUT_T 				0x26
#define LIS2DS12_REG_STATUS 				0x27
#define LIS2DS12_REG_OUT_X_L 				0x28
#define LIS2DS12_REG_OUT_X_H			 	0x29
#define LIS2DS12_REG_OUT_Y_L 				0x2A
#define LIS2DS12_REG_OUT_Y_H 				0x2B
#define LIS2DS12_REG_OUT_Z_L 				0x2C
#define LIS2DS12_REG_OUT_Z_H 				0x2D
#define LIS2DS12_REG_FIFO_THS 				0x2E
#define LIS2DS12_REG_FIFO_SRC 				0x2F
#define LIS2DS12_REG_FIFO_SAMPLES 			0x30
#define LIS2DS12_REG_TAP_6D_THS 			0x31
#define LIS2DS12_REG_INT_DUR 				0x32
#define LIS2DS12_REG_WAKE_UP_THS 			0x33
#define LIS2DS12_REG_WAKE_UP_DUR 			0x34
#define LIS2DS12_REG_FREE_FALL 				0x35
#define LIS2DS12_REG_STATUS_DUP 			0x36
#define LIS2DS12_REG_WAKE_UP_SRC 			0x37
#define LIS2DS12_REG_TAP_SRC 				0x38
#define LIS2DS12_REG_6D_SRC 				0x39
#define LIS2DS12_REG_STEP_COUNTER_MINTHS 		0x3A
#define LIS2DS12_REG_STEP_COUNTER_L 			0x3B
#define LIS2DS12_REG_STEP_COUNTER_H 			0x3C
#define LIS2DS12_REG_FUNC_CK_GATE	 		0x3D
#define LIS2DS12_REG_FUNC_SRC 				0x3E
#define LIS2DS12_REG_FUNC_CTRL 				0x3F

/* step, tilt, significant, func */
#define LIS2DS12_SIGN_MOTION_ENABLE_MASK 		0x02
#define LIS2DS12_STEP_ENABLE_MASK        		0x01
#define LIS2DS12_TILT_ENABLE_MASK        		0x10
#define LIS2DS12_INT_ACTIVE_MASK         		0x02

/*  interrupt det mask*/
#define LIS2DS12_STEP_INT_MASK           		0x04
#define LIS2DS12_SIGN_MOTION_INT_MASK    		0x08
#define LIS2DS12_TILT_INT_MASK           		0x10
#define LIS2DS12_INT2_ON_INT1_MASK		 	0x20

/*  interrupt det flag mask*/
#define LIS2DS12_FLAG_STEP_MASK           		0x02
#define LIS2DS12_FLAG_TILT_MASK           		0x80
#define LIS2DS12_FlAG_SIGN_MOTION_MASK    		0x10

/*
*  embeded register
*/
#define LIS2DS12_REG_PEDO_DEB            		0x2B
#define LIS2DS12_REG_EMBED_CTRL2	 		0x3F

#define LIS2DS12_EN			             	0x01
#define LIS2DS12_DIS			         	0x00

/*
*  register mask bit
*/
#define LIS2DS12_REG_CTRL2_MASK_FUNC_CFG_EN        	     0x10
#define LIS2DS12_REG_CTRL1_MASK_FS			     0x0C
#define LIS2DS12_REG_CTRL1_MASK_ODR		             0xF0
#define LIS2DS12_REG_CTRL10_C_MASK_FUNC_EN                   0x04
#define LIS2DS12_REG_STEP_COUNTER_MINTHS_MASK_PEDO_RST_STEP  0x80
#define LIS2DS12_REG_TAP_CFG_MASK_PEDO_EN                    0x40
/*
*  embeded register mask bit
*/
#define LIS2DS12_REG_PEDO_THS_MASK_PEDO_4G                   0x80
#define LIS2DS12_REG_PEDO_THS_MASK_PEDO_THS_MIN              0x1F
#define LIS2DS12_REG_PEDO_DEB_MASK_DEB_TIME                  0xF8
#define LIS2DS12_REG_PEDO_DEB_MASK_DEB_STEP                  0x07

/*
*  register value
*/
#define LIS2DS12_FIXED_DEVID_LIS2DS12			     0x43

#define LIS2DS12_REG_FUNC_CFG_ACCESS_ENABLE                  0x80
#define LIS2DS12_REG_FUNC_CFG_ACCESS_DISABLE                 0x00

#define LIS2DS12_REG_CTRL1_ODR_0HZ			0x00
#define LIS2DS12_REG_CTRL1_ODR_12_5HZ			0x01
#define LIS2DS12_REG_CTRL1_ODR_25HZ			0x02
#define LIS2DS12_REG_CTRL1_ODR_50HZ			0x03
#define LIS2DS12_REG_CTRL1_ODR_100HZ			0x04
#define LIS2DS12_REG_CTRL1_ODR_200HZ			0x05
#define LIS2DS12_REG_CTRL1_ODR_400HZ			0x06
#define LIS2DS12_REG_CTRL1_ODR_800HZ			0x07

#define LIS2DS12_REG_CTRL1_FS_2G			     0x00
#define LIS2DS12_REG_CTRL1_FS_4G			     0x02
#define LIS2DS12_REG_CTRL1_FS_8G			     0x03
#define LIS2DS12_REG_CTRL1_FS_16G			     0x01

#define LIS2DS12_REG_CTRL10_C_FUNC_ENABLE                    0x04
#define LIS2DS12_REG_CTRL10_C_FUNC_DISABLE                   0x00
#define LIS2DS12_REG_CTRL10_C_FUNC_PEDO_RST_STEP             0x02

#define LIS2DS12_REG_TAP_CFG_PEDO_ENABLE                     0x40
#define LIS2DS12_REG_TAP_CFG_PEDO_DISABLE                    0x00

/*
*  embeded register value
*/
#define LIS2DS12_REG_PEDO_THS_PEDO_4G_128MG            0x84
#define LIS2DS12_REG_PEDO_THS_PEDO_4G_160MG            0x85
#define LIS2DS12_REG_PEDO_THS_PEDO_4G_192MG            0x86
#define LIS2DS12_REG_PEDO_THS_PEDO_4G_224MG            0x87
#define LIS2DS12_REG_PEDO_THS_PEDO_4G_256MG            0x88
#define LIS2DS12_REG_PEDO_THS_PEDO_4G_288MG            0x89
#define LIS2DS12_REG_PEDO_THS_PEDO_4G_320MG            0x8A
#define LIS2DS12_REG_PEDO_THS_PEDO_4G_352MG            0x8B

#define LIS2DS12_REG_PEDO_DEB_TIME_800MS               0x50
#define LIS2DS12_REG_PEDO_DEB_TIME_880MS               0x58
#define LIS2DS12_REG_PEDO_DEB_TIME_960MS               0x60
#define LIS2DS12_REG_PEDO_DEB_TIME_1040MS              0x68
#define LIS2DS12_REG_PEDO_DEB_TIME_1120MS              0x70

#define LIS2DS12_REG_PEDO_DEB_STEP_5STEP               0x05
#define LIS2DS12_REG_PEDO_DEB_STEP_6STEP               0x06
#define LIS2DS12_REG_PEDO_DEB_STEP_7STEP               0x07

/*
*  return value
*/
#define LIS2DS12_SUCCESS				0
#define LIS2DS12_ERR_I2C				-1
#define LIS2DS12_ERR_STATUS				-3
#define LIS2DS12_ERR_SETUP_FAILURE			-4
#define LIS2DS12_ERR_GETGSENSORDATA			-5
#define LIS2DS12_ERR_IDENTIFICATION			-6

#define LIS2DS12_BUFSIZE				256

#define LIS2DS12_AXIS_X          0
#define LIS2DS12_AXIS_Y          1
#define LIS2DS12_AXIS_Z          2
#define LIS2DS12_AXES_NUM        3
#define LIS2DS12_DATA_LEN        6
#define LIS2DS12_DEV_NAME        "LIS2DS12"

//#define DEGREE_TO_RAD_1000		17
/* 1 rad = 180/PI degree, MAX_LSB = 131, */
/* 180*131/PI = 7506 */
#define DEGREE_TO_RAD    	  7506
#define CONFIG_LIS2DS12_LOWPASS   /*apply low pass filter on output*/       

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
    s16 raw[C_MAX_FIR_LENGTH][LIS2DS12_AXES_NUM];
    int sum[LIS2DS12_AXES_NUM];
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
struct lis2ds12_acc {
    u8 name[32];
    struct acc_hw *lis2ds12_acc_hw;
    struct hwmsen_convert cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t trace;
    atomic_t suspend;
    atomic_t selftest;
    atomic_t filter;
    s32 cali_sw[LIS2DS12_AXES_NUM];

    /*data*/
    s32 offset[LIS2DS12_AXES_NUM];
    s16 data[LIS2DS12_AXES_NUM];
    bool lis2ds12_acc_power;
    int odr;

#if defined(CONFIG_LIS2DS12_LOWPASS)
    atomic_t firlen;
    atomic_t fir_en;
    struct data_filter fir;
#endif 
};

struct lis2ds12_pedo {
    u8 name[32];
    
    atomic_t trace;
    atomic_t suspend;
    atomic_t selftest;
    atomic_t filter;

    u32 data;
    int overflow;
    bool lis2ds12_pedo_power;
    int odr;
};

struct lis2ds12_data {
    u8 name[32];
    u8 chip_id;
#if (CONFIG_HARDWARE_INTERRUPT)
    int irq;
    atomic_t irq_enabled;
    struct work_struct irq_work;
    struct workqueue_struct *irq_work_queue;
#endif
    struct i2c_client *client;
    struct lis2ds12_acc lis2ds12_acc_data;
    struct lis2ds12_pedo lis2ds12_pedo_data;
    u8 reg_addr;
    int acc_enabled;
    int step_c_enabled;
    int step_d_enabled;
    int significant_enabled;
    int tilt_enabled;
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

extern struct lis2ds12_data *obj_i2c_data;

int lis2ds12_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lis2ds12_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lis2ds12_i2c_write_with_mask(struct i2c_client *client,u8 addr, u8 mask, u8 data);

void dumpReg(struct lis2ds12_data *obj);
int lis2ds12_set_interrupt(void);

int lis2ds12_acc_set_power_mode(struct lis2ds12_acc *acc_obj, bool enable);
int lis2ds12_acc_init(struct lis2ds12_acc *acc_obj, int reset_cali);

extern struct acc_init_info lis2ds12_acc_init_info;
extern struct tilt_init_info  lis2ds12_tilt_init_info;
extern struct step_c_init_info lis2ds12_pedo_init_info;

#endif
