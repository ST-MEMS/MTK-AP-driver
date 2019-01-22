
/********************************* (C) COPYRIGHT 2018 STMicroelectronics ********************************
 *
 * File Name         : lsm6dso.h
 * Authors           : DP Fu
 * Version           : V3.0.0
 * Date              : 09/20/2018
 * Description       : lsm6dso driver source file
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
 * 3.0.0   | 05/15/2017    | MTK driver initial version
 *
 ****************************************************************************************************/


#ifndef LSM6DSO_H
#define LSM6DSO_H
	 
#include <linux/ioctl.h>
#include <hwmsensor.h>
#include <accel.h>
#include <gyroscope.h>
#include <step_counter.h>
//#include <tilt_detector.h>
#include <cust_acc.h>
#include <cust_gyro.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>

#define DEBUG 				 1

#define CONFIG_STEP_COUNTER              1
#define CONFIG_STEP_DETECT               0
#define CONFIG_SIGNIFICANT_MOTION        0
#define CONFIG_TILT                      0

#define CONFIG_PEDOMETER_ALWAYS_ON          0  //this mean the peodometer still work even system in susupend .

#if (CONFIG_STEP_DETECT || CONFIG_SIGNIFICANT_MOTION || CONFIG_TILT)
#define CONFIG_HARDWARE_INTERRUPT           1   //if use above feature , suggset use hardware interrupt 
#else
#define CONFIG_HARDWARE_INTERRUPT           0
#endif

#define ST_SENSOR_STEP_COUNTER              0
#define ST_SENSOR_STEP_DETECT               1
#define ST_SENSOR_SIGNIFICANT_MOTION        2
#define ST_SENSOR_TILT                      3
	 
/* lsm6dso Register Map  (Please refer to lsm6dso Specifications) */
#define LSM6DSO_FUNC_CFG_ACCESS_REG     0x01
#define LSM6DSO_REG_INT2_CTRL           0x0E
#define LSM6DSO_WHO_AM_I_REG		   0x0F
#define LSM6DSO_CTRL1_XL_REG		   0x10
#define LSM6DSO_CTRL2_G_REG		   0x11
#define LSM6DSO_CTRL3_C_REG		   		0x12
#define LSM6DSO_OUTX_L_G_REG 	   0x22
#define LSM6DSO_OUTX_L_A_REG 	   0x28
#define LSM6DSO_EMB_FUNC_STATUS_MAINPAGE_REG 0x35
#define LSM6DSO_MD1_CFG_REG        0x5E

/*
* INT make
*/
#define LSM6DSO_INT_ACTIVE_MASK         0x20


/* 
* step ,tilt , significant mask 
*/
#define LSM6DSO_STEP_ENABLE_MASK        0x08	
#define LSM6DO_TILT_ENABLE_MASK        0x10	
#define LSM6DSO_SIGN_MOTION_ENABLE_MASK 0x20
/*  interrupt det mask*/
#define LSM6DSO_STEP_INT_MASK           0x08
#define LSM6DSO_TILT_INT_MASK           0x10
#define LSM6DSO_SIGN_MOTION_INT_MASK    0x20
/*  interrupt det flag mask*/
#define LSM6DSO_FLAG_STEP_MASK          0x08
#define LSM6DSO_FLAG_TILT_MASK          0x10
#define LSM6DSO_FlAG_SIGN_MOTION_MASK   0x20
#define LSM6DSO_MASK_PEDO_RST             0x80

#define LSM6DSO_FLAG_EMB_FUNC_LIR_MASK    0x80
#define LSM6DSO_INT1_EMB_FUNC_MASK        0x02


/*
*  Embedded functions register mapping
*/
#define LSM6DSO_PAGE_SEL_REG			     0x02
#define LSM6DSO_EMB_FUNC_EN_A_REG			 0x04
#define LSM6DSO_EMB_FUNC_EN_B_REG			 0x05
#define LSM6DSO_PAGE_ADDRESS_REG			 0x08
#define LSM6DSO_PAGE_VALUE_REG			 0x09
#define LSM6DSO_EMB_FUNC_INT1_REG			 0x0A
#define LSM6DSO_FSM_INT1_A_REG			 0x0B
#define LSM6DSO_FSM_INT1_B_REG			 0x0C
#define LSM6DSO_EMB_FUNC_STATUS_REG      0x12
#define LSM6DSO_FSM_STATUS_A_REG      0x13
#define LSM6DSO_FSM_STATUS_B_REG      0x14
#define LSM6DSO_PAGE_RW_REG      	  0x17
#define LSM6DSO_EMB_FUNC_FIFO_CFG_REG      0x44
#define LSM6DSO_FSM_ENABLE_A_REG      0x46
#define LSM6DSO_FSM_ENABLE_B_REG      0x47
#define LSM6DSO_FSM_OUTS1_REG      0x4C
#define LSM6DSO_FSM_OUTS2_REG      0x4D
#define LSM6DSO_FSM_OUTS3_REG      0x4E
#define LSM6DSO_FSM_OUTS4_REG      0x4F
#define LSM6DSO_EMB_FUNC_ODR_CFG_B_REG 0x5F
#define LSM6DSO_STEP_COUNTER_L_REG 0x62
#define LSM6DSO_STEP_COUNTER_H_REG 0x63
#define LSM6DSO_EMB_FUNC_SRC_REG 0x64
#define LSM6DSO_EMB_FUNC_INIT_A_REG 0x66
#define LSM6DSO_EMB_FUNC_INIT_B_REG 0x67
/*
* Embedded advanced features page 1
*/
#define LSM6DSO_FSM_LC_TIMEOUT_L_REG      0x7A
#define LSM6DSO_FSM_LC_TIMEOUT_H_REG      0x7B
#define LSM6DSO_FSM_PROGRAMS_REG          0x7C
#define LSM6DSO_FSM_START_ADD_L_REG       0x7E
#define LSM6DSO_FSM_START_ADD_H_REG       0x7F
#define LSM6DSO_PEDO_CMD_REG_REG          0x83
#define LSM6DSO_PEDO_DEB_STEPS_CONF_REG   0x84
#define LSM6DSO_PEDO_SC_DELTAT_L_REG      0xD0
#define LSM6DSO_PEDO_SC_DELTAT_H_REG      0xD1

#define LSM6DSO_EN			 	   0x01
#define LSM6DSO_DIS			   0x00

/*
*  register mask bit
*/
#define LSM6DSO_REG_FUNC_CFG_ACCESS_MASK_FUNC_CFG_EN        0x80
#define LSM6DSO_REG_CTRL1_XL_MASK_FS_XL		       0x0C
#define LSM6DSO_REG_CTRL1_XL_MASK_ODR_XL		       0xF0
#define LSM6DSO_REG_CTRL2_G_MASK_FS_G		       0x0C
#define LSM6DSO_REG_CTRL2_G_MASK_ODR_G		       0xF0

/*
*  register value
*/
#define LSM6DO_FIXED_DEVID			               0x6C

#define LSM6DSO_REG_CTRL1_XL_ODR_416HZ		       0x06
#define LSM6DSO_REG_CTRL1_XL_ODR_208HZ		       0x05
#define LSM6DSO_REG_CTRL1_XL_ODR_104HZ		       0x04
#define LSM6DSO_REG_CTRL1_XL_ODR_52HZ		       0x03
#define LSM6DSO_REG_CTRL1_XL_ODR_26HZ		       0x02
#define LSM6DSO_REG_CTRL1_XL_ODR_13HZ		       0x01
#define LSM6DSO_REG_CTRL1_XL_ODR_0HZ			   0x00

#define LSM6DSO_REG_CTRL1_XL_FS_2G			       0x00
#define LSM6DSO_REG_CTRL1_XL_FS_4G			       0x02
#define LSM6DSO_REG_CTRL1_XL_FS_8G			       0x03
#define LSM6DSO_REG_CTRL1_XL_FS_16G			   0x01

#define LSM6DSO_REG_CTRL2_G_ODR_416HZ			   0x06
#define LSM6DSO_REG_CTRL2_G_ODR_208HZ		       0x05
#define LSM6DSO_REG_CTRL2_G_ODR_104HZ		       0x04
#define LSM6DSO_REG_CTRL2_G_ODR_52HZ			   0x03
#define LSM6DSO_REG_CTRL2_G_ODR_26HZ			   0x02
#define LSM6DSO_REG_CTRL2_G_ODR_0HZ			   0x00

#define LSM6DSO_REG_CTRL2_G_FS_245DPS		       0x00
#define LSM6DSO_REG_CTRL2_G_FS_500DPS		       0x01
#define LSM6DSO_REG_CTRL2_G_FS_1000DPS		       0x02
#define LSM6DSO_REG_CTRL2_G_FS_2000DPS		       0x03

/*
*  return value
*/
#define LSM6DSO_SUCCESS			         0
#define LSM6DSO_ERR_I2C			        -1
#define LSM6DSO_ERR_STATUS					-3
#define LSM6DSO_ERR_SETUP_FAILURE			-4
#define LSM6DSO_ERR_GETGSENSORDATA			-5
#define LSM6DSO_ERR_IDENTIFICATION			-6

#define LSM6DSO_BUFSIZE					64

#define LSM6DSO_AXIS_X          			0
#define LSM6DSO_AXIS_Y          			1
#define LSM6DSO_AXIS_Z          			2
#define LSM6DSO_AXES_NUM        			3
#define LSM6DSO_DATA_LEN        			6
#define LSM6DSO_DEV_NAME        			"LSM6DSO"
//#define DEGREE_TO_RAD_1000		17
/* 1 rad = 180/PI degree, MAX_LSB = 131, */
/* 180*131/PI = 7506 */
#define DEGREE_TO_RAD    				573
//#define CONFIG_LSM6DSO_LOWPASS   /*apply low pass filter on output*/       

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
    s16 raw[C_MAX_FIR_LENGTH][LSM6DSO_AXES_NUM];
    int sum[LSM6DSO_AXES_NUM];
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
struct lsm6dso_acc {
    //struct i2c_client *client;
    u8     name[32];
    struct acc_hw *lsm6dso_acc_hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t                filter;
    s32                     cali_sw[LSM6DSO_AXES_NUM];

    /*data*/
    s32                     offset[LSM6DSO_AXES_NUM];
    s16                     data[LSM6DSO_AXES_NUM];
    bool                    lsm6dso_acc_power;
	int                     odr;
	//int                     acc_enabled;
#if defined(CONFIG_LSM6DSO_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
};

struct lsm6dso_gyro {
    //struct i2c_client *client;
	u8     name[32];
    struct gyro_hw *lsm6dso_gyro_hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t                filter;
    s32                     cali_sw[LSM6DSO_AXES_NUM];

    /*data*/
    s32                     offset[LSM6DSO_AXES_NUM];
    s16                     data[LSM6DSO_AXES_NUM];
    bool                    lsm6dso_gyro_power;
    int                     odr;
    int                     enabled;
#if defined(CONFIG_LSM6DSO_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
};

struct lsm6dso_pedo {
    u8     		    name[32];
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t                filter;

    u32                     data;
    int                     overflow;
    bool                    lsm6dso_pedo_power;
    int                     odr;
  //int                     enabled;
};

struct lsm6dso_data {
	u8     name[32];
	u8     chip_id;
#if(CONFIG_HARDWARE_INTERRUPT)
	int    irq;
    atomic_t   irq_enabled;
	struct work_struct irq_work;
	struct workqueue_struct *irq_work_queue;
#endif
    struct i2c_client *client;
    struct lsm6dso_acc lsm6dso_acc_data;
    struct lsm6dso_gyro lsm6dso_gyro_data;
    struct lsm6dso_pedo lsm6dso_pedo_data;
    u8     reg_addr;
    int    acc_enabled;
    int    gyro_enabled;
    int    step_c_enabled;
    int    step_d_enabled;
    int    significant_enabled;
    int    tilt_enabled;
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

extern struct lsm6dso_data *obj_i2c_data;

int lsm6dso_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lsm6dso_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lsm6dso_i2c_write_with_mask(struct i2c_client *client,u8 addr, u8 mask, u8 data);

void dumpReg(struct lsm6dso_data *obj);
int lsm6dso_set_interrupt(void);

int lsm6dso_acc_set_power_mode(struct lsm6dso_acc *acc_obj, bool enable);
int lsm6dso_gyro_set_power_mode(struct lsm6dso_gyro *gyro_obj, bool enable);

extern struct acc_init_info lsm6dso_acc_init_info;
extern struct gyro_init_info lsm6dso_gyro_init_info;
#if (CONFIG_TILT)
extern struct tilt_init_info  lsm6dso_tilt_init_info ;
#endif
#if (CONFIG_STEP_COUNTER || CONFIG_STEP_DETECT || CONFIG_SIGNIFICANT_MOTION)
extern struct step_c_init_info lsm6dso_pedo_init_info;
#endif
#endif
