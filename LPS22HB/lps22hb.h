
#ifndef LPS22HB_H
#define LPS22HB_H
	 
#include <linux/ioctl.h>
#include <barometer.h>
#include <hwmsensor.h>
#include <cust_baro.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE
	 
/* LPS22HB Register Map  (Please refer to LPS22HB Specifications) */
#define LPS22HB_REG_WHO_AM_I		0x0F
#define LPS22HB_REG_CTRL_REG1		0x10
#define LPS22HB_REG_CTRL_REG2		0x11
#define LPS22HB_REG_CTRL_REG3		0x12
#define LPS22HB_REG_REF_P_XL		0x15
#define LPS22HB_REG_REF_P_L			0x16
#define LPS22HB_REG_REF_P_H			0x17
#define LPS22HB_REG_RES_CONF		0x1A
#define LPS22HB_REG_PRESS_OUT_XL	0x28
#define LPS22HB_REG_PRESS_OUT_L		0x29
#define LPS22HB_REG_PRESS_OUT_H		0x2A
#define LPS22HB_REG_TEMP_OUT_L		0x2B
#define LPS22HB_REG_TEMP_OUT_H		0x2C


#define LPS22HB_FIXED_DEVID			0xB1
	 
#define LPS22HB_BW_75HZ				0x50
#define LPS22HB_BW_50HZ				0x40
#define LPS22HB_BW_25HZ				0x30
#define LPS22HB_BW_10HZ            	0x20
#define LPS22HB_BW_1HZ            	0x10
#define LPS22HB_BW_0HZ             	0x00
	 
#define LPS22HB_SUCCESS						0
#define LPS22HB_ERR_I2C						-1
#define LPS22HB_ERR_STATUS					-3
#define LPS22HB_ERR_SETUP_FAILURE			-4
#define LPS22HB_ERR_GETGSENSORDATA			-5
#define LPS22HB_ERR_IDENTIFICATION			-6

#define LPS22HB_BUFSIZE						256

#define LPS22HB_AXES_NUM        3
#define LPS22HB_DATA_LEN        6
#define LPS22HB_DEV_NAME        "LPS22HB"

#define CONFIG_LPS22HB_LOWPASS   /*apply low pass filter on output*/       

#define CONFIG_LPS22HB_BARO_DRY

#define LPS22HB_BARO_DIV				100
#define LPS22HB_BARO_SENSITIVITY		4096
#define LPS22HB_TEMP_SENSITIVITY		100

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
struct lps22hb_baro {
    //struct i2c_client *client;
    struct baro_hw *lps22hb_baro_hw;
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
    bool                    lps22hb_baro_power;
	int                     odr;
	int                     enabled;
#if defined(CONFIG_LPS22HB_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
};

struct lps22hb_data {
    struct i2c_client *client;
    struct lps22hb_baro lps22hb_baro_data;
    u8     reg_addr;
};

#define ST_TAG                  "[ST] "
#define ST_ERR(fmt, args...)    printk(KERN_ERR ST_TAG "%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define ST_LOG(fmt, args...)    printk(KERN_ERR ST_TAG "%s %d : "fmt, __FUNCTION__, __LINE__, ##args)

#if defined(DEBUG)
    #define ST_FUN(f)               printk(KERN_INFO ST_TAG "%s\n", __FUNCTION__)
	#define ST_DBG(fmt, args...)    printk(KERN_ERR ST_TAG fmt, ##args)
#else
	#define ST_FUN(f)
	#define ST_DBG(fmt, args...)
#endif

int lps22hb_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lps22hb_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);

void dumpReg(struct lps22hb_data *obj);
int lps22hb_set_interrupt(struct lps22hb_data *obj, u8 intenable);

int lps22hb_baro_set_power_mode(struct lps22hb_baro *baro_obj, bool enable);
int lps22hb_baro_init(struct lps22hb_baro *baro_obj, int reset_cali);

//extern struct i2c_client *lps22hb_i2c_client;
extern struct lps22hb_data *obj_i2c_data;
extern int sensor_suspend;
extern struct baro_init_info lps22hb_baro_init_info;
extern int lps22hb_baro_init_flag;

#endif
