/* LSM6DS3C IMU driver
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

#include "lsm6ds3c.h"

#if (CONFIG_STEP_COUNTER || CONFIG_STEP_DETECT || CONFIG_SIGNIFICANT_MOTION || CONFIG_TILT)
static int LSM6DS3C_embeded_feature_enable(int sensor ,int enable)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    struct i2c_client *client = obj->client;
    int enable_bit = 0;
	
    if (enable) {
	enable_bit = LSM6DS3C_EN;
    } else {
	enable_bit = LSM6DS3C_DIS;
    }
   
#if (!CONFIG_PEDOMETER_ALWAYS_ON) 
    //enalbe func 
    if (((obj->tilt_enabled+obj->significant_enabled+obj->step_d_enabled+obj->step_c_enabled) == 0x01) && enable_bit) {
	if ((lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_CTRL10_C, LSM6DS3C_FUNC_ENABLE_MASK, LSM6DS3C_EN)) < 0) {
	    ST_ERR("write LSM6DS3C_REG_CTRL10_C  func enable err!\n");
	    return LSM6DS3C_ERR_I2C;
	}

	if (!obj->acc_enabled) {
	    if ((lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_CTRL1_XL, LSM6DS3C_REG_CTRL1_XL_MASK_ODR_XL, 
                                                LSM6DS3C_REG_CTRL1_XL_ODR_26HZ)) < 0) {
		ST_ERR("write LSM6DS3C_REG_CTRL1_XL  ODR enable err!\n");
		return LSM6DS3C_ERR_I2C;
	    }	
	}
    }

    //disable func 
    if (!(obj->tilt_enabled || obj->significant_enabled || obj->step_d_enabled || obj->step_c_enabled)) {
	if ((lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_CTRL10_C, LSM6DS3C_FUNC_ENABLE_MASK, LSM6DS3C_DIS)) < 0) {
	    ST_ERR("write LSM6DS3C_REG_CTRL10_C  func dis err!\n");
	    return LSM6DS3C_ERR_I2C;
	}

	if (!obj->acc_enabled) {
	    if ((lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_CTRL1_XL, LSM6DS3C_REG_CTRL1_XL_MASK_ODR_XL, 
	        LSM6DS3C_REG_CTRL1_XL_ODR_0HZ)) < 0) {
		ST_ERR("write LSM6DS3C_REG_CTRL1_XL  ODR dis err!\n");
		return LSM6DS3C_ERR_I2C;
	    }		
	}
     }
#endif

#if (!CONFIG_PEDOMETER_ALWAYS_ON)
    if ((sensor == ST_SENSOR_STEP_COUNTER) || (sensor == ST_SENSOR_STEP_DETECT) || (sensor == ST_SENSOR_SIGNIFICANT_MOTION)) {
        if ((((obj->step_d_enabled+obj->step_c_enabled+obj->significant_enabled) == 1) & enable_bit) ||
            (((obj->step_d_enabled+obj->step_c_enabled+obj->significant_enabled) == 0) & (!enable_bit))) {
	    if ((lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_CTRL10_C, LSM6DS3C_STEP_ENABLE_MASK,enable_bit ))<0) {
		ST_ERR("write LSM6DS3C_REG_CTRL10_C step enable err!\n");
		return LSM6DS3C_ERR_I2C;
	    } 
		 
	    ST_LOG("lsm6ds3c enable step ok  en=%d\n",enable_bit);
	}
    }
#endif

    if ((sensor == ST_SENSOR_STEP_DETECT) || (sensor == ST_SENSOR_SIGNIFICANT_MOTION)) {
	if ((((obj->significant_enabled+obj->step_d_enabled) == 0x01) && enable_bit) ||
	    (((obj->significant_enabled+obj->step_d_enabled) == 0x00) && (!enable_bit))) {
	    if ((lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_INT1_CTRL, LSM6DS3C_STEP_INT_MASK, enable_bit)) < 0) {
		ST_ERR("write LSM6DS3C_REG_INT1_CTRL step int enable err!\n");
		return LSM6DS3C_ERR_I2C;
	    } 
	
            ST_LOG("lsm6ds3c enable step d int ok en=%d\n",enable_bit);
	}
    }

    if (sensor == ST_SENSOR_SIGNIFICANT_MOTION) {
	if ((lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_CTRL10_C, LSM6DS3C_SIGN_MOTION_ENABLE_MASK, enable_bit)) < 0) {
	    ST_ERR("write LSM6DS3C_REG_CTRL10_C sig_motion enable err!\n");
	    return LSM6DS3C_ERR_I2C;
	}

        if ((lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_INT1_CTRL, LSM6DS3C_SIGN_MOTION_INT_MASK, enable_bit)) < 0) {
	    ST_ERR("write LSM6DS3C_REG_INT1_CTRL sign_motion int enable err!\n");
	    return LSM6DS3C_ERR_I2C;
	} 
    }

    if (sensor == ST_SENSOR_TILT) {
	if ((lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_CTRL10_C, LSM6DS3C_TILT_ENABLE_MASK, enable_bit)) < 0) {
	    ST_ERR("write  LSM6DS3C_REG_CTRL10_C tilt enable  err!\n");
	    return LSM6DS3C_ERR_I2C;
	}
		 
	if ((lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_MD1_CFG, LSM6DS3C_TILT_INT_MASK, enable_bit)) < 0) {
	    ST_ERR("write LSM6DS3C_REG_MD1_CFG tilt int enable err!\n");
	    return LSM6DS3C_ERR_I2C;
        } 
    }

#if(CONFIG_HARDWARE_INTERRUPT)
   //enable irq
    if (((obj->tilt_enabled+obj->significant_enabled+obj->step_d_enabled) == 0x01) && enable_bit) {
	if (!(atomic_read(&obj->irq_enabled))) {
	    enable_irq(obj->irq);  
	    atomic_set(&obj->irq_enabled,1);
	    ST_LOG("lsm6ds3c enable irq  \n");
        }
    }

    //disable irq
    if ((!(obj->tilt_enabled || obj->significant_enabled || obj->step_d_enabled)) && (!enable_bit)) {	
	if (atomic_read(&obj->irq_enabled)) {  
	    disable_irq_nosync(obj->irq);	 
	    atomic_set(&obj->irq_enabled, 0);
	    ST_LOG("lsm6ds3c disable irq\n");
	}	
    }
#endif	 

    return LSM6DS3C_SUCCESS;	  
}
#endif

//==============tilt================================
#if (CONFIG_TILT)
static int lsm6ds3c_tilt_open_report_data_intf(int en)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    int res = 0;

    obj->tilt_enabled=en;

    res = LSM6DS3C_embeded_feature_enable(ST_SENSOR_TILT, en);
    if (LSM6DS3C_SUCCESS != res) {
	ST_ERR("enable_tilt open failed!\n");
    }

    ST_LOG("enable_tilt open en=%d\n",en);
    return res;
}

static int lsm6ds3c_tilt_set_delay_intf(uint64_t delay)
{
    int res = 0;
    return res;
}

static int lsm6ds3c_tilt_get_data_intf(int *value, int *status)
{
    ST_FUN(); 

#if(!CONFIG_HARDWARE_INTERRUPT)
    tilt_notify();
#endif

    return LSM6DS3C_SUCCESS;
}

static int lsm6ds3c_tilt_local_init(void)
{
    // struct lsm6ds3c_data *obj = obj_i2c_data;
    int res = 0;
    struct tilt_control_path tilt_ctl={0};
    struct tilt_data_path tilt_data={0};

    ST_FUN();

    tilt_ctl.open_report_data = lsm6ds3c_tilt_open_report_data_intf;	
    tilt_ctl.set_delay = lsm6ds3c_tilt_set_delay_intf;
    res = tilt_register_control_path(&tilt_ctl);
    if (res) {
        ST_ERR("register tilt control path err\n");
        return res;
    }

    tilt_data.get_data = lsm6ds3c_tilt_get_data_intf;
    res = tilt_register_data_path(&tilt_data);
    if (res) {
        ST_ERR("register tilt data path err\n");
        return res;
    }
	
    return LSM6DS3C_SUCCESS;
}

static int lsm6ds3c_tilt_local_remove(void)
{
    ST_FUN();
    return LSM6DS3C_SUCCESS;
}

struct tilt_init_info  lsm6ds3c_tilt_init_info = {
    .name   = "LSM6DS3C_TILT",
    .init   = lsm6ds3c_tilt_local_init,
    .uninit = lsm6ds3c_tilt_local_remove,
};
#endif

//===========step_counter , step_detect , sign_motion ==========
#if (CONFIG_STEP_COUNTER || CONFIG_STEP_DETECT || CONFIG_SIGNIFICANT_MOTION)
static int lsm6ds3c_pedo_reset_counter(struct lsm6ds3c_pedo *pedo_obj)
{
    struct lsm6ds3c_data *obj = container_of(pedo_obj, struct lsm6ds3c_data, lsm6ds3c_pedo_data);
    struct i2c_client *client = obj->client; 
    int res = 0;

    res = lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_CTRL10_C, LSM6DS3C_REG_CTRL10_C_MASK_PEDO_RST_STEP,LSM6DS3C_EN);
    if (res < 0) {
        ST_ERR("pedometer reset err!\n");
        return LSM6DS3C_ERR_I2C;
    }

    return LSM6DS3C_SUCCESS;
}

static int lsm6ds3c_pedo_init(struct lsm6ds3c_pedo *pedo_obj, int reset_cali)
{
    return LSM6DS3C_SUCCESS ;//lsm6ds3c_pedo_reset_counter(pedo_obj);
}

static int lsm6ds3c_pedo_read_chip_name(struct lsm6ds3c_pedo *pedo_obj, u8 *buf, int bufsize)
{
    sprintf(buf, "%s", pedo_obj->name);
    return LSM6DS3C_SUCCESS;
}

static int lsm6ds3c_pedo_read_data(struct lsm6ds3c_pedo *pedo_obj, u32 *value)
{
    struct lsm6ds3c_data *obj = container_of(pedo_obj, struct lsm6ds3c_data, lsm6ds3c_pedo_data);
    struct i2c_client *client = obj->client;
    u16 counter = 0;
    static bool clear = false;
    int res = 0;

    if (atomic_read(&pedo_obj->suspend)) {
        ST_LOG("sensor in suspend read not data!\n");
        return 0;
    }
 
    res = lsm6ds3c_i2c_read_block(client, LSM6DS3C_REG_STEP_COUNTER_L, (u8*)&counter, 0x02);
    if (res < 0) {
        ST_ERR("read pedometer sensor data register err!\n");
        counter = 0; 
        return LSM6DS3C_ERR_I2C;
    } else {   
        if (clear) {
	    res = lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_CTRL10_C, LSM6DS3C_REG_CTRL10_C_MASK_PEDO_RST_STEP,
					         LSM6DS3C_DIS);
            if (res < 0) {
                ST_ERR("pedometer clear reset  flag err!\n");
                return LSM6DS3C_ERR_I2C;
            }

	    clear = false;
        }
	
        if (counter > 60000) {
            ST_LOG("step reset counter data: %d!\n", *value);
            lsm6ds3c_pedo_reset_counter(pedo_obj);
	    counter -= 60000;
            pedo_obj->overflow++;
	    clear = true;
        }
    }	
		

    pedo_obj->data = counter + pedo_obj->overflow*60000; 
    *value = pedo_obj->data;

    if (atomic_read(&pedo_obj->trace) & ADX_TRC_IOCTL) {
        ST_LOG("step counter data: %d!\n", *value);
        dumpReg(obj);
    }
    
    return LSM6DS3C_SUCCESS;
}

static ssize_t lsm6ds3c_attr_pedo_show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    struct lsm6ds3c_pedo *pedo_obj = &obj->lsm6ds3c_pedo_data;
    u8 strbuf[LSM6DS3C_BUFSIZE];
	
    if(NULL == obj->client){
        ST_ERR("i2c client is null!!\n");
        return 0;
    }
    
    lsm6ds3c_pedo_read_chip_name(pedo_obj, strbuf, LSM6DS3C_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

static ssize_t lsm6ds3c_attr_pedo_show_chipid_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    u8 strbuf[LSM6DS3C_BUFSIZE] = "unkown chipid";
	
    if (NULL == obj->client) {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }

    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t lsm6ds3c_attr_pedo_show_sensordata_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    struct lsm6ds3c_pedo *pedo_obj = &obj->lsm6ds3c_pedo_data;
    u32 value = 0;
    int res = 0;
    
    if (NULL == obj->client) {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }

    res = lsm6ds3c_pedo_read_data(pedo_obj, &value);

    return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t lsm6ds3c_attr_pedo_show_power_status(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    struct i2c_client *client = obj->client;
    u8 data;

    if (NULL == obj->client) {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }

    lsm6ds3c_i2c_read_block(client, LSM6DS3C_REG_CTRL1_XL, &data, 0x01);

    return snprintf(buf, PAGE_SIZE, "%x\n", data);
}

static ssize_t lsm6ds3c_attr_pedo_show_trace_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    struct lsm6ds3c_pedo *pedo_obj = &obj->lsm6ds3c_pedo_data; 
    ssize_t res;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&pedo_obj->trace));
 
    return res;
}

static ssize_t lsm6ds3c_attr_pedo_store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    struct lsm6ds3c_pedo *pedo_obj = &obj->lsm6ds3c_pedo_data; 
    int trace;
	
    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if (1 == sscanf(buf, "0x%x", &trace)) {
        atomic_set(&pedo_obj->trace, trace);
    } else {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }
    
    return count;    
}

static ssize_t lsm6ds3c_attr_pedo_show_chipinit_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    struct lsm6ds3c_pedo *pedo_obj = &obj->lsm6ds3c_pedo_data;
    ssize_t res;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&pedo_obj->trace)); 

    return res;
}

static ssize_t lsm6ds3c_attr_pedo_store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    struct lsm6ds3c_pedo *pedo_obj = &obj->lsm6ds3c_pedo_data;

    if (obj == NULL){
        ST_ERR("i2c_data obj is null!!\n");
        return count;
    }

    lsm6ds3c_pedo_init(pedo_obj, 0);
    dumpReg(obj);

    return count;
}

static DRIVER_ATTR(chipinfo,           S_IRUGO, lsm6ds3c_attr_pedo_show_chipinfo_value,   NULL);
static DRIVER_ATTR(chipid,             S_IRUGO, lsm6ds3c_attr_pedo_show_chipid_value,     NULL);
static DRIVER_ATTR(sensordata,         S_IRUGO, lsm6ds3c_attr_pedo_show_sensordata_value, NULL);
static DRIVER_ATTR(power,              S_IRUGO, lsm6ds3c_attr_pedo_show_power_status,     NULL);
static DRIVER_ATTR(trace,    S_IWUSR | S_IRUGO, lsm6ds3c_attr_pedo_show_trace_value,      lsm6ds3c_attr_pedo_store_trace_value);
static DRIVER_ATTR(chipinit, S_IWUSR | S_IRUGO, lsm6ds3c_attr_pedo_show_chipinit_value,   lsm6ds3c_attr_pedo_store_chipinit_value);

static struct driver_attribute *lsm6ds3c_attr_pedo_list[] = {
    &driver_attr_chipinfo,     /*chip information*/
    &driver_attr_chipid,       /*chip id*/
    &driver_attr_sensordata,   /*dump sensor data*/
    &driver_attr_power,        /*show power reg*/
    &driver_attr_trace,        /*trace log*/
    &driver_attr_chipinit,
};

int lsm6ds3c_pedo_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(lsm6ds3c_attr_pedo_list)/sizeof(lsm6ds3c_attr_pedo_list[0]));

    if (driver == NULL) {
        return -EINVAL;
    }

    for (idx = 0; idx < num; idx++) {
        if ((err = driver_create_file(driver, lsm6ds3c_attr_pedo_list[idx]))) {            
            ST_ERR("driver_create_file (%s) = %d\n", lsm6ds3c_attr_pedo_list[idx]->attr.name, err);
            break;
        }
    }
 
    return err;
}

int lsm6ds3c_pedo_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(lsm6ds3c_attr_pedo_list)/sizeof(lsm6ds3c_attr_pedo_list[0]));

    if (driver == NULL) {
        return -EINVAL;
    }
    
    for (idx = 0; idx < num; idx++) {
        driver_remove_file(driver, lsm6ds3c_attr_pedo_list[idx]);
    }
    
    return err;
}

static int lsm6ds3c_step_counter_open_report_data_intf(int open)
{
    return LSM6DS3C_SUCCESS;
}

static int lsm6ds3c_step_counter_enable_nodata_intf(int en)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    int res = 0; 

    obj->step_c_enabled = en;
    res = LSM6DS3C_embeded_feature_enable(ST_SENSOR_STEP_COUNTER,en);    
    if (res) {
        ST_ERR("lsm6ds3c_step_counter_enable fail!\n");
        return res;
    }

    ST_LOG("lsm6ds3c_step_counter_enable_nodata_intf OK! en=%d\n", en);
    return LSM6DS3C_SUCCESS;
}

#if (CONFIG_SIGNIFICANT_MOTION || CONFIG_STEP_COUNTER)
static int lsm6ds3c_enable_sigificant_intf(int en)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    int res = 0;

    obj->significant_enabled = en;
    res = LSM6DS3C_embeded_feature_enable(ST_SENSOR_SIGNIFICANT_MOTION, en);    
    if (res) {
        ST_ERR("lsm6ds3c_significant_enable fail!\n");
        return res;
    }

    ST_LOG("lsm6ds3c_significant_enable_intf OK! en=%d\n", en);
    return LSM6DS3C_SUCCESS;
}
#endif

#if(CONFIG_STEP_DETECT||CONFIG_STEP_COUNTER)
static int lsm6ds3c_enable_step_dectect_intf(int en)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    int res = 0;

    obj->step_d_enabled = en;
    res = LSM6DS3C_embeded_feature_enable(ST_SENSOR_STEP_DETECT, en);    
    if (res) {
        ST_ERR("lsm6ds3c_step_detect_enable fail!\n");
        return res;
    }

    ST_LOG("lsm6ds3c_step_detect_enable_intf OK! en=%d \n", en);
    return 0;
}
#endif


#if(CONFIG_STEP_DETECT||CONFIG_STEP_COUNTER)
static int lsm6ds3c_step_d_set_delay_intf(u64 ns)
{
    return LSM6DS3C_SUCCESS;
}
#endif

static int lsm6ds3c_step_c_set_delay_intf(u64 ns)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    struct lsm6ds3c_pedo *pedo_obj = &obj->lsm6ds3c_pedo_data;
    int value = 0;
    int sample_delay = 0;
    int err = 0;
	
    value = (int)ns/1000/1000;
    if (value <= 5) {
        sample_delay = LSM6DS3C_REG_CTRL1_XL_ODR_208HZ;
    } else if(value <= 10) {
        sample_delay = LSM6DS3C_REG_CTRL1_XL_ODR_104HZ;
    } else {
        sample_delay = LSM6DS3C_REG_CTRL1_XL_ODR_52HZ;
    }

    pedo_obj->odr = sample_delay;
    if (err != LSM6DS3C_SUCCESS ) {
        ST_ERR("Set delay parameter error!\n");
    }

    ST_LOG("lsm6ds3c_pedo_set_delay_intf (%d)\n",value);
    return LSM6DS3C_SUCCESS;
}

#if (CONFIG_SIGNIFICANT_MOTION || CONFIG_STEP_COUNTER)
static int lsm6ds3c_get_data_significant_intf(uint32_t *value, int *status)
{
#if (!CONFIG_HARDWARE_INTERRUPT)
    step_notify(TYPE_SIGNIFICANT);
#endif
    return LSM6DS3C_SUCCESS;
}
#endif


#if (CONFIG_STEP_DETECT || CONFIG_STEP_COUNTER)
static int lsm6ds3c_get_data_step_d_intf(uint32_t *value, int *status)
{
#if (!CONFIG_HARDWARE_INTERRUPT)
    step_notify(TYPE_STEP_DETECTOR);
#endif
    return LSM6DS3C_SUCCESS;
}
#endif

static int lsm6ds3c_get_data_step_c_intf(uint32_t *value, int *status)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    struct lsm6ds3c_pedo *pedo_obj = &obj->lsm6ds3c_pedo_data;
    int res = 0;

    res = lsm6ds3c_pedo_read_data(pedo_obj, value);
    if (res<0){
        ST_ERR("lsm6ds3c_pedo_read_data error\n");
        return res;
    }

    *status = SENSOR_STATUS_ACCURACY_HIGH;

    return LSM6DS3C_SUCCESS;
}

static int lsm6ds3c_pedo_local_init(void)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    struct lsm6ds3c_pedo *pedo_obj = &obj->lsm6ds3c_pedo_data;
    int res = 0;
    int retry = 0;
    struct step_c_control_path ctl= {0};
    struct step_c_data_path data = {0};

    ST_FUN();

    atomic_set(&pedo_obj->trace, 0);
    atomic_set(&pedo_obj->suspend, 0);
    
    if ((res = lsm6ds3c_pedo_create_attr(&(lsm6ds3c_pedo_init_info.platform_diver_addr->driver)))) {
        ST_ERR("create attribute err = %d\n", res);
        goto exit_create_attr_failed;
    }
	
    for (retry = 0; retry < 3; retry++) {
	res = lsm6ds3c_pedo_init(pedo_obj, 1);
	if (res == 0) {
	    break;
	} else {
	    ST_ERR("lsm6ds3c_pedo_device init cilent fail time: %d\n", retry);
	}		
    }

    if (res != 0)
        goto exit;


    sprintf(pedo_obj->name, "%s_PEDO", obj->name);

    ctl.open_report_data       = lsm6ds3c_step_counter_open_report_data_intf;
    ctl.enable_nodata          = lsm6ds3c_step_counter_enable_nodata_intf;
    ctl.enable_step_detect     = lsm6ds3c_enable_step_dectect_intf;
    ctl.enable_significant     = lsm6ds3c_enable_sigificant_intf;
    ctl.step_c_set_delay       = lsm6ds3c_step_c_set_delay_intf;
    ctl.step_d_set_delay       = lsm6ds3c_step_d_set_delay_intf;
    ctl.is_report_input_direct = false;
    ctl.is_support_batch       = false;

    res = step_c_register_control_path(&ctl);
    if (res) {
        ST_ERR("register acc control path err\n");
        goto exit;
    }

    data.get_data             = lsm6ds3c_get_data_step_c_intf;
    data.vender_div           = 1;
    data.get_data_step_d      = lsm6ds3c_get_data_step_d_intf;
    data.get_data_significant = lsm6ds3c_get_data_significant_intf;
	
    res = step_c_register_data_path(&data);
    if (res) {
        ST_ERR("register acc data path err\n");
        goto exit;
    }

    ST_LOG("%s: OK\n", __func__);    
    return LSM6DS3C_SUCCESS;

exit:
    ST_ERR("%s: err = %d\n", __func__, res);
    lsm6ds3c_pedo_delete_attr(&(lsm6ds3c_pedo_init_info.platform_diver_addr->driver));
exit_create_attr_failed:       
    return -1;
}

static int lsm6ds3c_pedo_local_remove(void)
{
    ST_FUN(); 
    lsm6ds3c_pedo_delete_attr(&(lsm6ds3c_pedo_init_info.platform_diver_addr->driver));
    
    return LSM6DS3C_SUCCESS;
}

struct step_c_init_info lsm6ds3c_pedo_init_info = {
    .name   = "lsm6ds3c_step",
    .init   = lsm6ds3c_pedo_local_init,
    .uninit = lsm6ds3c_pedo_local_remove,
};
#endif

MODULE_DESCRIPTION("STMicroelectronics lsm6ds3c driver");
MODULE_AUTHOR("Ian Yang, William Zeng");
MODULE_LICENSE("GPL v2");
