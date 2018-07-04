#include "lis2ds12.h"

#if (CONFIG_STEP_COUNTER || CONFIG_STEP_DETECT || CONFIG_SIGNIFICANT_MOTION || CONFIG_TILT)
static int LIS2DS12_embeded_feature_enable(int sensor ,int enable)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct i2c_client *client = obj->client;
    int enable_bit = 0;
    u8 dat;
	
    if (enable)
	enable_bit = LIS2DS12_EN;
    else
	enable_bit = LIS2DS12_DIS;
   
#if (!CONFIG_PEDOMETER_ALWAYS_ON) 
    if (((obj->tilt_enabled + obj->significant_enabled + obj->step_d_enabled + obj->step_c_enabled) == 0x01) && enable_bit) {
	if (!obj->acc_enabled) {
	    //enable XL, set ODR to 25Hz
	    if ((lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_CTRL1, LIS2DS12_REG_CTRL1_MASK_ODR, 
		 LIS2DS12_REG_CTRL1_ODR_25HZ)) < 0) {
		ST_ERR("write LIS2DS12_REG_CTRL1 ODR enable err!\n");
		return LIS2DS12_ERR_I2C;
	    }
				
    	    lis2ds12_i2c_read_block(client, LIS2DS12_REG_CTRL1, &dat, 0x01);
    	    ST_ERR("LIS2DS12_REG_CTRL1: 0x%02x\n", dat);
	}
    }
   
    if (!(obj->tilt_enabled || obj->significant_enabled || obj->step_d_enabled || obj->step_c_enabled)) {
	if (!obj->acc_enabled) {
	    //disable XL, set ODR to 0Hz
	    if ((lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_CTRL1, LIS2DS12_REG_CTRL1_MASK_ODR, 
		 LIS2DS12_REG_CTRL1_ODR_0HZ)) < 0) {
		ST_ERR("write LIS2DS12_REG_CTRL1 ODR dis err!\n");
		return LIS2DS12_ERR_I2C;
	    }		
    	
	}
    }
#endif

#if (!CONFIG_PEDOMETER_ALWAYS_ON)
    //pedometer algorithm enable/disable
    if ((sensor == ST_SENSOR_STEP_COUNTER) || (sensor == ST_SENSOR_STEP_DETECT) || (sensor == ST_SENSOR_SIGNIFICANT_MOTION)) {
    	if ((((obj->step_d_enabled + obj->step_c_enabled + obj->significant_enabled) == 1) & enable_bit) ||
            (((obj->step_d_enabled + obj->step_c_enabled + obj->significant_enabled) == 0) & (!enable_bit))) {
	    if ((lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_FUNC_CTRL, LIS2DS12_STEP_ENABLE_MASK, enable_bit)) < 0) {
		ST_ERR("write LIS2DS12_REG_FUNC_CTRL step enable err!\n");
		return LIS2DS12_ERR_I2C;
	    } 
		
	    ST_LOG("LIS2DS12 enable step ok en=%d\n", enable_bit);
	}
    }
#endif

	//Pedometer step recognition interrupt enable/disable on INT2 pad
    if ((sensor == ST_SENSOR_STEP_DETECT) || (sensor == ST_SENSOR_SIGNIFICANT_MOTION)) {
        if ((((obj->significant_enabled + obj->step_d_enabled) == 0x01) && enable_bit) ||
	    (((obj->significant_enabled + obj->step_d_enabled) == 0x00) && (!enable_bit))) {
	    if ((lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_CTRL5, LIS2DS12_STEP_INT_MASK, enable_bit)) < 0) {
		ST_ERR("write LIS2DS12_REG_CTRL5 step int enable err!\n");
		return LIS2DS12_ERR_I2C;
	    } 
		        
	    ST_LOG("LIS2DS12 enable step d int ok en=%d\n", enable_bit);
	}
    }
	
    if (sensor == ST_SENSOR_SIGNIFICANT_MOTION) {
	//enable significant motion function 
	if ((lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_FUNC_CTRL, LIS2DS12_SIGN_MOTION_ENABLE_MASK, enable_bit)) < 0) {
	    ST_ERR("write LIS2DS12_REG_FUNC_CTRL sig_motion enable err!\n");
	    return LIS2DS12_ERR_I2C;
	}

	//significant motion interrupt enable on INT1 pad
        if ((lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_CTRL5, LIS2DS12_SIGN_MOTION_INT_MASK, enable_bit)) < 0) {
	    ST_ERR("write LIS2DS12_REG_CTRL5 sign_motion int enable err!\n");
	    return LIS2DS12_ERR_I2C;
	} 
    }

    if (sensor == ST_SENSOR_TILT) {
	//Tilt calculation enable
	if ((lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_FUNC_CTRL, LIS2DS12_TILT_ENABLE_MASK, enable_bit)) < 0) {
	    ST_ERR("write  LIS2DS12_REG_FUNC_CTRL tilt enable  err!\n");
	    return LIS2DS12_ERR_I2C;
	}

	//Routing of tilt event on INT1
	if ((lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_CTRL5, LIS2DS12_TILT_INT_MASK, enable_bit)) < 0) {
	    ST_ERR("write LIS2DS12_REG_CTRL5 tilt int enable err!\n");
	    return LIS2DS12_ERR_I2C;
	} 
    }

#if (CONFIG_HARDWARE_INTERRUPT)
    //enable irq
    if (((obj->tilt_enabled + obj->significant_enabled + obj->step_d_enabled) == 0x01) && enable_bit) {
	if (!(atomic_read(&obj->irq_enabled))) {
	    enable_irq(obj->irq);  
	    atomic_set(&obj->irq_enabled,1);
	    ST_LOG("LIS2DS12 enable irq  \n");
        }
    }
	
    //disable irq
    if ((!(obj->tilt_enabled || obj->significant_enabled || obj->step_d_enabled)) && (!enable_bit)) {	
	if (atomic_read(&obj->irq_enabled)) {  
	    disable_irq_nosync(obj->irq);	 
	    atomic_set(&obj->irq_enabled,0);
	    ST_LOG("LIS2DS12 disable irq  \n");
	}
    }
#endif	 

    return LIS2DS12_SUCCESS;	  
}
#endif

//==============tilt================================
#if (CONFIG_TILT)
static int lis2ds12_tilt_open_report_data_intf(int en)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    int ret = 0;

    obj->tilt_enabled = en;

    ret = LIS2DS12_embeded_feature_enable(ST_SENSOR_TILT, en);
    if (LIS2DS12_SUCCESS != ret) {
	ST_ERR("enable_tilt  open failed!\n");
    }
	
    ST_LOG("enable_tilt open en=%d\n", en);
    return ret;
}

static int lis2ds12_tilt_set_delay_intf(uint64_t delay)
{
    return LIS2DS12_SUCCESS;
}

static int lis2ds12_tilt_get_data_intf(int *value, int *status)
{
    ST_FUN(); 

#if(!CONFIG_HARDWARE_INTERRUPT)
    tilt_notify();
#endif

    return LIS2DS12_SUCCESS;
}

static int lis2ds12_tilt_local_init(void)
{
    int ret = 0;
    struct tilt_control_path tilt_ctl = {0};
    struct tilt_data_path tilt_data = {0};

    ST_FUN();
	
    tilt_ctl.open_report_data = lis2ds12_tilt_open_report_data_intf;	
    tilt_ctl.set_delay = lis2ds12_tilt_set_delay_intf;
    ret = tilt_register_control_path(&tilt_ctl);
    if (ret) {
	ST_ERR("register tilt control path err\n");
	return ret;
    }

    tilt_data.get_data = lis2ds12_tilt_get_data_intf;
    ret = tilt_register_data_path(&tilt_data);
    if (ret) {
        ST_ERR("register tilt data path err\n");
        return ret;
    }
	
    return LIS2DS12_SUCCESS;
}

static int lis2ds12_tilt_local_remove(void)
{
    ST_FUN();
    return LIS2DS12_SUCCESS;
}

struct tilt_init_info  lis2ds12_tilt_init_info = {
    .name   = "LIS2DS12_TILT",
    .init   = lis2ds12_tilt_local_init,
    .uninit = lis2ds12_tilt_local_remove,
};
#endif

//===========step_counter , step_detect , sign_motion ==========
#if (CONFIG_STEP_COUNTER || CONFIG_STEP_DETECT || CONFIG_SIGNIFICANT_MOTION)
static int lis2ds12_pedo_reset_counter(struct lis2ds12_pedo *pedo_obj)
{
    struct lis2ds12_data *obj = container_of(pedo_obj, struct lis2ds12_data, lis2ds12_pedo_data);
    struct i2c_client *client = obj->client; 
    int ret = 0;

    ret = lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_STEP_COUNTER_MINTHS, 
				       LIS2DS12_REG_STEP_COUNTER_MINTHS_MASK_PEDO_RST_STEP, LIS2DS12_EN);
    if (ret < 0) {
        ST_ERR("pedometer reset  err!\n");
        return LIS2DS12_ERR_I2C;
    }

    return LIS2DS12_SUCCESS;
}

static int lis2ds12_pedo_init(struct lis2ds12_pedo *pedo_obj, int reset_cali)
{
    return LIS2DS12_SUCCESS ;//lis2ds12_pedo_reset_counter(pedo_obj);
}

static int lis2ds12_pedo_read_chip_name(struct lis2ds12_pedo *pedo_obj, u8 *buf, int bufsize)
{
    sprintf(buf, "%s", pedo_obj->name);
    return LIS2DS12_SUCCESS;
}

static int lis2ds12_pedo_read_data(struct lis2ds12_pedo *pedo_obj, u32 *value)
{
    struct lis2ds12_data *obj = container_of(pedo_obj, struct lis2ds12_data, lis2ds12_pedo_data);
    struct i2c_client *client = obj->client;
    u16 counter = 0;
    static bool clear = false;
    int ret = 0;

    if (atomic_read(&pedo_obj->suspend)) {
        ST_LOG("sensor in suspend read not data!\n");
        return 0;
    }
 
    ret = lis2ds12_i2c_read_block(client, LIS2DS12_REG_STEP_COUNTER_L, (u8*)&counter, 0x02);
    if (ret < 0) {
        ST_ERR("read pedometer sensor data register err!\n");
        counter = 0; 
        return LIS2DS12_ERR_I2C;
    } else {   
        if (clear) {
	    ret = lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_STEP_COUNTER_MINTHS, 
					       LIS2DS12_REG_STEP_COUNTER_MINTHS_MASK_PEDO_RST_STEP, LIS2DS12_DIS);
            if (ret < 0) {
                ST_ERR("pedometer clear reset  flag err!\n");
                return LIS2DS12_ERR_I2C;
            }
	
	    clear = false;
        }
	
        if (counter > 60000) {
            ST_LOG("step reset counter data: %d!\n", *value);
            lis2ds12_pedo_reset_counter(pedo_obj);
            pedo_obj->overflow++;
	    counter -= 60000;
	    clear = true;
        }
    }	
		
    pedo_obj->data = counter + pedo_obj->overflow * 60000; 
    *value = pedo_obj->data;

    if (atomic_read(&pedo_obj->trace) & ADX_TRC_IOCTL) {
        ST_LOG("step counter data: %d!\n", *value);
        dumpReg(obj);
    }
    
    return LIS2DS12_SUCCESS;
}

static ssize_t lis2ds12_attr_pedo_show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_pedo *pedo_obj = &obj->lis2ds12_pedo_data;
    u8 strbuf[LIS2DS12_BUFSIZE];
	
    if (NULL == obj->client) {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }
    
    lis2ds12_pedo_read_chip_name(pedo_obj, strbuf, LIS2DS12_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

static ssize_t lis2ds12_attr_pedo_show_chipid_value(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    u8 strbuf[LIS2DS12_BUFSIZE] = "chipid is 0x43";
	
    if (NULL == obj->client) {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }

    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t lis2ds12_attr_pedo_show_sensordata_value(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_pedo *pedo_obj = &obj->lis2ds12_pedo_data;
    u32 value = 0;
    int ret = 0;
    
    if (NULL == obj->client) {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }
    
    ret = lis2ds12_pedo_read_data(pedo_obj, &value);
	
    return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t lis2ds12_attr_pedo_show_power_status(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct i2c_client *client = obj->client;
    u8 data;

    if (NULL == obj->client) {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }

    lis2ds12_i2c_read_block(client, LIS2DS12_REG_CTRL1, &data, 0x01);
    return snprintf(buf, PAGE_SIZE, "%x\n", data);
}

static ssize_t lis2ds12_attr_pedo_show_trace_value(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_pedo *pedo_obj = &obj->lis2ds12_pedo_data; 
    ssize_t ret;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    ret = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&pedo_obj->trace));     
    return ret;
}

static ssize_t lis2ds12_attr_pedo_store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_pedo *pedo_obj = &obj->lis2ds12_pedo_data; 
    int trace;
	
    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if (1 == sscanf(buf, "0x%x", &trace))
        atomic_set(&pedo_obj->trace, trace); 
    else
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    
    return count;    
}

static ssize_t lis2ds12_attr_pedo_show_chipinit_value(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_pedo *pedo_obj = &obj->lis2ds12_pedo_data;
    ssize_t ret;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    ret = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&pedo_obj->trace)); 
    return ret;
}

static ssize_t lis2ds12_attr_pedo_store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_pedo *pedo_obj = &obj->lis2ds12_pedo_data;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return count;
    }

    lis2ds12_pedo_init(pedo_obj, 0);

    return count;
}

static DRIVER_ATTR(chipinfo,           S_IRUGO, lis2ds12_attr_pedo_show_chipinfo_value,   NULL);
static DRIVER_ATTR(chipid,             S_IRUGO, lis2ds12_attr_pedo_show_chipid_value,     NULL);
static DRIVER_ATTR(sensordata,         S_IRUGO, lis2ds12_attr_pedo_show_sensordata_value, NULL);
static DRIVER_ATTR(power,              S_IRUGO, lis2ds12_attr_pedo_show_power_status,     NULL);
static DRIVER_ATTR(trace,    S_IWUSR | S_IRUGO, lis2ds12_attr_pedo_show_trace_value,      lis2ds12_attr_pedo_store_trace_value);
static DRIVER_ATTR(chipinit, S_IWUSR | S_IRUGO, lis2ds12_attr_pedo_show_chipinit_value,   lis2ds12_attr_pedo_store_chipinit_value);

static struct driver_attribute *lis2ds12_attr_pedo_list[] = {
    &driver_attr_chipinfo,     /*chip information*/
    &driver_attr_chipid,       /*chip id*/
    &driver_attr_sensordata,   /*dump sensor data*/
    &driver_attr_power,        /*show power reg*/
    &driver_attr_trace,        /*trace log*/
    &driver_attr_chipinit,
};

int lis2ds12_pedo_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(lis2ds12_attr_pedo_list)/sizeof(lis2ds12_attr_pedo_list[0]));
	
    if (driver == NULL)
        return -EINVAL;

    for (idx = 0; idx < num; idx++) {
        if ((err = driver_create_file(driver, lis2ds12_attr_pedo_list[idx]))) {
            ST_ERR("driver_create_file (%s) = %d\n", lis2ds12_attr_pedo_list[idx]->attr.name, err);
            break;
        }
    }
	
    return err;
}

int lis2ds12_pedo_delete_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(lis2ds12_attr_pedo_list)/sizeof(lis2ds12_attr_pedo_list[0]));

    if (driver == NULL)
        return -EINVAL;
    
    for (idx = 0; idx < num; idx++)
        driver_remove_file(driver, lis2ds12_attr_pedo_list[idx]);
    
    return err;
}

/*----------------------------------------------------------------------------*/
static int lis2ds12_step_counter_open_report_data_intf(int open)
{
    return LIS2DS12_SUCCESS;
}

static int lis2ds12_step_counter_enable_nodata_intf(int en)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    int ret =0;

    obj->step_c_enabled = en;
    ret = LIS2DS12_embeded_feature_enable(ST_SENSOR_STEP_COUNTER, en);    
    if (ret) {
        ST_ERR("lis2ds12_step_counter_enable fail!\n");
        return ret;
    }
	
    ST_LOG("lis2ds12_step_counter_enable_nodata_intf OK! en=%d\n", en);
    return LIS2DS12_SUCCESS;
}

#if (CONFIG_SIGNIFICANT_MOTION)
static int lis2ds12_enable_sigificant_intf(int en)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    int ret = 0;

    obj->significant_enabled = en;
    ret = LIS2DS12_embeded_feature_enable(ST_SENSOR_SIGNIFICANT_MOTION, en);    
    if (ret) {
        ST_ERR("lis2ds12_significant_enable fail!\n");
        return ret;
    }
	
    ST_LOG("lis2ds12_significant_enable_intf OK! en=%d\n", en);
    return LIS2DS12_SUCCESS;
}
#endif

#if (CONFIG_STEP_DETECT || CONFIG_STEP_COUNTER)
static int lis2ds12_enable_step_dectect_intf(int en)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    int ret = 0;

    obj->step_d_enabled = en;

    ret = LIS2DS12_embeded_feature_enable(ST_SENSOR_STEP_DETECT, en);    
    if (ret) {
        ST_ERR("lis2ds12_step_detect_enable fail!\n");
        return ret;
    }
	
    ST_LOG("lis2ds12_step_detect_enable_intf OK! en=%d\n", en);
    return 0;
}
#endif

#if (CONFIG_STEP_DETECT || CONFIG_STEP_COUNTER)
static int lis2ds12_step_d_set_delay_intf(u64 ns)
{
    return LIS2DS12_SUCCESS;
}
#endif

static int lis2ds12_step_c_set_delay_intf(u64 ns)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_pedo *pedo_obj = &obj->lis2ds12_pedo_data;

    int value = 0;
    int sample_delay = 0;
    int err = 0;
	
    value = (int)ns/1000/1000;
    if (value <= 5)
        sample_delay = LIS2DS12_REG_CTRL1_ODR_200HZ;
    else if (value <= 10)
        sample_delay = LIS2DS12_REG_CTRL1_ODR_100HZ;
    else
        sample_delay = LIS2DS12_REG_CTRL1_ODR_50HZ;

    pedo_obj->odr = sample_delay;
    if (err != LIS2DS12_SUCCESS) {
        ST_ERR("Set delay parameter error!\n");
    }
	
    ST_LOG("lis2ds12_pedo_set_delay_intf (%d)\n",value);
    return LIS2DS12_SUCCESS;
}

#if (CONFIG_SIGNIFICANT_MOTION)
static int lis2ds12_get_data_significant_intf(uint32_t *value, int *status)
{
#if (!CONFIG_HARDWARE_INTERRUPT)
    step_notify(TYPE_SIGNIFICANT);
#endif
    return LIS2DS12_SUCCESS;
}
#endif

#if (CONFIG_STEP_DETECT || CONFIG_STEP_COUNTER)
static int lis2ds12_get_data_step_d_intf(uint32_t *value, int *status)
{
#if (!CONFIG_HARDWARE_INTERRUPT)
    step_notify(TYPE_STEP_DETECTOR);
#endif
    return LIS2DS12_SUCCESS;
}
#endif

static int lis2ds12_get_data_step_c_intf(uint32_t *value, int *status)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_pedo *pedo_obj = &obj->lis2ds12_pedo_data;
    int ret = 0;

    ret = lis2ds12_pedo_read_data(pedo_obj, value);
    if (ret < 0) {
        ST_ERR("lis2ds12_pedo_read_data error\n");
        return ret;
    }
	
    *status = SENSOR_STATUS_ACCURACY_HIGH;

    return LIS2DS12_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int lis2ds12_pedo_local_init(void)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_pedo *pedo_obj = &obj->lis2ds12_pedo_data;
    int ret = 0;
    int retry = 0;
    struct step_c_control_path ctl= {0};
    struct step_c_data_path data = {0};

    ST_FUN();

    atomic_set(&pedo_obj->trace, 0);
    atomic_set(&pedo_obj->suspend, 0);
    
    if ((ret = lis2ds12_pedo_create_attr(&(lis2ds12_pedo_init_info.platform_diver_addr->driver)))) {
        ST_ERR("create attribute err = %d\n", ret);
        goto exit_create_attr_failed;
    }
	
    for (retry = 0; retry < 3; retry++) {
       ret = lis2ds12_pedo_init(pedo_obj, 1);
       if (ret == 0) 
           break;
       else
           ST_ERR("lis2ds12_pedo_device init cilent fail time: %d\n", retry);	
    }

    if (ret != 0)
        goto exit;

    sprintf(pedo_obj->name, "%s_PEDO", obj->name);

    ctl.open_report_data       = lis2ds12_step_counter_open_report_data_intf;
    ctl.enable_nodata 	       = lis2ds12_step_counter_enable_nodata_intf;
    ctl.enable_step_detect     = lis2ds12_enable_step_dectect_intf;
#if (CONFIG_SIGNIFICANT_MOTION)
    ctl.enable_significant     = lis2ds12_enable_sigificant_intf;
#endif
    ctl.step_c_set_delay       = lis2ds12_step_c_set_delay_intf;
    ctl.step_d_set_delay       = lis2ds12_step_d_set_delay_intf;
    ctl.is_report_input_direct = false;
    ctl.is_support_batch       = false;

    ret = step_c_register_control_path(&ctl);
    if (ret) {
        ST_ERR("register acc control path err\n");
        goto exit;
    }

    data.get_data             = lis2ds12_get_data_step_c_intf;
    data.vender_div           = 1;
    data.get_data_step_d      = lis2ds12_get_data_step_d_intf;
#if (CONFIG_SIGNIFICANT_MOTION)
    data.get_data_significant = lis2ds12_get_data_significant_intf;
#endif
	
    ret = step_c_register_data_path(&data);
    if (ret) {
        ST_ERR("register acc data path err\n");
        goto exit;
    }

    ST_LOG("%s: OK\n", __func__);    
    return LIS2DS12_SUCCESS;

exit:
    ST_ERR("%s: err = %d\n", __func__, ret);
    lis2ds12_pedo_delete_attr(&(lis2ds12_pedo_init_info.platform_diver_addr->driver));	
exit_create_attr_failed:       
    return -1;
}

static int lis2ds12_pedo_local_remove(void)
{
    ST_FUN(); 
    lis2ds12_pedo_delete_attr(&(lis2ds12_pedo_init_info.platform_diver_addr->driver));
    
    return LIS2DS12_SUCCESS;
}

struct step_c_init_info lis2ds12_pedo_init_info = {
    .name = "lis2ds12_step",
    .init = lis2ds12_pedo_local_init,
    .uninit = lis2ds12_pedo_local_remove,
};
#endif

