/* LIS2DS12 AXL driver
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
 
#include "lis2ds12.h"

static struct data_resolution lis2ds12_acc_data_resolution[] = {
     /* combination by {FULL_RES,RANGE}*/
    {{ 0, 0}, 61},     //2g   1LSB=61 ug
    {{ 0, 0}, 488},    //16g
    {{ 0, 0}, 122},    //4g
    {{ 0, 0}, 244},    //8g
};

static struct data_resolution lis2ds12_offset_resolution = {{15, 6}, 64};
static struct GSENSOR_VECTOR3D gsensor_gain, gsensor_offset;
struct acc_hw lis2ds12_acc_cust_hw;

/*For driver get cust info*/
struct acc_hw *lis2ds12_get_cust_acc_hw(void)
{
    return &lis2ds12_acc_cust_hw;
}

static int lis2ds12_acc_set_resolution(struct lis2ds12_acc *acc_obj)
{
    struct lis2ds12_data *obj = container_of(acc_obj, struct lis2ds12_data, lis2ds12_acc_data);
    struct i2c_client *client = obj->client;
    int ret;
    u8 dat, reso;

    ret = lis2ds12_i2c_read_block(client, LIS2DS12_REG_CTRL1, &dat, 0x01);
    if (ret < 0) {
        ST_ERR("write data format fail!!\n");
        return ret;
    }

    /*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
    reso = (dat & LIS2DS12_REG_CTRL1_MASK_FS) >> 2;
    if (reso >= 0x3)
        reso = 0x3;
   
    ST_ERR("LIS2DS12_REG_CTRL1:0x%02x, reso:%d\n", dat, reso);

    if (reso < sizeof(lis2ds12_acc_data_resolution)/sizeof(lis2ds12_acc_data_resolution[0])) {        
        acc_obj->reso = &lis2ds12_acc_data_resolution[reso];
        return LIS2DS12_SUCCESS;
    } else {
        return -EINVAL;
    }
}

static int lis2ds12_acc_read_rawdata(struct lis2ds12_acc *acc_obj, s16 data[LIS2DS12_AXES_NUM])
{
    struct lis2ds12_data *obj = container_of(acc_obj, struct lis2ds12_data, lis2ds12_acc_data);
    struct i2c_client *client = obj->client;
    u8 buf[LIS2DS12_DATA_LEN] = {0};
    int ret = 0;

    if (NULL == client) {
        ret = -EINVAL;
    } else {
        if ((lis2ds12_i2c_read_block(client, LIS2DS12_REG_OUT_X_L, buf, LIS2DS12_DATA_LEN)) < 0) {
	    ST_ERR("read G-sensor data register err!\n");
            return -1;
        }
	
	data[LIS2DS12_AXIS_X] = ((s16)(((s16)(buf[LIS2DS12_AXIS_X*2+1] << 8)) | buf[LIS2DS12_AXIS_X*2]));
	data[LIS2DS12_AXIS_Y] = ((s16)(((s16)(buf[LIS2DS12_AXIS_Y*2+1] << 8)) | buf[LIS2DS12_AXIS_Y*2]));
	data[LIS2DS12_AXIS_Z] = ((s16)(((s16)(buf[LIS2DS12_AXIS_Z*2+1] << 8)) | buf[LIS2DS12_AXIS_Z*2]));
        
	if (atomic_read(&acc_obj->trace) & ADX_TRC_RAWDATA) {
            ST_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[LIS2DS12_AXIS_X], data[LIS2DS12_AXIS_Y], data[LIS2DS12_AXIS_Z],
                    data[LIS2DS12_AXIS_X], data[LIS2DS12_AXIS_Y], data[LIS2DS12_AXIS_Z]);
        }

        if (atomic_read(&acc_obj->trace) & ADX_TRC_RAWDATA) {
            ST_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[LIS2DS12_AXIS_X], data[LIS2DS12_AXIS_Y], data[LIS2DS12_AXIS_Z],
                    data[LIS2DS12_AXIS_X], data[LIS2DS12_AXIS_Y], data[LIS2DS12_AXIS_Z]);
        }
        
#ifdef CONFIG_LIS2DS12_LOWPASS
        if (atomic_read(&acc_obj->filter)) {
            if (atomic_read(&acc_obj->fir_en) && !atomic_read(&acc_obj->suspend)) {
                int idx, firlen = atomic_read(&acc_obj->firlen);   
                if (acc_obj->fir.num < firlen) {                
                    acc_obj->fir.raw[acc_obj->fir.num][LIS2DS12_AXIS_X] = data[LIS2DS12_AXIS_X];
                    acc_obj->fir.raw[acc_obj->fir.num][LIS2DS12_AXIS_Y] = data[LIS2DS12_AXIS_Y];
                    acc_obj->fir.raw[acc_obj->fir.num][LIS2DS12_AXIS_Z] = data[LIS2DS12_AXIS_Z];
                    acc_obj->fir.sum[LIS2DS12_AXIS_X] += data[LIS2DS12_AXIS_X];
                    acc_obj->fir.sum[LIS2DS12_AXIS_Y] += data[LIS2DS12_AXIS_Y];
                    acc_obj->fir.sum[LIS2DS12_AXIS_Z] += data[LIS2DS12_AXIS_Z];
                    if (atomic_read(&acc_obj->trace) & ADX_TRC_FILTER) {
                        ST_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", acc_obj->fir.num, 
				acc_obj->fir.raw[acc_obj->fir.num][LIS2DS12_AXIS_X], 
				acc_obj->fir.raw[acc_obj->fir.num][LIS2DS12_AXIS_Y], 
				acc_obj->fir.raw[acc_obj->fir.num][LIS2DS12_AXIS_Z],
                            	acc_obj->fir.sum[LIS2DS12_AXIS_X], acc_obj->fir.sum[LIS2DS12_AXIS_Y], 
				acc_obj->fir.sum[LIS2DS12_AXIS_Z]);
                    }
                    acc_obj->fir.num++;
                    acc_obj->fir.idx++;
                } else {
                    idx = acc_obj->fir.idx % firlen;
                    acc_obj->fir.sum[LIS2DS12_AXIS_X] -= acc_obj->fir.raw[idx][LIS2DS12_AXIS_X];
                    acc_obj->fir.sum[LIS2DS12_AXIS_Y] -= acc_obj->fir.raw[idx][LIS2DS12_AXIS_Y];
                    acc_obj->fir.sum[LIS2DS12_AXIS_Z] -= acc_obj->fir.raw[idx][LIS2DS12_AXIS_Z];
                    acc_obj->fir.raw[idx][LIS2DS12_AXIS_X] = data[LIS2DS12_AXIS_X];
                    acc_obj->fir.raw[idx][LIS2DS12_AXIS_Y] = data[LIS2DS12_AXIS_Y];
                    acc_obj->fir.raw[idx][LIS2DS12_AXIS_Z] = data[LIS2DS12_AXIS_Z];
                    acc_obj->fir.sum[LIS2DS12_AXIS_X] += data[LIS2DS12_AXIS_X];
                    acc_obj->fir.sum[LIS2DS12_AXIS_Y] += data[LIS2DS12_AXIS_Y];
                    acc_obj->fir.sum[LIS2DS12_AXIS_Z] += data[LIS2DS12_AXIS_Z];
                    acc_obj->fir.idx++;
                    data[LIS2DS12_AXIS_X] = acc_obj->fir.sum[LIS2DS12_AXIS_X]/firlen;
                    data[LIS2DS12_AXIS_Y] = acc_obj->fir.sum[LIS2DS12_AXIS_Y]/firlen;
                    data[LIS2DS12_AXIS_Z] = acc_obj->fir.sum[LIS2DS12_AXIS_Z]/firlen;
                    if (atomic_read(&acc_obj->trace) & ADX_TRC_FILTER) {
                        ST_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
                        	acc_obj->fir.raw[idx][LIS2DS12_AXIS_X], acc_obj->fir.raw[idx][LIS2DS12_AXIS_Y], 
				acc_obj->fir.raw[idx][LIS2DS12_AXIS_Z], acc_obj->fir.sum[LIS2DS12_AXIS_X], 
				acc_obj->fir.sum[LIS2DS12_AXIS_Y], acc_obj->fir.sum[LIS2DS12_AXIS_Z],
                        	data[LIS2DS12_AXIS_X], data[LIS2DS12_AXIS_Y], data[LIS2DS12_AXIS_Z]);
                    }
                }
            }
        }    
#endif
    }
	
    return ret;
}

static int lis2ds12_acc_reset_calibration(struct lis2ds12_acc *acc_obj)
{
    memset(acc_obj->cali_sw, 0x00, sizeof(acc_obj->cali_sw));
    return LIS2DS12_SUCCESS;     
}

static int lis2ds12_acc_read_calibration(struct lis2ds12_acc *acc_obj, int dat[LIS2DS12_AXES_NUM])
{
    dat[acc_obj->cvt.map[LIS2DS12_AXIS_X]] = acc_obj->cvt.sign[LIS2DS12_AXIS_X]*acc_obj->cali_sw[LIS2DS12_AXIS_X];
    dat[acc_obj->cvt.map[LIS2DS12_AXIS_Y]] = acc_obj->cvt.sign[LIS2DS12_AXIS_Y]*acc_obj->cali_sw[LIS2DS12_AXIS_Y];
    dat[acc_obj->cvt.map[LIS2DS12_AXIS_Z]] = acc_obj->cvt.sign[LIS2DS12_AXIS_Z]*acc_obj->cali_sw[LIS2DS12_AXIS_Z];            
                                       
    return LIS2DS12_SUCCESS;
}

static int lis2ds12_acc_write_calibration(struct lis2ds12_acc *acc_obj, int dat[LIS2DS12_AXES_NUM])
{
    ST_FUN();
	
    if (!acc_obj || !dat) {
        ST_ERR("null ptr!!\n");
        return -EINVAL;
    } else {        
        acc_obj->cali_sw[LIS2DS12_AXIS_X] = acc_obj->cvt.sign[LIS2DS12_AXIS_X]*dat[acc_obj->cvt.map[LIS2DS12_AXIS_X]];
        acc_obj->cali_sw[LIS2DS12_AXIS_Y] = acc_obj->cvt.sign[LIS2DS12_AXIS_Y]*dat[acc_obj->cvt.map[LIS2DS12_AXIS_Y]];
        acc_obj->cali_sw[LIS2DS12_AXIS_Z] = acc_obj->cvt.sign[LIS2DS12_AXIS_Z]*dat[acc_obj->cvt.map[LIS2DS12_AXIS_Z]];
    } 
    
    return 0;
}

static int lis2ds12_acc_set_full_scale(struct lis2ds12_acc *acc_obj, u8 dataformat)
{
    struct lis2ds12_data *obj = container_of(acc_obj, struct lis2ds12_data, lis2ds12_acc_data);
    struct i2c_client *client = obj->client;
    int ret = 0;

    ret = lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_CTRL1, LIS2DS12_REG_CTRL1_MASK_FS, dataformat);
    if (ret < 0) {
        ST_ERR("read LIS2DS12_REG_CTRL1 register err!\n");
        return LIS2DS12_ERR_I2C;
    }

    return lis2ds12_acc_set_resolution(acc_obj);
}

static int lis2ds12_acc_set_odr(struct lis2ds12_acc *acc_obj, u8 odr)
{
    struct lis2ds12_data *obj = container_of(acc_obj, struct lis2ds12_data, lis2ds12_acc_data);
    struct i2c_client *client = obj->client;
    int ret = 0;

#if (ST_SENSOR_TILT || ST_SENSOR_STEP_DETECT || ST_SENSOR_STEP_COUNTER || ST_SENSOR_SIGNIFICANT_MOTION)
    if (odr < LIS2DS12_REG_CTRL1_ODR_25HZ) {
	if (CONFIG_PEDOMETER_ALWAYS_ON || obj->step_c_enabled || 
            obj->step_d_enabled || obj->significant_enabled || obj->tilt_enabled)
	    odr = LIS2DS12_REG_CTRL1_ODR_25HZ;
	}
#endif
   
    ret = lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_CTRL1, LIS2DS12_REG_CTRL1_MASK_ODR, odr);
    if (ret < 0)
        return LIS2DS12_ERR_I2C;
    
    return LIS2DS12_SUCCESS;
}

int lis2ds12_acc_set_power_mode(struct lis2ds12_acc *acc_obj, bool state)
{ 
    int ret = 0;
	
    if (state == acc_obj->lis2ds12_acc_power) {
        ST_LOG("Sensor power status is newest!\n");
        return LIS2DS12_SUCCESS;
    }

    if (state == true) {
	if (acc_obj->odr == 0)
	    acc_obj->odr = LIS2DS12_REG_CTRL1_ODR_100HZ;
		
	ret = lis2ds12_acc_set_odr(acc_obj, acc_obj->odr);
    } else if (state == false) {
	ret = lis2ds12_acc_set_odr(acc_obj, LIS2DS12_REG_CTRL1_ODR_0HZ);
    } else {
	ST_ERR("set power state error!\n");
	return LIS2DS12_ERR_SETUP_FAILURE;
    }
	
    if (ret < 0) {
        ST_ERR("set power mode failed!\n");
        return LIS2DS12_ERR_I2C;
    } else if (atomic_read(&acc_obj->trace) & ADX_TRC_INFO) {
        ST_LOG("set power mode ok %d!\n", state);
    }
	
    acc_obj->lis2ds12_acc_power = state;
    return LIS2DS12_SUCCESS;
}

int lis2ds12_acc_init(struct lis2ds12_acc *acc_obj, int reset_cali)
{
    struct lis2ds12_data *obj = container_of(acc_obj, struct lis2ds12_data, lis2ds12_acc_data);
    struct i2c_client *client = obj->client;
    int ret = 0;
    u8 buf[2] = {0, 0};
	
    ST_FUN();
  
	//set LIS2DS12_REG_CTRL1 to 0
    buf[0] = 0x00;
    ret = lis2ds12_i2c_write_block(client, LIS2DS12_REG_CTRL1, buf, 0x01);
    if (ret < 0) {
        ST_ERR("LIS2DS12_REG_CTRL1 step 1!\n");
        return ret;
    }
    
	//power down, set ODR as 0Hz
    acc_obj->odr = 0;
    ret = lis2ds12_acc_set_odr(acc_obj, LIS2DS12_REG_CTRL1_ODR_0HZ);
    if (ret < 0) {
        ST_ERR("lis2ds12_acc_init step 2!\n");
        return ret;
    }

	//set FS as 4g, pedometer better work at 4g
    ret = lis2ds12_acc_set_full_scale(acc_obj, LIS2DS12_REG_CTRL1_FS_4G);
    if (ret < 0) {
        ST_ERR("lis2ds12_acc_init step 3!\n");
        return ret;
    }
	
    gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = acc_obj->reso->sensitivity;

    if (0 != reset_cali) { 
        //reset calibration only in power on
        ret = lis2ds12_acc_reset_calibration(acc_obj);
        if (ret < 0)
            return ret;
    }

#ifdef CONFIG_LIS2DS12_LOWPASS
    memset(&acc_obj->fir, 0x00, sizeof(acc_obj->fir));
#endif
    return LIS2DS12_SUCCESS;
}

static int lis2ds12_acc_read_chip_name(struct lis2ds12_acc *acc_obj, u8 *buf, int bufsize)
{
    sprintf(buf, "%s", acc_obj->name);
    return LIS2DS12_SUCCESS;
}

static int lis2ds12_acc_read_data(struct lis2ds12_acc *acc_obj, u8 *data, int bufsize)
{
    struct lis2ds12_data *obj = container_of(acc_obj, struct lis2ds12_data, lis2ds12_acc_data);
    struct i2c_client *client = obj->client;
    int acc[LIS2DS12_AXES_NUM];
    int ret = 0;

    if (NULL == data)
        return LIS2DS12_ERR_SETUP_FAILURE;
	
    if (NULL == client) {
        *data = 0;
        return LIS2DS12_ERR_SETUP_FAILURE;
    }

    if (atomic_read(&acc_obj->suspend)) {
        ST_LOG("sensor in suspend read not data!\n");
        return LIS2DS12_SUCCESS;
    }
	
    if ((ret = lis2ds12_acc_read_rawdata(acc_obj, acc_obj->data))) {        
        ST_ERR("I2C error: ret value=%d", ret);
        return ret;
    } else {
        acc_obj->data[LIS2DS12_AXIS_X] = (acc_obj->data[LIS2DS12_AXIS_X]*acc_obj->reso->sensitivity/1000)*GRAVITY_EARTH_1000/1000;
       	acc_obj->data[LIS2DS12_AXIS_Y] = (acc_obj->data[LIS2DS12_AXIS_Y]*acc_obj->reso->sensitivity/1000)*GRAVITY_EARTH_1000/1000;
        acc_obj->data[LIS2DS12_AXIS_Z] = (acc_obj->data[LIS2DS12_AXIS_Z]*acc_obj->reso->sensitivity/1000)*GRAVITY_EARTH_1000/1000;
        
	acc_obj->data[LIS2DS12_AXIS_X] += acc_obj->cali_sw[LIS2DS12_AXIS_X];
        acc_obj->data[LIS2DS12_AXIS_Y] += acc_obj->cali_sw[LIS2DS12_AXIS_Y];
        acc_obj->data[LIS2DS12_AXIS_Z] += acc_obj->cali_sw[LIS2DS12_AXIS_Z];
        
        /*remap coordinate*/
        acc[acc_obj->cvt.map[LIS2DS12_AXIS_X]] = acc_obj->cvt.sign[LIS2DS12_AXIS_X]*acc_obj->data[LIS2DS12_AXIS_X];
        acc[acc_obj->cvt.map[LIS2DS12_AXIS_Y]] = acc_obj->cvt.sign[LIS2DS12_AXIS_Y]*acc_obj->data[LIS2DS12_AXIS_Y];
        acc[acc_obj->cvt.map[LIS2DS12_AXIS_Z]] = acc_obj->cvt.sign[LIS2DS12_AXIS_Z]*acc_obj->data[LIS2DS12_AXIS_Z];
   
	sprintf(data, "%04x %04x %04x", acc[LIS2DS12_AXIS_X], acc[LIS2DS12_AXIS_Y], acc[LIS2DS12_AXIS_Z]);
        if (atomic_read(&acc_obj->trace) & ADX_TRC_IOCTL) {
            ST_LOG("gsensor data: %s!\n", data);
            dumpReg(obj);
        }
    }
    
    return LIS2DS12_SUCCESS;
}

static int lis2ds12_acc_read_rawdata_string(struct lis2ds12_acc *acc_obj, u8 *buf)
{
    struct lis2ds12_data *obj = container_of(acc_obj, struct lis2ds12_data, lis2ds12_acc_data);
    struct i2c_client *client = obj->client;
    int ret = 0;

    if (!buf || !client)
        return LIS2DS12_ERR_SETUP_FAILURE;
    
    if ((ret = lis2ds12_acc_read_rawdata(acc_obj, acc_obj->data))) {        
        ST_ERR("I2C error: ret value=%d", ret);
        return ret;
    } else {
        sprintf(buf, "%04x %04x %04x", acc_obj->data[LIS2DS12_AXIS_X], 
        acc_obj->data[LIS2DS12_AXIS_Y], acc_obj->data[LIS2DS12_AXIS_Z]);
    }
    
    return LIS2DS12_SUCCESS;
}

static ssize_t lis2ds12_attr_acc_show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;
    u8 strbuf[LIS2DS12_BUFSIZE];
	
    if (NULL == obj->client) {
        ST_ERR("i2c client is null!!\n");
        return LIS2DS12_SUCCESS;
    }
    
    lis2ds12_acc_read_chip_name(acc_obj, strbuf, LIS2DS12_BUFSIZE);
	
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t lis2ds12_attr_acc_show_chipid_value(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
	
    if (NULL == obj->client) {
        ST_ERR("i2c client is null!!\n");
        return LIS2DS12_SUCCESS;
    }

    return snprintf(buf, PAGE_SIZE, "0x%x\n", obj->chip_id);
}

static ssize_t lis2ds12_attr_acc_show_sensordata_value(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;
    u8 databuf[LIS2DS12_BUFSIZE];
    
    if (NULL == obj->client) {
        ST_ERR("i2c client is null!!\n");
        return LIS2DS12_SUCCESS;
    }
	
    lis2ds12_acc_read_data(acc_obj, databuf, LIS2DS12_BUFSIZE);
    
    return snprintf(buf, PAGE_SIZE, "%s\n", databuf);
}

static ssize_t lis2ds12_attr_acc_show_rawdata_value(struct device_driver *ddri, char *buf)
{   
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;
    u8 databuf[LIS2DS12_BUFSIZE];
    
    if (NULL == obj->client) {
        ST_ERR("i2c client is null!!\n");
        return LIS2DS12_SUCCESS;
    }

    lis2ds12_acc_read_rawdata_string(acc_obj, databuf);
	
    return snprintf(buf, PAGE_SIZE, "%s\n", databuf);
}

static ssize_t lis2ds12_attr_acc_show_cali_value(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data; 
    int ret, len = 0, mul;
    int tmp[LIS2DS12_AXES_NUM];    

    if (NULL == obj->client) {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }

    if ((ret = lis2ds12_acc_read_calibration(acc_obj, tmp))) {
        return -EINVAL;
    } else {    
        mul = acc_obj->reso->sensitivity/lis2ds12_offset_resolution.sensitivity;
        len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", 
		        mul, acc_obj->offset[LIS2DS12_AXIS_X], acc_obj->offset[LIS2DS12_AXIS_Y], acc_obj->offset[LIS2DS12_AXIS_Z],
			acc_obj->offset[LIS2DS12_AXIS_X], acc_obj->offset[LIS2DS12_AXIS_Y], acc_obj->offset[LIS2DS12_AXIS_Z]);
						
        len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			acc_obj->cali_sw[LIS2DS12_AXIS_X], acc_obj->cali_sw[LIS2DS12_AXIS_Y], acc_obj->cali_sw[LIS2DS12_AXIS_Z]);

        len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			acc_obj->offset[LIS2DS12_AXIS_X]*mul + acc_obj->cali_sw[LIS2DS12_AXIS_X],
			acc_obj->offset[LIS2DS12_AXIS_Y]*mul + acc_obj->cali_sw[LIS2DS12_AXIS_Y],
			acc_obj->offset[LIS2DS12_AXIS_Z]*mul + acc_obj->cali_sw[LIS2DS12_AXIS_Z],
			tmp[LIS2DS12_AXIS_X], tmp[LIS2DS12_AXIS_Y], tmp[LIS2DS12_AXIS_Z]);
        
        return len;
    }
}

static ssize_t lis2ds12_attr_acc_store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{ 
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data; 
    int ret, x, y, z;
    int dat[LIS2DS12_AXES_NUM];

    if (!strncmp(buf, "rst", 3)) {
        if ((ret = lis2ds12_acc_reset_calibration(acc_obj)))
            ST_ERR("reset offset err = %d\n", ret);
    } else if (3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z)) {
        dat[LIS2DS12_AXIS_X] = x;
        dat[LIS2DS12_AXIS_Y] = y;
        dat[LIS2DS12_AXIS_Z] = z;
        if ((ret = lis2ds12_acc_write_calibration(acc_obj, dat))) {
            ST_ERR("write calibration err = %d\n", ret);
        }        
    } else {
        ST_ERR("invalid format\n");
    }
    
    return count;
}

static ssize_t lis2ds12_attr_acc_show_power_status(struct device_driver *ddri, char *buf)
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

static ssize_t lis2ds12_attr_acc_show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_LIS2DS12_LOWPASS
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data; 
    
    if (atomic_read(&acc_obj->firlen)) {
    	int idx, len = atomic_read(&acc_obj->firlen);
        ST_LOG("len = %2d, idx = %2d\n", acc_obj->fir.num, acc_obj->fir.idx);

        for (idx = 0; idx < len; idx++)
            ST_LOG("[%5d %5d %5d]\n", acc_obj->fir.raw[idx][LIS2DS12_AXIS_X], acc_obj->fir.raw[idx][LIS2DS12_AXIS_Y], 
		    acc_obj->fir.raw[idx][LIS2DS12_AXIS_Z]);
        
        ST_LOG("sum = [%5d %5d %5d]\n", acc_obj->fir.sum[LIS2DS12_AXIS_X], acc_obj->fir.sum[LIS2DS12_AXIS_Y], 
		acc_obj->fir.sum[LIS2DS12_AXIS_Z]);
        ST_LOG("avg = [%5d %5d %5d]\n", acc_obj->fir.sum[LIS2DS12_AXIS_X]/len, acc_obj->fir.sum[LIS2DS12_AXIS_Y]/len, 
		acc_obj->fir.sum[LIS2DS12_AXIS_Z]/len);
    }

    return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&acc_obj->firlen));
#else
    return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}

static ssize_t lis2ds12_attr_acc_store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_LIS2DS12_LOWPASS
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data; 
    int firlen;

    if (1 != sscanf(buf, "%d", &firlen)) {
        ST_ERR("invallid format\n");
    } else if (firlen > C_MAX_FIR_LENGTH) {
        ST_ERR("exceeds maximum filter length\n");
    } else { 
        atomic_set(&acc_obj->firlen, firlen);
        if (0 == firlen) {
            atomic_set(&acc_obj->fir_en, 0);
        } else {
            memset(&acc_obj->fir, 0x00, sizeof(acc_obj->fir));
            atomic_set(&acc_obj->fir_en, 1);
        }
    }
#endif    
    return count;
}

static ssize_t lis2ds12_attr_acc_show_trace_value(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data; 
    ssize_t ret;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    ret = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&acc_obj->trace));     
    return ret;    
}

static ssize_t lis2ds12_attr_acc_store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data; 
    int trace;
	
    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if (1 == sscanf(buf, "0x%x", &trace))
        atomic_set(&acc_obj->trace, trace);
    else
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    
    return count;    
}

static ssize_t lis2ds12_attr_acc_show_status_value(struct device_driver *ddri, char *buf)
{    
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data; 
    ssize_t len = 0;
    
    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }    
    
    if (acc_obj->lis2ds12_acc_hw) {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: i2c_num=%d, direction=%d, sensitivity = %d,(power_id=%d, power_vol=%d)\n", 
			acc_obj->lis2ds12_acc_hw->i2c_num, acc_obj->lis2ds12_acc_hw->direction, acc_obj->reso->sensitivity, 
			acc_obj->lis2ds12_acc_hw->power_id, acc_obj->lis2ds12_acc_hw->power_vol);   
        dumpReg(obj);
    } else {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    }
	
    return len;
}

static ssize_t lis2ds12_attr_acc_show_chipinit_value(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;
    ssize_t ret;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    ret = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&acc_obj->trace)); 
    return ret;
}

static ssize_t lis2ds12_attr_acc_store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return count;
    }

    lis2ds12_acc_init(acc_obj, 0);

    return count;
}

static ssize_t lis2ds12_attr_acc_show_layout_value(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;

    if (NULL == obj) {
        ST_LOG("lis2ds12_acc is null!!\n");
        return -1;
    }

    return sprintf(buf, "(%d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
			 acc_obj->lis2ds12_acc_hw->direction, acc_obj->cvt.sign[0], acc_obj->cvt.sign[1],
                   	 acc_obj->cvt.sign[2], acc_obj->cvt.map[0], acc_obj->cvt.map[1], acc_obj->cvt.map[2]);
}

static ssize_t lis2ds12_attr_acc_store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;
    int layout = 0;

    if (NULL == obj) {
        ST_ERR("lis2ds12_acc is null!!\n");
        return count;
    }

    if (1 == sscanf(buf, "%d", &layout)) {
        if (!hwmsen_get_convert(layout, &acc_obj->cvt)) {
            ST_ERR("HWMSEN_GET_CONVERT function error!\r\n");
        } else if (!hwmsen_get_convert(acc_obj->lis2ds12_acc_hw->direction, &acc_obj->cvt)) {
            ST_LOG("invalid layout: %d, restore to %d\n", layout, acc_obj->lis2ds12_acc_hw->direction);
        } else {
            ST_ERR("invalid layout: (%d, %d)\n", layout, acc_obj->lis2ds12_acc_hw->direction);
            hwmsen_get_convert(0, &acc_obj->cvt);
        }
    } else {
        ST_LOG("invalid format = '%s'\n", buf);
    }

    return count;
}

static DRIVER_ATTR(chipinfo,           S_IRUGO, lis2ds12_attr_acc_show_chipinfo_value,   NULL);
static DRIVER_ATTR(chipid,             S_IRUGO, lis2ds12_attr_acc_show_chipid_value,     NULL);
static DRIVER_ATTR(rawdata,            S_IRUGO, lis2ds12_attr_acc_show_rawdata_value,    NULL);
static DRIVER_ATTR(sensordata,         S_IRUGO, lis2ds12_attr_acc_show_sensordata_value, NULL);
static DRIVER_ATTR(cali,     S_IWUSR | S_IRUGO, lis2ds12_attr_acc_show_cali_value,       lis2ds12_attr_acc_store_cali_value);
static DRIVER_ATTR(power,              S_IRUGO, lis2ds12_attr_acc_show_power_status,     NULL);
static DRIVER_ATTR(firlen,   S_IWUSR | S_IRUGO, lis2ds12_attr_acc_show_firlen_value,     lis2ds12_attr_acc_store_firlen_value);
static DRIVER_ATTR(trace,    S_IWUSR | S_IRUGO, lis2ds12_attr_acc_show_trace_value,      lis2ds12_attr_acc_store_trace_value);
static DRIVER_ATTR(status,             S_IRUGO, lis2ds12_attr_acc_show_status_value,     NULL);
static DRIVER_ATTR(chipinit, S_IWUSR | S_IRUGO, lis2ds12_attr_acc_show_chipinit_value,   lis2ds12_attr_acc_store_chipinit_value);
static DRIVER_ATTR(layout,   S_IRUGO | S_IWUSR, lis2ds12_attr_acc_show_layout_value,     lis2ds12_attr_acc_store_layout_value);

static struct driver_attribute *lis2ds12_attr_acc_list[] = {
    &driver_attr_chipinfo,     /*chip information*/
    &driver_attr_chipid,       /*chip id*/
    &driver_attr_sensordata,   /*dump sensor data*/
    &driver_attr_rawdata,      /*dump sensor raw data*/
    &driver_attr_cali,         /*show calibration data*/
    &driver_attr_power,        /*show power reg*/
    &driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
    &driver_attr_trace,        /*trace log*/
    &driver_attr_status,
    &driver_attr_chipinit,
    &driver_attr_layout,
};

int lis2ds12_acc_create_attr(struct device_driver *driver) 
{
    int idx, ret = 0;
    int num = (int)(sizeof(lis2ds12_attr_acc_list)/sizeof(lis2ds12_attr_acc_list[0]));
	
    if (driver == NULL)
        return -EINVAL;

    for (idx = 0; idx < num; idx++) {
        if ((ret = driver_create_file(driver, lis2ds12_attr_acc_list[idx]))) {            
            ST_ERR("driver_create_file (%s) = %d\n", lis2ds12_attr_acc_list[idx]->attr.name, ret);
            break;
        }
    }
	
    return ret;
}

int lis2ds12_acc_delete_attr(struct device_driver *driver)
{
    int idx ,ret = 0;
    int num = (int)(sizeof(lis2ds12_attr_acc_list)/sizeof(lis2ds12_attr_acc_list[0]));

    if (driver == NULL)
        return -EINVAL;
    
    for (idx = 0; idx < num; idx++)
        driver_remove_file(driver, lis2ds12_attr_acc_list[idx]);
   
    return ret;
}

static int lis2ds12_acc_open_report_data_intf(int open)
{
    return LIS2DS12_SUCCESS;
}

//if use this typ of enable, Gsensor only enabled but not report inputEvent to HAL
static int lis2ds12_acc_enable_nodata_intf(int en)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;
    int ret = 0;
    bool power = false;
    
    if (1 == en) {
        power = true;
    } else if (0 == en) {
        power = false;
    }
	
    obj->acc_enabled = en;

    ret = lis2ds12_acc_set_power_mode(acc_obj, power);
    if (ret != LIS2DS12_SUCCESS) {
        ST_ERR("lis2ds12_acc_set_power_mode fail!\n");
        return ret;
    }
	
    return LIS2DS12_SUCCESS;
}

static int lis2ds12_acc_set_delay_intf(u64 ns)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;

    int ms = 0;
    int odr = 0;
    int ret;
    
    ms = (int)ns/1000/1000;
    if (ms <= 5)
        odr = LIS2DS12_REG_CTRL1_ODR_200HZ;
    else if (ms <= 10)
        odr = LIS2DS12_REG_CTRL1_ODR_100HZ;
    else if (ms <= 20)
        odr = LIS2DS12_REG_CTRL1_ODR_50HZ;
    else
        odr = LIS2DS12_REG_CTRL1_ODR_25HZ;
	
    acc_obj->odr = odr;

    ret = lis2ds12_acc_set_odr(acc_obj, acc_obj->odr);
    if (ret != LIS2DS12_SUCCESS)
        ST_ERR("Set delay parameter error!\n");

#ifdef CONFIG_LIS2DS12_LOWPASS
    if (ms >= 50) {
        atomic_set(&acc_obj->filter, 0);
    } else {
        acc_obj->fir.num = 0;
        acc_obj->fir.idx = 0;
        acc_obj->fir.sum[LIS2DS12_AXIS_X] = 0;
        acc_obj->fir.sum[LIS2DS12_AXIS_Y] = 0;
        acc_obj->fir.sum[LIS2DS12_AXIS_Z] = 0;
        atomic_set(&acc_obj->filter, 1);
    }
#endif
    
    ST_LOG("lis2ds12_acc_set_delay_intf (%d)\n", ms);

    return LIS2DS12_SUCCESS;
}

static int lis2ds12_acc_batch_intf(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return lis2ds12_acc_set_delay_intf((u64)samplingPeriodNs);
}

static int lis2ds12_acc_flush_intf(void)
{
	return acc_flush_report();
}

static int lis2ds12_acc_get_data_intf(int *x ,int *y,int *z, int *status)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;
    u8 buff[LIS2DS12_BUFSIZE];
	
    lis2ds12_acc_read_data(acc_obj, buff, LIS2DS12_BUFSIZE);
	
    sscanf(buff, "%x %x %x", x, y, z);        
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

    return LIS2DS12_SUCCESS;
}

static int lis2ds12_acc_open(struct inode *inode, struct file *file)
{
    file->private_data = obj_i2c_data;

    if (file->private_data == NULL) {
        ST_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}

static int lis2ds12_acc_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static long lis2ds12_acc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct lis2ds12_data *obj = (struct lis2ds12_data*)file->private_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;
    struct SENSOR_DATA sensor_data;
    u8 strbuf[LIS2DS12_BUFSIZE];
    void __user *data = (void __user *)arg;
    long ret = 0;
    int cali[3];

    if (_IOC_DIR(cmd) & _IOC_READ)
        ret = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        ret = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

    if (ret) {
        ST_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch (cmd) {
        case GSENSOR_IOCTL_INIT:
            lis2ds12_acc_init(acc_obj, 0);            
            break;
			
        case GSENSOR_IOCTL_READ_CHIPINFO:
            if (data == NULL) {
                ret = -EINVAL;
                break;      
            }
			
            lis2ds12_acc_read_chip_name(acc_obj, strbuf, LIS2DS12_BUFSIZE);
            if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
                ret = -EFAULT;
                break;
            }
               
            break;
			
        case GSENSOR_IOCTL_READ_SENSORDATA:
            if (data == NULL) {
                ret = -EINVAL;
                break;      
            }
            
	    if (acc_obj->lis2ds12_acc_power == false) {
	        lis2ds12_acc_set_power_mode(acc_obj, true);
		//TODO: need turn-on time
	    }

            lis2ds12_acc_read_data(acc_obj, strbuf, LIS2DS12_BUFSIZE);
	    if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
                ret = -EFAULT;
                break;      
            }
			
            break;

	case GSENSOR_IOCTL_READ_GAIN:
            if (data == NULL) {
                ret = -EINVAL;
                break;      
            }
			
            if (copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D))) {
                ret = -EFAULT;
                break;
            }
			
            break;
			
        case GSENSOR_IOCTL_READ_OFFSET:
            if (data == NULL) {
                ret = -EINVAL;
                break;      
            }
            
            if (copy_to_user(data, &gsensor_offset, sizeof(struct GSENSOR_VECTOR3D))) {
                ret = -EFAULT;
                break;
            }
			
            break;
			
        case GSENSOR_IOCTL_READ_RAW_DATA:
            if (data == NULL) {
                ret = -EINVAL;
                break;      
            }
			
            lis2ds12_acc_read_rawdata_string(acc_obj, strbuf);
            
	    if (copy_to_user(data, &strbuf, strlen(strbuf) + 1)) {
                ret = -EFAULT;
                break;      
            }
			
            break;
			
        case GSENSOR_IOCTL_SET_CALI:
            if (data == NULL) {
                ret = -EINVAL;
                break;      
            }
			
            if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
                ret = -EFAULT;
                break;      
            }
			
            if (atomic_read(&acc_obj->suspend)) {
                ST_ERR("Perform calibration in suspend state!!\n");
		ret = -EINVAL;
	    } else {
                cali[LIS2DS12_AXIS_X] = sensor_data.x;
                cali[LIS2DS12_AXIS_Y] = sensor_data.y;
                cali[LIS2DS12_AXIS_Z] = sensor_data.z;
                ret = lis2ds12_acc_write_calibration(acc_obj, cali);             
            }
			
            break;
			
        case GSENSOR_IOCTL_CLR_CALI:
            ret = lis2ds12_acc_reset_calibration(acc_obj);
            break;
			
        case GSENSOR_IOCTL_GET_CALI:
            if (data == NULL) {
                ret = -EINVAL;
                break;      
            }
			
            if ((ret = lis2ds12_acc_read_calibration(acc_obj, cali)))
                break;
            
            sensor_data.x = cali[LIS2DS12_AXIS_X];
            sensor_data.y = cali[LIS2DS12_AXIS_Y];
            sensor_data.z = cali[LIS2DS12_AXIS_Z];
            if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
                ret = -EFAULT;
                break;
            }
			
            break;
			
        default:
            ST_ERR("unknown IOCTL: 0x%08x\n", cmd);
            ret = -ENOIOCTLCMD;
            break;
    }

    return ret;
}

#ifdef CONFIG_COMPAT
static long lis2ds12_acc_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long res = 0;

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {
        case COMPAT_GSENSOR_IOCTL_INIT:
	case COMPAT_GSENSOR_IOCTL_READ_CHIPINFO:
	case COMPAT_GSENSOR_IOCTL_READ_GAIN:
	case COMPAT_GSENSOR_IOCTL_READ_OFFSET:
	case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
	case COMPAT_GSENSOR_IOCTL_READ_RAW_DATA:
	case COMPAT_GSENSOR_IOCTL_SET_CALI:
	case COMPAT_GSENSOR_IOCTL_CLR_CALI:
	case COMPAT_GSENSOR_IOCTL_GET_CALI:
	    res = filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
	    if (res) {
                ST_ERR("unlocked_ioctl: 0x%x failed.", cmd);
                return res;
            }

	    break;

	default:
            ST_ERR("unknown IOCTL: %s 0x%x\n", __func__, cmd);
            res = -ENOIOCTLCMD;
 	    break;
    } 

    return res;
}
#endif

static struct file_operations lis2ds12_acc_fops = {
    .owner 	    = THIS_MODULE,
    .open 	    = lis2ds12_acc_open,
    .release 	    = lis2ds12_acc_release,
    .unlocked_ioctl = lis2ds12_acc_unlocked_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = lis2ds12_acc_compat_ioctl,
#endif
};

static struct miscdevice lis2ds12_acc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gsensor",
    .fops = &lis2ds12_acc_fops,
};

static int lis2ds12_acc_local_init(void)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;
	struct i2c_client *client = obj->client;
    int ret = 0, retry = 0;
    struct acc_control_path ctl = { 0 };
    struct acc_data_path data = { 0 };    
    //const u8 *name = "mediatek,lis2ds12_acc";
	
    ST_FUN();
#if 0	
    acc_obj->lis2ds12_acc_hw = get_accel_dts_func(name, &lis2ds12_acc_cust_hw);
    if (!acc_obj->lis2ds12_acc_hw) {
        ST_ERR("get lis2ds12 dts info failed\n");
    }
#else
    ret = get_accel_dts_func(client->dev.of_node, &lis2ds12_acc_cust_hw);
    if (ret < 0) {
	ST_ERR("get dts info fail\n");
	return -EFAULT;
    }

    acc_obj->lis2ds12_acc_hw = &lis2ds12_acc_cust_hw;
#endif
    if ((ret = hwmsen_get_convert(acc_obj->lis2ds12_acc_hw->direction, &acc_obj->cvt))) {
        ST_ERR("invalid direction: %d\n", acc_obj->lis2ds12_acc_hw->direction);
        goto exit_get_direction_failed;
    }

    atomic_set(&acc_obj->trace, 0);
    atomic_set(&acc_obj->suspend, 0);
	
#ifdef CONFIG_LIS2DS12_LOWPASS
    if (acc_obj->lis2ds12_acc_hw->firlen > C_MAX_FIR_LENGTH)
        atomic_set(&acc_obj->firlen, C_MAX_FIR_LENGTH);
    else
        atomic_set(&acc_obj->firlen, acc_obj->lis2ds12_acc_hw->firlen);
    
    if (atomic_read(&acc_obj->firlen) > 0)
        atomic_set(&acc_obj->fir_en, 1);
#endif

    for (retry = 0; retry < 3; retry++) {
	ret = lis2ds12_acc_init(acc_obj, 1);
        if (!ret) {
            ST_LOG("lis2ds12_acc_init successfully\n");
            break;
        }

        ST_ERR("lis2ds12_acc_device init cilent fail time: %d\n", retry);
    }
	
    if (ret != 0)
        goto exit_init_failed;

    sprintf(acc_obj->name, "%s_ACC", obj->name);
	
    if ((ret = misc_register(&lis2ds12_acc_device))) {
        ST_ERR("lis2ds12_acc_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    if ((ret = lis2ds12_acc_create_attr(&(lis2ds12_acc_init_info.platform_diver_addr->driver)))) {
        ST_ERR("create attribute err = %d\n", ret);
        goto exit_create_attr_failed;
    }
	
    ctl.open_report_data       = lis2ds12_acc_open_report_data_intf;
    ctl.enable_nodata          = lis2ds12_acc_enable_nodata_intf;
    ctl.set_delay              = lis2ds12_acc_set_delay_intf;
	ctl.batch 		   		   = lis2ds12_acc_batch_intf;	
	ctl.flush 		   		   = lis2ds12_acc_flush_intf;
	ctl.is_use_common_factory  = false;
    ctl.is_report_input_direct = false;
	ctl.is_support_batch 	   = acc_obj->lis2ds12_acc_hw->is_batch_supported;
    
    ret = acc_register_control_path(&ctl);
    if (ret) {
        ST_ERR("register acc control path err\n");
        goto exit_register_control_path_failed;
    }

    data.get_data = lis2ds12_acc_get_data_intf;
    data.vender_div = 1000;
    ret = acc_register_data_path(&data);
    if (ret) {
        ST_ERR("register acc data path err\n");
        goto exit_register_data_path_failed;
    }

    ST_LOG("%s: OK\n", __func__);    
    return 0;

exit_register_data_path_failed:
exit_register_control_path_failed:
	lis2ds12_acc_delete_attr(&(lis2ds12_acc_init_info.platform_diver_addr->driver));
exit_create_attr_failed:
    misc_deregister(&lis2ds12_acc_device);
exit_misc_device_register_failed:
exit_init_failed:
exit_get_direction_failed:
    ST_ERR("%s: err = %d\n", __func__, ret);       
    return -1;
}

static int lis2ds12_acc_local_remove(void)
{
    ST_FUN(); 
    misc_deregister(&lis2ds12_acc_device);
    lis2ds12_acc_delete_attr(&(lis2ds12_acc_init_info.platform_diver_addr->driver));
    
    return LIS2DS12_SUCCESS;
}

struct acc_init_info lis2ds12_acc_init_info = {
    .name = "lis2ds12_acc",
    .init = lis2ds12_acc_local_init,
    .uninit = lis2ds12_acc_local_remove,
};

MODULE_DESCRIPTION("STMicroelectronics lis2ds12 driver");
MODULE_AUTHOR("Ian Yang, William Zeng");
MODULE_LICENSE("GPL v2");
