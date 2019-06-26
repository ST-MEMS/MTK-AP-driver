/* LPS22HH driver
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

#include "lps22hh.h"

struct baro_hw lps22hh_cust_hw;
int lps22hh_baro_init_flag = -1; // 0<==>OK -1 <==> fail

/*For driver get cust info*/
struct baro_hw *lps22hh_get_cust_baro_hw(void)
{
    return &lps22hh_cust_hw;
}

static int lps22hh_baro_read_rawdata(struct lps22hh_baro *baro_obj, s32 *data)
{
    struct lps22hh_data *obj = container_of(baro_obj, struct lps22hh_data, lps22hh_baro_data);
	struct i2c_client *client = obj->client;
    s32 baro_data = 0;
    u8 buf[LPS22HH_DATA_LEN] = {0};
    int err = 0;

    if (NULL == client)
    {
        err = -EINVAL;
    }
    else
    {
        if ((lps22hh_i2c_read_block(client, LPS22HH_REG_PRESS_OUT_XL, buf, 3)) < 0)
        {
            ST_ERR("read barometer sensor data register err!\n");
            return -1;
        }

	baro_data = (s32)((((s32) buf[2]) << 16) | (buf[1] << 8) | (buf[0]));
//ST_LOG("buf: 0x%02x, 0x%02x, 0x%02x, data: %d\n", buf[2], buf[1], buf[0], baro_data);
        
        if (atomic_read(&baro_obj->trace) & ADX_TRC_RAWDATA)
        {
            ST_LOG("[%08X] => [%5d]\n", baro_data, baro_data);
        }
        
#ifdef CONFIG_LPS22HH_LOWPASS	
	if (atomic_read(&baro_obj->filter)) {
		if (atomic_read(&baro_obj->fir_en) && !atomic_read(&baro_obj->suspend)) {
			int idx, firlen = atomic_read(&baro_obj->firlen);
			if (baro_obj->fir.num < firlen) {
				baro_obj->fir.raw[baro_obj->fir.num] = baro_data;
				baro_obj->fir.sum += baro_data;
				if (atomic_read(&baro_obj->trace) & ADX_TRC_FILTER) {
					ST_LOG("add [%2d] [%5d] => [%5d]\n", baro_obj->fir.num,
							baro_obj->fir.raw[baro_obj->fir.num],
							baro_obj->fir.sum);
				}
				baro_obj->fir.num++;
				baro_obj->fir.idx++;
			} else {
				idx = baro_obj->fir.idx % firlen;
				baro_obj->fir.sum -= baro_obj->fir.raw[idx];
				baro_obj->fir.raw[idx]= baro_data;
				baro_obj->fir.sum += baro_data;
				baro_obj->fir.idx++;
				baro_data = baro_obj->fir.sum/firlen;
				if (atomic_read(&baro_obj->trace) & ADX_TRC_FILTER) {
					ST_LOG("add [%2d][%5d]=>[%5d]:[%5d]\n", idx, baro_obj->fir.raw[idx],
							baro_obj->fir.sum, baro_data);
				}
			}
		}
	}
#endif
        *data = baro_data;
    }
	
    return err;
}

#ifdef MISC_DEVICE_FACTORY
static int lps25h_baro_read_rawdata_temperature(struct lps22hh_baro *baro_obj, s32 *data)
{
    struct lps22hh_data *obj = container_of(baro_obj, struct lps22hh_data, lps22hh_baro_data);
	struct i2c_client *client = obj->client;
    u8 buf[LPS22HH_DATA_LEN] = {0};
    int res = 0;

	if (NULL == client) 
    {
		ST_ERR("i2c client is null!\n");
		return LPS22HH_ERR_I2C;
	}

    if ((lps22hh_i2c_read_block(client, LPS22HH_REG_TEMP_OUT_L, buf, 0x01))<0)
    {
        ST_ERR("read temperature sensor data register err!\n");
        return LPS22HH_ERR_I2C;
    }
        
    if ((lps22hh_i2c_read_block(client, LPS22HH_REG_PRESS_OUT_H, buf+1, 0x01))<0)
    {
        ST_ERR("read temperature sensor data register err!\n");
        return LPS22HH_ERR_I2C;
    }
	
	*data = (s16) ((((s8) buf[1]) << 8) | (buf[0]));

	return res;
}
#endif

static int lps22hh_baro_set_odr(struct lps22hh_baro *baro_obj, u8 odr)
{
    struct lps22hh_data *obj = container_of(baro_obj, struct lps22hh_data, lps22hh_baro_data);
	struct i2c_client *client = obj->client;
    u8 databuf[10];
    u8 addr = LPS22HH_REG_CTRL_REG1;
    int res = 0;

    memset(databuf, 0, sizeof(u8)*10);
    
    if ((lps22hh_i2c_read_block(client, addr, databuf, 0x01))<0)
    {
        ST_ERR("read reg_ctl_reg1 register err!\n");
        return LPS22HH_ERR_I2C;
    }

    databuf[0] &= ~0x70;
    databuf[0] |= odr;

    res = lps22hh_i2c_write_block(client, addr, databuf, 0x1);
    if (res < 0)
    {
		ST_ERR("write reg_ctl_reg1 register err!\n");
        return LPS22HH_ERR_I2C;
    }
    		
    return LPS22HH_SUCCESS;    
}

int lps22hh_baro_set_power_mode(struct lps22hh_baro *baro_obj, bool state)
{
    u8 databuf[2];    
    int res = 0;

    if (state == baro_obj->lps22hh_baro_power)
    {
        ST_LOG("Sensor power status is newest!\n");
        return LPS22HH_SUCCESS;
    }

    if (state == true)
    {
		res = lps22hh_baro_set_odr(baro_obj, baro_obj->odr);
    }
    else if (state == false)
    {
		res = lps22hh_baro_set_odr(baro_obj, LPS22HH_ODR_0HZ);
    }
	
    if (res < 0)
    {
        ST_LOG("set power mode failed!\n");
        return LPS22HH_ERR_I2C;
    }
    else if (atomic_read(&baro_obj->trace) & ADX_TRC_INFO)
    {
        ST_LOG("set power mode ok %d!\n", databuf[1]);
    }
	
	baro_obj->lps22hh_baro_power = state;
    return LPS22HH_SUCCESS;    
}

int lps22hh_baro_init(struct lps22hh_baro *baro_obj, int reset_cali)
{
#ifdef CONFIG_LPS22HH_BARO_DRY
    struct lps22hh_data *obj = container_of(baro_obj, struct lps22hh_data, lps22hh_baro_data);
#endif
    int res = 0;

    baro_obj->odr = 0;
    res = lps22hh_baro_set_odr(baro_obj, LPS22HH_ODR_0HZ);	//power down
    if (res < 0)
    {
        ST_ERR("lps22hh_baro_init step 2!\n");
        return res;
    }

#ifdef CONFIG_LPS22HH_BARO_DRY
    res = lps22hh_set_interrupt(obj, true);        
    if (res < 0)
    {
        ST_ERR("lps22hh_baro_init step 4!\n");
        return res;
    }
#endif

#ifdef CONFIG_LPS22HH_LOWPASS
    memset(&baro_obj->fir, 0x00, sizeof(baro_obj->fir));
#endif

    return LPS22HH_SUCCESS;
}

static int lps22hh_bero_read_chip_name(struct lps22hh_baro *baro_obj, u8 *buf, int bufsize)
{
    u8 databuf[] = "LPS22HH Barometer";
	
	if (bufsize < sizeof(databuf))
    {
        sprintf(buf, "LPS22HH Barometer");
    }
	else 
    {
        ST_ERR("bufsize is too small\n");
        return LPS22HH_ERR_SETUP_FAILURE;
    }
	
    return LPS22HH_SUCCESS;
}

static int lps22hh_baro_read_chip_id(struct lps22hh_baro *baro_obj, u8 *data)
{
	struct lps22hh_data *obj = container_of(baro_obj, struct lps22hh_data, lps22hh_baro_data);
    struct i2c_client *client = obj->client;
    int res = 0;

    if (NULL == data)
    {
        return LPS22HH_ERR_SETUP_FAILURE;
    }
    
    if (NULL == client)
    {
        return LPS22HH_ERR_I2C;
    }

	res = lps22hh_i2c_read_block(client, LPS22HH_REG_WHO_AM_I, data, 0x01);
    if (res)
    {
		return LPS22HH_ERR_I2C;
    }
	
    return LPS22HH_SUCCESS;
}

#ifdef MISC_DEVICE_FACTORY
static int lps22hh_baro_read_data_temperature(struct lps22hh_baro *baro_obj, u8 *buf, int bufsize)
{
    struct lps22hh_data *obj = container_of(baro_obj, struct lps22hh_data, lps22hh_baro_data);
	struct i2c_client *client = obj->client;
    u8 databuf[20];
    int res = 0;
	s32 temp_data;

    memset(databuf, 0, sizeof(u8)*10);

    if (NULL == buf)
    {
        return LPS22HH_ERR_SETUP_FAILURE;
    }
    if (NULL == client)
    {
        *buf = 0;
        return LPS22HH_ERR_I2C;
    }

    if (atomic_read(&baro_obj->suspend))
    {
        ST_LOG("sensor in suspend read not data!\n");
        return 0;
    }

    if ((res = lps25h_baro_read_rawdata_temperature(baro_obj, &temp_data)))
    {        
        ST_ERR("I2C error: ret value=%d", res);
        return LPS22HH_ERR_I2C;
    }
    else
    {
        temp_data = temp_data / LPS22HH_TEMP_SENSITIVITY;        

        sprintf(buf, "%04x", temp_data);
        if (atomic_read(&baro_obj->trace) & ADX_TRC_FILTER)
        {
            ST_LOG("temperature data: %d!\n", temp_data);
            dumpReg(obj);
        }
    }
	
    return LPS22HH_SUCCESS;
}
#endif

static int lps22hh_baro_read_data(struct lps22hh_baro *baro_obj, u8 *buf, int bufsize)
{
    struct lps22hh_data *obj = container_of(baro_obj, struct lps22hh_data, lps22hh_baro_data);
	struct i2c_client *client = obj->client;
    u8 databuf[20];
    int baro;
    int res = 0;

    memset(databuf, 0, sizeof(u8)*10);

    if (NULL == buf)
    {
        return LPS22HH_ERR_SETUP_FAILURE;
    }
    if (NULL == client)
    {
        *buf = 0;
        return LPS22HH_ERR_I2C;
    }

    if (atomic_read(&baro_obj->suspend))
    {
        ST_LOG("sensor in suspend read not data!\n");
        return 0;
    }

    if ((res = lps22hh_baro_read_rawdata(baro_obj, &baro_obj->data)))
    {        
        ST_ERR("I2C error: ret value=%d", res);
        return LPS22HH_ERR_I2C;
    }
    else
    { 
        baro = baro_obj->data + baro_obj->cali_sw;

        baro = baro * LPS22HH_BARO_DIV / LPS22HH_BARO_SENSITIVITY;        

        sprintf(buf, "%04x", baro);
        if (atomic_read(&baro_obj->trace) & ADX_TRC_FILTER)
        {
            ST_LOG("barometer data: %d!\n", baro);
            dumpReg(obj);
        }
    }
	
    return LPS22HH_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int lps22hh_baro_read_rawdata_string(struct lps22hh_baro *baro_obj, u8 *buf)
{
    struct lps22hh_data *obj = container_of(baro_obj, struct lps22hh_data, lps22hh_baro_data);
	struct i2c_client *client = obj->client;
    int res = 0;

    if (!buf || !client)
    {
        return EINVAL;
    }
    
	res = lps22hh_baro_read_rawdata(baro_obj, &baro_obj->data);
    if (res)
    {        
        ST_ERR("I2C error: ret value=%d", res);
        return LPS22HH_ERR_I2C;
    }
    else
    {
        sprintf(buf, "%04x", baro_obj->data);
    }
    
    return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t lps22hh_attr_baro_show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    struct lps22hh_data *obj = obj_i2c_data;
	struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;
    u8 strbuf[LPS22HH_BUFSIZE];
	int res;
	
    if (NULL == obj->client)
    {
        ST_ERR("i2c client is null!\n");
        return snprintf(buf, PAGE_SIZE, "i2c client is null!\n");
    }

    res = lps22hh_bero_read_chip_name(baro_obj, strbuf, LPS22HH_BUFSIZE);
	if (res)
	{
		return snprintf(buf, PAGE_SIZE, "get chip info error\n");
	}
	
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t lps22hh_attr_baro_show_chipid_value(struct device_driver *ddri, char *buf)
{
    struct lps22hh_data *obj = obj_i2c_data;
	struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;
    u8 chipid = 0x00;
	int res;
	
    if (NULL == obj->client)
    {
        ST_ERR("i2c client is null!\n");
        return snprintf(buf, PAGE_SIZE, "i2c client is null!\n");
    }

	res = lps22hh_baro_read_chip_id(baro_obj, &chipid);
	if (res)
    {
        return snprintf(buf, PAGE_SIZE, "read chip id error!\n");
    }
	
    return snprintf(buf, PAGE_SIZE, "0x%x\n", chipid);
}
/*----------------------------------------------------------------------------*/
static ssize_t lps22hh_attr_baro_show_sensordata_value(struct device_driver *ddri, char *buf)
{
    struct lps22hh_data *obj = obj_i2c_data;
    struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;
    u8 strbuf[LPS22HH_BUFSIZE];
	int res;
    
    if (NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return snprintf(buf, PAGE_SIZE, "i2c client is null!\n");;
    }
	
    res = lps22hh_baro_read_data(baro_obj, strbuf, LPS22HH_BUFSIZE);
	if (res)
    {
        return snprintf(buf, PAGE_SIZE, "read chip id error!\n");
    }
	
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t lps22hh_attr_baro_show_rawdata_value(struct device_driver *ddri, char *buf)
{   
    struct lps22hh_data *obj = obj_i2c_data;
    struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;
    u8 strbuf[LPS22HH_BUFSIZE];
    int res;
	
    if (NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return snprintf(buf, PAGE_SIZE, "read rawdata error !\n");
    }

    res = lps22hh_baro_read_rawdata_string(baro_obj, strbuf);
	if (res)
    {
        return snprintf(buf, PAGE_SIZE, "read chip id error!\n");
    }
	
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t lps22hh_attr_baro_show_power_status(struct device_driver *ddri, char *buf)
{
    struct lps22hh_data *obj = obj_i2c_data;
    struct i2c_client *client = obj->client;
    u8 data;
    int res;
	
    if (NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return snprintf(buf, PAGE_SIZE, "read power status error !\n");;
    }

    res = lps22hh_i2c_read_block(client, LPS22HH_REG_CTRL_REG1, &data, 0x01);
    if (res)
    {
        return snprintf(buf, PAGE_SIZE, "read chip id error!\n");
    }
	
    return snprintf(buf, PAGE_SIZE, "%x\n", data);
}
/*----------------------------------------------------------------------------*/
static ssize_t lps22hh_attr_baro_show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_LPS22HH_LOWPASS
    struct lps22hh_data *obj = obj_i2c_data;
    struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data; 
	
    if (atomic_read(&baro_obj->firlen))
    {
        int idx, len = atomic_read(&baro_obj->firlen);
        ST_LOG("len = %2d, idx = %2d\n", baro_obj->fir.num, baro_obj->fir.idx);

        for(idx = 0; idx < len; idx++)
        {
            ST_LOG("[%5d %5d %5d]\n", baro_obj->fir.raw[idx], baro_obj->fir.raw[idx], baro_obj->fir.raw[idx]);
        }
        
        ST_LOG("sum = [%5d %5d %5d]\n", baro_obj->fir.sum, baro_obj->fir.sum, baro_obj->fir.sum);
        ST_LOG("avg = [%5d %5d %5d]\n", baro_obj->fir.sum/len, baro_obj->fir.sum/len, baro_obj->fir.sum/len);
    }
	
    return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&baro_obj->firlen));
#else
    return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t lps22hh_attr_baro_store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_LPS22HH_LOWPASS
    struct lps22hh_data *obj = obj_i2c_data;
    struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data; 
    int firlen;

    if (1 != sscanf(buf, "%d", &firlen))
    {
        ST_ERR("invallid format\n");
    }
    else if (firlen > C_MAX_FIR_LENGTH)
    {
        ST_ERR("exceeds maximum filter length\n");
    }
    else
    { 
        atomic_set(&baro_obj->firlen, firlen);
        if (0 == firlen)
        {
            atomic_set(&baro_obj->fir_en, 0);
        }
        else
        {
            memset(&baro_obj->fir, 0x00, sizeof(baro_obj->fir));
            atomic_set(&baro_obj->fir_en, 1);
        }
    }
#endif

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t lps22hh_attr_baro_show_trace_value(struct device_driver *ddri, char *buf)
{
    struct lps22hh_data *obj = obj_i2c_data;
    struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data; 
    ssize_t res;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&baro_obj->trace));     
    return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t lps22hh_attr_baro_store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lps22hh_data *obj = obj_i2c_data;
    struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data; 
    int trace;
	
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if (1 == sscanf(buf, "0x%x", &trace))
    {
        atomic_set(&baro_obj->trace, trace);
    }    
    else
    {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }
    
    return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t lps22hh_attr_baro_show_status_value(struct device_driver *ddri, char *buf)
{    
    struct lps22hh_data *obj = obj_i2c_data;
    struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data; 
    ssize_t len = 0;
    
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }    
    
    if (baro_obj->lps22hh_baro_hw)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: i2c_num=%d, direction=%d, sensitivity = %d,(power_id=%d, power_vol=%d)\n", 
                baro_obj->lps22hh_baro_hw->i2c_num, baro_obj->lps22hh_baro_hw->direction, baro_obj->reso->sensitivity, baro_obj->lps22hh_baro_hw->power_id, baro_obj->lps22hh_baro_hw->power_vol);   
        dumpReg(obj);
    }
    else
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    }
	
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t lps22hh_attr_baro_show_chipinit_value(struct device_driver *ddri, char *buf)
{
    struct lps22hh_data *obj = obj_i2c_data;
    struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;
    ssize_t res;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&baro_obj->trace)); 
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t lps22hh_attr_baro_store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lps22hh_data *obj = obj_i2c_data;
    struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return count;
    }

    lps22hh_baro_init(baro_obj, 0);
    dumpReg(obj);

    return count;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, lps22hh_attr_baro_show_chipinfo_value,      NULL);
static DRIVER_ATTR(chipid,               S_IRUGO, lps22hh_attr_baro_show_chipid_value,        NULL);
static DRIVER_ATTR(rawdata,              S_IRUGO, lps22hh_attr_baro_show_rawdata_value,       NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, lps22hh_attr_baro_show_sensordata_value,    NULL);
static DRIVER_ATTR(power,                S_IRUGO, lps22hh_attr_baro_show_power_status,        NULL);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, lps22hh_attr_baro_show_firlen_value,        lps22hh_attr_baro_store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, lps22hh_attr_baro_show_trace_value,         lps22hh_attr_baro_store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, lps22hh_attr_baro_show_status_value,        NULL);
static DRIVER_ATTR(chipinit,   S_IWUSR | S_IRUGO, lps22hh_attr_baro_show_chipinit_value,      lps22hh_attr_baro_store_chipinit_value);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *lps22hh_attr_baro_list[] = {
    &driver_attr_chipinfo,     /*chip information*/
    &driver_attr_chipid,       /*chip id*/
    &driver_attr_sensordata,   /*dump sensor data*/
    &driver_attr_rawdata,      /*dump sensor raw data*/
    &driver_attr_power,        /*show power reg*/
    &driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
    &driver_attr_trace,        /*trace log*/
    &driver_attr_status,
    &driver_attr_chipinit,

};
/*----------------------------------------------------------------------------*/
int lps22hh_baro_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(lps22hh_attr_baro_list)/sizeof(lps22hh_attr_baro_list[0]));
	
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if ((err = driver_create_file(driver, lps22hh_attr_baro_list[idx])))
        {            
            ST_ERR("driver_create_file (%s) = %d\n", lps22hh_attr_baro_list[idx]->attr.name, err);
            break;
        }
    }
	
    return err;
}
/*----------------------------------------------------------------------------*/
int lps22hh_baro_delete_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(lps22hh_attr_baro_list)/sizeof(lps22hh_attr_baro_list[0]));

    if (driver == NULL)
    {
        return -EINVAL;
    }
    
    for(idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, lps22hh_attr_baro_list[idx]);
    }
    
    return err;
}

/*----------------------------------------------------------------------------*/

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int lps22hh_baro_open_report_data_intf(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int lps22hh_baro_enable_nodata_intf(int en)
{
    struct lps22hh_data *obj = obj_i2c_data;
    struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;
    int res = 0;
    bool power = false;
    
    if (1 == en)
    {
        power = true;
    }
    if (0 == en)
    {
        power = false;
    }
	
	baro_obj->enabled = en;
    res = lps22hh_baro_set_power_mode(baro_obj, power);
    if (res != LPS22HH_SUCCESS)
    {
        ST_ERR("lps22hh_baro_set_power_mode fail!\n");
        return -1;
    }
    ST_LOG("lps22hh_baro_enable_nodata_intf OK!\n");
    return 0;
}

static int lps22hh_baro_set_delay_intf(u64 ns)
{
    struct lps22hh_data *obj = obj_i2c_data;
    struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;
    int ms = 0;
    int odr = 0;
    int err;

    ms = (int)ns/1000/1000;
	if (ms <= 5) {
		odr = LPS22HH_ODR_200HZ;
	} else if (ms <= 10) {
		odr = LPS22HH_ODR_100HZ;
	} else if (ms <= 15) {
		odr = LPS22HH_ODR_75HZ;
	} else if (ms <= 20) {
		odr = LPS22HH_ODR_50HZ;
	} else {
		odr = LPS22HH_ODR_25HZ;
	}
	
	baro_obj->odr = odr;
	err = lps22hh_baro_set_odr(baro_obj, baro_obj->odr);
    if (err != LPS22HH_SUCCESS) 	//0x2C->ODR=100Hz
    {
        ST_ERR("Set delay parameter error!\n");
    }

    if (odr >= 50)
    {
        atomic_set(&baro_obj->filter, 0);
    }
    else
    {                    
#ifdef CONFIG_LPS22HH_LOWPASS	
        baro_obj->fir.num = 0;
        baro_obj->fir.idx = 0;
        baro_obj->fir.sum = 0;
        baro_obj->fir.sum = 0;
        baro_obj->fir.sum = 0;
        atomic_set(&baro_obj->filter, 1);
#endif
    }
    
    ST_LOG("lps22hh_baro_set_delay_intf (%d)ms\n", ms);
    return 0;
}

#ifdef KERNEL_4_4
static int lps22hh_baro_batch_intf(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
        return lps22hh_baro_set_delay_intf((u64)samplingPeriodNs);
}

static int lps22hh_baro_flush_intf(void)
{
        return baro_flush_report();
}
#endif

static int lps22hh_baro_get_data_intf(int* value, int* status)
{
    struct lps22hh_data *obj = obj_i2c_data;
    struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;

    u8 buff[LPS22HH_BUFSIZE];
    lps22hh_baro_read_data(baro_obj, buff, LPS22HH_BUFSIZE);
 
    sscanf(buff, "%x", value); 
    *status = SENSOR_STATUS_ACCURACY_HIGH;

    return 0;
}

#ifdef MISC_DEVICE_FACTORY
static int lps22hh_baro_open(struct inode *inode, struct file *file)
{
    file->private_data = obj_i2c_data;

    if (file->private_data == NULL)
    {
        ST_ERR("null pointer!!\n");
        return -EINVAL;
    }
	
    return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int lps22hh_baro_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}
/*----------------------------------------------------------------------------*/
static long lps22hh_baro_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct lps22hh_data *obj = (struct lps22hh_data*)file->private_data;
	struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;
	u8 strbuf[LPS22HH_BUFSIZE];
    void __user *data;
	u32 dat = 0;
    long err = 0;

    //ST_FUN(f);
    if (_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }

    if (err)
    {
        ST_ERR("baroess error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch(cmd)
    {
        case BAROMETER_IOCTL_INIT:
            lps22hh_baro_init(baro_obj, 0);            
            break;

        case BAROMETER_IOCTL_READ_CHIPINFO:
            data = (void __user *) arg;
            if (data == NULL)
            {
                err = -EINVAL;
                break;      
            }
            
            lps22hh_bero_read_chip_name(baro_obj, strbuf, LPS22HH_BUFSIZE);
            if (copy_to_user(data, strbuf, strlen(strbuf)+1))
            {
                err = -EFAULT;
                break;
            }                 
            break;      

		case BAROMETER_GET_PRESS_DATA:
			data = (void __user *) arg;
			if (NULL == data) {
				err = -EINVAL;
				break;
			}

			lps22hh_baro_read_data(baro_obj, strbuf, LPS22HH_BUFSIZE);
			sscanf(strbuf, "%x", &dat);
			if (copy_to_user(data, &dat, sizeof(dat))) {
				err = -EFAULT;
				break;
			}
			break;

		case BAROMETER_GET_TEMP_DATA:
			data = (void __user *) arg;
			if (NULL == data) {
				err = -EINVAL;
				break;
			}
			//lps25h_get_temperature(client, strbuf, LPS25H_BUFSIZE);
			lps22hh_baro_read_data_temperature(baro_obj, strbuf, LPS22HH_BUFSIZE);
			sscanf(strbuf, "%x", &dat);
			if (copy_to_user(data, &dat, sizeof(dat))) {
				err = -EFAULT;
				break;
			}
			break;

        default:
            ST_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
            break;
    }

    return err;
}

#ifdef CONFIG_COMPAT
static long lps22hh_baro_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	ST_FUN();

	if (!file->f_op || !file->f_op->unlocked_ioctl) {
		ST_ERR("compat_ion_ioctl file has no f_op or no f_op->unlocked_ioctl.\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case COMPAT_BAROMETER_IOCTL_INIT:
	case COMPAT_BAROMETER_IOCTL_READ_CHIPINFO:
	case COMPAT_BAROMETER_GET_PRESS_DATA:
	case COMPAT_BAROMETER_GET_TEMP_DATA: {
		ST_LOG("compat_ion_ioctl : BAROMETER_IOCTL_XXX command is 0x%x\n", cmd);
		return file->f_op->unlocked_ioctl(file, cmd,
			(unsigned long)compat_ptr(arg));
	}
	default:
		ST_ERR("compat_ion_ioctl : No such command!! 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}
}
#endif


static struct file_operations lps22hh_baro_fops = {
    .owner = THIS_MODULE,
    .open = lps22hh_baro_open,
    .release = lps22hh_baro_release,
    .unlocked_ioctl = lps22hh_baro_unlocked_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = lps22hh_baro_compat_ioctl,
#endif
};

static struct miscdevice lps22hh_baro_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "barometer",
    .fops = &lps22hh_baro_fops,
};
#else
static int lps22hh_baro_factory_do_self_test(void)
{
	return 0;
}

static int lps22hh_baro_factory_get_cali(int32_t *offset)
{
	struct lps22hh_data *obj = obj_i2c_data;
	struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;

	offset[0] = baro_obj->cali_sw;
	
	return 0;
}

static int lps22hh_baro_factory_set_cali(int32_t offset)
{
	struct lps22hh_data *obj = obj_i2c_data;
	struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;

	baro_obj->cali_sw = offset;

	return 0;
}

static int lps22hh_baro_factory_enable_calibration(void)
{
	return 0;
}

static int lps22hh_baro_factory_clear_cali(void)
{
	struct lps22hh_data *obj = obj_i2c_data;
	struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;

	baro_obj->cali_sw = 0;
	
	return 0;
}

static int lps22hh_baro_factory_get_raw_data(int32_t *data)
{
#if 0
	struct lps22hh_data *obj = obj_i2c_data;
	struct lps22hh_baro *acc_obj = &obj->lps22hh_baro_data;
	s16 databuff[3];

	lps22hh_baro_read_rawdata(acc_obj, databuff);
	data[0] = (s16)databuff[0];
	data[1] = (s16)databuff[1];
	data[2] = (s16)databuff[2];

	ST_LOG("lps22hh_factory_get_raw_data done!\n");
#endif
	return 0;
}

static int lps22hh_baro_factory_get_data(int32_t *data)
{
	int status = 0;
	return lps22hh_baro_get_data_intf(data, &status);
}

static int lps22hh_baro_factory_enable_sensor(bool enable, int64_t sample_periods_ms)
{
	int err;

	err = lps22hh_baro_enable_nodata_intf(enable == true ? 1 : 0);
	if (err) {
		ST_ERR("lps22hh_baro_enable_nodata_intf failed!\n");
		return -1;
	}

	err = lps22hh_baro_set_delay_intf(sample_periods_ms * 1000000);
	if (err) {
		ST_ERR("lps22hh_baro_set_delay_intf failed!\n");
		return -1;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static struct baro_factory_fops lps22hh_baro_factory_fops = {
	.enable_sensor      = lps22hh_baro_factory_enable_sensor,
	.get_data           = lps22hh_baro_factory_get_data,
	.get_raw_data       = lps22hh_baro_factory_get_raw_data,
	.enable_calibration = lps22hh_baro_factory_enable_calibration,
	.clear_cali         = lps22hh_baro_factory_clear_cali,
	.set_cali           = lps22hh_baro_factory_set_cali,
	.get_cali           = lps22hh_baro_factory_get_cali,
	.do_self_test       = lps22hh_baro_factory_do_self_test,
};

static struct baro_factory_public lps22hh_baro_factory_device = {
	.gain        = 1,
	.sensitivity = 1,
	.fops        = &lps22hh_baro_factory_fops,
};
#endif

/*----------------------------------------------------------------------------*/
static int lps22hh_baro_local_init(void)
{
    struct lps22hh_data *obj = obj_i2c_data;
    struct lps22hh_baro *baro_obj = &obj->lps22hh_baro_data;
    int err = 0, retry = 0;
    struct baro_control_path ctl={0};
    struct baro_data_path data={0};
#ifndef KERNEL_4_4		
    const u8 *name = "mediatek,lps22hh";
#else
	struct i2c_client *client = obj->client;
#endif
    ST_FUN();

#ifndef KERNEL_4_4	
    baro_obj->lps22hh_baro_hw = get_baro_dts_func(name, &lps22hh_cust_hw);
    if (!baro_obj->lps22hh_baro_hw) {
        ST_ERR("get lps22hh dts info failed\n");
	goto exit;
    }
#else
    err = get_baro_dts_func(client->dev.of_node, &lps22hh_cust_hw);
    if (err < 0) {
        ST_ERR("get lps22hh dts info fail\n");
	goto exit;
    }

    baro_obj->lps22hh_baro_hw = &lps22hh_cust_hw;
#endif

    atomic_set(&baro_obj->trace, 0);
    atomic_set(&baro_obj->suspend, 0);
    baro_obj->cali_sw = 0;
    
#ifdef CONFIG_LPS22HH_LOWPASS
    if (baro_obj->lps22hh_baro_hw->firlen > C_MAX_FIR_LENGTH)
    {
        atomic_set(&baro_obj->firlen, C_MAX_FIR_LENGTH);
    }    
    else
    {
        atomic_set(&baro_obj->firlen, baro_obj->lps22hh_baro_hw->firlen);
    }
    
    if (atomic_read(&baro_obj->firlen) > 0)
    {
        atomic_set(&baro_obj->fir_en, 1);
    }
#endif

    for(retry = 0; retry < 3; retry++){
        err = lps22hh_baro_init(baro_obj, 1);
	if (err) 
        {
            ST_ERR("lps22hh_baro_device init cilent fail time: %d\n", retry);
            continue;
        } else
	    break;
    }
    if (err != 0)
        goto exit_init_failed;

#ifdef MISC_DEVICE_FACTORY	
    if ((err = misc_register(&lps22hh_baro_device)))
    {
        ST_ERR("lps22hh_baro_device register failed\n");
        goto exit_misc_device_register_failed;
    }
#else
	err = baro_factory_device_register(&lps22hh_baro_factory_device);
	if (err) {
		ST_ERR("lps22hh_baro_factory_device register failed!\n");
		goto exit_misc_device_register_failed;
	}
#endif

    if ((err = lps22hh_baro_create_attr(&(lps22hh_baro_init_info.platform_diver_addr->driver))))
    {
        ST_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
   
    ctl.open_report_data = lps22hh_baro_open_report_data_intf;
    ctl.enable_nodata    = lps22hh_baro_enable_nodata_intf;
    ctl.set_delay        = lps22hh_baro_set_delay_intf;
#ifdef KERNEL_4_4
	ctl.batch            = lps22hh_baro_batch_intf;
    ctl.flush            = lps22hh_baro_flush_intf;
    ctl.is_support_batch = baro_obj->lps22hh_baro_hw->is_batch_supported;
#endif	
	ctl.is_use_common_factory  = false;
    ctl.is_report_input_direct = false;
    
    err = baro_register_control_path(&ctl);
    if (err)
    {
        ST_ERR("register baro control path err\n");
        goto exit_kfree;
    }

    data.get_data = lps22hh_baro_get_data_intf;
    data.vender_div = LPS22HH_BARO_DIV;
    err = baro_register_data_path(&data);
    if (err) {
        ST_ERR("register baro data path err\n");
        goto exit_kfree;
    }

    ST_LOG("%s: OK\n", __func__);
    lps22hh_baro_init_flag = 0;    
    return 0;

exit_kfree:
exit_create_attr_failed:
	lps22hh_baro_delete_attr(&(lps22hh_baro_init_info.platform_diver_addr->driver));
#ifdef MISC_DEVICE_FACTORY
    misc_deregister(&lps22hh_baro_device);
#else
	baro_factory_device_deregister(&lps22hh_baro_factory_device);
#endif
exit_misc_device_register_failed:
exit_init_failed:
    kfree(obj);
exit:
    ST_ERR("%s: err = %d\n", __func__, err);
    lps22hh_baro_init_flag = -1;
	
    return lps22hh_baro_init_flag;
}

/*----------------------------------------------------------------------------*/
static int lps22hh_baro_local_remove(void)
{
    ST_FUN(); 
#ifdef MISC_DEVICE_FACTORY
    misc_deregister(&lps22hh_baro_device);
#else
    baro_factory_device_deregister(&lps22hh_baro_factory_device);
#endif
    lps22hh_baro_delete_attr(&(lps22hh_baro_init_info.platform_diver_addr->driver));
    return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
struct baro_init_info lps22hh_baro_init_info = {
        .name = "lps22hh",
        .init = lps22hh_baro_local_init,
        .uninit = lps22hh_baro_local_remove,
};

MODULE_DESCRIPTION("STMicroelectronics lps22hh driver");
MODULE_AUTHOR("William ZENG");
MODULE_LICENSE("GPL v2");
