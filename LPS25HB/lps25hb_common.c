
/* ST  barometer LPS25HB driver
 *
 *
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
 *version info:
 *    V 1.0.0  init version of barometer function  by jonny


 */
 
#include "lps25hb.h"

static const struct i2c_device_id lps25hb_i2c_id[] = {{LPS25HB_DEV_NAME,0},{}};

#ifdef CONFIG_OF
static const struct of_device_id lps25hb_of_match[] = {
    {.compatible = "mediatek,barometer"},
    {}
};
#endif

static struct i2c_driver lps25hb_i2c_driver;

struct lps25hb_data *obj_i2c_data = NULL;

static DEFINE_MUTEX(lps25hb_i2c_mutex);
static DEFINE_MUTEX(lps25hb_op_mutex);

/*--------------------read function----------------------------------*/
int lps25hb_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    u8 beg = addr;
    int err;
    struct i2c_msg msgs[2]={{0},{0}};
    
    mutex_lock(&lps25hb_i2c_mutex);

   if(len > 1)
   	beg = beg | 0x80;
   
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len =1;
    msgs[0].buf = &beg;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;
    
    if (!client)
    {
        mutex_unlock(&lps25hb_i2c_mutex);
        return -EINVAL;
    }
    else if (len > C_I2C_FIFO_SIZE) 
    {
        ST_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&lps25hb_i2c_mutex);
        return -EINVAL;
    }
    err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
    
    if (err < 0) 
    {
        ST_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, err);
        err = -EIO;
    } 
    else 
    {
        err = 0;
    }
    mutex_unlock(&lps25hb_i2c_mutex);
    return err;
}

int lps25hb_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int err, idx, num;
    u8 buf[C_I2C_FIFO_SIZE];
    err =0;
    mutex_lock(&lps25hb_i2c_mutex);
    if (!client)
    {
        mutex_unlock(&lps25hb_i2c_mutex);
        return -EINVAL;
    }
    else if (len >= C_I2C_FIFO_SIZE) 
    {        
        ST_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&lps25hb_i2c_mutex);
        return -EINVAL;
    }    

    num = 0;

    if(len > 1)
         buf[num++] = addr | 0x80 ;
    else
         buf[num++] = addr;
	
    for (idx = 0; idx < len; idx++)
    {
        buf[num++] = data[idx];
    }

    err = i2c_master_send(client, buf, num);
    if (err < 0)
    {
        ST_ERR("send command error!!\n");
        mutex_unlock(&lps25hb_i2c_mutex);
        return -EFAULT;
    } 
    mutex_unlock(&lps25hb_i2c_mutex);
    return err; //if success will return transfer lenth
}
/*----------------------------------------------------------------------------*/

void dumpReg(struct lps25hb_data *obj)
{
    struct i2c_client *client = obj->client;

    int i=0;
    u8 addr = 0x10;
    u8 regdata=0;
    for(i=0; i<10; i++)
    {
        //dump all
        lps25hb_i2c_read_block(client,addr,&regdata,1);
        ST_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
        addr++;
    }
}
/*--------------------ADXL power control function----------------------------------*/
static void lps25hb_chip_power(struct lps25hb_data *obj, unsigned int on) 
{
    return;
}

int lps25hb_set_bdu(struct lps25hb_data *obj, bool flag)
{
	struct i2c_client *client = obj->client;

    u8 databuf[2];
    u8 addr = LPS25HB_REG_CTRL_REG1;
    int res = 0;

    memset(databuf, 0, sizeof(u8)*2); 

    if((lps25hb_i2c_read_block(client, addr, databuf, 0x01))<0)
    {
        ST_ERR("read reg_ctl_reg1 register err!\n");
        return LPS25HB_ERR_I2C;
    }

   if( true ==flag)
        databuf[0] = databuf[0] | 0x04;
   else
        databuf[0] &= ~0x04;

    res = lps25hb_i2c_write_block(client, LPS25HB_REG_CTRL_REG1, databuf, 0x01);
    if(res < 0)
    {
        return LPS25HB_ERR_I2C;
    }
    
    return LPS25HB_SUCCESS;    
}

#ifdef CONFIG_LPS25HB_BARO_DRY
int lps25hb_set_interrupt(struct lps25hb_data *obj, u8 intenable)
{
	struct i2c_client *client = obj->client;

    u8 databuf[2];
    u8 addr = LPS25HB_REG_CTRL_REG3;
    int res = 0;

    memset(databuf, 0, sizeof(u8)*2); 

    if((lps25hb_i2c_read_block(client, addr, databuf, 0x01))<0)
    {
        ST_ERR("read reg_ctl_reg1 register err!\n");
        return LPS25HB_ERR_I2C;
    }

    databuf[0] = 0x00;

    res = lps25hb_i2c_write_block(client, LPS25HB_REG_CTRL_REG3, databuf, 0x01);
    if(res < 0)
    {
        return LPS25HB_ERR_I2C;
    }
    
    return LPS25HB_SUCCESS;    
}
#endif

int lps25hb_check_device_id(struct lps25hb_data *obj)
{
	struct i2c_client *client = obj->client;
	u8 databuf[2];
	int err;
	
	err = lps25hb_i2c_read_block(client, LPS25HB_REG_WHO_AM_I, databuf, 0x01);
	if (err < 0)
	{
		return LPS25HB_ERR_I2C;
	}
	
    ST_LOG("LPS25HB who am I = 0x%x\n", databuf[0]);
	if(databuf[0]!=LPS25HB_FIXED_DEVID)
	{
		return LPS25HB_ERR_IDENTIFICATION;
	}
	
	return LPS25HB_SUCCESS;
}

static ssize_t lps25hb_attr_i2c_show_reg_value(struct device_driver *ddri, char *buf)
{
    struct lps25hb_data *obj = obj_i2c_data;

    u8 reg_value;
    ssize_t res;
    int err;
	
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    err = lps25hb_i2c_read_block(obj->client, obj->reg_addr, &reg_value, 0x01);
	if (err < 0)
	{
		res = snprintf(buf, PAGE_SIZE, "i2c read error!!\n");
        return res;
	}
	
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", reg_value); 
    return res;
}

static ssize_t lps25hb_attr_i2c_store_reg_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lps25hb_data *obj = obj_i2c_data;

    u8 reg_value;
	int res;
	
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if(1 == sscanf(buf, "0x%hhx", &reg_value))
    {
	    res = lps25hb_i2c_write_block(obj->client, obj->reg_addr, &reg_value, 0x01);
	    if(res < 0)
        {
            return LPS25HB_ERR_I2C;
        }
    }    
    else
    {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }

    return count;
}

static ssize_t lps25hb_attr_baro_show_reg_addr(struct device_driver *ddri, char *buf)
{
    struct lps25hb_data *obj = obj_i2c_data;

    ssize_t res;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", obj->reg_addr); 
    return res;
}

static ssize_t lps25hb_attr_i2c_store_reg_addr(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lps25hb_data *obj = obj_i2c_data;
	u8 reg_addr;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if(1 == sscanf(buf, "0x%hhx", &reg_addr))
    {
        obj->reg_addr = reg_addr;
    }    
    else
    {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }

    return count;
}
static DRIVER_ATTR(reg_value, S_IWUSR | S_IRUGO, lps25hb_attr_i2c_show_reg_value, lps25hb_attr_i2c_store_reg_value);
static DRIVER_ATTR(reg_addr,  S_IWUSR | S_IRUGO, lps25hb_attr_baro_show_reg_addr,  lps25hb_attr_i2c_store_reg_addr);

static struct driver_attribute *lps25hb_attr_i2c_list[] = {
    &driver_attr_reg_value,
	&driver_attr_reg_addr,
};

int lps25hb_i2c_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(lps25hb_attr_i2c_list)/sizeof(lps25hb_attr_i2c_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, lps25hb_attr_i2c_list[idx])))
        {            
            ST_ERR("driver_create_file (%s) = %d\n", lps25hb_attr_i2c_list[idx]->attr.name, err);
            break;
        }
    }    
    return err;
}

int lps25hb_i2c_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(lps25hb_attr_i2c_list)/sizeof(lps25hb_attr_i2c_list[0]));

    if(driver == NULL)
    {
        return -EINVAL;
    }
    

    for(idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, lps25hb_attr_i2c_list[idx]);
    }
    

    return err;
}

/*----------------------------------------------------------------------------*/
static int lps25hb_suspend(struct i2c_client *client, pm_message_t msg) 
{
    struct lps25hb_data *obj = i2c_get_clientdata(client);
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;
    int err = 0;
	
    ST_FUN();
    
    if((msg.event == PM_EVENT_SUSPEND) && (baro_obj->enabled == 1))
    {   
        mutex_lock(&lps25hb_op_mutex);
        if(obj == NULL)
        {    
            mutex_unlock(&lps25hb_op_mutex);
            ST_ERR("null pointer!!\n");
            return -EINVAL;
        }

        err = lps25hb_baro_set_power_mode(baro_obj, false);		
        if(err)
        {
            ST_ERR("write power control fail!!\n");
            mutex_unlock(&lps25hb_op_mutex);
            return err;        
        }
        
        atomic_set(&baro_obj->suspend, 1);
		mutex_unlock(&lps25hb_op_mutex);
        lps25hb_chip_power(obj, 0);
    }
    
	ST_LOG("lps25hb i2c suspended\n");
    return err;
}
/*----------------------------------------------------------------------------*/
static int lps25hb_resume(struct i2c_client *client)
{
    struct lps25hb_data *obj = i2c_get_clientdata(client);
    struct lps25hb_baro *baro_obj = &obj->lps25hb_baro_data;
    int err;
	
    ST_FUN();
    lps25hb_chip_power(obj, 1);
	if (baro_obj->enabled == 1) 
    {
	    mutex_lock(&lps25hb_op_mutex);
	    if(obj == NULL)
	    {
		    mutex_unlock(&lps25hb_op_mutex);
		    ST_ERR("null pointer!!\n");
		    return -EINVAL;
	    }
    
        err = lps25hb_baro_set_power_mode(baro_obj, true);
        if(err)
        {
            mutex_unlock(&lps25hb_op_mutex);
            ST_ERR("initialize client fail!!\n");
            return err;        
        }
        atomic_set(&baro_obj->suspend, 0);
        mutex_unlock(&lps25hb_op_mutex);
    }
	ST_LOG("lps25hb i2c resumed\n");
    return 0;
}
/*----------------------------------------------------------------------------*/
static int lps25hb_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
    strcpy(info->type, LPS25HB_DEV_NAME);
    return 0;
}

/*----------------------------------------------------------------------------*/

static int lps25hb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct lps25hb_data *obj;
	
    int err = 0;
	ST_FUN();
	
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
		return err;
    }
    memset(obj, 0, sizeof(struct lps25hb_data));
	obj_i2c_data = obj;
    obj->client = client;
    i2c_set_clientdata(client, obj);
	
	err = lps25hb_check_device_id(obj);
	if (err)
	{
        ST_ERR("check device error!\n");
		goto exit_check_device_failed;
	}

    err = lps25hb_i2c_create_attr(&lps25hb_i2c_driver.driver);
	if (err)
	{
        ST_ERR("create attr error!\n");
		goto exit_create_attr_failed;
	}
#ifdef CONFIG_LPS25HB_BARO_DRY
	//TODO: request irq for data ready or pedometer, SMD, TILT, etc.
#endif	
    baro_driver_add(&lps25hb_baro_init_info);

	ST_LOG("lps25hb_i2c_probe exit\n");
    return 0;
exit_create_attr_failed:
exit_check_device_failed:
    kfree(obj);
	return err;

}
/*----------------------------------------------------------------------------*/
static int lps25hb_i2c_remove(struct i2c_client *client)
{
	lps25hb_i2c_delete_attr(&lps25hb_i2c_driver.driver);
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}
/*----------------------------------------------------------------------------*/
static struct i2c_driver lps25hb_i2c_driver = {
    .driver = {
        .name           = LPS25HB_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = lps25hb_of_match,
#endif
    },
    .probe              = lps25hb_i2c_probe,
    .remove             = lps25hb_i2c_remove,
    .detect             = lps25hb_i2c_detect,
    .suspend            = lps25hb_suspend,
    .resume             = lps25hb_resume,
    .id_table           = lps25hb_i2c_id,
};

/*----------------------------------------------------------------------------*/
static int __init lps25hb_module_init(void)
{
    ST_FUN();
    if(i2c_add_driver(&lps25hb_i2c_driver))
    {
        ST_ERR("add driver error\n");
        return -1;
    }
   
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit lps25hb_module_exit(void)
{
    i2c_del_driver(&lps25hb_i2c_driver);

    ST_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(lps25hb_module_init);
module_exit(lps25hb_module_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LPS25HB I2C driver");
