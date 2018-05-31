
#include "lis2hh12.h"

static struct i2c_driver lis2hh12_i2c_driver;
struct lis2hh12_data *obj_i2c_data = NULL;

static DEFINE_MUTEX(lis2hh12_i2c_mutex);
static DEFINE_MUTEX(lis2hh12_op_mutex);

/*--------------------read function----------------------------------*/
int lis2hh12_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    u8 beg = addr;
    int ret;
    struct i2c_msg msgs[2] = {{0},{0}};
    
    mutex_lock(&lis2hh12_i2c_mutex);
    
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len =1;
    msgs[0].buf = &beg;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;
    
    if (!client) {
        mutex_unlock(&lis2hh12_i2c_mutex);
        return -EINVAL;
    } else if (len > C_I2C_FIFO_SIZE) {
        ST_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&lis2hh12_i2c_mutex);
        return -EINVAL;
    }
	
    ret = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
    if (ret < 0) {
        ST_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, ret);
        ret = -EIO;
    } else {
        ret = 0;
    }

    mutex_unlock(&lis2hh12_i2c_mutex);
    return ret; 
}

int lis2hh12_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   
    /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int ret = 0, idx, num;
    u8 buf[C_I2C_FIFO_SIZE];
	
    mutex_lock(&lis2hh12_i2c_mutex);
    if (!client) {
        ret = -EINVAL;
	goto exit_failed;
    } else if (len >= C_I2C_FIFO_SIZE) {        
        ST_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        ret = -EINVAL;
	goto exit_failed;
    }    

    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++)
        buf[num++] = data[idx];

    ret = i2c_master_send(client, buf, num);
    if (ret < 0) {
        ST_ERR("send command error!!\n");
        ret = -EFAULT;
    }
	
exit_failed:
    mutex_unlock(&lis2hh12_i2c_mutex);
    return ret; //if success will return transfer lenth
}

int lis2hh12_i2c_write_with_mask(struct i2c_client *client, u8 addr, u8 mask, u8 data)
{
    int ret;
    u8 new_data = 0x00, old_data = 0x00;

    ret = lis2hh12_i2c_read_block(client, addr, &old_data, 1);
    if (ret < 0)
	return ret;
	
    new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));
    if (new_data == old_data)
	return 0;
	
    return lis2hh12_i2c_write_block(client, addr, &new_data, 1);
}

void dumpReg(struct lis2hh12_data *obj)
{
    struct i2c_client *client = obj->client;
    u8 addr = 0x1E, regdata = 0;
    int i = 0;
	
    for (i = 0; i < 28; i++) {
        lis2hh12_i2c_read_block(client, addr, &regdata, 1);
        ST_LOG("Reg addr=0x%02x regdata=0x%02x\n", addr, regdata);
        addr++;
    }
}

static void lis2hh12_chip_power(struct lis2hh12_data *obj, unsigned int on) 
{
    return;
}

static int lis2hh12_read_chip_id(struct lis2hh12_data *obj, u8 *data)
{
    struct i2c_client *client = obj->client;
    int ret;
	
    ret = lis2hh12_i2c_read_block(client, LIS2HH12_REG_WHO_AM_I, data, 0x01);
    if (ret < 0)
	return LIS2HH12_ERR_I2C;
	
    return LIS2HH12_SUCCESS;
}

static int lis2hh12_check_device_id(struct lis2hh12_data *obj)
{
    u8 buf;
    int ret;

    ret = lis2hh12_read_chip_id(obj, &buf);
    if (ret < 0) {
	ST_ERR("read chip id error\n");
	return LIS2HH12_ERR_I2C;
    }

    obj->chip_id = buf;
    ST_LOG("LIS2HH12 who am I = 0x%x\n", buf);

    if (buf == LIS2HH12_FIXED_DEVID_LIS2HH12) {
	sprintf(obj->name, "LIS2HH12");
	return LIS2HH12_SUCCESS;
    } else {
	ST_ERR("not support chip id error\n");
	return LIS2HH12_ERR_IDENTIFICATION;
    }
}

static ssize_t lis2hh12_attr_i2c_show_reg_value(struct device_driver *ddri, char *buf)
{
    struct lis2hh12_data *obj = obj_i2c_data;
    u8 reg_value;
    int ret;
	
    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
	
    ret = lis2hh12_i2c_read_block(obj->client, obj->reg_addr, &reg_value, 0x01);
    if (ret < 0) {
	ret = snprintf(buf, PAGE_SIZE, "i2c read error!!\n");
        return ret;
    }
	
    ret = snprintf(buf, PAGE_SIZE, "0x%04X\n", reg_value); 
    return ret;
}

static ssize_t lis2hh12_attr_i2c_store_reg_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lis2hh12_data *obj = obj_i2c_data;
    u8 reg_value;
	int ret;
	
    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if (1 == sscanf(buf, "0x%hhx", &reg_value)) {
	ret = lis2hh12_i2c_write_block(obj->client, obj->reg_addr, &reg_value, 0x01);
	if (ret < 0)
            return LIS2HH12_ERR_I2C;
    } else {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }

    return count;
}

static ssize_t lis2hh12_attr_i2c_show_reg_addr(struct device_driver *ddri, char *buf)
{
    struct lis2hh12_data *obj = obj_i2c_data;
    ssize_t ret;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    ret = snprintf(buf, PAGE_SIZE, "0x%04X\n", obj->reg_addr); 
    return ret;
}

static ssize_t lis2hh12_attr_i2c_store_reg_addr(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lis2hh12_data *obj = obj_i2c_data;
    u8 reg_addr;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if (1 == sscanf(buf, "0x%hhx", &reg_addr))
        obj->reg_addr = reg_addr;
    else
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);

    return count;
}

static DRIVER_ATTR(reg_value, S_IWUSR | S_IRUGO, lis2hh12_attr_i2c_show_reg_value, lis2hh12_attr_i2c_store_reg_value);
static DRIVER_ATTR(reg_addr,  S_IWUSR | S_IRUGO, lis2hh12_attr_i2c_show_reg_addr,  lis2hh12_attr_i2c_store_reg_addr);

static struct driver_attribute *lis2hh12_attr_i2c_list[] = {
    &driver_attr_reg_value,
    &driver_attr_reg_addr,
};

int lis2hh12_i2c_create_attr(struct device_driver *driver) 
{
    int idx, ret = 0;
    int num = (int)(sizeof(lis2hh12_attr_i2c_list)/sizeof(lis2hh12_attr_i2c_list[0]));
	
    if (driver == NULL)
        return -EINVAL;

    for (idx = 0; idx < num; idx++) {
        if ((ret = driver_create_file(driver, lis2hh12_attr_i2c_list[idx]))) {           
            ST_ERR("driver_create_file (%s) = %d\n", lis2hh12_attr_i2c_list[idx]->attr.name, ret);
            break;
        }
    }
	
    return ret;
}

int lis2hh12_i2c_delete_attr(struct device_driver *driver)
{
    int idx;
    int num = (int)(sizeof(lis2hh12_attr_i2c_list)/sizeof(lis2hh12_attr_i2c_list[0]));

    if (driver == NULL)
        return -EINVAL;
    
    for (idx = 0; idx < num; idx++)
        driver_remove_file(driver, lis2hh12_attr_i2c_list[idx]);
    
    return 0;
}

/*----------------------------------------------------------------------------*/
static int lis2hh12_suspend(struct i2c_client *client, pm_message_t msg) 
{
    struct lis2hh12_data *obj = i2c_get_clientdata(client);
    struct lis2hh12_acc *acc_obj = &obj->lis2hh12_acc_data;
    int ret = 0;
	
    ST_FUN();
	
    if ((msg.event == PM_EVENT_SUSPEND) && (obj->acc_enabled == 1)) {   
        mutex_lock(&lis2hh12_op_mutex);
        if (obj == NULL) {    
            ST_ERR("null pointer!!\n");
            ret = -EINVAL;
	    goto exit_failed;
        }

        ret = lis2hh12_acc_set_power_mode(acc_obj, false);		
        if (ret) {
            ST_ERR("write power control fail!!\n");
            goto exit_failed;        
        }
        
        atomic_set(&acc_obj->suspend, 1);
	mutex_unlock(&lis2hh12_op_mutex);
        lis2hh12_chip_power(obj, 0);
    }

    ST_LOG("lis2hh12 i2c suspended\n");
    return ret;
	
exit_failed:
    mutex_unlock(&lis2hh12_op_mutex);
    return ret; 
}

static int lis2hh12_resume(struct i2c_client *client)
{
    struct lis2hh12_data *obj = i2c_get_clientdata(client);
    struct lis2hh12_acc *acc_obj = &obj->lis2hh12_acc_data;
    int ret;

    ST_FUN();
    lis2hh12_chip_power(obj, 1);

    if (obj->acc_enabled == 1) {
	mutex_lock(&lis2hh12_op_mutex);
	if (obj == NULL) {
	    ST_ERR("null pointer!!\n");
	    ret = -EINVAL;
	    goto exit_failed;
	}

    	ret = lis2hh12_acc_set_power_mode(acc_obj, true);
    	if (ret) {
	    ST_ERR("initialize client fail!!\n");
	    goto exit_failed;        
   	}

     	atomic_set(&acc_obj->suspend, 0);
	mutex_unlock(&lis2hh12_op_mutex);
    }

    ST_LOG("lis2hh12 i2c resumed\n");
    return 0;

exit_failed:
    mutex_unlock(&lis2hh12_op_mutex);
    return ret; 
}

static int lis2hh12_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
    strcpy(info->type, LIS2HH12_DEV_NAME);
    return 0;
}

static int lis2hh12_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct lis2hh12_data *obj;
    int ret = 0;

    ST_FUN();

    if (!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))) {
	ret = -ENOMEM;
	return ret;
    }

    memset(obj, 0, sizeof(struct lis2hh12_data));
    obj_i2c_data = obj;
    obj->client = client;
    i2c_set_clientdata(client, obj);

    obj->acc_enabled = 0;

    ret = lis2hh12_check_device_id(obj);
    if (ret) {
	ST_ERR("check device error!\n");
	goto exit_check_device_failed;
    }

    ret = lis2hh12_i2c_create_attr(&lis2hh12_i2c_driver.driver);
    if (ret) {
	ST_ERR("create attr error!\n");
	goto exit_check_device_failed;
    }

    ret = acc_driver_add(&lis2hh12_acc_init_info);

    ST_LOG("lis2hh12_i2c_probe successfully\n");
    return 0;

exit_check_device_failed:
    kfree(obj);
    return ret;
}

static int lis2hh12_i2c_remove(struct i2c_client *client)
{
    lis2hh12_i2c_delete_attr(&lis2hh12_i2c_driver.driver);
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}

static const struct i2c_device_id lis2hh12_i2c_id[] = {{LIS2HH12_DEV_NAME,0},{}};

#ifdef CONFIG_OF
static const struct of_device_id lis2hh12_of_match[] = {
    {.compatible = "mediatek,gsensor"},
    {}
};
#endif

static struct i2c_driver lis2hh12_i2c_driver = {
    .driver = {
        .name           = LIS2HH12_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = lis2hh12_of_match,
#endif
    },
    .probe              = lis2hh12_i2c_probe,
    .remove             = lis2hh12_i2c_remove,
    .detect             = lis2hh12_i2c_detect,
    .suspend            = lis2hh12_suspend,
    .resume             = lis2hh12_resume,
    .id_table           = lis2hh12_i2c_id,
};

/*----------------------------------------------------------------------------*/
static int __init lis2hh12_module_init(void)
{
    ST_FUN();

    if (i2c_add_driver(&lis2hh12_i2c_driver)) {
        ST_ERR("add acc driver error\n");
        return -1;
    }
	
    return LIS2HH12_SUCCESS;
}

static void __exit lis2hh12_module_exit(void)
{
    ST_FUN();
    i2c_del_driver(&lis2hh12_i2c_driver);
}

/*----------------------------------------------------------------------------*/
module_init(lis2hh12_module_init);
module_exit(lis2hh12_module_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LIS2HH12 I2C driver");
