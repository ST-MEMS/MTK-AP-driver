/* LSM6DSO IMU driver
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

#include "lsm6dso.h"

static struct i2c_driver lsm6dso_i2c_driver;
struct lsm6dso_data *obj_i2c_data = NULL;

static DEFINE_MUTEX(lsm6dso_i2c_mutex);
static DEFINE_MUTEX(lsm6dso_op_mutex);

int lsm6dso_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    u8 beg = addr;
    int res;
    struct i2c_msg msgs[2]={{0},{0}};
    
    mutex_lock(&lsm6dso_i2c_mutex);
    
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len =1;
    msgs[0].buf = &beg;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;
    
    if (!client) {
        mutex_unlock(&lsm6dso_i2c_mutex);
        return -EINVAL;
    } else if (len > C_I2C_FIFO_SIZE) {
        ST_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&lsm6dso_i2c_mutex);
        return -EINVAL;
    }

    res = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
    if (res < 0) {
        ST_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, res);
        res = -EIO;
    } else {
        res = 0;
    }

    mutex_unlock(&lsm6dso_i2c_mutex);

    return res; 
}

int lsm6dso_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   
    /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int res = 0, idx, num;
    u8 buf[C_I2C_FIFO_SIZE];
	
    mutex_lock(&lsm6dso_i2c_mutex);

    if (!client) {
        mutex_unlock(&lsm6dso_i2c_mutex);
        return -EINVAL;
    } else if (len >= C_I2C_FIFO_SIZE) {        
        ST_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&lsm6dso_i2c_mutex);
        return -EINVAL;
    }    

    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++) {
        buf[num++] = data[idx];
    }

    res = i2c_master_send(client, buf, num);
    if (res < 0) {
        ST_ERR("send command error!!\n");
        mutex_unlock(&lsm6dso_i2c_mutex);
        return -EFAULT;
    } 

    mutex_unlock(&lsm6dso_i2c_mutex);

    return res; //if success will return transfer lenth
}

int lsm6dso_i2c_write_with_mask(struct i2c_client *client,u8 addr, u8 mask, u8 data)
{
    int res;
    u8 new_data = 0x00, old_data = 0x00;

    res = lsm6dso_i2c_read_block(client, addr, &old_data,1);
    if (res < 0)
	return res;
	
    new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));
    if (new_data == old_data) {
	    return 0;
    }
	
    return lsm6dso_i2c_write_block(client, addr, &new_data,1);
}

#if (CONFIG_STEP_COUNTER || CONFIG_STEP_DETECT)
int lsm6dso_step_configure(struct lsm6dso_data *obj)
{
    struct i2c_client *client = obj->client;
    u8 databuf =0; 
    int res = 0;
   
    ST_FUN();    

    // Enable access to embedded functions registers
    res = lsm6dso_i2c_write_with_mask(client, LSM6DSO_FUNC_CFG_ACCESS_REG, 
				         LSM6DSO_REG_FUNC_CFG_ACCESS_MASK_FUNC_CFG_EN, LSM6DSO_EN); 
    if (res <0) {
	    ST_ERR("write LSM6DSO_FUNC_CFG_ACCESS_REG register enable err!\n");
	    return LSM6DSO_ERR_I2C;
    }
    // Select write operation mode
    databuf = 0xc0;
	 res = lsm6dso_i2c_write_block(client, LSM6DSO_PAGE_RW_REG, &databuf, 0x01); 
    if (res <0) {
	    ST_ERR("write lsm6dso step threshold err!\n");
	    return LSM6DSO_ERR_I2C;
    }
	// Select page 1
    databuf = 0x11;
	 res = lsm6dso_i2c_write_block(client, LSM6DSO_PAGE_SEL_REG, &databuf, 0x01); 
    if (res <0) {
	    ST_ERR("write lsm6dso step threshold err!\n");
	    return LSM6DSO_ERR_I2C;
    }
	// Set embedded advanced features register to be written (PEDO_CMD_REG)
    databuf = 0x83;
	 res = lsm6dso_i2c_write_block(client, LSM6DSO_PAGE_ADDRESS_REG, &databuf, 0x01); 
    if (res <0) {
	    ST_ERR("write lsm6dso step threshold err!\n");
	    return LSM6DSO_ERR_I2C;
    }
	// Enable false positive rejection block (FP_REJECTION_EN = 1)
    databuf = 0x04;
	 res = lsm6dso_i2c_write_block(client, LSM6DSO_PAGE_VALUE_REG, &databuf, 0x01); 
    if (res <0) {
	    ST_ERR("write lsm6dso step threshold err!\n");
	    return LSM6DSO_ERR_I2C;
    }
    // Set embedded advanced features register to be written (PEDO_DEB_STEPS_CONF)
    databuf = 0x84;
	 res = lsm6dso_i2c_write_block(client, LSM6DSO_PAGE_ADDRESS_REG, &databuf, 0x01); 
    if (res <0) {
	    ST_ERR("write lsm6dso step threshold err!\n");
	    return LSM6DSO_ERR_I2C;
    }
	// DEB_STEP = 0x07(7Step)
    databuf = 0x07;
	 res = lsm6dso_i2c_write_block(client, LSM6DSO_PAGE_VALUE_REG, &databuf, 0x01); 
    if (res <0) {
	    ST_ERR("write lsm6dso step threshold err!\n");
	    return LSM6DSO_ERR_I2C;
    }	
	
	// Disalbe write operation mode
    databuf = 0x80;
	 res = lsm6dso_i2c_write_block(client, LSM6DSO_PAGE_RW_REG, &databuf, 0x01); 
    if (res <0) {
	    ST_ERR("write lsm6dso step threshold err!\n");
	    return LSM6DSO_ERR_I2C;
    }
#if (CONFIG_PEDOMETER_ALWAYS_ON)
    // Enable pedometer
    res = lsm6dso_i2c_write_with_mask(client, LSM6DSO_EMB_FUNC_EN_A_REG, 0x08, LSM6DSO_DIS); 
    if (res <0) {
	    ST_ERR("write LSM6DSO_FUNC_CFG_ACCESS_REG register enable err!\n");
	    return LSM6DSO_ERR_I2C;
    }
#endif
	// Disable access to embedded functions registers
    res = lsm6dso_i2c_write_with_mask(client, LSM6DSO_FUNC_CFG_ACCESS_REG, 
				         LSM6DSO_REG_FUNC_CFG_ACCESS_MASK_FUNC_CFG_EN, LSM6DSO_DIS); 
    if (res <0) {
	    ST_ERR("write LSM6DSO_FUNC_CFG_ACCESS_REG register enable err!\n");
	    return LSM6DSO_ERR_I2C;
    }
	
#if (CONFIG_PEDOMETER_ALWAYS_ON)
    res = lsm6dso_i2c_write_with_mask(client, LSM6DSO_CTRL1_XL_REG, LSM6DSO_REG_CTRL1_XL_MASK_FS_XL, 
					 LSM6DSO_REG_CTRL1_XL_FS_4G);
    if (res < 0) {
	    ST_ERR("write 4g fullscalee err!\n");
	    return LSM6DSO_ERR_I2C;
    }

    res = lsm6dso_i2c_write_with_mask(client, LSM6DSO_CTRL1_XL_REG, LSM6DSO_REG_CTRL1_XL_MASK_ODR_XL, 
				         LSM6DSO_REG_CTRL1_XL_ODR_26HZ);
    if (res < 0) {
	    ST_ERR("write 26hz odr err!\n");
        return LSM6DSO_ERR_I2C;
    } 
#endif
	
    return LSM6DSO_SUCCESS;
}
#endif

void dumpReg(struct lsm6dso_data *obj)
{
    struct i2c_client *client = obj->client;
    u8 addr = 0x10, regdata = 0;
    int i = 0;
	
    for (i = 0; i < 10; i++) {
        lsm6dso_i2c_read_block(client, addr, &regdata, 1);
        ST_LOG("Reg addr=%x regdata=%x\n", addr, regdata);
        addr++;
    }
}
#if(CONFIG_HARDWARE_INTERRUPT)
static irqreturn_t lsm6dso_isr(int irq, void *dev)
{
    struct lsm6dso_data *obj = obj_i2c_data;

    if (atomic_read(&obj->irq_enabled)) {
	    disable_irq_nosync(obj->irq);
	    atomic_set(&obj->irq_enabled,0);
    }

    queue_work(obj->irq_work_queue, &obj->irq_work);
	
    return IRQ_HANDLED;
}

static void lsm6dso_irq_work_func(struct work_struct *work)
{
    struct lsm6dso_data *obj = obj_i2c_data;
    struct i2c_client *client = obj->client;
    int res;
    u8 buf;

    ST_FUN();
	
    res = lsm6dso_i2c_read_block(client, LSM6DSO_EMB_FUNC_STATUS_MAINPAGE_REG, &buf, 0x01);
    if (res < 0) {
	    goto work_exit;
    }

#if (CONFIG_SIGNIFICANT_MOTION)
    if ((LSM6DSO_FlAG_SIGN_MOTION_MASK & buf) && obj->significant_enabled) {
	step_notify(TYPE_SIGNIFICANT);
    }
#endif

#if (CONFIG_STEP_DETECT)
    if ((LSM6DSO_FLAG_STEP_MASK & buf) && obj->step_d_enabled) {
	step_notify(TYPE_STEP_DETECTOR);
    }
#endif

#if (CONFIG_TILT)
    if ((LSM6DSO_FLAG_TILT_MASK & buf) && obj->tilt_enabled) {	
	tilt_notify();
    }
#endif

work_exit:
    if (!atomic_read(&obj->irq_enabled)) {
	    enable_irq(obj->irq);
	    atomic_set(&obj->irq_enabled,1);
    }
}

int lsm6dso_set_interrupt(void)
{
    struct lsm6dso_data *obj = obj_i2c_data;
    //struct i2c_client *client = obj->client;
    struct device_node *node = NULL;
    u32 ints[2] = {0, 0};
    int res = -1;
    const struct of_device_id gyro_of_match[] = {
	    { .name = "GYRO", },
	    { .compatible = "mediatek,gyroscope", },
	    {},
    };
	
    ST_FUN();

#if 0
    /* set sensor interrupt low trigger, sensor default is high trigger */
    res = lsm6dso_i2c_write_with_mask(client, LSM6DSO_CTRL3_C_REG, LSM6DSO_INT_ACTIVE_MASK, LSM6DSO_EN);
    if (res < 0) {
	    ST_ERR("write interrupt high or low active  err!\n");
	    return LSM6DSO_ERR_I2C;
    }
#endif
    node = of_find_matching_node(node, gyro_of_match);
    if (node) {
	    ST_LOG("irq node is ok!");
	    res = of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
	    if (res == 0) {
		    ST_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
		    gpio_set_debounce(ints[0], ints[1]);
	    } else {
		    ST_LOG("debounce time not found\n");
	}

	obj->irq = irq_of_parse_and_map(node, 0);
	ST_LOG("step_irq = %d\n", obj->irq);
	if (!obj->irq) {
	    ST_ERR("irq_of_parse_and_map fail!!\n");
	    return -EINVAL;
	}
		
	INIT_WORK(&obj->irq_work, lsm6dso_irq_work_func);
	obj->irq_work_queue = create_singlethread_workqueue("lsm6dso_step_wq");
	if (!obj->irq_work_queue) {
	    res = -ENOMEM;
	    ST_ERR("cannot create work queue1");
	    return res;
	}
		
	if (request_irq(obj->irq, lsm6dso_isr, IRQ_TYPE_LEVEL_HIGH, "gyroscope", NULL)) {		
	    ST_ERR("IRQ LINE NOT AVAILABLE!!\n");
	    return -EINVAL;
	}
		
	disable_irq_nosync(obj->irq);
	atomic_set(&obj->irq_enabled,0); 
    } else {
	ST_ERR("null irq node!!\n");
	return -EINVAL;
    }
	 
    return LSM6DSO_SUCCESS;
}
#endif

static int lsm6dso_read_chip_id(struct lsm6dso_data *obj, u8 *data)
{
    struct i2c_client *client = obj->client;
    int res;
	
    res = lsm6dso_i2c_read_block(client, LSM6DSO_WHO_AM_I_REG, data, 0x01);
    if (res < 0){
	return LSM6DSO_ERR_I2C;
    }

    return LSM6DSO_SUCCESS;
}

static int lsm6dso_check_device_id(struct lsm6dso_data *obj)
{
    u8 buf;
    int res;

    res = lsm6dso_read_chip_id(obj, &buf);
    if (res < 0) {
	    ST_ERR("read chip id error\n");
	    return LSM6DSO_ERR_I2C;
    }
    obj->chip_id = buf;
    ST_LOG("LSM6DSO WHO_AM_I = 0x%x\n", buf);
 
    if (buf ==LSM6DO_FIXED_DEVID) {
	    sprintf(obj->name, "LSM6DSO");
        return LSM6DSO_SUCCESS;
    }else {
	   ST_ERR("not support chip id error\n");
	   return LSM6DSO_ERR_IDENTIFICATION;
    }
}

static ssize_t lsm6dso_attr_i2c_show_reg_value(struct device_driver *ddri, char *buf)
{
    struct lsm6dso_data *obj = obj_i2c_data;
    u8 reg_value;
    int res;
	
    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }

    res = lsm6dso_i2c_read_block(obj->client, obj->reg_addr, &reg_value, 0x01);
    if (res < 0) {
	res = snprintf(buf, PAGE_SIZE, "i2c read error!!\n");
        return res;
    }
	
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", reg_value); 

    return res;
}

static ssize_t lsm6dso_attr_i2c_store_reg_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6dso_data *obj = obj_i2c_data;
    u8 reg_value;
    int res;
	
    if (obj == NULL) { 
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if (1 == sscanf(buf, "0x%hhx", &reg_value)) {
	res = lsm6dso_i2c_write_block(obj->client, obj->reg_addr, &reg_value, 0x01);
	if (res < 0) {
            return LSM6DSO_ERR_I2C;
        }
    } else {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }

    return count;
}

static ssize_t lsm6dso_attr_i2c_show_reg_addr(struct device_driver *ddri, char *buf)
{
    struct lsm6dso_data *obj = obj_i2c_data;
    ssize_t res;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", obj->reg_addr); 

    return res;
}

static ssize_t lsm6dso_attr_i2c_store_reg_addr(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6dso_data *obj = obj_i2c_data;
    u8 reg_addr;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if (1 == sscanf(buf, "0x%hhx", &reg_addr)) {
        obj->reg_addr = reg_addr;
    } else {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }

    return count;
}

static DRIVER_ATTR(reg_value, S_IWUSR | S_IRUGO, lsm6dso_attr_i2c_show_reg_value, lsm6dso_attr_i2c_store_reg_value);
static DRIVER_ATTR(reg_addr,  S_IWUSR | S_IRUGO, lsm6dso_attr_i2c_show_reg_addr,  lsm6dso_attr_i2c_store_reg_addr);

static struct driver_attribute *lsm6dso_attr_i2c_list[] = {
    &driver_attr_reg_value,
    &driver_attr_reg_addr,
};

int lsm6dso_i2c_create_attr(struct device_driver *driver) 
{
    int idx, res = 0;
    int num = (int)(sizeof(lsm6dso_attr_i2c_list)/sizeof(lsm6dso_attr_i2c_list[0]));

    if (driver == NULL) {
        return -EINVAL;
    }

    for (idx = 0; idx < num; idx++) {
        if ((res = driver_create_file(driver, lsm6dso_attr_i2c_list[idx]))) {            
            ST_ERR("driver_create_file (%s) = %d\n", lsm6dso_attr_i2c_list[idx]->attr.name, res);
            break;
        }
    }

    return res;
}

int lsm6dso_i2c_delete_attr(struct device_driver *driver)
{
    int idx, res = 0;
    int num = (int)(sizeof(lsm6dso_attr_i2c_list)/sizeof(lsm6dso_attr_i2c_list[0]));

    if (driver == NULL) {
        return -EINVAL;
    }
    
    for (idx = 0; idx < num; idx++) {
        driver_remove_file(driver, lsm6dso_attr_i2c_list[idx]);
    }
    
    return res;
}

static int lsm6dso_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
    strcpy(info->type, LSM6DSO_DEV_NAME);
    return 0;
}

static int lsm6dso_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct lsm6dso_data *obj;
    int res = 0;
	
    ST_FUN();
	
    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))){
        res = -ENOMEM;
		return res;
    }

    memset(obj, 0, sizeof(struct lsm6dso_data));
    obj_i2c_data = obj;
    obj->client = client;
    i2c_set_clientdata(client, obj);

    obj->acc_enabled = 0;
    obj->gyro_enabled = 0;
    obj->step_c_enabled = 0;
    obj->step_d_enabled = 0;
    obj->significant_enabled = 0;
    obj->tilt_enabled = 0;
	
    res = lsm6dso_check_device_id(obj);
    if (res) {
        ST_ERR("check device error!\n");
	    goto exit_check_device_failed;
    }
    
    res = lsm6dso_i2c_create_attr(&lsm6dso_i2c_driver.driver);
    if (res){
        ST_ERR("create attr error!\n");
	    goto exit_create_attr_failed;
    }

#if(CONFIG_STEP_COUNTER||CONFIG_STEP_DETECT)
    lsm6dso_step_configure(obj);
#endif

#if(CONFIG_HARDWARE_INTERRUPT)
    res = lsm6dso_set_interrupt();
    if (res) {
        ST_ERR("create interrupt error!\n");
	goto exit_create_interrupt_failed;
    }
#endif	

    acc_driver_add(&lsm6dso_acc_init_info);
    gyro_driver_add(&lsm6dso_gyro_init_info);
#if (CONFIG_STEP_COUNTER || CONFIG_STEP_DETECT || CONFIG_SIGNIFICANT_MOTION)
    step_c_driver_add(&lsm6dso_pedo_init_info);
#endif
#if (CONFIG_TILT)
    tilt_driver_add(&lsm6dso_tilt_init_info);
#endif

    ST_LOG("lsm6dso_i2c_probe exit\n");
    return 0;

#if (CONFIG_HARDWARE_INTERRUPT)
exit_create_interrupt_failed:
    lsm6dso_i2c_delete_attr(&lsm6dso_i2c_driver.driver);
#endif
exit_create_attr_failed:
exit_check_device_failed:
    kfree(obj);
    return res;
}

static int lsm6dso_i2c_remove(struct i2c_client *client)
{
    lsm6dso_i2c_delete_attr(&lsm6dso_i2c_driver.driver);
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}

static const struct i2c_device_id lsm6dso_i2c_id[] = {{LSM6DSO_DEV_NAME,0},{}};

#ifdef CONFIG_OF
static const struct of_device_id lsm6dso_of_match[] = {
    {.compatible = "mediatek,gsensor"},
    {}
};
#endif

static struct i2c_driver lsm6dso_i2c_driver = {
    .driver = {
        .name           = LSM6DSO_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = lsm6dso_of_match,
#endif
    },
    .probe              = lsm6dso_i2c_probe,
    .remove             = lsm6dso_i2c_remove,
    .detect             = lsm6dso_i2c_detect,
    .id_table           = lsm6dso_i2c_id,
};

static int __init lsm6dso_module_init(void)
{
    ST_FUN();

    if (i2c_add_driver(&lsm6dso_i2c_driver)) {
        ST_ERR("add acc driver error\n");
        return -1;
    }
	
    return LSM6DSO_SUCCESS;
}

static void __exit lsm6dso_module_exit(void)
{
    ST_FUN();
    i2c_del_driver(&lsm6dso_i2c_driver);
}

module_init(lsm6dso_module_init);
module_exit(lsm6dso_module_exit);

MODULE_DESCRIPTION("STMicroelectronics lsm6dso driver");
MODULE_AUTHOR("DP.FU");
MODULE_LICENSE("GPL v2");
