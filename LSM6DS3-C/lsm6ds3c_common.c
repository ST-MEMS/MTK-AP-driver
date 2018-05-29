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

static struct i2c_driver lsm6ds3c_i2c_driver;
struct lsm6ds3c_data *obj_i2c_data = NULL;

static DEFINE_MUTEX(lsm6ds3c_i2c_mutex);
static DEFINE_MUTEX(lsm6ds3c_op_mutex);

int lsm6ds3c_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    u8 beg = addr;
    int res;
    struct i2c_msg msgs[2]={{0},{0}};
    
    mutex_lock(&lsm6ds3c_i2c_mutex);
    
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len =1;
    msgs[0].buf = &beg;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;
    
    if (!client) {
        mutex_unlock(&lsm6ds3c_i2c_mutex);
        return -EINVAL;
    } else if (len > C_I2C_FIFO_SIZE) {
        ST_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&lsm6ds3c_i2c_mutex);
        return -EINVAL;
    }

    res = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
    if (res < 0) {
        ST_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, res);
        res = -EIO;
    } else {
        res = 0;
    }

    mutex_unlock(&lsm6ds3c_i2c_mutex);

    return res; 
}

int lsm6ds3c_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   
    /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int res = 0, idx, num;
    u8 buf[C_I2C_FIFO_SIZE];
	
    mutex_lock(&lsm6ds3c_i2c_mutex);

    if (!client) {
        mutex_unlock(&lsm6ds3c_i2c_mutex);
        return -EINVAL;
    } else if (len >= C_I2C_FIFO_SIZE) {        
        ST_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&lsm6ds3c_i2c_mutex);
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
        mutex_unlock(&lsm6ds3c_i2c_mutex);
        return -EFAULT;
    } 

    mutex_unlock(&lsm6ds3c_i2c_mutex);

    return res; //if success will return transfer lenth
}

int lsm6ds3c_i2c_write_with_mask(struct i2c_client *client,u8 addr, u8 mask, u8 data)
{
    int res;
    u8 new_data = 0x00, old_data = 0x00;

    res = lsm6ds3c_i2c_read_block(client, addr, &old_data,1);
    if (res < 0)
	return res;
	
    new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));
    if (new_data == old_data) {
	return 0;
    }
	
    return lsm6ds3c_i2c_write_block(client, addr, &new_data,1);
}

#if (CONFIG_STEP_COUNTER || CONFIG_STEP_DETECT)
int lsm6ds3c_step_configure(struct lsm6ds3c_data *obj)
{
    struct i2c_client *client = obj->client;
    u8 databuf =0; 
    int res = 0;
   
    ST_FUN();    

    //enable embeded registers
    res = lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_FUNC_CFG_ACCESS, 
				         LSM6DS3C_REG_FUNC_CFG_ACCESS_MASK_FUNC_CFG_EN, LSM6DS3C_EN); 
    if (res <0) {
	ST_ERR("write LSM6DS3C_REG_FUNC_CFG_ACCESS register enable err!\n");
	return LSM6DS3C_ERR_I2C;
    }

    //4g ,and threshould is 14*32mg=448mg
    databuf = 0x8e;
    res = lsm6ds3c_i2c_write_block(client, LSM6DS3C_REG_PEDO_THS, &databuf, 0x01); 
    if (res <0) {
	ST_ERR("write lsm6ds3c step threshold err!\n");
	return LSM6DS3C_ERR_I2C;
    }

    //DEB_TIME = 0x0B(11*80ms=880ms), DEB_STEP = 0x07(7Step)
    databuf = 0x5f;
    res = lsm6ds3c_i2c_write_block(client, LSM6DS3C_REG_PEDO_DEB, &databuf, 0x01); 
    if (res < 0) {
	ST_ERR("write lsm6ds3c step debounce  err!\n");
	return LSM6DS3C_ERR_I2C;
    }

    //DIS embeded registers
    res = lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_FUNC_CFG_ACCESS, 
	                                 LSM6DS3C_REG_FUNC_CFG_ACCESS_MASK_FUNC_CFG_EN, LSM6DS3C_DIS); 
    if (res < 0) {
	ST_ERR("write LSM6DS3C_REG_FUNC_CFG_ACCESS register disable err!\n");
	return LSM6DS3C_ERR_I2C;
    }

#if (CONFIG_PEDOMETER_ALWAYS_ON) //enable pedometer 
    res = lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_CTRL1_XL, LSM6DS3C_REG_CTRL1_XL_MASK_FS_XL, 
					 LSM6DS3C_REG_CTRL1_XL_FS_4G);
    if (res < 0) {
	ST_ERR("write 4g fullscalee err!\n");
	return LSM6DS3C_ERR_I2C;
    }

    res = lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_CTRL1_XL, LSM6DS3C_REG_CTRL1_XL_MASK_ODR_XL, 
				         LSM6DS3C_REG_CTRL1_XL_ODR_26HZ);
    if (res < 0) {
	ST_ERR("write 26hz odr err!\n");
        return LSM6DS3C_ERR_I2C;
    }

    res = lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_CTRL10_C, LSM6DS3C_FUNC_ENABLE_MASK, LSM6DS3C_EN);
    if (res < 0) {
	ST_ERR("enble func err!\n");
	return LSM6DS3C_ERR_I2C;
    }

    res = lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_TAP_CFG, LSM6DS3C_STEP_ENABLE_MASK,LSM6DS3C_EN );
    if (res < 0) {
	ST_ERR("enble step err!\n");
	return LSM6DS3C_ERR_I2C;
    }  
#endif
	
    return LSM6DS3C_SUCCESS;
}
#endif

void dumpReg(struct lsm6ds3c_data *obj)
{
    struct i2c_client *client = obj->client;
    u8 addr = 0x10, regdata = 0;
    int i = 0;
	
    for (i = 0; i < 10; i++) {
        lsm6ds3c_i2c_read_block(client, addr, &regdata, 1);
        ST_LOG("Reg addr=%x regdata=%x\n", addr, regdata);
        addr++;
    }
}
#if(CONFIG_HARDWARE_INTERRUPT)
static irqreturn_t lsm6ds3c_isr(int irq, void *dev)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;

    if (atomic_read(&obj->irq_enabled)) {
	disable_irq_nosync(obj->irq);
	atomic_set(&obj->irq_enabled,0);
    }

    queue_work(obj->irq_work_queue, &obj->irq_work);
	
    return IRQ_HANDLED;
}

static void lsm6ds3c_irq_work_func(struct work_struct *work)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    struct i2c_client *client = obj->client;
    int res;
    u8 buf;

    ST_FUN();
	
    res = lsm6ds3c_i2c_read_block(client, LSM6DS3C_REG_FUNC_SRC, &buf, 0x01);
    if (res < 0) {
	goto work_exit;
    }

#if (CONFIG_SIGNIFICANT_MOTION)
    if ((LSM6DS3C_FlAG_SIGN_MOTION_MASK & buf) && obj->significant_enabled) {
	step_notify(TYPE_SIGNIFICANT);
    }
#endif

#if (CONFIG_STEP_DETECT)
    if ((LSM6DS3C_FLAG_STEP_MASK & buf) && obj->step_d_enabled) {
	step_notify(TYPE_STEP_DETECTOR);
    }
#endif

#if (CONFIG_TILT)
    if ((LSM6DS3C_FLAG_TILT_MASK & buf) && obj->tilt_enabled) {	
	tilt_notify();
    }
#endif

work_exit:
    if (!atomic_read(&obj->irq_enabled)) {
	enable_irq(obj->irq);
	atomic_set(&obj->irq_enabled,1);
    }
}

int lsm6ds3c_set_interrupt(void)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    struct i2c_client *client = obj->client;
    struct device_node *node = NULL;
    //struct pinctrl *pinctrl;
    //struct pinctrl_state *pins_default;
    //struct pinctrl_state *pins_cfg;
    u32 ints[2] = {0, 0};
    int res = -1;
	
    ST_FUN();

    /*set sensor  interrupt low trigger ,sensor default is high trigger */
    res = lsm6ds3c_i2c_write_with_mask(client, LSM6DS3C_REG_CTRL3_C, LSM6DS3C_INT_ACTIVE_MASK, LSM6DS3C_EN);
    if (res < 0) {
	ST_ERR("write interrupt high or low active  err!\n");
	return LSM6DS3C_ERR_I2C;
    }
#if 0
    /* gpio setting */
    pinctrl = devm_pinctrl_get(&stepPltFmDev->dev);
    if (IS_ERR(pinctrl)) {
	res = PTR_ERR(pinctrl);
	ST_ERR("Cannot find step pinctrl!\n");
	return res;
    }

    /*pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
    if (IS_ERR(pins_default)) {
	res = PTR_ERR(pins_default);
	ST_ERR("Cannot find step pinctrl default!\n");
    }*/

    pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
    if (IS_ERR(pins_cfg)) {
	res = PTR_ERR(pins_cfg);
	ST_ERR("Cannot find step pinctrl pin_cfg!\n");
	return res;
    }

    pinctrl_select_state(pinctrl, pins_cfg);
#endif
	
    node = of_find_compatible_node(NULL, NULL, "mediatek,gyroscope");
    if (node) {
	ST_LOG("irq node is ok!");
	of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
	gpio_set_debounce(ints[0], ints[1]);
	ST_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

	obj->irq= irq_of_parse_and_map(node, 0);
	ST_LOG("step_irq = %d\n", obj->irq);
	if (!obj->irq) {
	    ST_ERR("irq_of_parse_and_map fail!!\n");
	    return -EINVAL;
	}
		
	INIT_WORK(&obj->irq_work, lsm6ds3c_irq_work_func);
	obj->irq_work_queue =create_singlethread_workqueue("lsm6ds3c_step_wq");
	if (!obj->irq_work_queue) {
	    res = -ENOMEM;
	    ST_ERR("cannot create work queue1");
	    return res;
	}
		
	if (request_irq(obj->irq, lsm6ds3c_isr, IRQ_TYPE_LEVEL_LOW, "gyroscope", NULL)) {		
	    ST_ERR("IRQ LINE NOT AVAILABLE!!\n");
	    return -EINVAL;
	}
		
	disable_irq_nosync(obj->irq);
	atomic_set(&obj->irq_enabled,0); 
    } else {
	ST_ERR("null irq node!!\n");
	return -EINVAL;
    }
	 
    return LSM6DS3C_SUCCESS;
}
#endif

static int lsm6ds3c_read_chip_id(struct lsm6ds3c_data *obj, u8 *data)
{
    struct i2c_client *client = obj->client;
    int res;
	
    res = lsm6ds3c_i2c_read_block(client, LSM6DS3C_REG_WHO_AM_I, data, 0x01);
    if (res < 0){
	return LSM6DS3C_ERR_I2C;
    }

    return LSM6DS3C_SUCCESS;
}

static int lsm6ds3c_check_device_id(struct lsm6ds3c_data *obj)
{
    u8 buf;
    int res;

    res = lsm6ds3c_read_chip_id(obj, &buf);
    if (res < 0){
	ST_ERR("read chip id error\n");
	return LSM6DS3C_ERR_I2C;
    }

    obj->chip_id = buf;
    ST_LOG("LSM6DS3C who am I = 0x%x\n", buf);
 
    if (buf ==LSM6DS3C_FIXED_DEVID_LSM6DS3) {
	sprintf(obj->name, "LSM6DS3");
	return LSM6DS3C_SUCCESS;
    } else if (buf ==LSM6DS3C_FIXED_DEVID_LSM6DS3C) {
	sprintf(obj->name, "LSM6DS3C");
	return LSM6DS3C_SUCCESS;
    } else if (buf ==LSM6DS3C_FIXED_DEVID_LSM6DSM) {
	sprintf(obj->name, "LSM6DSM");
	return LSM6DS3C_SUCCESS;
    } else {
	ST_ERR("not support chip id error\n");
	return LSM6DS3C_ERR_IDENTIFICATION;
    }
}

static ssize_t lsm6ds3c_attr_i2c_show_reg_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    u8 reg_value;
    int res;
	
    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }

    res = lsm6ds3c_i2c_read_block(obj->client, obj->reg_addr, &reg_value, 0x01);
    if (res < 0) {
	res = snprintf(buf, PAGE_SIZE, "i2c read error!!\n");
        return res;
    }
	
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", reg_value); 

    return res;
}

static ssize_t lsm6ds3c_attr_i2c_store_reg_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    u8 reg_value;
    int res;
	
    if (obj == NULL) { 
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if (1 == sscanf(buf, "0x%hhx", &reg_value)) {
	res = lsm6ds3c_i2c_write_block(obj->client, obj->reg_addr, &reg_value, 0x01);
	if (res < 0) {
            return LSM6DS3C_ERR_I2C;
        }
    } else {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }

    return count;
}

static ssize_t lsm6ds3c_attr_i2c_show_reg_addr(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
    ssize_t res;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", obj->reg_addr); 

    return res;
}

static ssize_t lsm6ds3c_attr_i2c_store_reg_addr(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3c_data *obj = obj_i2c_data;
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

static DRIVER_ATTR(reg_value, S_IWUSR | S_IRUGO, lsm6ds3c_attr_i2c_show_reg_value, lsm6ds3c_attr_i2c_store_reg_value);
static DRIVER_ATTR(reg_addr,  S_IWUSR | S_IRUGO, lsm6ds3c_attr_i2c_show_reg_addr,  lsm6ds3c_attr_i2c_store_reg_addr);

static struct driver_attribute *lsm6ds3c_attr_i2c_list[] = {
    &driver_attr_reg_value,
    &driver_attr_reg_addr,
};

int lsm6ds3c_i2c_create_attr(struct device_driver *driver) 
{
    int idx, res = 0;
    int num = (int)(sizeof(lsm6ds3c_attr_i2c_list)/sizeof(lsm6ds3c_attr_i2c_list[0]));

    if (driver == NULL) {
        return -EINVAL;
    }

    for (idx = 0; idx < num; idx++) {
        if ((res = driver_create_file(driver, lsm6ds3c_attr_i2c_list[idx]))) {            
            ST_ERR("driver_create_file (%s) = %d\n", lsm6ds3c_attr_i2c_list[idx]->attr.name, res);
            break;
        }
    }

    return res;
}

int lsm6ds3c_i2c_delete_attr(struct device_driver *driver)
{
    int idx, res = 0;
    int num = (int)(sizeof(lsm6ds3c_attr_i2c_list)/sizeof(lsm6ds3c_attr_i2c_list[0]));

    if (driver == NULL) {
        return -EINVAL;
    }
    
    for (idx = 0; idx < num; idx++) {
        driver_remove_file(driver, lsm6ds3c_attr_i2c_list[idx]);
    }
    
    return res;
}

#if 0
static int lsm6ds3c_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct lsm6ds3c_data *obj = i2c_get_clientdata(client);
    struct lsm6ds3c_acc *acc_obj = &obj->lsm6ds3c_acc_data;
    struct lsm6ds3c_gyro *gyro_obj = &obj->lsm6ds3c_gyro_data;
    int res = 0;

    ST_FUN();

    if (obj == NULL) {
        ST_ERR("obj: null pointer!!\n");
        return -EINVAL;
    }

    if (msg.event == PM_EVENT_SUSPEND) {
        if (obj->acc_enabled == 1) {
            if (acc_obj == NULL) {
                ST_ERR("acc_obj: null pointer!!\n");
                return -EINVAL;
            }

            mutex_lock(&lsm6ds3c_op_mutex);
            res = lsm6ds3c_acc_set_power_mode(acc_obj, false);
            if (res) {
                ST_ERR("acc: write power control fail!!\n");
                mutex_unlock(&lsm6ds3c_op_mutex);
                return res;
            }
            mutex_unlock(&lsm6ds3c_op_mutex);

            atomic_set(&acc_obj->suspend, 1);
        }

        if (obj->gyro_enabled == 1) {
            if (gyro_obj == NULL) {
                ST_ERR("gyro_obj: null pointer!!\n");
                return -EINVAL;
            }

            mutex_lock(&lsm6ds3c_op_mutex);
            res = lsm6ds3c_gyro_set_power_mode(gyro_obj, false);
            if (res) {
                ST_ERR("gyro: write power control fail!!\n");
                mutex_unlock(&lsm6ds3c_op_mutex);
                return res;
            }
            mutex_unlock(&lsm6ds3c_op_mutex);

            atomic_set(&gyro_obj->suspend, 1);
        }
    }

#if (CONFIG_HARDWARE_INTERRUPT)
	if (atomic_read(&obj->irq_enabled)) {  
		disable_irq_nosync(obj->irq);	
	}
#endif

    ST_LOG("lsm6ds3c i2c suspended\n");
    return res;
}

static int lsm6ds3c_resume(struct i2c_client *client)
{
    struct lsm6ds3c_data *obj = i2c_get_clientdata(client);
    struct lsm6ds3c_acc *acc_obj = &obj->lsm6ds3c_acc_data;
    struct lsm6ds3c_gyro *gyro_obj = &obj->lsm6ds3c_gyro_data;
    int res;

    ST_FUN();

    if (obj == NULL) {
        ST_ERR("obj: null pointer!!\n");
        return -EINVAL;
    }
    if (obj->acc_enabled == 1) {
        if (acc_obj == NULL) {
            ST_ERR("acc_obj: null pointer!!\n");
            return -EINVAL;
        }

        mutex_lock(&lsm6ds3c_op_mutex);
        res = lsm6ds3c_acc_set_power_mode(acc_obj, true);
        if (res) {
            mutex_unlock(&lsm6ds3c_op_mutex);
            ST_ERR("acc: initialize client fail!!\n");
            return res;
        }
        mutex_unlock(&lsm6ds3c_op_mutex);

        atomic_set(&acc_obj->suspend, 0);
    }

    if (obj->gyro_enabled == 1) {
        if (gyro_obj == NULL) {
            ST_ERR("gyro_obj: null pointer!!\n");
            return -EINVAL;
        }

        mutex_lock(&lsm6ds3c_op_mutex);
        res = lsm6ds3c_gyro_set_power_mode(gyro_obj, true);
        if (res) {
            mutex_unlock(&lsm6ds3c_op_mutex);
            ST_ERR("gyro: initialize client fail!!\n");
            return res;
        }
        mutex_unlock(&lsm6ds3c_op_mutex);

        atomic_set(&gyro_obj->suspend, 0);
    }

#if (CONFIG_HARDWARE_INTERRUPT)
	if (atomic_read(&obj->irq_enabled)) {  
		enable_irq(obj->irq);;	
	}
#endif

    ST_LOG("lsm6ds3c i2c resumed\n");

    return 0;
}
#endif

static int lsm6ds3c_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
    strcpy(info->type, LSM6DS3C_DEV_NAME);
    return 0;
}

static int lsm6ds3c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct lsm6ds3c_data *obj;
    int res = 0;
	
    ST_FUN();
	
    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))){
        res = -ENOMEM;
	return res;
    }

    memset(obj, 0, sizeof(struct lsm6ds3c_data));
    obj_i2c_data = obj;
    obj->client = client;
    i2c_set_clientdata(client, obj);

    obj->acc_enabled = 0;
    obj->gyro_enabled = 0;
    obj->step_c_enabled = 0;
    obj->step_d_enabled = 0;
    obj->significant_enabled = 0;
    obj->tilt_enabled = 0;
	
    res = lsm6ds3c_check_device_id(obj);
    if (res) {
        ST_ERR("check device error!\n");
	goto exit_check_device_failed;
    }

    res = lsm6ds3c_i2c_create_attr(&lsm6ds3c_i2c_driver.driver);
    if (res){
        ST_ERR("create attr error!\n");
	goto exit_create_attr_failed;
    }

#if(CONFIG_STEP_COUNTER||CONFIG_STEP_DETECT)
    lsm6ds3c_step_configure(obj);
#endif

#if(CONFIG_HARDWARE_INTERRUPT)
    //platform_driver_register(&lsm6ds3c_step_driver);
    res = lsm6ds3c_set_interrupt();
    if (res) {
        ST_ERR("create interrupt error!\n");
	goto exit_create_interrupt_failed;
    }
#endif	

	ST_LOG("lsm6ds3c_i2c_probe exit\n");
	return 0;

#if (CONFIG_HARDWARE_INTERRUPT)
exit_create_interrupt_failed:
	lsm6ds3c_i2c_delete_attr(&lsm6ds3c_i2c_driver.driver);
#endif
exit_create_attr_failed:
exit_check_device_failed:
    kfree(obj);
    return res;

}

static int lsm6ds3c_i2c_remove(struct i2c_client *client)
{
    lsm6ds3c_i2c_delete_attr(&lsm6ds3c_i2c_driver.driver);
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}

static const struct i2c_device_id lsm6ds3c_i2c_id[] = {{LSM6DS3C_DEV_NAME,0},{}};

#ifdef CONFIG_OF
static const struct of_device_id lsm6ds3c_of_match[] = {
    {.compatible = "mediatek,gsensor"},
    {}
};
#endif

static struct i2c_driver lsm6ds3c_i2c_driver = {
    .driver = {
        .name           = LSM6DS3C_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = lsm6ds3c_of_match,
#endif
    },
    .probe              = lsm6ds3c_i2c_probe,
    .remove             = lsm6ds3c_i2c_remove,
    .detect             = lsm6ds3c_i2c_detect,
#if 0
    .suspend            = lsm6ds3c_suspend,
    .resume             = lsm6ds3c_resume,
#endif
    .id_table           = lsm6ds3c_i2c_id,
};

static int __init lsm6ds3c_module_init(void)
{
    ST_FUN();

    if (i2c_add_driver(&lsm6ds3c_i2c_driver)) {
        ST_ERR("add acc driver error\n");
        return -1;
    }

    acc_driver_add(&lsm6ds3c_acc_init_info);
    gyro_driver_add(&lsm6ds3c_gyro_init_info);
#if (CONFIG_STEP_COUNTER || CONFIG_STEP_DETECT || CONFIG_SIGNIFICANT_MOTION)
    step_c_driver_add(&lsm6ds3c_pedo_init_info);
#endif
#if (CONFIG_TILT)
     tilt_driver_add(&lsm6ds3c_tilt_init_info);
#endif
    return LSM6DS3C_SUCCESS;
}

static void __exit lsm6ds3c_module_exit(void)
{
    ST_FUN();
    i2c_del_driver(&lsm6ds3c_i2c_driver);
}

module_init(lsm6ds3c_module_init);
module_exit(lsm6ds3c_module_exit);

MODULE_DESCRIPTION("STMicroelectronics lsm6ds3 driver");
MODULE_AUTHOR("Ian Yang, William Zeng");
MODULE_LICENSE("GPL v2");
