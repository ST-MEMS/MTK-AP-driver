
#include "lis2ds12.h"

static struct i2c_driver lis2ds12_i2c_driver;
struct lis2ds12_data *obj_i2c_data = NULL;

static DEFINE_MUTEX(lis2ds12_i2c_mutex);
static DEFINE_MUTEX(lis2ds12_op_mutex);

/*--------------------read function----------------------------------*/
int lis2ds12_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    u8 beg = addr;
    int ret;
    struct i2c_msg msgs[2] = {{0},{0}};
    
    mutex_lock(&lis2ds12_i2c_mutex);
    
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len =1;
    msgs[0].buf = &beg;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;
    
    if (!client) {
        mutex_unlock(&lis2ds12_i2c_mutex);
        return -EINVAL;
    } else if (len > C_I2C_FIFO_SIZE) {
        ST_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&lis2ds12_i2c_mutex);
        return -EINVAL;
    }
	
    ret = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
    if (ret < 0) {
        ST_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, ret);
        ret = -EIO;
    } else {
        ret = 0;
    }

    mutex_unlock(&lis2ds12_i2c_mutex);
    return ret; 
}

int lis2ds12_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   
    /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int ret = 0, idx, num;
    u8 buf[C_I2C_FIFO_SIZE];
	
    mutex_lock(&lis2ds12_i2c_mutex);
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
    mutex_unlock(&lis2ds12_i2c_mutex);
    return ret; //if success will return transfer lenth
}

int lis2ds12_i2c_write_with_mask(struct i2c_client *client, u8 addr, u8 mask, u8 data)
{
    int ret;
    u8 new_data = 0x00, old_data = 0x00;

    ret = lis2ds12_i2c_read_block(client, addr, &old_data, 1);
    if (ret < 0)
	return ret;
	
    new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));
    if (new_data == old_data)
	return 0;
	
    return lis2ds12_i2c_write_block(client, addr, &new_data, 1);
}

#if (CONFIG_STEP_COUNTER || CONFIG_STEP_DETECT)
int lis2ds12_step_configure(struct lis2ds12_data *obj)
{
    struct i2c_client *client = obj->client;
    u8 databuf = 0; 
    int ret = 0;
	
    ST_FUN();    

    //set minimum threshould as 14 * 32mg = 448mg, if FS = 4g
    databuf = 0x4e;
    ret = lis2ds12_i2c_write_block(client, LIS2DS12_REG_STEP_COUNTER_MINTHS, &databuf, 1);
    if (ret < 0) {
	ST_ERR("write LIS2DS12 step threshold err!\n");
	return LIS2DS12_ERR_I2C;
    }
    
    lis2ds12_i2c_read_block(client, LIS2DS12_REG_CTRL2, &databuf, 1);
    ST_ERR("LIS2DS12_REG_CTRL2:0x%02x\n", databuf);

    //enable embedded registers
    ret = lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_CTRL2, LIS2DS12_REG_CTRL2_MASK_FUNC_CFG_EN, LIS2DS12_EN); 
    if (ret < 0) {
	ST_ERR("write LIS2DS12_REG_CTRL2 register enable err!\n");
	return LIS2DS12_ERR_I2C;
    } 

    //set debounce time and step DEB_TIME = 0x0B (11*80ms=880ms), DEB_STEP = 0x07 (7Step)
    databuf = 0x5f;
    ret = lis2ds12_i2c_write_block(client, LIS2DS12_REG_PEDO_DEB, &databuf, 1); 
    if (ret < 0) {
	ST_ERR("write LIS2DS12 step debounce err!\n");
	return LIS2DS12_ERR_I2C;
    }

    //disable embedded registers
    ret = lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_EMBED_CTRL2, LIS2DS12_REG_CTRL2_MASK_FUNC_CFG_EN, LIS2DS12_DIS); 
    if (ret < 0) {
	ST_ERR("write LIS2DS12_REG_EMBED_CTRL2 register disable err!\n");
	return LIS2DS12_ERR_I2C;
    }

    lis2ds12_i2c_read_block(client, LIS2DS12_REG_CTRL2, &databuf, 1);
    ST_ERR("LIS2DS12_REG_CTRL2:0x%02x\n", databuf);

#if (CONFIG_PEDOMETER_ALWAYS_ON) 
    //set fullscale as 4g
    ret = lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_CTRL1, LIS2DS12_REG_CTRL1_MASK_FS, LIS2DS12_REG_CTRL1_FS_4G);
    if (ret < 0) {
	ST_ERR("write 4g fullscalee err!\n");
	return LIS2DS12_ERR_I2C;
    }

	//set ODR as 25Hz
    ret = lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_CTRL1, LIS2DS12_REG_CTRL1_MASK_ODR, LIS2DS12_REG_CTRL1_ODR_25HZ);
    if (ret < 0) {
	ST_ERR("write 26hz odr err!\n");
        return LIS2DS12_ERR_I2C;
    }

    //enable step counter
    ret = lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_FUNC_CTRL, LIS2DS12_STEP_ENABLE_MASK, LIS2DS12_EN);
    if (ret < 0) {
	ST_ERR("enble step err!\n");
	return LIS2DS12_ERR_I2C;
    }  
#endif

    return LIS2DS12_SUCCESS;
}
#endif

void dumpReg(struct lis2ds12_data *obj)
{
    struct i2c_client *client = obj->client;
    u8 addr = 0x20, regdata = 0;
    int i = 0;
	
    for (i = 0; i < 32; i++) {
        lis2ds12_i2c_read_block(client, addr, &regdata, 1);
        ST_LOG("Reg addr=0x%02x regdata=0x%02x\n", addr, regdata);
        addr++;
    }
}

static void lis2ds12_chip_power(struct lis2ds12_data *obj, unsigned int on) 
{
    return;
}

#if(CONFIG_HARDWARE_INTERRUPT)
struct platform_device *stepPltFmDev;

static int lis2ds12_probe(struct platform_device *pdev) 
{
    stepPltFmDev = pdev;
    return LIS2DS12_SUCCESS;
}

static int lis2ds12_remove(struct platform_device *pdev)
{
    return LIS2DS12_SUCCESS;
}

#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
    { .compatible = "mediatek,st_step_counter", },
    {},
};
#endif

static struct platform_driver lis2ds12_step_driver = {
    .probe      = lis2ds12_probe,
    .remove     = lis2ds12_remove,    
    .driver     = {
	.name  = "stepcounter",
//	.owner	= THIS_MODULE,
#ifdef CONFIG_OF
	.of_match_table = gsensor_of_match,
#endif
    }
};

static irqreturn_t lis2ds12_isr(int irq, void *dev)
{
    struct lis2ds12_data *obj = obj_i2c_data;

    if (atomic_read(&obj->irq_enabled)) {
	disable_irq_nosync(obj->irq);
	atomic_set(&obj->irq_enabled,0);
    }
	
    queue_work(obj->irq_work_queue, &obj->irq_work);
	
    return IRQ_HANDLED;
}

static void lis2ds12_irq_work_func(struct work_struct *work)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct i2c_client *client = obj->client;
    int ret;
    u8 buf;
	
    ST_FUN();
	
    ret = lis2ds12_i2c_read_block(client, LIS2DS12_REG_FUNC_CK_GATE, &buf, 0x01);
    if (ret < 0)
    	goto work_exit;

#if (CONFIG_SIGNIFICANT_MOTION)
    if ((LIS2DS12_FlAG_SIGN_MOTION_MASK & buf) && obj->significant_enabled) {
	step_notify(TYPE_SIGNIFICANT);
    }
#endif

#if (CONFIG_STEP_DETECT)
    if ((LIS2DS12_FLAG_STEP_MASK & buf) && obj->step_d_enabled) {
	step_notify(TYPE_STEP_DETECTOR);
    }
#endif

#if (CONFIG_TILT)
    if ((LIS2DS12_FLAG_TILT_MASK & buf) && obj->tilt_enabled) {
	tilt_notify();
    }
#endif

work_exit:
    if (!atomic_read(&obj->irq_enabled)) {
	enable_irq(obj->irq);
	atomic_set(&obj->irq_enabled,1);
    }
}

int lis2ds12_set_interrupt(void)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    struct i2c_client *client = obj->client;
    struct device_node *node = NULL;
    struct pinctrl *pinctrl;
    //struct pinctrl_state *pins_default;
    struct pinctrl_state *pins_cfg;
    u32 ints[2] = {0, 0};
    int ret = -1;
	
    ST_FUN();

    /* set sensor interrupt low trigger, sensor default is high trigger */
    ret = lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_CTRL3, LIS2DS12_INT_ACTIVE_MASK, LIS2DS12_EN);
    if (ret < 0) {
	ST_ERR("write interrupt high or low active err!\n");
	return LIS2DS12_ERR_I2C;
    }

    //INT2 also routed on INT1
    ret = lis2ds12_i2c_write_with_mask(client, LIS2DS12_REG_CTRL5, LIS2DS12_INT2_ON_INT1_MASK, LIS2DS12_EN);
    if (ret < 0) {
	ST_ERR("write LIS2DS12_REG_CTRL5 INT2 routed on INT1 err!\n");
	return LIS2DS12_ERR_I2C;
    } 

	
    /* gpio setting */
    pinctrl = devm_pinctrl_get(&stepPltFmDev->dev);
    if (IS_ERR(pinctrl)) {
	ret = PTR_ERR(pinctrl);
	ST_ERR("Cannot find step pinctrl!\n");
	return ret;
    }
/*
    pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
    if (IS_ERR(pins_default)) {
	ret = PTR_ERR(pins_default);
	ST_ERR("Cannot find step pinctrl default!\n");
    }
*/
    pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
    if (IS_ERR(pins_cfg)) {
	ret = PTR_ERR(pins_cfg);
	ST_ERR("Cannot find step pinctrl pin_cfg!\n");
	return ret;
    }
	
    pinctrl_select_state(pinctrl, pins_cfg);
	
    node = of_find_compatible_node(NULL, NULL, "mediatek,gyroscope");
    if (node) {
	ST_LOG("irq node is ok!");
	of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
	gpio_set_debounce(ints[0], ints[1]);
	ST_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

	obj->irq = irq_of_parse_and_map(node, 0);
	ST_LOG("step_irq = %d\n", obj->irq);
	if (!obj->irq) {
	    ST_ERR("irq_of_parse_and_map fail!!\n");
	    return -EINVAL;
	}
		
	INIT_WORK(&obj->irq_work, lis2ds12_irq_work_func);
	obj->irq_work_queue = create_singlethread_workqueue("lis2ds12_step_wq");
	if (!obj->irq_work_queue) {
	    ST_ERR("cannot create work queue1");
	    return -ENOMEM;
	}
		
	if (request_irq(obj->irq, lis2ds12_isr, IRQ_TYPE_LEVEL_LOW, "gyroscope", NULL)) {		
	    ST_ERR("IRQ LINE NOT AVAILABLE!!\n");
	    return -EINVAL;
	}
		
	disable_irq_nosync(obj->irq);
	atomic_set(&obj->irq_enabled, 0); 
    } else {
	ST_ERR("null irq node!!\n");
	return -EINVAL;
    }
	
    return LIS2DS12_SUCCESS;
}
#endif

static int lis2ds12_read_chip_id(struct lis2ds12_data *obj, u8 *data)
{
    struct i2c_client *client = obj->client;
    int ret;
	
    ret = lis2ds12_i2c_read_block(client, LIS2DS12_REG_WHO_AM_I, data, 0x01);
    if (ret < 0)
	return LIS2DS12_ERR_I2C;
	
    return LIS2DS12_SUCCESS;
}

static int lis2ds12_check_device_id(struct lis2ds12_data *obj)
{
    u8 buf;
    int ret;

    ret = lis2ds12_read_chip_id(obj, &buf);
    if (ret < 0) {
	ST_ERR("read chip id error\n");
	return LIS2DS12_ERR_I2C;
    }

    obj->chip_id = buf;
    ST_LOG("LIS2DS12 who am I = 0x%x\n", buf);

    if (buf == LIS2DS12_FIXED_DEVID_LIS2DS12) {
	sprintf(obj->name, "LIS2DS12");
	return LIS2DS12_SUCCESS;
    } else {
	ST_ERR("not support chip id error\n");
	return LIS2DS12_ERR_IDENTIFICATION;
    }
}

static ssize_t lis2ds12_attr_i2c_show_reg_value(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    u8 reg_value;
    int ret;
	
    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
	
    ret = lis2ds12_i2c_read_block(obj->client, obj->reg_addr, &reg_value, 0x01);
    if (ret < 0) {
	ret = snprintf(buf, PAGE_SIZE, "i2c read error!!\n");
        return ret;
    }
	
    ret = snprintf(buf, PAGE_SIZE, "0x%04X\n", reg_value); 
    return ret;
}

static ssize_t lis2ds12_attr_i2c_store_reg_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    u8 reg_value;
	int ret;
	
    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if (1 == sscanf(buf, "0x%hhx", &reg_value)) {
	ret = lis2ds12_i2c_write_block(obj->client, obj->reg_addr, &reg_value, 0x01);
	if (ret < 0)
            return LIS2DS12_ERR_I2C;
    } else {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }

    return count;
}

static ssize_t lis2ds12_attr_i2c_show_reg_addr(struct device_driver *ddri, char *buf)
{
    struct lis2ds12_data *obj = obj_i2c_data;
    ssize_t ret;

    if (obj == NULL) {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    ret = snprintf(buf, PAGE_SIZE, "0x%04X\n", obj->reg_addr); 
    return ret;
}

static ssize_t lis2ds12_attr_i2c_store_reg_addr(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lis2ds12_data *obj = obj_i2c_data;
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

static DRIVER_ATTR(reg_value, S_IWUSR | S_IRUGO, lis2ds12_attr_i2c_show_reg_value, lis2ds12_attr_i2c_store_reg_value);
static DRIVER_ATTR(reg_addr,  S_IWUSR | S_IRUGO, lis2ds12_attr_i2c_show_reg_addr,  lis2ds12_attr_i2c_store_reg_addr);

static struct driver_attribute *lis2ds12_attr_i2c_list[] = {
    &driver_attr_reg_value,
    &driver_attr_reg_addr,
};

int lis2ds12_i2c_create_attr(struct device_driver *driver) 
{
    int idx, ret = 0;
    int num = (int)(sizeof(lis2ds12_attr_i2c_list)/sizeof(lis2ds12_attr_i2c_list[0]));
	
    if (driver == NULL)
        return -EINVAL;

    for (idx = 0; idx < num; idx++) {
        if ((ret = driver_create_file(driver, lis2ds12_attr_i2c_list[idx]))) {           
            ST_ERR("driver_create_file (%s) = %d\n", lis2ds12_attr_i2c_list[idx]->attr.name, ret);
            break;
        }
    }
	
    return ret;
}

int lis2ds12_i2c_delete_attr(struct device_driver *driver)
{
    int idx;
    int num = (int)(sizeof(lis2ds12_attr_i2c_list)/sizeof(lis2ds12_attr_i2c_list[0]));

    if (driver == NULL)
        return -EINVAL;
    
    for (idx = 0; idx < num; idx++)
        driver_remove_file(driver, lis2ds12_attr_i2c_list[idx]);
    
    return 0;
}

/*----------------------------------------------------------------------------*/
static int lis2ds12_suspend(struct i2c_client *client, pm_message_t msg) 
{
    struct lis2ds12_data *obj = i2c_get_clientdata(client);
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;
    int ret = 0;
	
    ST_FUN();
	
    if ((msg.event == PM_EVENT_SUSPEND) && (obj->acc_enabled == 1)) {   
        mutex_lock(&lis2ds12_op_mutex);
        if (obj == NULL) {    
            ST_ERR("null pointer!!\n");
            ret = -EINVAL;
	    goto exit_failed;
        }

        ret = lis2ds12_acc_set_power_mode(acc_obj, false);		
        if (ret) {
            ST_ERR("write power control fail!!\n");
            goto exit_failed;        
        }
        
        atomic_set(&acc_obj->suspend, 1);
	mutex_unlock(&lis2ds12_op_mutex);
        lis2ds12_chip_power(obj, 0);
    }

#if (CONFIG_HARDWARE_INTERRUPT)
    if (atomic_read(&obj->irq_enabled)) 
	disable_irq_nosync(obj->irq);
#endif
	
    ST_LOG("lis2ds12 i2c suspended\n");
    return ret;
	
exit_failed:
    mutex_unlock(&lis2ds12_op_mutex);
    return ret; 
}

static int lis2ds12_resume(struct i2c_client *client)
{
    struct lis2ds12_data *obj = i2c_get_clientdata(client);
    struct lis2ds12_acc *acc_obj = &obj->lis2ds12_acc_data;
    int ret;

    ST_FUN();
    lis2ds12_chip_power(obj, 1);

    if (obj->acc_enabled == 1) {
	mutex_lock(&lis2ds12_op_mutex);
	if (obj == NULL) {
	    ST_ERR("null pointer!!\n");
	    ret = -EINVAL;
	    goto exit_failed;
	}

    	ret = lis2ds12_acc_set_power_mode(acc_obj, true);
    	if (ret) {
	    ST_ERR("initialize client fail!!\n");
	    goto exit_failed;        
   	}

     	atomic_set(&acc_obj->suspend, 0);
	mutex_unlock(&lis2ds12_op_mutex);
    }

#if (CONFIG_HARDWARE_INTERRUPT)
    if (atomic_read(&obj->irq_enabled)) 
	enable_irq(obj->irq);
#endif	
	
    ST_LOG("lis2ds12 i2c resumed\n");
    return 0;

exit_failed:
    mutex_unlock(&lis2ds12_op_mutex);
    return ret; 
}

static int lis2ds12_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
    strcpy(info->type, LIS2DS12_DEV_NAME);
    return 0;
}

static int lis2ds12_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct lis2ds12_data *obj;
    int ret = 0;

    ST_FUN();

    if (!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))) {
	ret = -ENOMEM;
	return ret;
    }

    memset(obj, 0, sizeof(struct lis2ds12_data));
    obj_i2c_data = obj;
    obj->client = client;
    i2c_set_clientdata(client, obj);

    obj->acc_enabled = 0;
    obj->step_c_enabled = 0;
    obj->step_d_enabled = 0;
    obj->significant_enabled = 0;
    obj->tilt_enabled = 0;

    ret = lis2ds12_check_device_id(obj);
    if (ret) {
	ST_ERR("check device error!\n");
	goto exit_check_device_failed;
    }

    ret = lis2ds12_i2c_create_attr(&lis2ds12_i2c_driver.driver);
    if (ret) {
	ST_ERR("create attr error!\n");
	goto exit_check_device_failed;
    }

#if (CONFIG_STEP_COUNTER || CONFIG_STEP_DETECT)
    lis2ds12_step_configure(obj);
#endif

#if (CONFIG_HARDWARE_INTERRUPT)
    platform_driver_register(&lis2ds12_step_driver);
    ret = lis2ds12_set_interrupt();
    if (ret) {
	ST_ERR("create interrupt error!\n");
	goto exit_set_interrupt_failed;
    }
#endif	

    acc_driver_add(&lis2ds12_acc_init_info);
#if (CONFIG_STEP_COUNTER || CONFIG_STEP_DETECT || CONFIG_SIGNIFICANT_MOTION)
    step_c_driver_add(&lis2ds12_pedo_init_info);
#endif
#if (CONFIG_TILT)
    tilt_driver_add(&lis2ds12_tilt_init_info);
#endif

    ST_LOG("lis2ds12_i2c_probe successfully\n");
    return 0;

#if (CONFIG_HARDWARE_INTERRUPT)
exit_set_interrupt_failed:	
    lis2ds12_i2c_delete_attr(&lis2ds12_i2c_driver.driver);
#endif	
exit_check_device_failed:
    kfree(obj);
    return ret;
}

static int lis2ds12_i2c_remove(struct i2c_client *client)
{
    lis2ds12_i2c_delete_attr(&lis2ds12_i2c_driver.driver);
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}

static const struct i2c_device_id lis2ds12_i2c_id[] = {{LIS2DS12_DEV_NAME,0},{}};

#ifdef CONFIG_OF
static const struct of_device_id lis2ds12_of_match[] = {
    {.compatible = "mediatek,gsensor"},
    {}
};
#endif

static struct i2c_driver lis2ds12_i2c_driver = {
    .driver = {
        .name           = LIS2DS12_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = lis2ds12_of_match,
#endif
    },
    .probe              = lis2ds12_i2c_probe,
    .remove             = lis2ds12_i2c_remove,
    .detect             = lis2ds12_i2c_detect,
    .suspend            = lis2ds12_suspend,
    .resume             = lis2ds12_resume,
    .id_table           = lis2ds12_i2c_id,
};

/*----------------------------------------------------------------------------*/
static int __init lis2ds12_module_init(void)
{
    ST_FUN();
	
    if (i2c_add_driver(&lis2ds12_i2c_driver)) {
        ST_ERR("add acc driver error\n");
        return -1;
    }
	
    return LIS2DS12_SUCCESS;
}

static void __exit lis2ds12_module_exit(void)
{
    ST_FUN();
    i2c_del_driver(&lis2ds12_i2c_driver);
}

/*----------------------------------------------------------------------------*/
module_init(lis2ds12_module_init);
module_exit(lis2ds12_module_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LIS2DS12 I2C driver");
