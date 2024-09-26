/*
 _  _  _              _           
(_)| || | _   _  ___ | | __ _   _ 
| || || || | | |/ __|| |/ /| | | |
| || || || |_| |\__ \|   < | |_| |
|_||_||_| \__, ||___/|_|\_\ \__, |
          |___/             |___/

Driver for Vishay vcnl4010 v1.0.0
by Code Crafter
Copyright © 2024 illysky
*/  

#define DT_DRV_COMPAT vishay_vcnl4010

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include "vcnl4010.h"


LOG_MODULE_REGISTER(vcnl4010, CONFIG_SENSOR_LOG_LEVEL);

// ##################################################################
// Config Structure 
// ##################################################################
struct vcnl4010_config 
{
	// Add more elements to config stuct as need, eg gpios for irqs
	// Remember to update instantiation if you need
	// them initialised at boot
	struct i2c_dt_spec i2c;
	int prox_rate; 
	int als_rate; 
	int iled; 
	struct gpio_dt_spec sel_gpio; 
	int id;
};


// ##################################################################
// Data Structure 
// ##################################################################
struct vcnl4010_data
{
	// Add more elements to the driver data structure that need to 
	// persist at runtime. Remove if not needed
	bool ready;
	
};

// ##################################################################
//
// ##################################################################
static inline int  vcnl4010_i2c_write (const struct device *dev, uint8_t addr, uint8_t size, uint8_t* buf)
{
	const struct vcnl4010_config *cfg = dev->config;


	if (cfg->id != 0xFF)
	{
		gpio_pin_set_dt(&cfg->sel_gpio, cfg->id); 
		k_msleep(1); 
	}

	__ASSERT(!i2c_burst_write_dt (&cfg->i2c, addr, buf, size), "device not responding");
	return 0; 
}

// ##################################################################
// 
// ##################################################################
static inline int vcnl4010_i2c_read (const struct device *dev, uint8_t addr, uint8_t size, uint8_t* buf)
{
	const struct vcnl4010_config *cfg = dev->config;
	
	if (cfg->id != 0xFF)
	{
		gpio_pin_set_dt(&cfg->sel_gpio, cfg->id); 
		k_msleep(1); 
	}
	
	__ASSERT(!i2c_burst_read_dt (&cfg->i2c, addr, buf, size), "device not responding");
	return 0; 
}

// ##################################################################
// Driver API Implementation Stubs
// ##################################################################
// Visit https://docs.zephyrproject.org/latest/develop/api/overview.html 
// to get the API calls you need to implement for this type of driver
// eg. sensor or charger. 
// Update the _driver_api struct accordingly 

// ##################################################################
// static int vcnl4010_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler)
// ##################################################################
static int vcnl4010_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler)
{
	return 0;
}

// ##################################################################
// static int vcnl4010_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
// ##################################################################
static int vcnl4010_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
{
	return 0;
}

// ##################################################################
// static int vcnl4010_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
// ##################################################################
static int vcnl4010_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	uint8_t buf = 0; 
	switch (chan)
    {
        case SENSOR_CHAN_LIGHT:
        {
			// Check if we are ready?
			vcnl4010_i2c_read (dev, VCNL4010_COMMAND_REGISTER_ADDR, VCNL4010_COMMAND_REGISTER_SIZE, &buf); 
			if (VCNL4010_COMMAND_REGISTER_ALS_DATA_RDY_GET(buf) == 0) return -EBUSY;  

			// Data ready...
			uint8_t data[2]  = {0}; 
			vcnl4010_i2c_read (dev, VCNL4010_AMBIENT_LIGHT_RESULT_LSB_ADDR, VCNL4010_AMBIENT_LIGHT_RESULT_LSB_SIZE, &data[0]); 
			vcnl4010_i2c_read (dev, VCNL4010_AMBIENT_LIGHT_RESULT_MSB_ADDR, VCNL4010_AMBIENT_LIGHT_RESULT_MSB_SIZE, &data[1]); 		
			float lux = (float) sys_get_le16(data) *  0.25; 
			sensor_value_from_float(val, lux); 
			break; 
        }
        case SENSOR_CHAN_PROX:
        {
			// Check if we are ready?
			vcnl4010_i2c_read (dev, VCNL4010_COMMAND_REGISTER_ADDR, VCNL4010_COMMAND_REGISTER_SIZE, &buf); 
			if (VCNL4010_COMMAND_REGISTER_PROX_DATA_RD_GET(buf) == 0) return -EBUSY;  

			// Data ready...
			uint8_t data[2]  = {0}; 
			vcnl4010_i2c_read (dev, VCNL4010_PROX_RESULT_LSB_ADDR, VCNL4010_PROX_RESULT_LSB_SIZE, &data[0]); 
			vcnl4010_i2c_read (dev, VCNL4010_PROX_RESULT_MSB_ADDR, VCNL4010_PROX_RESULT_MSB_SIZE, &data[1]); 		
			int32_t prox = sys_get_le16(data); 
			val->val1 = (int32_t)prox;  // Integer part
            break;
        }
        default:
        {
        	return -ENODEV; 
        }
    }

	
	return 0;
}

// ##################################################################
// static int vcnl4010_sample_fetch(const struct device *dev, enum sensor_channel chan)
// ##################################################################
static int vcnl4010_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	uint8_t buf = 0; 
	switch (chan)
    {
        case SENSOR_CHAN_LIGHT:
        {
			vcnl4010_i2c_read (dev, VCNL4010_COMMAND_REGISTER_ADDR, VCNL4010_COMMAND_REGISTER_SIZE, &buf); 
			buf = VCNL4010_COMMAND_REGISTER_ALS_OD_SET(buf, 1); 
			vcnl4010_i2c_write (dev, VCNL4010_COMMAND_REGISTER_ADDR, VCNL4010_COMMAND_REGISTER_SIZE, &buf); 
        }
        case SENSOR_CHAN_PROX:
        {
			vcnl4010_i2c_read (dev, VCNL4010_COMMAND_REGISTER_ADDR, VCNL4010_COMMAND_REGISTER_SIZE, &buf); 
			buf = VCNL4010_COMMAND_REGISTER_PROX_OD_SET(buf, 1); 
			vcnl4010_i2c_write (dev, VCNL4010_COMMAND_REGISTER_ADDR, VCNL4010_COMMAND_REGISTER_SIZE, &buf);
            break;
        }
        default:
        {
        	return -ENODEV; 
        }
    }

	
	return 0;
}

// ##################################################################
// Driver API 
// ##################################################################
static const struct sensor_driver_api vcnl4010_api = 
{
	.trigger_set = vcnl4010_trigger_set,
	.attr_set = vcnl4010_attr_set,
	.channel_get = vcnl4010_channel_get,
	.sample_fetch = vcnl4010_sample_fetch ,
};

// ##################################################################
// Init 
// ##################################################################
static int vcnl4010_init(const struct device *dev)
{
	const struct vcnl4010_config *cfg = dev->config;
 	uint8_t buf = 0; 



	
	
	// Reset Chip
	if (!device_is_ready(cfg->sel_gpio.port))
	{
		LOG_ERR("GPIO not ready to reset IC"); 
		return -1; 
	}
	gpio_pin_configure_dt(&cfg->sel_gpio, GPIO_OUTPUT_INACTIVE); 
	



	__ASSERT(device_is_ready(cfg->i2c.bus), "i2c not ready"); 



	// Check ID is correct 
	vcnl4010_i2c_read (dev, VCNL4010_PRODUCT_ID_REVISION_ADDR, VCNL4010_PRODUCT_ID_REVISION_SIZE, &buf); 
	__ASSERT(VCNL4010_PRODUCT_ID_REVISION_PRODUCT_ID_GET(buf) == 0x02, "id mismatch"); 
	LOG_INF("id: %02X", (unsigned int) VCNL4010_PRODUCT_ID_REVISION_PRODUCT_ID_GET(buf)); 

	// Configure Prox Rate
	vcnl4010_i2c_read (dev, VCNL4010_PRODUCT_ID_REVISION_ADDR, VCNL4010_PRODUCT_ID_REVISION_SIZE, &buf); 
	buf = VCNL4010_RATE_OF_PROXIMITY_MEASUREMENT_PROXIMITY_RATE_SET(buf, cfg->prox_rate); 
	vcnl4010_i2c_write (dev, VCNL4010_PRODUCT_ID_REVISION_ADDR, VCNL4010_PRODUCT_ID_REVISION_SIZE, &buf); 

	// Configure ALS Rate
	vcnl4010_i2c_read (dev, VCNL4010_AMBIENT_LIGHT_PARAMETER_ADDR, VCNL4010_AMBIENT_LIGHT_PARAMETER_SIZE, &buf); 
	buf = VCNL4010_AMBIENT_LIGHT_PARAMETER_AMBIENT_LIGHT_MEASUREMENT_RATE_SET(buf, cfg->als_rate); 
	vcnl4010_i2c_write (dev, VCNL4010_AMBIENT_LIGHT_PARAMETER_ADDR, VCNL4010_AMBIENT_LIGHT_PARAMETER_SIZE, &buf); 

	// Configure LED Current
	vcnl4010_i2c_read (dev, VCNL4010_LED_CURRENT_SETTING_PROXIMITY_MODE_ADDR, VCNL4010_LED_CURRENT_SETTING_PROXIMITY_MODE_SIZE, &buf); 
	buf = VCNL4010_LED_CURRENT_SETTING_PROXIMITY_MODE_IR_LED_CURRENT_VALUE_SET(buf, cfg->iled); 
	vcnl4010_i2c_write (dev, VCNL4010_LED_CURRENT_SETTING_PROXIMITY_MODE_ADDR, VCNL4010_LED_CURRENT_SETTING_PROXIMITY_MODE_SIZE, &buf); 

	LOG_INF("ready"); 
	return 0;
}


// ##################################################################
// Instantiation 
// ##################################################################
#define VCNL4010_INIT(inst) \
	static const struct vcnl4010_config vcnl4010_config_##inst = \
	{ \
		.prox_rate = DT_INST_ENUM_IDX(inst, prox_rate), \
		.als_rate = DT_INST_ENUM_IDX(inst, als_rate), \
		.iled = DT_INST_PROP(inst, iled), \
		.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.sel_gpio = GPIO_DT_SPEC_INST_GET_BY_IDX_OR(inst, sel_gpios, 0, {0}),\
		.id = DT_INST_PROP_OR(inst, id, 0xFF) \
	};\
	static struct vcnl4010_data vcnl4010_data_##inst;	\
	DEVICE_DT_INST_DEFINE(inst, vcnl4010_init, NULL, &vcnl4010_data_##inst, &vcnl4010_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &vcnl4010_api);
DT_INST_FOREACH_STATUS_OKAY(VCNL4010_INIT)