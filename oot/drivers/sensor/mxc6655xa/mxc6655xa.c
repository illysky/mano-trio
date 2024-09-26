/*
 _  _  _              _           
(_)| || | _   _  ___ | | __ _   _ 
| || || || | | |/ __|| |/ /| | | |
| || || || |_| |\__ \|   < | |_| |
|_||_||_| \__, ||___/|_|\_\ \__, |
          |___/             |___/

Driver for Memsic mxc6655xa v1.0.0
by Code Crafter
Copyright © 2024 illysky
*/  

#define DT_DRV_COMPAT memsic_mxc6655xa

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
#include <zephyr/sys/util.h>
#include "mxc6655xa.h"

LOG_MODULE_REGISTER(mxc6655xa, CONFIG_SENSOR_LOG_LEVEL);

// ##################################################################
// Config Structure 
// ##################################################################
struct mxc6655xa_config 
{
	// Add more elements to config stuct as need, eg gpios for irqs
	// Remember to update instantiation if you need
	// them initialised at boot
	struct i2c_dt_spec i2c;
	int32_t fsr; 
};

// ##################################################################
// Data Structure 
// ##################################################################
struct mxc6655xa_data
{
	// Add more elements to the driver data structure that need to 
	// persist at runtime. Remove if not needed
	bool ready;
	
};

// ##################################################################
//
// ##################################################################
static inline int  mxc6655xa_i2c_write (const struct device *dev, uint8_t addr, uint8_t size, uint8_t* buf)
{
	const struct mxc6655xa_config *cfg = dev->config;
	__ASSERT(!i2c_burst_read_dt (&cfg->i2c, addr,buf, size), "device not responding");
	return 0; 
}

// ##################################################################
// static inline int mxc6655xa_i2c_read (const struct device *dev, uint8_t addr, uint8_t size, uint8_t* buf)
// ##################################################################
static inline int mxc6655xa_i2c_read (const struct device *dev, uint8_t addr, uint8_t size, uint8_t* buf)
{
	const struct mxc6655xa_config *cfg = dev->config;
	__ASSERT(!i2c_burst_read_dt (&cfg->i2c, addr, buf, size), "device not responding");
	return 0; 
}

// ##################################################################
// static int mxc6655xa_convert(const struct device *dev, uint8_t* raw, struct sensor_value *val )
// ##################################################################
static int mxc6655xa_convert(const struct device *dev, uint8_t* data, struct sensor_value *val )
{
	const struct mxc6655xa_config *cfg = dev->config;
	float scale[3] = {1024.0, 512.0, 256.0}; 
	int16_t raw = (int16_t) sys_get_be16(data); 
	raw = raw >> 4; 
	sensor_value_from_float(val, ((float) raw) / scale[cfg->fsr]); 
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
// static int mxc6655xa_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler)
// ##################################################################
static int mxc6655xa_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler)
{
	return 0;
}

// ##################################################################
// static int mxc6655xa_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
// ##################################################################
static int mxc6655xa_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
{
	return 0;
}

// ##################################################################
// static int mxc6655xa_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
// ##################################################################
static int mxc6655xa_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	uint8_t buf[2] = {0}; 
	switch (chan) 
	{
        case SENSOR_CHAN_ACCEL_X:
        {
            mxc6655xa_i2c_read (dev, MXC6655XA_X_AXIS_XL_MSB_ADDR, 2, buf);
			mxc6655xa_convert(dev, buf, &val[0]);
			break;
        }
        case SENSOR_CHAN_ACCEL_Y:
        {
            mxc6655xa_i2c_read (dev, MXC6655XA_Y_AXIS_XL_MSB_ADDR, 2, buf);
			mxc6655xa_convert(dev, buf, &val[0]);
			break;
        }
        case SENSOR_CHAN_ACCEL_Z:
        {
            mxc6655xa_i2c_read (dev, MXC6655XA_Z_AXIS_XL_MSB_ADDR, 2, buf);
			mxc6655xa_convert(dev, buf, &val[0]);
			break;
        }

		case SENSOR_CHAN_ACCEL_XYZ:
        {
			mxc6655xa_i2c_read (dev, MXC6655XA_X_AXIS_XL_MSB_ADDR, 2,  buf);
			mxc6655xa_convert(dev, buf, &val[0]);
			mxc6655xa_i2c_read (dev, MXC6655XA_Y_AXIS_XL_MSB_ADDR, 2,  buf);
			mxc6655xa_convert(dev, buf, &val[1]);
			mxc6655xa_i2c_read (dev, MXC6655XA_Z_AXIS_XL_MSB_ADDR, 2, buf);
			mxc6655xa_convert(dev, buf, &val[2]);
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
// static int mxc6655xa_sample_fetch(const struct device *dev, enum sensor_channel chan)
// ##################################################################
static int mxc6655xa_sample_fetch(const struct device *dev, enum sensor_channel chan)
{

	switch (chan) 
	{
        case SENSOR_CHAN_ACCEL_X:
        {
			break;
        }
        case SENSOR_CHAN_ACCEL_Y:
        {
            break;
        }
        case SENSOR_CHAN_ACCEL_Z:
        {
            break;
        }
        case SENSOR_CHAN_ACCEL_XYZ:
        {
            break;
        }
        // Handle other channels if needed
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
static const struct sensor_driver_api mxc6655xa_api = 
{
	.trigger_set = mxc6655xa_trigger_set,
	.attr_set = mxc6655xa_attr_set,
	.channel_get = mxc6655xa_channel_get,
	.sample_fetch = mxc6655xa_sample_fetch 
};

// ##################################################################
// Init 
// ##################################################################
static int mxc6655xa_init(const struct device *dev)
{
	uint8_t buf = 0; 
	
	// Check ID is correct 
	mxc6655xa_i2c_read (dev, MXC6655XA_WHO_AM_I_ADDR, MXC6655XA_WHO_AM_I_SIZE, &buf); 

	//__ASSERT(MXC6655XA_WHO_AM_I_ID_GET(buf) == 0x0F, "id mismatch"); 
	
	LOG_INF("id: %02X", (unsigned int) MXC6655XA_WHO_AM_I_ID_GET(buf)); 
	
	// Init code added here
	mxc6655xa_i2c_read (dev, MXC6655XA_OPERATING_MODE_CTRL_ADDR, MXC6655XA_OPERATING_MODE_CTRL_SIZE,  &buf); 
	buf = MXC6655XA_OPERATING_MODE_CTRL_PD_SET(buf, 0); 			// Enable	
	buf = MXC6655XA_OPERATING_MODE_CTRL_FSR_SET(buf, 2); 			// FSR

	mxc6655xa_i2c_write (dev, MXC6655XA_OPERATING_MODE_CTRL_ADDR, MXC6655XA_OPERATING_MODE_CTRL_SIZE,  &buf);  
	LOG_INF("ready"); 
	return 0;
}


// ##################################################################
// Instantiation 
// ##################################################################
#define MXC6655XA_INIT(inst) \
	static const struct mxc6655xa_config mxc6655xa_config_##inst = { \
		.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.fsr = DT_INST_ENUM_IDX(inst, fsr)  \
	}; \
	static struct mxc6655xa_data mxc6655xa_data_##inst;	\
	DEVICE_DT_INST_DEFINE(inst, mxc6655xa_init, NULL, &mxc6655xa_data_##inst, &mxc6655xa_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &mxc6655xa_api);
DT_INST_FOREACH_STATUS_OKAY(MXC6655XA_INIT)