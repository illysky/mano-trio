/*
 _  _  _              _           
(_)| || | _   _  ___ | | __ _   _ 
| || || || | | |/ __|| |/ /| | | |
| || || || |_| |\__ \|   < | |_| |
|_||_||_| \__, ||___/|_|\_\ \__, |
          |___/             |___/

Driver for Awinic aw21036 v1.0.0
by Code Crafter
Copyright © 2024 illysky
*/  

#define DT_DRV_COMPAT awinic_aw21036

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include "aw21036.h"

LOG_MODULE_REGISTER(aw21036, CONFIG_LED_LOG_LEVEL);

// ##################################################################
// Config Structure 
// ##################################################################
struct aw21036_config 
{
	// Add more elements to config stuct as need, eg gpios for irqs
	// Remember to update instantiation if you need
	// them initialised at boot
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec enable;

};

// ##################################################################
// Data Structure 
// ##################################################################
struct aw21036_data
{
	// Add more elements to the driver data structure that need to 
	// persist at runtime. Remove if not needed
	bool ready;
	
};

// ##################################################################
// Driver API Implementation Stubs
// ##################################################################
// Visit https://docs.zephyrproject.org/latest/develop/api/overview.html 
// to get the API calls you need to implement for this type of driver
// eg. sensor or charger. 
// Update the _driver_api struct accordingly 

// ##################################################################
// static int aw21036_blink(const struct device *dev, uint32_t led,uint32_t delay_on, uint32_t delay_off)
// ##################################################################
static int aw21036_blink(const struct device *dev, uint32_t led,uint32_t delay_on, uint32_t delay_off)
{
	// Not implemented 
	return 0;
}

// ##################################################################
// static int aw21036_set_brightness(const struct device *dev, uint32_t led, uint8_t value)
// ##################################################################
static int aw21036_set_brightness(const struct device *dev, uint32_t led, uint8_t value)
{

	const struct aw21036_config *cfg = dev->config;
	int result = -1;
	if (led > 35) return -1; 
	if (i2c_burst_write_dt (&cfg->i2c, AW21036_LED_CHANNEL_CONTROL_BR01_ADDR+led, (uint8_t*) &value, AW21036_LED_CHANNEL_CONTROL_BR01_SIZE) == 0)
	{
		uint8_t buf = 0; 
		result = i2c_burst_write_dt (&cfg->i2c, AW21036_UPDATE_REGISTER_ADDR, (uint8_t*) &buf, AW21036_UPDATE_REGISTER_SIZE); 
	}; 
	return result;


}

// ##################################################################
// static inline int aw21036_off(const struct device *dev, uint32_t led)
// ##################################################################
static inline int aw21036_off(const struct device *dev, uint32_t led)
{
	const struct aw21036_config *cfg = dev->config;
	int result = -1;
	if (led > 35) return -1; 
	uint8_t buf = 0; 
	if (i2c_burst_write_dt (&cfg->i2c, AW21036_LED_CHANNEL_CONTROL_BR01_ADDR+led, (uint8_t*) &buf, AW21036_LED_CHANNEL_CONTROL_BR01_SIZE) == 0)
	{
		buf = 0; 
		result = i2c_burst_write_dt (&cfg->i2c, AW21036_UPDATE_REGISTER_ADDR, (uint8_t*) &buf, AW21036_UPDATE_REGISTER_SIZE); 
	}; 
	return result;
}

// ##################################################################
// static inline int aw21036_on(const struct device *dev, uint32_t led)
// ##################################################################
static inline int aw21036_on(const struct device *dev, uint32_t led)
{
	const struct aw21036_config *cfg = dev->config;
	int result = -1;
	if (led > 35) return -1; 
	uint8_t buf = 255; 
	if (i2c_burst_write_dt (&cfg->i2c, AW21036_LED_CHANNEL_CONTROL_BR01_ADDR+led, (uint8_t*) &buf, AW21036_LED_CHANNEL_CONTROL_BR01_SIZE) == 0)
	{
		buf = 0; 
		result = i2c_burst_write_dt (&cfg->i2c, AW21036_UPDATE_REGISTER_ADDR, (uint8_t*) &buf, AW21036_UPDATE_REGISTER_SIZE); 
	}; 
	return result;
}

// ##################################################################
// Driver API 
// ##################################################################
static const struct led_driver_api aw21036_api = 
{
	.blink = aw21036_blink,
	.set_brightness = aw21036_set_brightness,
	.off = aw21036_off,
	.on = aw21036_on 
};

// ##################################################################
// Init 
// ##################################################################
static int aw21036_init(const struct device *dev)
{
	const struct aw21036_config *cfg = dev->config;
	uint8_t buf = 0; 


	// Reset Chip
	if (!device_is_ready(cfg->enable.port))
	{
		LOG_ERR("GPIO not ready to reset IC"); 
		return -1; 
	}

	gpio_pin_configure_dt(&cfg->enable, GPIO_OUTPUT_INACTIVE); 
	k_msleep(50); 
	gpio_pin_set_dt(&cfg->enable, 1); 
	k_msleep(3); 


	//Check DT
	if (!device_is_ready(cfg->i2c.bus))
	{
		LOG_ERR("I2C not ready for comms"); 
		return -1; 
	}

	// Read ID 
	buf = 0; 
	if (i2c_burst_read_dt (&cfg->i2c, AW21036_VERSION_ADDR, (uint8_t*) &buf, AW21036_VERSION_SIZE)!=0)
	{
		LOG_ERR("device not responding"); 
		return -ENODEV;
	}

	if (AW21036_VERSION_VER_GET(buf) !=  0xA8)
	{
		LOG_ERR("chip ID wrong"); 
		return -ENODEV;
	}

	LOG_INF("Chip ID: %02X", (unsigned int) AW21036_VERSION_VER_GET(buf)); 

	buf = 0x00; 
	i2c_burst_write_dt (&cfg->i2c, AW21036_RESET_ADDR, (uint8_t*) &buf, AW21036_RESET_SIZE); 
	k_msleep(3); 

	// Enable LED Driver
	buf = 0; 
	i2c_burst_read_dt (&cfg->i2c, AW21036_GLOBAL_CONTROL_ADDR, (uint8_t*) &buf, AW21036_GLOBAL_CONTROL_SIZE);
	buf = AW21036_GLOBAL_CONTROL_CHIPEN_SET(buf, 1); 
	i2c_burst_write_dt (&cfg->i2c, AW21036_GLOBAL_CONTROL_ADDR, (uint8_t*) &buf, AW21036_GLOBAL_CONTROL_SIZE); 

	// Clear and Update LEDs
	buf = 0x00; 
	for (int a=0; a<36; a++) 
	{
		i2c_burst_write_dt (&cfg->i2c, AW21036_LED_CHANNEL_CONTROL_BR01_ADDR+a, (uint8_t*) &buf, AW21036_LED_CHANNEL_CONTROL_BR01_SIZE); 
	}

	i2c_burst_write_dt (&cfg->i2c, AW21036_UPDATE_REGISTER_ADDR, (uint8_t*) &buf, AW21036_UPDATE_REGISTER_SIZE); 
	
	// Clear and Update LEDs
	buf = 0xFF; 
	for (int a=0; a<36; a++) i2c_burst_write_dt (&cfg->i2c, AW21036_LED_COLOUR_PARAMETER_CHANNEL_01_ADDR+a, (uint8_t*) &buf, AW21036_LED_COLOUR_PARAMETER_CHANNEL_01_SIZE); 

	// Enable Current
	buf = 0xFF; 
	i2c_burst_write_dt (&cfg->i2c, AW21036_GLOBAL_CURRENT_CONTROL_ADDR, (uint8_t*) &buf, AW21036_GLOBAL_CURRENT_CONTROL_SIZE); 
	
	buf = 0x00; 
	for (int a=0; a<36; a++) 
	{
		i2c_burst_write_dt (&cfg->i2c, AW21036_LED_CHANNEL_CONTROL_BR01_ADDR+a, (uint8_t*) &buf, AW21036_LED_CHANNEL_CONTROL_BR01_SIZE); 
		i2c_burst_write_dt (&cfg->i2c, AW21036_UPDATE_REGISTER_ADDR, (uint8_t*) &buf, AW21036_UPDATE_REGISTER_SIZE); 

	}

	return 0;
}

// ##################################################################
// Instantiation 
// ##################################################################
#define AW21036_INIT(inst) \
	static const struct aw21036_config aw21036_config_##inst = \
	{\
		.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.enable = GPIO_DT_SPEC_INST_GET_BY_IDX(inst, en_gpios, 0) \
	};\
	static struct aw21036_data aw21036_data_##inst;	\
	DEVICE_DT_INST_DEFINE(inst, aw21036_init, NULL, &aw21036_data_##inst, &aw21036_config_##inst, POST_KERNEL, CONFIG_LED_INIT_PRIORITY, &aw21036_api);
DT_INST_FOREACH_STATUS_OKAY(AW21036_INIT)