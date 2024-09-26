/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT awinic_aw9109

/**
 * @file
 * @brief LED driver for the PCA9633 I2C LED driver (7-bit slave address 0x62)
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#define LOG_LEVEL CONFIG_LED_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(aw9109);



#define AW9109_IDRST 	0x00
#define AW9109_GCR 		0x01
#define AW9109_LER 		0x50
#define AW9109_LCR  	0x52 
#define AW9109_PROGMD 	0x53 
#define AW9109_RUNMD 	0x54 
#define AW9109_CTRS 	0x55 
#define AW9109_IMAX1 	0x57 
#define AW9109_IMAX2 	0x58 
#define AW9109_IMAX3 	0x59 
#define AW9109_TIER 	0x5C 
#define AW9109_TIVEC 	0x5D 
#define AW9109_ISR2 	0x5E 
#define AW9109_SADDR 	0x5F 
#define AW9109_PCR 		0x60 
#define AW9109_CMDR 	0x61 
#define AW9109_RA 		0x62 
#define AW9109_RB 		0x63 
#define AW9109_RC 		0x64 
#define AW9109_RD 		0x65 
#define AW9109_R1 		0x66 
#define AW9109_R2 		0x67 
#define AW9109_R3 		0x68 
#define AW9109_R4 		0x69 
#define AW9109_R5 		0x6A 
#define AW9109_R6 		0x6B 
#define AW9109_R7 		0x6C 
#define AW9109_R8	    0x6D
#define AW9109_WADDR 	0x7E 
#define AW9109_WDATA	0x7F


struct led_data {
	/* Minimum acceptable LED blinking time period (in ms) */
	uint32_t min_period;
	/* Maximum acceptable LED blinking time period (in ms) */
	uint32_t max_period;
	/* Minimum acceptable LED brightness value */
	uint16_t min_brightness;
	/* Maximum acceptable LED brightness value */
	uint16_t max_brightness;
};




struct aw9109_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec pdn;
	struct gpio_dt_spec irqn;
};

struct aw9109_data {
	struct led_data dev_data;
};


/****************************************************************************************
@overview: 
****************************************************************************************/
static int _set_current(const struct device *dev, uint32_t channel, uint8_t current)
{
	if (current > 7) current = 7; 
	const struct aw9109_config *config = dev->config;
	uint8_t reg = AW9109_IMAX1; 
	uint8_t shift = 0; 

	// Get the mapping of the current setting
	switch (channel)
	{
		case 0:
				{	
			reg = AW9109_IMAX1; 
			shift = 8; 
		}
		break; 
		case 1:
		{	
			reg = AW9109_IMAX1; 
			shift = 12; 
		}
		break; 
		case 2:
		{	
			reg = AW9109_IMAX2; 
			shift = 0; 
		}
		break; 
		case 3:
		{	
			reg = AW9109_IMAX2; 
			shift = 4; 
		}
		break; 
		case 4:
		{	
			reg = AW9109_IMAX2; 
			shift = 8; 
		}
		break; 
		case 5:
		{	
			reg = AW9109_IMAX2; 
			shift = 12; 
		}
		break; 		
		case 6:
		{	
			reg = AW9109_IMAX3; 
			shift = 0; 
		}
		break; 		
		case 7:
		{	
			reg = AW9109_IMAX3; 
			shift = 4; 
		}
		break; 		
		case 8:
		{	
			reg = AW9109_IMAX3; 
			shift = 8; 
		}
		break; 	
		default: 
		{
			LOG_ERR("channel not implemented");
			return -ENOSYS;
		}	
		break; 		
	}

	// Read Reg
	uint16_t imax = 0; 
	if(i2c_burst_read_dt (&config->i2c, reg, (uint8_t*) &imax, 2))
	{
		LOG_ERR("not responding");
		return -ENODEV;
	}
	
	imax = sys_be16_to_cpu(imax); 		// Swap
	imax &= ~(0x0F << shift);			// Clear
	imax |= ((current & 0x0F) << shift);	// Write
	imax = sys_cpu_to_be16(imax); 		// Swap
	if (i2c_burst_write_dt (&config->i2c, reg, (uint8_t*) &imax, 2))
	{
		LOG_ERR("not responding");
		return -ENODEV;
	}
	return 0;
}

/****************************************************************************************
@overview:  
****************************************************************************************/
static inline int _led_enable (const struct device *dev, uint8_t channel, bool enable)
{
	
	const struct aw9109_config *config = dev->config;
	uint16_t status = 0; 

	if (channel > 8) 
	{
		LOG_ERR("channel not implemented");
		return -ENOSYS;
	}


	if (i2c_burst_read_dt (&config->i2c, AW9109_LER, (uint8_t*) &status, 2)) 
	{
		LOG_ERR("not responding");
		return -ENODEV;
	}	

	status = sys_be16_to_cpu(status); 		// Swap
 	WRITE_BIT(status, channel+2, enable);	// Write
	status = sys_cpu_to_be16 (status); 		// Swap 

	if (i2c_burst_write_dt (&config->i2c, AW9109_LER, (uint8_t*) &status, 2)) 
	{
		LOG_ERR("not responding");
		return -ENODEV;
	}

	return 0;
}



/****************************************************************************************
@overview:  
****************************************************************************************/
static inline int _led_duty (const struct device *dev, uint8_t channel, uint8_t duty)
{
	const struct aw9109_config *config = dev->config;

	if (channel > 8) 
	{
		LOG_ERR("channel not implemented");
		return -ENOSYS;
	}

	uint16_t command = 0xA000 | ((channel+2) << 8) | duty; 
	command = sys_cpu_to_be16 (command); 		// Swap 
	i2c_burst_write_dt (&config->i2c, AW9109_CMDR, (uint8_t*) &command, 2); 
	return 0; 
}

/****************************************************************************************
@overview:  
****************************************************************************************/
static int aw9109_led_blink(const struct device *dev, uint32_t led,uint32_t delay_on, uint32_t delay_off)
{
	//struct aw9109_data *data = dev->data;
	//const struct aw9109_config *config = dev->config;
	//struct led_data *dev_data = &data->dev_data;
	return 0;
}

/****************************************************************************************
@overview:  
****************************************************************************************/
static int aw9109_led_set_brightness(const struct device *dev, uint32_t led, uint8_t value)
{
	return _led_duty (dev, led, value); 
}


/****************************************************************************************
@overview:  
****************************************************************************************/
static inline int aw9109_led_off(const struct device *dev, uint32_t led)
{
	return _led_enable (dev, led, false);
}

/****************************************************************************************
@overview:  
****************************************************************************************/
static inline int aw9109_led_on(const struct device *dev, uint32_t led)
{
	 
	return _led_enable (dev, led, true);;
}


/****************************************************************************************
@overview:  
****************************************************************************************/
static int aw9109_led_init(const struct device *dev)
{
	const struct aw9109_config *config = dev->config;
	uint16_t id = 0; 
	uint8_t buf [2]; 

	if (!device_is_ready(config->i2c.bus)) 
	{
		LOG_ERR("I2C bus is not ready");
		return -ENODEV;
	}

	// Reset
	if (config->pdn.port) 
	{
		gpio_pin_configure_dt(&config->pdn, GPIO_OUTPUT_ACTIVE); 
		k_msleep(10); 
		gpio_pin_set_dt(&config->pdn, 0); 
		k_msleep(10); 
	}
	else
	{
		LOG_INF("PDN is not configured in device tree" );
	}

	/* Read device */
	i2c_burst_read_dt (&config->i2c, AW9109_IDRST,(uint8_t*) &id, 2); 
	if (id != 0x23B2)
	{
		LOG_ERR("device not found");
		return -ENODEV;
	}

	/* Global Control */
	buf[0]=0x00;  // H
	buf[1]=0x01;  // L
	i2c_burst_write_dt (&config->i2c, AW9109_GCR,buf, 2); 

	/* Channel Enable: All on  */
	buf[0]=0x03;  // H
	buf[1]=0xFC;  // L
	i2c_burst_write_dt (&config->i2c, AW9109_LER,buf, 2); 

	/* Channel Mode: MCU  */
	buf[0]=0x03;  // H
	buf[1]=0xFC;  // L
	i2c_burst_write_dt (&config->i2c, AW9109_CTRS,buf, 2); 

	_set_current(dev, 0, CONFIG_AW9109_CURRENT_CHANNEL_1);
	_set_current(dev, 1, CONFIG_AW9109_CURRENT_CHANNEL_1);
	_set_current(dev, 2, CONFIG_AW9109_CURRENT_CHANNEL_2);
	_set_current(dev, 3, CONFIG_AW9109_CURRENT_CHANNEL_3);
	_set_current(dev, 4, CONFIG_AW9109_CURRENT_CHANNEL_4);
	_set_current(dev, 5, CONFIG_AW9109_CURRENT_CHANNEL_5);
	_set_current(dev, 6, CONFIG_AW9109_CURRENT_CHANNEL_6);
	_set_current(dev, 7, CONFIG_AW9109_CURRENT_CHANNEL_7);
	_set_current(dev, 8, CONFIG_AW9109_CURRENT_CHANNEL_8);

	return 0;
}
 
/****************************************************************************************
@overview:  
****************************************************************************************/
static const struct led_driver_api aw9109_led_api = {
	.blink = aw9109_led_blink,
	.set_brightness = aw9109_led_set_brightness,
	.on = aw9109_led_on,
	.off = aw9109_led_off,
};

/****************************************************************************************
@overview:  
****************************************************************************************/
#define AW9109_DEVICE(id)												\
	static const struct aw9109_config aw9109_##id##_cfg = {				\
		.i2c = I2C_DT_SPEC_INST_GET(id),								\
		.pdn = GPIO_DT_SPEC_INST_GET_OR(0, pdn_gpios, {0}), 			\
		.irqn = GPIO_DT_SPEC_INST_GET_OR(0, irqn_gpios, {0}),			\
	};																	\
	static struct aw9109_data aw9109_##id##_data;						\
	DEVICE_DT_INST_DEFINE(id, &aw9109_led_init, NULL,					\
			&aw9109_##id##_data,										\
			&aw9109_##id##_cfg, POST_KERNEL,							\
			CONFIG_LED_INIT_PRIORITY,									\
			&aw9109_led_api);

DT_INST_FOREACH_STATUS_OKAY(AW9109_DEVICE)
