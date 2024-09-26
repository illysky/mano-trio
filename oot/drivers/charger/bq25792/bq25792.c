/*
 _  _  _              _           
(_)| || | _   _  ___ | | __ _   _ 
| || || || | | |/ __|| |/ /| | | |
| || || || |_| |\__ \|   < | |_| |
|_||_||_| \__, ||___/|_|\_\ \__, |
          |___/             |___/

Driver for TI BQ25792 v1.0.0
by Code Crafter
Copyright © 2024 illysky
*/  

#define DT_DRV_COMPAT ti_bq25792

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include "bq25792.h"


LOG_MODULE_REGISTER(bq25792, CONFIG_CHARGER_LOG_LEVEL);

// ##################################################################
// Config Structure 
// ##################################################################
struct bq25792_config 
{
	// Add more elements to config stuct as need, eg gpios for irqs
	// Remember to update instantiation if you need
	// them initialised at boot
	struct i2c_dt_spec i2c;
	int input_ilim; 
	int charge_ilim; 
	struct gpio_dt_spec ce_gpio; 
	struct gpio_dt_spec irq_gpio; 
};

// ##################################################################
// Data Structure 
// ##################################################################
struct bq25792_data
{
	// Add more elements to the driver data structure that need to 
	// persist at runtime. Remove if not needed
	bool ready;
	
};


// ##################################################################
// static int vcnl4010_i2c_read (const struct device *dev, uint8_t addr, uint8_t size, uint8_t* buf)
// ##################################################################
static int  bq25792_i2c_write (const struct device *dev, uint8_t addr, uint8_t size, uint8_t* buf)
{
	const struct bq25792_config *cfg = dev->config;
	__ASSERT(!i2c_burst_write_dt (&cfg->i2c, addr, buf, size), "device not responding");
	return 0; 
}

// ##################################################################
// static  int vcnl4010_i2c_read (const struct device *dev, uint8_t addr, uint8_t size, uint8_t* buf)
// ##################################################################
static int bq25792_i2c_read (const struct device *dev, uint8_t addr, uint8_t size, uint8_t* buf)
{
	const struct bq25792_config *cfg = dev->config;
	__ASSERT(!i2c_burst_read_dt (&cfg->i2c, addr, buf, size), "device not responding");
	return 0; 
}

// ##################################################################
// int bq25792_chip_enable (bool enable)
// ##################################################################
static int bq25792_chip_enable (const struct device *dev, bool enable)
{
	uint8_t buf = 0; 
	const struct bq25792_config *cfg = dev->config;
	if (cfg->ce_gpio.port != NULL) 
	{
		gpio_pin_set_dt(&cfg->ce_gpio, enable); 
	}

	bq25792_i2c_read (dev, BQ25792_CHARGER_CONTROL_0_ADDR, BQ25792_CHARGER_CONTROL_0_SIZE, &buf); 
	buf = BQ25792_CHARGER_CONTROL_0_EN_CHG_SET(buf, enable); 
	bq25792_i2c_write (dev, BQ25792_CHARGER_CONTROL_0_ADDR, BQ25792_CHARGER_CONTROL_0_SIZE, &buf); 
	return 0; 
}

// ##################################################################
// int bq25792_set_charge_ilim (const struct device *dev, uint32_t ilim)
// ##################################################################
static int bq25792_set_charge_ilim (const struct device *dev, uint32_t mA)
{
	uint8_t buf[2] = {0}; 
	uint16_t raw = 0; 
	bq25792_i2c_read (dev, BQ25792_CHARGE_CURRENT_LIMIT_ADDR, BQ25792_CHARGE_CURRENT_LIMIT_SIZE, buf); 
	raw = BQ25792_CHARGE_CURRENT_LIMIT_ICHG_SET(sys_get_be16(buf), MAX(mA, 50) / 10); 
	sys_put_be16(raw, buf); 
	bq25792_i2c_write (dev, BQ25792_CHARGE_CURRENT_LIMIT_ADDR, BQ25792_CHARGE_CURRENT_LIMIT_SIZE, buf); 
	return 0; 
}



static int bq25792_set_en_extilim(const struct device *dev, bool  enable)
{
	uint8_t buf = 0; 
	bq25792_i2c_read (dev, BQ25792_CHARGER_CONTROL_5_ADDR, BQ25792_CHARGER_CONTROL_5_SIZE, &buf); 
	buf = BQ25792_CHARGER_CONTROL_5_EN_EXTILIM_SET(buf, enable); 
	bq25792_i2c_write (dev, BQ25792_CHARGER_CONTROL_5_ADDR, BQ25792_CHARGER_CONTROL_5_SIZE,&buf); 
	return 0; 
}

// ##################################################################
// int bq25792_set_charge_ilim (const struct device *dev, uint32_t ilim)
// ##################################################################
static int bq25792_get_charge_ilim (const struct device *dev, uint32_t* mA)
{
	uint8_t buf[2] = {0}; 
	bq25792_i2c_read (dev, BQ25792_CHARGE_CURRENT_LIMIT_ADDR, BQ25792_CHARGE_CURRENT_LIMIT_SIZE,buf); 
	(*mA) = BQ25792_CHARGE_CURRENT_LIMIT_ICHG_GET(sys_get_be16(buf)) * 10; 
	return 0; 
}

// ##################################################################
// 
// ##################################################################
static int bq25792_set_input_ilim (const struct device *dev, uint32_t mA)
{
	uint8_t buf[2] = {0}; 

	bq25792_set_en_extilim(dev, 0); 
	bq25792_i2c_read (dev, BQ25792_INPUT_CURRENT_LIMIT_ADDR, BQ25792_INPUT_CURRENT_LIMIT_SIZE, buf); 
	uint16_t raw = BQ25792_INPUT_CURRENT_LIMIT_IINDPM_SET(sys_get_be16(buf), MAX(mA, 100) / 10); 
	sys_put_be16(raw, buf); 
	bq25792_i2c_write (dev, BQ25792_INPUT_CURRENT_LIMIT_ADDR, BQ25792_INPUT_CURRENT_LIMIT_SIZE,buf); 
	return 0; 
}




// ##################################################################
// 
// ##################################################################
static int bq25792_get_input_ilim (const struct device *dev, uint32_t* mA)
{
	uint8_t buf[2] = {0}; 
	bq25792_i2c_read (dev, BQ25792_INPUT_CURRENT_LIMIT_ADDR, BQ25792_INPUT_CURRENT_LIMIT_SIZE, (uint8_t*)&buf); 
	(*mA) = BQ25792_INPUT_CURRENT_LIMIT_IINDPM_GET(sys_get_be16(buf)) * 10; 
	return 0; 
}

// ##################################################################
// 
// ##################################################################
static int bq25792_set_iterm (const struct device *dev, uint32_t mA)
{
	uint8_t buf = 0; 
	bq25792_i2c_read (dev, BQ25792_TERMINATION_CONTROL_ADDR, BQ25792_TERMINATION_CONTROL_SIZE, &buf); 
	buf = BQ25792_TERMINATION_CONTROL_ITERM_SET(buf, MAX(mA , 50) / 10); 
	bq25792_i2c_write (dev, BQ25792_TERMINATION_CONTROL_ADDR, BQ25792_TERMINATION_CONTROL_SIZE, &buf); 
	return 0;
}

// ##################################################################
// 
// ##################################################################
static int bq25792_get_iterm (const struct device *dev, uint32_t* mA)
{
	uint8_t buf = 0; 
	bq25792_i2c_read (dev, BQ25792_TERMINATION_CONTROL_ADDR, BQ25792_TERMINATION_CONTROL_SIZE, &buf); 
	(*mA) = BQ25792_TERMINATION_CONTROL_ITERM_GET(buf) * 10; 
	return 0; 
}

// ##################################################################
// 
// ##################################################################
static int bq25792_set_iprecharge(const struct device *dev, uint32_t mA)
{
	uint8_t buf = 0; 
	bq25792_i2c_read (dev, BQ25792_PRECHARGE_CONTROL_ADDR, BQ25792_PRECHARGE_CONTROL_SIZE, &buf); 
	buf = BQ25792_PRECHARGE_CONTROL_IPRECHG_SET(buf, MAX(mA , 40) / 40); 
	bq25792_i2c_write (dev, BQ25792_PRECHARGE_CONTROL_ADDR, BQ25792_PRECHARGE_CONTROL_SIZE, &buf); 
	return 0;
}

// ##################################################################
// 
// ##################################################################
static int bq25792_get_iprecharge (const struct device *dev, uint32_t* mA)
{
	uint8_t buf = 0; 
	bq25792_i2c_read (dev, BQ25792_PRECHARGE_CONTROL_ADDR, BQ25792_PRECHARGE_CONTROL_SIZE, &buf); 
	(*mA) = BQ25792_PRECHARGE_CONTROL_IPRECHG_GET(buf) * 40; 
	return 0; 
}

// ##################################################################
// 
// ##################################################################
static int bq25792_set_vreg(const struct device *dev, uint32_t mV)
{
	uint8_t buf[2] = {0}; 
	bq25792_i2c_read (dev, BQ25792_CHARGE_VOLTAGE_LIMIT_ADDR, BQ25792_CHARGE_VOLTAGE_LIMIT_SIZE,(uint8_t*) &buf); 
	uint16_t raw = BQ25792_CHARGE_VOLTAGE_LIMIT_VREG_SET(sys_get_be16(buf), MAX(mV, 3000) / 10); 
	sys_put_be16(raw, buf); 
	bq25792_i2c_write (dev, BQ25792_CHARGE_VOLTAGE_LIMIT_ADDR, BQ25792_CHARGE_VOLTAGE_LIMIT_SIZE, (uint8_t*) &buf); 
	return 0;
}

// ##################################################################
// 
// ##################################################################
static int bq25792_get_vreg (const struct device *dev, uint32_t* mV)
{
	uint8_t buf[2] = {0}; 
	bq25792_i2c_read (dev, BQ25792_CHARGE_VOLTAGE_LIMIT_ADDR, BQ25792_CHARGE_VOLTAGE_LIMIT_SIZE, (uint8_t*)&buf); 
	(*mV)= BQ25792_CHARGE_VOLTAGE_LIMIT_VREG_GET(sys_get_be16(buf)) * 10; 
	return 0; 
}

// ##################################################################
// 
// ##################################################################
static int bq25792_set_vindpm (const struct device *dev, uint32_t mv)
{
	uint8_t buf = 0; 
	bq25792_i2c_read (dev, BQ25792_INPUT_VOLTAGE_LIMIT_ADDR, BQ25792_INPUT_VOLTAGE_LIMIT_SIZE, &buf); 
	buf = BQ25792_INPUT_VOLTAGE_LIMIT_VINDPM_SET(buf, MAX(mv , 3600) / 100); 
	bq25792_i2c_write (dev, BQ25792_INPUT_VOLTAGE_LIMIT_ADDR, BQ25792_INPUT_VOLTAGE_LIMIT_SIZE, &buf); 
	return 0;
}

// ##################################################################
// 
// ##################################################################
static int bq25792_get_vindpm(const struct device *dev, uint32_t* mv)
{
	uint8_t buf = 0; 
	bq25792_i2c_read (dev, BQ25792_INPUT_VOLTAGE_LIMIT_ADDR, BQ25792_INPUT_VOLTAGE_LIMIT_SIZE, &buf); 
	(*mv) = BQ25792_INPUT_VOLTAGE_LIMIT_VINDPM_GET(buf) * 100; 
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
// static int bq25792_get_property(const struct device *dev, charger_prop_t prop, union charger_propval *val)
// ##################################################################
static int bq25792_get_property(const struct device *dev, charger_prop_t prop, union charger_propval *val)
{
	switch (prop)
    {
        case CHARGER_PROP_ONLINE:
        {
          	
			uint8_t buf = 0; 
			bq25792_i2c_read (dev, BQ25792_CHARGER_STATUS_0_ADDR, BQ25792_CHARGER_STATUS_0_SIZE, &buf); 
			if (BQ25792_CHARGER_STATUS_0_VBUS_PRESENT_STAT_GET(buf)) val->online = CHARGER_ONLINE_FIXED; 
			else val->online = CHARGER_ONLINE_OFFLINE;    
            break;

        }

        case CHARGER_PROP_PRESENT:
        {
			uint8_t buf = 0; 
			bq25792_i2c_read (dev, BQ25792_CHARGER_STATUS_2_ADDR, BQ25792_CHARGER_STATUS_2_SIZE, &buf); 
			val->present = (bool) BQ25792_CHARGER_STATUS_2_VBAT_PRESENT_STAT_GET(buf);   
            break;
        }

        case CHARGER_PROP_STATUS:
        {
            uint8_t buf = 0; 
			bq25792_i2c_read (dev, BQ25792_CHARGER_STATUS_1_ADDR, BQ25792_CHARGER_STATUS_1_SIZE, &buf); 
			uint8_t status = BQ25792_CHARGER_STATUS_1_CHG_STAT_GET(buf); 

			switch (status)
			{
				// Not Charging
				case 0: 
				{
					val->status = CHARGER_STATUS_NOT_CHARGING;
					break; 
				}
				// Charging
				case 1:
				case 2:
				case 3:  
				case 4:  
				case 6:  
				{
					val->status = CHARGER_STATUS_CHARGING;
					break; 
				}
				// Full
				case 7:
				{
					val->status = CHARGER_STATUS_FULL;
					break; 
				}
				default:
				{
					val->status = CHARGER_STATUS_UNKNOWN; 
				}
			}
            break;
        }

        case CHARGER_PROP_CHARGE_TYPE:
        {
			val->charge_type = CHARGER_CHARGE_TYPE_STANDARD; 
            break;
        }

        case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA: 
        {
			bq25792_get_charge_ilim (dev, &val->const_charge_current_ua); 
			val->const_charge_current_ua *= 1000; 
            break;
        }

        case CHARGER_PROP_PRECHARGE_CURRENT_UA:
        {
			bq25792_get_iprecharge (dev, &val->precharge_current_ua); 
			val->precharge_current_ua*=1000; 


            break;
        }

        case CHARGER_PROP_CHARGE_TERM_CURRENT_UA:
        {
			bq25792_get_iterm (dev, &val->charge_term_current_ua); 
			val->charge_term_current_ua *=  1000; 
            break;
        }
        case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV:
        {
			bq25792_get_vreg (dev, &val->const_charge_voltage_uv); 
			val->const_charge_voltage_uv *= 1000; 
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
// static int bq25792_set_property(const struct device *dev, charger_prop_t prop,const union charger_propval *val)
// ##################################################################
static int bq25792_set_property(const struct device *dev, charger_prop_t prop,const union charger_propval *val)
{

    switch (prop)
    {
        case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA:
        {
			bq25792_set_charge_ilim (dev, val->const_charge_current_ua/1000); 
            break;
        }

        case CHARGER_PROP_PRECHARGE_CURRENT_UA:
        {
			bq25792_set_iprecharge(dev, val->precharge_current_ua/1000); 
            break;
        }

        case CHARGER_PROP_CHARGE_TERM_CURRENT_UA:
        {
			bq25792_set_iterm (dev, val->charge_term_current_ua/1000); 
            break;
        }

        case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV:
        {
			bq25792_set_vreg(dev, val->const_charge_voltage_uv/1000); 
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
static const struct charger_driver_api bq25792_api = 
{
	.get_property = bq25792_get_property,
	.set_property = bq25792_set_property 
};

// ##################################################################
// Init 
// ##################################################################
static int bq25792_init(const struct device *dev)
{
	const struct bq25792_config *cfg = dev->config;
	uint8_t buf = 0; 
	if (cfg->ce_gpio.port != NULL) gpio_pin_configure_dt(&cfg->ce_gpio, GPIO_OUTPUT_INACTIVE); 
	if (cfg->irq_gpio.port != NULL) gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT); 

	// Setup Current Limit
	__ASSERT(device_is_ready(cfg->i2c.bus), "i2c not ready"); 

	// Check IDs
	bq25792_i2c_read (dev, BQ25792_PART_INFORMATION_ADDR, BQ25792_PART_INFORMATION_SIZE, &buf); 
	__ASSERT(BQ25792_PART_INFORMATION_PN_GET(buf) == 0x01, "id mismatch"); 
	LOG_INF("id: %02X", (unsigned int) BQ25792_PART_INFORMATION_PN_GET(buf)); 


	// Set Charge Current 
	bq25792_set_charge_ilim(dev, cfg->charge_ilim); 
	bq25792_set_input_ilim(dev, cfg->input_ilim); 
	bq25792_set_vindpm (dev, 4200); 

	uint32_t charge_ilim = 0; 
	bq25792_get_charge_ilim(dev, &charge_ilim); 
	LOG_INF("charge ilim: %u", charge_ilim); 
	charge_ilim = 0; 
	bq25792_get_input_ilim(dev, &charge_ilim); 
	LOG_INF("imput ilim: %u", charge_ilim);

	bq25792_chip_enable(dev, true); 

	k_msleep(100);

	uint32_t test = 0; 
	bq25792_get_vindpm (dev, &test);
	LOG_INF("BQ25792_INPUT_VOLTAGE_LIMIT_ADDR: %0u mV", test);

	uint8_t raw = 0; 
	bq25792_i2c_read (dev, BQ25792_CHARGER_STATUS_0_ADDR, BQ25792_CHARGER_STATUS_0_SIZE, &raw); 
	LOG_INF("BQ25792_CHARGER_STATUS_0_ADDR: %02X", raw);

	bq25792_i2c_read (dev, BQ25792_CHARGER_STATUS_2_ADDR, BQ25792_CHARGER_STATUS_2_SIZE, &raw); 
	LOG_INF("BQ25792_CHARGER_STATUS_2_ADDR: %02X", raw);

	return 0;
}


// ##################################################################
// Instantiation 
// ##################################################################
#define BQ25792_INIT(inst) \
	static const struct bq25792_config bq25792_config_##inst = { \
		.i2c = I2C_DT_SPEC_INST_GET(inst),\
		.ce_gpio = GPIO_DT_SPEC_INST_GET_BY_IDX_OR(inst, ce_gpios, 0, {0}),\
		.irq_gpio = GPIO_DT_SPEC_INST_GET_BY_IDX_OR(inst, irq_gpios, 0, {0}),\
		.charge_ilim = DT_INST_PROP(inst, charge_ilim),\
		.input_ilim = DT_INST_PROP(inst, input_ilim),\
	};\
	static struct bq25792_data bq25792_data_##inst;	\
	DEVICE_DT_INST_DEFINE(inst, bq25792_init, NULL, &bq25792_data_##inst, &bq25792_config_##inst, POST_KERNEL, CONFIG_CHARGER_INIT_PRIORITY, &bq25792_api);
DT_INST_FOREACH_STATUS_OKAY(BQ25792_INIT)