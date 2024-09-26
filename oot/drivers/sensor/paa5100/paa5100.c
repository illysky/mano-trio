/*
 * Copyright (c) 2019-2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#define DT_DRV_COMPAT pixart_paa5100
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include "paa5100.h"

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_instance.h>

#define LOG_MODULE_NAME 	paa5100
#define LOG_MODULE_LEVEL 	LOG_LEVEL_DBG

LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_MODULE_LEVEL);

/*** Initialisation Data  ***/
static const uint8_t cfg1_a[5] = {0x7F, 0x55, 0x50, 0x7F, 0x43};
static const uint8_t cfg1_d[5] = {0x00, 0x01, 0x07, 0x0E, 0x10};
static const uint8_t cfg2_a[5] = {0x7F, 0x51, 0x50, 0x55, 0x7F};
static const uint8_t cfg2_d[5] = {0x00, 0x7B, 0x00, 0x00, 0x0E};
static const uint8_t cfg3_a[67] = {0x7F, 0x61, 0x7F, 0x40, 0x7F, 0x41, 0x43, 0x45, 0x5F, 0x7B, 0x5E, 0x5B, 0x6D, 0x45, 0x70, 0x71, 0x7F, 0x44, 0x40, 0x4E, 0x7F, 0x66, 0x65, 0x6A, 0x61, 0x62, 0x7F, 0x4F, 0x5F, 0x48, 0x49, 0x57, 0x60, 0x61, 0x62, 0x63, 0x7F, 0x45, 0x7F, 0x4D, 0x55, 0x74, 0x75, 0x4A, 0x4B, 0x44, 0x45, 0x64, 0x65, 0x7F, 0x65, 0x66, 0x63, 0x6F, 0x7F, 0x48, 0x7F, 0x41, 0x43, 0x4B, 0x45, 0x44, 0x4C, 0x7F, 0x5B, 0x7F, 0x40};
static const uint8_t cfg3_d[67] = {0x00, 0xAD, 0x03, 0x00, 0x05, 0xB3, 0xF1, 0x14, 0x34, 0x08, 0x34, 0x11, 0x11, 0x17, 0xE5, 0xE5, 0x06, 0x1B, 0xBF, 0x3F, 0x08, 0x44, 0x20, 0x3A, 0x05, 0x05, 0x09, 0xAF, 0x40, 0x80, 0x80, 0x77, 0x78, 0x78, 0x08, 0x50, 0x0A, 0x60, 0x00, 0x11, 0x80, 0x21, 0x1F, 0x78, 0x78, 0x08, 0x50, 0xFF, 0x1F, 0x14, 0x67, 0x08, 0x70, 0x1C, 0x15, 0x48, 0x07, 0x0D, 0x14, 0x0E, 0x0F, 0x42, 0x80, 0x10, 0x02, 0x07, 0x41};
static const uint8_t cfg4_a[16] = {0x7F, 0x32, 0x7F, 0x40, 0x7F, 0x68, 0x69, 0x7F, 0x48, 0x6F, 0x7F, 0x5B, 0x4E, 0x5A, 0x40, 0x73};
static const uint8_t cfg4_d[16] = {0x00, 0x00, 0x07, 0x40, 0x06, 0xF0, 0x00, 0x0D, 0xC0, 0xD5, 0x00, 0xA0, 0xA8, 0x90, 0x80, 0x1F};

#define PAA5100_CS_WAIT_USEC 		5
#define PAA5100_DATA_WAIT_USEC 		10



// Data for each instance of the driver
struct paa5100_data 
{
	const struct device          *dev;					// Device structure for convenience 
	struct gpio_callback         irq_gpio_cb;			// A callback for the gpio module 
	struct k_work                kwork_irq_handler;		// The k_work structure for our the irq handler
	struct k_spinlock            lock;					// Mutex
	double                       x;						// Current delta of X 
	double                       y;						// Current delta of Y 
	sensor_trigger_handler_t     data_ready_handler;	// 
	double						 cpi; 
	bool 						 ready;					// Ready flag

};

struct paa5100_config 
{
	struct gpio_dt_spec irq_gpio;
	struct spi_dt_spec bus;
	struct gpio_dt_spec cs_gpio;
	LOG_INSTANCE_PTR_DECLARE(log);
};

/********************************************************************************************************
* @overview: 
********************************************************************************************************/
static int _cs_assert(const struct device *dev, bool assert)
{
	int err = 0;
	const struct paa5100_config *config = dev->config;
	if (!assert) k_busy_wait(PAA5100_CS_WAIT_USEC);
	err = gpio_pin_set_dt(&config->cs_gpio, (int)assert);
	if (err) 
	{
		LOG_ERR("no gpio binding for cs");
		return err;
	}
	if (assert) k_busy_wait(PAA5100_CS_WAIT_USEC);
	return err;
}

/********************************************************************************************************
* @overview: 
********************************************************************************************************/
static int _read (const struct device *dev, uint8_t reg, uint8_t *buf)
{
	int err = 0;
	const struct paa5100_config *config = dev->config;

	// Asset CS
	err = _cs_assert (dev, true);	
	if (err) return err;

	// Write Address
	reg &= ~0x80u; 
	const struct spi_buf tx_buf = {.buf = &reg, .len = 1};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	err = spi_write_dt(&config->bus, &tx);
	if (err) 
	{
		LOG_ERR("failed to write data");
		return err;
	}
	k_busy_wait(PAA5100_DATA_WAIT_USEC);
	
	// Read back data
	struct spi_buf rx_buf = {.buf = buf,.len = 1};
	const struct spi_buf_set rx = {.buffers = &rx_buf,.count = 1};
	err = spi_read_dt(&config->bus, &rx);
	if (err) 
	{
		LOG_ERR("failed to read data");
		return err;
	}

	/* Deassert nCS */
	err = _cs_assert (dev, false);
	if (err)return err;

	return 0;
}

/********************************************************************************************************
* @overview: 
********************************************************************************************************/
static int _read_array (const struct device *dev, uint8_t reg, uint8_t *buf, uint32_t size)
{
	int err = 0;
	const struct paa5100_config *config = dev->config;

	// Asset CS
	err = _cs_assert (dev, true);	
	if (err) return err;

	// Write Address
	reg &= ~0x80u; 
	const struct spi_buf tx_buf = {.buf = &reg, .len = 1};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	err = spi_write_dt(&config->bus, &tx);
	if (err) 
	{
		LOG_ERR("failed to write data");
		return err;
	}
	k_busy_wait(PAA5100_DATA_WAIT_USEC);
	
	// Read back data
	struct spi_buf rx_buf = {.buf = buf,.len = size};
	const struct spi_buf_set rx = {.buffers = &rx_buf,.count = 1};
	err = spi_read_dt(&config->bus, &rx);
	if (err) 
	{
		LOG_ERR("failed to read data");
		return err;
	}

	/* Deassert nCS */
	err = _cs_assert (dev, false);
	if (err)return err;

	return 0;
}


/********************************************************************************************************
* @overview: 
********************************************************************************************************/
static int _write(const struct device *dev, uint8_t reg, uint8_t val)
{
	int err;
	const struct paa5100_config *config = dev->config;

	// Assert CS
	err = _cs_assert(dev, true);
	if (err) return err;
	
	// Write Address + Data
	uint8_t buf[] = {0x80u | reg, val};
	const struct spi_buf tx_buf = {.buf = buf, .len = ARRAY_SIZE(buf)};
	const struct spi_buf_set tx = {.buffers = &tx_buf,.count = 1};
	err = spi_write_dt(&config->bus, &tx);
	if (err) {
		LOG_ERR("failed to write data");
		return err;
	}

	// Deassert CS
	err = _cs_assert(dev, false);
	if (err)return err;
	return 0;
}

/********************************************************************************************************
* @overview: gpio handler
********************************************************************************************************/
static void _gpio_handler(const struct device *gpiob, struct gpio_callback *cb, uint32_t pins)
{
	int err;
	struct paa5100_data *data = CONTAINER_OF(cb, struct paa5100_data, irq_gpio_cb);
	const struct device *dev = data->dev;
	const struct paa5100_config *config = dev->config;

	err = gpio_pin_interrupt_configure_dt(&config->irq_gpio,GPIO_INT_DISABLE);
	if (unlikely(err)) 
	{
		LOG_ERR("cannot disable irq");
		k_panic();
	}

	// Trigger the worker task for this
	k_work_submit(&data->kwork_irq_handler);
}

/********************************************************************************************************
* @overview: irq handler from paa51000
********************************************************************************************************/
static void _irq_handler (struct k_work *work)
{
	sensor_trigger_handler_t handler;
	int err = 0;
	struct paa5100_data *data = CONTAINER_OF(work, struct paa5100_data,kwork_irq_handler);
	const struct device *dev = data->dev;
	const struct paa5100_config *config = dev->config;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	handler = data->data_ready_handler;
	//LOG_INST_ERR(config->log,"irq! ");

	k_spin_unlock(&data->lock, key);




	if (!handler) {
		return;
	}

	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};



	handler(dev, &trig);

	key = k_spin_lock(&data->lock);
	if (data->data_ready_handler) {
		err = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
						      GPIO_INT_LEVEL_ACTIVE);
	}
	k_spin_unlock(&data->lock, key);

	if (unlikely(err)) {
		LOG_ERR("Cannot re-enable IRQ");
		k_panic();
	}
}

/********************************************************************************************************
* @overview: Second (async) initialisation  for paa5100 (called by paa5100_init)
********************************************************************************************************/
static int _config (const struct device *dev)
{
	uint8_t data = 0;
    int32_t c1 = 0; 
    int32_t c2 = 0;

	// Step 1: Send config data 1
    for (int i = 0; i < sizeof(cfg1_a); i++) _write (dev, (uint8_t) cfg1_a[i], cfg1_d[i]);

	// Step 2: WTF?
    _read (dev, 0x67, &data);
    data = data & 0x80u;
    if (data == 0x80u)  data = 0x04; 
    else                data = 0x02; 
    _write (dev,0x48, data);

	// Step 3: Send config data 2
    for (int i = 0; i < sizeof(cfg2_a); i++) _write (dev, (uint8_t) cfg2_a[i], cfg2_d[i]);

	// Step 4: This... secret bullshit?
    _read (dev, 0x73, &data);
    if (data == 0x00)
    {
        _read (dev, 0x70, &data);
        c1 = (int32_t)  data;
        if (c1 <= 28)   c1 += 14;
        else            c1 += 11;
        if (c1 > 63) c1 = 63;

        _read (dev, 0x71, &data);
        c2 = (int32_t)  data;
        c2 = (c2 * 45) / 100;
        data = 0x00;    _write (dev, 0x7F, data);
        data = 0xAD;    _write (dev, 0x61, data);
        data = 0x70;    _write (dev, 0x51, data);
        data = 0x0E;    _write (dev, 0x7F, data);
        _write (dev, 0x70, c1);
        _write (dev, 0x71, c2);
    }

	// Step 5: 
	for (int i = 0; i < sizeof(cfg3_a); i++) _write (dev, (uint8_t) cfg3_a[i], cfg3_d[i]);
	k_msleep(10);
    for (int i = 0; i < sizeof(cfg4_a); i++) _write (dev, (uint8_t) cfg4_a[i], cfg4_d[i]);
    k_msleep(10);
    data = 0x00;     _write (dev, 0x73, data);
    return 0; 
}

/********************************************************************************************************
* @overview: First initialisation point for paa5100 (called by kernel)
********************************************************************************************************/
static int _set_orientation (const struct device *dev, bool invert_x, bool invert_y, bool swap_xy)
{
    uint8_t val = 0x00;
    if (invert_x)  val |= 0x20; 
    if (invert_y)  val |= 0x40; 
    if (swap_xy)   val |= 0x80; 
	return _write(dev, PAA5100JE_ORIENTATION_ADDR, val); 	
}

/********************************************************************************************************
* @overview: First initialisation point for paa5100 (called by kernel)
********************************************************************************************************/
static int _set_resolution (const struct device *dev, uint8_t res)
{
    if (res >= 0xA8) res = 0xA8;
	return _write(dev, PAA5100JE_RESOLUTION_ADDR, res); 
}


/********************************************************************************************************
* @overview: First initialisation point for paa5100 (called by kernel)
********************************************************************************************************/
static void _set_working_height (const struct device *dev, double height)
{
	struct paa5100_data *data = dev->data;			
	if (height < 12.5) height = 12.5; 
	if (height > 37.5) height = 37.5; 

	// PixArt approximation formula from height to CPI nothing needs be sent to the device
    data->cpi = 11.914 * (1 / (height / 1000));
	return;   
}


/********************************************************************************************************
* @overview: First initialisation point for paa5100 (called by kernel)
********************************************************************************************************/
static int paa5100_init(const struct device *dev)
{
	int err=0;
	struct paa5100_data *data = dev->data;				// Data of the device instance
	const struct paa5100_config *config = dev->config;	// Config of the device instance
	data->dev = dev;									// Device is put into the data struct??

	// Check spi is ready
	if (!spi_is_ready(&config->bus))
	{
		LOG_INST_ERR(config->log, "spi not ready");
		return -ENODEV;
	}

	// cs setup
	if (!device_is_ready(config->cs_gpio.port)){
		LOG_INST_ERR(config->log,"cs gpio not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
	if (err) 
	{
		LOG_INST_ERR(config->log,"can't configure cs gpio");
		return err;
	}

	// irq setup
	if (!device_is_ready(config->irq_gpio.port)) 
	{
		LOG_INST_ERR(config->log,"irq gpio not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
	if (err) 
	{
		LOG_INST_ERR(config->log,"can't configure irq gpio");
		return err;
	}

	// irq setup
	k_work_init(&data->kwork_irq_handler, _irq_handler);								// set up irq_handler
	gpio_init_callback(&data->irq_gpio_cb, _gpio_handler, BIT(config->irq_gpio.pin));	// configure the gpio module to callback on this pin
	err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);					// add the callback. 
	if (err) 
	{
		LOG_INST_ERR(config->log,"cannot add gpio call back");
	}

	// Strobe to reset the PAA5100
	_cs_assert(dev, false); 
	k_msleep(1);
	_cs_assert(dev, true); 
	k_msleep(1);
	_cs_assert(dev, false); 
	k_msleep(1);

		// Powerup 
	_write(dev, PAA5100JE_POWER_UP_RESET_ADDR, 0x5A);
	k_msleep(150);

	// Read IDs
	uint8_t id0 = 0; 
	uint8_t id1 = 0; 
	_read(dev, PAA5100JE_PROD_ID_ADDR, &id0); 
	_read(dev, PAA5100JE_INV_PROD_ID_ADDR, &id1); 

	if (id0!=0x49 && id1!=0xB6)
	{
		LOG_INST_ERR(config->log,"paa5100 not found ");
		return -1; 
	}

	// Configure
	_config (dev); 

	_set_working_height (dev, 12.5); 

	// Signal that we are ready to go
	data->ready = true; 

	LOG_INST_INF(config->log,"paa5100 found (%02X%02X)", id0, id1);
	uint8_t buf[12] = {0}; 
	_read_array(dev, PAA5100JE_MOTION_BURST_ADDR, buf, 12); 
	LOG_INST_HEXDUMP_INF(config->log,buf,12,"data"); 


	_read_array(dev, PAA5100JE_MOTION_BURST_ADDR, buf, 12); 
	LOG_INST_HEXDUMP_INF(config->log,buf,12,"data"); 


	uint8_t v; 
	_read(dev, PAA5100JE_MOTION_ADDR, &v); 
	_read(dev, PAA5100JE_DELTA_XL_ADDR, &v); 
	_read(dev, PAA5100JE_DELTA_XH_ADDR, &v); 
	_read(dev, PAA5100JE_DELTA_YL_ADDR, &v); 
	_read(dev, PAA5100JE_DELTA_YH_ADDR, &v); 



	(void)_set_resolution; 
	(void)_set_orientation; 

	// err = gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_LEVEL_ACTIVE);

	return 0;
}


/********************************************************************************************************
* @overview: Fetch data from the PAA51000
********************************************************************************************************/
static int paa5100_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int err = 0; 
	struct paa5100_data *data = dev->data;
	const struct paa5100_config *config = dev->config;	// Config of the device instance

	if (unlikely(chan != SENSOR_CHAN_ALL)) 
	{
		return -ENOTSUP;
	}

	if (unlikely(!data->ready)) 
	{
		LOG_INST_ERR(config->log,"device not ready");
		return -EBUSY;
	}

	// TODO: Put in real data here.
	uint8_t buf[12] = {0}; 
	//_read_array(dev, PAA5100JE_MOTION_BURST_ADDR, buf, 12); 
	//LOG_INST_HEXDUMP_INF(config->log,buf,12,"data"); 

	uint8_t id0 = 0; 
	uint8_t id1 = 0; 
	_read(dev, PAA5100JE_PROD_ID_ADDR, &id0); 
	_read(dev, PAA5100JE_INV_PROD_ID_ADDR, &id1); 

	bool burst = true;
	if (burst)
	{
		_read_array(dev, PAA5100JE_MOTION_BURST_ADDR, buf, 12); 
		if (buf[0] & 0x80 && buf[6] >= 0x19 && buf[10] != 0x1F)
		{
			int16_t raw_dx = ((int16_t)buf[3] << 8) | buf[2];
			int16_t raw_dy = ((int16_t)buf[5] << 8) | buf[4]; 

			double dx =  (double) raw_dx * (25.4 / data->cpi); 
			double dy =  (double) raw_dy * (25.4 / data->cpi); 
		
			data->x = data->x + dx; 
			data->y = data->y + dy; 

			char b[32]; 
			sprintf(b, "Burst: X: %.1f Y: %.1f", data->x, data->y ); 
			LOG_INST_ERR(config->log,"%s", b);
			return 0;
		}
	}
	else 
	{
		uint8_t motion=0;
		_read(dev, PAA5100JE_MOTION_ADDR, &motion); 
		motion &= 0x80; 
		if (motion & 0x80)
		{	

			_read(dev, PAA5100JE_DELTA_XL_ADDR, &buf[0]); 
			_read(dev, PAA5100JE_DELTA_XH_ADDR, &buf[1]); 
			_read(dev, PAA5100JE_DELTA_YL_ADDR, &buf[2]); 
			_read(dev, PAA5100JE_DELTA_YH_ADDR, &buf[3]); 

			int16_t raw_dx = ((int16_t)buf[1] << 8) | buf[0];
			int16_t raw_dy = ((int16_t)buf[3] << 8) | buf[2]; 

			double dx =  (double) raw_dx * (25.4 / data->cpi); 
			double dy =  (double) raw_dy * (25.4 / data->cpi); 
		
			data->x = data->x + dx; 
			data->y = data->y + dy; 

			char b[32]; 
			sprintf(b, "Normal: X: %.1f Y: %.1f", data->x, data->y ); 
			LOG_INST_ERR(config->log,"%s", b);
			return 0;
		}
	}
	return err;
}

/********************************************************************************************************
* @overview: 
********************************************************************************************************/
static int paa5100_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct paa5100_data *data = dev->data;
	const struct paa5100_config *config = dev->config;	// Config of the device instance

	if (unlikely(!data->ready)) 
	{
		LOG_INST_ERR(config->log,"device not ready");
		return -EBUSY;
	}

	// Get the data from the instance
	switch (chan) 
	{
		case SENSOR_CHAN_POS_DX:
		{
			val->val1 = data->x;
			val->val2 = 0;
		}
		break;
		case SENSOR_CHAN_POS_DY:
		{
			val->val1 = data->y;
			val->val2 = 0;
		}
		break;
		default:
		{

			
		}
		return -ENOTSUP;
	}
	return 0;
}

/********************************************************************************************************
* @overview:  enable the irq for paa5100
********************************************************************************************************/
static int paa5100_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler)
{
	int err;
	struct paa5100_data *data = dev->data;
	const struct paa5100_config *config = dev->config;


	if (unlikely(trig->type != SENSOR_TRIG_DATA_READY)) 
	{
		return -ENOTSUP;
	}

	if (unlikely(trig->chan != SENSOR_CHAN_ALL)) 
	{
		return -ENOTSUP;
	}

	if (unlikely(!data->ready)) 
	{
		LOG_DBG("not initialized");
		return -EBUSY;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if (handler) 
	{
		err = gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_LEVEL_ACTIVE);
	} 
	else 
	{
		err = gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_DISABLE);
	}

	if (!err) 
	{
		data->data_ready_handler = handler;
	}

	k_spin_unlock(&data->lock, key);

	return err;
}

/********************************************************************************************************
* @overview:  enable the irq for paa5100
********************************************************************************************************/
static int paa5100_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
{
	struct paa5100_data *data = dev->data;
	int err;

	if (unlikely(chan != SENSOR_CHAN_ALL)) 
	{
		return -ENOTSUP;
	}

	if (unlikely(!data->ready)) 
	{
		LOG_DBG("not initialized");
		return -EBUSY;
	}

	switch ((uint32_t)attr) 
	{
		default:
		{
			LOG_ERR("Unknown attribute");
			return -ENOTSUP;
		}
	}
	return err;
}

static const struct sensor_driver_api paa5100_driver_api = 
{
	.sample_fetch = paa5100_sample_fetch,
	.channel_get  = paa5100_channel_get,
	.trigger_set  = paa5100_trigger_set,
	.attr_set     = paa5100_attr_set,
};


#define PAA5100_DEFINE(n)						       						\
	LOG_INSTANCE_REGISTER(LOG_MODULE_NAME, n, LOG_MODULE_LEVEL);			\
	static struct paa5100_data data##n;				      					\
	static const struct paa5100_config config##n = 							\
	{																		\
		.irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),	    			\
		.bus = {						       								\
			.bus = DEVICE_DT_GET(DT_INST_BUS(n)),		        			\
			.config = {					       								\
				.frequency = DT_INST_PROP(n,		       					\
							  spi_max_frequency),  							\
				.operation = SPI_WORD_SET(8) |		       					\
					     SPI_TRANSFER_MSB |		       						\
					     SPI_MODE_CPOL | SPI_MODE_CPHA,    					\
				.slave = DT_INST_REG_ADDR(n),		       					\
			},						       									\
		},							       									\
		.cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),				\
		LOG_INSTANCE_PTR_INIT(log, LOG_MODULE_NAME, n) 						\
	};								       									\
									       									\
	DEVICE_DT_INST_DEFINE(n, paa5100_init, NULL, &data##n, &config##n,  	\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,	       			\
			      &paa5100_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PAA5100_DEFINE)




