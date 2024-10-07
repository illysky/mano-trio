 /* 
  _   _                 _               
 | \ | |               | |              
 |  \| | ___   ___  ___| |__   ___  ___ 
 | . ` |/ _ \ / _ \/ __| '_ \ / _ \/ _ \
 | |\  | (_) | (_) \__ \ | | |  __/  __/
 |_| \_|\___/ \___/|___/_| |_|\___|\___|

 Module for Zephyr RTOS by Dominic Moffat [2023-08-10]
 v1.0.0
 Copyright (c) 2023 Nooshee LLC 
 */   
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>


#define LOG_MODULE_NAME application
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

const struct device* dev_i2c1 = 	DEVICE_DT_GET(DT_NODELABEL(i2c1));

void i2c_scan (const struct device *dev)
{
    int ret, i;
    for (i = 0; i < 0x7F; i++) 
    {
        uint8_t dummy_data;
        // Try to read from address 'i' to check if a device is present
        ret = i2c_read(dev, &dummy_data,  1, i); 
        //ret = i2c_reg_read_byte(dev,i,0x00, &dummy_data); 

        if (ret == 0) {
            LOG_INF("I2C Found: %02X on I2C", i); 
        } 
    }

    LOG_INF("I2C scan complete.\n");


}

const struct device* psensor_green = 	DEVICE_DT_GET(DT_NODELABEL(psensor_green));
const struct device* psensor_yellow = 	DEVICE_DT_GET(DT_NODELABEL(psensor_yellow));
const struct device* psensor_blue = 	DEVICE_DT_GET(DT_NODELABEL(psensor_blue));




int main(void)
{
	



	i2c_scan (dev_i2c1); 


	for (;;) {

		k_msleep(1000);
	}
}
