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
#include <zephyr/drivers/led.h>

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>


#define LOG_MODULE_NAME application
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);


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


/*


[00:00:20.628,662] <inf> application: I2C Found: 15 on I2C -> Accelerometer 
[00:00:20.630,432] <inf> application: I2C Found: 1C on I2C -> Awinic (Broadcast)
[00:00:20.635,375] <inf> application: I2C Found: 34 on I2C -> Awinic (Direct)
[00:00:20.641,204] <inf> application: I2C Found: 51 on I2C -> RTCC
[00:00:20.646,514] <inf> application: I2C Found: 6B on I2C -> BQ25792
[00:00:20.647,369] <inf> application: I2C Found: 6D on I2C -> Pressure Sensor

*/

//const struct device* dev_i2c0 = 	DEVICE_DT_GET(DT_NODELABEL(i2c0)); 
//const struct device* dev_i2c1 = 	DEVICE_DT_GET(DT_NODELABEL(i2c1));
//static const struct gpio_dt_spec i2c_sel = GPIO_DT_SPEC_GET(DT_NODELABEL(i2c_sel), gpios);
//static const struct gpio_dt_spec sda = GPIO_DT_SPEC_GET(DT_NODELABEL(sda), gpios);
//static const struct gpio_dt_spec scl = GPIO_DT_SPEC_GET(DT_NODELABEL(scl), gpios);





// ####################################################################################
// @brief	
// ####################################################################################
int main(void)
 {

    //gpio_pin_configure_dt(&i2c_sel, GPIO_OUTPUT_INACTIVE);
    //gpio_pin_configure_dt(&scl, GPIO_OUTPUT_INACTIVE);

    // gpio_pin_set_dt(&i2c_sel, false);
    //gpio_pin_set_dt(&scl, true);
    
    //LOG_INF("I2C0");
    //i2c_scan (dev_i2c0); 

    //LOG_INF("I2C1");
    //i2c_scan (dev_i2c1); 

    k_msleep(10);
    return 0; 
}


