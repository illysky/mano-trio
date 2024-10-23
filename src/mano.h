 /* 
 ____  __    __   _  _  ___  _  _  _  _ 
(_  _)(  )  (  ) ( \/ )/ __)( )/ )( \/ )
 _)(_  )(__  )(__ \  / \__ \ )  (  \  / 
(____)(____)(____)(__) (___/(_)\_) (__) 

 Module for Zephyr RTOS by Dominic Moffat [2023-08-10]
 v1.0.0
 Copyright (c) 2023 Nooshee LLC 
 */   

#ifndef ZEPHYR_INCLUDE_MANO_H_
#define ZEPHYR_INCLUDE_MANO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>


// Enum for command codes
typedef enum {
    CMD_SET_TIME = 0x01,
    CMD_GET_TIME = 0x02,
    CMD_ZERO_PRESSURE_SENSOR = 0x03,
    CMD_START_LOGGING = 0x04,
    CMD_STOP_LOGGING = 0x05,
    CMD_HARDWARE_CHECK = 0x06,
    CMD_GET_BATTERY = 0x07,
    CMD_FACTORY_RESET = 0x08,
    CMD_SINGLE_READ = 0x09
} mano_command_code_t;




#ifdef __cplusplus
}
#endif
#endif 
