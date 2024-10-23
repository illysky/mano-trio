 /* 
 ____  __    __   _  _  ___  _  _  _  _ 
(_  _)(  )  (  ) ( \/ )/ __)( )/ )( \/ )
 _)(_  )(__  )(__ \  / \__ \ )  (  \  / 
(____)(____)(____)(__) (___/(_)\_) (__) 

 Module for Zephyr RTOS by Dominic Moffat [2023-08-10]
 v1.0.0
 Copyright (c) 2023 Nooshee LLC 
 */   

#ifndef ZEPHYR_INCLUDE_DATA_H_
#define ZEPHYR_INCLUDE_DATA_H_

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


#define DATA_UART_BUF_SIZE 512
#define DATA_JSON_STRING_MAX_SIZE DATA_UART_BUF_SIZE  
#define DATA_IN_QUEUE_SIZE 1
#define DATA_IN_QUEUE_ALIGNMENT 1
#define DATA_OUT_QUEUE_SIZE 10
#define DATA_OUT_QUEUE_ALIGNMENT 1


extern struct k_msgq data_in_queue; 
extern struct k_msgq data_out_queue; 
extern struct k_msgq data_settings_queue; 



#ifdef __cplusplus
}
#endif
#endif 
