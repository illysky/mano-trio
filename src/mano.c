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
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "mano.h"


// ####################################
// Logging
// ####################################
#define LOG_MODULE_NAME mano
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

// ####################################
// Task
// ####################################
#define STACK_SIZE 1024
#define PRIORITY 5
K_THREAD_STACK_DEFINE(mano_thread_stack, STACK_SIZE);
struct k_thread mano_thread_data;


#define QUEUE_SIZE 10
#define COMMAND_QUEUE_ALIGNMENT 4
K_MSGQ_DEFINE(mano_command_queue, sizeof(mano_command_t), QUEUE_SIZE, COMMAND_QUEUE_ALIGNMENT);


// ####################################################################################
// 
// ####################################################################################
void mano_task(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("task started");
    while (1)
    {
        mano_command_t msg = {0}; 
        if (k_msgq_get(&mano_command_queue, &msg, K_NO_WAIT) == 0) 
        {
            LOG_HEXDUMP_INF(&msg, sizeof(msg), "");  
        }
        k_msleep(10); 
    }
}

// ####################################################################################
// 
// ####################################################################################
int mano_task_init(void)
{
    // Start the thread
    k_tid_t tid = k_thread_create(&mano_thread_data, mano_thread_stack, STACK_SIZE, mano_task, NULL, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
    if (!tid) 
    {
        LOG_ERR("task failed to start"); 
        return -1; 
    }
    return 0; 
}

SYS_INIT(mano_task_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);