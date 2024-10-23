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
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <cJSON.h>
#include <string.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include "mano.h"
#include "data.h"



// ####################################
// Logging
// ####################################
#define LOG_MODULE_NAME mano
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

// ####################################
// Task
// ####################################
#define STACK_SIZE 4096
#define PRIORITY 5
K_THREAD_STACK_DEFINE(mano_thread_stack, STACK_SIZE);
struct k_thread mano_thread_data;

// Store Settings
#define NVS_PARTITION              nvs_storage
#define NVS_PARTITION_ID           FIXED_PARTITION_DEVICE (NVS_PARTITION)
#define NVS_PARTITION_OFFSET	   FIXED_PARTITION_OFFSET(NVS_PARTITION)



#include <stdint.h>
#include <stdio.h>

static bool mano_log_status = 0; 

struct k_timer mano_interval;

bool update_calibration_offset = false;

#define NVS_CALIBRATION_OFFSET_GRN 1
#define NVS_CALIBRATION_OFFSET_YEL 2
#define NVS_CALIBRATION_OFFSET_BLU 3
double mano_calibration_offset_grn = 0;
double mano_calibration_offset_yel = 0;
double mano_calibration_offset_blu = 0;


struct k_work sensor_work;
void mano_interval_handler (struct k_timer *logging); 


struct k_sem sensor_calib_sem;

const struct device *rtc_dev = DEVICE_DT_GET(DT_NODELABEL(rtcc));
const struct device* psensor_green = 	DEVICE_DT_GET(DT_NODELABEL(psensor_green));
const struct device* psensor_yellow = 	DEVICE_DT_GET(DT_NODELABEL(psensor_yellow));
const struct device* psensor_blue = 	DEVICE_DT_GET(DT_NODELABEL(psensor_blue));

static struct nvs_fs nvs;


// ####################################################################################
// 
// ####################################################################################
static void mano_get_pressure_offset (uint32_t id, double* val)
{
	if (nvs_read(&nvs, id, val, sizeof(double)) > 0) 
    { 
		LOG_INF ("Retrieving calibration offset for channel (%d): %f mmHg", id, *val);
	} 
	else   
    {
		*val = 0; 
        LOG_WRN ("Calibration offset (%d) missing, setting default", id);
		nvs_write(&nvs, id, val, sizeof(double));
	}
}

// ####################################################################################
// 
// ####################################################################################
static void mano_set_pressure_offset (uint32_t id, double* val)
{
    if (nvs_write(&nvs, id, val, sizeof(double)) >= 0)
    {
        LOG_INF ("Updating calibration offset (%d): %f mmHg", id, *val);
    }
    else 
    {
        LOG_ERR ("Error updating calibration offset (%d): %f mmHg", id, *val);
    }
}



static int init_flash_nvs (void)
{
	int rc = 0;
	struct flash_pages_info info;

	nvs.flash_device =  NVS_PARTITION_ID;
	if (!device_is_ready(nvs.flash_device)) {
		printk("NVS device %s is not ready", nvs.flash_device->name);
		return 0;
	}
	
	nvs.offset = NVS_PARTITION_OFFSET;
	rc = flash_get_page_info_by_offs(nvs.flash_device, nvs.offset, &info);
	if (rc) {
		printk("Unable to get page info, rc=%d", rc);
		return 0;
	}
	nvs.sector_size = info.size;
	nvs.sector_count = 3U;

	rc = nvs_mount(&nvs);
	if (rc) {
		printk("Flash Init failed, rc=%d", rc);
		return 0;
	}
	return rc;
}

void read_pressure (const struct device *sensor, double* pressure_f) 
{
    if (!device_is_ready(sensor)) {
        LOG_ERR("Sensor is not ready\n");
        return;
    }

    int ret = sensor_sample_fetch(sensor);
    if (ret) {
        LOG_ERR("Failed to fetch sample:  %d\n", ret);
        return;
    }

    struct sensor_value pressure;
    ret = sensor_channel_get(sensor, SENSOR_CHAN_PRESS, &pressure);
    if (ret) {
        LOG_ERR("Failed to get pressure value: %d\n", ret);
        return;
    }

    (*pressure_f) = sensor_value_to_float(&pressure);
}



void read_sensor_data_work(struct k_work *work)
{
    double p_grn = 0; 
    double p_yel = 0;
    double p_blu = 0;

    read_pressure(psensor_green, &p_grn);
    read_pressure(psensor_yellow, &p_yel);
    read_pressure(psensor_blue, &p_blu);

    // Check if we have triggered calibration
    if (k_sem_take(&sensor_calib_sem, K_NO_WAIT) == 0) 
    {
        update_calibration_offset = false; 
        mano_calibration_offset_grn = p_grn; 
        mano_calibration_offset_yel = p_yel; 
        mano_calibration_offset_blu = p_blu; 

        mano_set_pressure_offset(NVS_CALIBRATION_OFFSET_GRN, &mano_calibration_offset_grn); 
        mano_set_pressure_offset(NVS_CALIBRATION_OFFSET_YEL, &mano_calibration_offset_yel); 
        mano_set_pressure_offset(NVS_CALIBRATION_OFFSET_BLU, &mano_calibration_offset_blu); 
    } 

    // Remove Offset
    p_grn = p_grn - mano_calibration_offset_grn; 
    p_yel = p_yel - mano_calibration_offset_yel; 
    p_blu = p_blu - mano_calibration_offset_blu; 

    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "stream", k_uptime_get()); 
    cJSON_AddNumberToObject(json, "chan_g", p_grn);  
    cJSON_AddNumberToObject(json, "chan_y", p_yel);  
    cJSON_AddNumberToObject(json, "chan_b", p_blu);  
    cJSON_AddStringToObject(json, "units", "mmHg");  
    char *output = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);

    if (output)
    {        
        k_msgq_put(&data_out_queue, output, K_NO_WAIT); 
        free(output);
    }
    k_yield(); 

}


/** 
 * @brief Set the device time.
 * 
 * @param args Array containing time data as follows:
 * - args[0]: Year
 * - args[1]: Month
 * - args[2]: Day
 * - args[3]: Hour
 * - args[4]: Minute
 * - args[5]: Second
 */

static char* set_time(cJSON *json) 
{


    cJSON *json_out = cJSON_CreateObject();
    char *json_str = NULL; 
    cJSON_AddNumberToObject(json_out, "response", CMD_SET_TIME); 
    
    cJSON *year = cJSON_GetObjectItem(json, "year");
    cJSON *month = cJSON_GetObjectItem(json, "month");
    cJSON *day = cJSON_GetObjectItem(json, "day");
    cJSON *hour = cJSON_GetObjectItem(json, "hour");
    cJSON *minute = cJSON_GetObjectItem(json, "minute");
    cJSON *second = cJSON_GetObjectItem(json, "second");
    if (!cJSON_IsNumber(year)   || !cJSON_IsNumber(month)   || 
        !cJSON_IsNumber(day)    || !cJSON_IsNumber(hour)    || 
        !cJSON_IsNumber(minute) || !cJSON_IsNumber(second))
    {
        printf("Error: Invalid time fields in CMD_SET_TIME\n");
        cJSON_AddStringToObject(json_out, "result", "error"); 
        cJSON_AddStringToObject(json_out, "reason", "date format incorrect"); 
        goto finish;
    }

    struct rtc_time new_time = {
        .tm_sec = second->valueint,
        .tm_min = minute->valueint,
        .tm_hour = hour->valueint,
        .tm_mday = day->valueint,
        .tm_mon = month->valueint - 1,         /* tm_mon is 0-based (0 = January) */
        .tm_year = year->valueint - 1900,      /* tm_year is year since 1900 */
        .tm_nsec = 0                          /* Set nanoseconds to 0 */
    };

    if (!device_is_ready(rtc_dev)) 
    {
        LOG_ERR("RTCC device not ready");
        cJSON_AddStringToObject(json_out, "result", "error"); 
        cJSON_AddStringToObject(json_out, "reason", "RTCC hardware error"); 
        goto finish;
    }

    int ret = rtc_set_time(rtc_dev, &new_time); 
    if (ret) 
    {
        LOG_ERR("Failed to set time on RTC, error: %d", ret);
        cJSON_AddStringToObject(json_out, "result", "error"); 
        cJSON_AddStringToObject(json_out, "reason", "RTCC hardware error"); 
        goto finish;
    }
    
    LOG_INF("Set Time Command: %d-%02d-%02d %02d:%02d:%02d\n",
           year->valueint, month->valueint, day->valueint,
           hour->valueint, minute->valueint, second->valueint);
    cJSON_AddStringToObject(json_out, "result", "ok"); 
    finish: 
        json_str = cJSON_PrintUnformatted(json_out);
        cJSON_Delete(json_out);
        return json_str;
}

/** 
 * @brief Get the current device time.
 * 
 * @note No parameters are needed.
 */
static char* get_time(void) 
{
    LOG_INF("Getting current time\n");

    if (!device_is_ready(rtc_dev)) 
    {
        LOG_ERR("RTCC device not ready");
        return NULL;
    }

    struct rtc_time current_time;
    int ret = rtc_get_time(rtc_dev, &current_time);
    if (ret) 
    {
        LOG_ERR("Failed to get time from RTC, error: %d", ret);
        return NULL;
    }

    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "response", CMD_GET_TIME); 
    cJSON_AddStringToObject(json, "result", "ok"); 
    cJSON_AddNumberToObject(json, "year", current_time.tm_year + 1900);  // Convert to full year
    cJSON_AddNumberToObject(json, "month", current_time.tm_mon + 1);     // Convert to 1-based month
    cJSON_AddNumberToObject(json, "day", current_time.tm_mday);
    cJSON_AddNumberToObject(json, "hour", current_time.tm_hour);
    cJSON_AddNumberToObject(json, "minute", current_time.tm_min);
    cJSON_AddNumberToObject(json, "second", current_time.tm_sec);
    char *json_str = cJSON_PrintUnformatted(json);
    // Clean up the JSON object (json_str needs to be freed by the caller)
    cJSON_Delete(json);
    return json_str;
}

/** 
 * @brief Zero the pressure sensor.
 * 
 * @note No parameters are needed.
 */
 static char* zero_pressure_sensor(void) 
{
    LOG_INF("Zeroing pressure sensor");


    // Zero here if not running
    if (mano_log_status == false)
    {
        double p_grn = 0; 
        double p_yel = 0;
        double p_blu = 0;

        read_pressure(psensor_green, &p_grn);
        read_pressure(psensor_yellow, &p_yel);
        read_pressure(psensor_blue, &p_blu);

        mano_calibration_offset_grn = p_grn; 
        mano_calibration_offset_yel = p_yel; 
        mano_calibration_offset_blu = p_blu; 

        mano_set_pressure_offset(NVS_CALIBRATION_OFFSET_GRN, &mano_calibration_offset_grn); 
        mano_set_pressure_offset(NVS_CALIBRATION_OFFSET_YEL, &mano_calibration_offset_yel); 
        mano_set_pressure_offset(NVS_CALIBRATION_OFFSET_BLU, &mano_calibration_offset_blu); 

    }
    // Do it on the next read
    else 
    {
        k_sem_give(&sensor_calib_sem);
    }

    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "response", CMD_ZERO_PRESSURE_SENSOR); 
    cJSON_AddStringToObject(json, "result", "ok"); 
    char *json_str = cJSON_PrintUnformatted(json);
    // Clean up the JSON object (json_str needs to be freed by the caller)
    cJSON_Delete(json);
    return json_str;
}

/** 
 * @brief Start logging data at a specified interval.
 * 
 * @param interval Logging interval in milliseconds.
 */
static char* start_logging(cJSON* json) 
{
    // Response
    cJSON *jout = cJSON_CreateObject();
    cJSON_AddNumberToObject(jout, "response", CMD_START_LOGGING); 

    cJSON *interval = cJSON_GetObjectItem(json, "interval");
    if (interval && cJSON_IsNumber(interval))
    {
        LOG_INF("Starting logging with interval: %d ms", interval->valueint);
        cJSON_AddStringToObject(jout, "result", "ok"); 

        if (mano_log_status == false)
        {
            k_timer_start(&mano_interval, K_MSEC(interval->valueint), K_MSEC(interval->valueint));
            mano_log_status = true; 
        }
    } 
    else 
    {
        LOG_WRN("Starting logging with interval: error, invalid interval");
        cJSON_AddStringToObject(jout, "result", "error"); 
        cJSON_AddStringToObject(jout, "reason", "invalid interval"); 
    }

    char *jout_str = cJSON_PrintUnformatted(jout);
    // Clean up the JSON object (json_str needs to be freed by the caller)
    cJSON_Delete(jout);
    return jout_str;
}

/** 
 * @brief Stop logging data.
 * 
 * @note No parameters are needed.
 */
static char* stop_logging(void) 
{
    LOG_INF("Stopping logging");

    k_timer_stop(&mano_interval); 
    mano_log_status = false; 
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "response", CMD_STOP_LOGGING); 
    cJSON_AddStringToObject(json, "result", "ok"); 
    char *json_str = cJSON_PrintUnformatted(json);
    // Clean up the JSON object (json_str needs to be freed by the caller)
    cJSON_Delete(json);
    return json_str;
}

/** 
 * @brief Perform a hardware check.
 * 
 * @note No parameters are needed.
 */
static char* hardware_check(void) 
{
    LOG_INF("Performing hardware check");
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "response", CMD_HARDWARE_CHECK); 
    cJSON_AddStringToObject(json, "result", "ok"); 

    cJSON_AddNumberToObject(json, "bq25792", 1);
    cJSON_AddNumberToObject(json, "hsc0", 1);  
    cJSON_AddNumberToObject(json, "hsc1", 1);  
    cJSON_AddNumberToObject(json, "hsc3", 1);  
    cJSON_AddNumberToObject(json, "pcf85063", 1);  
    cJSON_AddNumberToObject(json, "fs", 1);
    cJSON_AddNumberToObject(json, "btn0", 1);
    cJSON_AddNumberToObject(json, "btn1", 1);
    char *json_str = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    return json_str;
}

/** 
 * @brief Perform a hardware check.
 * 
 * @note No parameters are needed.
 */
static char* get_battery(void) 
{
    LOG_INF("Performing hardware check");
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "response", CMD_GET_BATTERY); 
    cJSON_AddStringToObject(json, "result", "ok"); 
    cJSON_AddNumberToObject(json, "soc", 100);
    char *json_str = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    return json_str;
}

/** 
 * @brief Perform a factory reset.
 * 
 * @note No parameters are needed.
 */
static char* factory_reset(void) 
{
    LOG_INF("Performing factory reset");
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "response", CMD_FACTORY_RESET); 
    cJSON_AddStringToObject(json, "result", "ok"); 
    char *json_str = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    return json_str;
}

/** 
 * @brief Perform a single read from a specified sensor.
 * 
 * @param sensor_id The ID of the sensor to read:
 * - 0: Temperature sensor
 * - 1: Humidity sensor
 * - 2: Pressure sensor
 */
static char* single_read(void) 
{
    LOG_INF("Single read from sensors");
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "response", CMD_SINGLE_READ); 
    cJSON_AddStringToObject(json, "result", "ok"); 
    cJSON_AddNumberToObject(json, "hsc0", 1);  
    cJSON_AddNumberToObject(json, "hsc1", 1);  
    cJSON_AddNumberToObject(json, "hsc3", 1);  
    char *json_str = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    return json_str;
}



static void mano_parse_command(char *json_str) 
{
    char* output = NULL; 
   
    // Parse the JSON string
    cJSON *json = cJSON_Parse(json_str);
    if (json == NULL) {
        printf("Error: Failed to parse JSON!\n");
        return;
    }

    // Get the "cmd" field from the JSON
    cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
    if (!cJSON_IsNumber(cmd_item)) {
        printf("Error: Invalid or missing 'cmd' field!\n");
        cJSON_Delete(json);
        return;
    }

    int cmd_code = cmd_item->valueint;
    // Switch based on the command code
    switch (cmd_code)
     {
        case CMD_SET_TIME:
        {
            output = set_time(json);  
            break;
        }

        case CMD_GET_TIME:
        {
            output = get_time(); 
            break;
        }

        case CMD_ZERO_PRESSURE_SENSOR:
        {
            output = zero_pressure_sensor(); 
            break;
        }

        case CMD_START_LOGGING:
        {
            output = start_logging(json); 
            break;
        }

        case CMD_STOP_LOGGING:
        {
            output = stop_logging(); 
            break;
        }

        case CMD_HARDWARE_CHECK:
        {
            output = hardware_check(); 
            break;
        }

        case CMD_FACTORY_RESET:
        {
            output = factory_reset(); 
            break;
        }

        case CMD_GET_BATTERY:
        {
            output = get_battery(); 
            break;
        }

        case CMD_SINGLE_READ:
        {
            output = single_read(); 
            break;
        }
        default:
        {
            LOG_WRN("Error: Unknown command code %d", cmd_code);
            cJSON *json_out = cJSON_CreateObject();
            cJSON_AddNumberToObject(json_out, "response", cmd_code); 
            cJSON_AddStringToObject(json_out, "result", "error"); 
            cJSON_AddStringToObject(json_out, "reason", "unknown command"); 
            output = cJSON_PrintUnformatted(json_out);
            cJSON_Delete(json_out);
            break;
        }
    }

    if (output)
    {        
        k_msgq_put(&data_out_queue, output, K_NO_WAIT); 
        free(output);
    }

    // Clean up cJSON object
    cJSON_Delete(json);
}





// ####################################################################################
// 
// ####################################################################################
void mano_interval_handler (struct k_timer *logging)
{
    k_work_submit(&sensor_work);  // Schedule the work on the system workqueue
}




// ####################################################################################
// 
// ####################################################################################
void mano_task(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("task started");

    k_sem_init(&sensor_calib_sem, 0, 1);

    k_timer_init(&mano_interval, mano_interval_handler, NULL);
    k_work_init(&sensor_work, read_sensor_data_work);


	init_flash_nvs(); 
    mano_get_pressure_offset (NVS_CALIBRATION_OFFSET_GRN, &mano_calibration_offset_grn); 
    mano_get_pressure_offset (NVS_CALIBRATION_OFFSET_YEL, &mano_calibration_offset_yel); 
    mano_get_pressure_offset (NVS_CALIBRATION_OFFSET_BLU, &mano_calibration_offset_blu); 


    while (1)
    {
 
        char json_msg[DATA_JSON_STRING_MAX_SIZE] = {0};
        /* Dequeue the JSON string from the message queue */
        if (k_msgq_get(&data_in_queue, json_msg, K_NO_WAIT) == 0)
        {
            mano_parse_command(json_msg); 
            LOG_INF("Received JSON message: %s", json_msg);

            memset(json_msg, 0, sizeof(json_msg)); 
            /* Process the JSON message (you could use a JSON parser here) */
        }

        k_msleep(1); 
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