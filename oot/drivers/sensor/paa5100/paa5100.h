/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#ifndef ZEPHYR_INCLUDE_PAA5100_H_
#define ZEPHYR_INCLUDE_PAA5100_H_

/**
 * @file paa5100.h
 *
 * @brief Header file for the paa5100 driver.
 */

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif



enum paa5100_attribute {
	/** Sensor CPI for both X and Y axes. */
	PAA5100_ATTR_CPI = SENSOR_ATTR_PRIV_START,

	/** Enable or disable sleep modes. */
	PAA5100_ATTR_REST_ENABLE,

	/** Entering time from Run mode to REST1 mode [ms]. */
	PAA5100_ATTR_RUN_DOWNSHIFT_TIME,

	/** Entering time from REST1 mode to REST2 mode [ms]. */
	PAA5100_ATTR_REST1_DOWNSHIFT_TIME,

	/** Entering time from REST2 mode to REST3 mode [ms]. */
	PAA5100_ATTR_REST2_DOWNSHIFT_TIME,

	/** Sampling frequency time during REST1 mode [ms]. */
	PAA5100_ATTR_REST1_SAMPLE_TIME,

	/** Sampling frequency time during REST2 mode [ms]. */
	PAA5100_ATTR_REST2_SAMPLE_TIME,

	/** Sampling frequency time during REST3 mode [ms]. */
	PAA5100_ATTR_REST3_SAMPLE_TIME,
};


/*** Typedefs ***/
#define PAA5100JE_PROD_ID_ADDR 0x00
#define PAA5100JE_PROD_ID_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned id:8;
    }; 
} paa5100je_prod_id_t;

#define PAA5100JE_REV_ID_ADDR 0x01
#define PAA5100JE_REV_ID_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned id:8;
    }; 
} paa5100je_rev_id_t;

#define PAA5100JE_INV_PROD_ID_ADDR 0x5F
#define PAA5100JE_INV_PROD_ID_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned id:8;
    }; 
} paa5100je_inv_prod_id_t;

#define PAA5100JE_MOTION_ADDR 0x02
#define PAA5100JE_MOTION_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned motion:8;
    }; 
} paa5100je_motion_t;

#define PAA5100JE_DELTA_XL_ADDR 0x03
#define PAA5100JE_DELTA_XL_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned delta_l:8;
    }; 
} paa5100je_delta_xl_t;

#define PAA5100JE_DELTA_XH_ADDR 0x04
#define PAA5100JE_DELTA_XH_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned delta_h:8;
    }; 
} paa5100je_delta_xh_t;

#define PAA5100JE_DELTA_YL_ADDR 0x05
#define PAA5100JE_DELTA_YL_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned delta_l:8;
    }; 
} paa5100je_delta_yl_t;

#define PAA5100JE_DELTA_YH_ADDR 0x06
#define PAA5100JE_DELTA_YH_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned delta_h:8;
    }; 
} paa5100je_delta_yh_t;

#define PAA5100JE_SQUAL_ADDR 0x07
#define PAA5100JE_SQUAL_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned sq:8;
    }; 
} paa5100je_squal_t;

#define PAA5100JE_RAW_DATA_SUM_ADDR 0x08
#define PAA5100JE_RAW_DATA_SUM_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned sum:8;
    }; 
} paa5100je_raw_data_sum_t;

#define PAA5100JE_MAXIMUM_RAWDATA_ADDR 0x09
#define PAA5100JE_MAXIMUM_RAWDATA_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned max:8;
    }; 
} paa5100je_maximum_rawdata_t;

#define PAA5100JE_MINIMUM_RAWDATA_ADDR 0x0A
#define PAA5100JE_MINIMUM_RAWDATA_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned min:8;
    }; 
} paa5100je_minimum_rawdata_t;

#define PAA5100JE_SHUTTER_L_ADDR 0x0B
#define PAA5100JE_SHUTTER_L_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned shutter_l:8;
    }; 
} paa5100je_shutter_l_t;

#define PAA5100JE_SHUTTER_H_ADDR 0x0C
#define PAA5100JE_SHUTTER_H_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned shutter_h:8;
    }; 
} paa5100je_shutter_h_t;

#define PAA5100JE_OBSERVATION_ADDR 0x15
#define PAA5100JE_OBSERVATION_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned observation:8;
    }; 
} paa5100je_observation_t;

#define PAA5100JE_MOTION_BURST_ADDR 0x16
#define PAA5100JE_MOTION_BURST_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned motion_burst:8;
    }; 
} paa5100je_motion_burst_t;

#define PAA5100JE_POWER_UP_RESET_ADDR 0x3A
#define PAA5100JE_POWER_UP_RESET_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned power_up_reset:8;
    }; 
} paa5100je_power_up_reset_t;

#define PAA5100JE_SHUTDOWN_ADDR 0x3B
#define PAA5100JE_SHUTDOWN_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned shutdown:8;
    }; 
} paa5100je_shutdown_t;

#define PAA5100JE_RESOLUTION_ADDR 0x4E
#define PAA5100JE_RESOLUTION_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned resolution:8;
    }; 
} paa5100je_resolution_t;

#define PAA5100JE_RAWDATA_GRAB_ADDR 0x58
#define PAA5100JE_RAWDATA_GRAB_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned rawdata_grab:8;
    }; 
} paa5100je_rawdata_grab_t;

#define PAA5100JE_RAWDATA_GRAB_STATUS_ADDR 0x59
#define PAA5100JE_RAWDATA_GRAB_STATUS_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned rawdata_grab_status:8;
    }; 
} paa5100je_rawdata_grab_status_t;

#define PAA5100JE_ORIENTATION_ADDR 0x5B
#define PAA5100JE_ORIENTATION_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned orientation:8;
    }; 
} paa5100je_orientation_t;

#define PAA5100JE_TEST_ADDR 0xFF
#define PAA5100JE_TEST_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned out:1;
        unsigned sl:2;
        unsigned reserved:5;
    }; 
} paa5100je_test_t;

#define PAA5100JE_LARGE_ADDR 0x88
#define PAA5100JE_LARGE_SIZE 1
typedef union
{
    uint8_t val;
    struct
    {
        unsigned out:1;
        unsigned sl:2;
        unsigned large:5;
        unsigned big:8;
    }; 
} paa5100je_large_t;


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_PAA5100_H_ */
