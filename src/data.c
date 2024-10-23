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


// USB Mass Storage and File System
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/flash_map.h>
#include <ff.h>

// USB CDC Serial 
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/crc.h>
#include <stdlib.h>
#include <stdio.h>

#include "mano.h"
#include "data.h"


// ####################################
// Logging
// ####################################
#define LOG_MODULE_NAME data
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

// ####################################
// Task
// ####################################
#define STACK_SIZE 4096
#define PRIORITY 1
K_THREAD_STACK_DEFINE(data_thread_stack, STACK_SIZE);
struct k_thread data_thread_data;


// ####################################
// MSC and Filesystem
// ####################################
#define STORAGE_PARTITION              external_flash
#define STORAGE_PARTITION_ID           FIXED_PARTITION_ID(STORAGE_PARTITION)
static struct fs_mount_t fs_mnt;




// ####################################
// USB UART 
// ####################################

uint8_t uart_buffer[DATA_UART_BUF_SIZE];
uint32_t uart_buffer_idx = 0; 

// ####################################
// Data Queue
// ####################################
K_MSGQ_DEFINE(data_out_queue, DATA_JSON_STRING_MAX_SIZE, DATA_IN_QUEUE_SIZE, DATA_IN_QUEUE_ALIGNMENT);
K_MSGQ_DEFINE(data_in_queue, DATA_JSON_STRING_MAX_SIZE, DATA_OUT_QUEUE_SIZE, DATA_IN_QUEUE_ALIGNMENT);

#define CRC16_POLY 0x1021
#define CRC16_INIT_VALUE 0xFFFF



// ##############################################################################
// 
// ##############################################################################
static int init_flash (struct fs_mount_t *mnt)
{
	int rc = 0;
	unsigned int id;
	const struct flash_area *pfa;

	mnt->storage_dev = (void *)STORAGE_PARTITION_ID;
	id = STORAGE_PARTITION_ID;

	rc = flash_area_open(id, &pfa);
	LOG_INF("Area %u at 0x%x on %s for %u bytes\n", id, (unsigned int)pfa->fa_off, pfa->fa_dev->name, (unsigned int)pfa->fa_size);

	if (rc < 0 && IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) 
	{
		LOG_INF("Erasing flash area ... ");
		rc = flash_area_erase(pfa, 0, pfa->fa_size);
		LOG_INF("%d\n", rc);
	}

	if (rc < 0) {
		flash_area_close(pfa);
	}
	return rc;
}

// ##############################################################################
// 
// ##############################################################################
static int init_fs(struct fs_mount_t *mnt)
{
	int rc;
	static FATFS fat_fs;
	mnt->type = FS_FATFS;
	mnt->fs_data = &fat_fs;
	mnt->mnt_point = "/NAND:";
	rc = fs_mount(mnt);
	return rc;
}

// ##############################################################################
// 
// ##############################################################################
static int init_disk(void)
{
	struct fs_mount_t *mp = &fs_mnt;
	struct fs_dir_t dir;
	struct fs_statvfs sbuf;
	int rc;

	fs_dir_t_init(&dir);

	rc = init_flash(mp);
	if (rc < 0) 
    {
		LOG_ERR("Failed to setup flash area");
		return -1;
	}

	rc = init_fs(mp);
	if (rc < 0) {
		LOG_ERR("Failed to mount filesystem");
		return -1;
	}

	k_sleep(K_MSEC(50));

	LOG_INF("Mount %s: %d", fs_mnt.mnt_point, rc);

	rc = fs_statvfs(mp->mnt_point, &sbuf);
	if (rc < 0) {
		LOG_ERR("FAIL: statvfs: %d", rc);
		return -1;
	}

	LOG_INF("%s: bsize = %lu ; frsize = %lu ;"" blocks = %lu ; bfree = %lu",
                    mp->mnt_point,
                    sbuf.f_bsize, 
                    sbuf.f_frsize,
                    sbuf.f_blocks, 
                    sbuf.f_bfree);

	rc = fs_opendir(&dir, mp->mnt_point);
	LOG_INF ("%s opendir: %d", mp->mnt_point, rc);

	if (rc < 0) {
		LOG_ERR("Failed to open directory");
        return -1; 
	}

	while (rc >= 0) 
    {
		struct fs_dirent ent = { 0 };

		rc = fs_readdir(&dir, &ent);
		if (rc < 0) {
			LOG_ERR("Failed to read directory entries");
			break;
		}
		if (ent.name[0] == 0) {
			LOG_INF ("End of files");
			break;
		}
		LOG_INF ("  %c %u %s",
		       (ent.type == FS_DIR_ENTRY_FILE) ? 'F' : 'D',
		        ent.size,
		        ent.name);
	}

	(void)fs_closedir(&dir);

	return 0;
}

// ##############################################################################
// 
// ##############################################################################
static void uart_irq_handler (const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) 
	{
		// Data Received
        if (uart_irq_rx_ready(dev)) 
		{
			int recv_len = 0; 
			while (1)
			{
				uint8_t c; 
				recv_len = uart_fifo_read(dev, &c, 1);
				if (recv_len>0)
				{
					if (uart_buffer_idx < DATA_UART_BUF_SIZE)
					{
						uart_buffer[uart_buffer_idx++] = c; 									
						if (uart_buffer[uart_buffer_idx-1] == '\n')
						{
							
							uart_buffer[uart_buffer_idx-1] = 0; 
							k_msgq_put(&data_in_queue, uart_buffer, K_NO_WAIT); 
							uart_buffer_idx=0; 						
						}
					}
					else 
					{
						uart_buffer_idx=0; 
					}
				}
				else 
				{
					break; 
				}
			}
		}
        // Data Transmitted
		if (uart_irq_tx_ready(dev)) 
		{
			

		}
	}
}



// ####################################################################################
// 
// ####################################################################################
void data_task(void *arg1, void *arg2, void *arg3)
{
   	int ret; 

    
    LOG_INF("task started");

    // Start Disk and Filesystem
    if (init_disk()!=0)
    {
        LOG_ERR("Failed to start disk storage");
    }

    // Start USB CDC
	const struct device *dev;
	uint32_t baudrate, dtr, prev_dtr = 0U;
	dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(dev)) {
		LOG_ERR("CDC ACM device not ready");
		return;
	}
    uart_irq_callback_set(dev, uart_irq_handler);
    ret = usb_enable(NULL);

	if (ret != 0) 
	{
		LOG_ERR("Failed to enable USB");
		return;
	}

    uart_irq_rx_enable(dev);

    while (1)
    {
        uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
        if (dtr != prev_dtr)
        {
            if (dtr)
            {
                ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
                if (ret == 0) 
                {
                    LOG_INF("Terminal Connected @ %d", baudrate);
                } 
                else 
                {
                    LOG_INF("Terminal Connected @ UNKNOWN");
                }
            }
            else 
            {
                LOG_INF("Terminal Disconnected");
            }
            prev_dtr = dtr; 
        }

		// If connected to USB
		if (dtr)
		{
			char json_msg[DATA_JSON_STRING_MAX_SIZE] = {0};
			memset(json_msg, 0, sizeof(json_msg)); 
			if (k_msgq_get(&data_out_queue, json_msg, K_NO_WAIT) == 0)
			{
				uart_fifo_fill(dev, (const uint8_t*) json_msg, strlen(json_msg));
				uart_fifo_fill(dev, "\n", 1);
			}
			
		}

		k_msleep(1); 
	}
}

// ####################################################################################
// 
// ####################################################################################
int data_task_init(void)
{
    // Start the thread
    k_tid_t tid = k_thread_create(&data_thread_data, data_thread_stack, STACK_SIZE, data_task, NULL, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
    if (!tid) 
    {
        LOG_ERR("task failed to start"); 
        return -1; 
    }
    return 0; 
}

SYS_INIT(data_task_init, APPLICATION, 80);