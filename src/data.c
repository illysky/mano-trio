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


// ####################################
// Logging
// ####################################
#define LOG_MODULE_NAME data
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

// ####################################
// Task
// ####################################
#define STACK_SIZE 4096
#define PRIORITY 5
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
#define UART_BUF_SIZE 1024
uint8_t uart_buffer[UART_BUF_SIZE];
uint32_t uart_buffer_idx = 0; 


// ####################################
// Command Queue
// ####################################


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

   // mnt->storage_dev = (void *)flash_dev;
	//id = 1;

	rc = flash_area_open(id, &pfa);
	LOG_INF("Area %u at 0x%x on %s for %u bytes\n", id, (unsigned int)pfa->fa_off, pfa->fa_dev->name, (unsigned int)pfa->fa_size);

	if (rc < 0 && IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
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

	LOG_INF("Mount %s: %d\n", fs_mnt.mnt_point, rc);

	rc = fs_statvfs(mp->mnt_point, &sbuf);
	if (rc < 0) {
		LOG_ERR("FAIL: statvfs: %d\n", rc);
		return -1;
	}

	LOG_INF("%s: bsize = %lu ; frsize = %lu ;"" blocks = %lu ; bfree = %lu\n",
                    mp->mnt_point,
                    sbuf.f_bsize, 
                    sbuf.f_frsize,
                    sbuf.f_blocks, 
                    sbuf.f_bfree);

	rc = fs_opendir(&dir, mp->mnt_point);
	LOG_INF ("%s opendir: %d\n", mp->mnt_point, rc);

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
			LOG_INF ("End of files\n");
			break;
		}
		LOG_INF ("  %c %u %s\n",
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
					if (uart_buffer_idx < UART_BUF_SIZE)
					{
						uart_buffer[uart_buffer_idx++] = c; 									
						if (uart_buffer[uart_buffer_idx-1] == '\n')
						{
							// Get CRC 
							uint16_t r_crc = strtol((const char*) &uart_buffer[uart_buffer_idx-5], NULL, 16); 
							uint16_t c_crc = crc16_ccitt(CRC16_INIT_VALUE, (uint8_t *) uart_buffer, uart_buffer_idx - 5);
							LOG_HEXDUMP_INF(uart_buffer, uart_buffer_idx, "");  
							LOG_INF("CRC: %u, %u", r_crc, c_crc); 
							//if (r_crc==c_crc)
							if (1)
							{
								
								uint32_t nbytes = sizeof(mano_command_t); 
								uint8_t* bytes = malloc (nbytes);  // Create buffer minus CRC\n 
								
								if (bytes)
								{
									memset(bytes, 0, nbytes); 
									for (size_t i = 0; i < nbytes; i++) 
									{	
										sscanf((const char*) uart_buffer + (2 * i), "%2hhx", &bytes[i]);
									}
									LOG_HEXDUMP_INF(bytes, nbytes, "");  
									k_msgq_put(&mano_command_queue, bytes, K_NO_WAIT); 
									free(bytes); 
								}
							}
							else
							{
								LOG_WRN("CRC Fail. Dropping Packet"); 
							}



							uart_buffer_idx=0; 	// Clear


							//k_msgq_put(&mano_command_queue, (mano_command_t*) uart_buffer);


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

			//send_len = uart_fifo_fill(dev, buffer, rb_len);

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
        k_msleep(5); 
        
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