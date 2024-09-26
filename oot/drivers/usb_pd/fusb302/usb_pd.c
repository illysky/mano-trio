//#############################################################################
// Quick and dirty zephyr wrapper for the FUSB302 driver for usb-typec-pd
// Manages the port
//#############################################################################

//########################################################
// Zephyr Includes
//########################################################
#define DT_DRV_COMPAT onsemi_fusb302
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#define LOG_LEVEL CONFIG_FUSB302_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(fusb302);


//########################################################
// FUSB Includes
//########################################################
#include "core.h"
#include "Port.h"
#include "FSCTypes.h"
#include "PD_Types.h"
#include "platform.h"
#include "usb_pd.h"


//#############################################################################
struct fusb302_config 
//#############################################################################
{
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec irqn;
};

//#############################################################################
struct fusb302_data 
{
	Port_t port;            // port data
    DevicePolicyPtr_t dpm;  // device policy
    bool pdo_ready;         // pdo
    bool pdo_booted; 
};






 
//#############################################################################
void platform_set_vbus_lvl_enable(void* dev,VBUS_LVL level,FSC_BOOL enable,FSC_BOOL disableOthers)
// Callback to set the vbus voltage level if we are a source
//#############################################################################
{
    unsigned int i;
    switch (level) {
    case VBUS_LVL_5V:
        break;
    default:
        break;
    }

    /* Turn off other levels, if requested */
    if (disableOthers || ((level == VBUS_LVL_ALL) && (enable == FALSE)))
    {
        i = 0;

        do {
            /* Skip the current level */
            if( i == level ) continue;

            /* Turn off the other level(s) */
            // platform_set_vbus_lvl_enable( i, FALSE, FALSE );
        } while (++i < VBUS_LVL_ALL);
    }
}

//#############################################################################
FSC_BOOL platform_get_vbus_lvl_enable(void* dev, VBUS_LVL level)
// Callback to enable the vbus if we are a source device
//#############################################################################
{
    switch (level) 
    {
        case VBUS_LVL_5V:
        break;
        default:
        break;
    }
    return FALSE;
}

//#############################################################################
void platform_set_vbus_discharge(void* dev, FSC_BOOL enable)
// 
//#############################################################################
{


}

//#############################################################################
FSC_BOOL platform_get_device_irq_state(void* dev)
// Callback to get the IRQ status
//#############################################################################
{
    const struct fusb302_config *cfg = ((const struct device*)dev)->config;

    uint32_t value = gpio_pin_get_dt(&cfg->irqn);
    if (value)
    {
        return true; 
    }
    else 
    {
        return false;
    }
}

//#############################################################################
FSC_BOOL platform_i2c_write(void* dev, FSC_U8 RegAddrLength,FSC_U8 DataLength,FSC_U8 PacketSize,FSC_U8 IncSize,FSC_U32 RegisterAddress,FSC_U8* Data)
// Callback to write to the i2c bus associated with this instance
//#############################################################################
{
    const struct fusb302_config *cfg = ((const struct device*)dev)->config; 
    int32_t result = i2c_burst_write_dt (&cfg->i2c, (uint8_t) RegisterAddress, Data, DataLength); 
    //LOG_INF("  %02X", RegisterAddress); 
    //LOG_HEXDUMP_INF(Data, DataLength); 

    if (result)
    {
        LOG_ERR("platform_i2c_write: i2c error");
    }
    return ((result == 0) ? true : false);
}

//#############################################################################
FSC_BOOL platform_i2c_read(void* dev, FSC_U8 RegAddrLength,FSC_U8 DataLength,FSC_U8 PacketSize,FSC_U8 IncSize,FSC_U32 RegisterAddress,FSC_U8* Data)
// Callback to read from the i2c associated with thie instance
//#############################################################################
{
    const struct fusb302_config *cfg = ((const struct device*)dev)->config;
    int32_t result = i2c_burst_read_dt (&cfg->i2c, RegisterAddress, Data, DataLength); 
    if (result)
    {
        LOG_ERR("platform_i2c_read: i2c error");
    }
    return ((result == 0) ? TRUE : FALSE);
}

//#############################################################################
void platform_delay_10us(FSC_U32 delayCount)
// Callback to issue a 10us delay
//#############################################################################
{
    k_usleep(delayCount*10); 
}

//#############################################################################
void platform_set_pps_voltage(void* dev, FSC_U32 mv)
// Callback to set pps_voltage
//#############################################################################
{
    // Not implimented
    LOG_INF("platform_set_pps_voltage"); 
}

//#############################################################################
FSC_U16 platform_get_pps_voltage(void* dev)
// Callback to get the pps voltage of the bus
//#############################################################################
{
    LOG_INF("platform_get_pps_voltage"); 
    return 0;
}

//#############################################################################
void platform_set_pps_current(void* dev, FSC_U32 ma)
// Callback to get the pps current of the bus
//#############################################################################
{
    LOG_INF("platform_set_pps_current"); 
}

//#############################################################################
FSC_U16 platform_get_pps_current(void* dev)
// Callback to get the pps current of the bus
//#############################################################################
{
    return 0;
}

//#############################################################################
FSC_U32 platform_get_system_time(void)
// Callback to get the system time
//#############################################################################
{
    uint32_t t = k_uptime_get();
    return t; 
}

//#############################################################################
FSC_U32 platform_get_log_time(void)
// Callback to get the system time
//#############################################################################
{
    uint32_t t = k_uptime_get();
    return t; 
}

//#############################################################################
static void core_event_handler (uint32_t event, void* dev, void *usr_ctx, void *app_ctx)
// Event handler
//#############################################################################
{
    struct fusb302_data *data = ((const struct device*)dev)->data;


    LOG_INF("core_event_handler"); 
    if (event & PD_NEW_CONTRACT)
    {
        LOG_INF("PD_NEW_CONTRACT"); 
        /* handle event */
    }

    if (event & PD_CONTRACT_ALL)
    {
        LOG_INF("PD_CONTRACT_ALL"); 
        data->pdo_ready = true; 
    }

    if (event & PD_STATE_CHANGED)
    {
        LOG_INF("PD_STATE_CHANGED"); 
        data->pdo_booted = true; 
        /* handle event */
    }

}

//#############################################################################
static void run (const struct device *dev)
// 
//#############################################################################
{
    struct fusb302_data *data = ((const struct device*)dev)->data;
    core_state_machine(&data->port);
}

//#############################################################################
static bool pdo_ready (const struct device *dev)
// 
//#############################################################################
{
    struct fusb302_data *data = ((const struct device*)dev)->data;
    return data->pdo_ready; 
}

//#############################################################################
static void pdo_list (const struct device *dev)
// 
//#############################################################################
{
    struct fusb302_data *data = ((const struct device*)dev)->data;
    for (uint8_t a=0; a<7; a++)
    {
        if (data->port.SrcCapsReceived[a].PDO.SupplyType == 0)
        {
            uint32_t voltage = data->port.SrcCapsReceived[a].FPDOSupply.Voltage / 20; 
            uint32_t current = data->port.SrcCapsReceived[a].FPDOSupply.MaxCurrent / 100; 
             LOG_INF("FPDO Supply [%u]: %uV %uA (%uW)" , a, voltage, current, voltage*current);
        }
    }
}

//#############################################################################
static int32_t request_sink (const struct device *dev, uint32_t voltage)
// 
//#############################################################################
{
    struct fusb302_data *data = ((const struct device*)dev)->data;
    uint32_t r = voltage * 20;
    for (uint8_t a=0; a<7; a++)
    {
        if (data->port.SrcCapsReceived[a].PDO.SupplyType == 0)
        {
            if (data->port.SrcCapsReceived[a].FPDOSupply.Voltage == r)
            {
                LOG_INF("Requesting %uV: PDO [%u] Match", voltage, a); 
                data->port.SinkRequest.FVRDO.MinMaxCurrent = 200; 
                data->port.SinkRequest.FVRDO.OpCurrent = 200; 
                data->port.SinkRequest.FVRDO.UnChnkExtMsgSupport = 0;
                data->port.SinkRequest.FVRDO.NoUSBSuspend = 1;
                data->port.SinkRequest.FVRDO.USBCommCapable = 0;
                data->port.SinkRequest.FVRDO.CapabilityMismatch = 0;
                data->port.SinkRequest.FVRDO.GiveBack = 0;
                data->port.SinkRequest.FVRDO.ObjectPosition = a+1; // Set the PDO we are using
                SetPEState(&data->port,peSinkSelectCapability);
                return 0; 
            }
        }
    }
    LOG_INF("Requesting %uV: Not Found", voltage); 
    return -1; 
}

//#############################################################################
static int init(const struct device *dev)
// Init call to this function
//#############################################################################
{
    const struct fusb302_config *cfg = dev->config;
    struct fusb302_data *data = dev->data;
    LOG_INF("Initialisation for device %X", cfg->i2c.addr);

    data->pdo_ready = false; 
    data->pdo_booted = false; 
    
    gpio_pin_configure_dt(&cfg->irqn, (GPIO_INPUT| GPIO_PULL_UP));

    uint8_t id = 0; 
    int32_t result = i2c_burst_read_dt (&cfg->i2c, 0x01, &id, 1); 
    if (result)
    {
        LOG_ERR("_init: i2c error");
    }
    else 
    {
        LOG_INF("_init: success. Found FUSB302 %02X", id);
    }

    memset((void*) &data->port, 0, sizeof(Port_t)); 
    memset((void*) &data->dpm, 0, sizeof(DevicePolicyPtr_t)); 
    core_initialize((void*) &data->port, (void*) dev); 
    core_enable_typec((void*) &data->port, true);
    DPM_Init((void*) &data->dpm); 
    DPM_AddPort(data->dpm, (void*) &data->port); 
    data->port.dpm = data->dpm; 
    register_observer(EVENT_ALL, &core_event_handler, 0);
    return 0;  
}

static const struct usb_pd_driver_api fusb302_usb_pd_driver_api = {
	.run = run,
    .pdo_ready = pdo_ready,
    .pdo_list = pdo_list,
    .request_sink = request_sink,
};

// ###############################################################################################
// DT STRUCT  
// ###############################################################################################
#define FUSB302_DEVICE(id)												\
	static const struct fusb302_config fusb302_##id##_cfg = {		    \
		.i2c = I2C_DT_SPEC_INST_GET(id),								\
		.irqn =  GPIO_DT_SPEC_INST_GET(id, irqn_gpios),			        \
	};																	\
	static struct fusb302_data fusb302_##id##_data;						\
	DEVICE_DT_INST_DEFINE(id, &init, NULL,					            \
			&fusb302_##id##_data,										\
			&fusb302_##id##_cfg, POST_KERNEL,							\
			CONFIG_FUSB302_INIT_PRIORITY,							    \
		    &fusb302_usb_pd_driver_api);

DT_INST_FOREACH_STATUS_OKAY(FUSB302_DEVICE)
