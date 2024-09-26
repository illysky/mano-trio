
//#############################################################################
// Quick and dirty zephyr wrapper for the FUSB302 driver for usb-typec-pd
// Manages the port
//#############################################################################
#ifndef USB_PD_H
#define USB_PD_H

#include <zephyr/device.h>


typedef void (*usb_pd_run_t)(const struct device *dev);
typedef bool (*usb_pd_pdo_ready_t)(const struct device *dev);
typedef void (*usb_pd_pdo_list_t)(const struct device *dev);
typedef int32_t (*usb_pd_request_sink_t)(const struct device *dev, uint32_t voltage);

struct usb_pd_driver_api {
    usb_pd_run_t            run;
    usb_pd_pdo_ready_t      pdo_ready;
    usb_pd_pdo_list_t       pdo_list; 
    usb_pd_request_sink_t   request_sink; 
};

static inline void usb_pd_run(const struct device *dev)
{
    struct usb_pd_driver_api *api;
    api = (struct usb_pd_driver_api*) dev->api;
    return api->run(dev);
}

static inline bool usb_pd_pdo_ready (const struct device *dev)
{
    struct usb_pd_driver_api *api = (struct usb_pd_driver_api*) dev->api;
    return api->pdo_ready(dev);
}

static inline void usb_pd_pdo_list (const struct device *dev)
{
    struct usb_pd_driver_api *api = (struct usb_pd_driver_api*) dev->api;
    return api->pdo_list(dev);
}

static inline int32_t usb_pd_request_sink (const struct device *dev, uint32_t voltage)
{
    struct usb_pd_driver_api *api = (struct usb_pd_driver_api*) dev->api;
    return api->request_sink(dev, voltage);
}


#endif 