#ifndef _HID_GAMEPAD_INCLUDED
#define _HID_GAMEPAD_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"

#include "nrf.h"
#include "app_util_platform.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_drv_power.h"

#include "app_timer.h"
#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_hid_generic.h"
#include "app_usbd_hid_mouse.h"
#include "app_usbd_hid_kbd.h"
#include "app_error.h"
#include "bsp.h"

#include "bsp_cli.h"
#include "nrf_cli.h"
#include "nrf_cli_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**
 * @brief Enable USB power detection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

/**
 * @brief HID generic class interface number.
 * */
#define HID_GENERIC_INTERFACE  1
#define HID_GENERIC_INTERFACE2  2

/**
 * @brief HID generic class endpoint number.
 * */

#define HID2_GENERIC_EPIN       NRF_DRV_USBD_EPIN3
#define U2F2_HID_EPOUT          NRF_DRV_USBD_EPOUT3

#define HID3_GENERIC_EPIN       NRF_DRV_USBD_EPIN2
#define U2F3_HID_EPOUT          NRF_DRV_USBD_EPOUT2


/* HID report layout */
#define HID_REP_SIZE  (4)/**< The size of the report */

/**
 * @brief Number of reports defined in report descriptor.
 */
#define REPORT_IN_QUEUE_SIZE    2

/**
 * @brief Size of maximum output report. HID generic class will reserve
 *        this buffer size + 1 memory space. 
 *
 * Maximum value of this define is 63 bytes. Library automatically adds
 * one byte for report ID. This means that output report size is limited
 * to 64 bytes.
 */
#define REPORT_OUT_MAXSIZE  63

/**
 * @brief HID generic class endpoints count.
 * */
#define HID_GENERIC_EP_COUNT  2


/**
 * @brief List of HID generic class endpoints.
 * */
#define ENDPOINT_LIST()                                      \
(                                                            \
        HID3_GENERIC_EPIN,                                    \
        U2F3_HID_EPOUT                                        \
)


/**
 * @brief List of HID generic class endpoints.
 * */
#define ENDPOINT_LIST2()                                      \
(                                                            \
        HID2_GENERIC_EPIN,                                    \
        U2F2_HID_EPOUT                                        \
)
/**
 * @brief Additional key release events
 *
 * This example needs to process release events of used buttons
 */
enum {
    BSP_USER_EVENT_RELEASE_0 = BSP_EVENT_KEY_LAST + 1, /**< Button 0 released */
    BSP_USER_EVENT_RELEASE_1,                          /**< Button 1 released */
    BSP_USER_EVENT_RELEASE_2,                          /**< Button 2 released */
    BSP_USER_EVENT_RELEASE_3,                          /**< Button 3 released */
    BSP_USER_EVENT_RELEASE_4,                          /**< Button 4 released */
    BSP_USER_EVENT_RELEASE_5,                          /**< Button 5 released */
    BSP_USER_EVENT_RELEASE_6,                          /**< Button 6 released */
    BSP_USER_EVENT_RELEASE_7,                          /**< Button 7 released */
};


#define APP_USBD_HID_U2F_REPORT_DSC() {   \
  0x05, 0x01,                   /*USAGE_PAGE (Generic Desktop)*/\
  0x09, 0x05,                   /*USAGE (Game Pad)*/\
  0xa1, 0x01,                   /*COLLECTION (APPLICATION)*/\
  0xa1, 0x00,                    /*   COLLECTION (Physical)*/\
  0x05, 0x09,                   /*USAGE_PAGE (Button)*/\
  0x19, 0x01,                   /*USAGE_MINIMUM (Button1)*/\
  0x29, 0x10,                   /*USAGE_MAXIMUM (Button 16)*/\
  0x15, 0x00,                   /*LOGICAL_MINIMUM (0)*/\
  0x25, 0x01,                   /*LOGICAL_MAXIMUM(1)*/\
  0x95, 0x10,                   /*REPORT_COUNT (16)*/\
  0x75, 0x01,                   /*REPORT_SIZE (1)*/\
  0x81, 0x02,                   /*INPUT(Data, Var, Abs)*/\
  0x05, 0x01,                   /*USAGE_PAGE (Generic Desktop)*/\
  0x09, 0x30,                   /*USAGE (X)*/\
  0x09, 0x31,                   /*USAGE (Y)*/\
  0x15, 0x81,                   /*LOGICAL_MINIMUM(-127)*/\
  0x25, 0x7f,                   /*LOGICAL_MAXIMUM(127)*/\
  0x75, 0x08,                   /*REPORT_SIZE(8)*/\
  0x95, 0x02,                   /*REPORT_COUNT(4)*/\
  0x81, 0x02,                   /*INPUT(Data,Var,Abs)*/\
  0xc0,                     /*END_Collection*/\
  0xc0                      /*END_Collection*/\
}




void init_controllers(void);

void controller_sendpacket(uint8_t controllerID, uint8_t* data, uint8_t dataLength);
#endif /* _HID_GAMEPAD_INCLUDED*/

