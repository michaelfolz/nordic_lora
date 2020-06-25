#include "hid_gamepad.h"



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


static void hid_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event);


APP_USBD_HID_GENERIC_SUBCLASS_REPORT_DESC(controller_one,APP_USBD_HID_U2F_REPORT_DSC());
APP_USBD_HID_GENERIC_SUBCLASS_REPORT_DESC(controller_two,APP_USBD_HID_U2F_REPORT_DSC());


static const app_usbd_hid_subclass_desc_t * reps[] = {&controller_one};
static const app_usbd_hid_subclass_desc_t * reps2[] = {&controller_two};



/**
 * @brief Global HID generic instance
 */
APP_USBD_HID_GENERIC_GLOBAL_DEF(m_app_hid_generic,
                                HID_GENERIC_INTERFACE,
                                hid_user_ev_handler,
                                ENDPOINT_LIST(),
                                reps,
                                REPORT_IN_QUEUE_SIZE,
                                REPORT_OUT_MAXSIZE,
                                APP_USBD_HID_SUBCLASS_NONE,  // non booting sub class
                                APP_USBD_HID_PROTO_GENERIC); // generic hid protocol 



/**
 * @brief Global HID generic instance
 */
APP_USBD_HID_GENERIC_GLOBAL_DEF(m_app_hid_generic2,
                                HID_GENERIC_INTERFACE2,
                                hid_user_ev_handler,
                                ENDPOINT_LIST2(),
                                reps2,
                                REPORT_IN_QUEUE_SIZE,
                                REPORT_OUT_MAXSIZE,
                                APP_USBD_HID_SUBCLASS_NONE,  // non booting sub class
                                APP_USBD_HID_PROTO_GENERIC); // generic hid protocol 



/**
 * @brief Mark the ongoing transmission
 *
 * Marks that the report buffer is busy and cannot be used until transmission finishes
 * or invalidates (by USB reset or suspend event).
 */
static bool m_report_pending;




/**
 * @brief USBD library specific event handler.
 *
 * @param event     USBD library event.
 * */
static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SOF:
            break;
        case APP_USBD_EVT_DRV_RESET:
            m_report_pending = false;
            break;
        case APP_USBD_EVT_DRV_SUSPEND:
            m_report_pending = false;
            app_usbd_suspend_req(); // Allow the library to put the peripheral into sleep mode
            break;
        case APP_USBD_EVT_DRV_RESUME:
            m_report_pending = false;
            break;
        case APP_USBD_EVT_STARTED:
            m_report_pending = false;
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}


/**
 * @brief Class specific event handler.
 *
 * @param p_inst    Class instance.
 * @param event     Class specific event.
 * */
static void hid_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event)
{
    switch (event)
    {
        case APP_USBD_HID_USER_EVT_OUT_REPORT_READY:
        {
            /* No output report defined for this example.*/
            ASSERT(0);
            break;
        }
        case APP_USBD_HID_USER_EVT_IN_REPORT_DONE:
        {
            m_report_pending = false;
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_BOOT_PROTO:
        {
            UNUSED_RETURN_VALUE(hid_generic_clear_buffer(p_inst));
            NRF_LOG_INFO("SET_BOOT_PROTO");
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_REPORT_PROTO:
        {
            UNUSED_RETURN_VALUE(hid_generic_clear_buffer(p_inst));
            NRF_LOG_INFO("SET_REPORT_PROTO");
            break;
        }
        default:
            break;
    }
}




static ret_code_t idle_handle(app_usbd_class_inst_t const * p_inst, uint8_t report_id)
{
    switch (report_id)
    {
        case 0:
        {
            uint8_t report[] = {0xBE, 0xEF};
            return app_usbd_hid_generic_idle_report_set(
              &m_app_hid_generic,
              report,
              sizeof(report));
        }
        default:
            return NRF_ERROR_NOT_SUPPORTED;
    }
    
}

static ret_code_t idle_handle2(app_usbd_class_inst_t const * p_inst, uint8_t report_id)
{
    switch (report_id)
    {
        case 0:
        {
            uint8_t report[] = {0xBE, 0xEF};
            return app_usbd_hid_generic_idle_report_set(
              &m_app_hid_generic2,
              report,
              sizeof(report));
        }
        default:
            return NRF_ERROR_NOT_SUPPORTED;
    }
    
}


void init_controllers(void)
{
	uint32_t ret; 

    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };
    

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);



    app_usbd_class_inst_t const * class_controller;
    class_controller = app_usbd_hid_generic_class_inst_get(&m_app_hid_generic);

    ret = hid_generic_idle_handler_set(class_controller, idle_handle);
    APP_ERROR_CHECK(ret);

    ret = app_usbd_class_append(class_controller);

/// cnt 2

    app_usbd_class_inst_t const * class_controller_2;
    class_controller_2 = app_usbd_hid_generic_class_inst_get(&m_app_hid_generic2);

    ret = hid_generic_idle_handler_set(class_controller_2, idle_handle2);
    APP_ERROR_CHECK(ret);
  
    ret = app_usbd_class_append(class_controller_2);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
    }
	return;

}


void controller_sendpacket(uint8_t controllerID, uint8_t* data, uint8_t dataLength)
{
    static uint8_t report[HID_REP_SIZE];

    if(dataLength > HID_REP_SIZE)
    	return;

    if (m_report_pending)
        return;

    memcpy(report, data, HID_REP_SIZE);

    /* Start the transfer */
    if(controllerID == 1)
    	app_usbd_hid_generic_in_report_set(&m_app_hid_generic, report, sizeof(report));
    else if(controllerID == 2 )
 	    app_usbd_hid_generic_in_report_set(&m_app_hid_generic2, report, sizeof(report));

    return;
}
