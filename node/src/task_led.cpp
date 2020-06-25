#include "task_led.h"

#define TASK_DELAY        2000           /**< Task delay. Delays a LED0 task for 200 ms */

/**
 * basic led toggle function, used for testing 
 * @param pvParameter 
 */
void led_toggle_task_function(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    uint32_t i =0; 

    while (true)
    {
        sw_spi_send_packet("test", 4);
        bsp_board_led_invert(i++%LEDS_NUMBER);
        /* Delay a task for a given number of ticks */
        vTaskDelay(TASK_DELAY);
         NRF_LOG_INFO("led_toggle_task_function ");    
        /* Tasks must be implemented to never return... */
    }

    // Task should never end! 
    return; 
}