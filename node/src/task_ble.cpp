#include "task_ble.h"

#include "nrf_ble.h"

#define TASK_DELAY        2000

/**
 * ble task responsible for handelling all ble communications in/out 
 * @param pvParameter 
 */
void ble_task_function(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    uint32_t i =0; 

   i = ble_init();
  
    while (true)
    {
        /* Delay a task for a given number of ticks */
        vTaskDelay(TASK_DELAY);


        /* Tasks must be implemented to never return... */
    
       // heart_rate_meas_timeout_handler(2);
        battery_level_update();
    }

    // Task should never end! 
    return; 
}