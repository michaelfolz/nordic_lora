#include "task_imu.h"

#define TASK_DELAY        2000         

void usbd_task_function(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    
    while (true)
    {
        /* Delay a task for a given number of ticks */
        vTaskDelay(TASK_DELAY);

        /* Tasks must be implemented to never return... */
    }

    // Task should never end! 
    return; 
}