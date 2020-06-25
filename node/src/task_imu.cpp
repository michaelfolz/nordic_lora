#include "task_imu.h"

#define TASK_DELAY        2000         

/**
 * imu function, responsible for setting up the imu device, collecting data and sending it out to the control task 
 * @param pvParameter 
 */
void imu_task_function(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    uint32_t i =0; 
	

	twi_init();
    
    while (true)
    {
        /* Delay a task for a given number of ticks */
        vTaskDelay(TASK_DELAY);

        /* Tasks must be implemented to never return... */
    }

    // Task should never end! 
    return; 
}