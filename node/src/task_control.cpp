#include "task_control.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"



#define TASK_DELAY        2     

/**
 * main control task, responsible for controlling all IO
 * @param pvParameter 
 */
void control_task_function(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    uint32_t i =0; 

    NRF_LOG_INFO("control_task_function begin");       
    
    while (true)
    {
    	
        /* Delay a task for a given number of ticks */
        vTaskDelay(TASK_DELAY);


        /* Process logs */
        NRF_LOG_PROCESS();
        
        /* Tasks must be implemented to never return... */
    }

    // Task should never end! 
    return; 
}