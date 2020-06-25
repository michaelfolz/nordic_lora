#include "task_lora.h"

#define TASK_DELAY        2000         

int16_t lora_tx_interrupt(void)
{
   sw_spi_send_packet("pin1", 4);
    return 1;
}


int16_t lora_rx_interrupt(void)
{
    sw_spi_send_packet("pin2", 4);
    return 1;
}

void lora_task_function(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
	
    gpio_init();
    spi_init(false);

    nrf_gpio_cfg_output(31);




    while (true)
    {
        /* Delay a task for a given number of ticks */
        nrf_gpio_pin_set(31);
        vTaskDelay(100);
        nrf_gpio_pin_clear(31);

        vTaskDelay(100);
        /* Tasks must be implemented to never return... */
    }

    // Task should never end! 
    return; 
}