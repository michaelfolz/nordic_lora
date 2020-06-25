#include "task_lora.h"

#include "SX1272.h"
#include "sw_spi.h"


#define TASK_DELAY        2000         

SX127X sx127X = SX127X();

#define GATEWAY_DESTINATION_ADDR 1
#define NODE_ADDRESS 6


void lora_tx_interrupt(void)
{
   sw_spi_send_packet("pin1", 4);
    return;
}


void lora_rx_interrupt(void)
{
    sw_spi_send_packet("pin2", 4);
    return;
}

void lora_task_function(void * pvParameter)
{
    uint16_t error; 
    UNUSED_PARAMETER(pvParameter);
	
    void (*tx_int)() = &lora_tx_interrupt; 
    void (*rx_int)() = &lora_rx_interrupt; 

   // gpio_init(tx_int, rx_int);
    spi_init(false);

    nrf_gpio_cfg_output(31);

    error = sx127X.ON();
    if(error !=0)
    {
        sw_spi_send_packet("issues setting up the SX127X module", 35);
    }


    // Set the node address and print the result
    error = sx127X.setNodeAddress(NODE_ADDRESS);
    sw_spi_send_packet("Setting node addr: state ", 25);

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