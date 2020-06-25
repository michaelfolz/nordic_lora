#include "error.h"


void error_panic(char *data, unsigned char length)
{

    // needs to terminate the free rtos, set led, reset the system etc 
    // for now just while loop and print message
    
    while(1)
    {
        sw_spi_send_packet(data, length);
        nrf_delay_ms(1);
    }


}