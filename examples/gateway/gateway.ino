
// Include the SX1272 
#include "SX1272.h"
// and SPI library on Arduino platforms
#include <SPI.h>

SX1272 sx1272 = SX1272();



#define LORA_ADDR 1

char sprintf_buf[100];

const byte ledPin = 13;
const byte interruptPin = 3;

volatile uint8_t rx_packet =0 ;

void setup()
{
  int error = 0 ;


  // setup - spi communications - MSB, mode 0, 2Mhz 
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setDataMode(SPI_MODE0);

  // Open serial communications and wait for port to open:
  Serial.begin(38400);

  // Power ON the module
  error = sx1272.ON();
  if (error != 0) 
  {
    Serial.print("issues setting up the SX127X module"); 
  }

  sx1272._nodeAddress=LORA_ADDR;

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING);
  
  delay(100);
}


void loop(void)
{ 
    int error = 0;
    uint8_t tmp_length;
    int16_t rssi_packet = 0;

    sx1272.receive();
  
    while (1) 
    {
        delay(1000);   

        if (rx_packet) 
        {
            rx_packet = 0;

            // pull packet from FIFO
            sx1272.receive();

            rssi_packet = sx1272.getRSSIpacket();

            tmp_length = sx1272._payloadlength;

            sprintf(sprintf_buf,"\n --- rxlora. dst=%d type=0x%.2X src=%d seq=%d len=%d SNR=%d RSSIpkt=%d\n", 
                   sx1272._packet_received.dst,
                   sx1272._packet_received.type, 
                   sx1272._packet_received.src,
                   sx1272._packet_received.packnum,
                   tmp_length, 
                   sx1272._SNR,
                   rssi_packet);
                   
            Serial.print(sprintf_buf);

        }  
    }  
} 


/**
 * @brief      called by cdc_usbd libraries, will be called every time a packet is received on the cdc lines 
 *
 * @return     { description_of_the_return_value }
 */
uint8_t cdc_recieved_packet(uint8_t * p_buff, size_t size)
{ 
    uint8_t err =0; 
    Serial.println("cdc_recieved_packet."); 
    return err; 
}

uint8_t gpio_mode(uint8_t pin, uint8_t mode)
{ 
    pinMode(pin,mode);
    return 1;
}

uint8_t gpio_write(uint8_t pin, uint8_t data)
{ 
    digitalWrite(pin,data);
    return 1;
}


uint8_t spi_txrx_byte(uint8_t byte)
{ 
    uint8_t retbyte = SPI.transfer(byte);
    // this function must be overridden by the application software. 
    return retbyte;
}

uint8_t delay_ms(uint16_t delayms)
{ 
    // this function must be overridden by the application software. 
    delay(delayms);
    return 1;
}


void blink() 
{
    uint8_t error =0;
    // If packet received, getPacket
    Serial.print("\nReceived Packet: ");

    error = sx1272.getPacket();

    if(!error)
    {
        rx_packet = 1; 
    }
}
