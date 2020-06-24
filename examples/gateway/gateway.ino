
// Include the SX1272 
#include "SX1272.h"
// and SPI library on Arduino platforms
#include <SPI.h>

SX127X sx127X = SX127X();

#define SX127X_TX_INTERRUPT_PIN  2
#define SX127X_RX_INTERRUPT_PIN  3

#define LORA_ADDR 1

#define GATEWAY_DESTINATION_ADDR 1
#define NODE_ADDRESS 6

char sprintf_buf[100];

const byte ledPin = 13;
const byte interruptPin = 3;
uint8_t message[100];
volatile uint8_t rx_packet =0 ;
volatile SX127X_TX_Packet_States tx_state =TX_NONE; 

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
    error = sx127X.ON();
    if (error != 0) 
    {
        Serial.println("issues setting up the SX127X module");
        Serial.print(error); 
        // should panic
    }
    Serial.print(" set up the SX127X module"); 
    sx127X._nodeAddress=LORA_ADDR;

    pinMode(interruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(SX127X_RX_INTERRUPT_PIN), LoRA_RX_Interrupt_Routine, RISING);


    pinMode(SX127X_TX_INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(SX127X_TX_INTERRUPT_PIN), LoRA_TX_Interrupt_Routine, FALLING);

    tx_state = TX_NONE; 
    delay(100);
}


void loop(void)
{ 
    static uint16_t counter =0;
    static bool rx_mode = false; 
    int error = 0;
    uint8_t r_size;
    uint8_t tmp_length;
    int16_t rssi_packet = 0;
    int8_t SNR; 

    // setup rx mode while tx_state is done or idle 
    if((tx_state == TX_COMPLETE) ||
       (tx_state == TX_NONE) || (tx_state == TX_ERROR))
    {
          if(!rx_mode)
          {
            // set into recieve mode 
              sx127X.receive();
              rx_mode = true; 
              Serial.print(F("Setup RX MODE wait "));
          }         
    }

    if (tx_state == TX_REQUEST_SEND)
    {
        r_size=sprintf((char*)message, "%04d server packet 0x%x",counter, counter++);  

        Serial.print(F("Sending "));
        Serial.println((char*)(message));

        Serial.print(F("Real payload size is "));
        Serial.println(r_size);


        tx_state =  TX_SETTING_UP_TRANSMISSION;
        rx_packet = 0;
        rx_mode = false; 
        // Send message to the gateway and print the result
        // with the app key if this feature is enabled
        error = sx127X.sendPacket(NODE_ADDRESS, message, r_size);
        if(error != 0)
        {
           tx_state = TX_ERROR;
           sprintf((char*)message, "Error sending paket error 0x%x",error);  
           Serial.println((char*)(message));
        }
    }

    if(tx_state == TX_COMPLETE)
    {
        Serial.print(F("DONE "));
        tx_state = TX_NONE; 
    }

    if(tx_state == TX_ERROR)
    {
        Serial.print(F("ERROR "));
        tx_state = TX_NONE; 
    }

    // if packet has been rx print it out 
    if (rx_packet && rx_mode) 
    {
        sx127X.getPacket();
        // pull packet from FIFO
        rssi_packet = sx127X.getRSSIpacket();
        SNR = sx127X.getSNR();
        tmp_length = sx127X._payloadlength;

        sprintf(sprintf_buf,"\n --- rxlora. dst=%d type=0x%.2X src=%d seq=%d len=%d SNR=%d RSSIpkt=%d\n", 
               sx127X._packet_received.dst,
               sx127X._packet_received.type, 
               sx127X._packet_received.src,
               sx127X._packet_received.packnum,
               tmp_length, 
               SNR,
               rssi_packet);
        Serial.print(sprintf_buf);
        rx_packet = 0;
        rx_mode = false; 
       tx_state = TX_REQUEST_SEND;
    }
  delay(500); 
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


void LoRA_TX_Interrupt_Routine() 
{
    uint8_t error =0;

    switch(tx_state)
    {
        case TX_SETTING_UP_TRANSMISSION:
            tx_state = TX_IN_TRANSMISSION;
            break;

        case TX_IN_TRANSMISSION: 
        {
            error = sx127X.checkTransmissionStatus(); 
            if(error ==0)
            {
                tx_state = TX_COMPLETE;
            }
            else 
            {
                tx_state = TX_ERROR;
            }

            break;
        }
    }    
}

void LoRA_RX_Interrupt_Routine() 
{
    uint8_t error =0;
    static bool ignore_first_trigger = true;
    if(ignore_first_trigger)
    { 
      ignore_first_trigger = false; 
      return;
    }
    
    rx_packet = 1; 
}