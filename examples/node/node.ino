/*
 *  temperature sensor on analog 8 to test the LoRa gateway
 *
 *  Copyright (C) 2015 Congduc Pham, University of Pau, France
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************
 */
 // and SPI library on Arduino platforms
#include <SPI.h>
// Include the sx127X
#include "SX1272.h"
SX127X sx127X = SX127X();


#define GATEWAY_DESTINATION_ADDR 1
#define NODE_ADDRESS 6


unsigned long lastTransmissionTime=0;
unsigned long delayBeforeTransmit=0;
uint8_t message[100];

#define SX127X_TX_INTERRUPT_PIN  2
#define SX127X_RX_INTERRUPT_PIN  3

volatile uint8_t rx_packet =0 ;
volatile SX127X_TX_Packet_States tx_state =TX_NONE; 

void setup()
{
    int error;

    // MSB, mode 0, 2Mhz 
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.setDataMode(SPI_MODE0);

    // Open serial communications and wait for port to open:
    Serial.begin(38400);
    // Print a start message
    Serial.println(F("SX1272 module and Arduino: send packets without ACK"));

    // Power ON the module
    error = sx127X.ON();
    if(error !=0)
    {
         Serial.println(F("issues setting up the SX127X module"));
    }

    // Set the node address and print the result
    error = sx127X.setNodeAddress(NODE_ADDRESS);
    Serial.print(F("Setting node addr: state "));

    // Print a success message
    Serial.println(F("sx127X successfully configured"));

    pinMode(SX127X_TX_INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(SX127X_TX_INTERRUPT_PIN), LoRA_TX_Interrupt_Routine, FALLING);

    pinMode(SX127X_RX_INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(SX127X_RX_INTERRUPT_PIN), LoRA_RX_Interrupt_Routine, RISING);


    tx_state = TX_NONE; 

    delay(500);
}


char sprintf_buf[100];

void loop(void)
{
    long startSend;
    long endSend;
    uint8_t app_key_offset=0;
    int e;
    uint8_t tmp_length;
    int16_t rssi_packet = 0;
    int8_t SNR; 


    // wait to send out next packet 
    if (millis()-lastTransmissionTime > delayBeforeTransmit)
    {
        startSend=millis();
        endSend=millis();  
        lastTransmissionTime=millis();
        delayBeforeTransmit=4000+random(15,60)*100;
        tx_state = TX_REQUEST_SEND; 
    }

    if (tx_state == TX_REQUEST_SEND)
    {
      //  sx127X.setTX(); 
        uint8_t r_size;
        static uint16_t counter =0;
        r_size=sprintf((char*)message, "%04d testing packet 0x%x",counter, counter++);  

        Serial.print(F("Sending "));
        Serial.println((char*)(message+app_key_offset));

        Serial.print(F("Real payload size is "));
        Serial.println(r_size);

        int pl=r_size+app_key_offset;

        tx_state =  TX_SETTING_UP_TRANSMISSION;

        // Send message to the gateway and print the result
        // with the app key if this feature is enabled
        e = sx127X.sendPacket(GATEWAY_DESTINATION_ADDR, message, r_size);
        if(e != 0)
        {
           tx_state = TX_ERROR;
           sprintf((char*)message, "Error sending paket error 0x%x",e);  
           Serial.println((char*)(message));
        }
    }

    if(tx_state == TX_COMPLETE)
    {
          Serial.print(F("DONE "));
          sx127X.receive();
      //    delay(1500);
      
         tx_state = TX_NONE; 
    }

    if(tx_state == TX_ERROR)
    {
        //  Serial.print(F("ERROR "));
         tx_state = TX_NONE; 
    }

      if (rx_packet) 
      {
          sx127X.getPacket();
          rx_packet = 0;

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
      }
      


}

   

void LoRA_RX_Interrupt_Routine() 
{
    uint8_t error =0;
    // If packet received, getPacket
    Serial.print("\nReceived Packet: ");
     rx_packet = 1; 
}

void LoRA_TX_Interrupt_Routine() {
    uint8_t error =0;
    Serial.println("tx packet."); 
    Serial.print(tx_state);
    switch(tx_state)
    {
        case TX_NONE: 
        case TX_COMPLETE: 
        case TX_SETTING_UP_TRANSMISSION:
            tx_state = TX_IN_TRANSMISSION;
            Serial.print("T");
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

uint8_t sx127x_print_error(const char *message, uint8_t length)
{ 
    // this function must be overridden by the application software. 
    Serial.write((const uint8_t*)message,length); 
    return 1;
}