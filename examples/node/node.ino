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
// Include the SX1272
#include "SX1272.h"
SX1272 sx1272 = SX1272();

#define BAND868


/*

#ifdef BAND868
  // Select frequency channel
  e = sx1272.setChannel(CH_10_868);
#else // assuming #defined BAND900
  // Select frequency channel
  e = sx1272.setChannel(CH_05_900);
#endif
 */

//#define BAND900


#define TEMP_SCALE  5000.0

#define DEFAULT_DEST_ADDR 1
#define LORAMODE  4
#define node_addr 6
#define field_index 3


#define TEMP_PIN_READ  A0
// use digital 8 to power the temperature sensor
#define TEMP_PIN_POWER 8
  


double temp;
unsigned long lastTransmissionTime=0;
unsigned long delayBeforeTransmit=0;
uint8_t message[100];
// receive window
uint16_t w_timer=1000;

int loraMode=LORAMODE;

uint8_t my_appKey[4]={5, 6, 7, 8};

const byte interruptPin = 2;

 
typedef enum {
    TX_NONE                                      = 0,
    TX_COMPLETE                                  = 1,
    TX_IN_TRANSMISSION                           = 2,
    TX_ERROR                                     = 3,
} SX127X_TX_Packet_States;



volatile SX127X_TX_Packet_States tx_state =TX_NONE; 

void setup()
{
  int e;
  
      // MSB, mode 0, 2Mhz 
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setDataMode(SPI_MODE0);

  // for the temperature sensor
  pinMode(TEMP_PIN_READ, INPUT);
  // and to power the temperature sensor
  pinMode(TEMP_PIN_POWER,OUTPUT);


  delay(3000);

  // Open serial communications and wait for port to open:
  Serial.begin(38400);

  // Print a start message
  Serial.println(F("SX1272 module and Arduino: send packets without ACK"));


  // Power ON the module
  sx1272.ON();

  
  // Set transmission mode and print the result
  e = sx1272.setMode(loraMode);
  Serial.print(F("Setting Mode: state "));
  Serial.println(e, DEC);

    sx1272._enableCarrierSense=false;  

  if (loraMode==1)
    w_timer=2500;

  e = sx1272.setChannel(CH_10_868);

  Serial.print(F("Setting Channel: state "));
  Serial.println(e, DEC);


  e = sx1272.setPower('x');
  Serial.print(F("Setting Power: state "));
  Serial.println(e);
  


  // Set the node address and print the result
  e = sx1272.setNodeAddress(node_addr);
  Serial.print(F("Setting node addr: state "));
  Serial.println(e, DEC);
  
  // Print a success message
  Serial.println(F("SX1272 successfully configured"));
 
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, FALLING);
  

  delay(500);
}



void loop(void)
{
  long startSend;
  long endSend;
  uint8_t app_key_offset=0;
  int e;

  tx_state = TX_COMPLETE; 

  delay(100);
  if (tx_state == TX_COMPLETE) 
  {
      int value = analogRead(TEMP_PIN_READ);

      // change here how the temperature should be computed depending on your sensor type
      //  
      temp = value*TEMP_SCALE/1024.0;
    
      Serial.print(F("Reading "));
      Serial.println(value);
      
      //temp = temp - 0.5;
      temp = temp / 10.0;

      Serial.print(F("(Temp is "));
      Serial.println(temp);

      uint8_t r_size;

      r_size=sprintf((char*)message+app_key_offset, "\\!#%d#%d", field_index, (int)temp);  

      Serial.print(F("Sending "));
      Serial.println((char*)(message+app_key_offset));

      Serial.print(F("Real payload size is "));
      Serial.println(r_size);
      
      int pl=r_size+app_key_offset;
  

      startSend=millis();

      // indicate that we have an appkey
      sx1272.setPacketType(PKT_TYPE_DATA | PKT_FLAG_DATA_WAPPKEY);

 
      // Send message to the gateway and print the result
      // with the app key if this feature is enabled
      e = sx1272.sendPacketTimeout(DEFAULT_DEST_ADDR, message, pl);
     
      endSend=millis();
  
      Serial.print(F("LoRa pkt seq "));
      Serial.println(sx1272.packet_sent.packnum);
    
      Serial.print(F("LoRa Sent in "));
      Serial.println(endSend-startSend);
          
      Serial.print(F("LoRa Sent w/CAD in "));

      Serial.println(endSend-sx1272._startDoCad);

      Serial.print(F("Packet sent, state "));
      Serial.println(e);
  
      lastTransmissionTime=millis();
      delayBeforeTransmit=1000+random(15,60)*100;
  }

}

   
void blink() {
  uint8_t e =0;
  Serial.println("rx packet."); 

  switch(tx_state)
  {
    case TX_COMPLETE: 
      tx_state = TX_IN_TRANSMISSION;
      break;
      
    case TX_IN_TRANSMISSION: 
      tx_state = TX_COMPLETE;
      break;
      
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
