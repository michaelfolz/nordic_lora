
// Include the SX1272 
#include "SX1272.h"


SX1272 sx1272 = SX1272();
#define BAND868
//#define BAND900


// and SPI library on Arduino platforms
#include <SPI.h>
  
  #define PRINTLN                   Serial.println("")              
  #define PRINT_CSTSTR(fmt,param)   Serial.print(F(param))
  #define PRINT_STR(fmt,param)      Serial.print(param)
  #define PRINT_VALUE(fmt,param)    Serial.print(param)
  #define FLUSHOUTPUT               Serial.flush();



#define LORAMODE 4

#ifdef BAND868
  #define MAX_NB_CHANNEL 9
  #define STARTING_CHANNEL 10
  #define ENDING_CHANNEL 18
  uint8_t loraChannelIndex=0;
  uint32_t loraChannelArray[MAX_NB_CHANNEL]={CH_10_868,CH_11_868,CH_12_868,CH_13_868,CH_14_868,CH_15_868,CH_16_868,CH_17_868,CH_18_868};
#else // assuming #defined BAND900
  #define MAX_NB_CHANNEL 13
  #define STARTING_CHANNEL 0
  #define ENDING_CHANNEL 12
  uint8_t loraChannelIndex=5;
  uint32_t loraChannelArray[MAX_NB_CHANNEL]={CH_00_900,CH_01_900,CH_02_900,CH_03_900,CH_04_900,CH_05_900,CH_06_900,CH_07_900,CH_08_900,
                                            CH_09_900,CH_10_900,CH_11_900,CH_12_900};
#endif

// use the dynamic ACK feature of our modified SX1272 lib
#define GW_AUTO_ACK 



#define DEFAULT_DEST_ADDR 1

#define LORA_ADDR 1
// to unlock remote configuration feature
#define UNLOCK_PIN 1234
// will use 0xFF0xFE to prefix data received from LoRa, so that post-processing stage can differenciate
// data received from radio
#define WITH_DATA_PREFIX

#ifdef WITH_DATA_PREFIX
#define DATA_PREFIX_0 0xFF
#define DATA_PREFIX_1 0xFE

#endif



int dest_addr=DEFAULT_DEST_ADDR;

char sprintf_buf[100];
int msg_sn=0;

// number of retries to unlock remote configuration feature
uint8_t unlocked_try=3;
boolean unlocked=false;
boolean receivedFromSerial=false;
boolean receivedFromLoRa=false;
boolean withAck=false;


// configuration variables
//////////////////////////
bool radioON=false;
bool RSSIonSend=true;
uint8_t loraMode=LORAMODE;
uint32_t loraChannel=loraChannelArray[loraChannelIndex];
#if defined RADIO_RFM92_95 || defined RADIO_INAIR9B || defined RADIO_20DBM
// HopeRF 92W/95W and inAir9B need the PA_BOOST
// so 'x' set the PA_BOOST but then limit the power to +14dBm 
char loraPower='x';
#else
// other radio board such as Libelium LoRa or inAir9 do not need the PA_BOOST
// so 'M' set the output power to 15 to get +14dBm
char loraPower='M';
#endif
uint8_t loraAddr=LORA_ADDR;

unsigned int inter_pkt_time=0;
unsigned int random_inter_pkt_time=0;
long last_periodic_sendtime=0;

unsigned long startDoCad, endDoCad;
bool extendedIFS=true;
uint8_t SIFS_cad_number;
uint8_t send_cad_number=0;
uint8_t SIFS_value[11]={0, 183, 94, 44, 47, 23, 24, 12, 12, 7, 4};
uint8_t CAD_value[11]={0, 62, 31, 16, 16, 8, 9, 5, 3, 1, 1};

bool optAESgw=false;
uint16_t optBW=0; 
uint8_t optSF=0;
uint8_t optCR=0;
uint8_t optCH=0;
bool  optRAW=false;
double optFQ=-1.0;
uint8_t optSW=0x12;
  
//////////////////////////

#if defined ARDUINO && not defined _VARIANT_ARDUINO_DUE_X_ && not defined __MK20DX256__
int freeMemory () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
#endif



const byte ledPin = 13;
const byte interruptPin = 3;


void startConfig() {
  uint16_t preamblelength =0;
  int e;

  // has customized LoRa settings    
  if (optBW!=0 || optCR!=0 || optSF!=0) {

    e = sx1272.setCR(optCR-4);
    PRINT_CSTSTR("%s","^$LoRa CR ");
    PRINT_VALUE("%d", optCR);    
    PRINT_CSTSTR("%s",": state ");
    PRINT_VALUE("%d", e);
    PRINTLN;

    e = sx1272.setSF(optSF);
    PRINT_CSTSTR("%s","^$LoRa SF ");
    PRINT_VALUE("%d", optSF);    
    PRINT_CSTSTR("%s",": state ");
    PRINT_VALUE("%d", e);
    PRINTLN;
    
    e = sx1272.setBW( (optBW==125)?BW_125:((optBW==250)?BW_250:BW_500) );
    PRINT_CSTSTR("%s","^$LoRa BW ");
    PRINT_VALUE("%d", optBW);    
    PRINT_CSTSTR("%s",": state ");
    PRINT_VALUE("%d", e);
    PRINTLN;

    // indicate that we have a custom setting
    loraMode=0;
  
    if (optSF<10)
      SIFS_cad_number=6;
    else 
      SIFS_cad_number=3;
      
  }
  else {
    
    // Set transmission mode and print the result
    PRINT_CSTSTR("%s","^$LoRa mode ");
    PRINT_VALUE("%d", loraMode);
    PRINTLN;
        
    e = sx1272.setMode(loraMode);
    PRINT_CSTSTR("%s","^$Setting mode: state ");
    PRINT_VALUE("%d", e);
    PRINTLN;
  
  
    if (loraMode>7)
      SIFS_cad_number=6;
    else 
      SIFS_cad_number=3;

  }
  
  // Select frequency channel
  if (loraMode==11) {
    // if we start with mode 11, then switch to 868.1MHz for LoRaWAN test
    // Note: if you change to mode 11 later using command /@M11# for instance, you have to use /@C18# to change to the correct channel
    e = sx1272.setChannel(CH_18_868);
    PRINT_CSTSTR("%s","^$Channel CH_18_868: state ");    
  }
  else {
    // work also for loraMode 0
    e = sx1272.setChannel(loraChannel);

    if (optFQ>0.0) {
      PRINT_CSTSTR("%s","^$Frequency ");
      PRINT_VALUE("%f", optFQ);
      PRINT_CSTSTR("%s",": state ");      
    }
    else {
#ifdef BAND868      
      PRINT_CSTSTR("%s","^$Channel CH_1");
      PRINT_VALUE("%d", loraChannelIndex);
      PRINT_CSTSTR("%s","_868: state ");
#else
      PRINT_CSTSTR("%s","^$Channel CH_");
      PRINT_VALUE("%d", loraChannelIndex);
      PRINT_CSTSTR("%s","_900: state ");
#endif
    }
  }  
  PRINT_VALUE("%d", e);
  PRINTLN; 
  
  // Select output power (Max, High or Low)
  e = sx1272.setPower(loraPower);

  PRINT_CSTSTR("%s","^$Set LoRa Power to ");
  PRINT_VALUE("%c",loraPower);  
  PRINTLN;
                
  PRINT_CSTSTR("%s","^$Power: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;
 
  // get preamble length
  preamblelength = sx1272.getPreambleLength();
  PRINT_CSTSTR("%s","^$Get Preamble Length: state ");
  PRINT_VALUE("%d", e);
  PRINTLN; 
  PRINT_CSTSTR("%s","^$Preamble Length: ");
  PRINT_VALUE("%d", preamblelength);
  PRINTLN;
  
  // Set the node address and print the result
  //e = sx1272.setNodeAddress(loraAddr);
  sx1272._nodeAddress=loraAddr;
  e=0;
  PRINT_CSTSTR("%s","^$LoRa addr ");
  PRINT_VALUE("%d", loraAddr);
  PRINT_CSTSTR("%s",": state ");
  PRINT_VALUE("%d", e);
  PRINTLN;


  // Print a success message
  PRINT_CSTSTR("%s","^$SX1272/76 configured ");

  PRINT_CSTSTR("%s","as LR-BS. Waiting RF input for transparent RF-serial bridge\n");
 
}

volatile uint8_t rx_packet =0 ;
void setup()
{
  int e;

  delay(3000);

      // MSB, mode 0, 2Mhz 
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setDataMode(SPI_MODE0);

  randomSeed(analogRead(14));

  // Open serial communications and wait for port to open:
  Serial.begin(38400);


  // Print a start message
  Serial.print(freeMemory());
  Serial.println(F(" bytes of free memory.")); 


  // Power ON the module
  e = sx1272.ON();
  
  PRINT_CSTSTR("%s","^$**********Power ON: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;

  if (optSW!=0x12) {
    e = sx1272.setSyncWord(optSW);

    PRINT_CSTSTR("%s","^$Set sync word to 0x");
    Serial.print(optSW, HEX);    
    PRINTLN;
    PRINT_CSTSTR("%s","^$LoRa sync word: state ");
    PRINT_VALUE("%d",e);  
    PRINTLN;
  }
  
  if (!e) {
    radioON=true;
    startConfig();
  }

  
  
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
  
  delay(1000);

}


void loop(void)
{ 
  int i=0, e;
  int cmdValue;

  receivedFromLoRa=false;

  sx1272.receive();
      
  while (radioON) 
  {
       delay(1000);   
       
       //e=  sx1272.availableData(0);
       PRINT_STR("%s","wait again");

      if (rx_packet) 
      {
         // If packet received, getPacket
       
         PRINT_STR("%s","RX packet time");
          
        rx_packet = 0;
      
        
       sx1272.receive();


         int16_t rssi_packet = sx1272.getRSSIpacket();

         uint8_t tmp_length=sx1272._payloadlength;
         
         sprintf(sprintf_buf,"--- rxlora. dst=%d type=0x%.2X src=%d seq=%d len=%d SNR=%d RSSIpkt=%d\n", 
                   sx1272._packet_received.dst,
                   sx1272._packet_received.type, 
                   sx1272._packet_received.src,
                   sx1272._packet_received.packnum,
                   tmp_length, 
                   sx1272._SNR,
                   rssi_packet);
                   
         PRINT_STR("%s",sprintf_buf);

         // provide a short output for external program to have information about the received packet
         // ^psrc_id,seq,len,SNR,RSSI
         sprintf(sprintf_buf,"^p%d,%d,%d,%d,%d,%d,%d\n",
                   sx1272._packet_received.dst,
                   sx1272._packet_received.type,                   
                   sx1272._packet_received.src,
                   sx1272._packet_received.packnum, 
                   tmp_length,
                   sx1272._SNR,
                   rssi_packet);
 
         PRINT_STR("%s",sprintf_buf);  

    
       
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


void blink() {
  uint8_t e =0;
  Serial.println("rx packet."); 
  e=sx1272.getPacket();
  if(!e)
         rx_packet = 1; 
          
}
