/*
 * This sketch pipes data directly to/from a NRF24 transceiver to COSMOS running on a PC.
 * Stolen from: William Osman
 *
 * Modified by Trent Buckley 07/09/2021
 * 
 * There is an issue when trying to send too much data at the same time. I'm not sure which end it's on.
 * It's in the tens of bytes.
 * 
 */
 
#include <SPI.h>
#include "RF24.h"

#define SERIAL_SPEED  57600

#define PAYLOAD_BUF 64  //size of payload buffer

//NRF24 Radio
#define CE_PIN  15
#define CS_PIN  10



byte address[][6] = {"1Node", "2Node"};

RF24 radio(CE_PIN, CS_PIN); //(CE_pin, CS_pin)

HardwareSerial &SerialCom = Serial;

void setup() {
  // put your setup code here, to run once:
  SerialCom.begin(SERIAL_SPEED);
  radio_config();
  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:
  check_radio();
  check_serial();

}
//************** telemetry **********************************************
void check_serial()
{
  if(SerialCom.available())
  {
    uint8_t buf[PAYLOAD_BUF];
    uint8_t len = 0;  //length of buffer
    
    radio.stopListening();
    while(SerialCom.available())
    {
      buf[len] = SerialCom.read();
      len++;
    }
    radio.write(buf, len);
  }
    radio.startListening();
}

void check_radio()
{
  uint8_t data[PAYLOAD_BUF];
  if( radio.available() )
  {
    uint8_t len = radio.getDynamicPayloadSize();
    radio.read(data, len);

    for(int i = 0; i < len; i++)
    {
      Serial.write(data[i]);
    }
  }
}
//************** NRF24 RADIO *****************************************
void radio_config()
{
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicPayloads();
  //radio.setPayloadSize(sizeof(myData));
  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(1,address[0]);
  radio.setDataRate(RF24_1MBPS);
  
  //radio.setChannel(53);
  //radio.setRetries(2, 5);
  //radio.setAutoAck(1);

  radio.startListening();
}


