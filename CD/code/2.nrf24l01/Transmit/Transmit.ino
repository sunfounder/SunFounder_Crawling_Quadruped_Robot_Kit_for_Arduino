/* -----------------------------------------------------------------------------
 - File:   nRF24l01 Transmit Test
 - Author: panerqiang@sunfounder.com
 - Date:   2015/1/31
 * -----------------------------------------------------------------------------
 - Overview
  - This project was written to test nRF24l01 module. If it runs, arduino will 
    reads Analog values on A0, A1 and transmits them over a nRF24L01 Radio Link 
    to another transceiver.
 - Request
   - FR24 library
 - Connections
   - nRF24L01 to Arduino
     1 GND   	 GND
     2 VCC	 3V3!! NOT 5V
     3 CE	 D9
     4 CSN	 D10
     5 SCK	 D13
     6 MOSI	 D11
     7 MISO	 D12
     8 UNUSED	
   - Joystick to Arduino
     GND         GND
     VCC         5V
     X           A0
     Y           A1
 * ---------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
/* Ports ---------------------------------------------------------------------*/
#define CE_PIN   9
#define CSN_PIN 10
#define JOYSTICK_X A0
#define JOYSTICK_Y A1
/* nRF24l01 ------------------------------------------------------------------*/
// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
/* Joystick ------------------------------------------------------------------*/
int joystick[2]; // 2 element array holding Joystick readings
/* ---------------------------------------------------------------------------*/

/*
 - setup function
 * ---------------------------------------------------------------------------*/
void setup()
{
  radio.begin();
  radio.setRetries(0, 15);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(pipe);
}

/*
 - loop function
 * ---------------------------------------------------------------------------*/
void loop()
{
  joystick[0] = analogRead(JOYSTICK_X);
  joystick[1] = analogRead(JOYSTICK_Y);
  
  radio.write( joystick, sizeof(joystick) );
}
