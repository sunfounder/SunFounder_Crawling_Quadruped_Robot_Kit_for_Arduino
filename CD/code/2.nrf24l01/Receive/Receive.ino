/* -----------------------------------------------------------------------------
 - File:   nRF24l01 Receive Test
 - Author: panerqiang@sunfounder.com
 - Date:   2015/1/31
 * -----------------------------------------------------------------------------
 - Overview
  - This project was written to test nRF24l01 module. If it runs, arduino will 
    receives data from another transceiver with 2 Analog values from a Joystick
    or 2 Potentiometers Displays received values on Serial Monitor.
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
 * ---------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
/* Ports ---------------------------------------------------------------------*/
#define CE_PIN   9
#define CSN_PIN 10
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
  Serial.begin(9600);
  delay(1000);
  Serial.println("nRF24l01 Receiver Starting");
  radio.begin();
  radio.setRetries(0, 15);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(1,pipe);
  radio.startListening();;
}

/*
 - loop function
 * ---------------------------------------------------------------------------*/
void loop()
{
  if ( radio.available() )
  {
    // Read the data payload until we've received everything
    bool done = false;
    while (!done)
    {
      // Fetch the data payload
      done = radio.read( joystick, sizeof(joystick) );
      Serial.print("X = ");
      Serial.print(joystick[0]);
      Serial.print(" Y = ");      
      Serial.println(joystick[1]);
    }
  }
  else
  {    
    Serial.println("No radio available");
  }
}
