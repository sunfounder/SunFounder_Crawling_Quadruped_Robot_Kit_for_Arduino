/* -----------------------------------------------------------------------------
 - Project: Universal remote of mobile robot
 - Author:	panerqiang@sunfounder.com
 - Date:	2015/1/27
 * -----------------------------------------------------------------------------
 - Overview
  - This project was written for the universal remote of mobile robot desigened
    by Sunfounder, which can used to control the wireless control mobile robot.
    This remote use 6 different orders to control the robot, (byte)0 - (byte)5.
	You can modify it to suit your own robot.
 - Request
  - This project requires some library files, which you can find in the head of
    this file. Make sure you have installed these files.
 - How to
  - We recommend you to use [Remote control Crawling robot] desigened by Sunfounder.
    When your robot is prepared, you can control it.
 * -----------------------------------------------------------------------------
 - Connections
  - nRF24L01 to Arduino board
	1 GND   	GND
	2 VCC		3V3!! NOT 5V
	3 CE		D9
	4 CSN		D10
	5 SCK		D13
	6 MOSI		D11
	7 MISO		D12
	8 UNUSED	
 * ---------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
//nRF24L01 needed 
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
/* Private define ------------------------------------------------------------*/
//ports
#define CE_PIN   9
#define CSN_PIN 10
#define JOYSTICK_X A0
#define JOYSTICK_Y A1
#define JOYSTICK_K 2
/* Private constants ---------------------------------------------------------*/
// Define the transmit pipe
// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL;
/* Private variables ---------------------------------------------------------*/
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
int joystick[2];	// 2 element array holding Joystick readings
int key_pressed = 0;// Record the time button is pressed
byte serial_order;
byte order;

/*
 - setup function
 * ---------------------------------------------------------------------------*/
void setup()
{
	pinMode(JOYSTICK_K, INPUT_PULLUP);
	//start serial for debug and conrol 
	Serial.begin(115200);
	//start radio
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

	joystick[0] -= 512;
	joystick[1] = 512 - joystick[1];

	float alpha = atan2(joystick[1], joystick[0]);
	float power = sqrt(pow(joystick[0], 2) + pow(joystick[1], 2));

	if (power > 50)
	{
		if (alpha > -PI / 4 && alpha<=PI / 4)
			order = 4;
		else if (alpha > PI / 4 && alpha<=PI / 4 * 3)
			order = 1;
		else if (alpha > -PI / 4 * 3 && alpha<=-PI / 4)
			order = 2;
		else
			order = 3;
	}

	key_pressed = 0;
	while (digitalRead(JOYSTICK_K) == LOW)
	{
		delayMicroseconds(2);
		if (key_pressed++ > 10)
		{
			order = 5;
			break;
		}
	}

	if (serial_order > 0)
	{
		order = serial_order;
		serial_order = 0;
	}
	
	Serial.print("Order: ");
	Serial.println(order);

	radio.write(&order, 1);
	order = 0;
}

/*
 - serial received event 
 * ---------------------------------------------------------------------------*/
void serialEvent()
{
	serial_order = Serial.read();
}
