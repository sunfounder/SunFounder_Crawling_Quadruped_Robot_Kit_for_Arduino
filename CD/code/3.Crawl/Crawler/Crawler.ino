/* -----------------------------------------------------------------------------
 - Project: Remote control Crawling robot
 - Author:	panerqiang@sunfounder.com
 - Date:	2015/1/27
 * -----------------------------------------------------------------------------
 - Overview
  - This project was written for the Crawling robot desigened by Sunfounder.
    This version of the robot has 4 legs, and each leg is driven by 3 servos.
	This robot is driven by a Ardunio Nano Board with an expansion Board.
	We recommend that you view the product documentation before using. 
 - Request
  - This project requires some library files, which you can find in the head of
    this file. Make sure you have installed these files.
 - How to
  - Before use,you must to adjust the robot,in order to make it more accurate.
    - Adjustment operation
	  1.uncomment ADJUST, make and run
	  2.comment ADJUST, uncomment VERIFY
	  3.measure real sites and set to real_site[4][3], make and run
	  4.comment VERIFY, make and run
	The document describes in detail how to operate.
 * ---------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <Servo.h>		//to define and control servos
#include <FlexiTimer2.h>//to set a timer to manage all servos
#include <EEPROM.h>		//to save errors of all servos
#include <SPI.h>		//nRF24L01 module need 1/3
#include <nRF24L01.h>	//nRF24L01 module need 2/3
#include <RF24.h>		//nRF24L01 module need 3/3
/* Installation and Adjustment -----------------------------------------------*/
//#define INSTALL	//uncomment only this to install the robot 	
//#define ADJUST	//uncomment only this to adjust the servos 
//#define VERIFY	//uncomment only this to verify the adjustment
const float adjust_site[3] = { 100, 80, 42 };
const float real_site[4][3] = { { 100, 80, 42 }, { 100, 80, 42 },
                                { 100, 80, 42 }, { 100, 80, 42 } };
/* Servos --------------------------------------------------------------------*/
//define 12 servos for 4 legs
Servo servo[4][3];
//define servos' ports
const int servo_pin[4][3] = { 2, 3, 4, 5, 6, 7, 14, 15, 16, 17, 18, 19 };
/* Wireless communication ----------------------------------------------------*/
//dfine RF24 for nRF24l01
RF24 radio(9, 10);	
//define RF24 transmit pipe
//NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL;
/* Size of the robot ---------------------------------------------------------*/
const float length_a = 55;
const float length_b = 80;
const float length_c = 22.75;
const float length_side = 66;
const float z_absolute = -12;
/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -10, z_boot = z_absolute;
const float x_default = 70, x_offset = 0;
const float y_start = 0, y_step = 50;
/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];		//real-time coordinates of the end of each leg 
volatile float site_expect[4][3];	//expected coordinates of the end of each leg
float temp_speed[4][3];		//each axis' speed, needs to be recalculated before each movement
float move_speed;			//movement speed
float speed_multiple = 1;	//movement speed multiple
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;			//+1/0.02s, for automatic rest  
const int wait_rest_time = 3 * 50;	//3s*50Hz, the time wait for automatic rest 
//functions' parameter
const float KEEP = 255;
//define PI for calculation
const float pi = 3.1415926;
/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b*cos(temp_alpha);
const float turn_y0 = temp_b*sin(temp_alpha) - turn_y1 - length_side;
/* ---------------------------------------------------------------------------*/

/*
 - setup function
 * ---------------------------------------------------------------------------*/
void setup()
{
#ifdef INSTALL
	//initialize all servos
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			servo[i][j].attach(servo_pin[i][j]);
			delay(100);
		}
	}
	while (1);
#endif
#ifdef ADJUST
	adjust();
	while (1);
#endif
#ifdef VERIFY
	verify();
	while (1);
#endif

	//start serial for debug
	Serial.begin(115200);
	Serial.println("Robot starts initialization");
	//start listen radio
	radio.begin();
	radio.openReadingPipe(1,pipe);
	radio.setRetries(0, 15);
	radio.setPALevel(RF24_PA_HIGH);
	radio.startListening();
	Serial.println("Radio listening started");
	//initialize default parameter
	set_site(0, x_default - x_offset, y_start + y_step, z_boot);
	set_site(1, x_default - x_offset, y_start + y_step, z_boot);
	set_site(2, x_default + x_offset, y_start, z_boot);
	set_site(3, x_default + x_offset, y_start, z_boot);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			site_now[i][j] = site_expect[i][j];
		}
	}
	//start servo service
	FlexiTimer2::set(20, servo_service);
	FlexiTimer2::start();
	Serial.println("Servo service started");
	//initialize servos
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			servo[i][j].attach(servo_pin[i][j]);
			delay(100);
		}
	}
	Serial.println("Servos initialized");
	Serial.println("Robot initialization Complete");
}

/*
 - loop function
 * ---------------------------------------------------------------------------*/
void loop()
{
#ifdef INSTALL
	while (1);
#endif
#ifdef ADJUST
	while (1);
#endif
#ifdef VERIFY
	while (1);
#endif

//put your code here ---------------------------------------------------------
	Serial.println("Waiting for radio signal");

	byte order;
	while (1)
	{
		if (radio.available())
		{
			if (radio.read(&order, 1))
			{
				if (order > 0)
				{
					Serial.print("Order:");
					Serial.print(order);
					Serial.print(" //");

					if (order < 5)
						if (!is_stand())
							stand();

					switch (order)
					{
					case 1:
						Serial.println("Step forward");
						step_forward(1);
						break;
					case 2:
						Serial.println("Step back");
						step_back(1);
						break;
					case 3:
						Serial.println("Turn left");
						turn_left(1);
						break;
					case 4:
						Serial.println("Turn right");
						turn_right(1);
						break;
					case 5:
						if (is_stand())
						{
							Serial.println("Sit");
							sit();
						}
						else
						{
							Serial.println("Stand");
							stand();
						}
						break;
					}

					rest_counter = 0;
				}
			}
			while (radio.read(&order, 1));
		}
		if (rest_counter > wait_rest_time)
		{
			if (is_stand())
			{
				Serial.println("Auto sit");
				sit();
				rest_counter = 0;
			}
		}
	}

//end of your code -----------------------------------------------------------
	while (1);
}

/*
 - adjustment function 
 - move each leg to adjustment site, so that you can measure the real sites.
 * ---------------------------------------------------------------------------*/
void adjust(void)
{
	//initializes eeprom's errors to 0
	//number -100 - +100 is map to 0 - +200 in eeprom 
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			EEPROM.write(i * 6 + j * 2, 100);
			EEPROM.write(i * 6 + j * 2 + 1, 100);
		}
	}

	//initializes the relevant variables to adjustment position
	for (int i = 0; i < 4; i++)
	{
		set_site(i, adjust_site[0], adjust_site[1], adjust_site[2] + z_absolute);
		for (int j = 0; j < 3; j++)
		{
			site_now[i][j] = site_expect[i][j];
		}
	}
	//start servo service
	FlexiTimer2::set(20, servo_service);
	FlexiTimer2::start();
	//initialize servos
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			servo[i][j].attach(servo_pin[i][j]);
			delay(100);
		}
	}
}

/*
 - verify function
 - verify the adjustment results, it will calculate errors and save to eeprom.
 * ---------------------------------------------------------------------------*/
void verify(void)
{
	//calculate correct degree
	float alpha0, beta0, gamma0;
	cartesian_to_polar(alpha0, beta0, gamma0, adjust_site[0], adjust_site[1], adjust_site[2] + z_absolute);
	//calculate real degree and errors
	float alpha, beta, gamma;
	float degree_error[4][3];
	for (int i = 0; i < 4; i++)
	{
		cartesian_to_polar(alpha, beta, gamma, real_site[i][0], real_site[i][1], real_site[i][2] + z_absolute);
		degree_error[i][0] = alpha0 - alpha;
		degree_error[i][1] = beta0 - beta;
		degree_error[i][2] = gamma0 - gamma;
	}
	//save errors to eeprom
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			EEPROM.write(i * 6 + j * 2, (int)degree_error[i][j] + 100);
			EEPROM.write(i * 6 + j * 2 + 1, (int)(degree_error[i][j] * 100) % 100 + 100);
		}
	}

	//initializes the relevant variables to adjustment position
	for (int i = 0; i < 4; i++)
	{
		set_site(i, adjust_site[0], adjust_site[1], adjust_site[2] + z_absolute);
		for (int j = 0; j < 3; j++)
		{
			site_now[i][j] = site_expect[i][j];
		}
	}
	//start servo service
	FlexiTimer2::set(20, servo_service);
	FlexiTimer2::start();
	//initialize servos
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			servo[i][j].attach(servo_pin[i][j]);
			delay(100);
		}
	}
}

/*
 - sit
 - blocking function
 * ---------------------------------------------------------------------------*/
void sit(void)
{
	move_speed = stand_seat_speed;
	for (int leg = 0; leg < 4; leg++)
	{
		set_site(leg, KEEP, KEEP, z_boot);
	}
	wait_all_reach();
}

/*
 - stand
 - blocking function
 * ---------------------------------------------------------------------------*/
void stand(void)
{
	move_speed = stand_seat_speed;
	for (int leg = 0; leg < 4; leg++)
	{
		set_site(leg, KEEP, KEEP, z_default);
	}
	wait_all_reach();
}

/*
 - is_stand
 * ---------------------------------------------------------------------------*/
bool is_stand(void)
{
	if (site_now[0][2] == z_default)
		return true;
	else
		return false;
}

/*
 - spot turn to left
 - blocking function
 - parameter step steps wanted to turn 
 * ---------------------------------------------------------------------------*/
void turn_left(unsigned int step)
{
	move_speed = spot_turn_speed;
	while (step-- > 0)
	{
		if (site_now[3][1] == y_start)
		{
			//leg 3&1 move
			set_site(3, x_default + x_offset, y_start, z_up);
			wait_all_reach();

			set_site(0, turn_x1 - x_offset, turn_y1, z_default);
			set_site(1, turn_x0 - x_offset, turn_y0, z_default);
			set_site(2, turn_x1 + x_offset, turn_y1, z_default);
			set_site(3, turn_x0 + x_offset, turn_y0, z_up);
			wait_all_reach();

			set_site(3, turn_x0 + x_offset, turn_y0, z_default);
			wait_all_reach();

			set_site(0, turn_x1 + x_offset, turn_y1, z_default);
			set_site(1, turn_x0 + x_offset, turn_y0, z_default);
			set_site(2, turn_x1 - x_offset, turn_y1, z_default);
			set_site(3, turn_x0 - x_offset, turn_y0, z_default);
			wait_all_reach();

			set_site(1, turn_x0 + x_offset, turn_y0, z_up);
			wait_all_reach();

			set_site(0, x_default + x_offset, y_start, z_default);
			set_site(1, x_default + x_offset, y_start, z_up);
			set_site(2, x_default - x_offset, y_start + y_step, z_default);
			set_site(3, x_default - x_offset, y_start + y_step, z_default);
			wait_all_reach();

			set_site(1, x_default + x_offset, y_start, z_default);
			wait_all_reach();
		}
		else
		{
			//leg 0&2 move
			set_site(0, x_default + x_offset, y_start, z_up);
			wait_all_reach();

			set_site(0, turn_x0 + x_offset, turn_y0, z_up);
			set_site(1, turn_x1 + x_offset, turn_y1, z_default);
			set_site(2, turn_x0 - x_offset, turn_y0, z_default);
			set_site(3, turn_x1 - x_offset, turn_y1, z_default);
			wait_all_reach();

			set_site(0, turn_x0 + x_offset, turn_y0, z_default);
			wait_all_reach();

			set_site(0, turn_x0 - x_offset, turn_y0, z_default);
			set_site(1, turn_x1 - x_offset, turn_y1, z_default);
			set_site(2, turn_x0 + x_offset, turn_y0, z_default);
			set_site(3, turn_x1 + x_offset, turn_y1, z_default);
			wait_all_reach();

			set_site(2, turn_x0 + x_offset, turn_y0, z_up);
			wait_all_reach();

			set_site(0, x_default - x_offset, y_start + y_step, z_default);
			set_site(1, x_default - x_offset, y_start + y_step, z_default);
			set_site(2, x_default + x_offset, y_start, z_up);
			set_site(3, x_default + x_offset, y_start, z_default);
			wait_all_reach();

			set_site(2, x_default + x_offset, y_start, z_default);
			wait_all_reach();
		}
	}
}

/*
 - spot turn to right
 - blocking function
 - parameter step steps wanted to turn
 * ---------------------------------------------------------------------------*/
void turn_right(unsigned int step)
{
	move_speed = spot_turn_speed;
	while (step-- > 0)
	{
		if (site_now[2][1] == y_start)
		{
			//leg 2&0 move
			set_site(2, x_default + x_offset, y_start, z_up);
			wait_all_reach();

			set_site(0, turn_x0 - x_offset, turn_y0, z_default);
			set_site(1, turn_x1 - x_offset, turn_y1, z_default);
			set_site(2, turn_x0 + x_offset, turn_y0, z_up);
			set_site(3, turn_x1 + x_offset, turn_y1, z_default);
			wait_all_reach();

			set_site(2, turn_x0 + x_offset, turn_y0, z_default);
			wait_all_reach();

			set_site(0, turn_x0 + x_offset, turn_y0, z_default);
			set_site(1, turn_x1 + x_offset, turn_y1, z_default);
			set_site(2, turn_x0 - x_offset, turn_y0, z_default);
			set_site(3, turn_x1 - x_offset, turn_y1, z_default);
			wait_all_reach();

			set_site(0, turn_x0 + x_offset, turn_y0, z_up);
			wait_all_reach();

			set_site(0, x_default + x_offset, y_start, z_up);
			set_site(1, x_default + x_offset, y_start, z_default);
			set_site(2, x_default - x_offset, y_start + y_step, z_default);
			set_site(3, x_default - x_offset, y_start + y_step, z_default);
			wait_all_reach();

			set_site(0, x_default + x_offset, y_start, z_default);
			wait_all_reach();
		}
		else
		{
			//leg 1&3 move
			set_site(1, x_default + x_offset, y_start, z_up);
			wait_all_reach();

			set_site(0, turn_x1 + x_offset, turn_y1, z_default);
			set_site(1, turn_x0 + x_offset, turn_y0, z_up);
			set_site(2, turn_x1 - x_offset, turn_y1, z_default);
			set_site(3, turn_x0 - x_offset, turn_y0, z_default);
			wait_all_reach();

			set_site(1, turn_x0 + x_offset, turn_y0, z_default);
			wait_all_reach();

			set_site(0, turn_x1 - x_offset, turn_y1, z_default);
			set_site(1, turn_x0 - x_offset, turn_y0, z_default);
			set_site(2, turn_x1 + x_offset, turn_y1, z_default);
			set_site(3, turn_x0 + x_offset, turn_y0, z_default);
			wait_all_reach();

			set_site(3, turn_x0 + x_offset, turn_y0, z_up);
			wait_all_reach();

			set_site(0, x_default - x_offset, y_start + y_step, z_default);
			set_site(1, x_default - x_offset, y_start + y_step, z_default);
			set_site(2, x_default + x_offset, y_start, z_default);
			set_site(3, x_default + x_offset, y_start, z_up);
			wait_all_reach();

			set_site(3, x_default + x_offset, y_start, z_default);
			wait_all_reach();
		}
	}
}

/*
 - go forward
 - blocking function
 - parameter step steps wanted to go
 * ---------------------------------------------------------------------------*/
void step_forward(unsigned int step)
{
	move_speed = leg_move_speed;
	while (step-- > 0)
	{
		if (site_now[2][1] == y_start)
		{
			//leg 2&1 move
			set_site(2, x_default + x_offset, y_start, z_up);
			wait_all_reach();
			set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
			wait_all_reach();
			set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
			wait_all_reach();

			move_speed = body_move_speed;

			set_site(0, x_default + x_offset, y_start, z_default);
			set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
			set_site(2, x_default - x_offset, y_start + y_step, z_default);
			set_site(3, x_default - x_offset, y_start + y_step, z_default);
			wait_all_reach();

			move_speed = leg_move_speed;

			set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
			wait_all_reach();
			set_site(1, x_default + x_offset, y_start, z_up);
			wait_all_reach();
			set_site(1, x_default + x_offset, y_start, z_default);
			wait_all_reach();
		}
		else
		{
			//leg 0&3 move
			set_site(0, x_default + x_offset, y_start, z_up);
			wait_all_reach();
			set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
			wait_all_reach();
			set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
			wait_all_reach();

			move_speed = body_move_speed;

			set_site(0, x_default - x_offset, y_start + y_step, z_default);
			set_site(1, x_default - x_offset, y_start + y_step, z_default);
			set_site(2, x_default + x_offset, y_start, z_default);
			set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
			wait_all_reach();

			move_speed = leg_move_speed;

			set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
			wait_all_reach();
			set_site(3, x_default + x_offset, y_start, z_up);
			wait_all_reach();
			set_site(3, x_default + x_offset, y_start, z_default);
			wait_all_reach();
		}
	}
}

/*
 - go back
 - blocking function
 - parameter step steps wanted to go
 * ---------------------------------------------------------------------------*/
void step_back(unsigned int step)
{
	move_speed = leg_move_speed;
	while (step-- > 0)
	{
		if (site_now[3][1] == y_start)
		{
			//leg 3&0 move
			set_site(3, x_default + x_offset, y_start, z_up);
			wait_all_reach();
			set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
			wait_all_reach();
			set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
			wait_all_reach();

			move_speed = body_move_speed;

			set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
			set_site(1, x_default + x_offset, y_start, z_default);
			set_site(2, x_default - x_offset, y_start + y_step, z_default);
			set_site(3, x_default - x_offset, y_start + y_step, z_default);
			wait_all_reach();

			move_speed = leg_move_speed;

			set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
			wait_all_reach();
			set_site(0, x_default + x_offset, y_start, z_up);
			wait_all_reach();
			set_site(0, x_default + x_offset, y_start, z_default);
			wait_all_reach();
		}
		else
		{
			//leg 1&2 move
			set_site(1, x_default + x_offset, y_start, z_up);
			wait_all_reach();
			set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
			wait_all_reach();
			set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
			wait_all_reach();

			move_speed = body_move_speed;

			set_site(0, x_default - x_offset, y_start + y_step, z_default);
			set_site(1, x_default - x_offset, y_start + y_step, z_default);
			set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
			set_site(3, x_default + x_offset, y_start, z_default);
			wait_all_reach();

			move_speed = leg_move_speed;

			set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
			wait_all_reach();
			set_site(2, x_default + x_offset, y_start, z_up);
			wait_all_reach();
			set_site(2, x_default + x_offset, y_start, z_default);
			wait_all_reach();
		}
	}
}

/*
 - microservos service /timer interrupt function/50Hz
 - when set site expected,this function move the end point to it in a straight line
 - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
 * ---------------------------------------------------------------------------*/
void servo_service(void)
{
	sei();
	static float alpha, beta, gamma;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
				site_now[i][j] += temp_speed[i][j];
			else
				site_now[i][j] = site_expect[i][j];
		}

		cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
		polar_to_servo(i, alpha, beta, gamma);
	}

	rest_counter++;
}

/*
 - set one of end points' expect site 
 - this founction will set temp_speed[4][3] at same time 
 - non - blocking function
 * ---------------------------------------------------------------------------*/
void set_site(int leg, float x, float y, float z)
{
	float length_x = 0, length_y = 0, length_z = 0;

	if (x != KEEP)
		length_x = x - site_now[leg][0];
	if (y != KEEP)
		length_y = y - site_now[leg][1];
	if (z != KEEP)
		length_z = z - site_now[leg][2];

	float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

	temp_speed[leg][0] = length_x / length * move_speed*speed_multiple;
	temp_speed[leg][1] = length_y / length * move_speed*speed_multiple;
	temp_speed[leg][2] = length_z / length * move_speed*speed_multiple;

	if (x != KEEP)
		site_expect[leg][0] = x;
	if (y != KEEP)
		site_expect[leg][1] = y;
	if (z != KEEP)
		site_expect[leg][2] = z;
}

/*
 - wait one of end points move to expect site 
 - blocking function 
 * ---------------------------------------------------------------------------*/
void wait_reach(int leg)
{
	while (1)
		if (site_now[leg][0] == site_expect[leg][0])
			if (site_now[leg][1] == site_expect[leg][1])
				if (site_now[leg][2] == site_expect[leg][2])
					break;
}

/*
 - wait one of end points move to one site 
 - blocking function 
 * ---------------------------------------------------------------------------*/
void wait_reach(int leg, float x, float y, float z)
{
	while (1)
		if (site_now[leg][0] == x)
			if (site_now[leg][1] == y)
				if (site_now[leg][2] == z)
					break;
}

/*
 - wait all of end points move to expect site 
 - blocking function 
 * ---------------------------------------------------------------------------*/
void wait_all_reach(void)
{
	for (int i = 0; i < 4; i++)
		wait_reach(i);
}

/*
 - trans site from polar to cartesian 
 - mathematical model 1/2
 * ---------------------------------------------------------------------------*/
void polar_to_cartesian(volatile float alpha, volatile float beta, volatile float gamma, volatile float &x, volatile float &y, volatile float &z)
{
	//trans degree 180->pi
	alpha = alpha / 180 * pi;
	beta = beta / 180 * pi;
	gamma = gamma / 180 * pi;
	//calculate w-z position
	float v, w;
	v = length_a*cos(alpha) - length_b*cos(alpha + beta);
	z = length_a*sin(alpha) - length_b*sin(alpha + beta);
	w = v + length_c;
	//calculate x-y-z position
	x = w*cos(gamma);
	y = w*sin(gamma);
}

/*
 - trans site from cartesian to polar 
 - mathematical model 2/2
 * ---------------------------------------------------------------------------*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
	//calculate w-z degree
	float v, w;
	w = (x >= 0 ? 1 : -1)*(sqrt(pow(x, 2) + pow(y, 2)));
	v = w - length_c;
	alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
	beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
	//calculate x-y-z degree
	gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
	//trans degree pi->180
	alpha = alpha / pi * 180;
	beta = beta / pi * 180;
	gamma = gamma / pi * 180;
}

/*
 - trans site from polar to microservos
 - mathematical model map to fact
 - the errors saved in eeprom will be add
 * ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
	float alpha_error = EEPROM.read(leg * 6 + 0) - 100 + ((float)EEPROM.read(leg * 6 + 1) - 100) / 100;
	float beta_error  = EEPROM.read(leg * 6 + 2) - 100 + ((float)EEPROM.read(leg * 6 + 3) - 100) / 100;
	float gamma_error = EEPROM.read(leg * 6 + 4) - 100 + ((float)EEPROM.read(leg * 6 + 5) - 100) / 100;

	alpha += alpha_error;
	beta += beta_error;
	gamma += gamma_error;

	if (leg == 0)
	{
		alpha = 90 - alpha;
		beta = beta;
		gamma = 90 - gamma;
	}
	else if (leg == 1)
	{
		alpha += 90;
		beta = 180 - beta;
		gamma += 90;
	}
	else if (leg == 2)
	{
		alpha += 90;
		beta = 180 - beta;
		gamma += 90;
	}
	else if (leg == 3)
	{
		alpha = 90 - alpha;
		beta = beta;
		gamma = 90 - gamma;
	}

	servo[leg][0].write(alpha);
	servo[leg][1].write(beta);
	servo[leg][2].write(gamma);
}
