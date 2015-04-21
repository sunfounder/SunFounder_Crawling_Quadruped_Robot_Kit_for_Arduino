/* -----------------------------------------------------------------------------
 - File:   Servo Test
 - Author: panerqiang@sunfounder.com
 - Date:   2015/1/30
 * -----------------------------------------------------------------------------
 - Overview
  - This project was written to test a Servo, if it runs, the Servo will sweep.
 * ---------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <Servo.h> 
/* Private variables ---------------------------------------------------------*/
Servo myservo;  // create servo object to control a servo 
int pos = 0;    // variable to store the servo position 
/* ---------------------------------------------------------------------------*/
 
/*
 - setup function
 * ---------------------------------------------------------------------------*/
void setup() 
{ 
  myservo.attach(2);  // attaches the servo on pin 2 to the servo object 
} 
 
/*
 - loop function
 * ---------------------------------------------------------------------------*/
void loop() 
{ 
  for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(10);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(10);                       // waits 15ms for the servo to reach the position 
  } 
} 
