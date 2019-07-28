#include <Arduino.h>
#include "ActualAttitude.hpp"
#include "DesiredAttitude.hpp"

/////////////////////////////////////////////////////////
//GLOBAL VARIABLES 
/////////////////////////////////////////////////////////
elapsedMicros time;

// Variables for debugging
bool debug = true;
const int printTimer = 10000;
int lastPrint = 0;
const int led = 13;

// Filter sample rate
const int updateTime = 4000;
int lastUpdate = 0;

// IMU variable outputs 
extern float pitchActual, rollActual, yawActual;
extern float pitchRateActual, rollRateActual, yawRateActual;

//Receiver Pins//
const int ch1 = 24;  		//Right Stick - Horizontal (Roll)
const int ch2 = 25;			//Right Stick - Vertical (Pitch)
const int ch3 = 26; 		//Left Stick - Vertical (Throttle)
const int ch4 = 27;			//Left Stick - Horizontal (Yaw)
const int ch5 = 28; 		//Left Nob 
const int ch6 = 29;  		//Right Nob


//Receiver Signals// 			  
unsigned long timer[7] = {0}; 	            //Timer [us]
volatile int R[7] = {0};	          		//Hand-Held Receiver Signals [us]

// Desired attitude 
extern float pitchDesired, rollDesired, yawDesired;    		            //Desired Angles [deg]
extern float pitchRateDesired, rollRateDesired, yawRateDesired; 		//Desired Rates [deg/s]

//External Interrupts//
void ch1Int(){  if (digitalReadFast(ch1)) timer[1] = time;  else R[1] = time - timer[1];}
void ch2Int(){ 	if (digitalReadFast(ch2)) timer[2] = time;  else R[2] = time - timer[2];}
void ch3Int(){ 	if (digitalReadFast(ch3)) timer[3] = time;  else R[3] = time - timer[3];}
void ch4Int(){  if (digitalReadFast(ch4)) timer[4] = time;  else R[4] = time - timer[4];}
void ch5Int(){ 	if (digitalReadFast(ch5)) timer[5] = time;  else R[5] = time - timer[5];}
void ch6Int(){  if (digitalReadFast(ch6)) timer[6] = time;	else R[6] = time - timer[6];}

void setup() {

  // Connect to serial monitor if debugging 
  if( debug == true)
  {
	Serial.begin(115200);
  	while(!Serial);
  	Serial.println("DEBUGING");
  }

  // Intilizise the IMU 
  ImuIntilization();

  // Inilizise Interrupts
  attachInterrupt(ch1,ch1Int,CHANGE);	attachInterrupt(ch2,ch2Int,CHANGE); attachInterrupt(ch3,ch3Int,CHANGE);  //Receiver Channels 1-3
  attachInterrupt(ch4,ch4Int,CHANGE);	attachInterrupt(ch5,ch5Int,CHANGE); attachInterrupt(ch6,ch6Int,CHANGE);  //Receiver Channels 4-6

  // Setup completed 
  Serial.println("Finished Setup");
  digitalWrite(led,HIGH);
  delay(1000);

}

void loop() 
{
	// Update pulse to motors every 250hz
	if ((time - lastUpdate) > updateTime)
	{
		lastUpdate = time;

		// Get the rates and angles
		GetActualAttitude();
		GetDesiredAttitude();
	}

	if ((time - lastPrint) >= printTimer && debug == true)
	{
		lastPrint = time;
		Serial.println(rollRateDesired);
	}
}