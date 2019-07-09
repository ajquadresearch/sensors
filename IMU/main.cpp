#include <Arduino.h>
#include "imu.hpp"

/////////////////////////////////////////////////////////
//GLOBAL VARIABLES 
/////////////////////////////////////////////////////////
elapsedMicros time;

bool debug = true; 

// Variables for debugging
const int printTimer = 10000;
int lastPrint = 0;

// led 
int led = 13; 

// Filter sample rate
const int updateTime = 4000;
int lastUpdate = 0;

// IMU variable outputs 
extern float pitch, roll, yaw;
extern float pitchD, rollD, yawD;

void setup() {

  // Connect to serial monitor 
  Serial.begin(115200);
  while(!Serial);
  Serial.println("DEBUGING");

  // Intilizise the IMU 
  ImuIntilization();

  // Setup completed 
  digitalWrite(led,HIGH);
  Serial.println("Finished Setup");
  delay(1000);

}

void loop() 
{
	// Update pulse to motors every 250hz
	if ((time - lastUpdate) > updateTime)
	{
		lastUpdate = time;

		// Get the rates and angles
		GetIMU();
	}

	// Check output variables
	if (debug == true)
	{
		if ((time - lastPrint) >= printTimer)
		{
			lastPrint = time;
			Serial.println(yawD);
		}
	}
}
