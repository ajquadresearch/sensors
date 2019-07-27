#include <Arduino.h>
#include "imu.hpp"

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
		GetIMU();
	}

	if ((time - lastPrint) >= printTimer && debug == true)
	{
		lastPrint = time;
		Serial.println(yawRateActual);

	}
}