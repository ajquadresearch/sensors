#include <Arduino.h>
#include "IMU.hpp"
#include "InputDevice.hpp"

/////////////////////////////////////////////////////////
//GLOBAL VARIABLES 
/////////////////////////////////////////////////////////
elapsedMicros time;

// Variables for debugging
const int printTimer = 10000;
int lastPrint = 0;

// Filter sample rate
const int updateTime = 4000;
int lastUpdate = 0;

// IMU variable outputs 
extern float pitch, roll, yaw;
extern float pitchD, rollD, yawD;

// Variables from input device 
extern float pitchDes, rollDes, yawDes;    //Desired Angles [deg]
extern float pitchDesD, rollDesD, yawDesD; //Desired Rates [deg/s]

void setup() {

  // Connect to serial monitor 
  Serial.begin(115200);
  while(!Serial);
  Serial.println("DEBUGING");

  // Intilizise the IMU 
  ImuIntilization();

  // Intilize the InputDevice
  InitializeInputDevice();

  // Setup completed 
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

    // Get the desired rates 
    DesiredRates();
	}

	if ((time - lastPrint) >= printTimer)
	{
		lastPrint = time;
		
    // IMU output 
    Serial.print(rollD);
		Serial.print(", ");
		Serial.print(pitchD);
		Serial.print(", ");
		Serial.print(yawD);
		Serial.print(", ");
		Serial.print(roll);
		Serial.print(", ");
		Serial.print(pitch);
		Serial.print(", ");
		Serial.println(yaw);

    // InputDevice Output
    Serial.print(pitchDesD); 
    Serial.print(", ");
    Serial.print(rollDesD);
    Serial.print(", ");
    Serial.print(yawDesD);


	}
}
