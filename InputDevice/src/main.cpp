#include <Arduino.h>
#include "InputDevice.hpp"

const int led = 13;
elapsedMicros time;

// Variables for debugging
const int printTimer = 10000;
int lastPrint = 0;

extern float pitchDes, rollDes, yawDes;    //Desired Angles [deg]
extern float pitchDesD, rollDesD, yawDesD; //Desired Rates [deg/s]

void setup() {
  // put your setup code here, to run once:

  // Connect to serial monitor 
  Serial.begin(115200);
  while(!Serial);
  Serial.println("DEBUGING");

  void InitializeInputDevice();

  // Setup completed 
  digitalWrite(led,HIGH);
  Serial.println("Finished Setup");
  delay(1000);
}

void loop() {
  if ((time - lastPrint) >= printTimer)
		{
			lastPrint = time;
			Serial.println(pitchDesD);
		}
}
