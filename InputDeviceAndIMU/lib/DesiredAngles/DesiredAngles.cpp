#include <Arduino.h>

float pitchDesD = 0, rollDesD = 0, yawDesD = 0;
extern volatile int R[7];

//Receiver Settings//
const float deadBand = 10; 	    //Controller Performance Deadband [us]
const float pulseScale = 500.0;  //Pulse Full Scale [us]
const float maxPitch = 164.0;    //Max Desired Pitch [deg/s]
const float maxRoll = 164.0;     //Max Desired Roll [deg/S]
const float maxYaw = 64;         //Max Desired Yaw Rate [deg/s]
const float escMid = 1500;	      //Middle Receiver Output Pulse [us]
const float freq = 250.0; 	      //Loop Frequency [Hz]

void GetDesiredRates(){

  pitchDesD = (abs(R[2] - escMid) < deadBand) ? 0 : -(R[2] - escMid)*maxPitch/pulseScale;     //Desired Pitch Angle [deg]
  rollDesD = (abs(R[1] - escMid) < deadBand) ? 0 : (R[1] - escMid)*maxRoll/pulseScale;        //Desired Roll Angle [deg]
  yawDesD = (abs(R[4] - escMid) < deadBand) ? 0 : -(R[4] - escMid)*maxYaw/pulseScale;        //Desired Yaw Rate [deg/s]
  // yawDes += yawDesD/freq;  if (yawDes < 0) yawDes += 360;  if (yawDes > 360) yawDes -= 360;  //Desired Yaw Angle [deg]
  // if (R[5] < escMid) yawDes = yaw;   
  
  // pitchDes = 0;
  // rollDes = 0;                                                        //Prevent Jumpstart

  return;
}