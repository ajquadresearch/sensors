///////////////////////////////////////////////////////////////////////////////////////
//THIS SUBROUTINE DETERMINES THE DESIRED ATTITUDE OF THE QUADCOPTER USING AN RECIEVER//
///////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>

extern volatile int R[7];

const int deadBand = 10;	   //Controller Performance Deadband [us]
const int pulseScale = 500.0;  //Pulse Full Scale [us]
const int maxPitch = 164.0;    //Max Desired Pitch [deg/s]
const int maxRoll = 164.0;     //Max Desired Roll [deg/S]
const int maxYaw = 64;         //Max Desired Yaw Rate [deg/s]
const int escMid = 1500;	   //Middle Receiver Output Pulse [us]

float pitchDesired = 0, rollDesired = 0, yawDesired = 0;
float pitchRateDesired = 0, rollRateDesired = 0, yawRateDesired = 0;



//Receiver Control Inputs//
void GetDesiredAttitude()
{

  pitchRateDesired = (abs(R[2] - escMid) < deadBand) ? 0 : -(R[2] - escMid)*maxPitch/pulseScale;     //Desired Pitch Angle [deg]
  rollRateDesired = (abs(R[1] - escMid) < deadBand) ? 0 : (R[1] - escMid)*maxRoll/pulseScale;        //Desired Roll Angle [deg]
  yawRateDesired = (abs(R[4] - escMid) < deadBand) ? 0 : (R[4] - escMid)*maxYaw/pulseScale;        //Desired Yaw Rate [deg/s]                                                      //Prevent Jumpstart

}