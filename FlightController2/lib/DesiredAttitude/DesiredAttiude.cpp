///////////////////////////////////////////////////////////////////////////////////////
//THIS SUBROUTINE DETERMINES THE DESIRED ATTITUDE OF THE QUADCOPTER USING AN RECIEVER//
///////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>

extern volatile int R[7];

const int deadBand = 8;	   //Controller Performance Deadband [us]
const int pulseScale = 500.0;  //Pulse Full Scale [us]
const int maxPitch = 164.0;    //Max Desired Pitch [deg/s]
const int maxRoll = 164.0;     //Max Desired Roll [deg/S]
const int maxYaw = 64;         //Max Desired Yaw Rate [deg/s]
const int escMid = 1500;	   //Middle Receiver Output Pulse [us]

float pitchDesired = 0, rollDesired = 0, yawDesired = 0;
float pitchRateDesired = 0, rollRateDesired = 0, yawRateDesired = 0;
extern float pitchActual, rollActual;

int autoPitch = 0;
int autoRoll = 0;



//Receiver Control Inputs//
void GetDesiredAttitude()
{

  // Find Desired Pitch Rate 
  if( (R[2] - escMid) >  deadBand)
    pitchRateDesired = escMid + deadBand - R[2];
  else if ( (R[2] - escMid) < -deadBand)
    pitchRateDesired = escMid - deadBand - R[2];
  else 
    pitchRateDesired = 0;
  
  // Find Desired Roll Rate 
  if( (R[1] - escMid) > deadBand)
    rollRateDesired = R[1] - escMid + deadBand;
  else if ( (R[1] -escMid) < -deadBand)
    rollRateDesired = R[1] - escMid - deadBand;
  else 
    rollRateDesired = 0;

    // Find Desired Yaw Rate 
  if( (R[4] - escMid) > deadBand)
    yawRateDesired = R[4] - escMid + deadBand;
  else if ( (R[4] -escMid) < -deadBand)
    yawRateDesired = R[4] - escMid - deadBand;
  else 
    yawRateDesired = 0;

  // AutoLevel
  autoPitch = 15 * pitchActual;
  autoRoll = 15 * rollActual;

  pitchRateDesired -= autoPitch;
  pitchRateDesired /= 3;

  rollRateDesired -= autoRoll;
  rollRateDesired /= 3; 

  yawRateDesired /= 3; 

  return;
}