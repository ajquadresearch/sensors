#include <Arduino.h>

//Receiver Pins//
const int ch1 = 24;  		//Right Stick - Horizontal (Roll)
const int ch2 = 25;			//Right Stick - Vertical (Pitch)
const int ch3 = 26; 		//Left Stick - Vertical (Throttle)
const int ch4 = 27;		    //Left Stick - Horizontal (Yaw)
const int ch5 = 28; 		//Left Nob (Start)
const int ch6 = 29; 		//Right Nob

//Receiver Settings//
const int deadBand = 10; 	    //Controller Performance Deadband [us]
const float pulseScale = 500.0;  //Pulse Full Scale [us]
const float maxPitch = 164.0;    //Max Desired Pitch [deg/s]
const float maxRoll = 164.0;     //Max Desired Roll [deg/S]
const float maxYaw = 64;         //Max Desired Yaw Rate [deg/s]
const float escMid = 1500;      //Middle Receiver Output Pulse [us]
const float freq = 250.0; 	      //Loop Frequency [Hz]

//Receiver Signals//
extern unsigned long time;   			   //Clock [us]
unsigned long timer[7] = {0}; 	          //Timer [us]
volatile int R[7] = {0};	          //Hand-Held Receiver Signals [us]
float pitchDes = 0, rollDes = 0, yawDes = 0;    //Desired Angles [deg]
float pitchDesD = 0, rollDesD = 0, yawDesD = 0; //Desired Rates [deg/s]

//External Interrupts//
void ch1Int(){  if (digitalReadFast(ch1)) timer[1] = time;  else R[1] = time - timer[1];}
void ch2Int(){ 	if (digitalReadFast(ch2)) timer[2] = time;  else R[2] = time - timer[2];}
void ch3Int(){ 	if (digitalReadFast(ch3)) timer[3] = time;  else R[3] = time - timer[3];}
void ch4Int(){  if (digitalReadFast(ch4)) timer[4] = time;  else R[4] = time - timer[4];}
void ch5Int(){ 	if (digitalReadFast(ch5)) timer[5] = time;  else R[5] = time - timer[5];}
void ch6Int(){  if (digitalReadFast(ch6)) timer[6] = time;	else R[6] = time - timer[6];}

void InitializeInputDevice()
{

    attachInterrupt(ch1,ch1Int,CHANGE);	attachInterrupt(ch2,ch2Int,CHANGE); attachInterrupt(ch3,ch3Int,CHANGE);  //Receiver Channels 1-3
    attachInterrupt(ch4,ch4Int,CHANGE);	attachInterrupt(ch5,ch5Int,CHANGE); attachInterrupt(ch6,ch6Int,CHANGE);  //Receiver Channels 4-6
    
    //Receiver Safety Check//
    while(R[3] == 0 || R[3] > 1100) Serial.println("Turn the receiver on and the throttle low.");
    while(R[5] > escMid) Serial.println("Turn the ignition off. It is the top leftmost receiver switch.");
}
void DesiredRates(){ 

  pitchDesD = (abs(R[2] - escMid) < deadBand) ? 0 : -(R[2] - escMid)*maxPitch/pulseScale;     //Desired Pitch Angle [deg]
  rollDesD = (abs(R[1] - escMid) < deadBand) ? 0 : (R[1] - escMid)*maxRoll/pulseScale;        //Desired Roll Angle [deg]
  yawDesD = (abs(R[4] - escMid) < deadBand) ? 0 : -(R[4] - escMid)*maxYaw/pulseScale;        //Desired Yaw Rate [deg/s]
  yawDes += yawDesD/freq;  if (yawDes < 0) yawDes += 360;  if (yawDes > 360) yawDes -= 360;  //Desired Yaw Angle [deg]
  // if (R[5] < escMid) yawDes = yaw;   need to attck files to IMU for this line to work
  
  pitchDes = 0;
  rollDes = 0;                                                        //Prevent Jumpstart

}
