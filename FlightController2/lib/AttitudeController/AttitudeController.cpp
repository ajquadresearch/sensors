//////////////////////////////////////////////////////////////
//THIS SUBROUTINE DETERMINES THE SPEED OF THE MOTOR ROTATION//
//////////////////////////////////////////////////////////////

//Inputs 
extern float pitchActual, rollActual, yawActual;               //Desired Angles [deg]
extern float pitchRateActual, rollRateActual, yawRateActual;   //Desired Rates [deg/s]

// Desired attitude 
extern float pitchDesired, rollDesired, yawDesired;    		            //Desired Angles [deg]
extern float pitchRateDesired, rollRateDesired, yawRateDesired; 		//Desired Rates [deg/s]

extern volatile int R[7];

// Varables for controller

// Error Terms 
float errorPitchRate = 0;
float lastErrorPitchRate = 0;

float errorRollRate = 0;
float lastErrorRollRate = 0;

float errorYawRate = 0;
float lastErrorYawRate = 0;

// Proportional Terms
float proportionalPitch = 0;
float proportionalRoll = 0;
float proportionalYaw = 0;

// Integral Terms
float integralPitch = 0;
float integralRoll = 0;
float integralYaw = 0;

// Derivative terms 
float derivativePitch = 0;
float derivativeRoll = 0;
float derivativeYaw = 0;

// Output pulse
float pitchPulseRate = 0;
float rollPulseRate = 0;
float yawPulseRate = 0;

// Bound pulse 
float pitchRateMax = 300; 
float rollRateMax = 300; 
float yawRateMax = 300;

// esc Pulses
int escPulse1 = 0, escPulse2 = 0, escPulse3 = 0, escPulse4 = 0;

//Gains 
const float pPitch = 2, iPitch = 0.02, dPitch = 18;
const float pRoll = pPitch, iRoll = iPitch, dRoll = dPitch;
const float pYaw = 4, iYaw = 0.02, dYaw = 0;

// pulse bounds
int minPulse = 1100;
int maxPulse = 2000;


void GetAttitudeController()
{
    // Find error term
    errorPitchRate = pitchRateDesired - pitchRateActual;
    errorRollRate = rollRateDesired - rollRateActual;
    errorYawRate = yawRateDesired - yawRateActual;
    
    // Find proportional term
    proportionalPitch = pPitch * errorPitchRate;
    proportionalRoll = pRoll * errorRollRate;
    proportionalYaw = pYaw * errorYawRate;

    // Find derivative term
    derivativePitch = dPitch * (errorPitchRate - lastErrorPitchRate);
    derivativeRoll = dRoll * (errorRollRate - lastErrorRollRate);
    derivativeYaw = dYaw * (errorYawRate - lastErrorYawRate);

    // Find integral term (fucked for some reason)
    // integralPitch += iPitch * errorPitchRate;
    // integralRoll += iRoll * errorRollRate;
    // integralYaw += iYaw * errorYawRate;

    // Output Pulse 
    pitchPulseRate = proportionalPitch + derivativePitch + integralPitch;
    rollPulseRate = proportionalRoll + derivativeRoll + integralRoll;
    yawPulseRate = proportionalYaw + derivativeYaw + integralYaw;

    // Bound Output pulse 

    // Pitch
    if( pitchPulseRate > pitchRateMax)
        pitchPulseRate = pitchRateMax;

    if( pitchPulseRate < -pitchRateMax)
        pitchPulseRate = -pitchRateMax;

    // Roll 
    if( rollPulseRate > rollRateMax)
        rollPulseRate = rollRateMax;

    if( rollPulseRate < -rollRateMax)
        rollPulseRate = -rollRateMax;

    // Yaw 
    if( yawPulseRate > yawRateMax)
        yawPulseRate = yawRateMax;

    if( yawPulseRate < -yawRateMax)
        yawPulseRate = -yawRateMax;

    escPulse1 = R[3] + pitchPulseRate - rollPulseRate + yawPulseRate;
    escPulse2 = R[3] - pitchPulseRate - rollPulseRate - yawPulseRate;
    escPulse3 = R[3] - pitchPulseRate + rollPulseRate + yawPulseRate;
    escPulse4 = R[3] + pitchPulseRate + rollPulseRate - yawPulseRate;

    // Bound Max pulse 


    lastErrorPitchRate = errorPitchRate;
    lastErrorRollRate = errorRollRate;
    lastErrorYawRate = errorYawRate;

    // Upper Bound 
	if (escPulse1 > maxPulse)
	{
		escPulse1 = maxPulse;
	}

	if (escPulse2 > maxPulse)
	{
		escPulse2 = maxPulse;
	}

	if (escPulse3 > maxPulse)
	{
		escPulse3 = maxPulse;
	}

	if (escPulse4 > maxPulse)
	{
		escPulse4 = maxPulse;
	}

	// LowerBound 
	if (escPulse1 < minPulse)
	{
		escPulse1 = minPulse;
	}

	if (escPulse2 < minPulse)
	{
		escPulse2 = minPulse;
	}

	if (escPulse3 < minPulse)
	{
		escPulse3 = minPulse;
	}

	if (escPulse4 < minPulse)
	{
		escPulse4 = minPulse;
	}


}
