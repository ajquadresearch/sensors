#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Mahony.h>
#include <Madgwick.h>
#define AHRS_VARIANT   NXP_FXOS8700_FXAS21002


/* 
 * QuadSat Hardware Test
 * Teensy 3.5
 * Adafruit NXP IMU
 * 
 * IMU angles
 *
 * 3/20/2019
 * 
 * MAKE SURE YOU CALIBRATE BEFORE USE 
 */


Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
elapsedMicros elapsedTime;


/////////////////////////////////////////
// IMU Calibration
/////////////////////////////////////////

// Offsets applied to raw x/y/z mag values
 float mag_offsets[3] = { 28.00F, 19.49F, 55.95F };

// Soft iron error compensation matrix
 float mag_softiron_matrix[3][3] = { {  0.898,  -0.059,  0.056 },
									 {  -0.059,  1.044, -0.026 },
									 {  0.056, -0.026,  1.074 } };

 float mag_field_strength = 48.76F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
 float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

////////////////////////////////////////
// Configurations
////////////////////////////////////////

// Filter sample rate
 int updateFreq = 250;
 int updateTime = 4000;

// Filter type
 //Mahony filter;
 Madgwick filter;

// How often to print variables
 int printTimer = 5000;

// Wait for Serial ?
 bool waitSerial = true;

// Print Angles 
 bool angle = true; 

// Print Rates 
bool rates = false; 

///////////////////////////////////
// Variables
//////////////////////////////////
// Pins 
 int led = 13;

// Printing
long lastUpdate = 0;
long lastPrint = 0;

// IMU 
 float pitch, roll, yaw;
 float pitch_rate, roll_rate, yaw_rate;
///////////////////////////////////////////////////////////////////////
// IMU
//////////////////////////////////////////////////////////////////////

void getIMU()
{
	 sensors_event_t gyro_event;
	 sensors_event_t accel_event;
	 sensors_event_t mag_event;
	 gyro.getEvent(&gyro_event);
	 accelmag.getEvent(&accel_event, &mag_event);

	 // Apply mag offset compensation (base values in uTesla)
	 float x = mag_event.magnetic.x - mag_offsets[0];
	 float y = mag_event.magnetic.y - mag_offsets[1];
	 float z = mag_event.magnetic.z - mag_offsets[2];

	 // Apply mag soft iron error compensation
	 float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
	 float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
	 float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

	 // Apply gyro zero-rate error compensation
	 float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
	 float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
	 float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

	 // The filter library expects gyro data in degrees/s, but adafruit sensor
	 // uses rad/s so we need to convert them first (or adapt the filter lib
	 // where they are being converted)
	 gx *= 57.2958F;
	 gy *= 57.2958F;
	 gz *= 57.2958F;

		filter.update(gx, gy, gz,
								 accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
								 mx, my, mz);

	 pitch = filter.getRoll();
	 roll = filter.getPitch();
	 yaw = filter.getYaw();
	 yaw_rate = gyro_event.gyro.z;
	 pitch_rate = gyro_event.gyro.x;
	 roll_rate = gyro_event.gyro.y;
}

////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() {
	pinMode(led,OUTPUT);
	Serial.begin(115200);
 
 // Still need to check for gyro if Serial is activated or not 
 	if (waitSerial == true)
	{
		 while(!Serial) {}
		 // Wait for serial monitor to activate
	}

	Serial.println("Setup ... ");
	delay(1000);
	Serial.print("Filter Update Frequency  ");
	Serial.println(updateFreq);
	Serial.print("Filter Update Timer  ");
	Serial.println(updateTime);
	delay(1000);

	if(!gyro.begin(GYRO_RANGE_250DPS))
	{
		/* There was a problem detecting the gyro ... check your connections */
		Serial.println("Ooops, no gyro detected ... Check your wiring!");
		while(1);
	}

	if(!accelmag.begin(ACCEL_RANGE_4G))
	{
		Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
		while(1);
	}

	digitalWrite(led,HIGH);
	filter.begin(updateFreq);
	Serial.print("Setup Finished!");
	delay(1000);

}

////////////////////////////////////////////////
// Main
/////////////////////////////////////////////
void loop() {

if ((elapsedTime - lastUpdate) > updateTime)
{
	 lastUpdate = elapsedTime;
	 getIMU();
}


if ((elapsedTime - lastPrint) >= printTimer)
{ 
	if (angle == true)
	{
		// Serial.print("Time Elapsed ");
	 	Serial.print(elapsedTime);
		// Serial.print("Pitch: ");
		Serial.print(", ");
		Serial.print(pitch);
		Serial.print(", ");
		//Serial.print("Roll: ");
		Serial.print(roll);
		// Serial.print("Yaw: ");
		Serial.print(", ");
		Serial.print(yaw);
		Serial.println();
	}
	if (rates == true)
	{
		Serial.println();
		Serial.print("Time Elapsed ");
	 	Serial.println(elapsedTime);
		Serial.print("Pitch Rate: ");
  	Serial.println(pitch_rate);
  	Serial.print("Roll Rate: ");
  	Serial.println(roll_rate);
  	Serial.print("Yaw Rate: ");
  	Serial.print(yaw_rate);
		Serial.println();
	}

	lastPrint = elapsedTime;
}
}

