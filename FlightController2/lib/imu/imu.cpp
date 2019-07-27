#include <Adafruit_Sensor.h> 				
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Madgwick.h>

 // Filter sample rate
const int updateFreq = 250;

//Mag Bias Calibration [uT]//
float mag_offsets[3] = { 20.74F, -34.59F, 42.05F };

//Gyro Bias Calibration [deg/s]//
// Offsets applied to compensate for gyro zero-drift error for x/y/z
 float gyro_zero_offsets[3] = { 0.0F, 0.0F, 0.0F };

//Angular Kinematics//
float pitchActual = 0, rollActual = 0, yawActual = 0;     //Madgwick Euler Angles [deg]
float pitchRateActual = 0, rollRateActual = 0, yawRateActual = 0;  //Gyro Body Angular Rates [deg/s]

//Magnetometer Calibration//
float mag_field_strength = 37.27F;                           //Magnetic Field Strength [uT]
float mag_softiron_matrix[3][3] = {  { 0.978, -0.035,  0.020 },
								   	 { -0.035,  0.987, -0.042 },
								   	 { 0.020, -0.042,  1.039 } };

//Sensors and Filtering//
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);        //Gyroscope Model 
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);  //Accelerometer and Magnetometer Model 
Madgwick filter;                                                   //Madgwick AHRS Euler Angle Filter

void ImuIntilization()
{

	// Gyro
	while(!gyro.begin(GYRO_RANGE_500DPS))
		Serial.println("Incorrect Wiring");

	// Accellerometer
	while(!accelmag.begin(ACCEL_RANGE_4G))
		Serial.println("Incorrect Wiring");

	// Start IMU filter for desired frequency
	filter.begin(updateFreq);

	return;

}

 void GetIMU()
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

	 // Degrees (LOOKS LIKE WE WON'T NEED OFFSET FOR ANGLES)
	 pitchActual = filter.getRoll();
	 rollActual = filter.getPitch();
	 yawActual = -1*filter.getYaw(); 										// negative sign added to correct sign convention

	 // Degrees per second 
	 pitchRateActual = gyro_event.gyro.x*(180/3.14);
	 rollRateActual = gyro_event.gyro.y*(180/3.14);
	 yawRateActual = -1*gyro_event.gyro.z*(180/3.14);    // negative sign added to correct sign convention

	return; 
}