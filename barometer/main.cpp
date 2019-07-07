/*  Teensy uses SCL0 and SDA0 pins by default. The wiring diagram based on the wires I have in my barometer is:
  GND (black) to GND
  Vin (red) to 3.3V
  SCL (blue) to pin 19 (SCL0)
  SDA (white) to pin 18 (SDA0)
  */



#include <Arduino.h>
#include <i2c_t3.h>   //Wire library for Teensy 3.x

//During flight the battery voltage drops and the motors are spinning at a lower RPM. This has a negative effecct on the
//altitude hold function. With the battery_compensation variable it's possible to compensate for the battery voltage drop.
//Increase this value when the quadcopter drops due to a lower battery voltage during a non altitude hold flight.
float battery_compensation = 40.0;         

uint8_t MS5611_address = 0x77;             //The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.
uint8_t compass_address = 0x1E;            //The I2C address of the HMC5883L is 0x1E in hexadecimal form.

float low_battery_warning = 10.5;          //Set the battery warning at 10.5V (default = 10.5V).

//Tuning parameters/settings is explained in this video: https://youtu.be/ys-YpOaA2ME
#define variable_1_to_adjust dummy_float   //Change dummy_float to any setting that you want to tune.
#define variable_2_to_adjust dummy_float   //Change dummy_float to any setting that you want to tune.
#define variable_3_to_adjust dummy_float   //Change dummy_float to any setting that you want to tune.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer

uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint8_t check_byte, flip32, start;
uint8_t error, error_counter, error_led;
uint8_t flight_mode, flight_mode_counter, flight_mode_led;
uint8_t takeoff_detected, manual_altitude_change;
uint8_t telemetry_send_byte, telemetry_bit_counter, telemetry_loop_counter;
uint8_t channel_select_counter;
uint8_t level_calibration_on;

uint32_t telemetry_buffer_byte;

int16_t esc_1, esc_2, esc_3, esc_4;
int16_t manual_throttle;
int16_t throttle, takeoff_throttle, cal_int;
int16_t temperature, count_var;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;

int32_t channel_1_start, channel_1, pid_roll_setpoint_base;
int32_t channel_2_start, channel_2, pid_pitch_setpoint_base;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t measured_time, measured_time_start;
int32_t acc_total_vector, acc_total_vector_at_start;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int16_t acc_pitch_cal_value;
int16_t acc_roll_cal_value;

int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total ;
int16_t acc_z_average_short[26], acc_z_average_long[51];

uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;

int32_t acc_alt_integrated;

uint32_t loop_timer, error_timer, flight_mode_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
float battery_voltage, dummy_float;

//Pressure variables.
float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT, dT_C5;
double altitude, referenceAltitude;
double seaLevelPressure = 101325;                     //Needed for altitude calculations, taken from google (same value as used in JoopBarometerTest)
//Altitude PID variables
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;


//Adjust settings online
uint32_t setting_adjust_timer;
uint16_t setting_click_counter;
uint8_t previous_channel_6;
float adjustable_setting_1, adjustable_setting_2, adjustable_setting_3;


//******************************************************************************************************************************************************


void read_barometer(void) {
  barometer_counter ++;
  
  //Every time this function is called the barometer_counter variable is incremented. This way a specific action
  //is executed at the correct moment. This is needed because requesting data from the MS5611 takes around 9ms to complete.

  if (barometer_counter == 1) {                                                 //When the barometer_counter variable is 1.
    if (temperature_counter == 0) {                                             //And the temperature counter is 0.
      //Get temperature data from MS-5611
      Wire.beginTransmission(MS5611_address);                                   //Open a connection with the MS5611
      Wire.write((u_int8_t)0x00);                                                         //Send a 0 to indicate that we want to poll the requested data.
      Wire.endTransmission();                                                   //End the transmission with the MS5611.

      Wire.requestFrom(MS5611_address, 3);                                       //Poll 3 data bytes from the MS5611.
      delayMicroseconds(100);
      // Store the temperature in a 5 location rotating memory to prevent temperature spikes.
      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      
      raw_temperature_rotating_memory[average_temperature_mem_location] = (Wire.read() << 16 | Wire.read() << 8 | Wire.read());

      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;                      //Calculate the avarage temperature of the last 5 measurements.
    }
    else {
      //Get pressure data from MS-5611
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      Wire.write((u_int8_t)0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
 
      Wire.requestFrom(MS5611_address, 3);                                     //Poll 3 data bytes from the MS5611.
      delayMicroseconds(100);
      raw_pressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read();     //Shift the individual bytes in the correct position and add them to the raw_pressure variable.

    }

    temperature_counter ++;                                                     //Increase the temperature_counter variable.
    if (temperature_counter == 20) {                                            //When the temperature counter equals 20.
      temperature_counter = 0;                                                  //Reset the temperature_counter variable.
      //Request temperature data
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      Wire.write((u_int8_t)0x58);                                                        //Send a 0x58 to indicate that we want to request the temperature data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
    }
    else {                                                                      //If the temperature_counter variable does not equal 20.
      //Request pressure data
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
      Wire.write((u_int8_t)0x48);                                                        //Send a 0x48 to indicate that we want to request the pressure data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
    }
  }
  if (barometer_counter == 2) {                                                 //If the barometer_counter variable equals 2.
    //Calculate pressure as explained in the datasheet of the MS-5611.
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
    //To get a smoother pressure value we will use a 20 location rotating memory.
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                                //Calculate the new change between the actual pressure and the previous measurement.
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term avarage value.
    pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_total_avarage / 20.0;                                              //Calculate the average pressure of the last 20 pressure readings.

    //To get better results we will use a complementary fillter that can be adjusted by the fast average.
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Calculate the difference between the fast and the slow avarage value.
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
    //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;                                                                   //The actual_pressure is used in the program for altitude calculations.
    altitude = (44330.0f * (1.0f - pow((double)actual_pressure / (double)seaLevelPressure, 0.1902949f)));
    Serial.print("actual pressure: ");
    Serial.println(actual_pressure);
    Serial.print("altitude: ");
    Serial.println(altitude);
  }

  if (barometer_counter == 3) {                                                                               //When the barometer counter is 3
    barometer_counter = 0;                                                                                    //Set the barometer counter to 0 for the next measurements.

  }
}

//*******************************************************************************************************************************************

void setup() {  
  Serial.begin(9600);
  
  //Check if the MS5611 barometer is responding.
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000); //Start the I2C as master  *IMPORTANT FOR HIGH-SPEED I2C ON TEENSY*
  Wire.beginTransmission(MS5611_address);                      //Start communication with the MS5611.
  error = Wire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the MS5611 did not responde.
    Wire.beginTransmission(MS5611_address);
    error = Wire.endTransmission();
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }

  //Create a 5 second delay before calibration.
  for (count_var = 0; count_var < 1250; count_var++) {          //1250 loops of 4 microseconds = 5 seconds.
    if (count_var % 125 == 0) {                                 //Every 125 loops (500ms).
      //digitalWrite(PB4, !digitalRead(PB4));                     //Change the led status.
    }
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }
  count_var = 0;                                                //Set start back to 0.

  
  //For calculating the pressure the 6 calibration values need to be polled from the MS5611.
  //These 2 byte values are stored in the memory location 0xA2 and up.
  for (start = 1; start <= 6; start++) {
    Wire.beginTransmission(MS5611_address);                    //Start communication with the MPU-6050.
    Wire.write(0xA0 + start * 2);                              //Send the address that we want to read.
    Wire.endTransmission();                                    //End the transmission.

    Wire.requestFrom(MS5611_address, 2);                       //Request 2 bytes from the MS5611.
    C[start] = Wire.read() << 8 | Wire.read();                //Add the low and high byte to the C[x] calibration variable.
  }

  OFF_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
  SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.

  //The MS5611 needs a few readings to stabilize.
  for (start = 0; start < 100; start++) {                       //This loop runs 100 times.
    read_barometer();                                           //Read and calculate the barometer data.
    delay(4);                                                   //The main program loop also runs 250Hz (4ms per loop).
  }

  actual_pressure = 0;                                          //Reset the pressure calculations.

  loop_timer = micros();                                        //Set the timer for the first loop.
}

//******************************************************************************************************************************************************

void loop() {
    int t=micros();
    read_barometer();
    
  delay(4);

  if (micros() - loop_timer > 4050)error = 2;                                      //Output an error if the loop time exceeds 4050us.
  while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
  loop_timer = micros();                                                           //Set the timer for the next loop.
}

//***************************************************************************************************************************************************
