#include <Arduino.h>
// SPDX-FileCopyrightText: 2023 Carter Nelson for Adafruit Industries
//
// SPDX-License-Identifier: MIT
// --------------------------------------
// i2c_scanner
//
// Modified from https://playground.arduino.cc/Main/I2cScanner/
// --------------------------------------
#include <SPI.h>
// #include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// #define WIRE Wire

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 16.67;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
// Adafruit_BNO055(5,0x28,)
// Wire.setPins(16,17);
// Wire.begin();

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
void printEvent(sensors_event_t* event);

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin(OPERATION_MODE_M4G))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setAxisRemap(bno.REMAP_CONFIG_P0);
  bno.setAxisSign(bno.REMAP_SIGN_P4);
//   bno.set_axis_remap(BNO055.AXIS_REMAP_Y,BNO055.AXIS_REMAP_Z,BNO055.AXIS_REMAP_X,BNO055.AXIS_REMAP_POSITIVE,BNO055.AXIS_REMAP_NEGATIVE,BNO055.AXIS_REMAP_NEGATIVE)

  delay(1000);
}

void loop(void)
{
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
//   sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
//   bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
//   bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
//   bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
//   bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
//   bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
//   bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

//   printEvent(&orientationData);
//   printEvent(&angVelocityData);
//   printEvent(&linearAccelData);
//   printEvent(&magnetometerData);
//   printEvent(&accelerometerData);
//   printEvent(&gravityData);

//   int8_t boardTemp = bno.getTemp();
//   Serial.println();
//   Serial.print(F("temperature: "));
//   Serial.println(boardTemp);
imu::Quaternion quat = bno.getQuat();
  
  Serial.print(F("Quaternion: "));
  Serial.print((float)quat.w(), 4);
  Serial.print(F(", "));
  Serial.print((float)quat.x(), 4);
  Serial.print(F(", "));
  Serial.print((float)quat.y(), 4);
  Serial.print(F(", "));
  Serial.print((float)quat.z(), 4);
  Serial.print(F("; "));

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
//   Serial.println();
  Serial.print("Calibration: ");
  Serial.print(system);
//   Serial.print(" Gyro=");
//   Serial.print(gyro);
//   Serial.print(" Accel=");
//   Serial.print(accel);
//   Serial.print(" Mag=");
//   Serial.println(mag);

  Serial.println("");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}

