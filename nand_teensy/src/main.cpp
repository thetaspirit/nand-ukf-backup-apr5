/*
  Read NMEA sentences over I2C using Ublox module SAM-M8Q, NEO-M8P, etc
  By: Nathan Seidle
  SparkFun Electronics
  Date: August 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example reads the NMEA characters over I2C and pipes them to MicroNMEA
  This example will output your current long/lat and satellites in view

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  For more MicroNMEA info see https://github.com/stevemarple/MicroNMEA

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
  Go outside! Wait ~25 seconds and you should see your lat/long
*/

#define RFM69_CS 2
#define RFM69_INT 3
#define RFM69_RST 4

#include "ukf.h"

#include <Arduino.h>
#include <Wire.h> //Needed for I2C to GPS
#include <SD.h>
#include <CSV_Parser.h>

#include <Adafruit_BNO08x.h>

#define BNO_085_INT 20

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// #include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
// SFE_UBLOX_GPS myGPS;

#include <cmath>
#include <stdio.h>
#include <stdlib.h>

char *buf;

double *timestamp;
double *pos_x;
double *pos_y;
double *input;
int n;
bool run = true;

void setup()
{
  Serial.begin(9600);
  Serial.println("Kalman Filter Testing");

  /*
  uint32_t sp;
  asm("mov %0, sp" : "=r"(sp));
  Serial.printf("Initial stack pointer is %x\n", sp);
  */

  if (CrashReport)
  {
    Serial.print(CrashReport);
  }

  if (!SD.begin(BUILTIN_SDCARD))
  {
    while (1)
      Serial.println("SD card not detected. Freezing");
  }

  CSV_Parser measure_cp(/*format*/ "fff", /*has_header*/ true, /*delimiter*/ ',');
  // The line below (readSDfile) wouldn't work if SD.begin wasn't called before.
  // readSDfile can be used as conditional, it returns 'false' if the file does not exist.
  if (measure_cp.readSDfile("/measure.csv"))
  {
    timestamp = (double *)measure_cp["timestamp"];
    pos_x = (double *)measure_cp["pos_x"];
    pos_y = (double *)measure_cp["pos_y"];

    if (!timestamp)
    {
      Serial.println("ERROR: timestamp column not found.");
      Serial.println(timestamp[0]);
    }
    if (!pos_x)
    {
      Serial.println("ERROR: pos_x column not found.");
      Serial.println(pos_x[0]);
    }
    if (!pos_y)
    {
      Serial.println("ERROR: pos_y column not found.");
      Serial.println(pos_y[0]);
    }
  }
  else
  {
    Serial.println("ERROR: File called '/measure.csv' does not exist...");
  }

  CSV_Parser input_cp(/*format*/ "ff", /*has_header*/ true, /*delimiter*/ ',');
  if (input_cp.readSDfile("/input.csv"))
  {
    input = (double *)input_cp["input"];

    if (!input)
    {
      Serial.println("ERROR: input column not found.");
      Serial.println(input[0]);
    }
  }
  else
  {
    Serial.println("ERROR: File called '/input.csv' does not exist...");
  }
  n = input_cp.getRowsCount();
}

void loop()
{
  if (!run)
  {
    return;
  }

  /**************** Kalman Filtering! ****************/
  // file setup
  File filter_out = SD.open("filter.csv", FILE_WRITE);
  filter_out.println("timestamp,pos_x,pos_y,heading");
  File cov_out = SD.open("covariance.csv", FILE_WRITE);
  cov_out.println("timestamp,c1,c2,c3,c4,c5,c6,c7,c8,c9");

  // value setup
  double dt = 0.01;
  state_vector_t curr_state_est{{0, 0, 0}};
  state_cov_matrix_t curr_state_cov{{1, 0, 0},
                                    {0, 1, 0},
                                    {0, 0, 1}};
  state_cov_matrix_t process_nosie{{0.0001, 0, 0},
                                   {0, 0.0001, 0},
                                   {0, 0, 0.000001}};
  measurement_cov_matrix_t sensor_noise{{0.5, 0},
                                        {0, 0.5}};

  UKF Filter = UKF(1, (1 / 3), process_nosie, sensor_noise);

  // running filter!
  for (int i = 0; i < n; i++)
  {
    Serial.printf("\nFILTERING %d\n", i);
    Serial.println("PREDICT");
    state_vector_t predicted_state_est;
    state_cov_matrix_t predicted_state_cov;
    Filter.predict(curr_state_est, curr_state_cov, (input_vector_t){input[i]}, dt, predicted_state_est, predicted_state_cov);

    Serial.println("UPDATE");
    state_vector_t updated_state_est;
    state_cov_matrix_t updated_state_cov;
    Filter.update(predicted_state_est, predicted_state_cov, (measurement_vector_t){pos_x[i], pos_y[i]}, updated_state_est, updated_state_cov);
    Serial.printf("Filtered state: %f,%f,%f\n", updated_state_est(0, 0), updated_state_est(1, 0), updated_state_est(2, 0));

    filter_out.printf("%f,%f,%f,%f\n", timestamp[i], updated_state_est(0, 0), updated_state_est(1, 0), updated_state_est(2, 0));
    cov_out.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", timestamp[i], updated_state_cov(0, 0), updated_state_cov(0, 1), updated_state_cov(0, 2),
                   updated_state_cov(1, 0), updated_state_cov(1, 1), updated_state_cov(1, 2),
                   updated_state_cov(2, 0), updated_state_cov(2, 1), updated_state_cov(2, 2));

    curr_state_est = updated_state_est;
    curr_state_cov = updated_state_cov;
  }

  filter_out.close();
  cov_out.close();
  run = false;
  Serial.println("FILTER DONE");
}