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

double *gps_timestamp;
double *pos_x;
double *pos_y;
double *gps_accuracy;
int gps_len;

double *encoder_timestamp;
double *speed;
int encoder_len;

double *steering_timestamp;
double *steering;
int steering_len;

bool run = true;

CSV_Parser gps_cp(/*format*/ "ffff", /*has_header*/ true, /*delimiter*/ ',');
CSV_Parser encoder_cp(/*format*/ "ff", /*has_header*/ true, /*delimiter*/ ',');
CSV_Parser steering_cp(/*format*/ "ff", /*has_header*/ true, /*delimiter*/ ',');

void halt_and_catch_fire(const char *msg) {
  while (1) {
    Serial.println(msg);
    delay(1000);
  }
}

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

  // The line below (readSDfile) wouldn't work if SD.begin wasn't called before.
  // readSDfile can be used as conditional, it returns 'false' if the file does not exist.
  if (gps_cp.readSDfile("/log44-gps.csv"))
  {
    gps_timestamp = (double *)gps_cp["timestamp"];
    pos_x = (double *)gps_cp["pos_x"];
    pos_y = (double *)gps_cp["pos_y"];
    gps_accuracy = (double *)gps_cp["accuracy"];

    if (!gps_timestamp)
    {
      halt_and_catch_fire("ERROR: timestamp column not found.");
    }
    if (!pos_x)
    {
      halt_and_catch_fire("ERROR: pos_x column not found.");
    }
    if (!pos_y)
    {
      halt_and_catch_fire("ERROR: pos_y column not found.");
    }
    if (!gps_accuracy)
    {
      halt_and_catch_fire("ERROR: gps_accuracy column not found.");
    }
  }
  else
  {
    halt_and_catch_fire("ERROR: File called '/log44-gps.csv' does not exist...");
  }

  if (encoder_cp.readSDfile("/log44-encoder.csv"))
  {
    encoder_timestamp = (double *)encoder_cp["timestamp"];
    speed = (double *)encoder_cp["speed"];

    if (!encoder_timestamp)
    {
      halt_and_catch_fire("ERROR: timestamp column not found.");
    }
    if (!speed) {
      halt_and_catch_fire("ERROR: speed column not found.");
    }
  }
  else
  {
    halt_and_catch_fire("ERROR: File called '/log44-encoder.csv' does not exist...");
  }

  if (steering_cp.readSDfile("/log44-steering.csv"))
  {
    steering_timestamp = (double *)steering_cp["timestamp"];
    steering = (double *)steering_cp["steering"];

    if (!steering_timestamp)
    {
      halt_and_catch_fire("ERROR: steering timestamp column not found.");
    }
    if (!steering) {
      halt_and_catch_fire("ERROR: steering column not found.");
    }
  }
  else
  {
    halt_and_catch_fire("ERROR: File called '/log44-encoder.csv' does not exist...");
  }

  gps_len = gps_cp.getRowsCount();
  encoder_len = encoder_cp.getRowsCount();
  steering_len = steering_cp.getRowsCount();

  // convert timestamps from ms to s
  for (int i = 0; i < gps_len; i++) {
    gps_timestamp[i] = gps_timestamp[i] / 1000.0;
  }
  for (int i = 0; i < encoder_len; i++) {
    encoder_timestamp[i] = encoder_timestamp[i] / 1000.0;
  }
  for (int i = 0; i < steering_len; i++) {
    steering_timestamp[i] = steering_timestamp[i] / 1000.0;
  }
}

void loop()
{
  if (!run)
  {
    return;
  }

  /**************** Kalman Filtering! ****************/
  // output file setup
  File filter_out = SD.open("filter.csv", FILE_WRITE);
  filter_out.println("timestamp,pos_x,pos_y,heading");
  File cov_out = SD.open("covariance.csv", FILE_WRITE);
  cov_out.println("timestamp,c1,c2,c3,c4,c5,c6,c7,c8,c9");

  // ukf value setup

  state_vector_t curr_state_est{{0, 0, 0}}; 
  // TODO in real buggy code, set initial x and y first gps reading
  // set heading to where it's facing in tent upon power on, probably
  state_cov_matrix_t curr_state_cov{{1, 0, 0},
                                    {0, 1, 0},
                                    {0, 0, 1}};
  state_cov_matrix_t process_nosie{{0.0001, 0, 0},
                                   {0, 0.0001, 0},
                                   {0, 0, 0.000001}};

  measurement_cov_matrix_t gps_noise{{0.01, 0},
                                        {0, 0.01}};

  UKF Filter = UKF(1, (1 / 3), process_nosie, gps_noise);

  state_vector_t predicted_state_est;
  state_cov_matrix_t predicted_state_cov;
  state_vector_t updated_state_est;
  state_cov_matrix_t updated_state_cov;

  int gps_row = 0;
  int encoder_row = 0;
  int steering_row = 0;

  input_vector_t current_steering = (input_vector_t){steering[0]};
  double last_predict_timestamp = steering_timestamp[0];
  // running filter!
  while (gps_row < gps_len && encoder_row < encoder_len && steering_row < steering_len)
  {
    if (gps_timestamp[gps_row] < encoder_timestamp[encoder_row] && gps_timestamp[gps_row] < steering_timestamp[steering_row]) {
      // gps is the next timestamp
      // set the new gps noise
      // predict using most recent steering and velocity
      // then update using the current gps noise
      double dt = gps_timestamp[gps_row] - last_predict_timestamp;

      Filter.set_gps_noise(gps_accuracy[gps_row]);

      Filter.predict(curr_state_est, curr_state_cov, current_steering,
      dt, predicted_state_est, predicted_state_cov);
      Filter.update(predicted_state_est, predicted_state_cov,
      (measurement_vector_t){pos_x[gps_row], pos_y[gps_row]}, updated_state_est, updated_state_cov);

      last_predict_timestamp = gps_timestamp[gps_row];
      gps_row++;
    }

    else if (encoder_timestamp[encoder_row] < steering_timestamp[steering_row]) {
      // encoder is the next timestamp
      // set the new speed stored by instance of UKF
      // predict using most recent steering and new speed
      double dt = encoder_timestamp[encoder_row] - last_predict_timestamp;

      Filter.set_speed(speed[encoder_row]);
      Filter.predict(curr_state_est, curr_state_cov, current_steering,
      dt, predicted_state_est, predicted_state_cov);

      last_predict_timestamp = encoder_timestamp[encoder_row];
      encoder_row++;
    }

    else {
      // steering is the next timestamp
      // predict using this new steering
      double dt = steering_timestamp[steering_row] - last_predict_timestamp;

      Filter.predict(curr_state_est, curr_state_cov, current_steering,
      dt, predicted_state_est, predicted_state_cov);

      last_predict_timestamp = steering_timestamp[steering_row];
      current_steering(0, 0) = steering[steering_row];
      steering_row++;
    }



    filter_out.printf("%f,%f,%f,%f\n", last_predict_timestamp, updated_state_est(0, 0), updated_state_est(1, 0), updated_state_est(2, 0));
    cov_out.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", last_predict_timestamp, updated_state_cov(0, 0), updated_state_cov(0, 1), updated_state_cov(0, 2),
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