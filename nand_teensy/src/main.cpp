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

/*
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
*/

bool run = true;

#define LINE_BUF_SIZE 100

enum Measurement {
  Gps,
  Encoder,
  Steering,
};

class DataStream {
public:
  DataStream() :
    next_gps_time(0xFFFFFFFF),
    next_encoder_time(0xFFFFFFFF),
    next_steering_time(0xFFFFFFFF),

    gps_file(SD.open("nand-lgap-gps.csv")),
    encoder_file(SD.open("nand-lgap-encoder.csv")),
    steering_file(SD.open("nand-lgap-steering.csv"))
  {
    if (!gps_file || !encoder_file || !steering_file) {
      while (1) {
        Serial.println("Failed to open a file");
        delay(1000);
      }
    }

    char buf[LINE_BUF_SIZE];

    memset(buf, 0, sizeof(buf));
    gps_file.readBytesUntil('\n', buf, LINE_BUF_SIZE - 1);
    Serial.printf("gps: %s\n", buf);

    memset(buf, 0, sizeof(buf));
    encoder_file.readBytesUntil('\n', buf, LINE_BUF_SIZE - 1);
    Serial.printf("encoder: %s\n", buf);

    memset(buf, 0, sizeof(buf));
    steering_file.readBytesUntil('\n', buf, LINE_BUF_SIZE - 1);
    Serial.printf("steering: %s\n", buf);

    read_gps();
    read_encoder();
    read_steering();
  }

  double next_gps_x, next_gps_y, next_gps_acc;

  double next_encoder_speed;

  double next_steering_angle;

  Measurement next_measurement() {
    if (next_gps_time <= next_encoder_time && next_gps_time <= next_steering_time) {
      return Measurement::Gps;
    } else if (next_encoder_time <= next_gps_time && next_encoder_time <= next_steering_time) {
      return Measurement::Encoder;
    } else {
      return Measurement::Steering;
    }
  }

  void advance(Measurement m) {
    switch (m) {
    case Measurement::Gps:      read_gps();      break;
    case Measurement::Encoder:  read_encoder();  break;
    case Measurement::Steering: read_steering(); break;
    default: break;
    }
  }
  
  bool finished() {
    // TODO:
    return (next_gps_time == 0xFFFFFFFF && next_encoder_time == 0xFFFFFFFF && next_steering_time == 0xFFFFFFFF);
  }

  double gps_time() {
    return next_gps_time / 1000.0;
  }

  double encoder_time() {
    return next_encoder_time / 1000.0;
  }

  double steering_time() {
    return next_steering_time / 1000.0;
  }

private:

  uint32_t next_gps_time, next_encoder_time, next_steering_time;

  void read_gps() {
    char buf[LINE_BUF_SIZE] = { 0 };
    size_t num_read = gps_file.readBytesUntil('\n', buf, LINE_BUF_SIZE - 1);
    buf[num_read] = '\0';

    if (gps_file.getReadError() != 0) {
      next_gps_time = 0xFFFFFFFF;
      return;
    }

    assert(gps_file.getReadError() == 0);

    assert(sscanf(buf, "%lu,%lf,%lf,%lf", &next_gps_time, &next_gps_x, &next_gps_y, &next_gps_acc) == 4);
  }

  void read_encoder() {
    char buf[LINE_BUF_SIZE] = { 0 };
    size_t num_read = encoder_file.readBytesUntil('\n', buf, LINE_BUF_SIZE - 1);
    buf[num_read] = '\0';

    if (encoder_file.getReadError() != 0) {
      next_encoder_time = 0xFFFFFFFF;
      return;
    }

    assert(sscanf(buf, "%lu,%lf", &next_encoder_time, &next_encoder_speed) == 2);
  }

  void read_steering() {
    char buf[LINE_BUF_SIZE] = { 0 };
    size_t num_read = steering_file.readBytesUntil('\n', buf, LINE_BUF_SIZE - 1);
    buf[num_read] = '\0';

    if (steering_file.getReadError() != 0) {
      next_steering_time = 0xFFFFFFFF;
      return;
    }

    assert(sscanf(buf, "%lu,%lf", &next_steering_time, &next_steering_angle) == 2);

    // convert degrees to radians
    next_steering_angle = next_steering_angle * PI / 180.;
  }

  File gps_file;
  File encoder_file;
  File steering_file;
};

void halt_and_catch_fire(const char *msg) {
  while (1) {
    Serial.println(msg);
    delay(1000);
  }
}

void run_kalman(DataStream &s);

void setup()
{
  Serial.begin(9600);
  Serial.println("Kalman Filter Testing");

  delay(1000);

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

  DataStream s {};

  run_kalman(s);

  /*gps_len = gps_cp.getRowsCount();
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
  }*/
}

void run_kalman(DataStream &stream)
{
  if (!run)
  {
    return;
  }

  SD.remove("filter.csv");
  SD.remove("covariance.csv");

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
  curr_state_est(0, 0) = stream.next_gps_x;
  curr_state_est(1, 0) = stream.next_gps_y;
  // curr_state_est(2, 0) = /* ???? */

  state_cov_matrix_t curr_state_cov{{1, 0, 0},
                                    {0, 1, 0},
                                    {0, 0, 1}};
  state_cov_matrix_t process_noise{{0.0001, 0, 0},
                                   {0, 0.0001, 0},
                                   {0, 0, 0.000001}};

  measurement_cov_matrix_t gps_noise{{0.01, 0},
                                        {0, 0.01}};

  UKF Filter = UKF(1, (1 / 3), process_noise, gps_noise);

  state_vector_t predicted_state_est;
  state_cov_matrix_t predicted_state_cov;
  state_vector_t updated_state_est;
  state_cov_matrix_t updated_state_cov;

  input_vector_t current_steering { stream.next_steering_angle };
  double last_predict_timestamp = stream.steering_time();
  // running filter!
  while (!stream.finished())
  {
    static int point = 0;

    uint32_t mic0 = micros();

    Measurement m = stream.next_measurement();

    Serial.printf("State cov before: %f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", last_predict_timestamp, curr_state_cov(0, 0), curr_state_cov(0, 1), curr_state_cov(0, 2),
                   curr_state_cov(1, 0), curr_state_cov(1, 1), curr_state_cov(1, 2),
                   curr_state_cov(2, 0), curr_state_cov(2, 1), curr_state_cov(2, 2));
    if (m == Measurement::Gps) {
      // gps is the next timestamp
      // set the new gps noise
      // predict using most recent steering and velocity
      // then update using the current gps noise
      double dt = stream.gps_time() - last_predict_timestamp;

      Filter.set_gps_noise(stream.next_gps_acc);

      Filter.predict(
        curr_state_est, curr_state_cov, current_steering, dt,
        predicted_state_est, predicted_state_cov
      );

      Filter.update(
        predicted_state_est, predicted_state_cov,
        measurement_vector_t { stream.next_gps_x, stream.next_gps_y },
        updated_state_est, updated_state_cov
      );

      curr_state_est = updated_state_est;
      curr_state_cov = updated_state_cov;

      last_predict_timestamp = stream.gps_time();
    }
    else if (m == Measurement::Encoder) {
      // encoder is the next timestamp
      // set the new speed stored by instance of UKF
      // predict using most recent steering and new speed
      double dt = stream.encoder_time() - last_predict_timestamp;

      Filter.set_speed(stream.next_encoder_speed);

      if (stream.next_encoder_speed != 0) {
        Filter.predict(
          curr_state_est, curr_state_cov, current_steering, dt,
          predicted_state_est, predicted_state_cov
        );
      }

      last_predict_timestamp = stream.encoder_time();

      curr_state_est = predicted_state_est;
      curr_state_cov = predicted_state_cov;
    }
    else {
      // steering is the next timestamp
      // predict using this new steering
      double dt = stream.steering_time() - last_predict_timestamp;

      Filter.predict(curr_state_est, curr_state_cov, current_steering,
      dt, predicted_state_est, predicted_state_cov);

      last_predict_timestamp = stream.steering_time();
      current_steering(0, 0) = stream.next_steering_angle;

      curr_state_est = predicted_state_est;
      curr_state_cov = predicted_state_cov;
    }
    Serial.printf("State cov after: %f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", last_predict_timestamp, curr_state_cov(0, 0), curr_state_cov(0, 1), curr_state_cov(0, 2),
                   curr_state_cov(1, 0), curr_state_cov(1, 1), curr_state_cov(1, 2),
                   curr_state_cov(2, 0), curr_state_cov(2, 1), curr_state_cov(2, 2));

    uint32_t diff0 = micros() - mic0;
    //Serial.printf("filter took %lu micros\n", diff0);

    uint32_t mic = micros();
    stream.advance(m);
    uint32_t diff = micros() - mic;
    //Serial.printf("advance took %lu micros\n", diff);

    uint32_t mic2 = micros();
    filter_out.printf("%f,%f,%f,%f\n", last_predict_timestamp, updated_state_est(0, 0), updated_state_est(1, 0), updated_state_est(2, 0));
    cov_out.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", last_predict_timestamp, updated_state_cov(0, 0), updated_state_cov(0, 1), updated_state_cov(0, 2),
                   updated_state_cov(1, 0), updated_state_cov(1, 1), updated_state_cov(1, 2),
                   updated_state_cov(2, 0), updated_state_cov(2, 1), updated_state_cov(2, 2));


    ++point;

    static int i = 0;
    if (++i >= 1000) {
      i = 0;
      filter_out.flush();
      cov_out.flush();

      Serial.println("===========================\n");
      Serial.printf("Data point %d, time %f\n", point, last_predict_timestamp);
      Serial.println("===========================\n");
    }
    uint32_t diff2 = micros() - mic2;
    //Serial.printf("printf took %lu micros\n", diff2);

  }

  filter_out.close();
  cov_out.close();
  run = false;
  Serial.println("FILTER DONE");
}

void loop() {}