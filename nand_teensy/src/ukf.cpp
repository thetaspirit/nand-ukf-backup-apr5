#include "ukf.h"
#include <Arduino.h>

UKF::UKF(params_t params, state_cov_matrix_t process_noise, measurement_cov_matrix_t sensor_noise)
{
  this->params = params;
  this->process_noise = process_noise;
  this->sensor_noise = sensor_noise;
}