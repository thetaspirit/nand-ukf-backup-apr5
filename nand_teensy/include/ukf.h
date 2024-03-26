#pragma once

#include <ArduinoEigenDense.h>

#define STATE_SPACE_DIM 6
typedef Eigen::Matrix<double, STATE_SPACE_DIM, 1> state_vector_t;
typedef Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> state_cov_matrix_t;

#define GPS_SPACE_DIM 3
typedef Eigen::Matrix<double, GPS_SPACE_DIM, 1> gps_vector_t;
typedef Eigen::Matrix<double, GPS_SPACE_DIM, GPS_SPACE_DIM> gps_cov_matrix_t;
#define IMU_SPACE_DIM 2
typedef Eigen::Matrix<double, IMU_SPACE_DIM, 1> imu_vector_t;
typedef Eigen::Matrix<double, IMU_SPACE_DIM, GPS_SPACE_DIM> imu_cov_matrix_t;

#define INPUT_SPACE_DIM 1
typedef Eigen::Matrix<double, INPUT_SPACE_DIM, 1> input_vector_t;

enum measurement_type {GPS, IMU};

struct params_t
{
  double lf; // length to the front wheel (from center of mass)
  double lr; // length to rear axle (from center of mass)
  double la; // length from antenna (from center of mass)
  double wheelbase;
  double velocity;
};

class UKF
{
private:
  params_t params;
  double zeroth_sigma_point_weight;

  state_cov_matrix_t process_noise;
  gps_cov_matrix_t gps_cov;
  imu_cov_matrix_t imu_cov;

  state_vector_t dynamcis(state_vector_t state, input_vector_t input);
  state_vector_t rk4(state_vector_t state, input_vector_t input, double dt);

  void generate_sigmas(state_vector_t mean, state_cov_matrix_t covariance, state_vector_t sigmas[2 * STATE_SPACE_DIM + 1], double weights[2 * STATE_SPACE_DIM + 1]);

  gps_vector_t state_to_gps_measurement(state_vector_t state, input_vector_t input);
  imu_vector_t state_to_imu_measurement(state_vector_t state, input_vector_t input);

public:
  UKF(params_t params, double zeroth_sigma_point_weight, state_cov_matrix_t process_noise, measurement_cov_matrix_t sensor_noise);

  void predict(state_vector_t curr_state_est, state_cov_matrix_t curr_state_cov, input_vector_t input, double dt,
               state_vector_t &predicted_state_est, state_cov_matrix_t &predicted_state_cov);

  void update(state_vector_t curr_state_est, state_cov_matrix_t curr_state_cov, measurement_vector_t measurement, input_vector_t input,
              state_vector_t &updated_state_est, state_cov_matrix_t &updated_state_cov, enum measurement_type m_type);
};

state_vector_t get_col(state_cov_matrix_t A, int i);
state_cov_matrix_t square_root(state_cov_matrix_t matrix);