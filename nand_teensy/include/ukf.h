#pragma once

#include <ArduinoEigenDense.h>

#define STATE_SPACE_DIM 3
typedef Eigen::Matrix<double, STATE_SPACE_DIM, 1> state_vector_t;
typedef Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> state_cov_matrix_t;

#define MEASUREMENT_SPACE_DIM 2
typedef Eigen::Matrix<double, MEASUREMENT_SPACE_DIM, 1> measurement_vector_t;
typedef Eigen::Matrix<double, MEASUREMENT_SPACE_DIM, MEASUREMENT_SPACE_DIM> measurement_cov_matrix_t;

#define INPUT_SPACE_DIM 1
typedef Eigen::Matrix<double, INPUT_SPACE_DIM, 1> input_vector_t;

#define EIGEN_MAX_ITERS 50;

struct params_t
{
  double wheelbase;
  double velocity;
};

class UKF
{
private:
  params_t params;
  double zeroth_sigma_point_weight;

  state_cov_matrix_t process_noise;
  measurement_cov_matrix_t sensor_noise;

  state_vector_t dynamcis(state_vector_t state, input_vector_t input);
  state_vector_t rk4(state_vector_t state, input_vector_t input, double dt);

  void generate_sigmas(state_vector_t mean, state_cov_matrix_t covariance, state_vector_t sigmas[2 * STATE_SPACE_DIM + 1], double weights[2 * STATE_SPACE_DIM + 1]);

  measurement_vector_t state_to_measurement(state_vector_t vector);

public:
  UKF(params_t params, double zeroth_sigma_point_weight, state_cov_matrix_t process_noise, measurement_cov_matrix_t sensor_noise);

  void predict(state_vector_t curr_state_est, state_cov_matrix_t curr_state_cov, input_vector_t input, double dt,
               state_vector_t &predicted_state_est, state_cov_matrix_t &predicted_state_cov);

  void update(state_vector_t curr_state_est, state_cov_matrix_t curr_state_cov, measurement_vector_t measurement,
              state_vector_t &updated_state_est, state_cov_matrix_t &updated_state_cov);
};

state_vector_t get_col(state_cov_matrix_t A, int i);
state_cov_matrix_t square_root(state_cov_matrix_t matrix);