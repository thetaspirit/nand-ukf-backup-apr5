#include "ukf.h"
#include <math.h>
#include <Arduino.h>

UKF::UKF(params_t params, double zeroth_sigma_point_weight, state_cov_matrix_t process_noise, measurement_cov_matrix_t sensor_noise)
{
  this->params = params;
  this->zeroth_sigma_point_weight = zeroth_sigma_point_weight;
  this->process_noise = process_noise;
  this->sensor_noise = sensor_noise;
}

state_vector_t get_col(state_cov_matrix_t matrix, int col)
{
  state_vector_t c;
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    c(i, 0) = matrix(i, col);
  }
  return c;
}

double magnitude(state_vector_t vector)
{
  double sum = 0;
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    sum += vector(i, 0) * vector(i, 0);
  }
  return sqrt(sum);
}

/**
 * @brief Does Gram-Schmidt algorithm.
 * Computes and returns an orthogonal matrix whose columns form the orthonormal basis for the columnspace of the input matrix.
 */
state_cov_matrix_t gram_schmidt(state_cov_matrix_t matrix)
{
  state_cov_matrix_t Q;

  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    state_vector_t x = get_col(matrix, i);
    state_vector_t projection; // projection of x onto the span of the previous i-1 columns
    projection.fill(0);
    for (int j = 0; j < i - 1; j++)
    {
      state_vector_t q = get_col(Q, j);
      double s = q.transpose() * x;
      projection += q * s;
    }

    x -= projection;

    state_vector_t q = x / magnitude(x);

    // writing q into the respective column of Q
    for (int j = 0; j < STATE_SPACE_DIM; j++)
    {
      Q(j, i) = q(j, 0);
    }
  }

  return Q;
}

/**
 * @brief Computes and returns the square root of the given state covariance matrix.
 */
state_cov_matrix_t square_root(state_cov_matrix_t matrix)
{
  Eigen::EigenSolver<state_cov_matrix_t> solver(matrix);

  state_cov_matrix_t D;
  D.fill(0);
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    D(i, i) = solver.eigenvalues()[i].real();
  }

  state_cov_matrix_t eigenvectors;
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    for (int j = 0; j < STATE_SPACE_DIM; j++)
    {
      eigenvectors(i, j) = solver.eigenvectors()(i, j).real();
    }
  }
  state_cov_matrix_t Q = gram_schmidt(eigenvectors);

  // D is a diagonal matrix
  // elements along D's diagonal are eigenvalues
  // take the square root of D
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    D(i, i) = sqrt(D(i, i));
  }

  return (Q * D) * Q.transpose();
}

/**
 * @brief Generates sigma points given a mean and covariance.
 *
 * @param mean Vector in state space
 * @param covariance Covariance matrix (in state space)
 * @param sigmas Pointer to a list of 2*STATE_SPACE_DIM + 1 state space vectors for this function to write to
 * @param weights Pointer to a list of 2*STATE_SPACE_DIM + 1 double for this function to write to.
 */
void UKF::generate_sigmas(state_vector_t mean, state_cov_matrix_t covariance, state_vector_t sigmas[2 * STATE_SPACE_DIM + 1], double weights[2 * STATE_SPACE_DIM + 1])
{
  state_cov_matrix_t A = square_root(covariance);

  weights[0] = this->zeroth_sigma_point_weight;
  sigmas[0] = mean;

  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    double s = sqrt(STATE_SPACE_DIM / (1 - weights[0]));
    state_vector_t A_col = get_col(A, i);
    A_col = A_col * s;

    sigmas[i + 1] = mean + A_col;
    sigmas[i + 1 + STATE_SPACE_DIM] = mean - A_col;
  }

  for (int i = 1; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    weights[i] = (1 - weights[0]) / (2 * STATE_SPACE_DIM);
  }
}

/**
 * @brief Calculates and returns the dynamics of the system
 */
state_vector_t UKF::dynamcis(state_vector_t state, input_vector_t input)
{
  state_vector_t x;
  x(0, 0) = this->params.velocity * cos(state(2, 0));
  x(1, 0) = this->params.velocity * sin(state(2, 0));
  x(2, 0) = this->params.velocity * tan(input(0, 0)) / this->params.wheelbase;
  return x;
}

state_vector_t UKF::rk4(state_vector_t state, input_vector_t input, double dt)
{
  state_vector_t k1 = this->dynamcis(state, input);
  state_vector_t k2 = this->dynamcis(state + (k1 * (dt / 2)), input);
  state_vector_t k3 = this->dynamcis(state + (k2 * (dt / 2)), input);
  state_vector_t k4 = this->dynamcis(state + (k3 * dt), input);

  return state + ((k1 + (k2 * (double)2) + (k3 * (double)2) + k4) * (dt / 6));
}

/**
 * @brief Tranforms the given state space vector into gps measurement space.
 */
measurement_vector_t* UKF::state_to_gps_measurement(state_vector_t state, input_vector_t input)
{
  measurement_vector_t *m = new Eigen::Matrix<float, GPS_SPACE_DIM, 1>;

  (*m)(0, 0) = state(0, 0) + this->params.la * cos(state(2, 0));
  (*m)(1, 0) = state(1, 0) + this->params.la * sin(state(2, 0));
  (*m)(2, 0) = state(3, 0) * cos(input(0, 0)) - (state(4, 0) - this->params.lf * state(5, 0)) * sin(input(0, 0));

  return m;
}

/**
 * @brief Tranforms the given state space vector into imu measurement space.
 */
measurement_vector_t *UKF::state_to_imu_measurement(state_vector_t state, input_vector_t input)
{
  measurement_vector_t *m = new Eigen::Matrix<float, IMU_SPACE_DIM, 1>;
  (*m)(0, 0) = state(5, 0);
  (*m)(1, 0) = state(3, 0) * cos(input(0, 0)) - (state(4, 0) - this->params.lf * state(5, 0)) * sin(input(0, 0));

  return m;
}

void UKF::predict(state_vector_t curr_state_est, state_cov_matrix_t curr_state_cov, input_vector_t input, double dt,
                  state_vector_t &predicted_state_est, state_cov_matrix_t &predicted_state_cov)
{
  state_vector_t state_sigmas[2 * STATE_SPACE_DIM + 1];
  double state_weights[2 * STATE_SPACE_DIM + 1];
  this->generate_sigmas(curr_state_est, curr_state_cov, state_sigmas, state_weights);

  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    state_sigmas[i] = rk4(state_sigmas[i], input, dt);
    // Serial.printf("State sigma %d: %f, %f, %f\n", i, state_sigmas[i](0, 0), state_sigmas[i](1, 0), state_sigmas[i](2, 0));
  }

  predicted_state_est.fill(0);
  predicted_state_cov.fill(0);
  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    predicted_state_est += state_sigmas[i] * state_weights[i];
  }

  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    state_vector_t m = state_sigmas[i] - predicted_state_est;
    predicted_state_cov += ((m * m.transpose()) * state_weights[i]);
  }
  predicted_state_cov += this->process_noise;
}

void UKF::update(state_vector_t curr_state_est, state_cov_matrix_t curr_state_cov, measurement_vector_t measurement, input_vector_t input,
                 state_vector_t &updated_state_est, state_cov_matrix_t &updated_state_cov, enum measurement_type m_type)
{
  int measurement_dimension = (m_type == GPS) ? GPS_SPACE_DIM : IMU_SPACE_DIM;

  state_vector_t state_sigmas[2 * STATE_SPACE_DIM + 1];
  double weights[2 * STATE_SPACE_DIM + 1];
  this->generate_sigmas(curr_state_est, curr_state_cov, state_sigmas, weights);

  measurement_vector_t *measurement_sigmas[2 * STATE_SPACE_DIM + 1];
  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    if (m_type == GPS) {
      measurement_sigmas[i] = state_to_gps_measurement(state_sigmas[i], input);
    }
    else {
      measurement_sigmas[i] = state_to_imu_measurement(state_sigmas[i], input);
    }
    // Serial.printf("State sigma %d: %f, %f, %f\n", i, state_sigmas[i](0, 0), state_sigmas[i](1, 0), state_sigmas[i](2, 0));
    // Serial.printf("Measurement sigma %d: %f, %f\n", i, measurement_sigmas[i](0, 0), measurement_sigmas[i](1, 0));
  }

  measurement_vector_t predicted_measurement;
  predicted_measurement.fill(0);
  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    predicted_measurement += *(measurement_sigmas[i]) * weights[i];
  }

  measurement_cov_matrix_t innovation_cov;
  Eigen::Matrix<double, STATE_SPACE_DIM, Eigen::Dynamic> cross_cov(STATE_SPACE_DIM, measurement_dimension);
  innovation_cov.fill(0);
  cross_cov.fill(0);
  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    measurement_vector_t m = *(measurement_sigmas[i]) - predicted_measurement;
    innovation_cov += (m * m.transpose()) * weights[i];
    cross_cov += ((state_sigmas[i] - curr_state_est) * m.transpose()) * weights[i];
  }
  innovation_cov += this->sensor_noise;

  Eigen::Matrix<double, STATE_SPACE_DIM, Eigen::Dynamic> kalman_gain = cross_cov * innovation_cov.inverse();

  Serial.printf("Measurement: %f, %f\n", measurement(0, 0), measurement(1, 0));
  Serial.printf("Predicted measurement: %f, %f\n", predicted_measurement(0, 0), predicted_measurement(1, 0));
  Serial.printf("Kalman gain:\n%f,%f\n%f,%f\n%f,%f\n", kalman_gain(0, 0), kalman_gain(0, 1), kalman_gain(1, 0), kalman_gain(1, 1), kalman_gain(2, 0), kalman_gain(2, 1));
  updated_state_est = curr_state_est + (kalman_gain * (measurement - predicted_measurement));
  updated_state_cov = curr_state_cov - (kalman_gain * (innovation_cov * kalman_gain.transpose()));
}