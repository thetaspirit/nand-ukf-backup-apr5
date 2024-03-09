#include "ukf.h"
#include <math.h>
#include <Arduino.h>

UKF::UKF(params_t params, float zeroth_sigma_point_weight, state_cov_matrix_t process_noise, measurement_cov_matrix_t sensor_noise)
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

float magnitude(state_vector_t vector)
{
  float sum = 0;
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    sum += vector(i, 0);
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
    projection.Fill(0);
    for (int j = 0; j < i - 1; j++)
    {
      state_vector_t q = get_col(Q, j);
      BLA::Matrix<1, 1> s = ~q * x;
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

void find_eigen(state_cov_matrix_t matrix, state_cov_matrix_t &eigenvalues, state_cov_matrix_t &eigenvectors)
{
  eigenvalues.Fill(0);
  eigenvectors.Fill(0);

  Eigen::Matrix<float, STATE_SPACE_DIM, STATE_SPACE_DIM> A(STATE_SPACE_DIM, STATE_SPACE_DIM);
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    for (int j = 0; j < STATE_SPACE_DIM; j++)
    {
      A(i, j) = matrix(i, j);
    }
  }

  Eigen::EigenSolver<Eigen::Matrix<float, STATE_SPACE_DIM, STATE_SPACE_DIM>> solver(A);
  Eigen::Matrix3cf vectors = solver.eigenvectors();
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    for (int j = 0; j < STATE_SPACE_DIM; j++)
    {
      eigenvectors(i, j) = vectors(i, j).real();
    }
  }
  Eigen::Vector3cf values = solver.eigenvalues();
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    eigenvalues(i, i) = values(i).real();
  }

  /*
  for (int i = 0; i < 50; i++)
  // TODO I tried #define EIGEN_MAX_ITERS 50, but it wouldn't build.  no idea why.
  {
    state_cov_matrix_t Q = gram_schmidt(A);
    state_cov_matrix_t R = ~Q * A;

    A = R * Q;
    eigenvectors = eigenvectors * Q;
  }

  eigenvalues.Fill(0);
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    eigenvalues(i, i) = A(i, i);
  }
  */
}

/**
 * @brief Computes and returns the square root of the given state covariance matrix.
 */
state_cov_matrix_t square_root(state_cov_matrix_t matrix)
{
  state_cov_matrix_t D;
  state_cov_matrix_t eigenvectors;
  find_eigen(matrix, D, eigenvectors);

  state_cov_matrix_t Q = gram_schmidt(eigenvectors);

  // D is a diagonal matrix
  // elements along D's diagonal are eigenvalues
  // take the square root of D
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    D(i, i) = sqrt(D(i, i));
  }

  return (Q * D) * ~Q;
}

/**
 * @brief Generates sigma points given a mean and covariance.
 *
 * @param mean Vector in state space
 * @param covariance Covariance matrix (in state space)
 * @param sigmas Pointer to a list of 2*STATE_SPACE_DIM + 1 state space vectors for this function to write to
 * @param weights Pointer to a list of 2*STATE_SPACE_DIM + 1 floats for this function to write to.
 */
void UKF::generate_sigmas(state_vector_t mean, state_cov_matrix_t covariance, state_vector_t sigmas[2 * STATE_SPACE_DIM + 1], float weights[2 * STATE_SPACE_DIM + 1])
{
  state_cov_matrix_t A = square_root(covariance);

  weights[0] = this->zeroth_sigma_point_weight;
  sigmas[0] = mean;

  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    float s = sqrt(STATE_SPACE_DIM / (1 - weights[0]));
    state_vector_t A_col = get_col(A, i);
    A_col = A_col * s;

    sigmas[i + 1] = mean + A_col;
    sigmas[i + 1 + STATE_SPACE_DIM] = mean - A_col;
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
  x(2, 0) = this->params.velocity * cos(input(0, 0)) / this->params.wheelbase;
  return x;
}

state_vector_t UKF::rk4(state_vector_t state, input_vector_t input, float dt)
{
  state_vector_t k1 = this->dynamcis(state, input);
  state_vector_t k2 = this->dynamcis(state + (k1 * (dt / 2)), input);
  state_vector_t k3 = this->dynamcis(state + (k2 * (dt / 2)), input);
  state_vector_t k4 = this->dynamcis(state + (k3 * dt), input);

  return state + ((k1 + (k2 * (float)2) + (k3 * (float)2) + k4) * (dt / 6));
}

/**
 * @brief Tranforms the given state space vector into measurement space.
 */
measurement_vector_t UKF::state_to_measurement(state_vector_t vector)
{
  measurement_vector_t m;
  m(0, 0) = vector(0, 0);
  m(1, 0) = vector(1, 0);
  return m;
}

void UKF::predict(state_vector_t curr_state_est, state_cov_matrix_t curr_state_cov, input_vector_t input, float dt,
                  state_vector_t &predicted_state_est, state_cov_matrix_t &predicted_state_cov)
{
  state_vector_t state_sigmas[2 * STATE_SPACE_DIM + 1];
  float state_weights[2 * STATE_SPACE_DIM + 1];
  this->generate_sigmas(curr_state_est, curr_state_cov, state_sigmas, state_weights);

  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    Serial.printf("State sigma %d: %f, %f, %f  ", i, state_sigmas[i](0, 0), state_sigmas[i](1, 0), state_sigmas[i](2, 0));
    state_sigmas[i] = rk4(state_sigmas[i], input, dt);
    Serial.printf("After rk4: %f, %f, %f\n", i, state_sigmas[i](0, 0), state_sigmas[i](1, 0), state_sigmas[i](2, 0));
  }

  predicted_state_est.Fill(0);
  predicted_state_cov.Fill(0);
  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    predicted_state_est += state_sigmas[i] * state_weights[i];
  }

  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    state_vector_t m = state_sigmas[i] - predicted_state_est;
    predicted_state_cov += ((m * ~m) * state_weights[i]);
  }
  predicted_state_cov += this->process_noise;
}

void UKF::update(state_vector_t curr_state_est, state_cov_matrix_t curr_state_cov, measurement_vector_t measurement,
                 state_vector_t &updated_state_est, state_cov_matrix_t &updated_state_cov)
{
  state_vector_t state_sigmas[2 * STATE_SPACE_DIM + 1];
  float weights[2 * STATE_SPACE_DIM + 1];
  this->generate_sigmas(curr_state_est, curr_state_cov, state_sigmas, weights);

  measurement_vector_t measurement_sigmas[2 * STATE_SPACE_DIM + 1];
  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    measurement_sigmas[i] = state_to_measurement(state_sigmas[i]);
    Serial.printf("State sigma %d: %f, %f, %f\n", i, state_sigmas[i](0, 0), state_sigmas[i](1, 0), state_sigmas[i](2, 0));
    Serial.printf("Measurement sigma %d: %f, %f\n", i, measurement_sigmas[i](0, 0), measurement_sigmas[i](1, 0));
  }

  measurement_vector_t predicted_measurement;
  predicted_measurement.Fill(0);
  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    predicted_measurement += measurement_sigmas[i] * weights[i];
  }

  measurement_cov_matrix_t innovation_cov;
  BLA::Matrix<STATE_SPACE_DIM, MEASUREMENT_SPACE_DIM> cross_cov;
  innovation_cov.Fill(0);
  cross_cov.Fill(0);
  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    measurement_vector_t m = measurement_sigmas[i] - predicted_measurement;
    innovation_cov += (m * ~m) * weights[i];
    cross_cov += ((state_sigmas[i] - curr_state_est) * ~m) * weights[i];
  }
  innovation_cov += this->sensor_noise;

  BLA::Matrix<STATE_SPACE_DIM, MEASUREMENT_SPACE_DIM> kalman_gain = cross_cov * BLA::Inverse(innovation_cov);

  // Serial.printf("Measurement: %f, %f\n", measurement(0, 0), measurement(1, 0));
  // Serial.printf("Predicted measurement: %f, %f\n", predicted_measurement(0, 0), predicted_measurement(1, 0));
  // Serial.printf("Kalman gain:\n%f,%f\n%f,%f\n%f,%f\n", kalman_gain(0, 0), kalman_gain(0, 1), kalman_gain(1, 0), kalman_gain(1, 1), kalman_gain(2, 0), kalman_gain(2, 1));
  updated_state_est = curr_state_est + (kalman_gain * (measurement - predicted_measurement));
  updated_state_cov = curr_state_cov - (kalman_gain * innovation_cov * ~kalman_gain);
}