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

/**
 * @brief Computes and returns the square root of the given state covariance matrix.
 */
state_cov_matrix_t square_root(state_cov_matrix_t matrix)
{
}

/**
 * @brief Generates sigma points given a mean and covariance.
 *
 * @param mean Vector in state space
 * @param covariance Covariance matrix (in state space)
 * @param sigmas Pointer to a list of 2*STATE_SPACE_DIM + 1 state space vectors for this function to write to
 * @param weights Pointer to a list of 2*STATE_SPACE_DIM + 1 floats for this function to write to.
 */
// void UKF::generate_sigmas(state_vector_t mean, state_cov_matrix_t covariance, state_vector_t sigmas[2 * STATE_SPACE_DIM + 1], float weights[2 * STATE_SPACE_DIM + 1])
// {
//   state_cov_matrix_t A; // = square_root(covariance);

//   weights[0] = this->zeroth_sigma_point_weight;
//   sigmas[0] = mean;

//   for (int i = 1; i < STATE_SPACE_DIM; i++) {
//     sigmas[i] =
//   }
// }