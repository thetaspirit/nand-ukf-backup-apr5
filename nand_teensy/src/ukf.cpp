#include "ukf.h"
#include <Arduino.h>

UKF::UKF(params_t params, float zeroth_sigma_point_weight, state_cov_matrix_t process_noise, measurement_cov_matrix_t sensor_noise)
{
  this->params = params;
  this->zeroth_sigma_point_weight = zeroth_sigma_point_weight;
  this->process_noise = process_noise;
  this->sensor_noise = sensor_noise;
}

/**
 * @brief Does Gram-Schmidt algorithm.
 * Computes and returns an orthogonal matrix whose columns form the orthonormal basis for the columnspace of the input matrix.
 */
state_cov_matrix_t UKF::gram_schmidt(state_cov_matrix_t matrix)
{
}

/**
 * @brief Computes and returns the square root of the given state covariance matrix.
 */
state_cov_matrix_t UKF::square_root(state_cov_matrix_t matrix)
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