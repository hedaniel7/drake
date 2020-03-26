#pragma once

#include "drake/common/polynomial.h"

namespace drake {
namespace test {

/// Obtains a matrix of random unvariate Polynomials of the specified size.
template <typename T = double>
static Eigen::Matrix<Polynomial<T>, Eigen::Dynamic, Eigen::Dynamic>
RandomPolynomialMatrix(Eigen::Index num_coefficients_per_polynomial,
                       Eigen::Index rows, Eigen::Index cols) {
  Eigen::Matrix<Polynomial<T>, Eigen::Dynamic, Eigen::Dynamic>
      mat(rows, cols);
  for (Eigen::Index row = 0; row < mat.rows(); ++row) {
    for (Eigen::Index col = 0; col < mat.cols(); ++col) {
      auto coeffs =
          (Eigen::Matrix<T, Eigen::Dynamic, 1>::Random(
              num_coefficients_per_polynomial)).eval();
      mat(row, col) = Polynomial<T>(coeffs);
    }
  }
  return mat;
}

}  // namespace test
}  // namespace drake
