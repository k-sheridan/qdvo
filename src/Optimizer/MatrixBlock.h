#pragma once

#include <Eigen/Core>

namespace LittleOptimizer {

template<typename Scalar, size_t rows, size_t cols>
class MatrixBlock {

typedef Eigen::Matrix<Scalar, rows, cols> matrix_t;



private:

bool zero = true; // tracks whether the sub matrix is zero or not.

matrix_t matrix;

};
}