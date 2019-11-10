#pragma once

#include "Containers.h"
#include "MetaHelpers.h"
#include "GaussianPrior"
#include "SlotMap.h"
#include "SlotArray.h"

namespace ArgMin {

template <typename... T>
class PSDLinearSystem;

/**
 * This class provides the functions to solve a levenberg marquardt or gauss newton iteration.
 * It sets up the sparse linear system, and solves it by exploiting the sparsity using the schur complement.
 * 
 * Internally, the linear system, which contains a square PSD matrix, is split into 3 blocks
 * [A,  B]
 * [B', D]
 * A is a dense matrix, D is a block diagonal matrix, and B is a dense matrix correlating A and D.
 */
template <typename ScalarType, typename... ErrorTerms, typename... Variables>
class PSDLinearSystem<Scalar<ScalarType>, ErrorTermGroup<ErrorTerms...>, VariableGroup<Variables...>> {
public:

template <typename VariableType>
using BVector = SlotArray<VariableKey<VariableType>, Eigen::Matrix<ScalarType, Eigen::Dynamic, VariableType::dimension>;
template <typename VariableType>
using DBlock = Eigen::Matrix<ScalarType, VariableType::dimension, VariableType::dimension>;
template <typename VariableType>
using DVector = SlotArray<VariableKey<VariableType>, DBlock<VariableType>>;


Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> A; // Dense matrix in the upper right corner
std::tuple<BVector<Variables>>...> B; // Top right block matrix. Equal to bottom left transposed.
std::tuple<DVector<Variables>>...> D; // Block diagonal matrix in the bottom right. Each block is invertible.


};

} // namespace ArgMin