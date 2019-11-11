#pragma once

#include "Containers.h"
#include "MetaHelpers.h"
#include "GaussianPrior.h"
#include "SlotMap.h"
#include "SlotArray.h"

namespace ArgMin
{

template <typename...>
class PSDLinearSystem;

/**
 * This class provides the functions to solve a levenberg marquardt or gauss newton iteration.
 * It sets up the sparse linear system, and solves it by exploiting the sparsity using the schur complement.
 * 
 * Internally, the linear system, which contains a square PSD matrix, is split into 3 blocks
 * [A,  B]
 * [B', D]
 * A is a dense matrix, D is a block diagonal matrix, and B is a dense matrix correlating A and D.
 * 
 * Uncorrelated variables are variables which share no off diagonal elements.
 * It is assumed that variables are unique across both variable sets.
 * For example: CorrelatedSet = {T1, T2, T3}, Uncorrelated Set = {T4, T5, T6}
 */
template <typename ScalarType, typename... ErrorTerms, typename... Variables, typename... UncorrelatedVariables>
class PSDLinearSystem<Scalar<ScalarType>, ErrorTermGroup<ErrorTerms...>, VariableGroup<Variables...>, VariableGroup<UncorrelatedVariables...>>
{
public:
    template <typename VariableType>
    using BVector = SlotArray<Eigen::Matrix<ScalarType, Eigen::Dynamic, VariableType::dimension>, VariableKey<VariableType>>;
    template <typename VariableType>
    using DBlock = Eigen::Matrix<ScalarType, VariableType::dimension, VariableType::dimension>;
    template <typename VariableType>
    using DVector = SlotArray<DBlock<VariableType>, VariableKey<VariableType>>;

    Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> A; // Dense matrix in the upper right corner
    std::tuple<BVector<UncorrelatedVariables>...> B;             // Top right block matrix. Equal to bottom left transposed.
    std::tuple<DVector<UncorrelatedVariables>...> D;             // Block diagonal matrix in the bottom right. Each block is invertible.

    Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> dx; // Pre allocated solution vector.
    BlockVector<Scalar<ScalarType>, Dimension<1>, VariableGroup<Variables...>> dxBlockVector; // Block vector form of the dx vector. Used to solve ordering issues.
    
    Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> b; // Preallocated rhs vector.

    PSDLinearSystem()
    {
    }

    /**
     * This function is ran once before a solve. This could be used to preallocate memory or precompute values.
     */
    void initialize(VariableContainer<Variables...>& variables, ErrorTermContainer<ErrorTerms...>& errorTerms) {

    }

    /**
     * This function will linearize the error terms if necessary, build the problem, and solve for the perturbation
     * while exploiting the knowledge of the uncorrelated variable set.
     * 
     * After the solve, it can be assumed that the error terms are in the linearized state used for the final iteration.
     */
    void solve(VariableContainer<Variables...>& variables, ErrorTermContainer<ErrorTerms...>& errorTerms) {

    }
};

} // namespace ArgMin