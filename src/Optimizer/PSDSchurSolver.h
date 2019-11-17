#pragma once

#include "Containers.h"
#include "MetaHelpers.h"
#include "GaussianPrior.h"
#include "SlotMap.h"
#include "SlotArray.h"

namespace ArgMin
{

template <typename...>
class PSDSchurSolver;

/**
 * This class provides the functions to solve a levenberg marquardt or gauss newton iteration.
 * It sets up the sparse linear system, and solves it by exploiting the sparsity using the schur complement.
 * 
 * Internally, the linear system, which contains a square PSD matrix, is split into 3 blocks
 * 
 * \f$ \left[
 * \begin{array}{c|c}
 * A & B \\
 * \hline
 * B^{T} & D
 * \end{array}
 * \right] \f$
 * 
 * A is a dense matrix, D is a block diagonal matrix, and B is a dense matrix correlating A and D.
 * 
 * Uncorrelated variables are variables which share no off diagonal elements.
 * The uncorrelated variable set must be a subset of all variables.
 */
template <typename ScalarType, typename... ErrorTerms, typename... Variables, typename... UncorrelatedVariables>
class PSDSchurSolver<Scalar<ScalarType>, ErrorTermGroup<ErrorTerms...>, VariableGroup<Variables...>, VariableGroup<UncorrelatedVariables...>>
{
public:
    template <typename VariableType>
    using BVector = SlotArray<Eigen::Matrix<ScalarType, Eigen::Dynamic, VariableType::dimension>, VariableKey<VariableType>>;
    template <typename VariableType>
    using DBlock = Eigen::Matrix<ScalarType, VariableType::dimension, VariableType::dimension>;
    template <typename VariableType>
    using DVector = SlotArray<DBlock<VariableType>, VariableKey<VariableType>>;
    template <typename VariableType>
    using IndexMap = SlotArray<size_t, VariableKey<VariableType>>;

    Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> A; // Dense matrix in the upper right corner
    std::tuple<BVector<UncorrelatedVariables>...> B;             // Top right block matrix. Equal to bottom left transposed.
    std::tuple<DVector<UncorrelatedVariables>...> D;             // Block diagonal matrix in the bottom right. Each block is invertible.

    Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> dx;                                          // Pre allocated solution vector.
    BlockVector<Scalar<ScalarType>, Dimension<1>, VariableGroup<Variables...>> dxBlockVector; // Block vector form of the dx vector. Used to solve ordering issues.

    Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> b; // Preallocated rhs vector.

    std::tuple<IndexMap<Variables>...> variableToIndexMaps; // Tuple of slot arrays which store the current index of a given variable in A.
    size_t dimensionOfA = 0; // The current dimension of the A matrix. This exists because we do not need to reduce the size of A.

    PSDSchurSolver()
    {

    }

    /**
     * This function is ran once before a solve. This could be used to preallocate memory or precompute values.
     */
    void initialize(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &errorTerms)
    {
        precomputeIndexMapAndResizeA(variables);
    }

    /**
     * This function will linearize the error terms if necessary, build the problem, and solve for the perturbation
     * while exploiting the knowledge of the uncorrelated variable set.
     * 
     * After the solve, it can be assumed that the error terms are in the linearized state used for the final iteration.
     */
    void solve(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &errorTerms)
    {
    }

    /// Linearizes all error terms stored in this container.
    void linearize(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &errorTerms)
    {
        // loop through all error terms and linearize all of them.
        internal::static_for(errorTerms.tupleOfErrorTermMaps, [&](auto i, auto &errorTermMap) {
            for (auto &errorTerm : errorTermMap)
            {
                errorTerm.evaluate(variables, true);
            }
        });
    }

    /// Zeros all matrices in the problem.
    void reset() {
        A.setZero();
        b.setZero();

        // iterate over all tuple elements and zero them.
        internal::static_for(B, [&](auto i, auto &array) {
            for (auto& matrix : array) {
                matrix.setZero();
            }
        });

        // iterate over all tuple elements and zero them.
        internal::static_for(D, [&](auto i, auto &array) {
            for (auto& matrix : array) {
                matrix.setZero();
            }
        });
    }

    /// Precomputes the map bewteen correlated variable keys and their index in A.
    /// Resizes A if too small 
    void precomputeIndexMapAndResizeA(VariableContainer<Variables...> &variables) {
        dimensionOfA = 0;

        internal::static_for(variables.tupleOfVariableMaps, [&](auto i, auto &variableMap) {
            typedef typename std::tuple_element<i, std::tuple<Variables...>>::type ThisVariable;

            // Only set the dimensions if this variable is not part of the uncorrelated set.
            if constexpr(internal::Is_in_tuple<ThisVariable, std::tuple<UncorrelatedVariables...>>::value) {

            } else {
                for (size_t i = 0; i < variableMap.size(); ++i) 
                {
                    auto key = variableMap.getKeyFromDataIndex(i);

                    std::get<IndexMap<ThisVariable>>(variableToIndexMaps).insert(key, dimensionOfA);

                    dimensionOfA += ThisVariable::dimension;
                }
            }

        });

        // Resize A if necessary.
        assert(A.rows() == A.cols());
        if (A.rows() < dimensionOfA)
        {  
            A.resize(dimensionOfA, dimensionOfA);
        }
    }
};

} // namespace ArgMin