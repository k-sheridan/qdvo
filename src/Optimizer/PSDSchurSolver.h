#pragma once

#include "Containers.h"
#include "MetaHelpers.h"
#include "GaussianPrior.h"
#include "SlotMap.h"
#include "SlotArray.h"
#include "BlockVector.h"
#include "HuberLossFunction.h"
#include <cassert>

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
    using RHSBlockVector = BlockVector<Scalar<ScalarType>, Dimension<1>, VariableGroup<UncorrelatedVariables...>>;
    template <typename VariableType>
    using BVector = SlotArray<Eigen::Matrix<ScalarType, Eigen::Dynamic, VariableType::dimension>, VariableKey<VariableType>>;
    template <typename VariableType>
    using DBlock = Eigen::Matrix<ScalarType, VariableType::dimension, VariableType::dimension>;
    template <typename VariableType>
    using DVector = SlotArray<DBlock<VariableType>, VariableKey<VariableType>>;
    template <typename VariableType>
    using IndexMap = SlotArray<size_t, VariableKey<VariableType>>;
    using LossFunction = HuberLossFunction<ScalarType>;

    LossFunction lossFunction;

    Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> A; // Dense matrix in the upper right corner
    std::tuple<BVector<UncorrelatedVariables>...> B;             // Top right block matrix. Equal to bottom left transposed.
    std::tuple<DVector<UncorrelatedVariables>...> D;             // Block diagonal matrix in the bottom right. Each block is invertible.

    Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> dx;                                          // Pre allocated solution vector.
    BlockVector<Scalar<ScalarType>, Dimension<1>, VariableGroup<Variables...>> dxBlockVector; // Block vector form of the dx vector. Used to solve ordering issues.

    Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> b_correlated; // Preallocated rhs vector.
    RHSBlockVector b_uncorrelated; // preallocated uncorreleted part of the b vector.


    std::tuple<IndexMap<Variables>...> variableToIndexMaps; // Tuple of slot arrays which store the current index of a given variable in A.
    size_t dimensionOfA = 0;                                // The current dimension of the A matrix. This exists because we do not need to reduce the size of A.

    PSDSchurSolver() : lossFunction(1e-6)
    {
    }

    /**
     * This function is ran once before a solve. This could be used to preallocate memory or precompute values.
     */
    void initialize(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &errorTerms)
    {
        addNewVariablesToSlotArrays(variables);

        precomputeIndexMapAndResizeMatrices(variables);

        updateVariablePointers(variables, errorTerms);
    }

    /**
     * This function will linearize the error terms if necessary, build the problem, and solve for the perturbation
     * while exploiting the knowledge of the uncorrelated variable set.
     * 
     * After the solve, it can be assumed that the error terms are in the linearized state used for the final iteration.
     */
    template <typename GaussianPriorType>
    void solve(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &errorTerms, GaussianPriorType &prior)
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
    void setZero()
    {
        A.setZero();
        b_correlated.setZero();

        // iterate over all tuple elements and zero them.
        internal::static_for(B, [&](auto i, auto &array) {
            for (auto &matrix : array)
            {
                matrix.setZero();
            }
        });

        // iterate over all tuple elements and zero them.
        internal::static_for(D, [&](auto i, auto &array) {
            for (auto &matrix : array)
            {
                matrix.setZero();
            }
        });

        // Hacky way of iterating over variables.
        std::tuple<UncorrelatedVariables*...> uncorrelatedVariablesTuple;
        internal::static_for(uncorrelatedVariablesTuple, [&](auto i, auto &temp) {
            typedef typename std::tuple_element<i, std::tuple<UncorrelatedVariables...>>::type RowVariable;

            auto& rowMap = b_uncorrelated.template getRowMap<RowVariable>();

            for (auto& block : rowMap)
            {
                block.setZero();
            }
        });
    }

    /**
     * Using the set of linearized error terms, Build up a linear system by computing
     * 
     * \f$ A = A_{0} + \sum_{i=0}^n J_{i}^{\top} J_{i} \f$
     * 
     * \f$ b = b_{0} + \sum_{i=0}^n J_{i}^{\top} e_{i} \f$
     * 
     * Where \f$ e_{i} \f$ is the residual and \f$ J_{i} \f$ is the jacobian of error w.r.t to
     * all variables.
     */
    template <typename GaussianPriorType>
    void buildProblem(GaussianPriorType &prior, ErrorTermContainer<ErrorTerms...> &linearizedErrorTerms)
    {
        // Initialize the current problem to the prior.
        setProblemToPrior(prior);

        // iterate through all error terms
        internal::static_for(linearizedErrorTerms.tupleOfErrorTermMaps, [&](auto errorTermTypeIndex, auto &errorTermMap) {
            for (auto& errorTerm : errorTermMap)
            {
                // Check if the linearization is valid for this error term.
                if (errorTerm.linearizationValid)
                {
                    // Iterate through all independent variables.
                    internal::static_for(errorTerm.variableKeys, [&](auto i, auto &outerVariableKey) {
                        internal::static_for(errorTerm.variableKeys, [&](auto j, auto &innerVariableKey) {
                            // TODO Compute pJtJ and pJte for this error term.
                            double sqError = errorTerm.residual.squaredNorm();
                            double error = sqrt(sqError);
                            double loss = lossFunction.loss(error, sqError);
                        }); 
                    });
                }
            }
        });
    }

    /**
     * Sets the problem exactly to the prior.
     * 
     * \f$ A = A_{0} \f$
     * 
     * \f$ b = b_{0} \f$
     */
    void setProblemToPrior(GaussianPrior<Scalar<ScalarType>, VariableGroup<Variables...>> &prior)
    {
        // Zero the problem.
        setZero();

        auto &A0 = prior.A0;
        auto &b0 = prior.b0;

        // Semi hack way of iterating through all variable types
        std::tuple<Variables *...> variableTuple;

        // Iterate through A0
        internal::static_for(variableTuple, [&](auto i, auto &temp) {
            typedef typename std::tuple_element<i, std::tuple<Variables...>>::type RowVariable;

            auto &rowMap = A0.template getRowMap<RowVariable>();

            for (auto &keySparseBlockRowPair : rowMap)
            {
                // Get the key for this row.
                const VariableKey<RowVariable> &rowKey = keySparseBlockRowPair.first;
                auto &sparseBlockRow = keySparseBlockRowPair.second;

                internal::static_for(variableTuple, [&](auto i, auto &temp) {
                    typedef typename std::tuple_element<i, std::tuple<Variables...>>::type ColumnVariable;

                    auto &variableMap = sparseBlockRow.template getVariableMap<ColumnVariable>();

                    for (auto &keyColumnMatrixPair : variableMap)
                    {
                        // Get the key for this column.
                        const VariableKey<ColumnVariable> &columnKey = keyColumnMatrixPair.first;

                        // Add the block matrix to the problem.
                        const auto &blockMatrix = keyColumnMatrixPair.second;
                        addBlockToLHS(rowKey, columnKey, blockMatrix);

                    }
                });
            }
        });

        // Iterate through b0
        internal::static_for(variableTuple, [&](auto i, auto &temp) {
            typedef typename std::tuple_element<i, std::tuple<Variables...>>::type RowVariable;

            auto& priorRHSRowMap = prior.b0.template getRowMap<RowVariable>();

            for (auto it = priorRHSRowMap.begin(); it != priorRHSRowMap.end(); it++)
            {
                //  Get the key for this block.
                auto key = priorRHSRowMap.getKeyFromDataIndex(it - priorRHSRowMap.begin());

                // add the block to b.
                addBlockToRHS(key, *(it));
            }
            
        });
    }

    /**
     * Adds a given column block to the left hand side of the problem.
     * This functions assumes that the index map has been computed.
     */
    template <typename RowVariable>
    void addBlockToRHS(const VariableKey<RowVariable> &rowKey, const Eigen::Matrix<ScalarType, RowVariable::dimension, 1> &block)
    {
        constexpr bool variable_is_uncorrelated = internal::Is_in_tuple<RowVariable, std::tuple<UncorrelatedVariables...>>::value;

        if constexpr(!variable_is_uncorrelated)
        {
            auto& indexMap = std::get<IndexMap<RowVariable>>(variableToIndexMaps);

            auto indexIt = indexMap.at(rowKey);

            assert(indexIt != indexMap.end());

            b_correlated.template block<RowVariable::dimension, 1>(*(indexIt), 0) += block;
        } 
        if constexpr(variable_is_uncorrelated)
        {
            b_uncorrelated.getRowBlock(rowKey) += block;
        }
    }

    /**
     * Adds a given column block to the left hand side of the problem.
     * This functions assumes that the index map has been computed.
     */
    template <typename RowVariable, typename ColumnVariable>
    void addBlockToLHS(const VariableKey<RowVariable> &rowKey, const VariableKey<ColumnVariable> &columnKey, const Eigen::Matrix<ScalarType, RowVariable::dimension, ColumnVariable::dimension> &block)
    {
        // Verify that these keys are part of the variable set.
        static_assert(internal::Is_in_tuple<RowVariable, std::tuple<Variables...>>::value);
        static_assert(internal::Is_in_tuple<ColumnVariable, std::tuple<Variables...>>::value);

        constexpr bool row_variable_is_uncorrelated = internal::Is_in_tuple<RowVariable, std::tuple<UncorrelatedVariables...>>::value;
        constexpr bool column_variable_is_uncorrelated = internal::Is_in_tuple<ColumnVariable, std::tuple<UncorrelatedVariables...>>::value;

        // Compile time if statements used to determin where the block matrix should be added to.
        if constexpr(row_variable_is_uncorrelated && column_variable_is_uncorrelated) {
            // Add this block to D.
            // These keys should be the same type.
            static_assert(std::is_same<RowVariable, ColumnVariable>::value);
            // These keys should be equal.
            assert(rowKey == columnKey);

            // Add the block to D.
            auto& slotArray = std::get<DVector<RowVariable>>(D);
            auto it = slotArray.at(rowKey);

            // The matrix should exist.
            assert(it != slotArray.end());

            *(it) += block;
        }

        if constexpr(!row_variable_is_uncorrelated && !column_variable_is_uncorrelated) {
            // Add this block to A.
            const size_t rowIdx = *(std::get<IndexMap<RowVariable>>(variableToIndexMaps).at(rowKey));
            const size_t colIdx = *(std::get<IndexMap<ColumnVariable>>(variableToIndexMaps).at(columnKey));

            A.template block<RowVariable::dimension, ColumnVariable::dimension>(rowIdx, colIdx) += block;
        }

        if constexpr(!row_variable_is_uncorrelated && column_variable_is_uncorrelated) {
            // Add this block to B.
            const size_t rowIdx = *(std::get<IndexMap<RowVariable>>(variableToIndexMaps).at(rowKey));

            auto& slotArray = std::get<BVector<ColumnVariable>>(B);
            auto it = slotArray.at(columnKey);

            assert(it != slotArray.end());
            auto& bMatrix = *(it);
            bMatrix.template block<RowVariable::dimension, ColumnVariable::dimension>(rowIdx, 0) += block;
        }
    }

    /// Precomputes the map bewteen correlated variable keys and their index in A.
    void precomputeIndexMapAndResizeMatrices(VariableContainer<Variables...> &variables)
    {

        dimensionOfA = 0;

        /// compute the index map.
        internal::static_for(variables.tupleOfVariableMaps, [&](auto i, auto &variableMap) {
            typedef typename std::tuple_element<i, std::tuple<Variables...>>::type ThisVariable;

            // Only set the dimensions if this variable is not part of the uncorrelated set.
            if constexpr (!(internal::Is_in_tuple<ThisVariable, std::tuple<UncorrelatedVariables...>>::value))
            {
                for (size_t idx = 0; idx < variableMap.size(); ++idx)
                {
                    auto key = variableMap.getKeyFromDataIndex(idx);

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

        // Resize b_correlated if necessary
        if (b_correlated.rows() < dimensionOfA)
        {
            b_correlated.resize(dimensionOfA, 1);
        }

        // Resize the matrices of B if necessary
        internal::static_for(B, [&](auto i, auto &array) {
            typedef typename std::tuple_element<i, std::tuple<UncorrelatedVariables...>>::type ThisVariable;
            for (auto &matrix : array)
            {
                if (matrix.rows() < dimensionOfA)
                {
                    matrix.resize(dimensionOfA, ThisVariable::dimension);
                }
            }
        });
    }

    /// Ensures that the slot arrays stored in the solver are in sync with the current variable set.
    void addNewVariablesToSlotArrays(VariableContainer<Variables...> &variables)
    {
        // Insert variables if they do not exist.
        internal::static_for(variables.tupleOfVariableMaps, [&](auto i, auto &variableMap) {
            typedef typename std::tuple_element<i, std::tuple<Variables...>>::type ThisVariable;

            // Only do this for uncorrelated variables.
            if constexpr (internal::Is_in_tuple<ThisVariable, std::tuple<UncorrelatedVariables...>>::value)
            {
                Eigen::Matrix<ScalarType, ThisVariable::dimension, 1> zeroRHSMatrix = RHSBlockVector::template MatrixBlock<ThisVariable>::Zero();

                for (size_t idx = 0; idx < variableMap.size(); ++idx)
                {
                    auto key = variableMap.getKeyFromDataIndex(idx);

                    // Check if the key exists in B
                    if (std::get<BVector<ThisVariable>>(B).at(key) == std::get<BVector<ThisVariable>>(B).end())
                    {
                        std::get<BVector<ThisVariable>>(B).insert(key, Eigen::Matrix<ScalarType, Eigen::Dynamic, ThisVariable::dimension>::Zero(dimensionOfA));
                    }

                    // Check if the key exists in D
                    if (std::get<DVector<ThisVariable>>(D).at(key) == std::get<DVector<ThisVariable>>(D).end())
                    {
                        std::get<DVector<ThisVariable>>(D).insert(key, DBlock<ThisVariable>::Zero());
                    }

                    // Check if the key exists in b_uncorrelated
                    if (!b_uncorrelated.blockExists(key))
                    {
                        b_uncorrelated.addRowBlock(key, zeroRHSMatrix);
                    }
                }
            }
        });
    }

    /// Updates the pointers to variables inside the error terms.
    void updateVariablePointers(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &errorTerms)
    {
        // iterate over all error term maps
        internal::static_for(errorTerms.tupleOfErrorTermMaps, [&](auto i, auto &errorTermMap) {
            for (auto &errorTerm : errorTermMap)
            {
                errorTerm.updateVariablePointers(variables);
            }
        });
    }
};

} // namespace ArgMin