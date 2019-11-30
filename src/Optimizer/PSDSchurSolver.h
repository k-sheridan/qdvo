#pragma once

#include "Containers.h"
#include "MetaHelpers.h"
#include "GaussianPrior.h"
#include "SlotMap.h"
#include "SlotArray.h"
#include "BlockVector.h"
#include "HuberLossFunction.h"
#include <cassert>
#include <type_traits>

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
template <typename ScalarType, typename LossFunctionType, typename... ErrorTerms, typename... Variables, typename... UncorrelatedVariables>
class PSDSchurSolver<Scalar<ScalarType>, LossFunction<LossFunctionType>, ErrorTermGroup<ErrorTerms...>, VariableGroup<Variables...>, VariableGroup<UncorrelatedVariables...>>
{
public:
    using RHSBlockVector = BlockVector<Scalar<ScalarType>, Dimension<1>, VariableGroup<UncorrelatedVariables...>>;
    using DxBlockVector = BlockVector<Scalar<ScalarType>, Dimension<1>, VariableGroup<Variables...>>;
    template <typename VariableType>
    using BVector = SlotArray<Eigen::Matrix<ScalarType, Eigen::Dynamic, VariableType::dimension>, VariableKey<VariableType>>;
    template <typename VariableType>
    using DBlock = Eigen::Matrix<ScalarType, VariableType::dimension, VariableType::dimension>;
    template <typename VariableType>
    using DVector = SlotArray<DBlock<VariableType>, VariableKey<VariableType>>;
    template <typename VariableType>
    using IndexMap = SlotArray<size_t, VariableKey<VariableType>>;
    using LossFunction = HuberLossFunction<ScalarType>;

    /// Loss function used to weight the error for each error term.
    LossFunctionType lossFunction;

    /// Dense matrix in the upper right corner
    Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> A;
    /// Inverse Schur Complement of D.
    Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> inverseSchurComplementOfD;
    /// Top right block matrix. Equal to bottom left transposed.
    std::tuple<BVector<UncorrelatedVariables>...> B;
    /// Used during the schur solve to store a precomputed: \f$ -B D^{-1} \f$
    std::tuple<BVector<UncorrelatedVariables>...> negativeBDinv;
    /// Block diagonal matrix in the bottom right. Each block is invertible.
    std::tuple<DVector<UncorrelatedVariables>...> D;
    /// Pre allocated solution vector.
    Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> dx;
    DxBlockVector dxBlockVector;
    /// Preallocated rhs vector.
    Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> b_correlated;
    /// preallocated uncorreleted part of the b vector.
    RHSBlockVector b_uncorrelated;
    /// Tuple of slot arrays which store the current index of a given variable in A.
    std::tuple<IndexMap<Variables>...> variableToIndexMaps;
    /// The current dimension of the A matrix. This exists because we do not need to reduce the size of A.
    size_t dimensionOfA = 0;
    /// The current total problem dimension.
    size_t totalDimension = 0;

    PSDSchurSolver(LossFunctionType &lossFunction) : lossFunction(lossFunction)
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

    /**
     * Applies the perturbation to all variables using their box plus operator.
     * Assumes that the linear system has just been solved.
     * 
     * @param revert If true, this will apply the reverse update to the variables. This can be used to revert a negative update.
     */
    template <bool Revert = false>
    void applyUpdateToVariables(VariableContainer<Variables...> &variables)
    {
        Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> temporary;

        internal::static_for(variables.tupleOfVariableMaps, [&](auto i, auto &variableMap) {
            typedef typename std::tuple_element<i, std::tuple<Variables...>>::type ThisVariable;

            if constexpr (Revert)
            {
                temporary.resize(ThisVariable::dimension, 1);
            }

            for (auto it = variableMap.begin(); it != variableMap.end(); it++)
            {
                auto &variable = *(it);

                auto key = variableMap.getKeyFromDataIndex(it - variableMap.begin());

                auto &indexMap = std::get<IndexMap<ThisVariable>>(variableToIndexMaps);
                auto indexIt = indexMap.at(key);
                assert(indexIt != indexMap.end());

                if constexpr (Revert)
                {
                    temporary = -dx.template block<ThisVariable::dimension, 1>(*(indexIt), 0);
                    variable.update(temporary);
                }
                else
                {
                    variable.update(dx.template block<ThisVariable::dimension, 1>(*(indexIt), 0));
                }
            }
        });
    }

    /**
     * Solves the linear system currently setup.
     * Warning the linear system is invalidated after this runs.
     * It is assumed that the error terms are linearized.
     * 
     * From: https://en.wikipedia.org/wiki/Schur_complement
     * 
     * \f$ {\displaystyle M=\left[{\begin{matrix}A&B\\C&D\end{matrix}}\right]} \f$
     * 
     * \f$ {\displaystyle M/D:=A-BD^{-1}C\,} \f$
     * 
     * \f$ {\displaystyle M/A:=D-CA^{-1}B.} \f$
     * 
     * \f$ {\displaystyle {\begin{aligned}&{\begin{bmatrix}A&B\\C&D\end{bmatrix}}^{-1}={\begin{bmatrix}I_{p}&0\\-D^{-1}C&I_{q}\end{bmatrix}}{\begin{bmatrix}\left(A-BD^{-1}C\right)^{-1}&0\\0&D^{-1}\end{bmatrix}}{\begin{bmatrix}I_{p}&-BD^{-1}\\0&I_{q}\end{bmatrix}}\\[4pt]={}&{\begin{bmatrix}\left(A-BD^{-1}C\right)^{-1}&-\left(A-BD^{-1}C\right)^{-1}BD^{-1}\\-D^{-1}C\left(A-BD^{-1}C\right)^{-1}&D^{-1}+D^{-1}C\left(A-BD^{-1}C\right)^{-1}BD^{-1}\end{bmatrix}}\\[4pt]={}&{\begin{bmatrix}\left(A-BD^{-1}C\right)^{-1}&-\left(A-BD^{-1}C\right)^{-1}BD^{-1}\\-D^{-1}C\left(A-BD^{-1}C\right)^{-1}&\left(D-CA^{-1}B\right)^{-1}\end{bmatrix}}\\[4pt]={}&{\begin{bmatrix}\left(M/D\right)^{-1}&-\left(M/D\right)^{-1}BD^{-1}\\-D^{-1}C\left(M/D\right)^{-1}&\left(M/A\right)^{-1}\end{bmatrix}}.\end{aligned}}} \f$
     * 
     */
    template <typename GaussianPriorType>
    void solveLinearSystem(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &linearizedErrorTerms, GaussianPriorType &prior)
    {

        // Solve for the deltas using the Schur Complement.
        // First invert the D matrix
        internal::static_for(D, [&](auto i, auto &matrixSlotArray) {
            // Get the variable for this section of the matrix.
            typedef typename std::tuple_element<i, std::tuple<UncorrelatedVariables...>>::type RowVariable;

            for (Eigen::Matrix<ScalarType, RowVariable::dimension, RowVariable::dimension> &matrix : matrixSlotArray)
            {
                // TODO Maybe avoid the copy.
                matrix = matrix.inverse();
            }
        });

        // Compute -B Dinv, and compute the inverse Schur Complement of D.
        // This will use A to store the schur complement before inversion.
        internal::static_for(B, [&](auto i, auto &matrixSlotArray) {
            // Get the variable for this section of the matrix.
            typedef typename std::tuple_element<i, std::tuple<UncorrelatedVariables...>>::type RowVariable;
            for (auto it = matrixSlotArray.begin(); it != matrixSlotArray.end(); it++)
            {
                // The original b matrix.
                const Eigen::Matrix<ScalarType, Eigen::Dynamic, RowVariable::dimension> &bMatrix = *(it);

                auto key = matrixSlotArray.getKeyFromDataIndex(it - matrixSlotArray.begin());
                assert(variables.variableExists(key));

                // At this point Dinv should have been computed.
                auto dinvIt = std::get<DVector<RowVariable>>(D).at(key);
                assert(dinvIt != std::get<DVector<RowVariable>>(D).end());
                const Eigen::Matrix<ScalarType, RowVariable::dimension, RowVariable::dimension> &dinv = *(dinvIt);

                // Get the matrix we are going to compute.
                auto negativeBDinvMatrixIt = std::get<BVector<RowVariable>>(negativeBDinv).at(key);
                assert(negativeBDinvMatrixIt != std::get<BVector<RowVariable>>(negativeBDinv).end());
                Eigen::Matrix<ScalarType, Eigen::Dynamic, RowVariable::dimension> &negativeBDinvMatrix = *(negativeBDinvMatrixIt);

                // It is possible that this matrix has more rows than needed.
                negativeBDinvMatrix.block(0, 0, dimensionOfA, RowVariable::dimension).noalias() = (bMatrix.block(0, 0, dimensionOfA, RowVariable::dimension) * -dinv).eval();

                // Add BDinvB' to A.
                A.block(0, 0, dimensionOfA, dimensionOfA).noalias() += (negativeBDinvMatrix.block(0, 0, dimensionOfA, RowVariable::dimension) * bMatrix).eval();
            }
        });

        // Compute the inverse of the schur complement of D.
        inverseSchurComplementOfD.block(0, 0, dimensionOfA, dimensionOfA) = A.block(0, 0, dimensionOfA, dimensionOfA).inverse();

        // At this point we have computed the inverse of the LHS.
        // Now we just have to multiply our results with the RHS.

        // Multiply -BDinv * b_uncorrelated.
        internal::static_for(negativeBDinv, [&](auto i, auto &matrixSlotArray) {
            typedef typename std::tuple_element<i, std::tuple<UncorrelatedVariables...>>::type RowVariable;
            for (auto it = matrixSlotArray.begin(); it != matrixSlotArray.end(); it++)
            {
                const Eigen::Matrix<ScalarType, Eigen::Dynamic, RowVariable::dimension> &negativeBDinvMatrix = *(it);

                auto key = matrixSlotArray.getKeyFromDataIndex(it - matrixSlotArray.begin());
                assert(variables.variableExists(key));

                const Eigen::Matrix<ScalarType, RowVariable::dimension, 1> &rhsBlockMatrix = b_uncorrelated.getRowBlock(key);

                b_correlated.block(0, 0, dimensionOfA, 1).noalias() += (negativeBDinvMatrix * rhsBlockMatrix).eval();
            }
        });

        // Multiply the inverse schur complement of D by the correlated b vector.
        dx.block(0, 0, dimensionOfA, 1).noalias() = inverseSchurComplementOfD.block(0, 0, dimensionOfA, dimensionOfA) * b_correlated.block(0, 0, dimensionOfA, 1);

        // Multiply Dinv by the b_uncorrelated vector.
        internal::static_for(D, [&](auto i, auto &matrixSlotArray) {
            typedef typename std::tuple_element<i, std::tuple<UncorrelatedVariables...>>::type RowVariable;
            for (auto it = matrixSlotArray.begin(); it != matrixSlotArray.end(); it++)
            {
                // D should be inverted at this point.
                const Eigen::Matrix<ScalarType, RowVariable::dimension, RowVariable::dimension> &DinvMatrix = *(it);

                auto key = matrixSlotArray.getKeyFromDataIndex(it - matrixSlotArray.begin());
                assert(variables.variableExists(key));

                const Eigen::Matrix<ScalarType, Eigen::Dynamic, RowVariable::dimension> &bMatrixBlock = b_uncorrelated.getRowBlock(key);

                auto &indexMap = std::get<IndexMap<RowVariable>>(variableToIndexMaps);
                auto indexIt = indexMap.at(key);
                assert(indexIt != indexMap.end());

                dx.template block<RowVariable::dimension, 1>(*(indexIt), 0).noalias() = (DinvMatrix * bMatrixBlock).eval();
            }
        });

        // At this point the partial solution is stored in the dx vector.
        // Compute the final sweep of (-BDinv)^T * dx_uncorrelated.
        // This is correct  because Dinv is symmetric, and C = B^T
        internal::static_for(negativeBDinv, [&](auto i, auto &matrixSlotArray) {
            typedef typename std::tuple_element<i, std::tuple<UncorrelatedVariables...>>::type RowVariable;
            for (auto it = matrixSlotArray.begin(); it != matrixSlotArray.end(); it++)
            {
                const Eigen::Matrix<ScalarType, Eigen::Dynamic, RowVariable::dimension> &negativeBDinvMatrix = *(it);

                auto key = matrixSlotArray.getKeyFromDataIndex(it - matrixSlotArray.begin());
                assert(variables.variableExists(key));

                auto &indexMap = std::get<IndexMap<RowVariable>>(variableToIndexMaps);
                auto indexIt = indexMap.at(key);
                assert(indexIt != indexMap.end());

                dx.template block<RowVariable::dimension, 1>(*(indexIt), 0).noalias() += (negativeBDinvMatrix.transpose() * dx.block(0, 0, dimensionOfA, 1)).eval();
            }
        });

        // Set the dx block vector from the index map and dx vector
        internal::static_for(variableToIndexMaps, [&](auto i, auto &indexMap) {
            typedef typename std::tuple_element<i, std::tuple<Variables...>>::type ThisVariable;

            for (auto it = indexMap.begin(); it != indexMap.end(); it++)
            {
                auto key = indexMap.getKeyFromDataIndex(it - indexMap.begin());

                assert(dxBlockVector.blockExists(key));

                Eigen::Matrix<ScalarType, ThisVariable::dimension, 1> &block = dxBlockVector.getRowBlock(key);

                block = dx.template block<ThisVariable::dimension, 1>(*(it), 0);
            }
        });
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
        std::tuple<UncorrelatedVariables *...> uncorrelatedVariablesTuple;
        internal::static_for(uncorrelatedVariablesTuple, [&](auto i, auto &temp) {
            typedef typename std::tuple_element<i, std::tuple<UncorrelatedVariables...>>::type RowVariable;

            auto &rowMap = b_uncorrelated.template getRowMap<RowVariable>();

            for (auto &block : rowMap)
            {
                block.setZero();
            }
        });
    }

    /**
     * Using the set of linearized error terms, Build up a linear system by computing
     * 
     * \f$ A = A_{0} + \sum_{i=0}^n J_{i}^{\top} \Sigma^{-1} J_{i} \rho \f$
     * 
     * \f$ b = b_{0} + \sum_{i=0}^n J_{i}^{\top} \Sigma^{-1} e_{i} \rho \f$
     * 
     * Where \f$ e_{i} \f$ is the residual and \f$ J_{i} \f$ is the jacobian of error w.r.t to
     * all variables.
     */
    template <typename GaussianPriorType>
    void buildLinearSystem(GaussianPriorType &prior, ErrorTermContainer<ErrorTerms...> &linearizedErrorTerms)
    {
        // Initialize the current problem to the prior.
        setProblemToPrior(prior);

        // iterate through all error terms
        internal::static_for(linearizedErrorTerms.tupleOfErrorTermMaps, [&](auto errorTermTypeIndex, auto &errorTermMap) {
            for (auto &errorTerm : errorTermMap)
            {
                // Check if the linearization is valid for this error term.
                if (errorTerm.linearizationValid)
                {
                    double sqError = errorTerm.residual.squaredNorm();
                    double error = sqrt(sqError);
                    double weight = lossFunction.computeWeight(error, sqError);

                    // Iterate through all independent variables.
                    internal::static_for(errorTerm.variableKeys, [&](auto i, auto &outerVariableKey) {
                        typedef typename std::remove_reference<decltype(outerVariableKey)>::type OuterVariableKeyType;
                        typedef typename std::remove_reference<decltype(errorTerm)>::type ErrorTermType;

                        // Cache the error transformation.
                        Eigen::Matrix<ScalarType, OuterVariableKeyType::variable_type::dimension, ErrorTermType::residual_dimension> rhoJtW = (std::get<i>(errorTerm.variableJacobians).transpose() * errorTerm.information * weight).eval();

                        // Add to rhs.
                        addBlockToRHS(outerVariableKey, rhoJtW * -errorTerm.residual);

                        internal::static_for(errorTerm.variableKeys, [&](auto j, auto &innerVariableKey) {
                            // Extract the variable types from the keys.
                            // These keys must be of type VariableKey<VariableType>.
                            typedef typename std::remove_reference<decltype(innerVariableKey)>::type InnerVariableKeyType;

                            constexpr bool is_inner_variable_uncorrelated = internal::Is_in_tuple<typename InnerVariableKeyType::variable_type, std::tuple<UncorrelatedVariables...>>::value;
                            constexpr bool is_outer_variable_uncorrelated = internal::Is_in_tuple<typename OuterVariableKeyType::variable_type, std::tuple<UncorrelatedVariables...>>::value;

                            //Compute pJtJ and pJte for this error term.
                            // The block is computed as: outer^T * inner.
                            // outer = row. inner = column.
                            // Do not compute uncorrelated x correlated.
                            if constexpr (!(is_outer_variable_uncorrelated && !is_inner_variable_uncorrelated))
                            {
                                // Add to lhs.
                                addBlockToLHS(outerVariableKey, innerVariableKey, rhoJtW * std::get<j>(errorTerm.variableJacobians));
                            }
                        });
                    });
                }
                else
                {
                    std::cout << "linearization invalid for error term." << std::endl;
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

                internal::static_for(variableTuple, [&](auto j, auto &temp) {
                    typedef typename std::tuple_element<j, std::tuple<Variables...>>::type ColumnVariable;

                    constexpr bool row_variable_is_uncorrelated = internal::Is_in_tuple<RowVariable, std::tuple<UncorrelatedVariables...>>::value;
                    constexpr bool column_variable_is_uncorrelated = internal::Is_in_tuple<ColumnVariable, std::tuple<UncorrelatedVariables...>>::value;

                    // Skip if the variables are both uncorrelated and different.
                    if constexpr (!(row_variable_is_uncorrelated && column_variable_is_uncorrelated && !std::is_same<RowVariable, ColumnVariable>::value))
                    {
                        auto &variableMap = sparseBlockRow.template getVariableMap<ColumnVariable>();

                        for (auto &keyColumnMatrixPair : variableMap)
                        {
                            // Get the key for this column.
                            const VariableKey<ColumnVariable> &columnKey = keyColumnMatrixPair.first;

                            // Add the block matrix to the problem.
                            const auto &blockMatrix = keyColumnMatrixPair.second;
                            addBlockToLHS(rowKey, columnKey, blockMatrix);
                        }
                    }
                });
            }
        });

        // Iterate through b0
        internal::static_for(variableTuple, [&](auto i, auto &temp) {
            typedef typename std::tuple_element<i, std::tuple<Variables...>>::type RowVariable;

            auto &priorRHSRowMap = prior.b0.template getRowMap<RowVariable>();

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

        if constexpr (!variable_is_uncorrelated)
        {
            auto &indexMap = std::get<IndexMap<RowVariable>>(variableToIndexMaps);

            auto indexIt = indexMap.at(rowKey);

            assert(indexIt != indexMap.end());

            b_correlated.template block<RowVariable::dimension, 1>(*(indexIt), 0).noalias() += block;
        }
        if constexpr (variable_is_uncorrelated)
        {
            b_uncorrelated.getRowBlock(rowKey).noalias() += block;
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
        if constexpr (row_variable_is_uncorrelated && column_variable_is_uncorrelated)
        {
            // Add this block to D.
            // These keys should be the same type.
            static_assert(std::is_same<RowVariable, ColumnVariable>::value);
            // These keys should be equal.
            assert(rowKey == columnKey);

            // Add the block to D.
            auto &slotArray = std::get<DVector<RowVariable>>(D);
            auto it = slotArray.at(rowKey);

            // The matrix should exist.
            assert(it != slotArray.end());

            auto &lhs = *(it);
            lhs.noalias() += block;
        }

        if constexpr (!row_variable_is_uncorrelated && !column_variable_is_uncorrelated)
        {
            // Add this block to A.
            const size_t rowIdx = *(std::get<IndexMap<RowVariable>>(variableToIndexMaps).at(rowKey));
            const size_t colIdx = *(std::get<IndexMap<ColumnVariable>>(variableToIndexMaps).at(columnKey));

            A.template block<RowVariable::dimension, ColumnVariable::dimension>(rowIdx, colIdx).noalias() += block;
        }

        if constexpr (!row_variable_is_uncorrelated && column_variable_is_uncorrelated)
        {
            // Add this block to B.
            const size_t rowIdx = *(std::get<IndexMap<RowVariable>>(variableToIndexMaps).at(rowKey));

            auto &slotArray = std::get<BVector<ColumnVariable>>(B);
            auto it = slotArray.at(columnKey);

            assert(it != slotArray.end());
            auto &bMatrix = *(it);
            bMatrix.template block<RowVariable::dimension, ColumnVariable::dimension>(rowIdx, 0).noalias() += block;
        }
    }

    /// Precomputes the map bewteen correlated variable keys and their index in A.
    void precomputeIndexMapAndResizeMatrices(VariableContainer<Variables...> &variables)
    {

        dimensionOfA = 0;

        /// compute the index map starting with only the correlated variables.
        internal::static_for(variables.tupleOfVariableMaps, [&](auto i, auto &variableMap) {
            typedef typename std::tuple_element<i, std::tuple<Variables...>>::type ThisVariable;

            // Only set the dimensions if this variable is not part of the uncorrelated set.
            if constexpr (!(internal::Is_in_tuple<ThisVariable, std::tuple<UncorrelatedVariables...>>::value))
            {
                for (size_t idx = 0; idx < variableMap.size(); ++idx)
                {
                    auto key = variableMap.getKeyFromDataIndex(idx);
                    assert(variables.variableExists(key));

                    std::get<IndexMap<ThisVariable>>(variableToIndexMaps).insert(key, dimensionOfA);

                    dimensionOfA += ThisVariable::dimension;
                }
            }
        });

        totalDimension = dimensionOfA;

        /// Compute the remaining variables in the index map (uncorrelated set).
        internal::static_for(variables.tupleOfVariableMaps, [&](auto i, auto &variableMap) {
            typedef typename std::tuple_element<i, std::tuple<Variables...>>::type ThisVariable;

            // Only set the dimensions if this variable is part of the uncorrelated set.
            if constexpr ((internal::Is_in_tuple<ThisVariable, std::tuple<UncorrelatedVariables...>>::value))
            {
                for (size_t idx = 0; idx < variableMap.size(); ++idx)
                {
                    auto key = variableMap.getKeyFromDataIndex(idx);
                    assert(variables.variableExists(key));

                    std::get<IndexMap<ThisVariable>>(variableToIndexMaps).insert(key, totalDimension);

                    totalDimension += ThisVariable::dimension;
                }
            }
        });

        // Resize the dense portion of dx.
        if (dx.rows() < totalDimension)
        {
            dx.resize(totalDimension, 1);
        }

        // Resize A if necessary.
        assert(A.rows() == A.cols());
        if (A.rows() < dimensionOfA)
        {
            A.resize(dimensionOfA, dimensionOfA);
        }

        // Resize inverseSchurComplementOfD if necessary.
        assert(inverseSchurComplementOfD.rows() == inverseSchurComplementOfD.cols());
        if (inverseSchurComplementOfD.rows() < dimensionOfA)
        {
            inverseSchurComplementOfD.resize(dimensionOfA, dimensionOfA);
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

        // Resize the matrices of negativeBDinv if necessary
        internal::static_for(negativeBDinv, [&](auto i, auto &array) {
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

            Eigen::Matrix<ScalarType, ThisVariable::dimension, 1> zeroRHSMatrix = RHSBlockVector::template MatrixBlock<ThisVariable>::Zero();

            for (size_t idx = 0; idx < variableMap.size(); ++idx)
            {
                auto key = variableMap.getKeyFromDataIndex(idx);
                assert(variables.variableExists(key));
                // Only do this for uncorrelated variables.
                if constexpr (internal::Is_in_tuple<ThisVariable, std::tuple<UncorrelatedVariables...>>::value)
                {

                    // Check if the key exists in B
                    if (std::get<BVector<ThisVariable>>(B).at(key) == std::get<BVector<ThisVariable>>(B).end())
                    {
                        std::get<BVector<ThisVariable>>(B).insert(key, Eigen::Matrix<ScalarType, Eigen::Dynamic, ThisVariable::dimension>::Zero(dimensionOfA));
                    }

                    // Check if the key exists in negativeBDinv
                    if (std::get<BVector<ThisVariable>>(negativeBDinv).at(key) == std::get<BVector<ThisVariable>>(negativeBDinv).end())
                    {
                        std::get<BVector<ThisVariable>>(negativeBDinv).insert(key, Eigen::Matrix<ScalarType, Eigen::Dynamic, ThisVariable::dimension>::Zero(dimensionOfA));
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

                // Add an elements to the dx block vector.
                if (!dxBlockVector.blockExists(key))
                {
                    dxBlockVector.addRowBlock(key, zeroRHSMatrix);
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