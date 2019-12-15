
.. _program_listing_file_src_Optimizer_PSDSchurSolver.h:

Program Listing for File PSDSchurSolver.h
=========================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_PSDSchurSolver.h>` (``src/Optimizer/PSDSchurSolver.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

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
   #include <numeric>
   #include <cmath>
   
   #include "spdlog/spdlog.h"
   
   namespace ArgMin
   {
   
   template <typename...>
   class PSDSchurSolver;
   
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
   
       struct Settings {
           double initialLambda = 1e3;
           double lambdaReductionMultiplier = 10;
           int maximumIterations = 25;
       } settings;
   
       struct SolveResult {
           std::vector<ScalarType> whitenedSqError;
       };
   
       LossFunctionType lossFunction;
   
       Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> A;
       std::tuple<BVector<UncorrelatedVariables>...> B;
       std::tuple<BVector<UncorrelatedVariables>...> negativeBDinv;
       std::tuple<DVector<UncorrelatedVariables>...> D;
       Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> dx;
       DxBlockVector dxBlockVector;
       Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> b_correlated;
       RHSBlockVector b_uncorrelated;
       std::tuple<IndexMap<Variables>...> variableToIndexMaps;
       size_t dimensionOfA = 0;
       size_t totalDimension = 0;
   
       PSDSchurSolver(LossFunctionType &lossFunction) : lossFunction(lossFunction)
       {
       }
   
       void initialize(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &errorTerms)
       {
           addNewVariablesToSlotArrays(variables);
   
           precomputeIndexMapAndResizeMatrices(variables);
   
           updateVariablePointers(variables, errorTerms);
       }
   
       template <typename GaussianPriorType>
       SolveResult solveLevenbergMarquardt(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &errorTerms, GaussianPriorType &prior)
       {
           spdlog::trace("Starting Levenberg-Marquardt solve.");
           // Initialize the solver.
           initialize(variables, errorTerms);
   
           SolveResult result;
           ScalarType lambda = settings.initialLambda;
   
           // Iterate and solve.
           for (int iteration = 0; iteration < settings.maximumIterations; ++iteration)
           {
               // Linearize the error terms.
               linearize(variables, errorTerms);
               // Build the linear system.
               double whitenedSqError = buildLinearSystem(prior, errorTerms);
   
               spdlog::trace("Iteration: {} Whitened Error: {} Lambda: {}", iteration, sqrt(whitenedSqError), lambda);
   
               // Add the current error to the error array.
               result.whitenedSqError.push_back(whitenedSqError);
   
               // If this is not the first iteration, check if the error was increased
               if (iteration != 0)
               {
                   if (*(result.whitenedSqError.end()-1) < *(result.whitenedSqError.end()-2))
                   {
                       // The error decreased, reduce lambda.
                       lambda = lambda / settings.lambdaReductionMultiplier;
                   }
                   else
                   {
                       // The error increased or stagnated, break.
                       //lambda = lambda * settings.lambdaReductionMultiplier;
                       break;
                   }
               }
   
               // Add lambda to the linear system.
               addLambdaToLinearSystem(lambda);
   
               // Solve the linear system.
               solveLinearSystem(variables, errorTerms, prior);
   
               // Verify that the perturbation is not nan.
               if (isUpdateValid())
               {
                   // Apply the update.
                   spdlog::trace("Updating variables");
                   applyUpdateToVariables(variables);
                   spdlog::trace("Updating prior");
                   prior.update(dxBlockVector);
               } else {
                   // Return early without updating
                   spdlog::trace("Perturbation invalid, returning early");
                   return result;
               }
           }
   
           spdlog::trace("Finished Levenberg-Marquardt solve.");
           return result;
       }
   
       void addLambdaToLinearSystem(ScalarType lambda)
       {
           A.block(0, 0, dimensionOfA, dimensionOfA).diagonal().array() += lambda;
   
           internal::static_for(D, [&](auto i, auto &blockArray) {
               for (auto& block : blockArray)
               {
                   block.diagonal().array() += lambda;
               }
           });
       }
   
       bool isUpdateValid()
       {
           for (int i = 0; i < dx.rows(); ++i)
           {
               if (std::isnan(dx(i, 0)))
               {
                   return false;
               }
           }
           return true;
       }
   
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
   
       template <typename GaussianPriorType>
       void solveLinearSystem(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &linearizedErrorTerms, GaussianPriorType &prior)
       {
           spdlog::trace("Starting Schur Solve with problem dimension: {} and a correlated dimension of: {}", totalDimension, dimensionOfA);
           spdlog::trace("Computing D^{-1}");
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
   
           spdlog::trace("Computing -B D^{-1}");
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
                   A.block(0, 0, dimensionOfA, dimensionOfA).noalias() += (negativeBDinvMatrix.block(0, 0, dimensionOfA, RowVariable::dimension) * bMatrix.block(0, 0, dimensionOfA, RowVariable::dimension).transpose()).eval();
               }
           });
   
           // At this point we have computed the inverse of the LHS.
           // Now we just have to multiply our results with the RHS.
   
           spdlog::trace("Computing -B D^{-1} b_{uncorrelated}");
           // Multiply -BDinv * b_uncorrelated.
           internal::static_for(negativeBDinv, [&](auto i, auto &matrixSlotArray) {
               typedef typename std::tuple_element<i, std::tuple<UncorrelatedVariables...>>::type RowVariable;
               for (auto it = matrixSlotArray.begin(); it != matrixSlotArray.end(); it++)
               {
                   const Eigen::Matrix<ScalarType, Eigen::Dynamic, RowVariable::dimension> &negativeBDinvMatrix = *(it);
   
                   auto key = matrixSlotArray.getKeyFromDataIndex(it - matrixSlotArray.begin());
                   assert(variables.variableExists(key));
   
                   const Eigen::Matrix<ScalarType, RowVariable::dimension, 1> &rhsBlockMatrix = b_uncorrelated.getRowBlock(key);
   
                   b_correlated.block(0, 0, dimensionOfA, 1).noalias() += (negativeBDinvMatrix.block(0, 0, dimensionOfA, RowVariable::dimension) * rhsBlockMatrix).eval();
               }
           });
   
           spdlog::trace("Computing dx_{correlated} = (A - B D^{-1} B^{T})^{-1} b_{correlated}");
           // Multiply the inverse schur complement of D by the correlated b vector.
           dx.block(0, 0, dimensionOfA, 1) = A.block(0, 0, dimensionOfA, dimensionOfA).ldlt().solve(b_correlated.block(0, 0, dimensionOfA, 1));
   
   
           spdlog::trace("Computing D^{-1} b_{uncorrelated}");
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
   
           spdlog::trace("Computing dx_{uncorrelated} = -B D^{-1} dx_{correlated}");
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
   
                   dx.template block<RowVariable::dimension, 1>(*(indexIt), 0).noalias() += (negativeBDinvMatrix.block(0, 0, dimensionOfA, RowVariable::dimension).transpose() * dx.block(0, 0, dimensionOfA, 1)).eval();
               }
           });
   
           spdlog::trace("Setting the dx block vector");
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
   
           spdlog::trace("Schur Solve complete. ");
       }
   
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
   
       template <typename GaussianPriorType>
       double buildLinearSystem(GaussianPriorType &prior, ErrorTermContainer<ErrorTerms...> &linearizedErrorTerms)
       {
           // Initialize the current problem to the prior.
           setProblemToPrior(prior);
   
           double whitenedSqError = 0;
           int nErrorTerms = 0;
   
           // iterate through all error terms
           internal::static_for(linearizedErrorTerms.tupleOfErrorTermMaps, [&](auto errorTermTypeIndex, auto &errorTermMap) {
               spdlog::trace("Building problem with Error Term Type: {}", typeid(typename std::tuple_element<errorTermTypeIndex, std::tuple<ErrorTerms...>>::type).name());
               for (auto &errorTerm : errorTermMap)
               {
                   // Check if the linearization is valid for this error term.
                   if (errorTerm.linearizationValid)
                   {
                       double sqError = errorTerm.residual.squaredNorm();
                       double error = sqrt(sqError);
                       double weight = lossFunction.computeWeight(error, sqError);
   
                       whitenedSqError += sqError;
                       ++nErrorTerms;
   
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
   
           return whitenedSqError / nErrorTerms;
       }
   
       void setProblemToPrior(GaussianPrior<Scalar<ScalarType>, VariableGroup<Variables...>> &prior)
       {
           spdlog::trace("Setting problem to prior.");
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
               auto rowIdxIt = std::get<IndexMap<RowVariable>>(variableToIndexMaps).at(rowKey);
               auto colIdxIt = std::get<IndexMap<ColumnVariable>>(variableToIndexMaps).at(columnKey);
               assert(rowIdxIt != std::get<IndexMap<RowVariable>>(variableToIndexMaps).end());
               assert(colIdxIt != std::get<IndexMap<ColumnVariable>>(variableToIndexMaps).end());
   
               const size_t rowIdx = *(rowIdxIt);
               const size_t colIdx = *(colIdxIt);
   
               A.template block<RowVariable::dimension, ColumnVariable::dimension>(rowIdx, colIdx).noalias() += block;
           }
   
           if constexpr (!row_variable_is_uncorrelated && column_variable_is_uncorrelated)
           {
               // Add this block to B.
               auto rowIdxIt = std::get<IndexMap<RowVariable>>(variableToIndexMaps).at(rowKey);
               assert(rowIdxIt != std::get<IndexMap<RowVariable>>(variableToIndexMaps).end());
               const size_t rowIdx = *(rowIdxIt);
   
               auto &slotArray = std::get<BVector<ColumnVariable>>(B);
               auto it = slotArray.at(columnKey);
   
               assert(it != slotArray.end());
               auto &bMatrix = *(it);
               bMatrix.template block<RowVariable::dimension, ColumnVariable::dimension>(rowIdx, 0).noalias() += block;
           }
       }
   
       void precomputeIndexMapAndResizeMatrices(VariableContainer<Variables...> &variables)
       {
   
           dimensionOfA = 0;
   
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
