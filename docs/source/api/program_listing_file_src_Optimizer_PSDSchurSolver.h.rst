
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
   
   namespace ArgMin
   {
   
   template <typename...>
   class PSDSchurSolver;
   
   template <typename ScalarType, typename LossFunctionType, typename... ErrorTerms, typename... Variables, typename... UncorrelatedVariables>
   class PSDSchurSolver<Scalar<ScalarType>, LossFunction<LossFunctionType>, ErrorTermGroup<ErrorTerms...>, VariableGroup<Variables...>, VariableGroup<UncorrelatedVariables...>>
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
   
       LossFunctionType lossFunction;
   
       Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> A; 
       std::tuple<BVector<UncorrelatedVariables>...> B;      
       std::tuple<DVector<UncorrelatedVariables>...> D;             
       Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> dx;    
       BlockVector<Scalar<ScalarType>, Dimension<1>, VariableGroup<Variables...>> dxBlockVector; 
       Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> b_correlated; 
       RHSBlockVector b_uncorrelated;                             
       std::tuple<IndexMap<Variables>...> variableToIndexMaps; 
       size_t dimensionOfA = 0;                                
   
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
       void solve(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &errorTerms, GaussianPriorType &prior)
       {
       }
   
       template <typename GaussianPriorType>
       void iterate(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &linearizedErrorTerms, GaussianPriorType &prior)
       {
           // Build the problem.
           buildProblem(prior, linearizedErrorTerms);
   
           // Solve for the deltas using the Schur Complement.
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
       void buildProblem(GaussianPriorType &prior, ErrorTermContainer<ErrorTerms...> &linearizedErrorTerms)
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
                           // Cache the error transformation.
                           auto rhoJtW = (std::get<i>(errorTerm.variableJacobians).transpose() * errorTerm.information * weight).eval();
   
                           // Add to rhs.
                           addBlockToRHS(outerVariableKey, rhoJtW * errorTerm.residual);
   
                           internal::static_for(errorTerm.variableKeys, [&](auto j, auto &innerVariableKey) {
                               // Extract the variable types from the keys.
                               // These keys must be of type VariableKey<VariableType>.
                               typedef typename std::remove_reference<decltype(innerVariableKey)>::type InnerVariableKeyType;
                               typedef typename std::remove_reference<decltype(outerVariableKey)>::type OuterVariableKeyType;
   
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
   
               b_correlated.template block<RowVariable::dimension, 1>(*(indexIt), 0) += block;
           }
           if constexpr (variable_is_uncorrelated)
           {
               b_uncorrelated.getRowBlock(rowKey) += block;
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
   
               *(it) += block;
           }
   
           if constexpr (!row_variable_is_uncorrelated && !column_variable_is_uncorrelated)
           {
               // Add this block to A.
               const size_t rowIdx = *(std::get<IndexMap<RowVariable>>(variableToIndexMaps).at(rowKey));
               const size_t colIdx = *(std::get<IndexMap<ColumnVariable>>(variableToIndexMaps).at(columnKey));
   
               A.template block<RowVariable::dimension, ColumnVariable::dimension>(rowIdx, colIdx) += block;
           }
   
           if constexpr (!row_variable_is_uncorrelated && column_variable_is_uncorrelated)
           {
               // Add this block to B.
               const size_t rowIdx = *(std::get<IndexMap<RowVariable>>(variableToIndexMaps).at(rowKey));
   
               auto &slotArray = std::get<BVector<ColumnVariable>>(B);
               auto it = slotArray.at(columnKey);
   
               assert(it != slotArray.end());
               auto &bMatrix = *(it);
               bMatrix.template block<RowVariable::dimension, ColumnVariable::dimension>(rowIdx, 0) += block;
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
