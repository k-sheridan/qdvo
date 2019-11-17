
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
   
   namespace ArgMin
   {
   
   template <typename...>
   class PSDSchurSolver;
   
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
   
       void initialize(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &errorTerms)
       {
           precomputeIndexMapAndResizeA(variables);
       }
   
       void solve(VariableContainer<Variables...> &variables, ErrorTermContainer<ErrorTerms...> &errorTerms)
       {
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
