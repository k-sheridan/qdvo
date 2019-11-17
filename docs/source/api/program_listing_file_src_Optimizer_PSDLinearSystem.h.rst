
.. _program_listing_file_src_Optimizer_PSDLinearSystem.h:

Program Listing for File PSDLinearSystem.h
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_PSDLinearSystem.h>` (``src/Optimizer/PSDLinearSystem.h``)

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
   class PSDLinearSystem;
   
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
   
       void initialize(VariableContainer<Variables...>& variables, ErrorTermContainer<ErrorTerms...>& errorTerms) {
   
       }
   
       void solve(VariableContainer<Variables...>& variables, ErrorTermContainer<ErrorTerms...>& errorTerms) {
   
       }
   };
   
   } // namespace ArgMin
