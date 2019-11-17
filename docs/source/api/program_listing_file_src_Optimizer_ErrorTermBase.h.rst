
.. _program_listing_file_src_Optimizer_ErrorTermBase.h:

Program Listing for File ErrorTermBase.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_ErrorTermBase.h>` (``src/Optimizer/ErrorTermBase.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include <Eigen/Core>
   #include "Optimizer/Key.h"
   #include "Optimizer/Containers.h"
   #include "Optimizer/MetaHelpers.h"
   
   namespace ArgMin
   {
   
   template <typename...>
   class ErrorTermBase;
   
   template <int ResidualDimension, typename ScalarType, typename... IndependentVariables>
   class ErrorTermBase<Scalar<ScalarType>, Dimension<ResidualDimension>, VariableGroup<IndependentVariables...>>
   {
   public:
   
       std::tuple<Eigen::Matrix<ScalarType, ResidualDimension, IndependentVariables::dimension>...> variableJacobians;
       std::tuple<VariableKey<IndependentVariables>...> variableKeys;
       std::tuple<IndependentVariables*...> variablePointers;
   
       Eigen::Matrix<ScalarType, ResidualDimension, 1> residual;
   
       template <typename... Variables>
       void updateVariablePointers(VariableContainer<Variables...> &variableContainer)
       {
           internal::static_for(variableKeys, [&](auto i, auto &variableKey) {
               auto& variableMap = variableContainer.template getVariableMap<typename std::tuple_element<i, std::tuple<IndependentVariables...>>::type>();
               auto variableIterator = variableMap.at(variableKey);
   
               assert(variableIterator != variableMap.end());
   
               std::get<i>(variablePointers) = &(*(variableIterator));
           });
       }
   
       template <typename... Variables>
       bool checkVariablePointerConsistency(VariableContainer<Variables...> &variableContainer)
       {
           bool flag = true;
           internal::static_for(variableKeys, [&](auto i, auto &variableKey) {
               auto& variableMap = variableContainer.template getVariableMap<typename std::tuple_element<i, std::tuple<IndependentVariables...>>::type>();
               auto variableIterator = variableMap.at(variableKey);
   
               assert(variableIterator != variableMap.end());
   
               // The pointers should match.
               if (std::get<i>(variablePointers) != &(*(variableIterator)))
               {
                   flag = false;
               }
           });
   
           return flag;
       }
   };
   
   } // namespace ArgMin
