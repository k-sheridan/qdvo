
.. _program_listing_file_src_Optimizer_Marginalizer.h:

Program Listing for File Marginalizer.h
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_Marginalizer.h>` (``src/Optimizer/Marginalizer.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "Optimizer/GaussianPrior.h"
   #include "Optimizer/Key.h"
   #include "Optimizer/MetaHelpers.h"
   #include "Optimizer/Containers.h"
   #include "ErrorTermBase.h"
   #include <type_traits>
   
   namespace ArgMin {
   
   template <typename... T>
   class Marginalizer;
   
   template <typename ScalarType, typename... Variables, typename... ErrorTerms>
   class Marginalizer<Scalar<ScalarType>, VariableGroup<Variables...>, ErrorTermGroup<ErrorTerms...>> {
   
   public:
   
       template <typename RowVariable, typename ColumnVariable>
       using MatrixArray = SlotArray<Eigen::Matrix<ScalarType, RowVariable::dimension, ColumnVariable::dimension>, VariableKey<ColumnVariable>>;
   
       template <typename RowVariable>
       using Row = std::tuple<MatrixArray<RowVariable, Variables>...>;
   
       Marginalizer() {}
   
       template <typename VariableType, typename... IgnoredVariables>
       void marginalizeVariable(VariableKey<VariableType>& marginalizedKey, GaussianPrior<Scalar<ScalarType>, VariableGroup<Variables...>>& prior, ErrorTermContainer<ErrorTerms...>& linearizedErrorTerms, VariableGroup<IgnoredVariables...> ignoredVariableTypes = VariableGroup<IgnoredVariables...>())
       {
   
           // Iterate through all error terms.
           internal::static_for(linearizedErrorTerms.tupleOfErrorTermMaps, [&](auto i, auto &errorTermMap) {
               typedef typename std::tuple_element<i, std::tuple<ErrorTerms...>>::type ThisErrorTerm;
   
               // A vector of error term keys which will be removed.
               std::vector<ErrorTermKey<ThisErrorTerm>> errorTermsToRemove;
   
               // Check if this error term is a function of the marginalized variable type.
               if constexpr(internal::Is_in_tuple<VariableKey<typename decltype(marginalizedKey)::variable_type>, typename ThisErrorTerm::VariableKeys>::value)
               {   
                   // This error term type contains the same key type as the marginalized key.
                   for (auto& errorTerm : errorTermMap)
                   {
                       // Runtime check if the marginalized variable key is in this error term.
                       bool errorTermContainsMarginalizedVariable = false;
                       internal::static_for(errorTerm.variableKeys, [&](auto i, auto &key) {
                           if (key == marginalizedKey)
                           {
                               errorTermContainsMarginalizedVariable = true;
                           }
                       });
   
                       // If this error term is a function of the marginalized variable, marginalize it.
                       if (errorTermContainsMarginalizedVariable)
                       {
   
                       }
   
                   }
               }
           });
       }
   
   };
   
   }
