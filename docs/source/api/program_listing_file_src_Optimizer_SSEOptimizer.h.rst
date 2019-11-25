
.. _program_listing_file_src_Optimizer_SSEOptimizer.h:

Program Listing for File SSEOptimizer.h
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_SSEOptimizer.h>` (``src/Optimizer/SSEOptimizer.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "MetaHelpers.h"
   #include "Key.h"
   #include "Containers.h"
   #include "GaussianPrior.h"
   
   namespace ArgMin
   {
   
   template <typename...>
   class SSEOptimizer;
   
   template <typename ScalarType, typename... Variables, typename... ErrorTerms>
   class SSEOptimizer<Scalar<ScalarType>, VariableGroup<Variables...>, ErrorTermGroup<ErrorTerms...>>
   {
   public:
       template <typename VariableType>
       VariableKey<VariableType> addVariable(VariableType &optimizableVariable)
       {
           return variables.template getVariableMap<VariableType>().insert(optimizableVariable);
       }
   
       // Updates the marginal information of the given variable.
       template <typename VariableType>
       void setVariablePrior(VariableKey<VariableType> &variableKey, Eigen::Matrix<ScalarType, VariableType::dimension, VariableType::dimension> &covariance)
       {
           assert(false);
       }
   
       template <typename VariableType>
       void marginalizeVariable(VariableKey<VariableType>& key)
       {
           assert(false);
       }
   
       template <typename VariableType>
       void removeVariable(VariableKey<VariableType>& key)
       {
           variables.template getVariableMap<VariableType>().erase(key);
   
           // TODO erase the variables from the sparse prior.
       }
   
       template <typename ErrorTermType>
       ErrorTermKey<ErrorTermType> addErrorTerm(ErrorTermType& errorTerm)
       {
           assert(false);
       }
   
       template <typename ErrorTermType>
       void removeErrorTerm(ErrorTermKey<ErrorTermType>& errorTermKey)
       {
           assert(false);
       }
   
       void optimize()
       {
           assert(false);
       }
   
       VariableContainer<Variables...> variables;
       ErrorTermContainer<ErrorTerms...> errorTerms;
   
       GaussianPrior<Scalar<ScalarType>, VariableGroup<Variables...>> prior;
   };
   
   } //namespace ArgMin
