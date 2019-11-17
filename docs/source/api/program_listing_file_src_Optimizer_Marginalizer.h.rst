
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
   
   namespace ArgMin {
   
   template <typename... T>
   class Marginalizer;
   
   template <typename ScalarType, typename... Variables, typename... ErrorTerms>
   class Marginalizer<Scalar<ScalarType>, VariableGroup<Variables...>, ErrorTermGroup<ErrorTerms...>> {
   
   public:
   
       Marginalizer() {}
   
       template <typename VariableType>
       void marginalizeVariable(VariableKey<VariableType>& marginalizedKey, GaussianPrior<Scalar<ScalarType>, VariableGroup<Variables...>> ErrorTermContainer<ErrorTerms...>& linearizedErrorTerms)
       {
   
       }
   
   }
   
   };
