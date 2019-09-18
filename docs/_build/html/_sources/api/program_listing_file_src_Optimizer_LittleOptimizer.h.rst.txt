
.. _program_listing_file_src_Optimizer_LittleOptimizer.h:

Program Listing for File LittleOptimizer.h
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_LittleOptimizer.h>` (``src/Optimizer/LittleOptimizer.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include <tuple>
   #include <vector>
    
   #include "PSDLinearSystem.h"
   #include "Key.h"
   #include "slot_map.h"
   
   namespace LittleOptimizer {
   
   template <typename... T>
   class Optimizer;
   
   template <typename... Variables, typename... ErrorTerms>
   class Optimizer<VariableGroup<Variables...>, ErrorTermGroup<ErrorTerms...>> 
   {
       std::tuple<slot_map<Variables>...> variableVectors; // Tuple of vectors of variables.
   
       std::tuple<std::vector<ErrorTerms>...> errorTermVectors; // Tuple of vectors of error terms.
   
       PSDLinearSystem<Scalar<double>, VariableGroup<Variables...>> linearSystem; // Stores A and b for solving.
   
       template <typename VariableType>
       VariableKey<VariableType> addVariable(VariableType var) {
           // Set up the key 
           VariableKey<VariableType> key;
           // Add the variable
           key.slotMapKey = std::get<slot_map<VariableType>>(variableVectors).insert(var);
   
           // Make a spot in the LinearSystems for this variable, and ensure its key is consistent
   
   
           return key;
       }
   
       template <typename ErrorTermType>
       void addErrorTerm(ErrorTermType errorTerm) {
           // Add error term to its vector.
           std::get<std::vector<ErrorTermType>>(errorTermVectors).push_back(errorTerm);
       }
   
   };
   
   } // namespace LittleOptimizer
