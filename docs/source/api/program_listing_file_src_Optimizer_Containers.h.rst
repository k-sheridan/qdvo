
.. _program_listing_file_src_Optimizer_Containers.h:

Program Listing for File Containers.h
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_Containers.h>` (``src/Optimizer/Containers.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "MetaHelpers.h"
   #include "SlotMap.h"
   #include "Key.h"
   
   namespace ArgMin
   {
   
   template <typename... Variables>
   class VariableContainer
   {
   public:
      template <typename VariableType>
      using VariableMap = SlotMap<VariableType, VariableKey<VariableType>>;
   
      template <typename VariableType>
      VariableMap<VariableType> &getVariableMap()
      {
         return std::get<VariableMap<VariableType>>(tupleOfVariableMaps);
      }
   
      template <typename VariableType>
      size_t variableIndex(VariableKey<VariableType> &key)
      {
         size_t variableIndex = 0;
         static const int tupleIndexOfVariable = internal::Index<VariableType, std::tuple<Variables...>>::value;
   
         internal::static_for(tupleOfVariableMaps, [&](auto i, auto &v) {
            if constexpr (i < tupleIndexOfVariable)
            {
               variableIndex += v.size() * std::tuple_element<i, std::tuple<Variables...>>::type::dimension;
            }
   
            if constexpr (i == tupleIndexOfVariable)
            {
               auto it = v.at(key);
               assert(it != v.end());
   
               variableIndex += (it - v.begin()) * std::tuple_element<i, std::tuple<Variables...>>::type::dimension;
            }
   
         });
   
         return variableIndex;
      }
   
      size_t totalDimensions()
      {
         size_t dims = 0;
         internal::static_for(tupleOfVariableMaps, [&](auto i, auto &v) { dims += v.size() * std::tuple_element<i, std::tuple<Variables...>>::type::dimension; });
         return dims;
      }
   
      // Stores the variables.
      std::tuple<VariableMap<Variables>...> tupleOfVariableMaps;
   };
   
   template <typename... ErrorTerms>
   class ErrorTermContainer
   {
   public:
      template <typename ErrorTermType>
      using ErrorTermMap = SlotMap<ErrorTermType, ErrorTermKey<ErrorTermType>>;
   
      template <typename ErrorTermType>
      ErrorTermMap<ErrorTermType> &getErrorTermMap()
      {
         return std::get<ErrorTermMap<ErrorTermType>>(tupleOfErrorTermMaps);
      }
   
      void linearize() {
         assert(false && "not ready");
      }
   
      // Stores the error terms.
      std::tuple<ErrorTermMap<ErrorTerms>...> tupleOfErrorTermMaps;
   };
   
   } // namespace ArgMin
