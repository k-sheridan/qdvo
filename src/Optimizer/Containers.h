#pragma once

#include "MetaHelpers.h"
#include "SlotMap.h"
#include "Key.h"

namespace ArgMin
{

template <typename... Variables>
class VariableContainer
{

   /// Gets the map container for a variable type.
   template <typename VariableType>
   auto &getVariableMap()
   {  
      assert(false);
   }

   /// Gets the index of the first scalar of the given variable.
   /// This is used to build and operate on a matrix.
   template <typename VariableType>
   size_t variableIndex(VariableKey<VariableType>& key)
   {
      assert(false);
   }

   /// Computes the total dimensionality of the variables stored in this container.
   size_t totalDimensions()
   {
      assert(false);
   }

   // Stores the variables.
   std::tuple<SlotMap<Variables, VariableKey<Variables>>...> tupleOfVariableMaps;
};

/**
 * Container for all error terms.
 */
template <typename... ErrorTerms>
class ErrorTermContainer
{
   // Stores the error terms.
   std::tuple<SlotMap<ErrorTerms, ErrorTermKey<ErrorTerms>>...> tupleOfErrorTermMaps;
};

} // namespace ArgMin
