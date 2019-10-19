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

   /// Gets the map container for a variable type.
   template <typename VariableType>
   VariableMap<VariableType> &getVariableMap()
   {
      return std::get<VariableMap<VariableType>>(tupleOfVariableMaps);
   }

   /// Gets the index of the first scalar of the given variable.
   /// This is used to build and operate on a matrix.
   template <typename VariableType>
   size_t variableIndex(VariableKey<VariableType> &key) const
   {
      assert(false);
   }

   /// Computes the total dimensionality of the variables stored in this container.
   size_t totalDimensions()
   {
      size_t dims = 0;
      internal::static_for(tupleOfVariableMaps, [&](auto i, auto &v) { dims += v.size() * std::tuple_element<i, std::tuple<Variables...>>::type::dimension; });
      return dims;
   }

private:
   // Stores the variables.
   std::tuple<VariableMap<Variables>...> tupleOfVariableMaps;
};

/**
 * Container for all error terms.
 */
template <typename... ErrorTerms>
class ErrorTermContainer
{
public:
   template <typename ErrorTermType>
   using ErrorTermMap = SlotMap<ErrorTermType, ErrorTermKey<ErrorTermType>>;

   /// Gets the map container for a variable type.
   template <typename ErrorTermType>
   ErrorTermMap<ErrorTermType> &getErrorTermMap()
   {
      return std::get<ErrorTermMap<ErrorTermType>>(tupleOfErrorTermMaps);
   }

private:
   // Stores the error terms.
   std::tuple<ErrorTermMap<ErrorTerms>...> tupleOfErrorTermMaps;
};

} // namespace ArgMin
