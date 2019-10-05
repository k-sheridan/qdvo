#pragma once

#include "MetaHelpers.h"
#include "slot_map.h"

namespace ArgMin {

template <typename ...>
struct VariableContainer;

template <typename ...>
struct ErrorTermContainer;

template <typename... Variables>
struct VariableContainer : public std::tuple<slot_map<Variables>...> {
   /// Gets the index of the first scalar of the given variable. 
   /// This is used to build and operate on a matrix.
   template <typename V>
   size_t variableIndex(VariableKey<V> key) {
      //TODO
   }

   /// Computes the total dimensionality of the variables stored in this container.
   size_t totalDimensions() {
      //TODO
   }
};

/**
 * Container for all error terms.
 */
template <typename... ErrorTerms>
struct ErrorTermContainer : public std::tuple<slot_map<ErrorTerms>...> {

};

} // namespace ArgMin

