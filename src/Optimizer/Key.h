#pragma once

#include "Types.h"
#include "slot_map.h"

namespace ArgMin {

template <typename T>
struct VariableKey : public std::pair<unsigned, unsigned> {
     // Slot Map key (index, generation).
     bool operator< (const VariableKey<T>& other) {
         this->first < other.first;
     } 
};

template <typename T>
struct ErrorTermKey : public std::pair<unsigned, unsigned> {
     // Slot Map key (index, generation).
     bool operator< (const ErrorTermKey<T>& other) {
         this->first < other.first;
     } 
};

} // namespace ArgMin