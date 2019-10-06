#pragma once

#include "Types.h"
#include "SlotMap.h"

namespace ArgMin
{

template <typename T>
class VariableKey : public SlotMapKeyBase<unsigned, unsigned>
{
    typedef T variable_type;
    // Slot Map key (index, generation).
    bool operator<(const VariableKey<T> &other) const 
    {
        return this->index < other.index;
    }
};

template <typename T>
class ErrorTermKey : public SlotMapKeyBase<unsigned, unsigned>
{
    typedef T errorterm_type;
    // Slot Map key (index, generation).
    bool operator<(const ErrorTermKey<T> &other) const
    {
        return this->index < other.index;
    }
};

} // namespace ArgMin