#pragma once

#include "Types.h"
#include "SlotMap.h"

namespace ArgMin
{

template <typename T>
class VariableKey : public SlotMapKeyBase
{
public:
    typedef T variable_type;
    // Slot Map key (index, generation).
    bool operator<(const VariableKey<T> &other) const
    {
        return this->index < other.index;
    }

    /// Compares two keys by their index, but not generation.
    bool operator==(const VariableKey<T> &other) const
    {
        return this->index == other.index;
    }
};

template <typename T>
class ErrorTermKey : public SlotMapKeyBase
{
public:
    typedef T errorterm_type;
    // Slot Map key (index, generation).
    bool operator<(const ErrorTermKey<T> &other) const
    {
        return this->index < other.index;
    }
};

} // namespace ArgMin