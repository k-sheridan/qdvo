#pragma once

#include "Optimizer/SlotMap.h"

namespace ArgMin {
template <typename DataType, typename KeyType,
          typename DataContainer = std::vector<DataType>, bool Direct = false>
using SlotArray = SlotMap<DataType, KeyType, DataContainer, Direct, true>;
}
