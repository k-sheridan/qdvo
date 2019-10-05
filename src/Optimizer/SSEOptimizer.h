#pragma once

#include "MetaHelpers.h"
#include "Key.h"
#include "Containers.h"


namespace ArgMin {

template <typename...>
class SSEOptimizer;

template <typename... Variables, typename... ErrorTerms>
class SSEOptimizer<VariableGroup<Variables...>, ErrorTermGroup<ErrorTerms...>> {

};

} //namespace ArgMin 