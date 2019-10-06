#pragma once

#include "MetaHelpers.h"
#include "Key.h"
#include "Containers.h"

namespace ArgMin
{

template <typename...>
class SSEOptimizer;

template <typename... Variables, typename... ErrorTerms>
class SSEOptimizer<VariableGroup<Variables...>, ErrorTermGroup<ErrorTerms...>>
{
public:

    template <typename VariableType>
    VariableKey<VariableType> addVariable(VariableType& optimizableVariable) 
    {
        //variables.totalDimensions();
        //return variables.template getVariableMap<Variable>().insert(optimizableVariable);
        //return VariableKey<VariableType>();
    }

private:
    /// Containers for both variables and error terms.
    VariableContainer<Variables...> variables;
    ErrorTermContainer<ErrorTerms...> errorTerms;
};

} //namespace ArgMin