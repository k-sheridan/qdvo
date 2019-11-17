#pragma once

#include <Eigen/Core>
#include "Optimizer/Key.h"
#include "Optimizer/Containers.h"
#include "Optimizer/MetaHelpers.h"

namespace ArgMin
{

template <typename...>
class ErrorTermBase;

/**
 * This serves as the base of a simple error term with only a handful of independent variables.
 * This base stores the linearized error term and the pointers to to extract the variables.
 */
template <int ResidualDimension, typename ScalarType, typename... IndependentVariables>
class ErrorTermBase<Scalar<ScalarType>, Dimension<ResidualDimension>, VariableGroup<IndependentVariables...>>
{
public:

    std::tuple<Eigen::Matrix<ScalarType, ResidualDimension, IndependentVariables::dimension>...> variableJacobians;
    std::tuple<VariableKey<IndependentVariables>...> variableKeys;
    std::tuple<IndependentVariables*...> variablePointers;

    Eigen::Matrix<ScalarType, ResidualDimension, 1> residual;

    /// Extracts the most recent pointer to each of the variables using their key and updates them.
    template <typename... Variables>
    void updateVariablePointers(VariableContainer<Variables...> &variableContainer)
    {
        internal::static_for(variableKeys, [&](auto i, auto &variableKey) {
            auto& variableMap = variableContainer.template getVariableMap<typename std::tuple_element<i, std::tuple<IndependentVariables...>>::type>();
            auto variableIterator = variableMap.at(variableKey);

            assert(variableIterator != variableMap.end());

            std::get<i>(variablePointers) = &(*(variableIterator));
        });
    }

    /// Verifies that the pointers stored are equal to the true variable location.
    template <typename... Variables>
    bool checkVariablePointerConsistency(VariableContainer<Variables...> &variableContainer)
    {
        bool flag = true;
        internal::static_for(variableKeys, [&](auto i, auto &variableKey) {
            auto& variableMap = variableContainer.template getVariableMap<typename std::tuple_element<i, std::tuple<IndependentVariables...>>::type>();
            auto variableIterator = variableMap.at(variableKey);

            assert(variableIterator != variableMap.end());

            // The pointers should match.
            if (std::get<i>(variablePointers) != &(*(variableIterator)))
            {
                flag = false;
            }
        });

        return flag;
    }
};

} // namespace ArgMin