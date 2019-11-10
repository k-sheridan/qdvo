#pragma once

#include "Containers.h"
#include "MetaHelpers.h"
#include "GaussianPrior"
#include "SlotMap.h"
#include "SlotArray.h"

namespace ArgMin {

template <typename... T>
class PSDLinearSystem;

template <typename ScalarType, typename... ErrorTerms, typename... Variables>
class PSDLinearSystem<Scalar<ScalarType>, ErrorTermGroup<ErrorTerms...>, VariableGroup<Variables...>> {
public:

template <typename VariableType>
using BVector = SlotArray<VariableKey<VariableType>, Eigen::Matrix<ScalarType, Eigen::Dynamic, VariableType::dimension>;
template <typename VariableType>
using DBlock = Eigen::Matrix<ScalarType, VariableType::dimension, VariableType::dimension>;
template <typename VariableType>
using DVector = SlotArray<VariableKey<VariableType>, DBlock<VariableType>>;


Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> A; // Dense matrix in the upper right corner
std::tuple<BVector<Variables>>...> B; // Top right block matrix. Equal to bottom left transposed.
std::tuple<DVector<Variables>>...> D; // Block diagonal matrix in the bottom right. Each block is invertible.


};

} // namespace ArgMin