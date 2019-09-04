#pragma once

#include <tuple>
#include <vector>
 
#include "SparseBlockMatrix.h"

namespace LittleOptimizer {

template <typename... T>
struct VariableGroup {};

template <typename... T>
struct ErrorTermGroup {};

template <typename... T>
class Optimizer;

template <typename... Variables, typename... ErrorTerms>
class Optimizer<VariableGroup<Variables...>, ErrorTermGroup<ErrorTerms...>> 
{
    typedef std::tuple<std::vector<Variables>...> variableVectors; // Tuple of vectors of variables.

    typedef std::tuple<std::vector<ErrorTerms>...> errorTermVectors; // Tuple of vectors of error terms.

    //SparseBlockMatrix::SparseBlockMatrix<SparseBlockMatrix::Scalar<double>, SparseBlockMatrix::VariableGroup<Variables...>> priorA;

};

} // namespace LittleOptimizer