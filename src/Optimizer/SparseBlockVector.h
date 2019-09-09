#pragma once

#include "MetaHelpers.h"
#include "SparseBlockMatrix.h"
#include "SubMatrix.h"

namespace LittleOptimizer::SparseBlockVector {

namespace internal {
    template <typename T>
    bool add_vector_of_sub_matrices(std::vector<T>& lhsVector, std::vector<T>& rhsVector) {
        assert(lhsVector.size() == rhsVector.size());

        for (size_t idx = 0; idx < lhsVector.size(); ++idx) {
            //lhsVector.at(idx) += rhsVector.at(idx);
        }

        return true;
    }

    template <typename T, size_t...Is>
    void add_sparse_block_vectors(T& lhsVector, T& rhsVector, std::integer_sequence<size_t, Is...>) {
        auto l = {add_vector_of_sub_matrices(std::get<Is>(lhsVector), std::get<Is>(rhsVector))...};
        (void)l;
    }
}

template <typename... Ts>
class SparseBlockVector;

template <typename ScalarType, typename... Variables>
class SparseBlockVector<Scalar<ScalarType>, VariableGroup<Variables...>> {

private:
    using variable_list = std::tuple<Variables...>;
    static_assert(sizeof...(Variables) > 0); // ensure that there is at least on variable.

    template <typename VariableType>
    using variable_index = LittleOptimizer::internal::Index<VariableType, variable_list>;

    typedef std::tuple<std::vector<SubMatrix<ScalarType, Variables::dimension, 1>>...> vector_t;
    vector_t vector;

public:

    SparseBlockVector() {

    }

    template <typename RowType>
    size_t rowsOfType() {
        std::get<variable_index<RowType>::value>(vector).size();
    }

    template <typename RowType>
    SubMatrix<ScalarType, RowType::dimension, 1> get(TypedIndex<RowType> row) {
        return std::get<variable_index<RowType>::value>(vector).at(row.index);
    }

    template <typename RowType>
    void addRow() {
        std::get<variable_index<RowType>::value>(vector).emplace_back();
    }

    SparseBlockVector<Scalar<ScalarType>, VariableGroup<Variables...>>& operator+=(SparseBlockVector<Scalar<ScalarType>, VariableGroup<Variables...>>& rhs){
        internal::add_sparse_block_vectors(vector, rhs.vector, std::index_sequence_for<Variables...>{});
        return *this;
    }
};

}