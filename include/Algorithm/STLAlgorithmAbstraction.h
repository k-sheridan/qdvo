#pragma once

#include <algorithm>
#include <mutex>
#include <thread>
#include <vector>
#include <math.h>
#include "ThreadingHelpers.h"
//#include <thrust>

namespace QDVO::STLAlgorithmAbstraction
{
enum ExecutionType
{
    SEQUENTIAL,
    PARALLEL_CPU,
    PARALLEL_CUDA
};

template <class InputIterator, class OutputIterator, class UnaryOperation>
void transform(ExecutionType execution, InputIterator first, InputIterator last, OutputIterator result, UnaryOperation op)
{
    if (execution == ExecutionType::SEQUENTIAL)
    {
        std::transform(first, last, result, op);
    }
    else if (execution == PARALLEL_CPU)
    {
        auto problemDividers = createListDividers(last - first);
        std::vector<std::thread> threads;
        threads.reserve(problemDividers.size());

        // Launch the threads.
        for (auto &divider : problemDividers)
            threads.emplace_back(std::transform<InputIterator, OutputIterator, UnaryOperation>, first + std::get<0>(divider), first + std::get<1>(divider), result + std::get<0>(divider), op);
        
        joinThreads(threads);
    }
    else if (execution == PARALLEL_CUDA)
    {
        assert(false);
    }
}

} // namespace QDVO::STLAlgorithmAbstraction
