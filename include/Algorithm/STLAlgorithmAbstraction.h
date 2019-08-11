#pragma once

#include <algorithm>
#include <mutex>
#include <thread>
#include <vector>
#include <math.h>
//#include <thrust>

#define DEFAULT_THREAD_COUNT 4

namespace QDVO::STLAlgorithmAbstraction
{
enum ExecutionType
{
    SEQUENTIAL,
    PARALLEL_CPU,
    PARALLEL_CUDA
};

// Finds the number of threads on the user system or returns a default thread count.
unsigned getSuggestedThreadCount()
{
    auto nCores = std::thread::hardware_concurrency();
    if (nCores == 0)
    {
        return DEFAULT_THREAD_COUNT;
    }
    else
    {
        return nCores;
    }
}

// Computes a set of uniformly spaced indices s.t. the first index is 0, the last index is (length), and the total number of indices is (nThreads+1).
std::vector<int> splitProblem(unsigned nThreads, unsigned length)
{   
    assert(nThreads > 0);
    std::vector<int> indices(nThreads + 1);

    for (size_t i = 0; i < indices.size(); ++i)
    {
        indices.at(i) = round((float(i) / nThreads) * (length));
    }

    assert(indices.at(0) == 0);
    assert(indices.back() == length);

    return indices;
}

template <class InputIterator, class OutputIterator, class UnaryOperation>
void transform(ExecutionType execution, InputIterator first, InputIterator last, OutputIterator result, UnaryOperation op)
{
    if (execution == ExecutionType::SEQUENTIAL)
    {
        std::transform(first, last, result, op);
    }
    else if (execution == PARALLEL_CPU)
    {
        const auto nCores = getSuggestedThreadCount(); // Get the number of cores on the user's machine.
        std::vector<std::thread> threads;
        threads.reserve(nCores);
        auto splitIndices = splitProblem(nCores, (last - first));

        // Launch the threads.
        for (size_t idx = 0; idx < nCores; ++idx)
        {
            threads.emplace_back(std::transform<InputIterator, OutputIterator, UnaryOperation>, first + splitIndices.at(idx), first + splitIndices.at(idx+1), result + splitIndices.at(idx), op);
        }

        // Join all threads
        for (auto& th : threads)
        {
            th.join();
        }
    }
}

} // namespace QDVO::STLAlgorithmAbstraction
