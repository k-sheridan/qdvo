#include "gtest/gtest.h"
#include "STLAlgorithmAbstraction.h"
#include <random>
#include "GlobalDefinitions.h"

TEST(STLAlgorithmAbstraction, parallel_transform)
{
    std::vector<int> numbers(500);
    //std::generate(numbers.begin(), numbers.end(), std::rand);
    std::iota(numbers.begin(), numbers.end(), 1);
    //std::fill(numbers.begin(), numbers.end(), 10);

    std::vector<int> result(numbers.size());

    const int wasteIter = 100000;

    auto fn = [](int a) -> int{for (int i = 0; i < wasteIter; ++i){a = std::max(a, i);} return a + 10; };

    TIK
    QDVO::STLAlgorithmAbstraction::transform(QDVO::STLAlgorithmAbstraction::ExecutionType::SEQUENTIAL, numbers.begin(), numbers.end(),result.begin(), fn);
    TOK
    std::vector<int> result2(numbers.size());
    RETIK
    QDVO::STLAlgorithmAbstraction::transform(QDVO::STLAlgorithmAbstraction::ExecutionType::PARALLEL_CPU, numbers.begin(), numbers.end(),result2.begin(), fn);
    RETOK
    for (int i = 0; i < result.size(); ++i)
    {
        //std::cout << result.at(i) << std::endl;
        EXPECT_EQ(result.at(i), result2.at(i));
    }
}
