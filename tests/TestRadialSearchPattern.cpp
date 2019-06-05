#include "gtest/gtest.h"
#include <RadialSearchPattern.h>

TEST(RadialSearchPattern, Basic)
{
    QDVO::RadialSearchPattern pattern(30);

    std::cout << pattern.searchPattern.size() << std::endl;
}
