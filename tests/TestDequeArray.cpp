#include "gtest/gtest.h"
#include <RadialSearchPattern.h>
#include <DequeArray.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

TEST(DequeArray, Basic)
{
    QDVO::DequeArray<double> arr(100, 100);

    arr.get(Eigen::Vector2i(10, 10));
}
