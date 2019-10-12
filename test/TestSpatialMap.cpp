#include "gtest/gtest.h"
#include "RadialSearchPattern.h"
#include "SpatialMap.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

TEST(SpatialMap, Basic)
{
    struct RandomType {
        double* data = nullptr;
    };

    QDVO::SpatialMap<RandomType> map(512);

    for (int i = 0; i < 10; ++i)
    {
        TIK
        for (int x = 0; x < 512; ++x)
        {
            for (int y = 0; y < 512; ++y)
            {
                map.get(Eigen::Vector2i(x, y));
            }
        }
        TOK
    }
}
