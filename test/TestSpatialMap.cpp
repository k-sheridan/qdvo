#include "gtest/gtest.h"
#include "DataStructures/RadialSearchPattern.h"
#include "DataStructures/SpatialMap.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

TEST(SpatialMap, Basic)
{
    struct RandomType {
        double* data = nullptr;
        void reset(){}
    };

    QDVO::SpatialMap<RandomType> map(512);

    

    map.reset();
}
