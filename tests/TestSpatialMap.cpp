#include "gtest/gtest.h"
#include <RadialSearchPattern.h>
#include <SpatialMap.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

TEST(SpatialMap, Basic)
{
    QDVO::SpatialMap<double> map(512);

    std::cout << map.mapOfMaps.size() << std::endl;
    //std::cout << map.get(Eigen::Vector2i(-255, 0)) << std::endl;


    for (int i = 0; i < 1000000; ++i)
    {
        map.get(Eigen::Vector2i(10, 0));
        map.get(Eigen::Vector2i(250, 0));
        map.get(Eigen::Vector2i(10, 250));

        map.get(Eigen::Vector2i(-40, 0));
        map.get(Eigen::Vector2i(-250, 0));
        map.get(Eigen::Vector2i(10, -250));

        map.get(Eigen::Vector2i(-40, -40));
        map.get(Eigen::Vector2i(-250, -250));
        map.get(Eigen::Vector2i(250, 250));
        map.get(Eigen::Vector2i(-250, 250));
    }
}
