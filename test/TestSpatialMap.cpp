#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "DataStructures/RadialSearchPattern.h"
#include "DataStructures/SpatialMap.h"
#include "gtest/gtest.h"

TEST(SpatialMap, Basic) {
  struct Example {
    double data = -1;
    void reset() {}
  };

  QDVO::SpatialMap<Example> map(512);

  // Insert a few points.
  map.get(Eigen::Vector2i(0, 0)) = {1};
  map.get(Eigen::Vector2i(511, 0)) = {2};
  map.get(Eigen::Vector2i(0, 511)) = {3};
  map.get(Eigen::Vector2i(511, 511)) = {4};

  EXPECT_EQ(map.get(Eigen::Vector2i(0, 0)).data, 1);
  EXPECT_EQ(map.get(Eigen::Vector2i(511, 0)).data, 2);
  EXPECT_EQ(map.get(Eigen::Vector2i(0, 511)).data, 3);
  EXPECT_EQ(map.get(Eigen::Vector2i(511, 511)).data, 4);

  // Sanity check.
  EXPECT_EQ(map.get(Eigen::Vector2i(1, 0)).data, -1);
  EXPECT_EQ(map.get(Eigen::Vector2i(0, 1)).data, -1);
  EXPECT_EQ(map.get(Eigen::Vector2i(1, 1)).data, -1);

  for (int i = 0; i < 512; ++i) {
    for (int j = 0; j < 512; ++j) {
      if (!(i == 0 || j == 0 || i == 511 || j == 511)) {
        EXPECT_EQ(map.get(Eigen::Vector2i(j, i)).data, -1);
      }
    }
  }
}
