#include <Eigen/Core>

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

TEST(SpatialMap, Hashing) {
  struct Example {
    double data = -1;
    void reset() {}
  };

  QDVO::SpatialMap<Example> map(512);

  std::vector<std::vector<bool>> table;

  for (int x = 0; x < 512; ++x) {
    for (int y = 0; y < 512; ++y) {
      auto topHash = map.topHash(x, y);
      if (topHash >= table.size()) {
        table.resize(topHash + 1);
      }
      auto& subtable = table.at(topHash);
      auto bottomHash = map.bottomHash(x, y);

      if (bottomHash >= subtable.size()) {
        subtable.resize(bottomHash + 1, false);
      }
      EXPECT_FALSE(subtable.at(bottomHash));
      subtable.at(bottomHash) = true;
    }
  }
}
