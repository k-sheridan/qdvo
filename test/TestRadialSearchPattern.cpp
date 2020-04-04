#include "DataStructures/RadialSearchPattern.h"
#include "TestFixtures.h"
#include "gtest/gtest.h"

TEST(RadialSearchPattern, Basic) {
  constexpr int searchRadius = 100;
  QDVO::RadialSearchPattern pattern(searchRadius);

  EXPECT_EQ(pattern.searchPattern.size(), searchRadius + 1);

  Eigen::MatrixXd render(2 * (searchRadius + 1), 2 * (searchRadius + 1));
  render.setConstant(-1);

  int radius = 0;
  for (auto& e : pattern.searchPattern) {
    for (auto& f : e) {
      auto& renderPx = render(f(1) + searchRadius + 1, f(0) + searchRadius + 1);
      EXPECT_EQ(renderPx, -1);
      renderPx = radius;
    }
    radius++;
  }

  for (int row = 0; row < render.rows(); ++row) {
    for (int col = 0; col < render.cols(); ++col) {
      auto delta = Eigen::Vector2i(row, col) -
                   Eigen::Vector2i(searchRadius + 1, searchRadius + 1);

      int radius = std::round(delta.norm());
      if (radius < searchRadius) {
        EXPECT_NEAR(radius, render(row, col), 1);
      }
    }
  }
}
