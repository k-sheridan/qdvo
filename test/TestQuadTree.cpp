#include <Eigen/Core>

#include "DataStructures/GenericQuadTree.h"
#include "gtest/gtest.h"

TEST(QuadTree, Basic) {
  struct Foo {
    double bar;
    double bing;
  };

  QDVO::GenericQuadTree<Foo> tree(512, 513);
  // QDVO::GenericQuadTree<int> tree;

  ASSERT_EQ(tree.root->quadrants.at(0).get(), nullptr);
}

TEST(QuadTree, Insert) {
  struct Foo {
    double bar;
    double bing;
  };

  QDVO::GenericQuadTree<Foo*> tree(512, 512);

  Foo* garbage = new Foo();
  garbage->bar = 1090;
  garbage->bing = 111;

  Foo* moreGarbage = new Foo();
  moreGarbage->bar = 109;

  tree.insert(Eigen::Vector2i(21, 21), garbage);
  tree.insert(Eigen::Vector2i(20, 21), moreGarbage);

  delete garbage;
  delete moreGarbage;
}

TEST(QuadTree, Delete) {
  struct Foo {
    double bar;
    double bing;
  };

  QDVO::GenericQuadTree<Foo*> tree(512, 512);

  Foo* garbage = new Foo();
  garbage->bar = 1090;
  garbage->bing = 111;

  Foo* moreGarbage = new Foo();
  moreGarbage->bar = 109;

  tree.insert(Eigen::Vector2i(21, 21), garbage);
  tree.insert(Eigen::Vector2i(20, 21), moreGarbage);

  tree.clear();

  ASSERT_EQ(tree.root->getQuad(Eigen::Vector2i(21, 21))->level, 0);

  delete garbage;
  delete moreGarbage;
}

TEST(QuadTree, ApproximateNearestNeigbors) {
  QDVO::GenericQuadTree<Eigen::Vector2d> tree(512, 512);

  auto pt1 = Eigen::Vector2d(1, 1);
  auto pt2 = Eigen::Vector2d(10, 1);
  auto pt3 = Eigen::Vector2d(511, 1);
  auto pt4 = Eigen::Vector2d(511, 500);
  auto pt5 = Eigen::Vector2d(50, 499);

  tree.insert(pt1.cast<int>(), pt1);
  tree.insert(pt2.cast<int>(), pt2);
  tree.insert(pt3.cast<int>(), pt3);
  tree.insert(pt4.cast<int>(), pt4);
  tree.insert(pt5.cast<int>(), pt5);

  std::vector<Eigen::Vector2d> children;
  EXPECT_EQ(tree.root->nChildren, 5);
  tree.root->getChildren(children);
  EXPECT_EQ(children.size(), 5);

  EXPECT_EQ(tree.getLowestQuad(pt1.cast<int>()).nChildren, 0);

  auto neighbors = tree.getApproximateNeighbors(pt1.cast<int>(), 2);
  EXPECT_EQ(neighbors.size(), 2);
  EXPECT_LE((neighbors.at(0) - pt1).norm(), 20);
  EXPECT_LE((neighbors.at(1) - pt1).norm(), 20);

  neighbors = tree.getApproximateNeighbors(pt1.cast<int>(), 5);
  EXPECT_EQ(neighbors.size(), 5);

  neighbors = tree.getApproximateNeighbors(pt4.cast<int>(), 1);
  EXPECT_EQ(neighbors.size(), 1);
  EXPECT_EQ(neighbors.at(0)(0), 511);
  EXPECT_EQ(neighbors.at(0)(1), 500);
}

TEST(QuadTree, NearestNeigbors) {
  QDVO::GenericQuadTree<Eigen::Vector2d> tree(512, 512);

  auto pt1 = Eigen::Vector2d(1, 1);
  auto pt2 = Eigen::Vector2d(10, 1);
  auto pt3 = Eigen::Vector2d(511, 1);
  auto pt4 = Eigen::Vector2d(511, 500);
  auto pt5 = Eigen::Vector2d(50, 499);

  tree.insert(pt1.cast<int>(), pt1);
  tree.insert(pt2.cast<int>(), pt2);
  tree.insert(pt3.cast<int>(), pt3);
  tree.insert(pt4.cast<int>(), pt4);
  tree.insert(pt5.cast<int>(), pt5);

  std::vector<Eigen::Vector2d> children;
  EXPECT_EQ(tree.root->nChildren, 5);
  tree.root->getChildren(children);
  EXPECT_EQ(children.size(), 5);

  EXPECT_EQ(tree.getLowestQuad(pt1.cast<int>()).nChildren, 0);

  auto neighbors = tree.getNeighbors(pt1.cast<int>(), 3, 2);
  EXPECT_EQ(neighbors.size(), 2);
  EXPECT_LE((neighbors.at(0) - pt1).norm(), 20);
  EXPECT_LE((neighbors.at(1) - pt1).norm(), 20);

  neighbors = tree.getNeighbors(pt1.cast<int>(), 2, 5);
  EXPECT_EQ(neighbors.size(), 5);

  neighbors = tree.getNeighbors(pt4.cast<int>(), 3, 1);
  EXPECT_EQ(neighbors.size(), 1);
  EXPECT_EQ(neighbors.at(0)(0), 511);
  EXPECT_EQ(neighbors.at(0)(1), 500);
}
