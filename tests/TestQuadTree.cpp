#include "gtest/gtest.h"
#include <GenericQuadTree.h>

TEST(QuadTree, Basic)
{
    struct Foo {
        double bar;
        double bing;
    };

    QDVO::GenericQuadTree<Foo> tree(512, 513);
    //QDVO::GenericQuadTree<int> tree;

    std::cout << tree.root.get() << std::endl;
    std::cout << tree.root->quadrants.at(0).get() << tree.root->quadrants.at(1).get() << tree.root->quadrants.at(2).get() << tree.root->quadrants.at(3).get() << std::endl;
    ASSERT_EQ(tree.root->quadrants.at(0).get(), nullptr);
    std::cout << tree.bottomLevel << std::endl;
    std::cout << tree.root->dimensions << std::endl;

    std::cout << tree.root->getQuad(Eigen::Vector2i(2, 2)) << std::endl;

}


TEST(QuadTree, Insert)
{
    struct Foo {
        double bar;
        double bing;
    };

    QDVO::GenericQuadTree<Foo> tree(512, 512);

    Foo garbage;
    garbage.bar = 1090;
    garbage.bing = 111;

    Foo moreGarbage;
    moreGarbage.bar = 69;

    tree.insert(Eigen::Vector2i(21, 21), &garbage);
    tree.insert(Eigen::Vector2i(20, 21), &moreGarbage);

    std::cout << tree.root->nChildren << std::endl;

    std::cout << tree.root->getQuad(Eigen::Vector2i(20, 21))->data.at(0)->bar << std::endl;

}
