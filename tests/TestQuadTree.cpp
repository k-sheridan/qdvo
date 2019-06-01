#include "gtest/gtest.h"
#include <GenericQuadTree.h>

TEST(QuadTree, Basic)
{
    struct Foo {
        double bar;
    };

    QDVO::GenericQuadTree<double> tree(512, 512);
    //QDVO::GenericQuadTree<int> tree;

    std::cout << tree.root.get() << std::endl;

}


