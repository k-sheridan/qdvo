#ifndef GENERICQUADTREE_H
#define GENERICQUADTREE_H

#include <vector>
#include <Eigen/Core>
#include <iostream>

namespace QDVO {

template <typename T>
class GenericQuadTree {

public:
    struct Quad
    {
        Quad* parentQuad = nullptr; // pointer to this quad's parent. If null, this is the root.

        // all children quadrants.
        std::unique_ptr<Quad> quadrant1;
        std::unique_ptr<Quad> quadrant2;
        std::unique_ptr<Quad> quadrant3;
        std::unique_ptr<Quad> quadrant4;

        unsigned nChildren = 0; // stores the total number of leafs (T) in all children quads.

        std::unique_ptr<std::vector<T*> > data = nullptr; // stores pointers to data within this quad.

        Eigen::Vector2i center, topLeft, bottomRight; // center point of quad, and dimensions of the quad.
    };

    // default constructor
    GenericQuadTree();
    /*
     * Constructs a quadtree with the desired size.
     */
    GenericQuadTree(unsigned width, unsigned height);


    std::unique_ptr<Quad> root;

};

template <typename T> GenericQuadTree<T>::GenericQuadTree(){}

template <typename T> GenericQuadTree<T>::GenericQuadTree(unsigned width, unsigned height)
{
    this->root = std::unique_ptr<Quad>(new Quad());
}

}

#endif // GENERICQUADTREE_H
