#ifndef GENERICQUADTREE_H
#define GENERICQUADTREE_H

#include <vector>

template <class T>
class GenericQuadTree {

public:
    struct Quad
    {
        Quad* quadrant1 = nullptr;
        Quad* quadrant2 = nullptr;
        Quad* quadrant3 = nullptr;
        Quad* quadrant4 = nullptr;

        unsigned nChildren = 0;

        std::unique_ptr<std::vector<T*> > data = nullptr; // stores the data within this
    };

    // default constructor
    GenericQuadTree(){}
    /*
     * Constructs a quadtree with the desired size.
     */
    GenericQuadTree(unsigned width, unsigned height)
    {

    }

};

#endif // GENERICQUADTREE_H
