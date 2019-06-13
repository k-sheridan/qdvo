#ifndef RADIALSEARCHPATTERN_H
#define RADIALSEARCHPATTERN_H

#include <opencv2/core.hpp>
#include <GlobalDefinitions.h>
#include <Eigen/Core>
#include <iostream>

namespace QDVO {

class RadialSearchPattern {

public:
    /*
     * Constructs a radial search pattern which allows for a rapid search for nearest neighbors.
     * Very expensive and meant to be only ran once.
     */
    RadialSearchPattern(const unsigned maxRadius);

    RadialSearchPattern(){}

    std::vector<std::vector<Eigen::Vector2i> > searchPattern; // index directly correlates to the delta indices which are to be used for a given radius.
};
}

#endif // RADIALSEARCHPATTERN_H
