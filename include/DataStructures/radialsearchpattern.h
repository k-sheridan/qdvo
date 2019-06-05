#ifndef RADIALSEARCHPATTERN_H
#define RADIALSEARCHPATTERN_H

#include <opencv2/core.hpp>

class RadialSearchPattern {

public:
    /*
     * Constructs a radial search pattern which allows for a rapid search for nearest neighbors.
     */
    RadialSearchPattern(const unsigned maxRadius)
    {
        unsigned dim = maxRadius;
        if (maxRadius % 2 == 0){dim++;} // ensure that the center of the array is an integer.

        cv::Mat mask = cv::Mat::zeros(dim, dim, CV_8U);

    }

    RadialSearchPattern(){}

    std::vector<std::vector<cv::Point2i> > searchPattern; // index directly correlates to the delta indices which are to be used for a given radius.
};

#endif // RADIALSEARCHPATTERN_H
