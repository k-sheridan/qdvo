#pragma once

#include <opencv4/opencv2/core.hpp>
#include <GlobalDefinitions.h>

namespace  QDVO {

class Feature
{
public:
    Feature();

    cv::Point_<SCALAR_TYPE> px;
};

}
