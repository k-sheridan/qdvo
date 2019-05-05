#pragma once

#include <opencv4/opencv2/core.hpp>
#include <GlobalDefinitions.h>

class Feature
{
public:
    Feature();

    cv::Point_<SCALAR_TYPE> px;
};

