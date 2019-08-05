#pragma once

#include "GlobalDefinitions.h"
#include "Image.h"

namespace QDVO {
class ImagePyramid
{
public:
    /**
     * creates a new image pyramid from a base image.
     */
    ImagePyramid(int levels = 1);

    QDVO::Image& getImage(const size_t level = 0);

    void generate(cv::Mat& baseImage);

    size_t levels() const {return this->imageLevels.size();}

protected:
    std::vector<QDVO::Image> imageLevels; // level 0 is full resolution.

};
}

