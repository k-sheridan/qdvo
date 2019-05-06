#include "ImagePyramid.h"
#include <cmath>

QDVO::ImagePyramid::ImagePyramid(const cv::Mat& baseImage, int levels)
{
    // deallocate any old images. Should be empty though.
    this->imageLevels.clear();

    // make sure that we can create enough levels with the current image size.
    const int level0Rows = baseImage.rows;
    const int level0Cols = baseImage.cols;

    if ((level0Rows % int(std::pow(2, levels))) || (level0Cols % int(std::pow(2, levels))))
    {
        throw std::runtime_error("Image size not able to be reduced to the specified pyramid level.");
    }

    this->imageLevels.resize(levels); // create the containers.
    this->imageLevels.at(0) = baseImage;

    for(int i = 1; i < levels; ++i)
    {
        //TODO replace this with a custom version. (no need for the gaussian down sample.)
        cv::pyrDown(this->imageLevels.at(i-1), this->imageLevels.at(i), cv::Size(this->imageLevels.at(i-1).cols/2, this->imageLevels.at(i-1).rows/2));
    }

}

cv::Mat& QDVO::ImagePyramid::getImage(const int level)
{
    return this->imageLevels.at(level-1);
}
