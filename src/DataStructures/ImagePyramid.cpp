#include "ImagePyramid.h"
#include <cmath>

QDVO::ImagePyramid::ImagePyramid(int levels)
{
    // deallocate any old images. Should be empty though.
    this->imageLevels.clear();

    this->imageLevels.resize(levels); // create the containers.

}

void QDVO::ImagePyramid::generate(cv::Mat& baseImage)
{
    // make sure that we can create enough levels with the current image size.
    const int level0Rows = baseImage.rows;
    const int level0Cols = baseImage.cols;
    const int levels = this->imageLevels.size();

    if ((level0Rows % int(std::pow(2, levels))) || (level0Cols % int(std::pow(2, levels))))
    {
        throw std::runtime_error("Image size not able to be reduced to the specified pyramid level.");
    }


    this->imageLevels.at(0) = QDVO::Image(baseImage);

    assert(this->imageLevels.size() == 1); // not supported yet.

    /*for(size_t i = 1; i < this->imageLevels.size(); ++i)
    {
        //TODO replace this with a custom version. (no need for the gaussian down sample.)
        cv::pyrDown(this->imageLevels.at(i-1), this->imageLevels.at(i), cv::Size(this->imageLevels.at(i-1).cols/2, this->imageLevels.at(i-1).rows/2));
    }*/
}

QDVO::Image& QDVO::ImagePyramid::getImage(const size_t level)
{
    return this->imageLevels.at(level);
}
