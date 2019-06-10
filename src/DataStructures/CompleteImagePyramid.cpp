#include "CompleteImagePyramid.h"

QDVO::CompleteImagePyramid::CompleteImagePyramid(const int levels, const int standardDeviationTableResolution) : image(levels), dx(levels), dy(levels)
{

    this->standardDeviationTable = cv::Mat(standardDeviationTableResolution, standardDeviationTableResolution, CV_32F);

    //this->image = QDVO::ImagePyramid(levels);

    //this->dx = QDVO::ImagePyramid(levels);
    //this->dy = QDVO::ImagePyramid(levels);

}

void QDVO::CompleteImagePyramid::generate(const cv::Mat& baseImage)
{
    this->image.generate(baseImage);

    // compute the image gradients
    cv::Sobel(baseImage, this->dx.getImage(), CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    this->dx.generate(this->dx.getImage());

    cv::Sobel(baseImage, this->dy.getImage(), CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
    this->dy.generate(this->dy.getImage());
}
