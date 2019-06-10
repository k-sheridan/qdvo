#include "CompleteImagePyramid.h"

QDVO::CompleteImagePyramid::CompleteImagePyramid(const int levels, const int standardDeviationTableResolution)
{

    this->standardDeviationTable = cv::Mat(standardDeviationTableResolution, standardDeviationTableResolution, CV_32F);

    this->image = QDVO::ImagePyramid(levels);

    this->dx = QDVO::ImagePyramid(levels);
    this->dy = QDVO::ImagePyramid(levels);

}

void QDVO::CompleteImagePyramid::generate(const cv::Mat& baseImage)
{
    this->image.generate(baseImage);

    // compute the image gradients
    cv::Mat temp;
    cv::Sobel( this->image.getImage(), temp, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    this->dx.generate(temp);

    cv::Sobel( this->image.getImage(), temp, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
    this->dy.generate(temp);
}
