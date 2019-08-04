#include "Image.h"
#include <opencv2/core/eigen.hpp>

QDVO::Image::Image(cv::Mat& cvImage)
{
    cv::cv2eigen(cvImage, this->image);
}

cv::Mat QDVO::Image::toOpenCVImage()
{
    cv::Mat img;
    cv::eigen2cv(this->image, img);
    return img;
}