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

QDVO::Result<QDVO::ImageIntensityType> QDVO::Image::getSubPixelIntensity(QDVO::Vector2 px)
{
            int x0 = (int)px.x();
            int y0 = (int)px.y();
            int x1 = x0 + 1;
            int y1 = y0 + 1;

            if ((x0 >= (this->cols() + 1) || x0 < 0) || (y0 >= (this->rows() + 1) || y0 < 0))
            {
                //throw std::runtime_error("pixel out of bounds");
                return {};
            }

            float a = px.x() - (float)x0;
            float c = px.y() - (float)y0;

            return ((image(y0, x0) * (1.f - a) + image(y0, x1) * a) * (1.f - c)
                                   + (image(y1, x0) * (1.f - a) + image(y1, x1) * a) * c);
}