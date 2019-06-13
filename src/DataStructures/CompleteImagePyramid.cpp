#include "CompleteImagePyramid.h"

QDVO::CompleteImagePyramid::CompleteImagePyramid(const int levels) : image(levels)
{

}

void QDVO::CompleteImagePyramid::generate(const cv::Mat& baseImage)
{
    this->image.generate(baseImage);

    // compute the image gradients
    /*cv::Sobel(baseImage, this->dx.getImage(), CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    this->dx.generate(this->dx.getImage());

    cv::Sobel(baseImage, this->dy.getImage(), CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
    this->dy.generate(this->dy.getImage());*/

    // compute the standard deviation table
    this->setupTables(baseImage);

}


void QDVO::CompleteImagePyramid::setupTables(const cv::Mat& baseImage)
{
    double colSize, rowSize;
    if (baseImage.rows >= baseImage.cols)
    {
        rowStride = baseImage.rows / double(IMAGE_STDDEV_RESOLUTION);
        colSize = std::round(baseImage.cols / rowStride);
        colStride = baseImage.cols / colSize;
        rowSize = IMAGE_STDDEV_RESOLUTION;
    }
    else
    {
        colStride = baseImage.cols / double(IMAGE_STDDEV_RESOLUTION);
        rowSize = std::round(baseImage.rows / colStride);
        rowStride = baseImage.rows / rowSize;
        colSize = IMAGE_STDDEV_RESOLUTION;
    }

    // allocate and iterate through the LUTs.
    this->localStandardDeviationLUT = cv::Mat(rowSize, colSize, CV_32F);
    this->localMeanLUT = cv::Mat(rowSize, colSize, CV_32F);

    for (int i = 0; i < rowSize; ++i)
    {
        for (int j = 0; j < colSize; ++j)
        {
            int rl, ru, cl, cu;

            rl = std::floor(i*rowStride);
            cl = std::floor(j*colStride);
            ru = std::min(std::floor((i+1)*rowStride), float(baseImage.rows-1));
            cu = std::min(std::floor((j+1)*colStride), float(baseImage.cols-1));

            cv::Mat roi = baseImage(cv::Rect(cv::Point2i(cl, rl), cv::Point2i(cu, ru)));
            cv::Scalar mean, stddev;
            cv::meanStdDev(roi, mean, stddev);

            this->localStandardDeviationLUT.at<float>(cv::Point2i(j, i)) = stddev.val[0];
            this->localMeanLUT.at<float>(cv::Point2i(j, i)) = mean.val[0];

            //std::cout << mean[0] << " " << this->localMeanLUT.at<float>(cv::Point2i(j, i)) << std::endl;

        }
    }
}
