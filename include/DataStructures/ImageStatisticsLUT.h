#ifndef IMAGESTATISTICSLUT_H
#define IMAGESTATISTICSLUT_H

#include <opencv2/core.hpp>

namespace QDVO {

class ImageStatisticsLUT {

public:

    ImageStatisticsLUT(){}

    void setupTables(const cv::Mat& baseImage)
    {

        double colSize, rowSize;
        if (baseImage.rows >= baseImage.cols)
        {
            this->rowStride = baseImage.rows / double(IMAGE_STDDEV_RESOLUTION);
            colSize = std::round(baseImage.cols / this->rowStride);
            this->colStride = baseImage.cols / colSize;
            rowSize = double(IMAGE_STDDEV_RESOLUTION);

        }
        else
        {
            colStride = baseImage.cols / double(IMAGE_STDDEV_RESOLUTION);
            rowSize = std::round(baseImage.rows / colStride);
            rowStride = baseImage.rows / rowSize;
            colSize = IMAGE_STDDEV_RESOLUTION;
        }

        //std::cout << "this->rowStride << " " << this->colStride" << std::endl;

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

    float getMean(const cv::Point2f& px){return this->getColorSubpix(this->localMeanLUT, cv::Point2f(px.x / this->colStride, px.y / this->rowStride));}
    float getStdDev(const cv::Point2f& px){return this->getColorSubpix(this->localStandardDeviationLUT, cv::Point2f(px.x / this->colStride, px.y / this->rowStride));}

    float getColorSubpix(const cv::Mat& img, cv::Point2f pt)
    {
            int x = (int)pt.x;
            int y = (int)pt.y;

            int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REFLECT_101);
            int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REFLECT_101);
            int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REFLECT_101);
            int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REFLECT_101);

            float a = pt.x - (float)x;
            float c = pt.y - (float)y;

            return ((img.at<float>(y0, x0) * (1.f - a) + img.at<float>(y0, x1) * a) * (1.f - c)
                                   + (img.at<float>(y1, x0) * (1.f - a) + img.at<float>(y1, x1) * a) * c);
    }

    // these tables are generated from the top down through upsampling.
    cv::Mat localStandardDeviationLUT; // used to speed up the patch comparison metrics.
    cv::Mat localMeanLUT;
    float rowStride, colStride;

};
}

#endif // IMAGESTATISTICSLUT_H
