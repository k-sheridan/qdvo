#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include "GlobalDefinitions.h"
#include "Types.h"

namespace QDVO {

    typedef float ImageIntensityType;
    typedef Eigen::Matrix<ImageIntensityType, Eigen::Dynamic, Eigen::Dynamic> ImageType;

    class Image {
        public:
        Image(cv::Mat& cvImage);
        Image(){}

        cv::Mat toOpenCVImage();

        ImageType& getImageData(){return image;}

        int rows(){return image.rows();}
        int cols(){return image.cols();}

        QDVO::Result<ImageIntensityType> getSubPixelIntensity(QDVO::Vector2 px);

        private:
        ImageType image;
    };
}