#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include "GlobalDefinitions.h"

namespace QDVO {

    class Image {
        public:
        Image(cv::Mat& cvImage);
        Image(){}

        cv::Mat toOpenCVImage();

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& getImage(){return image;}

        private:
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> image;
    };
}