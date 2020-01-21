#pragma once

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include "GlobalDefinitions.h"
#include "Types.h"
#include "DataStructures/Patch.h"

namespace QDVO {

    typedef float ImageIntensityType;
    typedef Eigen::Matrix<ImageIntensityType, Eigen::Dynamic, Eigen::Dynamic> ImageType;

    class Image {
        public:
        Image(cv::Mat& cvImage){
            cv::cv2eigen(cvImage, this->image);
        }
        Image(){}

        cv::Mat toOpenCVImage(){
            cv::Mat img;
            cv::eigen2cv(this->image, img);
            return img;
        }

        ImageType& getImageData(){return image;}

        int rows(){return image.rows();}
        int cols(){return image.cols();}

        QDVO::Result<ImageIntensityType> getSubPixelIntensity(QDVO::Vector2 px);

	QDVO::Result<Patch> getSubPixelPatch(QDVO::Vector2 centerPixel, int patchWidth = PATCH_WIDTH);

        private:
        ImageType image;
    };
}
