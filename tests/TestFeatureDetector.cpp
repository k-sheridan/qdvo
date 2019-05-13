#include "gtest/gtest.h"
#include "VO.h"
#include <Frame.h>
#include <FeatureDetector.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>

TEST(FeatureDetector, Basic)
{
    cv::Mat img;
    img = cv::imread("/Users/kevinsheridan/Documents/Mac Library/RnD/qdvo/tests/images/1.png", cv::IMREAD_GRAYSCALE);
    QDVO::Frame f;
    f.image = img;

    std::cout << img.rows << " " << f.image.rows << std::endl;

    QDVO::FeatureDetector detector = QDVO::FeatureDetector();

    detector.detectFeatures(f);
}


