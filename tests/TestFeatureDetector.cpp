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

    std::vector<QDVO::Feature> features = detector.detectFeatures(f);

    // draw for debug
    /*cv::Mat render;
    cv::cvtColor(f.image, render, cv::COLOR_GRAY2BGR);

    for (auto& e : features)
    {
        cv::drawMarker(render, e.px, cv::Scalar(255, 0, 0));
    }

    cv::imshow("feature detection", render);
    cv::imshow("mask", detector.spatialMask);
    cv::waitKey(10000);*/
}


