#include "gtest/gtest.h"
#include "VO.h"
#include <Frame.h>
#include <CompleteImagePyramid.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

TEST(CompleteImagePyr, Basic)
{
    cv::Mat img;
    img = cv::imread("/Users/kevinsheridan/Documents/Mac Library/RnD/qdvo/tests/images/1.png", cv::IMREAD_GRAYSCALE);
    QDVO::Frame f;
    f.image = img;

    TIK
    QDVO::CompleteImagePyramid pyr(3);

    pyr.generate(img);
    TOK

    RETIK
    std::cout << pyr.getMean(cv::Point2f(1, 1)) << std::endl;
    RETOK

    // draw for debug
    //cv::imshow("testPyr", pyr.localMeanLUT);
    //cv::waitKey(10000);
}


