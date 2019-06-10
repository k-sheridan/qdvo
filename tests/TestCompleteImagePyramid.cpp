#include "gtest/gtest.h"
#include "VO.h"
#include <Frame.h>
#include <CompleteImagePyramid.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>

TEST(CompleteImagePyr, Basic)
{
    cv::Mat img;
    img = cv::imread("/Users/kevinsheridan/Documents/Mac Library/RnD/qdvo/tests/images/1.png", cv::IMREAD_GRAYSCALE);
    QDVO::Frame f;
    f.image = img;

    QDVO::CompleteImagePyramid pyr(3);

    pyr.generate(img);

    // draw for debug
    //cv::imshow("testPyr", pyr.dx.getImage(2));
    //cv::waitKey(10000);
}


