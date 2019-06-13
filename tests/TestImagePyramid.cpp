#include "gtest/gtest.h"
//#include "VO.h"
#include <Frame.h>
#include <ImagePyramid.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

TEST(ImagePyr, Basic)
{
    cv::Mat img;
    img = cv::imread("/Users/kevinsheridan/Documents/Mac Library/RnD/qdvo/tests/images/1.png", cv::IMREAD_GRAYSCALE);


    QDVO::ImagePyramid pyr(3);

    pyr.generate(img);

    // draw for debug
    //cv::imshow("testPyr", pyr.getImage(3));
    //cv::waitKey(10000);
}


