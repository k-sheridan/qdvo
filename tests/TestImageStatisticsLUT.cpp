#include "gtest/gtest.h"
#include "VO.h"
#include <Frame.h>
#include <ImageStatisticsLUT.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

TEST(StatisticsLUT, Basic)
{
    cv::Mat img;
    img = cv::imread("/Users/kevinsheridan/Documents/Mac Library/RnD/qdvo/tests/images/1.png", cv::IMREAD_GRAYSCALE);

    TIK
    QDVO::ImageStatisticsLUT lut;
    lut.setupTables(img);
    TOK

    // draw for debug
    //cv::imshow("testPyr", pyr.localMeanLUT);
    //cv::waitKey(10000);
}


