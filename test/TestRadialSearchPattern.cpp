#include "gtest/gtest.h"
#include <RadialSearchPattern.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

TEST(RadialSearchPattern, Basic)
{
    QDVO::RadialSearchPattern pattern(30);

    std::cout << pattern.searchPattern.size() << std::endl;

    /*cv::Mat render = cv::Mat::zeros(101, 101, CV_8U);

    u_int8_t radius = 0;
    for (auto& e : pattern.searchPattern)
    {
        for (auto& f : e)
        {
            render.at<uchar>(f(1) + 50, f(0) + 50) = radius;
        }
        radius++;
    }

    cv::imshow("radial search", render);
    cv::waitKey(10000);*/
}
