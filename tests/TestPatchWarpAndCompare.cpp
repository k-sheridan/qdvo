#include "gtest/gtest.h"
//#include "VO.h"
#include "Frame.h"
#include "Graph.h"
#include "PatchWarper.h"
#include "PatchComparer.h"
#include "EquidistantCameraModel.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

TEST(PatchWarpAndCompare, Basic)
{
    QDVO::EquidistantCameraModel cm = QDVO::EquidistantCameraModel(300, 301, 255, 256, PI/2.1, 512, 400, Eigen::Vector4d(0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182));
    cv::Mat img;
    img = cv::imread("/Users/kevinsheridan/Documents/Mac Library/RnD/qdvo/tests/images/1.png", cv::IMREAD_GRAYSCALE);
    QDVO::Frame f1;
    f1.imagePyr.generate(img);
    f1.camID = 1;
    f1.frameID = 1;
    f1.cm = &cm;
    f1.status = QDVO::Frame::ACTIVE;
    f1.initialized = true;
    f1.imustate.pos << 0, 0, 0;

    QDVO::PatchComparer pc;
    pc.meanStdDevTable.setupTables(f1.imagePyr.getImage(), &f1);

    QDVO::Landmark l;
    l.px = QDVO::Vector2(255, 255);
    l.dinv = 1;
    l.bearing = QDVO::Vector3(0, 0, 1);
    l.landmarkID = 1;
    f1.landmarks.push_back(l);

    QDVO::Frame f2;
    f2.camID = 1;
    f2.imagePyr.generate(img);
    f2.status = QDVO::Frame::INACTIVE;
    f2.frameID = 2;
    f2.cm = &cm;
    f2.imustate.pos << 0, 0, 0;

    QDVO::Graph g;
    QDVO::SE3 unitT(Eigen::Quaternion<QDVO::SE3::Scalar>(1, 0, 0, 0), Eigen::Matrix<QDVO::SE3::Scalar, 3, 1>(0, 0, 0));
    g.setExtrinsic(unitT, 1);


    QDVO::PatchWarper pw;
    QDVO::Patch wp;
    TIK
    for (int i = 0; i < 1000; ++i)
    {

        pw.warpPatchToTargetFrame(wp, l, f1, f2, g);


    }
    TOK

    QDVO::ResultType<SCALAR_TYPE> score;
    Eigen::Vector2i pxI(256, 255);

    RETIK
    for (int i = 0; i < 1000; ++i)
    {
         score = pc.compare(wp, f1, pxI);
    }
    RETOK
    std::cout << score.getResult() << std::endl;

    /* cv::Mat render;
    cv::Mat(PATCH_WIDTH, PATCH_WIDTH, CV_32F, wp.getImageData().data()).convertTo(render, CV_8U);
    cv::imshow("patch", render);
    cv::Rect roi(cv::Point2i(256, 255), cv::Size2i(PATCH_WIDTH, PATCH_WIDTH));
    cv::imshow("raw", img(roi));
    cv::waitKey(100000);*/

    ASSERT_NEAR(float(img.at<uint8_t>(260, 250)), f1.imagePyr.getColorSubpix(img, cv::Point2f(250, 260)), 1e-8);

}


