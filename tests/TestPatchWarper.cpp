#include "gtest/gtest.h"
//#include "VO.h"
#include "Frame.h"
#include "Graph.h"
#include "PatchWarper.h"
#include "EquidistantCameraModel.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

TEST(PatchWarp, Basic)
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

    QDVO::Landmark l;
    l.px = QDVO::Vector2(255, 255);
    l.dinv = 1;
    l.bearing = QDVO::Vector3(0.1, 0.1, 1);
    l.landmarkID = 1;
    f1.landmarks.push_back(l);

    QDVO::Frame f2;
    f2.camID = 1;
    f2.status = QDVO::Frame::INACTIVE;
    f2.frameID = 2;
    f2.cm = &cm;

    QDVO::Graph g;
    QDVO::SE3 unitT(Eigen::Quaternion<QDVO::SE3::Scalar>(1, 0, 0, 0), Eigen::Matrix<QDVO::SE3::Scalar, 3, 1>(0, 0, 0));
    g.setExtrinsic(unitT, 1);


    QDVO::PatchWarper pw;
    QDVO::Patch wp;
    TIK
    for (int i = 0; i < 200; ++i)
    {

        pw.warpPatchToTargetFrame(wp, l, f1, f2, g);

    }
    TOK

    cv::imshow("patch", wp.image);
    cv::waitKey(1000);

}


