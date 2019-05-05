#pragma once

#include <opencv4/opencv2/core.hpp>
#include <IMUState.h>
#include <CameraModel.hpp>
#include <Landmark.h>

class Frame
{
public:
    Frame();

    uint64_t camID, frameID; // camid: the id of the camera this frame is asociated to. frameID: the unique sequential id of this frame.

    IMUState imustate;

    std::unique_ptr<cv::Mat> image; // pointer to an opencv image.

    std::unique_ptr<CameraModel> cm; // pointer to the global camera model for this frame.

    std::vector<Landmark> landmarks; // array of landmarks hosted in this frame.
};

