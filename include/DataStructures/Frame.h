#pragma once

#include <opencv4/opencv2/core.hpp>
#include <IMUState.h>
#include <CameraModel.hpp>
#include <Landmark.h>

namespace  QDVO {
class Frame
{
public:
    Frame();

    ID_TYPE camID, frameID; // camid: the id of the camera this frame is asociated to. frameID: the unique sequential id of this frame.

    IMUState imustate;

    std::unique_ptr<cv::Mat> image; // pointer to an opencv image.

    CameraModel* cm = nullptr; // pointer to the global camera model for this frame.

    std::vector<Landmark> landmarks; // array of landmarks hosted in this frame. ID's should be ordered and landmarks should never be deleted.

    enum FrameStatus {
       INACTIVE,
       ACTIVE,
       MARGINALIZED
    } status;
};

}
