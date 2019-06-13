#pragma once

#include <opencv2/core.hpp>
#include <IMUState.h>
#include <CameraModel.hpp>
#include <Landmark.h>
#include <ImagePyramid.h>
#include <ImageStatisticsLUT.h>

namespace  QDVO {
class Frame
{
public:
    Frame();


    ID_TYPE camID, frameID; // camid: the id of the camera this frame is asociated to. frameID: the unique sequential id of this frame.

    IMUState imustate;

    ImagePyramid imagePyr; // image.

    ImageStatisticsLUT imageStatistics; // stores the precomputed approximate stddev and mean table.

    CameraModel* cm = nullptr; // pointer to the global camera model for this frame.

    std::vector<Landmark> landmarks; // array of landmarks hosted in this frame. ID's should be ordered and landmarks should never be deleted.


    enum FrameStatus {
       INACTIVE,
       ACTIVE,
       MARGINALIZED
    } status;

    uint16_t maxIntensity() {
        switch (imagePyr.getImage().type())
        {
        case CV_8U: return 255;

        //case CV_16U: return 65535;

        default: throw std::runtime_error("image type not supported.");

        }
    }

    void updateImage(cv::Mat& baseImage);

    /*
     * resets all members of this frame while leaving the memory used by them allocated.
     */
    void reset();

};

}
