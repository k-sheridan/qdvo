#pragma once

#include <opencv2/core.hpp>
#include "IMUState.h"
#include "CameraModel.hpp"
#include "Landmark.h"
#include "ImagePyramid.h"
#include "ImageStatisticsLUT.h"
#include "CorrespondenceDistribution.h"

namespace  QDVO {
class Frame
{
public:
    Frame();


    ID_TYPE camID, frameID; // camid: the id of the camera this frame is asociated to. frameID: the unique sequential id of this frame.

    IMUState imustate; // stores the state of the frame.

    ImagePyramid imagePyr; // holds the actual image for this frame.

    CameraModel* cm = nullptr; // pointer to the global camera model for this frame.

    std::vector<Landmark> landmarks; // array of landmarks hosted in this frame. ID's should be ordered and landmarks should never be deleted.

    /*
     * a preallocated array of correspondence distributions for this frame.
     * This array should never be shrunk. the correspondence distributions can be uninitialized though.
     */
    std::vector<CorrespondenceDistribution> correspondenceDistributions;


    enum FrameStatus {
       INACTIVE,
       ACTIVE,
       MARGINALIZED
    } status;

    int maxIntensity();

    void updateImage(cv::Mat& baseImage);

    /*
     * resets all members of this frame while leaving the memory used by them allocated.
     */
    void reset();

};

}
