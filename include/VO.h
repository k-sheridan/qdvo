#pragma once

#include <Frame.h>
#include <opencv4/opencv2/core.hpp>
#include <map>
#include <GlobalDefinitions.h>
#include <Settings.h>
#include <CameraModel.hpp>
#include <FeatureDetector.h>

namespace QDVO {
class VO
{
public:
    VO();

    void setCameraModel(std::unique_ptr<QDVO::CameraModel>& cameraModelPtr, const ID_TYPE cameraID); // Sets the camera model for the given cam ID. NOTE: QDVO creates its own local copy.


    //MEMBERS

    std::unique_ptr<QDVO::FeatureDetector> featureDetector; // pointer to a feature detector implementation

    std::map<ID_TYPE, std::unique_ptr<CameraModel>> cameraModelMap; // camID to camera model mapping. Done this way for memory/compute efficiency.

    std::map<ID_TYPE, std::unique_ptr<Frame>> keyframeSet; // gives mapping from keyframe ids to keyframes. bounds the memory consumption.

    std::unique_ptr<Frame> currentFrame; // A preallocated frame for the current frame to reside in.
};
}

