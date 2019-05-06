#pragma once

#include <Frame.h>
#include <opencv4/opencv2/core.hpp>
#include <map>

namespace QDVO {
class VO
{
public:
    VO();

    std::map<ID_TYPE, std::unique_ptr<CameraModel>> cameraModelMap; // camID to camera model mapping.

    std::map<ID_TYPE, std::unique_ptr<Frame>> keyframeSet; // gives mapping from keyframe ids to keyframes.
};
}

