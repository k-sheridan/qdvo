#pragma once

#include <opencv2/core.hpp>
#include <deque>
#include "IMUState.h"
#include "CameraModel.hpp"
#include "Landmark.h"
#include "Types.h"
#include "ImagePyramid.h"
#include "CorrespondenceDistribution.h"
#include "Optimizer/SlotMap.h"

namespace QDVO
{
class Frame
{
public:
    Frame();

    /// Stores a key to the camera model used for projecting points into the frame.
    CameraModelMap::key_type cameraModelKey;

    /// A key pointing the the imu to camera transform for this frame.
    ExtrinsicMap::key_type extrinsicKey;

    /// Stores the state of this frame.
    IMUState imustate;

    /// Stores the image measurement for this frame.
    ImagePyramid imagePyr;

    /// Array of keys to landmarks hosted in this frame.
    std::vector<LandmarkMap::key_type> landmarkKeys; 

    /**
     * A preallocated array of correspondence distributions for this frame.
     * This array should never be shrunk. the correspondence distributions can be uninitialized though.
     */
    std::deque<CorrespondenceDistribution> correspondenceDistributions;

    /// Flag to mark whether the frame has ever been setup.
    bool initialized = false;

    /// enum representing the state of this frame.
    enum FrameStatus
    {
        INACTIVE,
        ACTIVE,
        MARGINALIZED
    } status;

    void updateImage(cv::Mat &baseImage);

    int maxIntensity() { return this->maxImageIntensity; }

    /**
     * resets all members of this frame while leaving the memory used by them allocated.
     */
    void reset();

    void resetCorrespondenceDistributions()
    {
        // reset all correspondence distributions
        for (auto &e : this->correspondenceDistributions)
        {
            e.reset();
        }

        status = FrameStatus::INACTIVE;

        initialized = false;
    }

private:
    /// The maximum value any element in the image can be.
    int maxImageIntensity;
};

} // namespace QDVO
