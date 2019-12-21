#pragma once




#include <memory>
#include <mutex>
#include <algorithm>
#include <thread>
#include <tuple>
#include "GlobalDefinitions.h"
#include "Settings.h"
#include "FeatureDetector.h"
#include "Types.h"
#include "Optimizer/SlotMap.h"

namespace QDVO
{

class Frame; // fwd
class Landmark; // fwd
class CameraModel; // fwd

/**
 * The core datastructure for QDVO.
 */
class Graph
{
public:

    Graph();

    /// Inserts a camera into QDVO.
    /// @param cameraModel camera model used to project points into the camera
    /// @param initial_T_imu_cam Initial estimate of the imue to camera transform. If no imu exists, this is the pose the camera state is in.
    /// @return camera model key used to associate data to.
    CameraModelMap::key_type insertCamera(std::unique_ptr<QDVO::CameraModel> cameraModel, const QDVO::SE3 &Initial_T_imu_cam)
    {
        return cameraModelMap.insert(std::make_pair(std::move(cameraModel), Initial_T_imu_cam));
    }

    /// Get a reference to the current frame unique pointer.
    /// @return Unique pointer reference to the current frame.
    std::unique_ptr<Frame> &getCurrentFrame();

    CameraModelMap &getCameraModelMap();

    ExtrinsicMap &getExtrinsicMap();

    KeyframeMap &getKeyframeMap();

    LandmarkMap& getLandmarkMap();

    /**
     * This function will swap the current frame and the marginalized keyframe and update the hash table key to reflect the new keyframe id.
     * The current frame is now equal to the marginalized keyframe. For safety, you should always check that a frame is not marginalized when using it.
     *
     */
    void moveCurrentFrameIntoMarginalizedKeyframePosition(KeyframeMap::key_type marginalizedKeyframeKey);

    /// assuming there is enough room in the keyframe set, the current frame is moved to a new spot in the keyframe set.
    void moveCurrentFrameIntoNewKeyframePosition();

    /// generalized version of the two above functions
    void moveCurrentFrameIntoKeyframePosition();

    /// Projects all landmarks in all keyframes into the current frame to determin if they are visible. will NOT project marginalized landmarks.
    /// @param activeLandmarksOnly Should the function only return active landmarks.
    /// @param includeCurrentFrameLandmarks Should the landmarks hosted in the current frame be included?
    /// @return A vector of landmarks keys and pixels.
    std::vector<std::tuple<LandmarkMap::key_type, QDVO::Vector2>> getVisibleLandmarksInCurrentFrame(bool activeLandmarksOnly, bool includeCurrentFrameLandmarks = false);

    /// Transforms the landmark into a euclidean point in the target frame.
    QDVO::Vector3 projectLandmarkToCameraFrame(KeyframeMap::key_type targetFrameKey, KeyframeMap::key_type sourceFrameKey, LandmarkMap::key_type landmarkKey);

    /// Projects a landmark in the pixels in the current frame.
    QDVO::Result<QDVO::Vector2> projectLandmarkToPixel(KeyframeMap::key_type targetFrameKey, KeyframeMap::key_type sourceFrameKey, LandmarkMap::key_type landmarkKey);

private:
    /// Stores camera models, and initial estimates of the imu to camera extrinsic.
    CameraModelMap cameraModelMap;

    /// Stores a set of landmarks.
    LandmarkMap landmarks;

    /// Stores a set of keyframes.
    KeyframeMap keyframes;

    /// Stores a set of imuToCamera transformations.
    ExtrinsicMap extrinsics;

    /// A unique pointer to the current frame which is updated at camera rate.
    /// This frame is swapped into the marginalized keyframe slot when it is made into a keyframe.
    std::unique_ptr<Frame> currentFrame;
};
} // namespace QDVO
