#include "EpipolarDepthEstimator.h"

#include "DataStructures/Graph.h"
#include "CameraModel.hpp"
#include "PatchComparer.h"
#include "PatchWarper.h"
#include "Settings.h"
#include "DataStructures/Patch.h"
#include "Types.h" 

#include "spdlog/spdlog.h"

using namespace QDVO;

void EpipolarDepthEstimator::update(Graph& g, Frame& sourceKeyframe, Frame& targetKeyframe, Landmark& landmark, PatchComparer& patchComparer, PatchWarper& patchWarper)
{
    Settings s;

    // Ensure that the landmark is inactive.
    if (landmark.status != Landmark::LandmarkStatus::INACTIVE)
    {
	SPDLOG_WARN("Tried to run epipolar depth estimator on {} landmark.", landmark.status);
    }

    if (initialized)
    {
	// Return if the estimator already has finished.
	return;
    }

    // Increment the attempts.
    ++attempts;

    if (attempts >= s.epipolar_depth_estimator.maximumAttempts)
    {
	// There have been too many attempts marginalize the landmark.
	landmark.status = Landmark::LandmarkStatus::MARGINALIZED;
	return;
    }

    // Warp the landmark patch into the target frame.
    Result<Patch> templatePatch;        
    patchWarper.warpPatchToTargetFrame(templatePatch, landmark, sourceKeyframe, targetKeyframe, g);
    
    // TODO Search for the depth with a pixel resolution,

//          Eigen::Matrix<double, 2, 3> dPi;
//          dPi << 1/pointInTarget(2, 0), 0, -pointInTarget(0, 0)/(pointInTarget(2, 0) * pointInTarget(2, 0)),
//                      0, 1/pointInTarget(2, 0), -pointInTarget(1, 0)/(pointInTarget(2, 0) * pointInTarget(2, 0));
//  Landmark dummyLandmark;
//  dummyLandmark.bearing = landmark.bearing;
//  // Evaluate the score for each inverse depth.
//  for (int i = 0; i < dinv.size(); ++i)
//  {
//      // Set the depth of the dummy landmark.
//      dummyLandmark.dinv = landmark.dinv;

//      // Project the landmark into the target frame.
//      auto pixelResult = g.projectLandmarkToPixel(targetKeyframe, sourceKeyframe, dummyLandmark);

//      // If the landmark can't be projected, set the score to 0.
//      if (!pixelResult.has_value())
//      {
//          score.at(i) = 0;
//          continue;
//      }

//      // Extract a patch for this landmark depth.
//      auto patchResult = targetKeyframe.imagePyr.getImage().getSubPixelPatch(pixelResult.value());
//  }
//  
}
