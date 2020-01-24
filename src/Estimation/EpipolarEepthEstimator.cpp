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

    // If the template patch doesnt exist, return early.
    if (!templatePatch.has_value()){
	SPDLOG_INFO("Failed to warp patch for epipolar depth estimator.");
        return;
    }
    
    // Compute the relative transformation for this frame.
    auto T_target_source = g.computeRelativeKeyframeTransform(targetKeyframe, sourceKeyframe);

    // Get the target camera model.
    CameraModel& cm = *g.getCameraModelMap().at(targetKeyframe.cameraModelKey)->first;

    QDVO::Vector3 pointInTarget, pointInTarget_unitDepth;
    Eigen::Matrix<double, 2, 3> dPi;
    Eigen::Matrix<double, 2, 2> projJac;
    // Create a lambda which computes the pixel derivative w.r.t the landmark depth.
    // return [derivatives, pixel.]
    auto dPx_dz = [&](double z)->QDVO::Result<std::pair<QDVO::Vector2, QDVO::Vector2>>{
	// TODO precompute the rotation matrix.
        pointInTarget_unitDepth = T_target_source.unit_quaternion() * landmark.bearing; 
	pointInTarget = pointInTarget_unitDepth * z + T_target_source.translation();

	auto projectionResult = cm.project(pointInTarget, &projJac);

	if (!projectionResult.has_value()){
 	    return {};
	}

        dPi << 1/pointInTarget(2, 0), 0, -pointInTarget(0, 0)/(pointInTarget(2, 0) * pointInTarget(2, 0)),
               0, 1/pointInTarget(2, 0), -pointInTarget(1, 0)/(pointInTarget(2, 0) * pointInTarget(2, 0));

	return std::make_pair((projJac * dPi * pointInTarget_unitDepth).eval(), projectionResult.value());
    };

    // Set up the result vector.
    std::vector<double> depths, scores;
    // Search for the depth with a pixel resolution,
    double depth = s.epipolar_depth_estimator.minimumDepth;
    while (depth <= s.epipolar_depth_estimator.maximumDepth){
	// Evaluate the pixel position and pixel derivative for this depth.
	auto pixelResult = dPx_dz(depth);

	// Extract a patch if possible
	QDVO::Result<QDVO::Patch> patchResult;
	if (pixelResult.has_value()){
	    patchResult = targetKeyframe.imagePyr.getImage().getSubPixelPatch(pixelResult.value().second);
	}
	// Check if there is a valid result.
	if (!patchResult.has_value()){
	    // If we failed to project after already starting the search exit.
	    if (!scores.empty()){
		break;
	    } else {
		// Manually increment by a default value.
		// TODO Don't hard code this.
		depth += 0.2;
		continue;
	    }
	} 

	// Evaluate this pixel position.
	depths.push_back(depth);
	scores.push_back(patchComparer.compare(templatePatch.value(), patchResult.value()));

	// Increment the depth using pixel derivative.
	depth += s.epipolar_depth_estimator.resolution / pixelResult.value().first.norm();
    }

    SPDLOG_INFO("Evaluated {} depths during epipolar search.", depths.size());
}
