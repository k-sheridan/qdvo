#include "BasicPipeline.h"
#include <algorithm>
#include "DataStructures/Feature.h"
#include "DataStructures/Graph.h"
#include "Settings.h"
#include <cmath> 

QDVO::BasicPipeline::BasicPipeline()
{
}

void QDVO::BasicPipeline::initialize()
{
    // precompute the radial search pattern LUT
    radialSearchPatternPtr = std::make_shared<RadialSearchPattern>(MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS);

    // create the patch warper
    patchWarper = std::make_unique<QDVO::PatchWarper>();

    SPDLOG_INFO("Computed radial search pattern");

    // create a feature detector
    featureDetector = std::make_unique<QDVO::FeatureDetector>();
    SPDLOG_INFO("Created a new feature detector");

    patchComparer = std::make_shared<PatchComparer>();
}

QDVO::CameraModelMap::key_type QDVO::BasicPipeline::addCamera(std::unique_ptr<QDVO::CameraModel> &cameraModel)
{
    QDVO::SE3 unit(Eigen::Quaternion<QDVO::SE3::Scalar>(1, 0, 0, 0), Eigen::Matrix<QDVO::SE3::Scalar, 3, 1>(0, 0, 0));
    SPDLOG_INFO("Camera Model Initialized");
    return graph.insertCamera(std::move(cameraModel), unit);
}

void QDVO::BasicPipeline::addFrame(cv::Mat &image, const double &time, const CameraModelMap::key_type& cameraModelKey, const ExtrinsicMap::key_type& extrinsicKey)
{
    SPDLOG_INFO("Added frame.");
    // save the last imu state
    QDVO::IMUState lastImuState = graph.getCurrentFrame()->imustate;
    // Reset current frame
    graph.getCurrentFrame()->reset();
    // Setup the current frame.
    graph.getCurrentFrame()->updateImage(image);
    graph.getCurrentFrame()->status = QDVO::Frame::FrameStatus::INACTIVE;
    graph.getCurrentFrame()->cameraModelKey = cameraModelKey;
    graph.getCurrentFrame()->extrinsicKey = extrinsicKey; 
    // for the monocular case, we can assume no motion between frames initially
    graph.getCurrentFrame()->imustate = lastImuState;
    graph.getCurrentFrame()->imustate.time = time;
    graph.getCurrentFrame()->initialized = true;

    // Initialize correspondence distributions

    initializeCorrespondenceDistributionsForCurrentFrame();

    // Run front end visual odometry
    frontEndVisualOdometry.run(graph);

    // Check if the current frame meets the keyframe selection criteria
    if (isCurrentFrameAKeyframe())
    {
        // Move the current frame into the keyframe set
        auto newKeyframeKey = graph.getCurrentFrameKey();
        graph.moveCurrentFrameIntoKeyframePosition();
        // The new current frame should not be the same as the old one.
        assert(!(newKeyframeKey == graph.getCurrentFrameKey()));
        
        // TODO: The rest of this can be ran on a separate thread. 

        // Create new landmarks for the new keyframe
        createNewLandmarks(graph, newKeyframeKey, featureDetector);

        // set the new keyframe to active
        (*graph.getKeyframeMap().at(newKeyframeKey))->status = QDVO::Frame::ACTIVE;

        // run the sliding window estimator with the current keyframe set
        swe.run(graph);

        // marginalize excess keyframe
        runMarginalizationStrategy();

        // remove outliers found during sliding window estimation
        swe.removeOutliers(graph);

        // attempt to estimate the landmark depths using the new motion estimates
        runEpipolarDepthEstimators();

        // activate new landmarks if necessary
        activateNewLandmarks();

    }
    SPDLOG_INFO("Finished adding frame.");
}

void QDVO::BasicPipeline::runMarginalizationStrategy()
{
    swe.runMarginalizationStrategy(graph);
}

void QDVO::BasicPipeline::createNewLandmarks(Graph& graph, KeyframeMap::key_type keyframeKey, std::unique_ptr<QDVO::FeatureDetector> &featureDetector)
{
    SPDLOG_INFO("Creating new landmarks for keyframe idx:{}, gen:{}", keyframeKey.index, keyframeKey.generation);

    std::unique_ptr<Frame>& keyframe = *graph.getKeyframeMap().at(keyframeKey);

    // Detect new features in the keyframe
    std::vector<QDVO::Feature> newFeatures = featureDetector->detectFeatures(*keyframe);

    SPDLOG_INFO("Found {} new landmarks", newFeatures.size());

    // add the landmarks to the keyframe's landmark vector
    std::unique_ptr<QDVO::CameraModel> &cm = graph.getCameraModelMap().at(keyframe->cameraModelKey)->first;

    for (auto &f : newFeatures)
    {
        Landmark lm;
        lm.px = Eigen::Matrix<SCALAR_TYPE, 2, 1>(f.px.x, f.px.y);
        lm.parentFrameKey = keyframeKey;
        lm.dinv = DEFAULT_LANDMARK_DINV;

        auto result = cm->unproject(lm.px);
        if (!result.has_value())
        {
            SPDLOG_INFO("failed to unproject pixel.");
            continue;
        }

        lm.bearing = result.value();

        auto landmarkKey = graph.getLandmarkMap().insert(lm);

        keyframe->landmarkKeys.push_back(landmarkKey);
    }
}

void QDVO::BasicPipeline::initializeCorrespondenceDistributionsForCurrentFrame()
{
    SPDLOG_INFO("Initializing correspondence distributions for current frame.");
    // first update the patch comparers before initializing all the correspondence distributions
    updatePatchComparers();

    std::unique_ptr<QDVO::Frame> &cf = graph.getCurrentFrame();
    std::unique_ptr<CameraModel>& cm = graph.getCameraModelMap().at(cf->cameraModelKey)->first;

SPDLOG_INFO("here");
    // second reset correspondence distributions
    cf->resetCorrespondenceDistributions();

    // find the set of active landmarks visible in the current frame.
    // create and initialize the correspondence distribution for each of these landmarks
    std::vector<std::tuple<LandmarkMap::key_type, QDVO::Vector2>> visibleActiveLandmarks = graph.getVisibleLandmarksInCurrentFrame(true);

    // Make sure that there are enough correspondence distributions
    int deficit = std::max(int(visibleActiveLandmarks.size() - cf->correspondenceDistributions.size()), 0);
    for (int i = 0; i < deficit; ++i)
    {
        // create another correspondence distribution
        cf->correspondenceDistributions.push_back(QDVO::CorrespondenceDistribution(cm->width, cm->height, radialSearchPatternPtr)); 
    }

    SPDLOG_INFO("found {} visible and active landmarks for the current frame", visibleActiveLandmarks.size());

    auto initializationFn = [&](std::tuple<LandmarkMap::key_type, QDVO::Vector2> &tup, QDVO::CorrespondenceDistribution &cdRef) -> int {
        LandmarkMap::key_type lKey = std::get<0>(tup);

        Landmark& l = *graph.getLandmarkMap().at(lKey);

        Frame& f = *(*graph.getKeyframeMap().at(l.parentFrameKey));

        CameraModel& cm = *(graph.getCameraModelMap().at(f.cameraModelKey)->first);

        assert(cdRef.dormant == true);

        Frame& landmarkParentFrame = *(*graph.getKeyframeMap().at(l.parentFrameKey));

        // initialize the correspondence distribution
        auto px0 = graph.projectLandmarkToPixel(*graph.getCurrentFrame(), landmarkParentFrame, l);
        if (!px0.has_value())
        {
            SPDLOG_TRACE("landmark not visible in its parent frame!");
            return 1;
        }
        QDVO::Result<QDVO::Patch> warpedPatch = {};

        // warp the patch.
        patchWarper->warpPatchToTargetFrame(warpedPatch, l, landmarkParentFrame, *(graph.getCurrentFrame()), graph);
        if (!warpedPatch.has_value())
        {
            SPDLOG_TRACE("failed to warp patch");
            return 1;
        }

        cdRef.initializeDistribution(cm, *(graph.getCurrentFrame()), lKey, Eigen::Vector2i(std::round(px0.value()(0)), std::round(px0.value()(1))), MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS, patchComparer, warpedPatch.value());
        return 0;
    };

    std::vector<int> result(visibleActiveLandmarks.size());
    // Run the initialization function for all active and visible landmarks.
    QDVO::ParallelAlgorithms::transform(QDVO::ParallelAlgorithms::ExecutionType::PARALLEL_CPU, visibleActiveLandmarks.begin(), visibleActiveLandmarks.end(), cf->correspondenceDistributions.begin(), result.begin(), initializationFn);

    SPDLOG_INFO("Initialized correspondence distributions for this frame. Could not initialize: {} distributions" ,std::accumulate(result.begin(), result.end(), 0));
}

bool QDVO::BasicPipeline::isCurrentFrameAKeyframe()
{
    if (graph.getKeyframeMap().size() == 1)
    {
        SPDLOG_INFO("First frame is always a keyframe.");
        return true;
    }

    Settings s;

    // Compute the pixel flow for each keyframe to check if the current frame should be made a keyframe.
    // Find the lowest pixel flow score.
    double smallestPixelFlowScore = std::numeric_limits<double>::max();
    for (auto it = graph.getKeyframeMap().begin(); it != graph.getKeyframeMap().end(); it++) {
        auto thisKey = graph.getKeyframeMap().getKeyFromDataIndex(it - graph.getKeyframeMap().begin());
        if (thisKey == graph.getCurrentFrameKey()) {
            continue;
        }

        // Compute the pixel flow with the current frame.
        auto pixelFlows = computePixelFlowForCurrentFrame(thisKey);

        SPDLOG_INFO("For keyframe {}-{}, Average pixel flow: {} Average translational pixel flow: {}", thisKey.index, thisKey.generation, pixelFlows.first, pixelFlows.second);

        double score = s.weightAvgPixelFlow * pixelFlows.first + s.weightAvgTranslationalFlow * pixelFlows.second; 

        if (score < smallestPixelFlowScore) {
            smallestPixelFlowScore = score;
        }
    }

    assert(!std::isnan(smallestPixelFlowScore));
    // If this condition is met for any keyframe, The current frame pose is sufficiently "far" from all keyframes to warrant creating a
    // new keyframe.
    if (smallestPixelFlowScore > 1) {
        SPDLOG_INFO("Current frame qualifies as a keyframe!");
        return true;
    }

    return false;
}

std::pair<double, double> QDVO::BasicPipeline::computePixelFlowForCurrentFrame(KeyframeMap::key_type key) 
{
    Frame& latestKeyframe = *(*graph.getKeyframeMap().at(key));
    QDVO::SE3 currentFramePose = graph.getCurrentFrame()->imustate.getSE3();
    QDVO::SE3 latestKeyframePose = latestKeyframe.imustate.getSE3();
    double pixelFlow = 0;
    int numberOfObservedFeatures = 0;
    for (auto landmarkIt = graph.getLandmarkMap().begin(); landmarkIt != graph.getLandmarkMap().end(); landmarkIt++) {
        // Skip this landmark if it is not active.
        if (landmarkIt->status != Landmark::LandmarkStatus::ACTIVE) {
            continue;
        }

        auto lKey = graph.getLandmarkMap().getKeyFromDataIndex(landmarkIt - graph.getLandmarkMap().begin());

        auto keyframePixel = graph.projectLandmarkToPixel(key, landmarkIt->parentFrameKey, lKey);
        auto currentFramePixel = graph.projectLandmarkToPixel(graph.getCurrentFrameKey(), landmarkIt->parentFrameKey, lKey);

        if (keyframePixel.has_value() && currentFramePixel.has_value()) {
            pixelFlow += (currentFramePixel.value() - keyframePixel.value()).norm();
            ++numberOfObservedFeatures;
        }
    }

    pixelFlow = pixelFlow / numberOfObservedFeatures;

    double translationalPixelFlow = 0;
    // Set the current frame rotation to the keyframe rotation
    graph.getCurrentFrame()->imustate.attitude = latestKeyframePose.so3();
    numberOfObservedFeatures = 0;
    for (auto landmarkIt = graph.getLandmarkMap().begin(); landmarkIt != graph.getLandmarkMap().end(); landmarkIt++) {
        // Skip this landmark if it is not active.
        if (landmarkIt->status != Landmark::LandmarkStatus::ACTIVE) {
            continue;
        }

        auto lKey = graph.getLandmarkMap().getKeyFromDataIndex(landmarkIt - graph.getLandmarkMap().begin());

        auto keyframePixel = graph.projectLandmarkToPixel(key, landmarkIt->parentFrameKey, lKey);
        auto currentFramePixel = graph.projectLandmarkToPixel(graph.getCurrentFrameKey(), landmarkIt->parentFrameKey, lKey);

        if (keyframePixel.has_value() && currentFramePixel.has_value()) {
            translationalPixelFlow += (currentFramePixel.value() - keyframePixel.value()).norm();
            ++numberOfObservedFeatures;
        }
    }
    // Reset the current frame attitude
    graph.getCurrentFrame()->imustate.attitude = currentFramePose.so3();

    translationalPixelFlow = translationalPixelFlow / numberOfObservedFeatures;

    return std::make_pair(pixelFlow, translationalPixelFlow);
}

void QDVO::BasicPipeline::updatePatchComparers()
{

}

void QDVO::BasicPipeline::runEpipolarDepthEstimators()
{
}

void QDVO::BasicPipeline::activateNewLandmarks()
{
    /*
     * Warning: This is an absolute mess, but for now it will have to do.
     */

    SPDLOG_INFO("Activating landmarks." );

    // find all visible active and inactive landmarks
    std::vector<std::tuple<LandmarkMap::key_type, QDVO::Vector2>> visibleLandmarks = graph.getVisibleLandmarksInCurrentFrame(false, true);

    SPDLOG_INFO("there are currently {} active and inactive landmarks visible in the current frame" , visibleLandmarks.size());

    // janky way of getting a decent feature distribution.
    auto& cm = graph.getCameraModelMap().at(graph.getCurrentFrame()->cameraModelKey)->first;
    cv::Mat mask = cv::Mat::zeros(cm->imageHeight(), cm->imageWidth(), CV_8U);

    const int maskRadius = 5;

    int nActiveLandmarks = 0;

    // fill in the mask for all active landmarks.
    for (auto &t : visibleLandmarks)
    {
        LandmarkMap::key_type lKey = std::get<0>(t);

        Landmark& l = *graph.getLandmarkMap().at(lKey);

        if (l.status == QDVO::Landmark::ACTIVE)
        {
            cv::circle(mask, cv::Point2f(l.px(0), l.px(1)), maskRadius, cv::Scalar(255), -1);
            ++nActiveLandmarks;
        }
    }

    SPDLOG_INFO("{} Active visible landmarks before activation.", nActiveLandmarks );

    if (nActiveLandmarks >= N_FEATURES_DESIRED)
    {
        return;
    }

    // first activate initialized landmarks
    for (auto &t : visibleLandmarks)
    {
        LandmarkMap::key_type lKey = std::get<0>(t);

        Landmark& l = *graph.getLandmarkMap().at(lKey);
        
        if (l.status == QDVO::Landmark::INACTIVE && l.depthEstimator.initialized)
        {
            if (!mask.at<uint8_t>(cv::Point2f(l.px(0), l.px(1))))
            {
                l.status = QDVO::Landmark::ACTIVE;

                cv::circle(mask, cv::Point2f(l.px(0), l.px(1)), maskRadius, cv::Scalar(255), -1);
                ++nActiveLandmarks;
            }
        }

        if (nActiveLandmarks >= N_FEATURES_DESIRED)
        {
            break;
        }
    }

    SPDLOG_INFO("{} Active visible landmarks after activating initialized landmarks.", nActiveLandmarks);

    // if necessary activate uninitialized landmarks
    if (nActiveLandmarks < MINUMUM_ACTIVE_LANDMARKS)
    {
        for (auto &t : visibleLandmarks)
        {
            LandmarkMap::key_type lKey = std::get<0>(t);

            Landmark& l = *graph.getLandmarkMap().at(lKey);
        
            if (l.status == QDVO::Landmark::INACTIVE)
            {
                if (!mask.at<uint8_t>(cv::Point2f(l.px(0), l.px(1))))
                {
                    l.status = QDVO::Landmark::ACTIVE;

                    cv::circle(mask, cv::Point2f(l.px(0), l.px(1)), maskRadius, cv::Scalar(255), -1);
                    ++nActiveLandmarks;
                }
            }

            if (nActiveLandmarks >= MINUMUM_ACTIVE_LANDMARKS)
            {
                break;
            }
        }
        SPDLOG_INFO("{} active visible landmarks after activating uninitialized landmarks.", nActiveLandmarks);
    }
    SPDLOG_INFO("Finished activating landmakrs.");
}
