#include "BasicPipeline.h"
#include <algorithm>
#include "DataStructures/Feature.h"
#include "DataStructures/Graph.h"

QDVO::BasicPipeline::BasicPipeline()
{
}

void QDVO::BasicPipeline::initialize()
{
    // precompute the radial search pattern LUT
    radialSearchPatternPtr = std::make_shared<RadialSearchPattern>(MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS);

    // create the patch warper
    patchWarper = std::make_unique<QDVO::PatchWarper>();

    std::cout << "Computed radial search pattern" << std::endl;

    // create a feature detector
    featureDetector = std::make_unique<QDVO::FeatureDetector>();
    std::cout << "Created a new feature detector" << std::endl;

    patchComparer = std::make_shared<PatchComparer>();
}

QDVO::CameraModelMap::key_type QDVO::BasicPipeline::addCamera(std::unique_ptr<QDVO::CameraModel> &cameraModel)
{
    QDVO::SE3 unit(Eigen::Quaternion<QDVO::SE3::Scalar>(1, 0, 0, 0), Eigen::Matrix<QDVO::SE3::Scalar, 3, 1>(0, 0, 0));
    std::cout << "Camera Model Initialized" << std::endl;
    return graph.insertCamera(std::move(cameraModel), unit);
}

void QDVO::BasicPipeline::addFrame(cv::Mat &image, const double &time, const CameraModelMap::key_type& cameraModelKey, const ExtrinsicMap::key_type& extrinsicKey)
{
   
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
        // Create new landmarks for the new keyframe
        createNewLandmarks(graph, graph.getCurrentFrameKey(), featureDetector);

        // set the current frame to active
        graph.getCurrentFrame()->status = QDVO::Frame::ACTIVE;

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

        // finally move the current frame into the keyframe set
        graph.moveCurrentFrameIntoKeyframePosition();
    }
}

void QDVO::BasicPipeline::runMarginalizationStrategy()
{
    swe.runMarginalizationStrategy(graph);
}

void QDVO::BasicPipeline::createNewLandmarks(Graph& graph, KeyframeMap::key_type keyframeKey, std::unique_ptr<QDVO::FeatureDetector> &featureDetector)
{
    std::cout << "Creating new landmarks for keyframe: " << keyframeKey.index << "," << keyframeKey.generation << std::endl;

    std::unique_ptr<Frame>& keyframe = *graph.getKeyframeMap().at(keyframeKey);

    // Detect new features in the keyframe
    std::vector<QDVO::Feature> newFeatures = featureDetector->detectFeatures(*keyframe);

    std::cout << "Found " << newFeatures.size() << " new landmarks" << std::endl;

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
            std::cout << "failed to unproject pixel." << std::endl;
            continue;
        }

        lm.bearing = result.value();

        auto landmarkKey = graph.getLandmarkMap().insert(lm);

        keyframe->landmarkKeys.push_back(landmarkKey);
    }
}

void QDVO::BasicPipeline::initializeCorrespondenceDistributionsForCurrentFrame()
{
    // first update the patch comparers before initializing all the correspondence distributions
    updatePatchComparers();

    std::unique_ptr<QDVO::Frame> &cf = graph.getCurrentFrame();
    std::unique_ptr<CameraModel>& cm = graph.getCameraModelMap().at(cf->cameraModelKey)->first;

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

    std::cout << "found " << visibleActiveLandmarks.size() << " visible and active landmarks for the current frame" << std::endl;

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
            std::cout << "landmark not visible in its parent frame!" << std::endl;
            return 1;
        }
        QDVO::Result<QDVO::Patch> warpedPatch = {};

        // warp the patch.
        patchWarper->warpPatchToTargetFrame(warpedPatch, l, landmarkParentFrame, *(graph.getCurrentFrame()), graph);
        if (!warpedPatch.has_value())
        {
            std::cout << "failed to warp patch" << std::endl;
            return 1;
        }

        cdRef.initializeDistribution(cm, *(graph.getCurrentFrame()), lKey, Eigen::Vector2i(std::round(px0.value()(0)), std::round(px0.value()(1))), MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS, patchComparer, warpedPatch.value());
        return 0;
    };

    std::vector<int> result(visibleActiveLandmarks.size());
    // Run the initialization function for all active and visible landmarks.
    QDVO::ParallelAlgorithms::transform(QDVO::ParallelAlgorithms::ExecutionType::SEQUENTIAL, visibleActiveLandmarks.begin(), visibleActiveLandmarks.end(), cf->correspondenceDistributions.begin(), result.begin(), initializationFn);

    std::cout << "Initialized correspondence distributions for this frame. Could not initialize: " << std::accumulate(result.begin(), result.end(), 0) << " distributions." << std::endl;
}

bool QDVO::BasicPipeline::isCurrentFrameAKeyframe()
{
    if (graph.getKeyframeMap().size() == 1)
    {
        std::cout << "First frame is always a keyframe." << std::endl;
        return true;
    }

    return false;
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

    std::cout << "Activating landmarks." << std::endl;

    // find all visible active and inactive landmarks
    std::vector<std::tuple<LandmarkMap::key_type, QDVO::Vector2>> visibleLandmarks = graph.getVisibleLandmarksInCurrentFrame(false, true);

    std::cout << "there are currently " << visibleLandmarks.size() << " active and inactive landmarks visible in the current frame" << std::endl;

    // janky way of getting a decent feature distribution.
    cv::Mat mask = cv::Mat::zeros(graph.getCurrentFrame()->imagePyr.getImage().rows(), graph.getCurrentFrame()->imagePyr.getImage().cols(), CV_8U);

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

    std::cout << nActiveLandmarks << " active visible landmarks before activation." << std::endl;

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

    std::cout << nActiveLandmarks << " active visible landmarks after activating initialized landmarks." << std::endl;

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
        std::cout << nActiveLandmarks << " active visible landmarks after activating uninitialized landmarks." << std::endl;
    }
}
