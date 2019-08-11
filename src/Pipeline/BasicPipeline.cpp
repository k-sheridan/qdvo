#include "BasicPipeline.h"
#include <algorithm>

QDVO::BasicPipeline::BasicPipeline()
{
}

void QDVO::BasicPipeline::initialize()
{
    // precompute the radial search pattern LUT
    this->radialSearchPatternPtr = std::shared_ptr<RadialSearchPattern>(new QDVO::RadialSearchPattern(MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS));

    // create the patch warper
    this->patchWarper = std::unique_ptr<QDVO::PatchWarper>(new QDVO::PatchWarper());

    std::cout << "Computed radial search pattern" << std::endl;

    // create current frame
    this->graph.getCurrentFrame() = std::unique_ptr<QDVO::Frame>(new Frame());
    std::cout << "Allocated current frame" << std::endl;

    // create a feature detector
    this->featureDetector = std::unique_ptr<QDVO::FeatureDetector>(new QDVO::FeatureDetector());
    std::cout << "Created a new feature detector" << std::endl;
}

void QDVO::BasicPipeline::addCamera(std::unique_ptr<QDVO::CameraModel> &cameraModel, const ID_TYPE cameraID)
{
    this->graph.setCameraModel(cameraModel, cameraID);

    std::cout << "Camera Model Initialized" << std::endl;

    QDVO::SE3 unit(Eigen::Quaternion<QDVO::SE3::Scalar>(1, 0, 0, 0), Eigen::Matrix<QDVO::SE3::Scalar, 3, 1>(0, 0, 0));
    this->graph.setExtrinsic(unit, cameraID);

    std::cout << "Added default unit extrinsic" << std::endl;
}

void QDVO::BasicPipeline::addFrame(cv::Mat &image, const double &time, const ID_TYPE cameraID)
{
    //std::cout << "here" << std::endl;
    // save the last imu state
    QDVO::IMUState lastImuState = this->graph.getCurrentFrame()->imustate;
    // Reset current frame
    this->graph.getCurrentFrame()->reset();
    // Setup the current frame.
    this->graph.getCurrentFrame()->updateImage(image);
    this->graph.getCurrentFrame()->camID = cameraID;
    // for the monocular case, we can assume no motion between frames initially
    this->graph.getCurrentFrame()->imustate = lastImuState;
    this->graph.getCurrentFrame()->imustate.time = time;
    this->graph.getCurrentFrame()->cm = this->graph.getCameraModel(cameraID).get();
    this->graph.getCurrentFrame()->frameID = this->graph.getNewFrameID();
    this->graph.getCurrentFrame()->initialized = true;

    //std::cout << "here2" << std::endl;

    // Initialize correspondence distributions

    TIK this->initializeCorrespondenceDistributionsForCurrentFrame();
    TOK

        // Run front end visual odometry
        this->frontEndVisualOdometry.run(this->graph);

    // Check if the current frame meets the keyframe selection criteria
    if (this->isCurrentFrameAKeyframe())
    {
        // Create new landmarks for the new keyframe
        this->createNewLandmarks(this->graph.getCurrentFrame(), this->featureDetector);

        // set the current frame to active
        this->graph.getCurrentFrame()->status = QDVO::Frame::ACTIVE;

        // run the sliding window estimator with the current keyframe set
        this->swe.run(this->graph);

        // marginalize excess keyframe
        this->runMarginalizationStrategy();

        // remove outliers found during sliding window estimation
        this->swe.removeOutliers(this->graph);

        // attempt to estimate the landmark depths using the new motion estimates
        this->runEpipolarDepthEstimators();

        // activate new landmarks if necessary
        this->activateNewLandmarks();

        // finally move the current frame into the keyframe set
        this->graph.moveCurrentFrameIntoKeyframePosition();
    }
}

void QDVO::BasicPipeline::runMarginalizationStrategy()
{
    this->swe.runMarginalizationStrategy(this->graph);
}

void QDVO::BasicPipeline::createNewLandmarks(std::unique_ptr<QDVO::Frame> &keyframe, std::unique_ptr<QDVO::FeatureDetector> &featureDetector)
{
    std::cout << "Creating new landmarks for keyframe: " << keyframe->frameID << std::endl;

    // Detect new features in the keyframe
    std::vector<QDVO::Feature> newFeatures = featureDetector->detectFeatures(*(keyframe.get()));

    std::cout << "Found " << newFeatures.size() << " new landmarks" << std::endl;

    // add the landmarks to the keyframe's landmark vector
    std::unique_ptr<QDVO::CameraModel> &cm = this->graph.getCameraModel(keyframe->camID);

    for (auto &f : newFeatures)
    {
        Landmark lm;
        lm.px = Eigen::Matrix<SCALAR_TYPE, 2, 1>(f.px.x, f.px.y);
        lm.landmarkID = keyframe->landmarks.empty() ? 1 : keyframe->landmarks.size() + 1;
        lm.parentFrameID = keyframe->frameID;
        lm.dinv = DEFAULT_LANDMARK_DINV;

        auto result = cm->unproject(lm.px);
        if (!result.has_value())
        {
            std::cout << "failed to unproject pixel." << std::endl;
            continue;
        }

        lm.bearing = result.value();

        keyframe->landmarks.push_back(lm);
    }
}

void QDVO::BasicPipeline::initializeCorrespondenceDistributionsForCurrentFrame()
{
    // first update the patch comparers before initializing all the correspondence distributions
    this->updatePatchComparers();

    std::unique_ptr<QDVO::Frame> &cf = this->graph.getCurrentFrame();
    std::shared_ptr<QDVO::PatchComparer> patchCompPtr = this->patchComparers.at(cf->frameID);

    // second reset correspondence distributions
    cf->resetCorrespondenceDistributions();

    // find the set of active landmarks visible in the current frame.
    // create and initialize the correspondence distribution for each of these landmarks
    std::vector<std::tuple<QDVO::Landmark *, QDVO::Vector2>> visibleActiveLandmarks = this->graph.getVisibleLandmarksInCurrentFrame(true);

    // Make sure that there are enough correspondence distributions
    int deficit = std::max(int(visibleActiveLandmarks.size() - cf->correspondenceDistributions.size()), 0);
    for (int i = 0; i < deficit; ++i)
    {
        // create another correspondence distribution
        cf->correspondenceDistributions.push_back(QDVO::CorrespondenceDistribution(cf->cm->width, cf->cm->height, this->radialSearchPatternPtr, cf.get()));
    }

    std::cout << "found " << visibleActiveLandmarks.size() << " visible and active landmarks for the current frame" << std::endl;

    auto initializationFn = [&, this, patchCompPtr](std::tuple<QDVO::Landmark *, QDVO::Vector2> &tup, QDVO::CorrespondenceDistribution &cdRef) -> int {
        QDVO::Landmark *l = std::get<0>(tup);

        assert(cdRef.dormant == true);

        // initialize the correspondence distribution
        auto px0 = this->graph.projectLandmarkToPixel(cf->frameID, l->parentFrameID, l->landmarkID);
        if (!px0.has_value())
        {
            std::cout << "landmark not visible in its parent frame!" << std::endl;
            return 1;
        }
        QDVO::Result<QDVO::Patch> warpedPatch = {};

        // warp the patch.
        this->patchWarper->warpPatchToTargetFrame(warpedPatch, *(l), *(this->graph.getFrame(l->parentFrameID)), *(this->graph.getCurrentFrame()), this->graph);
        if (!warpedPatch.has_value())
        {
            std::cout << "failed to warp patch" << std::endl;
            return 1;
        }

        cdRef.initializeDistribution(Eigen::Vector2i(std::round(px0.value()(0)), std::round(px0.value()(1))), MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS, patchCompPtr, warpedPatch.value());
        return 0;
    };

    std::vector<int> result(visibleActiveLandmarks.size());
    // Run the initialization function for all active and visible landmarks.
    QDVO::ParallelAlgorithms::transform(QDVO::ParallelAlgorithms::ExecutionType::PARALLEL_CPU, visibleActiveLandmarks.begin(), visibleActiveLandmarks.end(), cf->correspondenceDistributions.begin(), result.begin(), initializationFn);

    std::cout << "Initialized correspondence distributions for this frame. Could not initialize: " << std::accumulate(result.begin(), result.end(), 0) << " distributions." << std::endl;
}

bool QDVO::BasicPipeline::isCurrentFrameAKeyframe()
{
    if (this->graph.getKeyframeSet().size() == 0)
    {
        std::cout << "First frame is always a keyframe." << std::endl;
        return true;
    }

    return false;
}

void QDVO::BasicPipeline::updatePatchComparers()
{

    std::unique_ptr<QDVO::Frame> &cf = this->graph.getCurrentFrame();

    // ensure that a patch comparer is in the table for the current frame id
    if (!this->patchComparers.count(cf->frameID))
    {
        // patch comparer does not exist for the current frame
        this->patchComparers.insert({cf->frameID, std::shared_ptr<QDVO::PatchComparer>()});
    }

    // determine if there is a patch comparer in the table which is not associated to a non existent kf or the current frame
    bool patchComparerHasNoMatchingFrame = false;
    ID_TYPE idToRemove = 0;
    for (auto &pair : this->patchComparers)
    {
        bool hasMatchingFrame = false;
        for (auto &kfPair : this->graph.getKeyframeSet())
        {
            if (pair.first == kfPair.first)
            {
                hasMatchingFrame = true;
                break;
            }
        }

        if (cf->frameID == pair.first)
        {
            hasMatchingFrame = true;
        }

        if (!hasMatchingFrame)
        {
            patchComparerHasNoMatchingFrame = true;
            idToRemove = pair.first;
            break;
        }
    }

    if (patchComparerHasNoMatchingFrame)
    {
        // swap the memory from this frame to the current patchComparer
        this->patchComparers.at(cf->frameID).swap(this->patchComparers.at(idToRemove));

        // remove the old/obsolete patch comparer
        this->patchComparers.erase(idToRemove);
    }

    // setup the current patch comparer
    if (this->patchComparers.at(cf->frameID) == nullptr)
    {
        this->patchComparers.at(cf->frameID) = std::shared_ptr<QDVO::PatchComparer>(new QDVO::PatchComparer());
    }

    this->patchComparers.at(cf->frameID)->meanStdDevTable.setupTables(cf->imagePyr.getImage().toOpenCVImage(), cf.get());

    std::cout << "there are " << this->patchComparers.size() << " patch comparers in the table" << std::endl;
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
    std::vector<std::tuple<QDVO::Landmark *, QDVO::Vector2>> visibleLandmarks = this->graph.getVisibleLandmarksInCurrentFrame(false, true);

    std::cout << "there are currently " << visibleLandmarks.size() << " active and inactive landmarks visible in the current frame" << std::endl;

    // janky way of getting a decent feature distribution.
    cv::Mat mask = cv::Mat::zeros(this->graph.getCurrentFrame()->imagePyr.getImage().rows(), this->graph.getCurrentFrame()->imagePyr.getImage().cols(), CV_8U);

    const int maskRadius = 5;

    int nActiveLandmarks = 0;

    // fill in the mask for all active landmarks.
    for (auto &t : visibleLandmarks)
    {
        QDVO::Landmark *l = std::get<0>(t);
        assert(l != nullptr);
        if (l->status == QDVO::Landmark::ACTIVE)
        {
            cv::circle(mask, cv::Point2f(l->px(0), l->px(1)), maskRadius, cv::Scalar(255), -1);
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
        QDVO::Landmark *l = std::get<0>(t);
        assert(l != nullptr);
        if (l->status == QDVO::Landmark::INACTIVE && l->depthEstimator.initialized)
        {
            if (!mask.at<uint8_t>(cv::Point2f(l->px(0), l->px(1))))
            {
                l->status = QDVO::Landmark::ACTIVE;

                cv::circle(mask, cv::Point2f(l->px(0), l->px(1)), maskRadius, cv::Scalar(255), -1);
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
            QDVO::Landmark *l = std::get<0>(t);
            assert(l != nullptr);
            if (l->status == QDVO::Landmark::INACTIVE)
            {
                if (!mask.at<uint8_t>(cv::Point2f(l->px(0), l->px(1))))
                {
                    l->status = QDVO::Landmark::ACTIVE;

                    cv::circle(mask, cv::Point2f(l->px(0), l->px(1)), maskRadius, cv::Scalar(255), -1);
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
