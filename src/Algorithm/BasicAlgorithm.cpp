#include "BasicAlgorithm.h"

QDVO::BasicAlgorithm::BasicAlgorithm()
{

}

void QDVO::BasicAlgorithm::initialize()
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

void QDVO::BasicAlgorithm::addCamera(std::unique_ptr<QDVO::CameraModel>& cameraModel, const ID_TYPE cameraID)
{
    this->graph.setCameraModel(cameraModel, cameraID);

    std::cout << "Camera Model Initialized" << std::endl;

    QDVO::SE3 unit(Eigen::Quaternion<QDVO::SE3::Scalar>(1, 0,0,0), Eigen::Matrix<QDVO::SE3::Scalar, 3, 1>(0,0,0));
    this->graph.setExtrinsic(unit, cameraID);

    std::cout << "Added default unit extrinsic" << std::endl;
}

void QDVO::BasicAlgorithm::addFrame(cv::Mat& image, const double& time, const ID_TYPE cameraID)
{
    //std::cout << "here" << std::endl;
    TIK
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
    this->initializeCorrespondenceDistributionsForCurrentFrame();

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

    TOK
}

void QDVO::BasicAlgorithm::runMarginalizationStrategy()
{
    this->swe.runMarginalizationStrategy(this->graph);
}

void QDVO::BasicAlgorithm::createNewLandmarks(std::unique_ptr<QDVO::Frame>& keyframe, std::unique_ptr<QDVO::FeatureDetector>& featureDetector)
{
    std::cout << "Creating new landmarks for keyframe: " << keyframe->frameID << std::endl;

    // Detect new features in the keyframe
    std::vector<QDVO::Feature> newFeatures = featureDetector->detectFeatures(*(keyframe.get()));

    std::cout << "Found " << newFeatures.size() << " new landmarks" << std::endl;

    // add the landmarks to the keyframe's landmark vector
    std::unique_ptr<QDVO::CameraModel>& cm = this->graph.getCameraModel(keyframe->camID);

    for (auto& f : newFeatures)
    {
        Landmark lm;
        lm.px = Eigen::Matrix<SCALAR_TYPE, 2, 1>(f.px.x, f.px.y);
        lm.landmarkID = keyframe->landmarks.empty() ? 1 : keyframe->landmarks.size() + 1;
        lm.parentFrameID = keyframe->frameID;
        lm.dinv = DEFAULT_LANDMARK_DINV;

        try {
            lm.bearing = cm->unproject(lm.px);
        } catch (std::runtime_error& e) {
            std::cout << "failed to unproject pixel." << std::endl;
            continue;
        }

        keyframe->landmarks.push_back(lm);
    }

}

void QDVO::BasicAlgorithm::initializeCorrespondenceDistributionsForCurrentFrame()
{
    // first update the patch comparers before initializing all the correspondence distributions
    this->updatePatchComparers();

    std::unique_ptr<QDVO::Frame>& cf = this->graph.getCurrentFrame();
    std::shared_ptr<QDVO::PatchComparer> patchCompPtr = this->patchComparers.at(cf->frameID);

    // second reset correspondence distributions
    cf->resetCorrespondenceDistributions();
    size_t cdIdx = 0;

    // find the set of active landmarks visible in the current frame.
    // create and initialize the correspondence distribution for each of these landmarks
    std::vector<std::tuple<QDVO::Landmark*, QDVO::Vector2>> visibleActiveLandmarks = this->graph.getVisibleLandmarksInCurrentFrame(true);

    std::cout << "found " << visibleActiveLandmarks.size() << " visible and active landmarks for the current frame" << std::endl;

    for (auto& tup : visibleActiveLandmarks)
    {
        QDVO::Landmark* l = std::get<0>(tup);

        if (cdIdx >= cf->correspondenceDistributions.size())
        {
            // create another correspondence distribution
            cf->correspondenceDistributions.push_back(QDVO::CorrespondenceDistribution(cf->cm->width, cf->cm->height, this->radialSearchPatternPtr, cf.get()));
        }

        assert(cdIdx < cf->correspondenceDistributions.size());
        assert(cf->correspondenceDistributions.at(cdIdx).dormant == true);



        // initialize the correspondence distribution
        QDVO::CorrespondenceDistribution& cdRef = cf->correspondenceDistributions.at(cdIdx);
        QDVO::Vector2 px0 = this->graph.projectLandmarkToPixel(cf->frameID, l->parentFrameID, l->landmarkID);
        QDVO::Patch warpedPatch;
        this->patchWarper->warpPatchToTargetFrame(warpedPatch, *(l), *(this->graph.getFrame(l->parentFrameID)), *(cf), this->graph);

        std::cout << int(warpedPatch.image.at<uint8_t>(PATCH_WIDTH, PATCH_WIDTH)) << std::endl;

        cdRef.initializeDistribution(Eigen::Vector2i(std::round(px0(0)), std::round(px0(1))), MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS, patchCompPtr, warpedPatch);

        ++cdIdx;
    }

}

bool QDVO::BasicAlgorithm::isCurrentFrameAKeyframe()
{
    if (this->graph.getKeyframeSet().size() == 0)
    {
        std::cout << "First frame is always a keyframe." << std::endl;
        return true;
    }


    return false;
}

void QDVO::BasicAlgorithm::updatePatchComparers()
{

    std::unique_ptr<QDVO::Frame>& cf = this->graph.getCurrentFrame();

    // ensure that a patch comparer is in the table for the current frame id
    if (!this->patchComparers.count(cf->frameID))
    {
        // patch comparer does not exist for the current frame
        this->patchComparers.insert({cf->frameID, std::shared_ptr<QDVO::PatchComparer>()});
    }



    // determine if there is a patch comparer in the table which is not associated to a non existent kf or the current frame
    bool patchComparerHasNoMatchingFrame = false;
    ID_TYPE idToRemove = 0;
    for (auto& pair : this->patchComparers)
    {
        bool hasMatchingFrame = false;
        for (auto& kfPair : this->graph.getKeyframeSet())
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



    if(patchComparerHasNoMatchingFrame)
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

    this->patchComparers.at(cf->frameID)->meanStdDevTable.setupTables(cf->imagePyr.getImage(0));

    std::cout << "there are " << this->patchComparers.size() << " patch comparers in the table" << std::endl;
}

void QDVO::BasicAlgorithm::runEpipolarDepthEstimators()
{

}

void QDVO::BasicAlgorithm::activateNewLandmarks()
{
    /*
     * Warning: This is an absolute mess, but for now it will have to do.
     */


    std::cout << "Activating landmarks." << std::endl;

    // find all visible active and inactive landmarks
    std::vector<std::tuple<QDVO::Landmark*, QDVO::Vector2>> visibleLandmarks = this->graph.getVisibleLandmarksInCurrentFrame(false, true);

    std::cout << "there are currently " << visibleLandmarks.size() << " active and inactive landmarks visible in the current frame" << std::endl;

    // janky way of getting a decent feature distribution.
    cv::Mat mask = cv::Mat::zeros(this->graph.getCurrentFrame()->imagePyr.getImage().rows, this->graph.getCurrentFrame()->imagePyr.getImage().cols, CV_8U);

    const int maskRadius = 5;

    int nActiveLandmarks = 0;

    // fill in the mask for all active landmarks.
    for (auto& t : visibleLandmarks)
    {
        QDVO::Landmark* l = std::get<0>(t);
        assert(l != nullptr);
        if (l->status == QDVO::Landmark::ACTIVE)
        {
            cv::circle(mask, cv::Point2f(l->px(0), l->px(1)), maskRadius, cv::Scalar(255), -1);
            ++nActiveLandmarks;
        }
    }

    std::cout << nActiveLandmarks << " active visible landmarks before activation." << std::endl;

    if (nActiveLandmarks >= N_FEATURES_DESIRED){return;}

    // first activate initialized landmarks
    for (auto& t : visibleLandmarks)
    {
        QDVO::Landmark* l = std::get<0>(t);
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

        if (nActiveLandmarks >= N_FEATURES_DESIRED){break;}
    }

    std::cout << nActiveLandmarks << " active visible landmarks after activating initialized landmarks." << std::endl;

    // if necessary activate uninitialized landmarks
    if (nActiveLandmarks < MINUMUM_ACTIVE_LANDMARKS)
    {
        for (auto& t : visibleLandmarks)
        {
            QDVO::Landmark* l = std::get<0>(t);
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

            if (nActiveLandmarks >= MINUMUM_ACTIVE_LANDMARKS){break;}
        }
        std::cout << nActiveLandmarks << " active visible landmarks after activating uninitialized landmarks." << std::endl;
    }



}
