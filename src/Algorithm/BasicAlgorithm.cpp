#include "BasicAlgorithm.h"

QDVO::BasicAlgorithm::BasicAlgorithm()
{

}

void QDVO::BasicAlgorithm::initialize()
{
    // precompute the radial search pattern LUT
    this->radialSearchPatternPtr = std::shared_ptr<RadialSearchPattern>(new QDVO::RadialSearchPattern(MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS));

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
}

void QDVO::BasicAlgorithm::addFrame(cv::Mat& image, const double& time, const ID_TYPE cameraID)
{
    // Reset current frame
    this->graph.getCurrentFrame()->reset();
    // Setup the current frame.
    this->graph.getCurrentFrame()->updateImage(image);
    this->graph.getCurrentFrame()->camID = cameraID;
    this->graph.getCurrentFrame()->imustate.time = time;
    this->graph.getCurrentFrame()->cm = this->graph.getCameraModel(cameraID).get();
    this->graph.getCurrentFrame()->frameID = this->graph.getNewFrameID();

    // Initialize correspondence distributions
    this->initializeCorrespondenceDistributionsForCurrentFrame();

    // Run front end visual odometry
    this->frontEndVisualOdometry.run(this->graph);

    // Check if the current frame meets the keyframe selection criteria
    if (this->isCurrentFrameAKeyframe())
    {
        // Create new landmarks for the new keyframe
        this->createNewLandmarks(this->graph.getCurrentFrame(), this->featureDetector);
    }
}

void QDVO::BasicAlgorithm::createNewLandmarks(std::unique_ptr<QDVO::Frame>& keyframe, std::unique_ptr<QDVO::FeatureDetector>& featureDetector)
{
    std::cout << "Creating new landmarks for keyframe: " << keyframe->frameID << std::endl;

    // Detect new features in the keyframe
    std::vector<QDVO::Feature> newFeatures = featureDetector->detectFeatures(*(keyframe.get()));

    // add the landmarks to the keyframe's landmark vector
    std::unique_ptr<QDVO::CameraModel>& cm = this->graph.getCameraModel(keyframe->camID);

    for (auto& f : newFeatures)
    {
        Landmark lm;
        lm.px = Eigen::Matrix<SCALAR_TYPE, 2, 1>(f.px.x, f.px.y);
        lm.landmarkID = keyframe->landmarks.empty() ? 1 : keyframe->landmarks.size() + 1;
        lm.parentFrameID = keyframe->frameID;
        lm.dinv = 1.0;

        keyframe->landmarks.push_back(lm);
    }

}

void QDVO::BasicAlgorithm::activateNewLandmarks()
{

}

void QDVO::BasicAlgorithm::initializeCorrespondenceDistributionsForCurrentFrame()
{

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
