
.. _program_listing_file_src_DataStructures_Graph.h:

Program Listing for File Graph.h
================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_DataStructures_Graph.h>` (``src/DataStructures/Graph.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "Frame.h"
   #include <opencv2/core.hpp>
   #include <unordered_map>
   #include <memory>
   #include <mutex>
   #include <algorithm>
   #include <thread>
   #include "GlobalDefinitions.h"
   #include "Settings.h"
   #include "CameraModel.hpp"
   #include "FeatureDetector.h"
   #include "Types.h"
   #include <tuple>
   
   namespace  QDVO {
   
   using KeyframeSetType = std::unordered_map<ID_TYPE, std::unique_ptr<Frame> >;
   using ExtrinsicSetType = std::unordered_map<ID_TYPE, QDVO::SE3 >;
   using CameraModelMapType = std::unordered_map<ID_TYPE, std::unique_ptr<CameraModel> >;
   
   class Graph
   {
   public:
       Graph();
   
       // Sets the camera model for the given cam ID. NOTE: QDVO creates its own local copy.
       void setCameraModel(std::unique_ptr<QDVO::CameraModel>& cameraModelPtr, const ID_TYPE cameraID)
       {
           this->cameraModelMap.insert(std::pair<ID_TYPE, std::unique_ptr<CameraModel> >(cameraID, std::unique_ptr<QDVO::CameraModel>()));
           // give ownership to the unique pointer in the table.
           this->cameraModelMap.at(cameraID).swap(cameraModelPtr);
       }
   
       // sets the transformation from the imu to camera for the given cam id
       void setExtrinsic(QDVO::SE3& T_imu_cam, const ID_TYPE cameraID)
       {
           this->extrinsicSet.insert_or_assign(cameraID, T_imu_cam);
       }
   
       std::unique_ptr<Frame>& getCurrentFrame();
   
       std::unique_ptr<CameraModel>& getCameraModel(const ID_TYPE cameraID = 1);
   
       QDVO::SE3 getExtrinsic(const ID_TYPE cameraID = 1);
   
       std::unique_ptr<Frame>& getKeyframe(const ID_TYPE keyframeID);
   
       // generalized version of the getKeyframe function
       std::unique_ptr<Frame>& getFrame(const ID_TYPE frameID);
   
       KeyframeSetType& getKeyframeSet(){return this->keyframeSet;}
   
       /*
        * Looks across the keyframe set and current frame for the highest frame ID and returns one id higher
        */
       ID_TYPE getNewFrameID();
   
       /*
        * This function will swap the current frame and the marginalized keyframe and update the hash table key to reflect the new keyframe id.
        * The current frame is now equal to the marginalized keyframe. For safety, you should always check that a frame is not marginalized when using it.
        *
        */
       void moveCurrentFrameIntoMarginalizedKeyframePosition(const ID_TYPE marginalizedKeyframeID);
   
       // assuming there is enough room in the keyframe set, the current frame is moved to a new spot in the keyframe set.
       void moveCurrentFrameIntoNewKeyframePosition();
   
       // generalized version of the two above functions
       void moveCurrentFrameIntoKeyframePosition();
   
       // Projects all landmarks in all keyframes into the current frame to determin if they are visible. will NOT project marginalized landmarks.
       std::vector<std::tuple<QDVO::Landmark*, QDVO::Vector2>> getVisibleLandmarksInCurrentFrame(bool activeLandmarksOnly, bool includeCurrentFrameLandmarks = false);
   
       // transforms the landmark into a euclidean point in the target frame.
       QDVO::Vector3 projectLandmarkToCameraFrame(ID_TYPE targetFrameID, ID_TYPE sourceFrameID, ID_TYPE landmarkID);
   
       QDVO::Result<QDVO::Vector2> projectLandmarkToPixel(ID_TYPE targetFrameID, ID_TYPE sourceFrameID, ID_TYPE landmarkID);
   
   
   private:
       // Members, nodes, and edges of the graph.
   
       CameraModelMapType cameraModelMap; // camID to camera model mapping. Done this way for memory/compute efficiency.
   
       KeyframeSetType keyframeSet; // gives mapping from keyframe ids to keyframes. bounds the memory consumption.
   
       ExtrinsicSetType extrinsicSet; // maps camera id to an imu2camera transform.
   
       std::unique_ptr<Frame> currentFrame;
   };
   }
   
