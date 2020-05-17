#include "QDVOVisualizer.h"

#include <pangolin/gl/gldraw.h>

#include <string>

#include "Profiling.h"

QDVOVisualizer::QDVOVisualizer() {}

void QDVOVisualizer::initialize(std::unique_ptr<QDVO::CameraModel> cameraModel,
                                const QDVO::SE3& T_imu_camera) {
  this->algorithm.initialize();

  cameraModelKey = this->algorithm.addCamera(std::move(cameraModel));

  extrinsicKey = this->algorithm.graph.getExtrinsicMap().insert(T_imu_camera);

  this->newFrameAdded = false;
}

void QDVOVisualizer::transferVisualizationData() {
  // draw current frame visualization
  if (this->algorithm.graph.getCurrentFrame()
          ->initialized)  // make sure the frame has been properly initialized
  {
    std::cout << "rendering the current frame" << std::endl;
    cv::Mat temp, render;
    this->algorithm.graph.getCurrentFrame()
        ->imagePyr.getImage()
        .toOpenCVImage()
        .convertTo(temp, CV_8U);
    cv::cvtColor(temp, render, cv::COLOR_GRAY2RGB);

    ColorMap hotCMap;

    std::unique_ptr<QDVO::CameraModel>& cm =
        this->algorithm.graph.getCameraModelMap()
            .at(this->algorithm.graph.getCurrentFrame()->cameraModelKey)
            ->first;
    // draw current frame landmarks.
    for (auto& key : this->algorithm.graph.getCurrentFrame()->landmarkKeys) {
      auto landmarkIt = algorithm.graph.getLandmarkMap().at(key);
      if (landmarkIt == algorithm.graph.getLandmarkMap().end()) {
        continue;
      }
      auto& e = *landmarkIt;
      Eigen::Matrix<SCALAR_TYPE, 2, 1> px = e.px;
      auto color = hotCMap.getColor(
          std::clamp(float(1 / e.dinv / MAX_VISUALIZATION_DEPTH), 0.0f, 1.0f));
      cv::circle(render, cv::Point2f(px(0), px(1)), 3, color, -1);
    }

    // cv::Mat temp2;
    // algorithm.graph.getCurrentFrame()
    //    ->imagePyr.getImage(3)
    //    .toOpenCVImage()
    //    .convertTo(temp2, CV_8U);
    // cv::imshow("test", temp2);
    // cv::waitKey(1);

    // draw correspondence distributions.
    for (auto& cd :
         this->algorithm.graph.getCurrentFrame()->correspondenceDistributions) {
      auto landmarkIt = algorithm.graph.getLandmarkMap().at(cd.landmarkKey);
      if (landmarkIt == algorithm.graph.getLandmarkMap().end()) {
        continue;
      }
      auto& l = *landmarkIt;
      auto px = this->algorithm.graph.projectLandmarkToPixel(
          this->algorithm.graph.getCurrentFrameKey(), l.parentFrameKey,
          cd.landmarkKey);
      auto point = this->algorithm.graph.projectLandmarkToCameraFrame(
          this->algorithm.graph.getCurrentFrameKey(), l.parentFrameKey,
          cd.landmarkKey);

      if (px.has_value()) {
        // TODO Draw the distribution.

        // Draw the landmark.
        // if initialized, draw with a depth color.
        if (cd.initialized) {
          cv::circle(
              render, cv::Point2f(px.value()(0), px.value()(1)), 2,
              hotCMap.getColor(std::clamp(
                  float(point(2) / MAX_VISUALIZATION_DEPTH), 0.0f, 1.0f)),
              -1);
        } else {
          // If not intiialized, draw purple.
          cv::circle(render, cv::Point2f(px.value()(0), px.value()(1)), 2,
                     cv::Scalar(255, 0, 204), -1);
        }
      }
    }

    this->visualizationData.currentFrameImage = pangolin::GlTexture(
        render.cols, render.rows, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
    this->visualizationData.currentFrameImage.Upload(render.data, GL_RGB,
                                                     GL_UNSIGNED_BYTE);
  } else {
    std::cout << "current frame not initialized. Not rendering." << std::endl;
  }

  // Set the pose of the current frame.
  this->visualizationData.currentFramePose =
      algorithm.graph.getCurrentFrame()->imustate.getSE3();

  // Render and transfer the keyframes.
  int kfidx = 0;
  int dataIndex = 0;
  for (auto it = algorithm.graph.getKeyframeMap().begin();
       it != algorithm.graph.getKeyframeMap().end(); it++) {
    auto key = algorithm.graph.getKeyframeMap().getKeyFromDataIndex(dataIndex);
    // If the keyframe is not the current frame.
    if (!(key == algorithm.graph.getCurrentFrameKey()) && (*it)->initialized) {
      cv::Mat temp, render;
      (*it)->imagePyr.getImage().toOpenCVImage().convertTo(temp, CV_8U);
      cv::cvtColor(temp, render, cv::COLOR_GRAY2RGB);

      // Set the pose.
      this->visualizationData.keyframePoses.at(dataIndex) =
          (*it)->imustate.getSE3();

      ColorMap hotCMap;

      // draw keyframe landmarks.
      for (auto& lKey : (*it)->landmarkKeys) {
        auto landmarkIt = algorithm.graph.getLandmarkMap().at(lKey);
        if (landmarkIt == algorithm.graph.getLandmarkMap().end()) {
          continue;
        }
        auto& e = *landmarkIt;
        Eigen::Matrix<SCALAR_TYPE, 2, 1> px = e.px;

        // Draw the landmark based on its status.
        if (e.status == QDVO::Landmark::LandmarkStatus::ACTIVE) {
          auto color = hotCMap.getColor(
              std::clamp((float)((float)1 / e.dinv / MAX_VISUALIZATION_DEPTH),
                         0.0f, 1.0f));
          cv::circle(render, cv::Point2f(px(0), px(1)), 3, color, -1);
        } else if (e.status == QDVO::Landmark::LandmarkStatus::INACTIVE) {
          if (e.depthEstimator.initialized) {
            // draw a smaller point.
            auto color = hotCMap.getColor(
                std::clamp((float)((float)1 / e.dinv / MAX_VISUALIZATION_DEPTH),
                           0.0f, 1.0f));
            cv::circle(render, cv::Point2f(px(0), px(1)), 2, color, -1);
          } else {
            auto color = hotCMap.getColor(
                std::clamp((float)((float)1 / e.dinv / MAX_VISUALIZATION_DEPTH),
                           0.0f, 1.0f));
            cv::circle(render, cv::Point2f(px(0), px(1)), 1, color, -1);
          }
        }
      }

      this->visualizationData.keyframeImages.at(kfidx) = pangolin::GlTexture(
          render.cols, render.rows, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
      this->visualizationData.keyframeImages.at(kfidx).Upload(
          render.data, GL_RGB, GL_UNSIGNED_BYTE);
      kfidx++;
    }
    dataIndex++;
  }

  // Cache the points in a common frame.
  visualizationData.inactivePoints.clear();
  visualizationData.activePoints.clear();
  visualizationData.marginalizedPoints.clear();
  for (auto& landmark : algorithm.graph.getLandmarkMap()) {
    QDVO::Frame& parentFrame =
        *(*algorithm.graph.getKeyframeMap().at(landmark.parentFrameKey));
    auto parentFramePose = parentFrame.imustate.getSE3();
    QDVO::Vector3 point =
        parentFramePose * landmark.bearing * (1 / landmark.dinv);

    if (landmark.status == QDVO::Landmark::INACTIVE &&
        landmark.depthEstimator.initialized) {
      // inactive.
      visualizationData.inactivePoints.push_back(point);
    } else if (landmark.status == QDVO::Landmark::ACTIVE) {
      // active.
      visualizationData.activePoints.push_back(point);
    } else {
      // marginalized.
      visualizationData.marginalizedPoints.push_back(point);
    }
  }
}

void QDVOVisualizer::runQDVO(cv::Mat& image, double time, bool notifyVisualizer,
                             QDVO::TrackingLog* trackingLog) {
  SPDLOG_INFO("Adding frame.");
  // if the newFrameAdded flag is true, then data is being transferred... wait
  while (notifyVisualizer && this->newFrameAdded == true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  this->algorithmMutex.lock();
  {
    PROFILE("addFrame");
    this->algorithm.addFrame(image, time, cameraModelKey, extrinsicKey);
  }
  // Log the tracking state.
  if (trackingLog != nullptr) {
    SPDLOG_INFO("Logging tracking state.");
    trackingLog->logTrackingState(this->algorithm.graph, this->algorithm.swe,
                                  this->algorithm.frontEndVisualOdometry);
  }
  this->algorithmMutex.unlock();

  // tell the visualizer to copy over the new data
  this->newFrameAdded = true;
}

void QDVOVisualizer::runVisualization() {
  // create a window and bind its context to the main thread
  auto& windowInterface = pangolin::CreateWindowAndBind(window_name);

  // Resize window.
  windowInterface.Resize(1000, 500);

  // enable depth
  glEnable(GL_DEPTH_TEST);

  // Initialization of windows

  pangolin::OpenGlMatrix proj =
      pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000);
  pangolin::OpenGlRenderState s_cam(
      proj, pangolin::ModelViewLookAt(-1, -1, -1, 0, 0, 0, pangolin::AxisY));

  pangolin::View& currentFrameView =
      pangolin::Display("Current Frame").SetBounds(0.3, 1, 0.6, 1);

  pangolin::View& pointCloudView =
      pangolin::Display("Point Cloud")
          .SetHandler(new pangolin::Handler3D(s_cam))
          .SetBounds(0.3, 1, 0, 0.6);

  // create the keyframes.
  this->visualizationData.keyframeImages.resize(N_KEYFRAMES);
  this->visualizationData.keyframePoses.resize(N_KEYFRAMES);

  pangolin::View& keyframeView =
      pangolin::Display("Keyframes")
          .SetBounds(0, 0.3, 0, 1)
          .SetLayout(pangolin::LayoutEqualHorizontal);

  // add a child for each keyframe.
  for (int i = 0; i < N_KEYFRAMES; ++i) {
    pangolin::View& image = pangolin::Display(std::to_string(i));
    keyframeView.AddDisplay(image);
  }

  while (!pangolin::ShouldQuit()) {
    // check if the visualization should be updated with new data
    if (this->newFrameAdded == true) {
      // At this point we can guarentee that the algorithm has finished an
      // update and is now waiting for us to copy its data over. Make a copy of
      // the graph.
      this->algorithmMutex.lock();

      this->transferVisualizationData();

      this->algorithmMutex.unlock();
      // reset the flag
      this->newFrameAdded = false;
    }

    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    currentFrameView.Activate();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    this->visualizationData.currentFrameImage.RenderToViewportFlipY();

    for (int i = 0; i < N_KEYFRAMES; ++i) {
      keyframeView[i].Activate();
      this->visualizationData.keyframeImages[i].RenderToViewportFlipY();
    }

    pointCloudView.Activate(s_cam);
    draw3DPointCloud();

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }
}

void drawFrustum(QDVO::SE3 pose, Eigen::Vector3f color, float lineWidth) {
  // Get near and far from the Projection matrix.
  const double near = 0.1;
  const double far = 0.4;

  // Get the sides of the near plane.
  const double nLeft = -0.05;
  const double nRight = 0.05;
  const double nTop = 0.05;
  const double nBottom = -0.05;

  // Get the sides of the far plane.
  const double fLeft = -0.2;
  const double fRight = 0.2;
  const double fTop = 0.2;
  const double fBottom = -0.2;

  /*
   0	glVertex3f(0.0f, 0.0f, 0.0f);
   1	glVertex3f(nLeft, nBottom, -near);
   2	glVertex3f(nRight, nBottom, -near);
   3	glVertex3f(nRight, nTop, -near);
   4	glVertex3f(nLeft, nTop, -near);
   5	glVertex3f(fLeft, fBottom, -far);
   6	glVertex3f(fRight, fBottom, -far);
   7	glVertex3f(fRight, fTop, -far);
   8	glVertex3f(fLeft, fTop, -far);
   */

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  // glLoadIdentity ();

  // TODO - Update: You need to invert the mv before multiplying it with the
  // current mv!
  Sophus::Matrix4f m = pose.matrix().cast<float>();
  glMultMatrixf((GLfloat*)m.data());

  glLineWidth(lineWidth);
  glColor3f(color[0], color[1], color[2]);
  glBegin(GL_LINES);

  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(fLeft, fBottom, -far);

  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(fRight, fBottom, -far);

  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(fRight, fTop, -far);

  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(fLeft, fTop, -far);

  // far
  glVertex3f(fLeft, fBottom, -far);
  glVertex3f(fRight, fBottom, -far);

  glVertex3f(fRight, fTop, -far);
  glVertex3f(fLeft, fTop, -far);

  glVertex3f(fRight, fTop, -far);
  glVertex3f(fRight, fBottom, -far);

  glVertex3f(fLeft, fTop, -far);
  glVertex3f(fLeft, fBottom, -far);

  // near
  glVertex3f(nLeft, nBottom, -near);
  glVertex3f(nRight, nBottom, -near);

  glVertex3f(nRight, nTop, -near);
  glVertex3f(nLeft, nTop, -near);

  glVertex3f(nLeft, nTop, -near);
  glVertex3f(nLeft, nBottom, -near);

  glVertex3f(nRight, nTop, -near);
  glVertex3f(nRight, nBottom, -near);

  glEnd();
  glLineWidth(1);
  glPopMatrix();
}

void QDVOVisualizer::draw3DPointCloud() {
  glClearColor(0.5f, 0.5f, 0.5f, 1.0f);

  glPointSize(5);
  glBegin(GL_POINTS);
  // Draw inactive points.
  glColor3f(0.3, 0.3, 0.3);
  for (auto pt : visualizationData.inactivePoints) {
    glVertex3f(pt[0], pt[1], pt[2]);
  }
  // Draw marginalized points.
  glColor3f(0, 0.0, 0.0);
  for (auto pt : visualizationData.marginalizedPoints) {
    // glVertex3f(pt[0], pt[1], pt[2]);
  }
  // Draw active points.
  glColor3f(1.0, 1.0, 1.0);
  for (auto pt : visualizationData.activePoints) {
    glVertex3f(pt[0], pt[1], pt[2]);
  }
  glEnd();

  drawFrustum(visualizationData.currentFramePose, Eigen::Vector3f(1, 0, 0), 3);

  int kfidx = 0;
  for (auto se3 : visualizationData.keyframePoses) {
    drawFrustum(se3, Eigen::Vector3f(1, 1, 1), 3);
  }
}
