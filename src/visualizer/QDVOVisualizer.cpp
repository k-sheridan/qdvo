#include "QDVOVisualizer.h"

QDVOVisualizer::QDVOVisualizer()
{

}

void QDVOVisualizer::initialize()
{
    this->algorithm.initialize();

    std::unique_ptr<QDVO::CameraModel> cm(new QDVO::EquidistantCameraModel(190.97847715128717, 190.9733070521226, 254.93170605935475, 256.8974428996504, 1.44*2, 512, 512,
                                                                           Eigen::Vector4d(0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182)));
    cameraModelKey = this->algorithm.addCamera(cm);

    extrinsicKey = this->algorithm.graph.getExtrinsicMap().insert(this->algorithm.graph.getCameraModelMap().at(cameraModelKey)->second);

    this->newFrameAdded = false;

}

void QDVOVisualizer::transferVisualizationData()
{
    //cv::Mat tempMat = this->algorithm.graph.getCurrentFrame()->imagePyr.getImage();
    //this->visualizationData.currentFrameTexture = pangolin::GlTexture(tempMat.cols,tempMat.rows,GL_LUMINANCE,false,0,GL_LUMINANCE,GL_UNSIGNED_BYTE);
    //this->visualizationData.currentFrameTexture.Upload(tempMat.data,GL_LUMINANCE,GL_UNSIGNED_BYTE);

    // draw current frame visualization
    if (this->algorithm.graph.getCurrentFrame()->initialized) // make sure the frame has been properly initialized
    {

        cv::Mat temp, render;
        this->algorithm.graph.getCurrentFrame()->imagePyr.getImage().toOpenCVImage().convertTo(temp, CV_8U);
        cv::cvtColor(temp, render, cv::COLOR_GRAY2RGB);

        ColorMap hotCMap;

        std::unique_ptr<QDVO::CameraModel>& cm = this->algorithm.graph.getCameraModelMap().at(this->algorithm.graph.getCurrentFrame()->cameraModelKey)->first;
        // draw current frame landmarks
        for (auto& key : this->algorithm.graph.getCurrentFrame()->landmarkKeys)
        {
            auto& e = *algorithm.graph.getLandmarkMap().at(key);
            Eigen::Matrix<SCALAR_TYPE, 2, 1> px = e.px;
            cv::circle(render, cv::Point2f(px(0), px(1)), 3, hotCMap.getColor(std::clamp(float(1/e.dinv/MAX_VISUALIZATION_DEPTH), 0.0f, 1.0f)), -1);

        }

        this->visualizationData.currentFrameImage = pangolin::GlTexture(render.cols,render.rows,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
        this->visualizationData.currentFrameImage.Upload(render.data,GL_RGB,GL_UNSIGNED_BYTE);
    }


}


void QDVOVisualizer::runQDVO(cv::Mat &image, double time)
{
    // if the newFrameAdded flag is true, then data is being transferred... wait
    while (this->newFrameAdded == true) {std::this_thread::sleep_for(std::chrono::milliseconds(1));}

    this->algorithmMutex.lock();
    TIK
    this->algorithm.addFrame(image, time, cameraModelKey, extrinsicKey);
    TOK
    this->algorithmMutex.unlock();

    // tell the visualizer to copy over the new data
    this->newFrameAdded = true;
}

void QDVOVisualizer::runVisualization()
{
    // create a window and bind its context to the main thread
    pangolin::CreateWindowAndBind(window_name, 1280, 850);

    // enable depth
    glEnable(GL_DEPTH_TEST);

    // Initialization of windows

    pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000);
    pangolin::OpenGlRenderState s_cam(proj, pangolin::ModelViewLookAt(-1,-1,-1,0,0,0, pangolin::AxisY) );

    pangolin::View& d_img1 = pangolin::Display("Current Frame")
            .SetBounds(0.3, 1, 0.6, 1);

    pangolin::View& pointCloudView = pangolin::Display("Point Cloud")
            .SetHandler(new pangolin::Handler3D(s_cam))
            .SetBounds(0.3, 1, 0, 0.6);



    while( !pangolin::ShouldQuit() )
    {
        // check if the visualization should be updated with new data
        if (this->newFrameAdded == true)
        {
            // At this point we can guarentee that the algorithm has finished an update and is now waiting for us to copy its data over.
            // Make a copy of the graph.
            this->algorithmMutex.lock();

            this->transferVisualizationData();

            this->algorithmMutex.unlock();
            // reset the flag
            this->newFrameAdded = false;
        }

        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_img1.Activate();
        glColor4f(1.0f,1.0f,1.0f,1.0f);
        this->visualizationData.currentFrameImage.RenderToViewportFlipY();

        pointCloudView.Activate(s_cam);
        glClearColor(0.4f,0.4f,0.4f,1.0f);
        glPointSize(3);
        glBegin(GL_POINTS);
        glColor3f(0,0.0,0.0);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);
        glVertex3f(0, 1, 0);
        glVertex3f(0, 0, 1);
        glEnd();


        // Swap frames and Process Events
        pangolin::FinishFrame();
    }


}
