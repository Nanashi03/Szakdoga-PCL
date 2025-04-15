//asdasd
// Created by kristof on 2025.03.17..
//

#include "Viewer.h"

EventClickListener Viewer::cloudSelectedEventListener = nullptr;
EventButtonListener Viewer::selectedCloudTranslateLeftEventListener = nullptr;

Viewer::Viewer() :
    boundingBoxTransform {Eigen::Affine3f::Identity()}
{
}

void Viewer::pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* viewer_void) {
    std::cout << "[INOF] Point picking event occurred." << std::endl;
    if (event.getPointIndex () == -1)
    {
        return;
    }
    std::string name = event.getCloudName();
    std::cout << "[INOF] Cloud name: " << name << std::endl;
    if (name.empty()) {
        //DO SOMETHING MAYBE SEARCH BETWEEN CLOUDS?
        float x,y,z;
        event.getPoint(x,y,z);
        std::cout << "[INOF] Cloud point (" << x << ";" << y << ";" << z << ")" << std::endl;
    } else {
        cloudSelectedEventListener(name);
    }
}

void Viewer::keyboardPressingEventOccurred(const pcl::visualization::KeyboardEvent& event, void* viewer_void) {
    if (event.getKeySym () == "Left" && event.keyDown())
    {
        std::cout << "[INOF] \"a\" pressing event occurred." << std::endl;
        selectedCloudTranslateLeftEventListener(1.0f,0.0f,0.0f);
    }
    if (event.getKeySym () == "Right" && event.keyDown())
    {
        std::cout << "[INOF] \"d\" pressing event occurred." << std::endl;
        selectedCloudTranslateLeftEventListener(-1.0f,0.0f,0.0f);
    }
    if (event.getKeySym () == "Up" && event.keyDown())
    {
        std::cout << "[INOF] \"w\" pressing event occurred." << std::endl;
        selectedCloudTranslateLeftEventListener(0.0f,0.0f,1.0f);
    }
    if (event.getKeySym () == "Down" && event.keyDown())
    {
        std::cout << "[INOF] \"s\" pressing event occurred." << std::endl;
        selectedCloudTranslateLeftEventListener(0.0f,0.0f,-1.0f);
    }
    if (event.getKeySym () == "a" && event.keyDown())
    {
        std::cout << "[INOF] \"f\" pressing event occurred." << std::endl;
        selectedCloudTranslateLeftEventListener(0.0f,1.0f,0.0f);
    }
    if (event.getKeySym () == "d" && event.keyDown())
    {
        std::cout << "[INOF] \"g\" pressing event occurred." << std::endl;
        selectedCloudTranslateLeftEventListener(0.0f,-1.0f,0.0f);
    }
}


void Viewer::run() {
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

void Viewer::addCloud(const std::string& id, PointCloudT::ConstPtr cloud) {
    const pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb { cloud };
    viewer->addPointCloud<PointType>(cloud, rgb, id);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);
}

void Viewer::addNormals(const std::string& id, PointCloudT::ConstPtr cloud) {

    viewer->addPointCloudNormals<PointType> (cloud, 10, 2, id);
}

void Viewer::updateCloud(const std::string& id, bool areNormalsPresent, PointCloudT::ConstPtr cloud) {
    removeCloud(id, areNormalsPresent);
    addCloud(id, cloud);
    if (areNormalsPresent)
        addNormals(id + "_normals", cloud);
}

void Viewer::removeCloud(const std::string& id, bool areNormalsPresent) {
    viewer->removePointCloud(id);
    if (areNormalsPresent)
        viewer->removePointCloud(id+"_normals");
}

void Viewer::addBoundingBoxCube(const BoundingBoxData& data)
{
    viewer->addCube(data.bboxTransform, data.bboxQuaternion, data.width, data.height, data.depth, data.NAME);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                             pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                        data.NAME);
}

void Viewer::removeBoundingBoxCube() {
   viewer->removeAllShapes();
    boundingBoxTransform = Eigen::Affine3f::Identity();
}


void Viewer::translateBoundingBoxCube(float x, float y, float z) {
    boundingBoxTransform.translation() += Eigen::Vector3f(x, y, z);
    viewer->updateShapePose("BBOX", boundingBoxTransform);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                             pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                        "BBOX");
}

void Viewer::init(vtkRenderer* renderer, vtkGenericOpenGLRenderWindow* renderWindow) {
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));

    viewer->setBackgroundColor (0,0,0);
    viewer->initCameraParameters ();
    viewer->registerPointPickingCallback(pointPickingEventOccurred, (void*)&viewer);
    viewer->registerKeyboardCallback(keyboardPressingEventOccurred, (void*)&viewer);

    //viewer->resetCamera();
    //viewer->addCoordinateSystem (0.05);
}


vtkSmartPointer<vtkRenderWindow> Viewer::getRender() {
    return viewer->getRenderWindow();
}

void Viewer::setupInteractor(vtkRenderWindowInteractor* interactor, vtkRenderWindow* renderWindow) {
    viewer->setupInteractor(interactor, renderWindow);
    viewer->addOrientationMarkerWidgetAxes(interactor);
}
