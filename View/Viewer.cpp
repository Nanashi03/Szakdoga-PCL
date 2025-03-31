//asdasd
// Created by kristof on 2025.03.17..
//

#include "Viewer.h"

EventClickListener Viewer::cloudSelectedEventListener = nullptr;
EventButtonListener Viewer::selectedCloudTranslateLeftEventListener = nullptr;

Viewer::Viewer() : viewer {new pcl::visualization::PCLVisualizer ("3D Viewer")} {
    viewer->setBackgroundColor (0,0,0);
    viewer->initCameraParameters ();
    viewer->registerPointPickingCallback(pointPickingEventOccurred, (void*)&viewer);
    viewer->registerKeyboardCallback(keyboardPressingEventOccurred, (void*)&viewer);
    viewer->addOrientationMarkerWidgetAxes(viewer->getRenderWindow()->GetInteractor());

    //viewer->resetCamera();
    //viewer->addCoordinateSystem (0.05);
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
    if (event.getKeySym () == "a" && event.keyDown())
    {
        std::cout << "[INOF] \"a\" pressing event occurred." << std::endl;
        selectedCloudTranslateLeftEventListener(1,0,0);
    }
    if (event.getKeySym () == "d" && event.keyDown())
    {
        std::cout << "[INOF] \"d\" pressing event occurred." << std::endl;
        selectedCloudTranslateLeftEventListener(-1,0,0);
    }
    if (event.getKeySym () == "w" && event.keyDown())
    {
        std::cout << "[INOF] \"w\" pressing event occurred." << std::endl;
        selectedCloudTranslateLeftEventListener(0,0,1);
    }
    if (event.getKeySym () == "s" && event.keyDown())
    {
        std::cout << "[INOF] \"s\" pressing event occurred." << std::endl;
        selectedCloudTranslateLeftEventListener(0,0,-1);
    }
    if (event.getKeySym () == "f" && event.keyDown())
    {
        std::cout << "[INOF] \"f\" pressing event occurred." << std::endl;
        selectedCloudTranslateLeftEventListener(0,1,0);
    }
    if (event.getKeySym () == "g" && event.keyDown())
    {
        std::cout << "[INOF] \"g\" pressing event occurred." << std::endl;
        selectedCloudTranslateLeftEventListener(0,-1,0);
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

void Viewer::updateCloud(const std::string& id, PointCloudT::ConstPtr cloud) {
    removeCloud(id);
    addCloud(id, cloud);
}

void Viewer::removeCloud(const std::string& id) {
    viewer->removePointCloud(id);
}