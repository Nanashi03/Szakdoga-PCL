//
// Created by kristof on 2025.03.17..
//

#include "Viewer.h"

Viewer::Viewer() : viewer {new pcl::visualization::PCLVisualizer ("3D Viewer")} {
    viewer->setBackgroundColor (255, 255, 255);
    viewer->addCoordinateSystem (0.05);
    viewer->initCameraParameters ();
}

void Viewer::run() {
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

void Viewer::addCloud(const std::string& id, const PointCloudT& cloud) {
    viewer->addPointCloud<PointType>(std::make_shared<PointCloudT>(cloud), id);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);
    viewer->resetCamera();
}