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
        //std::this_thread::sleep_for(100ms);
    }
}

void Viewer::addCloud(const std::string& id, const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud) {
    /*pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloudPTR = cloud;*/
    viewer->addPointCloud<pcl::PointXYZRGBNormal>(std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>(cloud), id);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);
    viewer->resetCamera();
}