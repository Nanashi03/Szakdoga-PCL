//
// Created by kristof on 2025.03.17..
//

#ifndef VIEWER_H
#define VIEWER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <memory>

using namespace std::chrono_literals;

class Viewer {
private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
public:
    Viewer();
    void run();
    void addCloud(const std::string&, const pcl::PointCloud<pcl::PointXYZRGBNormal>&);
};

#endif //VIEWER_H
